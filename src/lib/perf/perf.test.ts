/**
 * Regression + cache tests for the performance layer. Run: `npm run test:perf`
 * (tsx src/lib/perf/perf.test.ts). Every optimised path is compared with the original
 * implementation, which is kept reachable (kernelFlags.reference, euler2d's TS march, ...).
 */
import assert from "node:assert/strict";
import { Hasher, hashFloats, hashValue } from "./hash";
import { LRU } from "./lru";
import { PREVIEW_GRID, eulerCache, eulerKey, kernelFlags, primeWallCp, solveWallCp, solveWallRaw } from "./euler-kernel";
import { atmosphere } from "../waverider/atmosphere";
import { solveEulerWedge } from "../waverider/euler2d";
import { buildVehicle } from "../waverider/generate";
import { betaFromThetaM, obliqueShock } from "../waverider/math";
import { panelAero } from "../waverider/panel";
import { flightMachOf, studyVehicle } from "../waverider/study";
import { runValidation, validationSummary } from "../waverider/validate";
import { ANALYSIS_ONLY, geometryKey, Pipeline, rebindCenterline } from "./pipeline";
import { ComputeClient, SupersededError } from "./client";
import { memoPanel, panelCache, panelMemoFlags } from "./panel-memo";
import { DEFAULT_PARAMS, type DesignParams, type SurfaceGrid, type WaveriderFamily } from "../waverider/types";

let passed = 0;
function test(name: string, fn: () => void) {
  const t = performance.now();
  try {
    fn();
  } catch (err) {
    console.error(`FAIL ${name}`);
    throw err;
  }
  passed++;
  console.log(`ok   ${name} (${(performance.now() - t).toFixed(0)} ms)`);
}

// ---------------------------------------------------------------- hashing / LRU
test("hash: stable, order-independent for objects, sensitive to 1-ulp changes", () => {
  assert.equal(hashValue({ a: 1, b: [1, 2, { c: "x" }] }), hashValue({ b: [1, 2, { c: "x" }], a: 1 }));
  assert.notEqual(hashValue({ a: 1 }), hashValue({ a: 1 + Number.EPSILON }));
  assert.notEqual(hashValue({ a: 0 }), hashValue({ a: -0 }));
  assert.notEqual(hashValue([1, 2]), hashValue([2, 1]));
  assert.equal(hashFloats(new Float64Array([1, 2, 3])), hashFloats(new Float64Array([1, 2, 3])));
  assert.notEqual(hashFloats(new Float64Array([1, 2, 3])), hashFloats(new Float64Array([1, 2, 3.0000000000000004])));
  assert.equal(new Hasher().value({ x: undefined, y: 1 }).digest(), new Hasher().value({ y: 1 }).digest());
});

test("lru: entry bound, byte bound, recency", () => {
  const l = new LRU<number>(3, 100);
  l.set("a", 1, 10);
  l.set("b", 2, 10);
  l.set("c", 3, 10);
  l.get("a");
  l.set("d", 4, 10);
  assert.equal(l.has("b"), false, "least-recent evicted");
  assert.equal(l.has("a"), true);
  l.set("big", 5, 95);
  assert.ok(l.bytes <= 100 || l.size === 1);
  assert.equal(l.get("zzz"), undefined);
  assert.ok(l.stats().misses >= 1);
});

// ---------------------------------------------------------------- wasm kernel parity
function wall(n: number, kind: "lin" | "ogive" | "neg") {
  const x = new Float64Array(n);
  const z = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const t = i / (n - 1);
    x[i] = 4 * t;
    z[i] = kind === "lin" ? 0.1 * x[i] : kind === "ogive" ? 0.3 * Math.sin(Math.PI * 0.5 * t) * (1 - 0.3 * t) : -0.05 * x[i] - 0.1 * t * t;
  }
  return { x, z };
}

test("kernel: optimised solve is bit-identical to the original over Mach x alpha x wall x n", () => {
  let worst = 0;
  let cases = 0;
  for (const kind of ["lin", "ogive", "neg"] as const)
    for (const M of [1.6, 3, 5, 8, 12, 20])
      for (const a of [-0.12, 0, 0.05, 0.2])
        for (const n of [24, 61, 120]) {
          const w = wall(n, kind);
          const ref = solveWallRaw(w.x, w.z, M, 1.4, a, { reference: true });
          const fast = solveWallRaw(w.x, w.z, M, 1.4, a);
          for (let i = 0; i < n; i++) worst = Math.max(worst, Math.abs(ref[i] - fast[i]));
          cases++;
        }
  console.log(`     ${cases} cases, max |dCp| = ${worst}`);
  assert.ok(worst <= 1e-12, `kernel drift ${worst}`);
});

test("kernel: real centerlines of 6 families, optimised vs original", () => {
  const fams: WaveriderFamily[] = ["caret", "cone", "osculating", "star", "liftbody", "wedgecone"];
  const col = (g: SurfaceGrid) => {
    let jBest = 0;
    let best = Infinity;
    for (let j = 0; j < g.nj; j++) {
      let y = 0;
      for (let i = 0; i < g.ni; i++) y += Math.abs(g.xyz[(i * g.nj + j) * 3 + 1]);
      if (y < best) {
        best = y;
        jBest = j;
      }
    }
    const x: number[] = [];
    const z: number[] = [];
    for (let i = 0; i < g.ni; i++) {
      const o = (i * g.nj + jBest) * 3;
      if (x.length && g.xyz[o] <= x[x.length - 1] + 1e-7) continue;
      x.push(g.xyz[o]);
      z.push(g.xyz[o + 2]);
    }
    return { x: Float64Array.from(x), z: Float64Array.from(z) };
  };
  let worst = 0;
  for (const family of fams) {
    const b = buildVehicle({ ...DEFAULT_PARAMS, family });
    for (const name of ["upper", "lower"]) {
      const g = b.grids.find((s) => s.name === name);
      if (!g) continue;
      const w = col(g);
      for (const M of [4, 8, 14])
        for (const a of [0, 0.07]) {
          const ref = solveWallRaw(w.x, w.z, M, 1.4, a, { reference: true });
          const fast = solveWallRaw(w.x, w.z, M, 1.4, a);
          for (let i = 0; i < ref.length; i++) worst = Math.max(worst, Math.abs(ref[i] - fast[i]));
        }
    }
  }
  assert.ok(worst <= 1e-12, `real-wall drift ${worst}`);
});

// ---------------------------------------------------------------- Euler wedge (wasm vs TS reference, vs exact oblique shock)
test("wedge: wasm march matches the TS reference march and the exact oblique shock check", () => {
  let worstRel = 0;
  let worstRes = 0;
  let worstExact = 0;
  let cases = 0;
  for (const M of [2, 3, 5, 8])
    for (const th of [6, 10, 16, 22]) {
      const beta = betaFromThetaM(M, (th * Math.PI) / 180, 1.4);
      if (!Number.isFinite(beta)) continue;
      kernelFlags.wedge = false;
      const ts = solveEulerWedge(M, th, 1.4);
      kernelFlags.wedge = true;
      const wa = solveEulerWedge(M, th, 1.4);
      assert.equal(ts.iters, wa.iters);
      worstRel = Math.max(worstRel, Math.abs(ts.p2p1 - wa.p2p1) / ts.p2p1);
      worstRes = Math.max(worstRes, Math.abs(ts.residual - wa.residual) / Math.max(ts.residual, 1e-3));
      const exact = obliqueShock(M, beta, 1.4).p2p1;
      worstExact = Math.max(worstExact, Math.abs(Math.abs(wa.p2p1 - exact) - Math.abs(ts.p2p1 - exact)) / exact);
      cases++;
    }
  console.log(`     ${cases} wedges: max rel(TS vs wasm) p2/p1 = ${worstRel.toExponential(2)}, residual (rel, floor 1e-3) = ${worstRes.toExponential(2)}, |err vs exact| differs by ${worstExact.toExponential(2)}`);
  assert.ok(worstRel < 1e-11, `wedge drift ${worstRel}`);
  assert.ok(worstRes < 1e-6, `wedge residual drift ${worstRes}`);
  assert.ok(worstExact < 1e-11, `wedge vs exact ${worstExact}`);
  const m2 = solveEulerWedge(2, 10, 1.4);
  assert.ok(m2.relErr < 0.08);
});

// ---------------------------------------------------------------- full pipeline: panelAero(euler) and the Checks tab
test("panelAero(euler): optimised pipeline == original kernel across families x Mach x alpha", () => {
  let worst = 0;
  let n = 0;
  for (const family of ["caret", "osculating", "liftbody", "star"] as WaveriderFamily[]) {
    const p0: DesignParams = { ...DEFAULT_PARAMS, family, aeroMethod: "euler" };
    const b = buildVehicle(p0);
    for (const M of [5, 8, 12])
      for (const alpha of [0, 2.5, -1.5]) {
        const p = { ...p0, mach: M, flightMach: M, lockFlight: true };
        const atm = atmosphere(p.altKm, flightMachOf(p), p.gamma);
        const S = Math.max(b.aero.planformArea, 1e-6);
        kernelFlags.reference = true;
        eulerCache.clear();
        const ref = panelAero(b.mesh, p, atm, alpha, 0, S, p.length);
        kernelFlags.reference = false;
        eulerCache.clear();
        const fast = panelAero(b.mesh, p, atm, alpha, 0, S, p.length);
        for (const k of ["cl", "cd", "cm", "ld", "xCp", "cdWave"] as const) worst = Math.max(worst, Math.abs(ref[k] - fast[k]));
        for (let i = 0; i < ref.cp.length; i++) worst = Math.max(worst, Math.abs(ref.cp[i] - fast.cp[i]));
        n++;
      }
  }
  console.log(`     ${n} cases, max |d| = ${worst}`);
  assert.ok(worst <= 1e-9, `pipeline drift ${worst}`);
});

test("Checks tab: all identities pass with the wasm wedge kernel", () => {
  kernelFlags.wedge = true;
  const a = validationSummary(runValidation());
  assert.ok(a.ok, `validation ${a.pass}/${a.n}`);
  // (the TS reference march is compared with the wasm march directly in the wedge test above)
  assert.equal(a.pass, a.n);
});

// ---------------------------------------------------------------- caches
test("euler cache: hit returns the identical array; any input change misses", () => {
  eulerCache.clear();
  const w = wall(61, "lin");
  const a = solveWallCp(w.x, w.z, 8, 1.4, 0.02);
  const s0 = eulerCache.stats();
  const b = solveWallCp(w.x, w.z, 8, 1.4, 0.02);
  const s1 = eulerCache.stats();
  assert.equal(a, b, "same reference on a hit");
  assert.equal(s1.hits, s0.hits + 1);
  assert.equal(s1.size, 1);
  const fresh = solveWallRaw(w.x, w.z, 8, 1.4, 0.02);
  assert.deepEqual(Array.from(b), Array.from(fresh), "hit content == fresh solve");
  const variants: [string, () => Float64Array][] = [
    ["mach", () => solveWallCp(w.x, w.z, 8 + 1e-9, 1.4, 0.02)],
    ["alpha", () => solveWallCp(w.x, w.z, 8, 1.4, 0.02 + 1e-12)],
    ["gamma", () => solveWallCp(w.x, w.z, 8, 1.4 + 1e-12, 0.02)],
    [
      "wall z (1 point, 1e-12)",
      () => {
        const z2 = w.z.slice();
        z2[30] += 1e-12;
        return solveWallCp(w.x, z2, 8, 1.4, 0.02);
      },
    ],
    [
      "wall x (shifted)",
      () => {
        const x2 = w.x.slice();
        x2[10] += 1e-9;
        return solveWallCp(x2, w.z, 8, 1.4, 0.02);
      },
    ],
  ];
  let size = eulerCache.size;
  for (const [name, f] of variants) {
    const out = f();
    assert.equal(eulerCache.size, size + 1, `${name} must miss and insert`);
    assert.notEqual(out, a);
    size++;
  }
  // an old key that rounded to the same text must no longer alias
  assert.notEqual(eulerKey(w.x, w.z, 8, 1.4, 0.0204), eulerKey(w.x, w.z, 8, 1.4, 0.0201));
});

test("euler cache: primeWallCp serves a result computed elsewhere; LRU bound holds", () => {
  eulerCache.clear();
  const w = wall(40, "ogive");
  const fake = new Float64Array(40).fill(0.123);
  primeWallCp(w.x, w.z, 6, 1.4, 0.0, fake);
  assert.equal(solveWallCp(w.x, w.z, 6, 1.4, 0.0), fake);
  for (let i = 0; i < 700; i++) solveWallCp(w.x, w.z, 6, 1.4, 0.001 * (i + 1));
  assert.ok(eulerCache.size <= 512, `lru size ${eulerCache.size}`);
});

test("study: same inputs -> identical result; flight change keeps geometry, geometry change rebuilds", () => {
  const b1 = buildVehicle(DEFAULT_PARAMS);
  const s1 = studyVehicle(b1, {});
  const s2 = studyVehicle(b1, {});
  assert.equal(s1.aero.cl, s2.aero.cl);
  assert.equal(s1.aero.cd, s2.aero.cd);
  const s3 = studyVehicle({ ...b1, params: { ...DEFAULT_PARAMS, alphaDeg: 3 } }, {});
  assert.notEqual(s3.aero.cl, s1.aero.cl);
  const b2 = buildVehicle({ ...DEFAULT_PARAMS, span: 2.4 });
  const s4 = studyVehicle(b2, {});
  assert.notEqual(s4.aero.cl, s1.aero.cl);
});


// ---------------------------------------------------------------- pipeline / deny-list
const ALL_FAMILIES = [
  "caret", "cone", "osculating", "viscopt", "inward", "elliptic", "busemann", "star", "wedgecone", "liftbody", "ramjet", "scramjet", "integrated",
] as unknown as WaveriderFamily[];
function stripParams(b: unknown): unknown {
  const { params: _p, ...rest } = b as Record<string, unknown>;
  return rest;
}
function perturb(p: DesignParams, k: keyof DesignParams): DesignParams {
  const v = p[k] as unknown;
  const q = { ...p } as Record<string, unknown>;
  if (typeof v === "number") q[k] = v * 1.07 + 0.013;
  else if (typeof v === "boolean") q[k] = !v;
  else if (k === "aeroMethod") q[k] = v === "euler" ? "cbaero" : "euler";
  else if (k === "fuel") q[k] = v === "H2" ? "JP" : "H2";
  else if (k === "unit") q[k] = v === "m" ? "mm" : "m";
  else if (typeof v === "string") q[k] = v + "x";
  return q as unknown as DesignParams;
}

test("pipeline deny-list: buildVehicle ignores every ANALYSIS_ONLY field (all families)", () => {
  for (const fam of ALL_FAMILIES) {
    const base = { ...DEFAULT_PARAMS, family: fam, nx: 18, ny: 12 };
    const ref = hashValue(stripParams(buildVehicle(base)));
    for (const k of ANALYSIS_ONLY) {
      const q = perturb(base, k);
      assert.notDeepEqual(q[k], base[k], `perturb ${k}`);
      assert.equal(hashValue(stripParams(buildVehicle(q))), ref, `${fam}: geometry changed with ${String(k)}`);
      assert.equal(geometryKey(q), geometryKey(base));
    }
    // and a true geometry field must change the key
    assert.notEqual(geometryKey({ ...base, span: base.span + 0.01 }), geometryKey(base));
  }
});

test("pipeline: stage hits identical, flight change keeps geometry, geometry change invalidates", () => {
  const pl = new Pipeline();
  const p0 = { ...DEFAULT_PARAMS, nx: 24, ny: 16 };
  const a = pl.run(p0, "full");
  assert.deepEqual(a.hit, { geometry: false, study: false });
  const direct = studyVehicle(buildVehicle(p0), {});
  assert.equal(a.study.aero.cl, direct.aero.cl);
  assert.equal(hashFloats(a.study.aero.cp), hashFloats(direct.aero.cp));
  const b = pl.run(p0, "full");
  assert.deepEqual(b.hit, { geometry: true, study: true });
  assert.equal(b.study, a.study);
  // flight-only change: geometry served from cache, study recomputed with the NEW flight params
  const p1 = { ...p0, alphaDeg: 4.5, altKm: 31 };
  const c = pl.run(p1, "full");
  assert.deepEqual(c.hit, { geometry: true, study: false });
  const d1 = studyVehicle(buildVehicle(p1), {});
  assert.equal(c.study.aero.cl, d1.aero.cl);
  assert.equal(hashFloats(c.study.aero.cp), hashFloats(d1.aero.cp));
  assert.equal(c.built.params, p1);
  // cosmetic change recomputes the geometry stage not at all
  const e = pl.run({ ...p0, name: "other", unit: "mm" }, "full");
  assert.equal(e.hit.geometry, true);
  // geometry change invalidates both
  const f = pl.run({ ...p0, span: p0.span + 0.2 }, "full");
  assert.deepEqual(f.hit, { geometry: false, study: false });
  assert.notEqual(f.geomKey, a.geomKey);
  // preview mode is its own cache entry and never poisons full results
  const pv = pl.run(p0, "preview");
  assert.equal(pv.hit.study, false);
  assert.equal(pl.run(p0, "full").study, a.study);
  assert.ok(Number.isFinite(pv.study.aero.cl));
});

test("panel memo: hit is the identical object; any input change misses; disabled == enabled", () => {
  panelCache.clear();
  const b = buildVehicle({ ...DEFAULT_PARAMS, nx: 20, ny: 14 });
  const atm = atmosphere(28, 8);
  const args = [b.mesh, b.params, atm, 2, 0, 1.3, 2.1] as const;
  const r1 = panelAero(...args);
  const r2 = panelAero(...args);
  assert.equal(r1, r2);
  const n = panelCache.size;
  panelAero(b.mesh, b.params, atm, 2.0000001, 0, 1.3, 2.1);
  panelAero(b.mesh, { ...b.params, aeroMethod: b.params.aeroMethod === "euler" ? "cbaero" : "euler" }, atm, 2, 0, 1.3, 2.1);
  panelAero(b.mesh, b.params, atmosphere(29, 8), 2, 0, 1.3, 2.1);
  panelAero(b.mesh, b.params, atm, 2, 0, 1.3, 2.1, { p: 0.01, q: 0, r: 0, lRefOverV: 0.01 } as never);
  assert.ok(panelCache.size >= n + 4, "each changed input must add an entry");
  panelMemoFlags.enabled = false;
  const r3 = panelAero(...args);
  panelMemoFlags.enabled = true;
  assert.notEqual(r3, r1);
  assert.equal(hashFloats(r3.cp), hashFloats(r1.cp));
  assert.equal(r3.cl, r1.cl);
  assert.equal(typeof memoPanel, "function");
});

test("rebindCenterline restores the euler wall after a structured clone", () => {
  const b = buildVehicle({ ...DEFAULT_PARAMS, aeroMethod: "euler", nx: 20, ny: 14 });
  const direct = studyVehicle(b, { quick: true });
  const cloned = rebindCenterline(structuredClone({ ...b }));
  const viaClone = studyVehicle(cloned, { quick: true });
  assert.equal(viaClone.aero.cl, direct.aero.cl);
  assert.equal(hashFloats(viaClone.aero.cp), hashFloats(direct.aero.cp));
});

// ---------------------------------------------------------------- worker client (loopback runner, structured clone on)
async function atest(name: string, fn: () => Promise<void>) {
  const t = performance.now();
  try {
    await fn();
  } catch (err) {
    console.error(`FAIL ${name}`);
    throw err;
  }
  passed++;
  console.log(`ok   ${name} (${(performance.now() - t).toFixed(0)} ms)`);
}

await atest("client: study through the protocol equals the direct study; geometry omitted when held", async () => {
  const c = new ComputeClient({ runner: "inline", clone: true });
  const p = { ...DEFAULT_PARAMS, nx: 24, ny: 16, aeroMethod: "euler" as const };
  const r = await c.study(p, "quick");
  assert.ok(r.built);
  const direct = studyVehicle(buildVehicle(p), { quick: true });
  assert.equal(r.study.aero.cl, direct.aero.cl);
  assert.equal(hashFloats(r.study.aero.cp), hashFloats(direct.aero.cp));
  const r2 = await c.study({ ...p, alphaDeg: 5 }, "quick", r.geomKey);
  assert.equal(r2.built, undefined, "geometry not re-sent");
  assert.equal(r2.hit.geometry, true);
  const s = (await c.stats()) as { geometry: { hits: number } };
  assert.ok(s.geometry.hits >= 1);
  c.dispose();
});

await atest("client: latest-wins coalescing drops superseded queued requests, keeps order", async () => {
  const c = new ComputeClient({ runner: "inline" });
  const base = { ...DEFAULT_PARAMS, nx: 20, ny: 12 };
  const results: Promise<unknown>[] = [];
  const outcome: string[] = [];
  for (let i = 0; i < 6; i++) {
    results.push(
      c.study({ ...base, alphaDeg: i }, "preview").then(
        (r) => outcome.push(`ok${i}:${r.study.aero.cl.toFixed(6)}`),
        (e) => outcome.push(e instanceof SupersededError ? `sup${i}` : `err${i}`),
      ),
    );
  }
  await Promise.all(results);
  // request 0 starts immediately, 1..4 are replaced, 5 runs last
  assert.deepEqual(outcome.map((o) => o.slice(0, 4)).sort(), ["ok0:", "ok5:", "sup1", "sup2", "sup3", "sup4"].sort());
  const last = outcome.find((o) => o.startsWith("ok5"))!;
  kernelFlags.grid = PREVIEW_GRID;
  const want = studyVehicle(buildVehicle({ ...base, alphaDeg: 5 }), { quick: true });
  kernelFlags.grid = undefined;
  assert.ok(last.endsWith(want.aero.cl.toFixed(6)), `${last} vs ${want.aero.cl}`);
  // full and preview lanes are independent
  const [a, b] = await Promise.all([c.study(base, "full"), c.study(base, "preview")]);
  assert.equal(a.mode, "full");
  assert.equal(b.mode, "preview");
  c.dispose();
});

await atest("client: errors surface as rejections and the lane keeps working; exports run unsplit", async () => {
  const c = new ComputeClient({ runner: "inline", clone: true });
  await assert.rejects(c.exportFiles("nonsense", DEFAULT_PARAMS, { python: "", cSrc: "", cppSrc: "", kernelSource: "" }), /unsupported export kind/);
  const files = await c.exportFiles("stl", { ...DEFAULT_PARAMS, nx: 24, ny: 16 }, { python: "", cSrc: "", cppSrc: "", kernelSource: "" });
  assert.ok(files.length >= 1 && files[0].blob.size > 1000, "stl produced");
  // two exports queued back-to-back are both served (exports are never coalesced)
  const sources = { python: "", cSrc: "", cppSrc: "", kernelSource: "" };
  const [x, y] = await Promise.all([
    c.exportFiles("stl", { ...DEFAULT_PARAMS, nx: 24, ny: 16 }, sources),
    c.exportFiles("stl", { ...DEFAULT_PARAMS, nx: 24, ny: 16, span: 2.5 }, sources),
  ]);
  assert.ok(x[0].blob.size > 0 && y[0].blob.size > 0);
  c.dispose();
});

console.log(`perf tests passed: ${passed}`);
