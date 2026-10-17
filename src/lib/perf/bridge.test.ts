/**
 * JS <-> wasm bridge tests for the mesh-refinement kernel (`euler_solve_ex`) and the Euler quality tiers.
 *   npm run test:bridge   (tsx src/lib/perf/bridge.test.ts)
 * 1. wasm euler_solve_ex through euler-wasm.ts (embedded binary) vs the TypeScript port (strip-euler.ts), per preset
 * 2. the same solve inside a real worker thread (node:worker_threads), wasm loaded from bytes
 * 3. wedge vs the exact oblique-shock relation through the bridge, per tier
 * 4. quality tiers: cache separation, ordering of cost, legacy path untouched
 * 5. the compute client (loopback) routes preview / full / refine through the right tier
 */
import assert from "node:assert/strict";
import { Worker } from "node:worker_threads";
import { DEG, betaFromThetaM, obliqueShock } from "../waverider/math";
import { presetSettings, type RefinementPreset, type RefinementSettings } from "../waverider/refinement";
import { solveStrip, stripGridOf } from "../waverider/strip-euler";
import { refinedPlan } from "../waverider/sym-euler";
import { eulerSync, hasSolveEx, kernelFlags, solveWallEx, solveWallRaw, wasmSource } from "./euler-kernel";
import { QUALITY_VOLUMES, qualitySettings } from "./quality";
import { hashFloats } from "./hash";
import { buildVehicle } from "../waverider/generate";
import { clearSymmetryEulerCache, wallsOfMesh, solveCpForTest } from "../waverider/sym-euler";
import { MODE_QUALITY, Pipeline, studyKey } from "./pipeline";
import { hashValue } from "./hash";
import type { EulerQuality } from "./euler-kernel";
import { DEFAULT_PARAMS } from "../waverider/types";

let passed = 0;
async function test(name: string, fn: () => void | Promise<void>) {
  const t = performance.now();
  try {
    await fn();
  } catch (e) {
    console.error(`FAIL ${name}`);
    throw e;
  }
  passed++;
  console.log(`ok   ${name} (${(performance.now() - t).toFixed(0)} ms)`);
}

eulerSync();
type Wall = { x: Float64Array; z: Float64Array };
function wedge(theta: number, n = 80): Wall {
  const x = Float64Array.from({ length: n }, (_, i) => i / (n - 1));
  return { x, z: x.map((v) => v * Math.tan(theta * DEG)) };
}
function ogive(n = 60): Wall {
  const x = new Float64Array(n);
  const z = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const t = i / (n - 1);
    x[i] = 4 * t;
    z[i] = 0.3 * Math.sin(Math.PI * 0.5 * t) * (1 - 0.3 * t);
  }
  return { x, z };
}

function exSolve(w: Wall, mach: number, alpha: number, s: RefinementSettings): Float64Array {
  const { v, d, shockS } = refinedPlan(w, mach, 1.4, alpha, s, 0.02);
  const out = solveWallEx(w.x, w.z, mach, 1.4, alpha, { ni: d.ni, nj: d.nj, iters: d.iters, wallAmp: v.wallClustering, shockAmp: shockS ? v.shockClustering : 0, shockWidth: v.shockWidth, tol: v.tol ?? 0, sequence: v.sequence ?? 0, cfl: 0, shockS });
  assert.ok(out, "euler_solve_ex rejected its arguments");
  return out;
}
function tsSolve(w: Wall, mach: number, alpha: number, s: RefinementSettings): Float64Array {
  const { v, shockS } = refinedPlan(w, mach, 1.4, alpha, s, 0.02);
  return solveStrip(w.x, w.z, mach, 1.4, alpha, stripGridOf(v, shockS)).cp;
}

await test("bridge: shipped wasm exports euler_solve_ex and euler_solve_grid, loaded from the embedded binary", () => {
  assert.ok(hasSolveEx());
  const k = eulerSync();
  assert.equal(typeof k.euler_solve_ex, "function");
  assert.equal(typeof k.euler_solve_grid, "function");
  assert.equal(typeof k.euler_solve, "function");
  assert.equal(wasmSource, "embedded");
});

// ---- 1. parity wasm vs TS port, per preset
const PRESETS: Exclude<RefinementPreset, "custom">[] = ["coarse", "medium", "fine", "converged"];
const parity: string[] = [];
for (const preset of PRESETS) {
  await test(`parity: euler_solve_ex (wasm) vs TypeScript port, preset ${preset}`, () => {
    const s = presetSettings(preset);
    let worst = 0;
    let worstRel = 0;
    for (const [name, w, mach, alpha] of [["wedge10", wedge(10), 8, 0], ["wedge25", wedge(25), 6, 0], ["ogive", ogive(), 7, 0.035]] as const) {
      const a = exSolve(w, mach, alpha, s);
      const b = tsSolve(w, mach, alpha, s);
      assert.equal(a.length, b.length);
      let md = 0;
      let mc = 0;
      for (let i = 0; i < a.length; i++) {
        assert.ok(Number.isFinite(a[i]));
        md = Math.max(md, Math.abs(a[i] - b[i]));
        mc = Math.max(mc, Math.abs(b[i]));
      }
      worst = Math.max(worst, md);
      worstRel = Math.max(worstRel, md / Math.max(mc, 1e-9));
      parity.push(`${preset}/${name}: max|dCp|=${md.toExponential(2)}`);
    }
    console.log(`     ${preset}: worst |dCp| = ${worst.toExponential(2)} (rel ${worstRel.toExponential(2)})`);
    // fixed-iteration presets (coarse/medium/fine) are the same algorithm in the same order: ~round-off
    // (the Rust kernel and the TS port may reassociate a few ops); the tol-stopped preset may stop one sweep apart
    const limit = s.volume.tol ? 2e-3 : 1e-9;
    assert.ok(worst <= limit, `${preset}: ${worst} > ${limit}`);
  });
}

await test("bridge: euler_solve_ex rejects bad arguments (returns null -> TypeScript fallback), and never mutates results across calls", () => {
  const w = wedge(10);
  const s = presetSettings("coarse");
  const a = exSolve(w, 8, 0, s);
  assert.equal(solveWallEx(w.x, w.z, 8, 1.4, 0, { ni: 0, nj: 10, iters: 10, wallAmp: 0, shockAmp: 0, shockWidth: 0.09, tol: 0, sequence: 0, cfl: 0, shockS: null }), null);
  assert.equal(solveWallEx(w.x, w.z, 8, 1.4, 0, { ni: 40, nj: 10, iters: 10, wallAmp: 0, shockAmp: 0, shockWidth: 0.09, tol: 0, sequence: 0, cfl: 0, shockS: null }) === null, false);
  assert.equal(hashFloats(exSolve(w, 8, 0, s)), hashFloats(a), "repeat solve differs (scratch buffer reuse bug)");
});

// ---- 2. real worker thread
await test("worker thread: euler_solve_ex in a node worker (wasm from bytes) matches the main thread", async () => {
  const s = presetSettings("medium");
  const jobs = [
    { w: wedge(10), mach: 8, alpha: 0 },
    { w: ogive(), mach: 7, alpha: 0.035 },
  ];
  const mainRes = jobs.map((j) => hashFloats(exSolve(j.w, j.mach, j.alpha, s)));
  const worker = new Worker(`import { register } from "tsx/esm/api"; register(); await import(${JSON.stringify(new URL("./bridge.worker.ts", import.meta.url).href)});`, { eval: true });
  const res = await new Promise<string[]>((resolve, reject) => {
    worker.once("message", (m: { hashes?: string[]; source?: string; error?: string }) => (m.error ? reject(new Error(m.error)) : resolve([...(m.hashes ?? []), m.source ?? ""])));
    worker.once("error", reject);
    worker.postMessage({ jobs: jobs.map((j) => ({ x: j.w.x, z: j.w.z, mach: j.mach, alpha: j.alpha })), preset: "medium" });
  });
  await worker.terminate();
  assert.deepEqual(res.slice(0, jobs.length), mainRes);
  assert.equal(res[jobs.length], "file");
});

// ---- 3. wedge vs exact oblique shock through the bridge
const wedgeRows: string[] = [];
for (const tier of ["preview", "standard", "converged"] as const) {
  await test(`wedge vs exact oblique shock through the bridge, tier ${tier}`, () => {
    const lim = tier === "preview" ? 0.01 : 0.005; // fraction of exact Cp
    for (const [M, th] of [[8, 8], [8, 30], [5, 15], [12, 10]] as const) {
      const w = wedge(th);
      const cp = exSolve(w, M, 0, qualitySettings(tier)!);
      let sum = 0;
      let k = 0;
      for (let i = 0; i < w.x.length; i++) if (w.x[i] > 0.3 && w.x[i] < 0.8) (sum += cp[i]), k++;
      const beta = betaFromThetaM(M, th * DEG, 1.4);
      const p2 = obliqueShock(M, beta, 1.4).p2p1;
      const exact = (2 / (1.4 * M * M)) * (p2 - 1);
      const err = Math.abs(sum / k - exact) / exact;
      wedgeRows.push(`${tier} M${M} th${th}: ${(err * 100).toFixed(3)} %`);
      assert.ok(err < lim, `${tier} M${M} th${th}: ${(err * 100).toFixed(2)} %`);
    }
  });
}
await test("wedge: the legacy 42-sweep kernel is still the documented unconverged baseline (33 % / 93 % at M8, 8/30 deg)", () => {
  const err = (th: number) => {
    const w = wedge(th);
    const cp = solveWallRaw(w.x, w.z, 8, 1.4, 0);
    let s = 0;
    let k = 0;
    for (let i = 0; i < w.x.length; i++) if (w.x[i] > 0.3 && w.x[i] < 0.8) (s += cp[i]), k++;
    const exact = (2 / (1.4 * 64)) * (obliqueShock(8, betaFromThetaM(8, th * DEG, 1.4), 1.4).p2p1 - 1);
    return Math.abs(s / k - exact) / exact;
  };
  assert.ok(Math.abs(err(8) - 0.33) < 0.03, String(err(8)));
  assert.ok(Math.abs(err(30) - 0.93) < 0.03, String(err(30)));
});

// ---- 4. quality tiers
await test("quality: tiers are ordered by cost, `legacy` has no settings, library default stays legacy", () => {
  assert.equal(kernelFlags.quality, "legacy");
  assert.equal(qualitySettings("legacy"), undefined);
  const dims = (q: keyof typeof QUALITY_VOLUMES) => qualitySettings(q)!.volume;
  assert.ok(dims("preview").nStream < dims("standard").nStream);
  assert.ok(dims("standard").nStream <= dims("converged").nStream && dims("converged").nStream < dims("reference").nStream);
});

await test("quality: tiers never share cache entries (same wall + flight, different tier => different Cp); legacy output is untouched", () => {
  const mesh = buildVehicle({ ...DEFAULT_PARAMS, family: "caret" }).mesh;
  const walls = wallsOfMesh(mesh)!;
  const solve = (q: EulerQuality) => {
    const prev = kernelFlags.quality;
    kernelFlags.quality = q;
    try {
      return hashFloats(solveCpForTest(walls.upper, 8, 1.4, 2 * DEG));
    } finally {
      kernelFlags.quality = prev;
    }
  };
  clearSymmetryEulerCache();
  const legacy0 = solve("legacy");
  const preview = solve("preview");
  const standard = solve("standard");
  assert.equal(solve("legacy"), legacy0, "legacy changed after tiered solves");
  assert.equal(solve("preview"), preview, "tier solve not reproducible (cache poisoned?)");
  assert.notEqual(preview, standard);
  assert.notEqual(preview, legacy0);
  assert.notEqual(standard, legacy0);
  assert.equal(hashFloats(solveWallRaw(walls.upper.x, walls.upper.z, 8, 1.4, 2 * DEG)), legacy0, "legacy path is the plain shipped kernel");
  assert.equal(kernelFlags.quality, "legacy");
});

// ---- 5. pipeline mode -> tier
await test("pipeline: study modes map to tiers (preview/quick -> preview, full -> standard, refine -> converged); keys differ; the flag never leaks", () => {
  assert.deepEqual(MODE_QUALITY, { preview: "preview", quick: "preview", full: "standard", refine: "converged" });
  const params = { ...DEFAULT_PARAMS, family: "caret" as const, aeroMethod: "euler" as const };
  assert.notEqual(studyKey(params, "full"), studyKey(params, "refine"));
  assert.notEqual(studyKey(params, "quick"), studyKey(params, "full"));
  const p = new Pipeline();
  const quick = p.run(params, "quick");
  assert.equal(kernelFlags.quality, "legacy", "quality flag leaked out of a pipeline run");
  const forcedLegacy = new Pipeline("legacy").run(params, "quick");
  assert.notEqual(hashValue(quick.study.summary ?? quick.study), hashValue(forcedLegacy.study.summary ?? forcedLegacy.study));
});

console.log(`bridge tests passed: ${passed}`);
console.log(parity.join("\n"));
console.log(wedgeRows.join("\n"));
