/**
 * aerodb-perf: cuspis-panel wasm vs the TypeScript panel loop.
 * Run: npm run test:aerodb-perf   (cargo test + this file)
 *
 * Tolerance: |wasm − ts| ≤ 1e-6·max(|wasm|, |ts|) + ABS_FLOOR. The floor only matters for quantities that are
 * zero by symmetry (CY, Cn, Cll at β = 0 come out as ±1e-17 round-off on both sides).
 */
import assert from "node:assert/strict";
import { buildVehicle } from "./generate";
import { DEFAULT_PARAMS, type AeroMethod, type DesignParams, type WaveriderFamily } from "./types";
import { evalCaseTs, evalCaseWasm, packPoints, STRIDE_OUT, type BatchCase, type BatchPoint } from "./aerodb-batch";
import { PANEL_WASM_BYTES, panelKernel } from "./panel-kernel";

export const REL = 1e-6;
export const ABS_FLOOR = 1e-9;
const OUT_NAMES = ["cl", "cd", "cy", "cm", "cn", "cll", "ca", "ld", "cnA", "cdFric", "cdBase", "xCp"];

export function within(a: number, b: number): boolean {
  if (Number.isNaN(a) && Number.isNaN(b)) return true;
  return Math.abs(a - b) <= REL * Math.max(Math.abs(a), Math.abs(b)) + ABS_FLOOR;
}

const kernel = panelKernel();
assert.ok(kernel, "cuspis-panel wasm instantiates in Node");
console.log(`cuspis-panel wasm v${kernel.version()} ${PANEL_WASM_BYTES} bytes`);

const geo = (family: WaveriderFamily, patch: Partial<DesignParams> = {}) =>
  buildVehicle({ ...DEFAULT_PARAMS, family, nx: 24, ny: 16, mach: 8, ...patch });

const POINTS: BatchPoint[] = [];
for (const mach of [2.5, 6, 14]) for (const alpha of [-4, 0, 6, 14]) for (const beta of [0, 3]) POINTS.push({ mach, alpha, beta });
POINTS.push({ mach: 8, alpha: 4, beta: 0, rates: { p: 0, q: 6, r: 0 } }, { mach: 8, alpha: 4, beta: 0, rates: { p: 4, q: 0, r: 0 } }, { mach: 8, alpha: 4, beta: 0, rates: { p: 0, q: 0, r: 3 } });

// ---- batch-level goldens: every kernel method, cone and wedge families, rates
let checked = 0;
let worst = 0;
for (const [family, patch] of [
  ["caret", {}],
  ["cone", {}],
  ["osculating", {}],
  ["liftbody", { nx: 16, ny: 12 }],
  ["capsule", {}],
  ["missile", { span: 0.5, finHeight: 0.4, fins: 4 }],
  ["boostglide", { elevonDeg: 8 }],
] as [WaveriderFamily, Partial<DesignParams>][]) {
  const b = geo(family, patch);
  for (const method of ["newtonian", "tangent", "cbaero", "mixed"] as AeroMethod[]) {
    const c: BatchCase = { mesh: b.mesh, params: { ...b.params, aeroMethod: method }, altKm: 32, sRef: Math.max(b.aero.planformArea, 1e-6), lRef: Math.max(b.params.length, 1e-6) };
    const pts = packPoints(POINTS);
    const ts = evalCaseTs(c, pts);
    const wa = evalCaseWasm(kernel, c, pts);
    for (let i = 0; i < ts.length; i++) {
      const rel = Math.abs(ts[i] - wa[i]) / Math.max(Math.abs(ts[i]), 1e-3);
      if (rel > worst && Math.abs(ts[i]) > 1e-9) worst = rel;
      assert.ok(within(ts[i], wa[i]), `${family} ${method} pt ${Math.floor(i / STRIDE_OUT)} ${OUT_NAMES[i % STRIDE_OUT]}: ts ${ts[i]} wasm ${wa[i]}`);
      checked++;
    }
  }
}
console.log(`batch goldens: ${checked} values within 1e-6 rel (+${ABS_FLOOR} abs); worst rel on non-zero values ${worst.toExponential(2)}`);

// ---- generateAeroDatabase goldens: wasm (calling thread), loopback pool and worker_threads pool vs TS-only
import { AERODB_POINT_CAP, aeroDbPointCap, generateAeroDatabase, gridCount, type AeroDatabase, type AeroDbSpec } from "./aerodb";
import { AeroPool, loopbackPort } from "./aerodb-pool";
import { createNodeAeroPool } from "./aerodb-pool-node";
import { atmosphere } from "./atmosphere";
import { panelAero } from "./panel";

const NUM_FIELDS = ["cl", "cd", "cy", "cm", "cn", "cll", "ca", "ld", "cla", "cda", "cma", "cyb", "cnb", "clb", "cld", "cmd", "clq", "cmq", "clp", "cnp", "cyp", "cnr", "clr", "cyr"] as const;
const DYN_FIELDS = ["clq", "cmq", "clp", "cnp", "cyp", "cnr", "clr", "cyr"] as const;

function compareDb(ref: AeroDatabase, got: AeroDatabase, tag: string): number {
  assert.deepEqual(got.axes, ref.axes, `${tag} axes`);
  assert.equal(got.points.length, ref.points.length, `${tag} point count`);
  let n = 0;
  ref.points.forEach((r, i) => {
    const g = got.points[i];
    assert.equal(g.mach, r.mach);
    assert.equal(g.alpha_deg, r.alpha_deg);
    assert.equal(g.beta_deg, r.beta_deg);
    assert.equal(g.delta_deg, r.delta_deg);
    assert.equal(g.onDesign, r.onDesign);
    for (const f of NUM_FIELDS) {
      assert.ok(within(r[f], g[f]), `${tag} pt ${i} (M ${r.mach} α ${r.alpha_deg} β ${r.beta_deg} δ ${r.delta_deg}) ${f}: ts ${r[f]} vs ${g[f]}`);
      n++;
    }
  });
  assert.equal(got.dynamics.length, ref.dynamics.length);
  ref.dynamics.forEach((r, i) => {
    for (const f of DYN_FIELDS) {
      assert.ok(within(r[f], got.dynamics[i][f]), `${tag} dyn ${i} ${f}: ts ${r[f]} vs ${got.dynamics[i][f]}`);
      n++;
    }
  });
  for (const k of ["maxLd", "clMin", "clMax", "cdMin", "cdMax", "cmAtDesign"] as const) assert.ok(within(ref.summary[k], got.summary[k]), `${tag} summary ${k}`);
  return n;
}

const spec = (method: AeroMethod, patch: Partial<AeroDbSpec> = {}): AeroDbSpec => ({
  machMin: 3,
  machMax: 12,
  machStep: 4.5,
  alphaMin: -4,
  alphaMax: 12,
  alphaStep: 4,
  betaMin: -4,
  betaMax: 4,
  betaStep: 4,
  deltaMin: -10,
  deltaMax: 10,
  deltaStep: 10,
  method,
  eulerSymmetry: false,
  dynamicDerivatives: true,
  altKm: 30,
  ...patch,
});

const nodePool = createNodeAeroPool(3);
const loopPool = new AeroPool({ size: 2, spawn: () => loopbackPort() });
try {
  let values = 0;
  const cases: [WaveriderFamily, Partial<DesignParams>, AeroMethod[]][] = [
    ["caret", {}, ["newtonian", "tangent", "cbaero", "mixed"]],
    ["osculating", {}, ["cbaero", "mixed"]],
    ["cone", {}, ["tangent"]],
    ["boostglide", {}, ["cbaero", "mixed"]],
    ["missile", { span: 0.5, finHeight: 0.4, fins: 4 }, ["cbaero"]],
    ["liftbody", { nx: 16, ny: 12 }, ["newtonian"]],
  ];
  for (const [family, patch, methods] of cases) {
    const built = geo(family, patch);
    for (const method of methods) {
      const sp = spec(method);
      const t0 = performance.now();
      const ref = await generateAeroDatabase(built, sp, { engine: "ts" });
      const t1 = performance.now();
      assert.equal(ref.meta.engine, "ts");
      const wasm = await generateAeroDatabase(built, sp, { engine: "wasm" });
      const t2 = performance.now();
      assert.equal(wasm.meta.engine, "wasm");
      values += compareDb(ref, wasm, `${family}/${method}/wasm`);
      const pooled = await generateAeroDatabase(built, sp, { engine: "pool", pool: nodePool });
      const t3 = performance.now();
      assert.equal(pooled.meta.engine, "pool:3");
      values += compareDb(ref, pooled, `${family}/${method}/pool`);
      // deterministic order: the pool is bit-identical to the calling-thread kernel
      wasm.points.forEach((p, i) => assert.ok(p.cl === pooled.points[i].cl && p.cm === pooled.points[i].cm, `${family}/${method} pool order`));
      console.log(`${family.padEnd(10)} ${method.padEnd(9)} ${ref.points.length} pts  ts ${(t1 - t0).toFixed(0)} ms  wasm ${(t2 - t1).toFixed(0)} ms  pool×3 ${(t3 - t2).toFixed(0)} ms`);
    }
  }
  // loopback pool (no Worker available) gives the same table
  {
    const built = geo("caret");
    const sp = spec("cbaero");
    const ref = await generateAeroDatabase(built, sp, { engine: "ts" });
    const loop = await generateAeroDatabase(built, sp, { engine: "pool", pool: loopPool, chunkSize: 7 });
    values += compareDb(ref, loop, "caret/cbaero/loopback");
  }
  console.log(`generateAeroDatabase goldens: ${values} values, wasm / pool vs TS within 1e-6 rel (+${ABS_FLOOR} abs)`);

  // ---- behaviour
  const built = geo("caret");
  // backward-compatible call (no options) works and, in Node without Workers, runs the kernel on this thread
  const plain = await generateAeroDatabase(built, spec("cbaero", { betaMin: 0, betaMax: 0, deltaMin: 0, deltaMax: 0 }));
  assert.equal(plain.meta.engine, "wasm");
  // euler is not in the kernel: TS path, original cap
  assert.equal(aeroDbPointCap("euler"), AERODB_POINT_CAP);
  assert.equal(aeroDbPointCap("cbaero", "ts"), AERODB_POINT_CAP);
  assert.ok(aeroDbPointCap("cbaero") > AERODB_POINT_CAP);
  // cap: 600 points is over the TS cap but fine on the kernel; pointCap overrides both ways
  const big = spec("newtonian", { machMin: 2, machMax: 20, machStep: 2, alphaMin: -8, alphaMax: 20, alphaStep: 2, betaMin: 0, betaMax: 0, deltaMin: -10, deltaMax: 10, deltaStep: 5, dynamicDerivatives: false });
  const nBig = gridCount(big, built.params.mach, built.params.alphaDeg).points;
  assert.ok(nBig > AERODB_POINT_CAP, `big grid ${nBig}`);
  await assert.rejects(generateAeroDatabase(built, big, { engine: "ts" }), /cap is 480/);
  await assert.rejects(generateAeroDatabase(built, big, { engine: "wasm", pointCap: 100 }), /cap is 100/);
  const progress: number[] = [];
  const bigDb = await generateAeroDatabase(built, big, { engine: "pool", pool: nodePool, onProgress: (p) => progress.push(p.done) });
  assert.equal(bigDb.points.length, nBig);
  for (let i = 1; i < progress.length; i++) assert.ok(progress[i] >= progress[i - 1], "progress is monotone");
  assert.equal(progress[progress.length - 1], nBig, "progress ends at total");
  // cancellation: pool and calling thread both reject with AbortError, and the pool is usable afterwards
  for (const engine of ["pool", "wasm", "ts"] as const) {
    const ac = new AbortController();
    let seen = 0;
    const run = generateAeroDatabase(built, spec("cbaero"), {
      engine,
      pool: nodePool,
      chunkSize: 4,
      signal: ac.signal,
      onProgress: (p) => {
        if (p.done > 0 && ++seen === 2) ac.abort();
      },
    });
    await assert.rejects(run, (e: unknown) => e instanceof DOMException && e.name === "AbortError", `${engine} abort`);
  }
  const again = await generateAeroDatabase(built, spec("newtonian"), { engine: "pool", pool: nodePool });
  assert.equal(again.meta.engine, "pool:3");
  // the TS fallback's forcesOnly does not change coefficients
  {
    const atm = atmosphere(30, 8, built.params.gamma);
    const fp = { ...built.params, lockFlight: false, flightMach: 8, aeroMethod: "cbaero" as const };
    const a = panelAero(built.mesh, fp, atm, 5, 2, built.aero.planformArea, built.params.length, undefined, false);
    const b = panelAero(built.mesh, fp, atm, 5, 2, built.aero.planformArea, built.params.length, undefined, true);
    for (const k of ["cl", "cd", "cy", "cm", "cn", "cll", "ca", "ld"] as const) assert.equal(a[k], b[k], `forcesOnly ${k}`);
  }
  console.log("behaviour: default options, caps, progress, AbortSignal (pool / wasm / ts), forcesOnly identity OK");
} finally {
  nodePool.terminate();
  loopPool.terminate();
}
console.log("aerodb-perf tests passed");
