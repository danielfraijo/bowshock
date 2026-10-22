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
