/**
 * Batch coefficient evaluation for the aero database: one mesh, many (M, α, β, rates) points.
 *
 * Two engines with the same contract:
 *  - "wasm": cuspis-panel (native/cuspis-panel via panel-kernel.ts). Per-panel loop in Rust.
 *  - "ts":   panelAero(..., forcesOnly = true), i.e. exactly what generateAeroDatabase computed before.
 *
 * The atmosphere-dependent scalars of a point (q, V, body Cf, rarefaction weight, Hayes–Probstein p/p_inv)
 * are computed here in TS with the same functions panelAero uses and handed to the kernel per point.
 * Elevon deflection is a different mesh (aerodb.ts rebuilds the OML), so a "case" is one mesh + params.
 */
import { atmosphere } from "./atmosphere";
import { bodyCf } from "./heating";
import { clamp, knudsen, rarefactionWeight, viscousChi, viscousInteractionPressure } from "./math";
import { panelAero, type RateState } from "./panel";
import { STRIDE_IN, STRIDE_OUT, kernelSupports, panelKernel, type KernelConfig, type PanelKernel } from "./panel-kernel";
import type { AeroMethod, DesignParams, TriMesh } from "./types";

export { STRIDE_IN, STRIDE_OUT };

/** Same set as panel.ts / shockexp.ts (both private there). */
const CONE_FAMILIES = new Set(["cone", "osculating", "elliptic", "viscopt", "star"]);

/** f64 per packed request point: mach, alphaDeg, betaDeg, hasRates, p, q, r */
export const STRIDE_PT = 7;

export interface BatchCase {
  mesh: TriMesh;
  /** Vehicle params with aeroMethod / elevonDeg already set (aerodb.ts `params`). */
  params: DesignParams;
  altKm: number;
  sRef: number;
  lRef: number;
}

export interface BatchPoint {
  mach: number;
  alpha: number;
  beta: number;
  rates?: RateState;
}

export type BatchEngine = "wasm" | "ts";

export function packPoints(points: BatchPoint[]): Float64Array {
  const out = new Float64Array(points.length * STRIDE_PT);
  points.forEach((p, i) => {
    const o = i * STRIDE_PT;
    out[o] = p.mach;
    out[o + 1] = p.alpha;
    out[o + 2] = p.beta;
    if (p.rates) {
      out[o + 3] = 1;
      out[o + 4] = p.rates.p;
      out[o + 5] = p.rates.q;
      out[o + 6] = p.rates.r;
    }
  });
  return out;
}

export function caseConfig(c: BatchCase): KernelConfig {
  const p = c.params;
  return {
    method: p.aeroMethod,
    gamma: p.gamma,
    useCone: CONE_FAMILIES.has(p.family),
    cgX: clamp(p.cgFrac, 0.2, 0.85) * p.length,
    sRef: c.sRef,
    lRef: c.lRef,
  };
}

/** Kernel input rows (STRIDE_IN) for packed points: the per-point scalars panelAero / shockexp derive from the atmosphere. */
export function kernelInputs(c: BatchCase, pts: Float64Array): Float64Array {
  const n = Math.floor(pts.length / STRIDE_PT);
  const out = new Float64Array(n * STRIDE_IN);
  const p = c.params;
  const g = p.gamma;
  const Tw = Math.max(200, p.twK);
  const L = Math.max(c.lRef, 1e-8);
  let lastMach = NaN;
  let scal = [0, 0, 0, 0, 0];
  for (let i = 0; i < n; i++) {
    const s = i * STRIDE_PT;
    const mach = pts[s];
    if (mach !== lastMach) {
      lastMach = mach;
      const atm = atmosphere(c.altKm, mach, g);
      const M = Math.max(1.05, Number.isFinite(mach) ? mach : 1.05);
      const kn = knudsen(atm.T, atm.p, Math.max(p.length, 1e-6));
      const chiBar = viscousChi(M, Math.max(atm.ReL * p.length, 1), Tw, atm.T);
      scal = [atm.V, atm.q, bodyCf(atm.ReL * L, M, atm.T, Tw, g), rarefactionWeight(kn), viscousInteractionPressure(chiBar)];
    }
    const o = i * STRIDE_IN;
    out[o] = mach;
    out[o + 1] = pts[s + 1];
    out[o + 2] = pts[s + 2];
    out[o + 3] = pts[s + 3];
    out[o + 4] = pts[s + 4];
    out[o + 5] = pts[s + 5];
    out[o + 6] = pts[s + 6];
    out[o + 7] = scal[0];
    out[o + 8] = scal[1];
    out[o + 9] = scal[2];
    out[o + 10] = scal[3];
    out[o + 11] = scal[4];
  }
  return out;
}

/** TS reference: panelAero with forcesOnly (heating skipped; coefficients unchanged). */
export function evalCaseTs(c: BatchCase, pts: Float64Array): Float64Array {
  const n = Math.floor(pts.length / STRIDE_PT);
  const out = new Float64Array(n * STRIDE_OUT);
  for (let i = 0; i < n; i++) {
    const s = i * STRIDE_PT;
    const mach = pts[s];
    const atm = atmosphere(c.altKm, mach, c.params.gamma);
    const flight: DesignParams = { ...c.params, lockFlight: false, flightMach: mach };
    const rates = pts[s + 3] ? { p: pts[s + 4], q: pts[s + 5], r: pts[s + 6] } : undefined;
    const a = panelAero(c.mesh, flight, atm, pts[s + 1], pts[s + 2], c.sRef, c.lRef, rates, true);
    const o = i * STRIDE_OUT;
    out[o] = a.cl;
    out[o + 1] = a.cd;
    out[o + 2] = a.cy;
    out[o + 3] = a.cm;
    out[o + 4] = a.cn;
    out[o + 5] = a.cll;
    out[o + 6] = a.ca;
    out[o + 7] = a.ld;
    out[o + 8] = a.cnA;
    out[o + 9] = a.cdFric;
    out[o + 10] = a.cdBase;
    out[o + 11] = a.xCp;
  }
  return out;
}

export function evalCaseWasm(kernel: PanelKernel, c: BatchCase, pts: Float64Array): Float64Array {
  return kernel.eval(c.mesh, caseConfig(c), kernelInputs(c, pts));
}

/** Can this case run in the kernel here? (method in the kernel and wasm instantiable) */
export function wasmAvailableFor(method: AeroMethod): boolean {
  return kernelSupports(method) && panelKernel() !== null;
}

/** Evaluate with the requested engine; "wasm" silently falls back to "ts" when the kernel cannot run it. */
export function evalCase(c: BatchCase, pts: Float64Array, engine: BatchEngine = "wasm"): { out: Float64Array; engine: BatchEngine } {
  if (engine === "wasm" && kernelSupports(c.params.aeroMethod)) {
    const k = panelKernel();
    if (k) return { out: evalCaseWasm(k, c, pts), engine: "wasm" };
  }
  return { out: evalCaseTs(c, pts), engine: "ts" };
}
