/**
 * Symmetry-plane kernel in TypeScript: the optimised, selectable-grid counterpart of the Rust
 * crate in native/cuspis-euler (solve_wall_grid). Same physics as the shipped wasm (first-order HLLC
 * by default, slip wall, local time step, freestream inflow / far field, extrapolated outflow);
 * `strip-euler-ref.ts` is the untouched line-for-line port that this file is tested against.
 *
 * Why it exists: the shipped wasm hard-codes 88x28 cells x 42 iterations and cannot be rebuilt in this
 * environment (no wasm32 target). `medium` keeps using that wasm bit-for-bit; every other setting runs here
 * until the wasm is rebuilt from the updated crate.
 *
 * What is added on top of the reference algorithm (all opt-in; the defaults reproduce the reference):
 *  - convergence-based stopping (`tol`: residual drop relative to its running peak, held for 3 sweeps, plus a
 *    wall-pressure stationarity check) with `iters` as the hard cap;
 *  - grid sequencing (`sequence` = number of coarser levels, each solved first and injected into the finer one);
 *  - optional second order (`order: 2`): MUSCL on primitive variables with a van Albada or minmod limiter;
 *  - struct-of-arrays state in typed arrays, no allocation inside the sweep loop, precomputed face normals /
 *    areas / time-step factors, cached sound speed (one sqrt per cell update instead of ~5);
 *  - a generator driver so a solve can be chunked (see `solveStripAsync`) without blocking the UI thread.
 */
import {
  LEGACY,
  clusterCoordinates,
  detectShockFractions,
  smoothFractions,
  stripDims,
  type VolumeRefinement,
} from "./refinement";

export type Limiter = "vanalbada" | "minmod";

export interface StripGrid {
  ni: number;
  nj: number;
  /** Fixed iteration count, or the hard cap when `tol` > 0. */
  iters: number;
  /** Legacy wall-stretch exponent: eta = s^wallExp (1.55 in the shipped kernel). */
  wallExp: number;
  wallClustering: number;
  shockClustering: number;
  shockWidth: number;
  /** Per node column (ni + 1 values): predicted shock position in s. Required for shockClustering / adaptive. */
  shockS?: Float64Array | null;
  adaptive?: boolean;
  /** > 0: stop when the residual has dropped below tol x its running peak (3 sweeps in a row) and wall pressure has settled. 0 / undefined: fixed `iters`. */
  tol?: number;
  /** Never stop before this many sweeps on the finest level (default 30). */
  minIters?: number;
  /** 1 (default, shipped scheme) or 2 (MUSCL). */
  order?: 1 | 2;
  limiter?: Limiter;
  /** Coarser levels solved first and injected (0 = off). Needs tol > 0, ignored otherwise. */
  sequence?: number;
  /** CFL of the local time step (0.5, the shipped value, by default). */
  cfl?: number;
}

export function legacyStripGrid(): StripGrid {
  return {
    ni: LEGACY.strip.ni,
    nj: LEGACY.strip.nj,
    iters: LEGACY.strip.iters,
    wallExp: LEGACY.strip.wallExp,
    wallClustering: 0,
    shockClustering: 0,
    shockWidth: 0.09,
  };
}

function wallZ(x: number, xw: ArrayLike<number>, zw: ArrayLike<number>): number {
  if (x <= xw[0]) return zw[0];
  const last = xw.length - 1;
  if (x >= xw[last]) return zw[last];
  let lo = 0;
  let hi = last;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (xw[mid] <= x) lo = mid;
    else hi = mid;
  }
  const dx = xw[hi] - xw[lo];
  const t = Math.abs(dx) < 1e-14 ? 0 : (x - xw[lo]) / dx;
  return zw[lo] + (zw[hi] - zw[lo]) * t;
}

/** Domain geometry shared with the Rust kernel (and with the shock-height estimate in sym-euler.ts). */
export function stripDomain(xw: ArrayLike<number>, zw: ArrayLike<number>) {
  const n = xw.length;
  const xNose = xw[0];
  const xTe = xw[n - 1];
  const length = Math.max(Math.abs(xTe - xNose), 1e-4);
  const sign = zw[Math.floor(n / 2)] < -1e-8 ? -1 : 1;
  return {
    xmin: xNose - 0.12 * length,
    xmax: xTe + 0.02 * length,
    hFar: 0.55 * length,
    sign,
    length,
  };
}

export interface StripResult {
  cp: Float64Array;
  /** Mass-flux residual (RMS of d(rho)/dt) at the last sweep of the finest level. */
  residual: number;
  grid: {
    ni: number;
    nj: number;
    iters: number;
    passes: number;
    /** Sweeps actually run on the finest level (last pass). */
    itersUsed: number;
    /** Grid levels used in the last pass (1 = no sequencing). */
    levels: number;
  };
  /** True when the tolerance test (not the iteration cap) ended the finest level. Always false for fixed-iteration runs. */
  converged: boolean;
  /** Why the finest level stopped: tolerance, residual floor with stationary wall pressure, or the iteration cap. */
  stopReason: StopReason;
  /** True when the requested CFL diverged and the solve was repeated at 0.5. */
  cflFallback: boolean;
  /** Cell updates performed over all levels and passes (the cost measure). */
  work: number;
  /** Diagnostic: cell-column centres and the density-jump (half-jump) shock height above the wall (NaN: none found). */
  shock: { x: Float64Array; height: Float64Array };
}

// ---------------------------------------------------------------------------
// HLLC (same algebra as the Rust kernel; normals / areas precomputed)
// ---------------------------------------------------------------------------
const F = new Float64Array(4);

function hllc(
  rL: number,
  uL: number,
  vL: number,
  pL: number,
  aL: number,
  rR: number,
  uR: number,
  vR: number,
  pR: number,
  aR: number,
  nx: number,
  ny: number,
  area: number,
  gm1inv: number,
) {
  const tx = -ny;
  const ty = nx;
  const unL = uL * nx + vL * ny;
  const utL = uL * tx + vL * ty;
  const unR = uR * nx + vR * ny;
  const utR = uR * tx + vR * ty;
  const eL = pL * gm1inv + 0.5 * rL * (uL * uL + vL * vL);
  const eR = pR * gm1inv + 0.5 * rR * (uR * uR + vR * vR);
  const am = aL > aR ? aL : aR;
  const sl = (unL < unR ? unL : unR) - am;
  const sr = (unL > unR ? unL : unR) + am;
  const den = rL * (sl - unL) - rR * (sr - unR);
  const sm =
    Math.abs(den) < 1e-14
      ? 0.5 * (unL + unR)
      : (pR - pL + rL * unL * (sl - unL) - rR * unR * (sr - unR)) / den;
  let fr: number;
  let fun: number;
  let fut: number;
  let fe: number;
  if (sl >= 0) {
    fr = rL * unL;
    fun = rL * unL * unL + pL;
    fut = rL * unL * utL;
    fe = (eL + pL) * unL;
  } else if (sr <= 0) {
    fr = rR * unR;
    fun = rR * unR * unR + pR;
    fut = rR * unR * utR;
    fe = (eR + pR) * unR;
  } else {
    const left = sm >= 0;
    const r = left ? rL : rR;
    const un = left ? unL : unR;
    const ut = left ? utL : utR;
    const p = left ? pL : pR;
    const e = left ? eL : eR;
    const s = left ? sl : sr;
    const coeff = (r * (s - un)) / (s - sm);
    const eOver = e / r + (sm - un) * (sm + p / (r * (s - un)));
    fr = r * un + s * (coeff - r);
    fun = r * un * un + p + s * (coeff * sm - r * un);
    fut = r * un * ut + s * (coeff * ut - r * ut);
    fe = (e + p) * un + s * (coeff * eOver - e);
  }
  F[0] = fr * area;
  F[1] = (fun * nx + fut * tx) * area;
  F[2] = (fun * ny + fut * ty) * area;
  F[3] = fe * area;
}

// ---------------------------------------------------------------------------
// Grid levels
// ---------------------------------------------------------------------------
const G = 14; // per-cell geometry stride: 4 faces x (nx, ny, area), dtGeom, invVol
const WS = 5; // primitive stride: r, u, v, p, a

interface Level {
  ni: number;
  nj: number;
  nc: number;
  geo: Float64Array;
  xc: Float64Array;
  colS: Float64Array[];
  U: Float64Array;
  W: Float64Array;
}

interface Flow {
  gamma: number;
  gm1: number;
  gm1inv: number;
  rhoInf: number;
  uInf: number;
  vInf: number;
  pInf: number;
  aInf: number;
  eInf: number;
}

function buildLevel(
  ni: number,
  nj: number,
  xw: ArrayLike<number>,
  zw: ArrayLike<number>,
  dom: ReturnType<typeof stripDomain>,
  g: StripGrid,
  shockCols: Float64Array | null,
  shockAmp: number,
  clustered: boolean,
): Level {
  const { xmin, xmax, hFar, sign } = dom;
  const npx = ni + 1;
  const npz = nj + 1;
  const nc = ni * nj;
  const px = new Float64Array(npx * npz);
  const pz = new Float64Array(npx * npz);
  const colS: Float64Array[] = [];
  if (clustered) {
    for (let i = 0; i <= ni; i++) {
      const sc = shockCols ? Math.min(Math.max(shockCols[i], 0.04), 0.92) : 0.3;
      colS.push(
        clusterCoordinates(nj, {
          wall: g.wallClustering,
          shock: shockCols ? shockAmp : 0,
          shockPos: sc,
          shockWidth: g.shockWidth,
        }),
      );
    }
  }
  const legacyEta = !clustered && g.wallExp === 1.55;
  for (let i = 0; i < npx; i++) {
    const xf = i / ni;
    const x = xmin + (xmax - xmin) * xf;
    const zwall = x <= xw[0] ? zw[0] : wallZ(x, xw, zw);
    for (let j = 0; j < npz; j++) {
      const s = clustered ? colS[i][j] : j / nj;
      const eta = legacyEta || g.wallExp !== 1 ? Math.pow(s, g.wallExp) : s;
      const id = j * npx + i;
      px[id] = x;
      pz[id] = zwall + sign * eta * hFar;
    }
  }
  const geo = new Float64Array(nc * G);
  const xc = new Float64Array(nc);
  for (let j = 0; j < nj; j++) {
    for (let i = 0; i < ni; i++) {
      const sw = j * npx + i;
      const se = sw + 1;
      const nw = (j + 1) * npx + i;
      const ne = nw + 1;
      const id = j * ni + i;
      const o = id * G;
      const x0 = px[sw];
      const x1 = px[se];
      const x2 = px[ne];
      const x3 = px[nw];
      const z0 = pz[sw];
      const z1 = pz[se];
      const z2 = pz[ne];
      const z3 = pz[nw];
      const a2 = x0 * z1 - x1 * z0 + (x1 * z2 - x2 * z1) + (x2 * z3 - x3 * z2) + (x3 * z0 - x0 * z3);
      const vol = Math.max(Math.abs(a2 * 0.5), 1e-14);
      xc[id] = 0.25 * (x0 + x1 + x2 + x3);
      // face area vectors (same orientation convention as the Rust kernel), stored as unit normal + area
      const fx = [-(z3 - z0), z2 - z1, z1 - z0, -(z2 - z3)];
      const fz = [x3 - x0, -(x2 - x1), -(x1 - x0), x2 - x3];
      let per = 1e-8;
      for (let f = 0; f < 4; f++) {
        const ar = Math.sqrt(fx[f] * fx[f] + fz[f] * fz[f]);
        if (ar > per) per = ar;
        geo[o + f * 3] = ar < 1e-14 ? 0 : fx[f] / ar;
        geo[o + f * 3 + 1] = ar < 1e-14 ? 0 : fz[f] / ar;
        geo[o + f * 3 + 2] = ar < 1e-14 ? 0 : ar;
      }
      geo[o + 12] = vol / per;
      geo[o + 13] = 1 / Math.max(vol, 1e-12);
    }
  }
  return { ni, nj, nc, geo, xc, colS, U: new Float64Array(nc * 4), W: new Float64Array((nc + 1) * WS) };
}

function initUniform(L: Level, fl: Flow) {
  const { U, W, nc } = L;
  for (let c = 0; c < nc; c++) {
    U[c * 4] = fl.rhoInf;
    U[c * 4 + 1] = fl.rhoInf * fl.uInf;
    U[c * 4 + 2] = fl.rhoInf * fl.vInf;
    U[c * 4 + 3] = fl.eInf;
  }
  refreshW(L, fl);
}

function refreshW(L: Level, fl: Flow) {
  const { U, W, nc } = L;
  for (let c = 0; c < nc; c++) setW(U, W, c, fl);
  const o = nc * WS;
  W[o] = fl.rhoInf;
  W[o + 1] = fl.uInf;
  W[o + 2] = fl.vInf;
  W[o + 3] = fl.pInf;
  W[o + 4] = fl.aInf;
}

function setW(U: Float64Array, W: Float64Array, c: number, fl: Flow) {
  const o = c * 4;
  const r = Math.max(U[o], 1e-8);
  const uu = U[o + 1] / r;
  const vv = U[o + 2] / r;
  const p = Math.max(fl.gm1 * (U[o + 3] - 0.5 * r * (uu * uu + vv * vv)), 1e-8);
  const w = c * WS;
  W[w] = r;
  W[w + 1] = uu;
  W[w + 2] = vv;
  W[w + 3] = p;
  W[w + 4] = Math.sqrt((fl.gamma * p) / r);
}

/** Inject a coarse solution into a finer level (piecewise-constant, nested index mapping). */
function prolong(coarse: Level, fine: Level, fl: Flow) {
  const { ni: nic, nj: njc } = coarse;
  const { ni, nj } = fine;
  for (let j = 0; j < nj; j++) {
    const jc = Math.min(njc - 1, Math.floor((j * njc) / nj));
    for (let i = 0; i < ni; i++) {
      const ic = Math.min(nic - 1, Math.floor((i * nic) / ni));
      const a = (jc * nic + ic) * 4;
      const b = (j * ni + i) * 4;
      fine.U[b] = coarse.U[a];
      fine.U[b + 1] = coarse.U[a + 1];
      fine.U[b + 2] = coarse.U[a + 2];
      fine.U[b + 3] = coarse.U[a + 3];
    }
  }
  refreshW(fine, fl);
}

// ---------------------------------------------------------------------------
// Sweeps
// ---------------------------------------------------------------------------
interface March {
  level: Level;
  /** CFL used by the following sweeps. */
  setCfl: (c: number) => void;
  /** Sweep n times; returns the residual after the last one. */
  step: (n: number) => number;
  /** Sweeps run so far. */
  done: () => number;
  /** Wall-row primitive pressure (for the stationarity check), written into `out`. */
  wallPressure: (out: Float64Array) => void;
  residual: () => number;
  /** Largest residual seen (shared across the levels of one solve: the residual is a rate per unit volume, comparable between grids). */
  peak: () => number;
}

function vanAlbada(a: number, b: number): number {
  if (a * b <= 0) return 0;
  const eps = 1e-12;
  return ((a * a + eps) * b + (b * b + eps) * a) / (a * a + b * b + 2 * eps);
}
function minmod(a: number, b: number): number {
  if (a * b <= 0) return 0;
  return Math.abs(a) < Math.abs(b) ? a : b;
}

function makeMarch(L: Level, fl: Flow, order: 1 | 2, limiter: Limiter, peakRef: { v: number }): March {
  const { ni: NI, nj: NJ, nc, geo, U, W } = L;
  const { gamma, gm1, gm1inv, rhoInf, uInf, vInf, pInf, aInf } = fl;
  const second = order === 2;
  const lim = limiter === "minmod" ? minmod : vanAlbada;
  const fsO = nc * WS; // freestream slot
  let sweeps = 0;
  let residual = 1;
  let cflNow = 0.5;

  // face flux for the cell c (left state) against neighbour nb (right state); cm / nbp are the cells one step
  // further on each side, used by the second-order reconstruction (-1: not available -> first order).
  const face = (c: number, cm: number, nb: number, nbp: number, nx: number, ny: number, area: number) => {
    if (area === 0) {
      F[0] = F[1] = F[2] = F[3] = 0;
      return;
    }
    const a = c * WS;
    const b = nb * WS;
    if (second && cm >= 0 && nbp >= 0) {
      const m = cm * WS;
      const q = nbp * WS;
      const rL = W[a] + 0.5 * lim(W[a] - W[m], W[b] - W[a]);
      const uL = W[a + 1] + 0.5 * lim(W[a + 1] - W[m + 1], W[b + 1] - W[a + 1]);
      const vL = W[a + 2] + 0.5 * lim(W[a + 2] - W[m + 2], W[b + 2] - W[a + 2]);
      const pL = W[a + 3] + 0.5 * lim(W[a + 3] - W[m + 3], W[b + 3] - W[a + 3]);
      const rR = W[b] - 0.5 * lim(W[b] - W[a], W[q] - W[b]);
      const uR = W[b + 1] - 0.5 * lim(W[b + 1] - W[a + 1], W[q + 1] - W[b + 1]);
      const vR = W[b + 2] - 0.5 * lim(W[b + 2] - W[a + 2], W[q + 2] - W[b + 2]);
      const pR = W[b + 3] - 0.5 * lim(W[b + 3] - W[a + 3], W[q + 3] - W[b + 3]);
      if (rL > 1e-8 && pL > 1e-8 && rR > 1e-8 && pR > 1e-8) {
        hllc(
          rL, uL, vL, pL, Math.sqrt((gamma * pL) / rL),
          rR, uR, vR, pR, Math.sqrt((gamma * pR) / rR),
          nx, ny, area, gm1inv,
        );
        return;
      }
    }
    hllc(W[a], W[a + 1], W[a + 2], W[a + 3], W[a + 4], W[b], W[b + 1], W[b + 2], W[b + 3], W[b + 4], nx, ny, area, gm1inv);
  };

  const step = (n: number): number => {
    for (let it = 0; it < n; it++) {
      let resSum = 0;
      for (let j = 0; j < NJ; j++) {
        for (let i = 0; i < NI; i++) {
          const id = j * NI + i;
          const g0 = id * G;
          const o = id * 4;
          // west
          face(id, i + 1 < NI ? id + 1 : -1, i === 0 ? fsO / WS : id - 1, i >= 2 ? id - 2 : -1, geo[g0], geo[g0 + 1], geo[g0 + 2]);
          let d0 = F[0];
          let d1 = F[1];
          let d2 = F[2];
          let d3 = F[3];
          // east (outflow: zero-gradient, i.e. the cell itself)
          face(id, i >= 1 ? id - 1 : -1, i + 1 === NI ? id : id + 1, i + 2 < NI ? id + 2 : -1, geo[g0 + 3], geo[g0 + 4], geo[g0 + 5]);
          d0 += F[0];
          d1 += F[1];
          d2 += F[2];
          d3 += F[3];
          // north (far field = freestream)
          face(id, j >= 1 ? id - NI : -1, j + 1 === NJ ? fsO / WS : id + NI, j + 2 < NJ ? id + 2 * NI : -1, geo[g0 + 9], geo[g0 + 10], geo[g0 + 11]);
          d0 += F[0];
          d1 += F[1];
          d2 += F[2];
          d3 += F[3];
          // south: slip wall (pressure only) for j = 0
          if (j === 0) {
            const pw = W[id * WS + 3];
            d1 += pw * geo[g0 + 6] * geo[g0 + 8];
            d2 += pw * geo[g0 + 7] * geo[g0 + 8];
          } else {
            face(id, j + 1 < NJ ? id + NI : -1, id - NI, j >= 2 ? id - 2 * NI : -1, geo[g0 + 6], geo[g0 + 7], geo[g0 + 8]);
            d0 += F[0];
            d1 += F[1];
            d2 += F[2];
            d3 += F[3];
          }
          const w = id * WS;
          const spec = Math.sqrt(W[w + 1] * W[w + 1] + W[w + 2] * W[w + 2]) + W[w + 4];
          const scale = ((cflNow * geo[g0 + 12]) / (spec > 1e-6 ? spec : 1e-6)) * geo[g0 + 13];
          const nr = U[o] - d0 * scale;
          const nru = U[o + 1] - d1 * scale;
          const nrv = U[o + 2] - d2 * scale;
          const nre = U[o + 3] - d3 * scale;
          const rr = nr > 1e-8 ? nr : 1e-8;
          const u2 = nru / rr;
          const v2 = nrv / rr;
          const pnew = gm1 * (nre - 0.5 * rr * (u2 * u2 + v2 * v2));
          if (pnew > 1e-6 && nr > 1e-8) {
            U[o] = nr;
            U[o + 1] = nru;
            U[o + 2] = nrv;
            U[o + 3] = nre;
            W[w] = nr;
            W[w + 1] = u2;
            W[w + 2] = v2;
            W[w + 3] = pnew;
            W[w + 4] = Math.sqrt((gamma * pnew) / nr);
          }
          const src = d0 * geo[g0 + 13];
          resSum += src * src;
        }
      }
      residual = Math.sqrt(resSum / nc);
      if (residual > peakRef.v) peakRef.v = residual;
      sweeps++;
    }
    return residual;
  };
  void rhoInf;
  void uInf;
  void vInf;
  void pInf;
  void aInf;
  return {
    level: L,
    setCfl: (c) => {
      cflNow = c;
    },
    step,
    done: () => sweeps,
    wallPressure: (out) => {
      for (let i = 0; i < NI; i++) out[i] = W[i * WS + 3];
    },
    residual: () => residual,
    peak: () => peakRef.v,
  };
}

/** The kernel grid a volume refinement stands for (the single mapping used by sym-euler.ts, the study and the tests). */
export function stripGridOf(v: VolumeRefinement, shockS?: Float64Array | null): StripGrid {
  const d = stripDims(v);
  return {
    ni: d.ni,
    nj: d.nj,
    iters: d.iters,
    wallExp: LEGACY.strip.wallExp,
    wallClustering: v.wallClustering,
    shockClustering: v.shockClustering,
    shockWidth: v.shockWidth,
    shockS: shockS ?? null,
    adaptive: v.adaptive,
    tol: v.tol ?? 0,
    order: v.order ?? 1,
    sequence: v.sequence ?? 0,
  };
}

const CHUNK = 20;

export type StopReason = "tol" | "stalled" | "cap";

/**
 * Run one level until `iters` sweeps, convergence or stagnation. Yields progress (0..1) every `chunk` sweeps.
 *  - "tol": residual <= tol x (running peak) for 3 sweeps in a row AND wall pressure moved < 2 tol per 10 sweeps;
 *  - "stalled": residual floor reached (no 10 % drop over 150 sweeps) while the wall pressure is stationary
 *    (typical when the shock leaves through the far-field boundary: a residual floor that is not a
 *    convergence failure of the wall solution);
 *  - "cap": ran all `iters` sweeps.
 */
function* runLevel(
  m: March,
  iters: number,
  tol: number,
  minIters: number,
  chunk: number,
  cflMax: number,
): Generator<number, { reason: StopReason; used: number }> {
  const NI = m.level.ni;
  const prev = new Float64Array(NI);
  const cur = new Float64Array(NI);
  let below = 0;
  let reason: StopReason = "cap";
  let used = 0;
  m.wallPressure(prev);
  let sinceSnap = 0;
  let winMin = Infinity;
  let lastWinMin = Infinity;
  let sinceWin = 0;
  const wallMoved = () => {
    m.wallPressure(cur);
    let dmax = 0;
    for (let i = 0; i < NI; i++) {
      const d = Math.abs(cur[i] - prev[i]) / Math.max(prev[i], 1e-8);
      if (d > dmax) dmax = d;
    }
    return dmax;
  };
  while (used < iters) {
    const n = Math.min(tol > 0 ? 1 : chunk, iters - used);
    // CFL ramps from the shipped 0.5 to its target over the first 60 sweeps (tolerance runs only)
    m.setCfl(tol > 0 && cflMax > 0.5 ? Math.min(cflMax, 0.5 + ((cflMax - 0.5) * used) / 60) : cflMax);
    const res = m.step(n);
    used += n;
    if (tol > 0) {
      sinceSnap += n;
      sinceWin += n;
      if (res < winMin) winMin = res;
      if (used >= minIters) {
        const pk = m.peak();
        if (pk > 0 && res <= tol * pk) below++;
        else below = 0;
        if (below >= 3 && sinceSnap >= 10) {
          const d = wallMoved();
          if (d <= 2 * tol * (sinceSnap / 10)) {
            reason = "tol";
            break;
          }
          prev.set(cur);
          sinceSnap = 0;
        } else if (sinceWin >= 150) {
          if (winMin > 0.9 * lastWinMin && sinceSnap >= 10) {
            const d = wallMoved();
            if (d <= 2 * tol * (sinceSnap / 10)) {
              reason = "stalled";
              break;
            }
            prev.set(cur);
            sinceSnap = 0;
          }
          lastWinMin = winMin;
          winMin = Infinity;
          sinceWin = 0;
        }
      }
    }
    if (used % chunk === 0 || used >= iters) yield used / iters;
  }
  return { reason, used };
}

export function* stripGen(
  xw: ArrayLike<number>,
  zw: ArrayLike<number>,
  mach: number,
  gamma: number,
  alpha: number,
  g: StripGrid,
  chunk = CHUNK,
): Generator<number, StripResult> {
  const n = xw.length;
  const dom = stripDomain(xw, zw);
  const NI = g.ni;
  const NJ = g.nj;
  const clustered = g.wallClustering > 0 || g.shockClustering > 0 || !!g.adaptive;
  const shockAmp = g.shockClustering > 0 ? g.shockClustering : g.adaptive ? 6 : 0;
  const tol = g.tol && g.tol > 0 ? g.tol : 0;
  const order = g.order === 2 ? 2 : 1;
  // first order tolerates 0.8 (ramped); the MUSCL scheme needs the shipped 0.5
  const cflReq = Math.min(Math.max(g.cfl ?? (tol > 0 && order === 1 ? 0.8 : 0.5), 0.05), order === 2 ? 0.5 : 0.9);
  let cfl = cflReq;
  const minIters = g.minIters ?? 30;
  const limiter = g.limiter ?? "vanalbada";
  const gm1 = gamma - 1;
  const rhoInf = 1;
  const pInf = 1;
  const aInf = Math.sqrt((gamma * pInf) / rhoInf);
  const m = Math.max(mach, 1.05);
  const uInf = m * aInf * Math.cos(alpha);
  const vInf = m * aInf * Math.sin(alpha);
  const eInf = pInf / gm1 + 0.5 * rhoInf * (uInf * uInf + vInf * vInf);
  const fl: Flow = { gamma, gm1, gm1inv: 1 / gm1, rhoInf, uInf, vInf, pInf, aInf, eInf };
  let work = 0;
  let stop: StopReason = "cap";
  const peakRef = { v: 0 };
  let itersUsed = 0;
  let levelsUsed = 1;
  let lastResidual = 1;

  const solveOn = function* (shockCols: Float64Array | null, iters: number): Generator<number, Level> {
    const nSeq = tol > 0 ? Math.max(0, Math.min(3, Math.floor(g.sequence ?? 0))) : 0;
    const dims: [number, number][] = [];
    for (let k = nSeq; k >= 1; k--) {
      const ci = Math.max(16, Math.round(NI / 2 ** k));
      const cj = Math.max(8, Math.round(NJ / 2 ** k));
      const last = dims[dims.length - 1];
      if (ci < NI && cj < NJ && (!last || last[0] !== ci || last[1] !== cj)) dims.push([ci, cj]);
    }
    dims.push([NI, NJ]);
    let prevL: Level | null = null;
    let L: Level = null as unknown as Level;
    for (let k = 0; k < dims.length; k++) {
      const [li, lj] = dims[k];
      const cols = shockCols
        ? li === NI
          ? shockCols
          : Float64Array.from({ length: li + 1 }, (_, i) => shockCols[Math.min(NI, Math.round((i * NI) / li))])
        : null;
      L = buildLevel(li, lj, xw, zw, dom, g, cols, shockAmp, clustered);
      if (prevL) prolong(prevL, L, fl);
      else initUniform(L, fl);
      const mm = makeMarch(L, fl, order, limiter, peakRef);
      const last = k === dims.length - 1;
      const lt = last ? tol : tol * 10;
      const res = yield* runLevel(mm, iters, lt, last ? minIters : Math.min(minIters, 20), chunk, cfl);
      work += res.used * li * lj;
      if (last) {
        stop = res.reason;
        itersUsed = res.used;
        levelsUsed = dims.length;
        lastResidual = mm.residual();
      }
      prevL = L;
    }
    return L;
  };

  let fellBack = false;
  for (let attempt = 0; ; attempt++) {
    let run: Level;
    let passes = 1;
    if (g.adaptive && g.shockS) {
      const halfIters = Math.max(10, Math.round(g.iters * 0.5));
      const half = yield* solveOn(g.shockS, halfIters);
      const rho = new Float64Array(NI * NJ);
      for (let c = 0; c < NI * NJ; c++) rho[c] = half.U[c * 4];
      const sOfRow = (i: number, j: number) =>
        0.25 * (half.colS[i][j] + half.colS[i][j + 1] + half.colS[i + 1][j] + half.colS[i + 1][j + 1]);
      const raw = detectShockFractions(rho, NI, NJ, sOfRow, rhoInf, 0.35);
      const fb = new Float64Array(NI);
      for (let i = 0; i < NI; i++) fb[i] = 0.5 * (g.shockS[i] + g.shockS[i + 1]);
      const sm = smoothFractions(raw, fb, 8);
      const nodeS = new Float64Array(NI + 1);
      for (let i = 0; i <= NI; i++)
        nodeS[i] = i === 0 ? sm[0] : i === NI ? sm[NI - 1] : 0.5 * (sm[i - 1] + sm[i]);
      run = yield* solveOn(nodeS, g.iters);
      passes = 2;
    } else {
      run = yield* solveOn(g.shockS ?? null, g.iters);
    }

    const q = 0.5 * gamma * m * m;
    const cp = new Float64Array(n);
    const { xc, W } = run;
    let healthy = true;
    for (let k = 0; k < n; k++) {
      const x = xw[k];
      let best = 0;
      let bestD = Infinity;
      for (let i = 0; i < NI; i++) {
        const d = Math.abs(xc[i] - x);
        if (d < bestD) {
          bestD = d;
          best = i;
        }
      }
      cp[k] = (W[best * WS + 3] - 1) / q;
      // physical sanity: wall Cp cannot exceed ~2 (Newtonian stagnation) nor fall below vacuum (-2/(gamma M^2))
      if (!Number.isFinite(cp[k]) || cp[k] > 2.6 || cp[k] < -2.2 / (gamma * m * m) - 0.05) healthy = false;
    }
    if ((!healthy || (tol > 0 && stop === "cap")) && cfl > 0.5 + 1e-9 && attempt === 0) {
      // the larger CFL diverged: redo at the shipped, always-stable 0.5
      cfl = 0.5;
      fellBack = true;
      continue;
    }
    const nc = NI * NJ;
    const rhoF = new Float64Array(nc);
    for (let c = 0; c < nc; c++) rhoF[c] = run.U[c * 4];
    const sRow = (i: number, j: number) =>
      run.colS.length
        ? 0.25 * (run.colS[i][j] + run.colS[i][j + 1] + run.colS[i + 1][j] + run.colS[i + 1][j + 1])
        : (j + 0.5) / NJ;
    const sShock = detectShockFractions(rhoF, NI, NJ, sRow, rhoInf, 0.35);
    const height = new Float64Array(NI);
    for (let i = 0; i < NI; i++)
      height[i] = Number.isFinite(sShock[i]) ? Math.pow(sShock[i], g.wallExp) * dom.hFar : NaN;
    return {
      cp,
      residual: lastResidual,
      grid: { ni: NI, nj: NJ, iters: g.iters, passes, itersUsed, levels: levelsUsed },
      converged: stop !== "cap" && tol > 0,
      stopReason: tol > 0 ? stop : "cap",
      cflFallback: fellBack,
      work,
      shock: { x: xc.slice(0, NI), height },
    };
  }
}

export function solveStrip(
  xw: ArrayLike<number>,
  zw: ArrayLike<number>,
  mach: number,
  gamma: number,
  alpha: number,
  g: StripGrid,
): StripResult {
  const it = stripGen(xw, zw, mach, gamma, alpha, g, 1 << 30);
  for (;;) {
    const r = it.next();
    if (r.done) return r.value;
  }
}

/**
 * Same result as `solveStrip`, but yields to the event loop every `chunk` sweeps so a UI thread stays
 * responsive. `signal` aborts (rejects with an AbortError); `onProgress` gets 0..1 per level chunk.
 */
export async function solveStripAsync(
  xw: ArrayLike<number>,
  zw: ArrayLike<number>,
  mach: number,
  gamma: number,
  alpha: number,
  g: StripGrid,
  opts: { chunk?: number; signal?: { aborted: boolean }; onProgress?: (p: number) => void } = {},
): Promise<StripResult> {
  const it = stripGen(xw, zw, mach, gamma, alpha, g, opts.chunk ?? 40);
  for (;;) {
    if (opts.signal?.aborted) throw new DOMException("Euler solve aborted", "AbortError");
    const r = it.next();
    if (r.done) return r.value;
    opts.onProgress?.(r.value);
    await new Promise<void>((res) => setTimeout(res, 0));
  }
}
