/**
 * TypeScript port of the symmetry-plane kernel in native/cuspis-euler (solve_wall),
 * with the grid made selectable: cells, iterations, wall / shock clustering and an
 * optional solution-adaptive re-cluster.
 *
 * Why it exists: the shipped wasm (euler-wasm.ts) hard-codes 88x28 cells x 42 iterations
 * and cannot be rebuilt in this environment (no wasm32 target). `medium` keeps using that
 * wasm bit-for-bit; every other setting runs here until the wasm is rebuilt from the
 * updated crate (which has the same algorithm - see native/cuspis-euler/src/lib.rs tests).
 *
 * Numerics (first-order HLLC, slip wall, local dt, CFL 0.5, freestream inflow/far-field,
 * extrapolated outflow) are line-for-line the Rust ones. Do not change physics here.
 */
import { LEGACY, clusterCoordinates, detectShockFractions, smoothFractions } from "./refinement";

export interface StripGrid {
  ni: number;
  nj: number;
  iters: number;
  /** Legacy wall-stretch exponent: eta = s^wallExp (1.55 in the shipped kernel). */
  wallExp: number;
  wallClustering: number;
  shockClustering: number;
  shockWidth: number;
  /** Per node column (ni + 1 values): predicted shock position in s. Required for shockClustering / adaptive. */
  shockS?: Float64Array | null;
  adaptive?: boolean;
}

export function legacyStripGrid(): StripGrid {
  return { ni: LEGACY.strip.ni, nj: LEGACY.strip.nj, iters: LEGACY.strip.iters, wallExp: LEGACY.strip.wallExp, wallClustering: 0, shockClustering: 0, shockWidth: 0.09 };
}

const CFL = 0.5;

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
  return { xmin: xNose - 0.12 * length, xmax: xTe + 0.02 * length, hFar: 0.55 * length, sign, length };
}


const F = new Float64Array(4);
const NB = new Float64Array(4);

function hllc(rL: number, uL: number, vL: number, pL: number, rR: number, uR: number, vR: number, pR: number, ax: number, ay: number, gamma: number) {
  const area = Math.sqrt(ax * ax + ay * ay);
  if (area < 1e-14) {
    F[0] = F[1] = F[2] = F[3] = 0;
    return;
  }
  const nx = ax / area;
  const ny = ay / area;
  const tx = -ny;
  const ty = nx;
  const unL = uL * nx + vL * ny;
  const utL = uL * tx + vL * ty;
  const unR = uR * nx + vR * ny;
  const utR = uR * tx + vR * ty;
  const aL = Math.sqrt((gamma * pL) / rL);
  const aR = Math.sqrt((gamma * pR) / rR);
  const eL = pL / (gamma - 1) + 0.5 * rL * (uL * uL + vL * vL);
  const eR = pR / (gamma - 1) + 0.5 * rR * (uR * uR + vR * vR);
  const sl = Math.min(unL, unR) - Math.max(aL, aR);
  const sr = Math.max(unL, unR) + Math.max(aL, aR);
  const den = rL * (sl - unL) - rR * (sr - unR);
  const sm = Math.abs(den) < 1e-14 ? 0.5 * (unL + unR) : (pR - pL + rL * unL * (sl - unL) - rR * unR * (sr - unR)) / den;
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

export interface StripResult {
  cp: Float64Array;
  residual: number;
  grid: { ni: number; nj: number; iters: number; passes: number };
  /** Diagnostic: cell-column centres and the density-jump (half-jump) shock height above the wall (NaN: none found). */
  shock: { x: Float64Array; height: Float64Array };
}

export function solveStrip(
  xw: ArrayLike<number>,
  zw: ArrayLike<number>,
  mach: number,
  gamma: number,
  alpha: number,
  g: StripGrid,
): StripResult {
  const n = xw.length;
  const { xmin, xmax, hFar, sign } = stripDomain(xw, zw);
  const NI = g.ni;
  const NJ = g.nj;
  const clustered = g.wallClustering > 0 || g.shockClustering > 0 || !!g.adaptive;
  const shockAmp = g.shockClustering > 0 ? g.shockClustering : g.adaptive ? 6 : 0;
  const npx = NI + 1;
  const npz = NJ + 1;
  const nc = NI * NJ;
  const gm1 = gamma - 1;
  const rhoInf = 1;
  const pInf = 1;
  const aInf = Math.sqrt((gamma * pInf) / rhoInf);
  const m = Math.max(mach, 1.05);
  const uInf = m * aInf * Math.cos(alpha);
  const vInf = m * aInf * Math.sin(alpha);
  const eInf = pInf / gm1 + 0.5 * rhoInf * (uInf * uInf + vInf * vInf);

  const march = (shockS: Float64Array | null, iters: number) => {
    const px = new Float64Array(npx * npz);
    const pz = new Float64Array(npx * npz);
    const colS: Float64Array[] = [];
    if (clustered) {
      for (let i = 0; i <= NI; i++) {
        const sc = shockS ? Math.min(Math.max(shockS[i], 0.04), 0.92) : 0.3;
        colS.push(clusterCoordinates(NJ, { wall: g.wallClustering, shock: shockS ? shockAmp : 0, shockPos: sc, shockWidth: g.shockWidth }));
      }
    }
    for (let j = 0; j < npz; j++) {
      const s0 = j / NJ;
      for (let i = 0; i < npx; i++) {
        const xf = i / NI;
        const x = xmin + (xmax - xmin) * xf;
        const zwall = x <= xw[0] ? zw[0] : wallZ(x, xw, zw);
        const s = clustered ? colS[i][j] : s0;
        const eta = Math.pow(s, g.wallExp);
        const id = j * npx + i;
        px[id] = x;
        pz[id] = zwall + sign * eta * hFar;
      }
    }
    const vol = new Float64Array(nc);
    const fw = new Float64Array(nc * 2);
    const fe = new Float64Array(nc * 2);
    const fs = new Float64Array(nc * 2);
    const fn = new Float64Array(nc * 2);
    const xc = new Float64Array(nc);
    for (let j = 0; j < NJ; j++) {
      for (let i = 0; i < NI; i++) {
        const sw = j * npx + i;
        const se = sw + 1;
        const nw = (j + 1) * npx + i;
        const ne = nw + 1;
        const id = j * NI + i;
        const qx = [px[sw], px[se], px[ne], px[nw]];
        const qz = [pz[sw], pz[se], pz[ne], pz[nw]];
        let a = 0;
        for (let k = 0; k < 4; k++) {
          const nn = (k + 1) % 4;
          a += qx[k] * qz[nn] - qx[nn] * qz[k];
        }
        vol[id] = Math.max(Math.abs(a * 0.5), 1e-14);
        xc[id] = 0.25 * (qx[0] + qx[1] + qx[2] + qx[3]);
        fw[id * 2] = -(pz[nw] - pz[sw]);
        fw[id * 2 + 1] = px[nw] - px[sw];
        fe[id * 2] = pz[ne] - pz[se];
        fe[id * 2 + 1] = -(px[ne] - px[se]);
        fs[id * 2] = pz[se] - pz[sw];
        fs[id * 2 + 1] = -(px[se] - px[sw]);
        fn[id * 2] = -(pz[ne] - pz[nw]);
        fn[id * 2 + 1] = px[ne] - px[nw];
      }
    }
    const len = (arr: Float64Array, id: number) => Math.sqrt(arr[id * 2] * arr[id * 2] + arr[id * 2 + 1] * arr[id * 2 + 1]);
    const U = new Float64Array(nc * 4);
    for (let c = 0; c < nc; c++) {
      U[c * 4] = rhoInf;
      U[c * 4 + 1] = rhoInf * uInf;
      U[c * 4 + 2] = rhoInf * vInf;
      U[c * 4 + 3] = eInf;
    }
    // Gauss-Seidel-style in-place update, like the Rust kernel: neighbours are read live from U.
    const prim = (o: number) => {
      const r = Math.max(U[o], 1e-8);
      const uu = U[o + 1] / r;
      const vv = U[o + 2] / r;
      NB[0] = r;
      NB[1] = uu;
      NB[2] = vv;
      NB[3] = Math.max(gm1 * (U[o + 3] - 0.5 * r * (uu * uu + vv * vv)), 1e-8);
    };
    let residual = 1;
    for (let it = 0; it < iters; it++) {
      let resSum = 0;
      for (let j = 0; j < NJ; j++) {
        for (let i = 0; i < NI; i++) {
          const id = j * NI + i;
          const o = id * 4;
          prim(o);
          const r = NB[0];
          const uu = NB[1];
          const vv = NB[2];
          const pp = NB[3];
          let d0 = 0;
          let d1 = 0;
          let d2 = 0;
          let d3 = 0;
          if (i === 0) hllc(r, uu, vv, pp, rhoInf, uInf, vInf, pInf, fw[id * 2], fw[id * 2 + 1], gamma);
          else {
            prim((id - 1) * 4);
            hllc(r, uu, vv, pp, NB[0], NB[1], NB[2], NB[3], fw[id * 2], fw[id * 2 + 1], gamma);
          }
          d0 += F[0]; d1 += F[1]; d2 += F[2]; d3 += F[3];
          if (i + 1 === NI) hllc(r, uu, vv, pp, r, uu, vv, pp, fe[id * 2], fe[id * 2 + 1], gamma);
          else {
            prim((id + 1) * 4);
            hllc(r, uu, vv, pp, NB[0], NB[1], NB[2], NB[3], fe[id * 2], fe[id * 2 + 1], gamma);
          }
          d0 += F[0]; d1 += F[1]; d2 += F[2]; d3 += F[3];
          if (j + 1 === NJ) hllc(r, uu, vv, pp, rhoInf, uInf, vInf, pInf, fn[id * 2], fn[id * 2 + 1], gamma);
          else {
            prim((id + NI) * 4);
            hllc(r, uu, vv, pp, NB[0], NB[1], NB[2], NB[3], fn[id * 2], fn[id * 2 + 1], gamma);
          }
          d0 += F[0]; d1 += F[1]; d2 += F[2]; d3 += F[3];
          if (j === 0) {
            d1 += pp * fs[id * 2];
            d2 += pp * fs[id * 2 + 1];
          } else {
            prim((id - NI) * 4);
            hllc(r, uu, vv, pp, NB[0], NB[1], NB[2], NB[3], fs[id * 2], fs[id * 2 + 1], gamma);
            d0 += F[0]; d1 += F[1]; d2 += F[2]; d3 += F[3];
          }
          const spec = Math.sqrt(uu * uu + vv * vv) + Math.sqrt((gamma * pp) / r);
          const per = Math.max(len(fw, id), len(fe, id), len(fs, id), len(fn, id), 1e-8);
          const dt = (CFL * (vol[id] / per)) / Math.max(spec, 1e-6);
          const scale = dt / Math.max(vol[id], 1e-12);
          const nr = U[o] - d0 * scale;
          const nru = U[o + 1] - d1 * scale;
          const nrv = U[o + 2] - d2 * scale;
          const nre = U[o + 3] - d3 * scale;
          const rr = Math.max(nr, 1e-8);
          const u2 = nru / rr;
          const v2 = nrv / rr;
          const pnew = gm1 * (nre - 0.5 * rr * (u2 * u2 + v2 * v2));
          if (pnew > 1e-6 && nr > 1e-8) {
            U[o] = nr;
            U[o + 1] = nru;
            U[o + 2] = nrv;
            U[o + 3] = nre;
          }
          const src = d0 / Math.max(vol[id], 1e-12);
          resSum += src * src;
        }
      }
      residual = Math.sqrt(resSum / nc);
    }
    return { U, residual, xc, colS };
  };

  let run: ReturnType<typeof march>;
  let passes = 1;
  if (g.adaptive && g.shockS) {
    const half = march(g.shockS, Math.max(10, Math.round(g.iters * 0.5)));
    const rho = new Float64Array(nc);
    for (let c = 0; c < nc; c++) rho[c] = half.U[c * 4];
    const sOfRow = (i: number, j: number) => 0.25 * (half.colS[i][j] + half.colS[i][j + 1] + half.colS[i + 1][j] + half.colS[i + 1][j + 1]);
    const raw = detectShockFractions(rho, NI, NJ, sOfRow, rhoInf, 0.35);
    const fb = new Float64Array(NI);
    for (let i = 0; i < NI; i++) fb[i] = 0.5 * (g.shockS[i] + g.shockS[i + 1]);
    const sm = smoothFractions(raw, fb, 8);
    const nodeS = new Float64Array(NI + 1);
    for (let i = 0; i <= NI; i++) nodeS[i] = i === 0 ? sm[0] : i === NI ? sm[NI - 1] : 0.5 * (sm[i - 1] + sm[i]);
    run = march(nodeS, g.iters);
    passes = 2;
  } else {
    run = march(g.shockS ?? null, g.iters);
  }

  const q = 0.5 * gamma * m * m;
  const cp = new Float64Array(n);
  const { U, xc } = run;
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
    const o = best * 4;
    const r = Math.max(U[o], 1e-8);
    const uu = U[o + 1] / r;
    const vv = U[o + 2] / r;
    const p = Math.max(gm1 * (U[o + 3] - 0.5 * r * (uu * uu + vv * vv)), 1e-8);
    cp[k] = (p - 1) / q;
  }
  const rhoF = new Float64Array(nc);
  for (let c = 0; c < nc; c++) rhoF[c] = U[c * 4];
  const sRow = (i: number, j: number) =>
    run.colS.length ? 0.25 * (run.colS[i][j] + run.colS[i][j + 1] + run.colS[i + 1][j] + run.colS[i + 1][j + 1]) : (j + 0.5) / NJ;
  const sShock = detectShockFractions(rhoF, NI, NJ, sRow, rhoInf, 0.35);
  const height = new Float64Array(NI);
  for (let i = 0; i < NI; i++) height[i] = Number.isFinite(sShock[i]) ? Math.pow(sShock[i], g.wallExp) * hFar : NaN;
  return { cp, residual: run.residual, grid: { ni: NI, nj: NJ, iters: g.iters, passes }, shock: { x: xc.slice(0, NI), height } };
}

