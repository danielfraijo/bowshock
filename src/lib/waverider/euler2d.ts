/**
 * Steady 2-D Euler on a body-fitted ramp. First-order HLLC, local time step.
 * This is the Cart3D-class anchor for a wedge: an Euler solver must recover
 * the exact oblique shock. The panel method is scored against that same shock.
 *
 * Grid and scheme match native/bowshock_kernel.c.
 */
import { DEG, betaFromThetaM, obliqueShock } from "./math";
// [mesh-refinement] grid dimensions / clustering are now caller-selectable; defaults are the legacy 72x24x280.
import { LEGACY, clusterCoordinates, detectShockFractions, isBaseline, smoothFractions, type VolumeRefinement } from "./refinement";

export interface EulerAnchor {
  mach: number;
  thetaDeg: number;
  p2p1: number;
  exactP: number;
  relErr: number;
  cp: number;
  exactCp: number;
  residual: number;
  iters: number;
  /** [mesh-refinement] Shock angle located from the computed density field (half-jump crossing, least-squares line). */
  shockAngleDeg?: number;
  exactShockAngleDeg?: number;
  /** Grid actually used. */
  ni?: number;
  nj?: number;
  /** 1 = static grid; 2 = solution-adaptive re-cluster + re-solve. */
  passes?: number;
  /** Wall-clock of the solve in ms (not cached across calls with different settings). */
  ms?: number;
}

const NI0 = LEGACY.wedge.ni;
const NJ0 = LEGACY.wedge.nj;
const ITERS0 = LEGACY.wedge.iters;
const CFL = 0.45;

/** Test hook: scale the theory shock height used for static clustering (1 = nominal) to prove the adaptive pass recovers a bad estimate. */
export const wedgeShockBias = { value: 1 };

const F4 = new Float64Array(4);
const S4 = new Float64Array(4);
const eulerCache = new Map<string, EulerAnchor>();

function hllc(
  rL: number,
  uL: number,
  vL: number,
  pL: number,
  rR: number,
  uR: number,
  vR: number,
  pR: number,
  Ax: number,
  Ay: number,
  gamma: number,
) {
  const area = Math.hypot(Ax, Ay);
  if (area < 1e-14) {
    F4[0] = F4[1] = F4[2] = F4[3] = 0;
    return;
  }
  const nx = Ax / area;
  const ny = Ay / area;
  const tx = -ny;
  const ty = nx;
  const unL = uL * nx + vL * ny;
  const utL = uL * tx + vL * ty;
  const unR = uR * nx + vR * ny;
  const utR = uR * tx + vR * ty;
  const aL = Math.sqrt((gamma * pL) / rL);
  const aR = Math.sqrt((gamma * pR) / rR);
  const EL = pL / (gamma - 1) + 0.5 * rL * (uL * uL + vL * vL);
  const ER = pR / (gamma - 1) + 0.5 * rR * (uR * uR + vR * vR);
  const SL = Math.min(unL, unR) - Math.max(aL, aR);
  const SR = Math.max(unL, unR) + Math.max(aL, aR);
  const den = rL * (SL - unL) - rR * (SR - unR);
  const SM = den === 0 ? 0.5 * (unL + unR) : (pR - pL + rL * unL * (SL - unL) - rR * unR * (SR - unR)) / den;
  let fr: number;
  let fun: number;
  let fut: number;
  let fE: number;
  let r: number;
  let un: number;
  let ut: number;
  let p: number;
  let E: number;
  if (SL >= 0) {
    r = rL;
    un = unL;
    ut = utL;
    p = pL;
    E = EL;
    fr = r * un;
    fun = r * un * un + p;
    fut = r * un * ut;
    fE = (E + p) * un;
  } else if (SR <= 0) {
    r = rR;
    un = unR;
    ut = utR;
    p = pR;
    E = ER;
    fr = r * un;
    fun = r * un * un + p;
    fut = r * un * ut;
    fE = (E + p) * un;
  } else {
    const sideL = SM >= 0;
    r = sideL ? rL : rR;
    un = sideL ? unL : unR;
    ut = sideL ? utL : utR;
    p = sideL ? pL : pR;
    E = sideL ? EL : ER;
    const S = sideL ? SL : SR;
    const coeff = (r * (S - un)) / (S - SM);
    const Eover = E / r + (SM - un) * (SM + p / (r * (S - un)));
    const f0 = r * un;
    const f1 = r * un * un + p;
    const f2 = r * un * ut;
    const f3 = (E + p) * un;
    fr = f0 + S * (coeff - r);
    fun = f1 + S * (coeff * SM - r * un);
    fut = f2 + S * (coeff * ut - r * ut);
    fE = f3 + S * (coeff * Eover - E);
  }
  F4[0] = fr * area;
  F4[1] = (fun * nx + fut * tx) * area;
  F4[2] = (fun * ny + fut * ty) * area;
  F4[3] = fE * area;
}

function volKey(v?: VolumeRefinement): string {
  if (!v || isBaseline({ preset: "medium", surface: { nx: 0, ny: 0, leClustering: 0 }, volume: v })) return "";
  return `|${v.nStream}x${v.nCross}x${v.iters}|w${v.wallClustering}|s${v.shockClustering}|sw${v.shockWidth}|a${v.adaptive ? 1 : 0}|t${v.tol ?? 0}`;
}

/**
 * Wedge anchor. `vol` selects the grid (default: legacy 72x24x280, no clustering, bitwise unchanged).
 * With wall / shock clustering the column spacing is equidistributed toward the wall and toward the
 * theory shock line; `adaptive` re-clusters toward the computed density jump and re-solves.
 */
export function eulerWedge(M: number, thetaDeg: number, gamma = 1.4, vol?: VolumeRefinement): EulerAnchor {
  const cacheKey = `${M}|${thetaDeg}|${gamma}${volKey(vol)}${wedgeShockBias.value === 1 ? "" : `|bias${wedgeShockBias.value}`}`;
  const cached = eulerCache.get(cacheKey);
  if (cached) return cached;
  const solved = solveEulerWedge(M, thetaDeg, gamma, vol);
  eulerCache.set(cacheKey, solved);
  return solved;
}

/**
 * Dina: grid-study entry point (convergence test / scripts). Thin wrapper over eulerWedge(): static uniform grid,
 * no clustering, first order — i.e. the legacy anchor on an arbitrary ni × nj × iters.
 */
export function eulerWedgeGrid(M: number, thetaDeg: number, gamma: number, ni: number, nj: number, iters: number): EulerAnchor {
  return eulerWedge(M, thetaDeg, gamma, { nStream: ni, nCross: nj, iters, wallClustering: 0, shockClustering: 0, shockWidth: 0.09, adaptive: false });
}

function solveEulerWedge(M: number, thetaDeg: number, gamma = 1.4, volSet?: VolumeRefinement): EulerAnchor {
  const t0 = performance.now();
  const theta = thetaDeg * DEG;
  const beta = betaFromThetaM(M, theta, gamma);
  const exact = Number.isFinite(beta) ? obliqueShock(M, beta, gamma) : null;
  const exactP = exact ? exact.p2p1 : 1;
  const q = 0.5 * gamma * M * M;
  const exactCp = (exactP - 1) / q;
  if (!exact) {
    return { mach: M, thetaDeg, p2p1: 1, exactP, relErr: 1, cp: 0, exactCp, residual: 1, iters: 0 };
  }

  const NI = volSet ? volSet.nStream : NI0;
  const NJ = volSet ? volSet.nCross : NJ0;
  const ITERS = volSet ? volSet.iters : ITERS0;
  const wallAmp = volSet ? volSet.wallClustering : 0;
  const adaptive = !!volSet?.adaptive;
  const shockAmp = volSet ? (volSet.shockClustering > 0 ? volSet.shockClustering : adaptive ? 6 : 0) : 0;
  const shockW = volSet ? volSet.shockWidth : 0.09;
  const clustered = wallAmp > 0 || shockAmp > 0;
  const tolW = volSet?.tol && volSet.tol > 0 ? volSet.tol : 0;

  const Lx = 1;
  const x0 = 0.2;
  const tanBeta = Math.tan(beta);
  const tanTheta = Math.tan(theta);
  const yTop = tanBeta * (Lx - x0) * 1.45 + 0.08;
  const yWall = (x: number) => (x <= x0 ? 0 : (x - x0) * Math.tan(theta));

  /** One full march on a grid. shockS = per-node-column shock position in s (null: theory line). */
  const march = (shockS: Float64Array | null, iters: number) => {
    const nodesX = new Float64Array((NI + 1) * (NJ + 1));
    const nodesY = new Float64Array((NI + 1) * (NJ + 1));
    const nid = (i: number, j: number) => j * (NI + 1) + i;
    const colS: Float64Array[] = [];
    for (let i = 0; i <= NI; i++) {
      if (!clustered) break;
      const x = (i / NI) * Lx;
      let sc: number;
      if (shockS) sc = shockS[i];
      else {
        const run = Math.max(x, x0 + 0.02) - x0;
        sc = (run * (tanBeta - tanTheta) * wedgeShockBias.value) / Math.max(yTop - yWall(Math.max(x, x0 + 0.02)), 1e-9);
      }
      sc = Math.min(Math.max(sc, 0.04), 0.92);
      colS.push(clusterCoordinates(NJ, { wall: wallAmp, shock: shockAmp, shockPos: sc, shockWidth: shockW }));
    }
    for (let j = 0; j <= NJ; j++) {
      for (let i = 0; i <= NI; i++) {
        const x = (i / NI) * Lx;
        const yw = yWall(x);
        const y = clustered ? yw + colS[i][j] * (yTop - yw) : yw + (j / NJ) * (yTop - yw);
        const id = nid(i, j);
        nodesX[id] = x;
        nodesY[id] = y;
      }
    }

    const nc = NI * NJ;
    const vol = new Float64Array(nc);
    const fW = new Float64Array(nc * 2);
    const fE = new Float64Array(nc * 2);
    const fS = new Float64Array(nc * 2);
    const fN = new Float64Array(nc * 2);
    const cid = (i: number, j: number) => j * NI + i;
    for (let j = 0; j < NJ; j++) {
      for (let i = 0; i < NI; i++) {
        const sw = nid(i, j);
        const se = nid(i + 1, j);
        const ne = nid(i + 1, j + 1);
        const nw = nid(i, j + 1);
        const px = [nodesX[sw], nodesX[se], nodesX[ne], nodesX[nw]];
        const py = [nodesY[sw], nodesY[se], nodesY[ne], nodesY[nw]];
        let a = 0;
        for (let k = 0; k < 4; k++) {
          const n = (k + 1) % 4;
          a += px[k] * py[n] - px[n] * py[k];
        }
        const id = cid(i, j);
        vol[id] = Math.abs(a) * 0.5;
        const put = (arr: Float64Array, ax: number, ay: number) => {
          arr[id * 2] = ax;
          arr[id * 2 + 1] = ay;
        };
        const ex = nodesX[nw] - nodesX[sw];
        const ey = nodesY[nw] - nodesY[sw];
        put(fW, -ey, ex);
        const eex = nodesX[ne] - nodesX[se];
        const eey = nodesY[ne] - nodesY[se];
        put(fE, eey, -eex);
        const sx = nodesX[se] - nodesX[sw];
        const sy = nodesY[se] - nodesY[sw];
        put(fS, sy, -sx);
        const nx = nodesX[ne] - nodesX[nw];
        const ny = nodesY[ne] - nodesY[nw];
        put(fN, -ny, nx);
      }
    }

    const rhoInf = 1;
    const pInf = 1;
    const aInf = Math.sqrt(gamma * pInf / rhoInf);
    const uInf = M * aInf;
    const U = new Float64Array(nc * 4);
    for (let c = 0; c < nc; c++) {
      U[c * 4] = rhoInf;
      U[c * 4 + 1] = rhoInf * uInf;
      U[c * 4 + 2] = 0;
      U[c * 4 + 3] = pInf / (gamma - 1) + 0.5 * rhoInf * uInf * uInf;
    }

    const gm1 = gamma - 1;
    const load = (o: number) => {
      const r = U[o] > 1e-8 ? U[o] : 1e-8;
      const u = U[o + 1] / r;
      const v = U[o + 2] / r;
      const p = gm1 * (U[o + 3] - 0.5 * r * (u * u + v * v));
      S4[0] = r;
      S4[1] = u;
      S4[2] = v;
      S4[3] = p > 1e-8 ? p : 1e-8;
    };
    let residual = 1;
    let peak = 0;
    let below = 0;
    let used = iters;
    for (let iter = 0; iter < iters; iter++) {
      let resSum = 0;
      for (let j = 0; j < NJ; j++) {
        for (let i = 0; i < NI; i++) {
          const id = cid(i, j);
          const o = id * 4;
          const r = U[o] > 1e-8 ? U[o] : 1e-8;
          const u = U[o + 1] / r;
          const v = U[o + 2] / r;
          const p = gm1 * (U[o + 3] - 0.5 * r * (u * u + v * v));
          const pp = p > 1e-8 ? p : 1e-8;
          let d0 = 0;
          let d1 = 0;
          let d2 = 0;
          let d3 = 0;
          const take = () => {
            d0 += F4[0];
            d1 += F4[1];
            d2 += F4[2];
            d3 += F4[3];
          };
          if (i === 0) {
            hllc(r, u, v, pp, rhoInf, uInf, 0, pInf, fW[id * 2], fW[id * 2 + 1], gamma);
            take();
          } else {
            load(cid(i - 1, j) * 4);
            hllc(r, u, v, pp, S4[0], S4[1], S4[2], S4[3], fW[id * 2], fW[id * 2 + 1], gamma);
            take();
          }
          if (i === NI - 1) {
            hllc(r, u, v, pp, r, u, v, pp, fE[id * 2], fE[id * 2 + 1], gamma);
            take();
          } else {
            load(cid(i + 1, j) * 4);
            hllc(r, u, v, pp, S4[0], S4[1], S4[2], S4[3], fE[id * 2], fE[id * 2 + 1], gamma);
            take();
          }
          if (j === NJ - 1) {
            hllc(r, u, v, pp, rhoInf, uInf, 0, pInf, fN[id * 2], fN[id * 2 + 1], gamma);
            take();
          } else {
            load(cid(i, j + 1) * 4);
            hllc(r, u, v, pp, S4[0], S4[1], S4[2], S4[3], fN[id * 2], fN[id * 2 + 1], gamma);
            take();
          }
          if (j === 0) {
            d1 += pp * fS[id * 2];
            d2 += pp * fS[id * 2 + 1];
          } else {
            load(cid(i, j - 1) * 4);
            hllc(r, u, v, pp, S4[0], S4[1], S4[2], S4[3], fS[id * 2], fS[id * 2 + 1], gamma);
            take();
          }
          const spec = Math.hypot(u, v) + Math.sqrt((gamma * pp) / r);
          const per = Math.max(
            Math.hypot(fW[id * 2], fW[id * 2 + 1]),
            Math.hypot(fE[id * 2], fE[id * 2 + 1]),
            Math.hypot(fS[id * 2], fS[id * 2 + 1]),
            Math.hypot(fN[id * 2], fN[id * 2 + 1]),
            1e-8,
          );
          const dt = (CFL * (vol[id] / per)) / Math.max(spec, 1e-6);
          const scale = dt / Math.max(vol[id], 1e-12);
          const nr = U[o] - d0 * scale;
          const nu = U[o + 1] - d1 * scale;
          const nv = U[o + 2] - d2 * scale;
          const nE = U[o + 3] - d3 * scale;
          const rr = nr > 1e-8 ? nr : 1e-8;
          const uu = nu / rr;
          const vv = nv / rr;
          const pNew = gm1 * (nE - 0.5 * rr * (uu * uu + vv * vv));
          if (pNew > 1e-6 && nr > 1e-8) {
            U[o] = nr;
            U[o + 1] = nu;
            U[o + 2] = nv;
            U[o + 3] = nE;
          }
          resSum += (d0 / Math.max(vol[id], 1e-12)) ** 2;
        }
      }
      residual = Math.sqrt(resSum / nc);
      // [mesh-refinement] opt-in convergence stop (volume.tol > 0): residual <= tol x its running peak for 3 sweeps
      if (tolW > 0) {
        if (residual > peak) peak = residual;
        if (iter + 1 >= 30 && peak > 0 && residual <= tolW * peak) {
          if (++below >= 3) {
            used = iter + 1;
            break;
          }
        } else below = 0;
      }
    }

    return { U, residual, nodesY, colS, load, cid, used };
  };

  let run: ReturnType<typeof march>;
  let passes = 1;
  if (!adaptive) run = march(null, ITERS);
  else {
    // Pass 1 on the static grid (half the iterations) located the jump; re-cluster and re-solve.
    const half = march(null, Math.max(20, Math.round(ITERS * 0.5)));
    const rho = new Float64Array(NI * NJ);
    for (let c = 0; c < NI * NJ; c++) rho[c] = half.U[c * 4];
    const sOfRow = (i: number, j: number) =>
      0.25 * (half.colS[i][j] + half.colS[i][j + 1] + half.colS[i + 1][j] + half.colS[i + 1][j + 1]);
    const rawCell = detectShockFractions(rho, NI, NJ, sOfRow, 1, 0.35);
    // Static fallback per cell column, then smooth and expand to node columns.
    const fb = new Float64Array(NI);
    for (let i = 0; i < NI; i++) {
      const x = ((i + 0.5) / NI) * Lx;
      const xr = Math.max(x, x0 + 0.02);
      fb[i] = Math.min(Math.max(((xr - x0) * (tanBeta - tanTheta) * wedgeShockBias.value) / Math.max(yTop - yWall(xr), 1e-9), 0.04), 0.92);
    }
    const sm = smoothFractions(rawCell, fb, 8);
    const nodeS = new Float64Array(NI + 1);
    for (let i = 0; i <= NI; i++) nodeS[i] = i === 0 ? sm[0] : i === NI ? sm[NI - 1] : 0.5 * (sm[i - 1] + sm[i]);
    run = march(nodeS, ITERS);
    passes = 2;
  }
  const { U, residual, nodesY, load, cid, used } = run;

  let pSum = 0;
  let nSum = 0;
  for (let i = 0; i < NI; i++) {
    const x = ((i + 0.5) / NI) * Lx;
    if (x < x0 + 0.18 || x > Lx * 0.82) continue;
    load(cid(i, 0) * 4);
    pSum += S4[3];
    nSum++;
  }
  const p2 = nSum > 0 ? pSum / nSum : 1;
  const relErr = Math.abs(p2 - exactP) / Math.max(exactP, 1e-9);
  const angles = shockAngle();
  return {
    mach: M,
    thetaDeg,
    p2p1: p2,
    exactP,
    relErr,
    cp: (p2 - 1) / q,
    exactCp,
    residual,
    iters: tolW > 0 ? used : ITERS,
    ...angles,
    ni: NI,
    nj: NJ,
    passes,
    ms: performance.now() - t0,
  };

  /** Shock angle from the density field: half-jump crossing of exact post-shock density, line fit mid-wedge. */
  function shockAngle() {
    const rhoHalf = 1 + 0.5 * (exact!.r2r1 - 1);
    const xs: number[] = [];
    const ys: number[] = [];
    for (let i = 0; i < NI; i++) {
      const x = ((i + 0.5) / NI) * Lx;
      if (x < x0 + 0.25 * (Lx - x0) || x > x0 + 0.85 * (Lx - x0)) continue;
      for (let j = NJ - 2; j >= 0; j--) {
        const a = U[cid(i, j) * 4];
        const b = U[cid(i, j + 1) * 4];
        if (a >= rhoHalf && b < rhoHalf) {
          const t = (a - rhoHalf) / Math.max(a - b, 1e-12);
          const yc = (jj: number) => 0.25 * (nodesY[jj * (NI + 1) + i] + nodesY[jj * (NI + 1) + i + 1] + nodesY[(jj + 1) * (NI + 1) + i] + nodesY[(jj + 1) * (NI + 1) + i + 1]);
          xs.push(x);
          ys.push(yc(j) + (yc(j + 1) - yc(j)) * t);
          break;
        }
      }
    }
    if (xs.length < 4) return { shockAngleDeg: NaN, exactShockAngleDeg: beta / DEG };
    const n = xs.length;
    const mx = xs.reduce((s2, v) => s2 + v, 0) / n;
    const my = ys.reduce((s2, v) => s2 + v, 0) / n;
    let sxy = 0;
    let sxx = 0;
    for (let k = 0; k < n; k++) {
      sxy += (xs[k] - mx) * (ys[k] - my);
      sxx += (xs[k] - mx) ** 2;
    }
    return { shockAngleDeg: Math.atan(sxy / Math.max(sxx, 1e-30)) / DEG, exactShockAngleDeg: beta / DEG };
  }
}
