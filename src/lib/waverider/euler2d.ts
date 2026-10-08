/**
 * Steady 2-D Euler on a body-fitted ramp. First-order HLLC, local time step.
 * This is the Cart3D-class anchor for a wedge: an Euler solver must recover
 * the exact oblique shock. The panel method is scored against that same shock.
 *
 * Grid and scheme match native/bowshock_kernel.c.
 */
import { DEG, betaFromThetaM, obliqueShock } from "./math";

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
}

const NI = 72;
const NJ = 24;
const ITERS = 280;
const CFL = 0.45;

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

export function eulerWedge(M: number, thetaDeg: number, gamma = 1.4): EulerAnchor {
  const cacheKey = `${M}|${thetaDeg}|${gamma}`;
  const cached = eulerCache.get(cacheKey);
  if (cached) return cached;
  const solved = solveEulerWedge(M, thetaDeg, gamma);
  eulerCache.set(cacheKey, solved);
  return solved;
}

function solveEulerWedge(M: number, thetaDeg: number, gamma = 1.4): EulerAnchor {
  const theta = thetaDeg * DEG;
  const beta = betaFromThetaM(M, theta, gamma);
  const exact = Number.isFinite(beta) ? obliqueShock(M, beta, gamma) : null;
  const exactP = exact ? exact.p2p1 : 1;
  const q = 0.5 * gamma * M * M;
  const exactCp = (exactP - 1) / q;
  if (!exact) {
    return { mach: M, thetaDeg, p2p1: 1, exactP, relErr: 1, cp: 0, exactCp, residual: 1, iters: 0 };
  }

  const Lx = 1;
  const x0 = 0.2;
  const yTop = Math.tan(beta) * (Lx - x0) * 1.45 + 0.08;
  const yWall = (x: number) => (x <= x0 ? 0 : (x - x0) * Math.tan(theta));
  const nodesX = new Float64Array((NI + 1) * (NJ + 1));
  const nodesY = new Float64Array((NI + 1) * (NJ + 1));
  const nid = (i: number, j: number) => j * (NI + 1) + i;
  for (let j = 0; j <= NJ; j++) {
    for (let i = 0; i <= NI; i++) {
      const x = (i / NI) * Lx;
      const yw = yWall(x);
      const y = yw + (j / NJ) * (yTop - yw);
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
  for (let iter = 0; iter < ITERS; iter++) {
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
  }

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
  return {
    mach: M,
    thetaDeg,
    p2p1: p2,
    exactP,
    relErr,
    cp: (p2 - 1) / q,
    exactCp,
    residual,
    iters: ITERS,
  };
}
