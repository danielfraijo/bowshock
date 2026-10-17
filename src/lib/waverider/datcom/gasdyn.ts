/**
 * Perfect-gas relations used by the DATCOM-style estimator.
 * Standard compressible-flow results (Anderson, Modern Compressible Flow;
 * NACA Report 1135, Ames Research Staff 1953). Self-contained so the module
 * runs under plain `node --experimental-strip-types`.
 */
import { clamp } from "./util.ts";
import { DEG } from "./util.ts";

export const GAMMA_AIR = 1.4;

/** Prandtl–Meyer function nu(M) [rad]. NACA 1135 Eq. 175. */
export function prandtlMeyer(M: number, g = GAMMA_AIR): number {
  if (M <= 1) return 0;
  const a = Math.sqrt((g + 1) / (g - 1));
  const b = Math.sqrt(M * M - 1);
  return a * Math.atan(b / a) - Math.atan(b);
}
/** Inverse Prandtl–Meyer (Newton iteration on M). */
export function invPrandtlMeyer(nu: number, g = GAMMA_AIR): number {
  const numax = (Math.sqrt((g + 1) / (g - 1)) - 1) * (Math.PI / 2);
  if (nu >= numax) return Infinity;
  let M = 1 + nu * 1.5 + 0.5 * nu * nu;
  for (let i = 0; i < 60; i++) {
    const f = prandtlMeyer(M, g) - nu;
    const dnu = Math.sqrt(M * M - 1) / (M * (1 + ((g - 1) / 2) * M * M));
    const dM = f / Math.max(dnu, 1e-12);
    M = Math.max(1.0000001, M - dM);
    if (Math.abs(dM) < 1e-12) break;
  }
  return M;
}
/** p/p0 at Mach M (isentropic). */
export function pRatio0(M: number, g = GAMMA_AIR): number {
  return Math.pow(1 + ((g - 1) / 2) * M * M, -g / (g - 1));
}
/** Pressure coefficient of an isentropic expansion from M1 to M2. */
export function cpIsentropic(M1: number, M2: number, g = GAMMA_AIR): number {
  const pr = pRatio0(M2, g) / pRatio0(M1, g);
  return (2 / (g * M1 * M1)) * (pr - 1);
}
/** Vacuum Cp = -2/(gamma M^2). */
export function cpVacuum(M: number, g = GAMMA_AIR): number {
  return -2 / (g * M * M);
}
/** Rayleigh pitot formula p02/p1 (NACA 1135 Eq. 99); isentropic stagnation for M<=1. */
export function pitotRatio(M: number, g = GAMMA_AIR): number {
  if (M <= 1) return Math.pow(1 + ((g - 1) / 2) * M * M, g / (g - 1));
  const t1 = Math.pow(((g + 1) * M * M) / 2, g / (g - 1));
  const t2 = Math.pow((2 * g * M * M - (g - 1)) / (g + 1), 1 / (g - 1));
  return t1 / t2;
}
/** Newtonian / modified-Newtonian Cp,max = (p02 - p1) / q (Lees 1955). */
export function cpMaxNewton(M: number, g = GAMMA_AIR): number {
  const p02p1 = pitotRatio(M, g);
  return (2 / (g * M * M)) * (p02p1 - 1);
}
/** theta-beta-M: flow deflection for shock angle beta [rad]. */
export function thetaFromBeta(M: number, beta: number, g = GAMMA_AIR): number {
  const sb = Math.sin(beta);
  const num = 2 / Math.tan(beta) * (M * M * sb * sb - 1);
  const den = M * M * (g + Math.cos(2 * beta)) + 2;
  return Math.atan(num / den);
}
/** Maximum deflection angle for attached oblique shock [rad] (coarse scan + refine). */
export function thetaMax(M: number, g = GAMMA_AIR): number {
  if (M <= 1) return 0;
  const bmin = Math.asin(1 / M);
  let best = 0;
  let bb = bmin;
  const n = 200;
  for (let i = 1; i < n; i++) {
    const b = bmin + ((Math.PI / 2 - bmin) * i) / n;
    const t = thetaFromBeta(M, b, g);
    if (t > best) {
      best = t;
      bb = b;
    }
  }
  const lo = bb - (Math.PI / 2 - bmin) / n;
  const hi = bb + (Math.PI / 2 - bmin) / n;
  for (let i = 0; i <= 50; i++) {
    const b = lo + ((hi - lo) * i) / 50;
    const t = thetaFromBeta(M, clamp(b, bmin, Math.PI / 2), g);
    if (t > best) best = t;
  }
  return best;
}
/** Weak-shock angle beta [rad] for a deflection theta [rad]; null if detached. */
export function weakBeta(M: number, theta: number, g = GAMMA_AIR): number | null {
  if (M <= 1 || theta <= 0) return M > 1 ? Math.asin(1 / M) : null;
  if (theta > thetaMax(M, g)) return null;
  let lo = Math.asin(1 / M) + 1e-9;
  // find the beta at theta_max to bound the weak branch
  let bTop = lo;
  let tTop = 0;
  const n = 400;
  for (let i = 1; i <= n; i++) {
    const b = lo + ((Math.PI / 2 - lo) * i) / n;
    const t = thetaFromBeta(M, b, g);
    if (t >= tTop) {
      tTop = t;
      bTop = b;
    } else break;
  }
  let hi = bTop;
  if (thetaFromBeta(M, hi, g) < theta) return hi;
  for (let i = 0; i < 80; i++) {
    const mid = 0.5 * (lo + hi);
    if (thetaFromBeta(M, mid, g) < theta) lo = mid;
    else hi = mid;
  }
  return 0.5 * (lo + hi);
}
/** Static pressure ratio across an oblique shock of angle beta. NACA 1135 Eq. 142. */
export function obliquePRatio(M: number, beta: number, g = GAMMA_AIR): number {
  const mn = M * Math.sin(beta);
  return 1 + (2 * g * (mn * mn - 1)) / (g + 1);
}
/** Mach behind an oblique shock (NACA 1135 Eq. 138 form). */
export function obliqueM2(M: number, beta: number, theta: number, g = GAMMA_AIR): number {
  const mn = M * Math.sin(beta);
  const mn2sq = (1 + ((g - 1) / 2) * mn * mn) / (g * mn * mn - (g - 1) / 2);
  return Math.sqrt(mn2sq) / Math.sin(beta - theta);
}
/**
 * Exact 2-D surface Cp for a surface inclined by theta [rad] to the stream
 * (theta>0 compression, theta<0 expansion): oblique shock / Prandtl-Meyer.
 * Falls back to modified Newtonian when the shock detaches.
 * Also returns whether the exact branch was used.
 */
export function cpSurface2D(M: number, theta: number, g = GAMMA_AIR): { cp: number; exact: boolean } {
  if (M <= 1) return { cp: 0, exact: false };
  if (Math.abs(theta) < 1e-9) return { cp: 0, exact: true };
  const q = 0.5 * g * M * M;
  if (theta > 0) {
    const beta = weakBeta(M, theta, g);
    if (beta === null) return { cp: cpMaxNewton(M, g) * Math.sin(theta) ** 2, exact: false };
    return { cp: (obliquePRatio(M, beta, g) - 1) / q, exact: true };
  }
  const nu1 = prandtlMeyer(M, g);
  const M2 = invPrandtlMeyer(nu1 - theta, g); // theta<0 => nu increases
  if (!Number.isFinite(M2)) return { cp: cpVacuum(M, g), exact: true };
  return { cp: cpIsentropic(M, M2, g), exact: true };
}
/** Exact flat-plate section normal-force coefficient cn(alpha, M) (shock-expansion, both surfaces). */
export function flatPlateCn(M: number, alpha: number, g = GAMMA_AIR): number {
  const a = Math.abs(alpha);
  const lower = cpSurface2D(M, a, g).cp;
  const upper = cpSurface2D(M, -a, g).cp;
  return Math.sign(alpha) * (lower - upper) * Math.cos(a);
}
/** Ackeret linear section lift-curve slope 4/beta (per rad). */
export function ackeretClAlpha(M: number): number {
  return 4 / Math.sqrt(Math.max(M * M - 1, 1e-6));
}
export const DEG_TO_RAD = DEG;
