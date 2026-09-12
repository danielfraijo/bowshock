import type { Vec3 } from "./types";

export const DEG = Math.PI / 180;
export const RAD = 180 / Math.PI;

export function clamp(v: number, a: number, b: number) {
  return Math.max(a, Math.min(b, v));
}

export function lerp(a: number, b: number, t: number) {
  return a + (b - a) * t;
}

export function vadd(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function vsub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function vscale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}

export function vdot(a: Vec3, b: Vec3) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function vcross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

export function vlen(a: Vec3) {
  return Math.hypot(a[0], a[1], a[2]);
}

export function vnorm(a: Vec3): Vec3 {
  const n = vlen(a);
  return n < 1e-15 ? [0, 0, 0] : [a[0] / n, a[1] / n, a[2] / n];
}

export function vlerp(a: Vec3, b: Vec3, t: number): Vec3 {
  return [lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t)];
}

export function cluster(i: number, n: number, power: number) {
  if (n <= 1) return 0;
  const u = i / (n - 1);
  if (power <= 1.001) return u;
  return 1 - (1 - u) ** power;
}

export function cosineSpace(i: number, n: number) {
  if (n <= 1) return 0;
  return 0.5 * (1 - Math.cos((Math.PI * i) / (n - 1)));
}

/** θ-β-M relation. Returns flow deflection θ (rad) for shock angle β. */
export function thetaFromBetaM(M: number, beta: number, gamma = 1.4): number {
  const s = Math.sin(beta);
  const c = Math.cos(beta);
  if (Math.abs(s) < 1e-12) return 0;
  const M2 = M * M;
  const num = 2 * (c / s) * (M2 * s * s - 1);
  const den = M2 * (gamma + Math.cos(2 * beta)) + 2;
  return Math.atan(num / den);
}

export function maxTheta(M: number, gamma = 1.4): { theta: number; beta: number } {
  const mu = Math.asin(clamp(1 / M, 0, 1));
  let bestT = 0;
  let bestB = mu;
  const n = 80;
  for (let i = 1; i < n; i++) {
    const b = mu + ((Math.PI / 2 - 1e-4 - mu) * i) / n;
    const t = thetaFromBetaM(M, b, gamma);
    if (t > bestT) {
      bestT = t;
      bestB = b;
    }
  }
  return { theta: bestT, beta: bestB };
}

/** Weak-shock β for given deflection. Returns NaN if detached. */
export function betaFromThetaM(M: number, theta: number, gamma = 1.4): number {
  if (theta <= 1e-10) return Math.asin(clamp(1 / M, 0, 1));
  const cap = maxTheta(M, gamma);
  if (theta >= cap.theta * 0.999) return NaN;
  const mu = Math.asin(clamp(1 / M, 0, 1));
  let lo = mu + 1e-6;
  let hi = cap.beta;
  for (let i = 0; i < 48; i++) {
    const mid = 0.5 * (lo + hi);
    const t = thetaFromBetaM(M, mid, gamma);
    if (t < theta) lo = mid;
    else hi = mid;
  }
  return 0.5 * (lo + hi);
}

export function obliqueShock(M: number, beta: number, gamma = 1.4) {
  const Mn = M * Math.sin(beta);
  const Mn2 = Mn * Mn;
  const gp1 = gamma + 1;
  const gm1 = gamma - 1;
  const p2p1 = 1 + (2 * gamma) / gp1 * (Mn2 - 1);
  const r2r1 = (gp1 * Mn2) / (gm1 * Mn2 + 2);
  const t2t1 = p2p1 / r2r1;
  const Mn2sq = (Mn2 + 2 / gm1) / ((2 * gamma) / gm1 * Mn2 - 1);
  const theta = thetaFromBetaM(M, beta, gamma);
  const M2 = Math.sqrt(Math.max(0, Mn2sq)) / Math.sin(Math.max(1e-6, beta - theta));
  return { theta, p2p1, r2r1, t2t1, M2, Mn };
}

type TmTable = { theta: number[]; vr: number[]; vt: number[]; cone: number };

function tmDeriv(vr: number, vt: number, th: number, gamma: number) {
  const gm1h = (gamma - 1) / 2;
  const v2 = vr * vr + vt * vt;
  const a = gm1h * (1 - v2);
  const cot = Math.cos(th) / Math.max(1e-10, Math.sin(th));
  const num = vt * vt * vr - a * (2 * vr + vt * cot);
  const den = a - vt * vt;
  return den === 0 ? 0 : num / den;
}

/**
 * Integrate Taylor–Maccoll from the shock inward.
 * Velocities are nondimensional by Vmax.
 */
export function taylorMaccoll(M: number, beta: number, gamma = 1.4): TmTable | null {
  if (!(M > 1) || !(beta > 0)) return null;
  const gm1 = gamma - 1;
  const vInf = Math.sqrt((gm1 * 0.5 * M * M) / (1 + gm1 * 0.5 * M * M));
  const Mn = M * Math.sin(beta);
  const rho = ((gamma + 1) * Mn * Mn) / (gm1 * Mn * Mn + 2);
  let vr = vInf * Math.cos(beta);
  let vt = (-vInf * Math.sin(beta)) / rho;
  let th = beta;
  const theta = [th];
  const vrA = [vr];
  const vtA = [vt];
  const dth = -Math.min(0.25 * DEG, beta / 80);
  let cone = 0;
  for (let k = 0; k < 4000; k++) {
    if (th + dth <= 1e-3) break;
    const k1v = vt;
    const k1t = tmDeriv(vr, vt, th, gamma);
    const k2v = vt + 0.5 * dth * k1t;
    const k2t = tmDeriv(vr + 0.5 * dth * k1v, vt + 0.5 * dth * k1t, th + 0.5 * dth, gamma);
    const k3v = vt + 0.5 * dth * k2t;
    const k3t = tmDeriv(vr + 0.5 * dth * k2v, vt + 0.5 * dth * k2t, th + 0.5 * dth, gamma);
    const k4v = vt + dth * k3t;
    const k4t = tmDeriv(vr + dth * k3v, vt + dth * k3t, th + dth, gamma);
    vr += (dth / 6) * (k1v + 2 * k2v + 2 * k3v + k4v);
    vt += (dth / 6) * (k1t + 2 * k2t + 2 * k3t + k4t);
    th += dth;
    theta.push(th);
    vrA.push(vr);
    vtA.push(vt);
    if (vt >= 0) {
      cone = th - dth * (vtA[vtA.length - 2] / (vtA[vtA.length - 2] - vt + 1e-16));
      break;
    }
  }
  if (cone <= 0 || cone >= beta) return null;
  return { theta, vr: vrA, vt: vtA, cone };
}

export function solveConeShock(M: number, cone: number, gamma = 1.4): number {
  const mu = Math.asin(clamp(1 / M, 0, 1));
  let lo = Math.max(mu + 0.15 * DEG, cone + 0.2 * DEG);
  let hi = 70 * DEG;
  let best = lo;
  for (let i = 0; i < 28; i++) {
    const mid = 0.5 * (lo + hi);
    const tm = taylorMaccoll(M, mid, gamma);
    if (!tm) {
      hi = mid;
      continue;
    }
    best = mid;
    if (tm.cone < cone) lo = mid;
    else hi = mid;
  }
  return best;
}

export function interpTable(table: TmTable, th: number): { vr: number; vt: number } {
  const t = table.theta;
  if (th >= t[0]) return { vr: table.vr[0], vt: table.vt[0] };
  if (th <= t[t.length - 1]) {
    const i = t.length - 1;
    return { vr: table.vr[i], vt: table.vt[i] };
  }
  let lo = 0;
  let hi = t.length - 1;
  while (hi - lo > 1) {
    const m = (lo + hi) >> 1;
    if (t[m] > th) lo = m;
    else hi = m;
  }
  const u = (th - t[lo]) / (t[hi] - t[lo] + 1e-16);
  return {
    vr: lerp(table.vr[lo], table.vr[hi], u),
    vt: lerp(table.vt[lo], table.vt[hi], u),
  };
}

export function unitScale(unit: "m" | "mm" | "in") {
  if (unit === "mm") return 1000;
  if (unit === "in") return 1 / 0.0254;
  return 1;
}

export function fmt(n: number, digits = 4) {
  if (!Number.isFinite(n)) return "—";
  const a = Math.abs(n);
  if (a !== 0 && (a < 1e-3 || a >= 1e5)) return n.toExponential(3);
  return n.toFixed(digits);
}

export function prandtlMeyer(M: number, gamma = 1.4): number {
  if (M <= 1) return 0;
  const g = gamma;
  const k = Math.sqrt((g + 1) / (g - 1));
  return k * Math.atan(Math.sqrt(((g - 1) / (g + 1)) * (M * M - 1))) - Math.atan(Math.sqrt(M * M - 1));
}

export function invPrandtlMeyer(nu: number, gamma = 1.4): number {
  if (nu <= 0) return 1;
  let M = 1.5;
  for (let i = 0; i < 24; i++) {
    const f = prandtlMeyer(M, gamma) - nu;
    const dM = 1e-4;
    const fp = (prandtlMeyer(M + dM, gamma) - prandtlMeyer(M, gamma)) / dM;
    M = Math.max(1.0001, M - f / (fp || 1));
  }
  return M;
}

export function isentropic(M: number, gamma = 1.4) {
  const gm1 = gamma - 1;
  const t = 1 + 0.5 * gm1 * M * M;
  const p = t ** (gamma / gm1);
  const r = t ** (1 / gm1);
  return { TtT: t, ptP: p, rtR: r, T: 1 / t, p: 1 / p, r: 1 / r };
}

/** Rayleigh pitot (normal-shock + isentropic) p0,2 / p1 */
export function rayleighPitot(M: number, gamma = 1.4): number {
  const g = gamma;
  const M2 = M * M;
  const a = ((g + 1) * (g + 1) * M2) / (4 * g * M2 - 2 * (g - 1));
  const b = (1 - g + 2 * g * M2) / (g + 1);
  return a ** (g / (g - 1)) * b;
}

export function newtonianCpMax(M: number, gamma = 1.4): number {
  const q = 0.5 * gamma * M * M;
  return (rayleighPitot(M, gamma) - 1) / q;
}

export function vacuumCp(M: number, gamma = 1.4): number {
  return -2 / (gamma * M * M);
}

export function sutherlandMu(T: number): number {
  const T0 = 273.15;
  const mu0 = 1.716e-5;
  const S = 110.4;
  return (mu0 * (T / T0) ** 1.5 * (T0 + S)) / (T + S);
}

export function skinCf(Re: number): number {
  if (!(Re > 100)) return 0.01;
  const log = Math.log10(Math.max(Re, 1e3));
  return 0.455 / (log * log);
}

/**
 * van Driest II compressible turbulent flat-plate Cf (Hopkins & Inouye 1971;
 * White 2006). Incompressible Cf from Prandtl–Schlichting, then
 * Cf = Cfi / Fc with Fθ = μe/μw.
 */
export function vanDriestII(Re: number, M: number, Te: number, Tw: number, gamma = 1.4): number {
  const Cfi = skinCf(Re);
  if (!(M > 0.35) || !(Te > 0) || !(Tw > 0)) return Cfi;
  const r = 0.89;
  const TawTe = 1 + r * 0.5 * (gamma - 1) * M * M;
  const TwTe = clamp(Tw / Te, 0.2, 8);
  const a2 = (r * 0.5 * (gamma - 1) * M * M) / TwTe;
  const b = TawTe / TwTe - 1;
  const den = Math.sqrt(Math.max(1e-16, 4 * a2 + b * b));
  const A = clamp((2 * a2 - b) / den, -1, 1);
  const B = clamp(b / den, -1, 1);
  const span = Math.asin(A) + Math.asin(B);
  const Fc = Math.abs(span) < 1e-8 ? TwTe : (TawTe - 1) / (span * span);
  const muW = sutherlandMu(Tw);
  const muE = sutherlandMu(Te);
  const Rei = Math.max(1e3, Re * (muE / Math.max(muW, 1e-12)));
  const CfiStar = skinCf(Rei);
  return CfiStar / Math.max(Fc, 0.35);
}

/** Sutton–Graves / Tauber TP-2914 Earth stagnation, W/cm². ρ kg/m³, Rn m, V m/s. */
export function suttonGraves(rho: number, V: number, Rn: number, recov = 1): number {
  return 1.83e-8 * Math.sqrt(rho / Math.max(Rn, 1e-8)) * V ** 3 * recov;
}

/**
 * Detra–Kemp–Riddell stagnation, W/cm² (Earth).
 * q = 5.21×10⁴ √(ρ/Rn) (V/10⁴)^3.15  W/m² → /10⁴ for W/cm².
 */
export function detraKempRiddell(rho: number, V: number, Rn: number, recov = 1): number {
  return 5.21 * Math.sqrt(rho / Math.max(Rn, 1e-8)) * Math.pow(V / 1e4, 3.15) * recov;
}

/** Radiation-equilibrium wall temperature from a convective flux in W/cm². ε=0.8. */
export function twEquilibrium(qWcm2: number, eps = 0.8): number {
  const q = Math.max(0, qWcm2) * 1e4;
  const sig = 5.670374419e-8;
  return Math.pow(q / Math.max(eps * sig, 1e-12), 0.25);
}

/** Tauber laminar running-length heating, W/cm² (NASA TP-2914). */
export function tauberLaminar(rho: number, V: number, x: number, recov: number, sinth: number): number {
  return 1.83e-8 * Math.sqrt(rho / Math.max(x, 1e-8)) * V ** 3 * recov * Math.pow(Math.max(sinth, 0), 1.15);
}

/** Tauber turbulent running-length heating, W/cm² (TP-2914 / CBAERO fit). */
export function tauberTurbulent(rho: number, V: number, x: number, recov: number, sinth: number): number {
  return 3.7e-9 * rho ** 0.8 * V ** 3.37 * Math.max(x, 1e-8) ** -0.2 * recov * Math.pow(Math.max(sinth, 0), 1.6);
}

const _coneBeta = new Map<string, number>();
const _coneCp = new Map<string, number>();

function coneKey(M: number, cone: number, gamma: number) {
  return `${M.toFixed(2)}:${(cone * RAD).toFixed(1)}:${gamma}`;
}

/** Cached Taylor–Maccoll shock angle for a cone. */
export function solveConeShockCached(M: number, cone: number, gamma = 1.4): number {
  const k = coneKey(M, cone, gamma);
  const hit = _coneBeta.get(k);
  if (hit !== undefined) return hit;
  const b = solveConeShock(M, cone, gamma);
  _coneBeta.set(k, b);
  return b;
}

/**
 * Inviscid cone surface Cp from Taylor–Maccoll (Sims NASA SP-3004 method).
 * Used as the tangent-cone pressure on axisymmetric generating flows.
 */
export function coneSurfaceCp(M: number, cone: number, gamma = 1.4): number {
  if (cone <= 1e-5) return 0;
  const k = coneKey(M, cone, gamma);
  const hit = _coneCp.get(k);
  if (hit !== undefined) return hit;
  const beta = solveConeShockCached(M, cone, gamma);
  const tm = taylorMaccoll(M, beta, gamma);
  const q = 0.5 * gamma * M * M;
  if (!tm) {
    const cp = newtonianCpMax(M, gamma) * Math.sin(cone) ** 2;
    _coneCp.set(k, cp);
    return cp;
  }
  const sh = obliqueShock(M, beta, gamma);
  const v2 = tm.vr[0] ** 2 + tm.vt[0] ** 2;
  const T2T0 = Math.max(1e-6, 1 - v2);
  const p2 = sh.p2p1;
  const p02 = p2 * T2T0 ** (-gamma / (gamma - 1));
  const { vr } = interpTable(tm, tm.cone);
  const TcT0 = Math.max(1e-6, 1 - vr * vr);
  const pc = p02 * TcT0 ** (gamma / (gamma - 1));
  const cp = (pc - 1) / q;
  _coneCp.set(k, cp);
  return cp;
}


/** Rankine–Hugoniot normal shock. */
export function normalShock(M: number, gamma = 1.4) {
  const M2 = Math.max(M * M, 1);
  const gp1 = gamma + 1;
  const gm1 = gamma - 1;
  const p2p1 = 1 + ((2 * gamma) / gp1) * (M2 - 1);
  const r2r1 = (gp1 * M2) / (gm1 * M2 + 2);
  const t2t1 = p2p1 / r2r1;
  const M2n = Math.sqrt((M2 + 2 / gm1) / ((2 * gamma) / gm1 * M2 - 1));
  const iso1 = isentropic(M, gamma);
  const iso2 = isentropic(M2n, gamma);
  const ptRatio = (p2p1 * iso2.ptP) / iso1.ptP;
  return { p2p1, r2r1, t2t1, M2: M2n, pt2pt1: ptRatio };
}

/** A/A* for isentropic flow. */
export function areaRatio(M: number, gamma = 1.4): number {
  const gm1 = gamma - 1;
  const t = 1 + 0.5 * gm1 * M * M;
  return (1 / Math.max(M, 1e-8)) * (t / (0.5 * (gamma + 1))) ** ((gamma + 1) / (2 * gm1));
}

/** Invert A/A* → M (Newton). */
export function machFromArea(AR: number, gamma = 1.4, supersonic = true): number {
  let M = supersonic ? Math.max(1.2, Math.min(8, AR)) : 0.3;
  for (let i = 0; i < 28; i++) {
    const f = areaRatio(M, gamma) - AR;
    const fp = (areaRatio(M + 1e-4, gamma) - areaRatio(M, gamma)) / 1e-4;
    M -= f / (fp || 1);
    M = supersonic ? clamp(M, 1.0001, 12) : clamp(M, 0.05, 0.999);
  }
  return M;
}

/** Rayleigh Tt/Tt* (choking heat). */
export function rayleighTtStar(M: number, gamma = 1.4): number {
  const M2 = M * M;
  const num = 2 * (1 + gamma) * M2 * (1 + 0.5 * (gamma - 1) * M2);
  const den = (1 + gamma * M2) ** 2;
  return num / den;
}

/** Rayleigh p/p*. */
export function rayleighPStar(M: number, gamma = 1.4): number {
  return (1 + gamma) / (1 + gamma * M * M);
}

/**
 * Solve Rayleigh M2 given heat. Subsonic heat → M increases toward 1;
 * supersonic heat → M decreases toward 1.
 */
export function rayleighM2(M1: number, Tt2Tt1: number, gamma = 1.4): number {
  const r1 = rayleighTtStar(M1, gamma);
  const target = clamp(r1 * Tt2Tt1, 1e-4, 0.999);
  const sub = M1 < 1;
  let lo = sub ? M1 : 1.001;
  let hi = sub ? 0.999 : M1;
  if (!sub && hi < 1.05) hi = 1.05;
  for (let i = 0; i < 40; i++) {
    const mid = 0.5 * (lo + hi);
    const r = rayleighTtStar(mid, gamma);
    if (sub) {
      if (r < target) lo = mid;
      else hi = mid;
    } else {
      if (r < target) hi = mid;
      else lo = mid;
    }
  }
  return 0.5 * (lo + hi);
}

/** Kantrowitz self-start A_throat / A_capture (minimum). */
export function kantrowitz(M: number, gamma = 1.4): number {
  if (M <= 1) return 1;
  const ns = normalShock(M, gamma);
  return areaRatio(ns.M2, gamma) / areaRatio(M, gamma);
}

export function prandtlMeyerDeg(M: number, gamma = 1.4) {
  return prandtlMeyer(M, gamma) * RAD;
}


/** Tauber–Sutton 1991 Earth radiative heating, W/cm². Rn m, V m/s, rho SI. */
export function tauberSutton(rho: number, V: number, Rn: number): number {
  if (!(V > 2500) || !(rho > 0)) return 0;
  const RnE = Math.max(Rn, 0.01);
  return 4.736e8 * RnE ** 1.072 * rho ** 1.22 * (V / 10000) ** 8.5;
}
