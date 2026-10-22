/**
 * Closed-form flight science used beside the 3DOF glide and the Etkin modes.
 * Every function here is checked against a textbook limit in flight-science.test.ts
 * and again in the in-app validation list.
 *
 * Constants match trajectory.ts (spherical Earth, standard g).
 */

export const G0 = 9.80665;
export const RE_M = 6_371_000;

export function circularSpeed(re = RE_M, g0 = G0): number {
  return Math.sqrt(g0 * re);
}

/**
 * Sänger / Eggers equilibrium-glide range, metres.
 * S = ½ Re (L/D) ln[(1 − Vf²/Vc²) / (1 − V0²/Vc²)].
 * Lift balances weight minus centrifugal force; γ stays small.
 */
export function equilibriumGlideRange(ld: number, v0: number, vf: number, re = RE_M, g0 = G0): number {
  const vc2 = g0 * re;
  const num = 1 - (vf * vf) / vc2;
  const den = 1 - (v0 * v0) / vc2;
  if (!(ld > 0.2) || !(den > 1e-6) || !(num > 0)) return 0;
  return 0.5 * re * ld * Math.log(Math.max(num / den, 1.001));
}

/**
 * Flat-Earth unpowered glide, metres. S = (L/D) (V0² − Vf²) / (2g).
 * This is the V ≪ Vc limit of the equilibrium-glide integral.
 */
export function flatGlideRange(ld: number, v0: number, vf: number, g0 = G0): number {
  if (!(ld > 0) || !(v0 > vf)) return 0;
  return (ld * (v0 * v0 - vf * vf)) / (2 * g0);
}

/**
 * Kepler surface-to-surface range, metres. γ is from the local horizontal, radians.
 * Symmetric arc: both ends at r = Re, periapsis inside the Earth, range through apoapsis.
 * Suborbital and γ = 45° must recover the flat range V²/g.
 * Returns 0 for a dive (γ ≤ 0), a circular skim, or a trajectory that does not come back inside one hemisphere.
 */
export function vacuumRangeM(v: number, gamma: number, re = RE_M, g0 = G0): number {
  if (!(v > 0) || !(gamma > 1e-6) || gamma >= Math.PI / 2 - 1e-4) return 0;
  const nu = (v * v) / (g0 * re);
  if (!(nu > 0) || nu >= 1.98) return 0;
  const c = Math.cos(gamma);
  const e2 = 1 - nu * (2 - nu) * c * c;
  if (!(e2 > 1e-12)) return 0;
  const e = Math.sqrt(e2);
  const cosT = (nu * c * c - 1) / e;
  if (cosT <= -1 || cosT >= 1) return 0;
  const theta = Math.acos(cosT);
  const phi = 2 * (Math.PI - theta);
  if (!(phi > 0) || phi >= Math.PI) return 0;
  return phi * re;
}

/** Ballistic coefficient β = m / (CD S), kg/m². */
export function ballisticCoeff(mass: number, cd: number, sRef: number): number {
  if (!(mass > 0) || !(cd > 0) || !(sRef > 0)) return 0;
  return mass / (cd * sRef);
}

/** Sea-level density and the isothermal scale height used by the Allen–Eggers entry model. */
export const RHO_SL = 1.225;
export const SCALE_H = 7200;

/** Sutton–Graves Earth air, W/cm². Same constant as `suttonGraves` in math.ts. */
const SG_K = 1.7415e-8;

export interface AllenEggers {
  /** Peak deceleration, g. Independent of ballistic coefficient. */
  peakG: number;
  /** Altitude of that peak, km. Zero when the pulse would peak below the surface. */
  altKm: number;
  /** True when ρ* exceeds sea level: the vehicle hits the ground before the analytic g peak. */
  underground: boolean;
  /**
   * Speed at the analytic deceleration peak, m/s. Allen–Eggers: Ve / √e.
   * The vehicle never reaches this speed if `underground` is set.
   */
  vPeak: number;
  /** Altitude of the Sutton–Graves heating peak, km. H ln 3 above the g peak when both are in the air. */
  heatAltKm: number;
  /** Convective peak would be below the surface (one third of rho-star at or under sea level). */
  heatUnderground: boolean;
  /** Peak Sutton–Graves flux on this ballistic path, W/cm². Zero without a nose radius. */
  qPeak: number;
  /** Sutton–Graves load integrated to sea level, J/cm². Zero without a nose radius. */
  heatLoad: number;
}

/**
 * Hastings approximation, |error| < 1.5×10⁻⁷. The heat integral is an incomplete gamma of order 1/2,
 * which is √π erf(√(ρ/ρ*)).
 */
function erf(x: number): number {
  const sign = x < 0 ? -1 : 1;
  const ax = Math.abs(x);
  const t = 1 / (1 + 0.3275911 * ax);
  const y =
    1 -
    (((((1.061405429 * t - 1.453152027) * t + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t *
      Math.exp(-ax * ax));
  return sign * y;
}

/**
 * Allen & Eggers ballistic entry, NACA TN 4047, plus the Sutton–Graves pulse on that path.
 * Gravity is neglected and γ is held constant in an exponential atmosphere ρ = ρ0 e^(−h/H).
 * γRad is from the local horizontal, negative nose-down.
 *   n_max = V² sin|γ| / (2 e H g)          at  rho-star = β sin|γ| / H,   V-star = Ve / √e
 *   q̇ ∝ √ρ V³ peaks earlier, at rho = rho-star / 3,  V = Ve e^(−1/6),  Δh = H ln 3 above the g peak
 *   Q = k Ve² √(π β H / (Rn sin|γ|)) erf(√(ρ0/rho-star))     J/cm² down to sea level
 * Lift, skip, and a rotating Earth are outside this model. It is a steep-entry ceiling, not a glide load.
 */
export function allenEggers(
  v: number,
  gammaRad: number,
  beta: number,
  hScale = SCALE_H,
  rho0 = RHO_SL,
  g0 = G0,
  rn = 0.05,
): AllenEggers {
  const empty: AllenEggers = {
    peakG: 0,
    altKm: 0,
    underground: false,
    vPeak: 0,
    heatAltKm: 0,
    heatUnderground: false,
    qPeak: 0,
    heatLoad: 0,
  };
  const descent = -Math.sin(gammaRad);
  if (!(v > 0) || !(descent > 1e-8) || !(beta > 0) || !(hScale > 0) || !(rho0 > 0)) return empty;

  const peakG = (v * v * descent) / (2 * Math.E * hScale * g0);
  const rhoStar = (beta * descent) / hScale;
  const underground = !(rhoStar > 0) || rhoStar >= rho0;
  const altKm = underground ? 0 : (hScale * Math.log(rho0 / rhoStar)) / 1000;
  const vPeak = v / Math.sqrt(Math.E);

  const rhoHeat = rhoStar / 3;
  const heatUnderground = !(rhoHeat > 0) || rhoHeat >= rho0;
  const rhoAtQ = heatUnderground ? rho0 : rhoHeat;
  const heatAltKm = heatUnderground ? 0 : (hScale * Math.log(rho0 / rhoAtQ)) / 1000;
  const vAtQ = v * Math.exp(-rhoAtQ / (2 * rhoStar));

  let qPeak = 0;
  let heatLoad = 0;
  if (rn > 0 && rhoAtQ > 0 && Number.isFinite(vAtQ)) {
    qPeak = SG_K * Math.sqrt(rhoAtQ / rn) * vAtQ ** 3;
    const qInf = SG_K * v * v * Math.sqrt((Math.PI * beta * hScale) / (rn * descent));
    const frac = erf(Math.sqrt(Math.min(rho0 / rhoStar, 1e8)));
    heatLoad = qInf * Math.max(0, frac);
  }

  return { peakG, altKm, underground, vPeak, heatAltKm, heatUnderground, qPeak, heatLoad };
}

/** Specific energy as an equivalent height, metres. He = h + V²/(2g). */
export function energyHeight(h: number, v: number, g0 = G0): number {
  return h + (v * v) / (2 * g0);
}

/** Equilibrium-glide load factor L/W = 1 − V²/Vc². */
export function eqGlideLoad(v: number, re = RE_M, g0 = G0): number {
  return 1 - (v * v) / (g0 * re);
}

/**
 * Undamped short-period frequency, rad/s.
 * Etkin: ω² = −Mα when pitch damping is omitted.
 * Mα = q S c Cmα / Iyy, Cmα per radian. Returns 0 if the vehicle is not statically stable.
 */
export function shortPeriodUndamped(q: number, s: number, c: number, iy: number, cma: number): number {
  if (!(q > 0) || !(s > 0) || !(c > 0) || !(iy > 0) || !(cma < 0)) return 0;
  return Math.sqrt((-(q * s * c * cma)) / iy);
}

/**
 * Lanchester phugoid. ω = g √2 / V, ζ = 1 / (√2 L/D).
 * Incompressible energy oscillation; a screening number above about Mach 2, not a hypersonic mode.
 */
export function lanchesterPhugoid(v: number, ld: number, g0 = G0): { wn: number; zeta: number; period: number } {
  const wn = (g0 * Math.sqrt(2)) / Math.max(v, 1);
  const zeta = 1 / (Math.sqrt(2) * Math.max(ld, 1e-6));
  return { wn, zeta, period: (2 * Math.PI) / wn };
}
