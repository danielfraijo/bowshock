/**
 * Independent reference calculations + the extended validation checks.
 *
 * Every "expected" value here is produced one of three ways, and each check says which:
 *  - closed-form: an explicit analytic relation re-derived in this file from the textbook
 *    formula (not by calling the function under test),
 *  - independent-code: a separate implementation (scripts/indep_taylor_maccoll.py, a
 *    brute-force scan, a layered barometric integration) whose numbers are pasted in,
 *  - invariant: a conservation law or limit the answer must satisfy.
 * No published table value is quoted from memory. Where a textbook number is mentioned it is
 * because the closed form reproduces it, and the check compares against the closed form.
 */

import type { Check } from "./validate";
import {
  DEG,
  RAD,
  airMu,
  betaFromThetaM,
  coneSurfaceStateExact,
  detraKempRiddell,
  eckertStanton,
  fastInvPrandtlMeyer,
  isentropic,
  newtonianCpMax,
  normalShock,
  obliqueShock,
  prandtlMeyer,
  solveConeShock,
  stantonToCf,
  suttonGraves,
  thetaFromBetaM,
  vanDriestII,
  viscousInteractionPressure,
} from "./math";
import { atmosphere } from "./atmosphere";

/* ---------------- independent references ---------------- */

/** θ(β, M) straight from Anderson (4.17). */
export function refTheta(M: number, beta: number, g = 1.4): number {
  const s = Math.sin(beta);
  return Math.atan((2 / Math.tan(beta)) * (M * M * s * s - 1) / (M * M * (g + Math.cos(2 * beta)) + 2));
}

/** Maximum attached deflection by brute-force β scan. */
export function refThetaMax(M: number, g = 1.4): { theta: number; beta: number } {
  let best = 0;
  let bb = 0;
  const mu = Math.asin(1 / M);
  const N = 40000;
  for (let i = 1; i < N; i++) {
    const b = mu + ((Math.PI / 2 - mu) * i) / N;
    const t = refTheta(M, b, g);
    if (t > best) {
      best = t;
      bb = b;
    }
  }
  return { theta: best, beta: bb };
}

export function refNu(M: number, g = 1.4): number {
  const a = Math.sqrt((g + 1) / (g - 1));
  const b = Math.sqrt(M * M - 1);
  return a * Math.atan(b / a) - Math.atan(b);
}

export function refInvNu(nu: number, g = 1.4): number {
  let lo = 1;
  let hi = 200;
  for (let i = 0; i < 200; i++) {
    const m = 0.5 * (lo + hi);
    if (refNu(m, g) < nu) lo = m;
    else hi = m;
  }
  return 0.5 * (lo + hi);
}

/** US76 by layered barometric integration (geometric km → T, p). */
export function refUs76(hKm: number): { T: number; p: number; rho: number } {
  const g0 = 9.80665;
  const R = 287.0531;
  const r0 = 6356766;
  const h = hKm * 1000;
  const H = (r0 * h) / (r0 + h);
  const base = [0, 11000, 20000, 32000, 47000, 51000, 71000];
  const lapse = [-0.0065, 0, 0.001, 0.0028, 0, -0.0028, -0.002];
  let T = 288.15;
  let p = 101325;
  for (let i = 0; i < base.length; i++) {
    const top = i + 1 < base.length ? base[i + 1] : 1e9;
    const dz = Math.min(H, top) - base[i];
    if (dz <= 0) break;
    const L = lapse[i];
    if (Math.abs(L) < 1e-12) p *= Math.exp((-g0 * dz) / (R * T));
    else {
      const T2 = T + L * dz;
      p *= Math.pow(T2 / T, -g0 / (R * L));
      T = T2;
    }
    if (H <= top) break;
  }
  return { T, p, rho: p / (R * T) };
}

/** Eckert reference-temperature turbulent flat-plate Cf (Cf* = 0.0592 Re*^-0.2 form, Meador–Smart). */
export function refEckertTurbCf(Re: number, M: number, Te: number, Tw: number, g = 1.4): number {
  const r = 0.89;
  const Taw = Te * (1 + r * 0.5 * (g - 1) * M * M);
  const Ts = Te + 0.5 * (Tw - Te) + 0.22 * (Taw - Te);
  const mus = airMu(Ts);
  const mue = airMu(Te);
  // Re* = (ρ*/ρe)(μe/μ*) Re_e, ρ*/ρe = Te/T*
  const ReS = Re * (Te / Ts) * (mue / mus);
  // incompressible Cf at Re*, then Cf = Cf*(ρ*/ρe)
  const cfInc = 0.455 / Math.pow(Math.log10(ReS), 2.58);
  return cfInc * (Te / Ts);
}

/* ---------------- check builders ---------------- */

type Dom = Check["domain"];
function mk(
  id: string,
  domain: Dom,
  name: string,
  formula: string,
  expected: number,
  got: number,
  source: string,
  tol: number,
  basis: Check["basis"],
  absTol = 1e-9,
): Check {
  const absErr = Math.abs(got - expected);
  const relErr = absErr / Math.max(Math.abs(expected), 1e-12);
  // expected = 0 means "error metric must stay below absTol": report that bound as the tolerance.
  const bound = expected === 0;
  return {
    id, domain, name, formula, expected, got,
    relErr: bound ? absErr : relErr,
    absErr,
    tol: bound ? absTol : tol,
    basis, source,
    pass: Number.isFinite(got) && (bound ? absErr <= absTol : relErr <= tol || absErr <= absTol),
  };
}

export function extendedChecks(): Check[] {
  const out: Check[] = [];
  const g = 1.4;

  // --- strong-shock and weak-wave limits (closed form) ---
  const ns = normalShock(1e4, g);
  out.push(mk("ns-rho-inf", "gas", "Normal shock density ratio, M→∞", "ρ₂/ρ₁ → (γ+1)/(γ−1) = 6", 6, ns.r2r1, "Rankine–Hugoniot", 1e-4, "closed-form"));
  out.push(mk("ns-M2-inf", "gas", "Normal shock M₂, M→∞", "M₂ → √((γ−1)/(2γ)) = 0.37796", Math.sqrt((g - 1) / (2 * g)), normalShock(1e4, g).M2, "Rankine–Hugoniot", 1e-4, "closed-form"));
  const M0 = 6;
  const mu = Math.asin(1 / M0);
  out.push(mk("obl-mach-wave", "external", "Weak-wave limit, M=6, θ→0", "β → μ = asin(1/M)", mu * RAD, betaFromThetaM(M0, 1e-4 * DEG, g) * RAD, "oblique shock theory", 1e-3, "closed-form"));

  // --- oblique shock vs Anderson 4.17 for a sweep ---
  let worst = 0;
  for (const M of [1.5, 2, 3, 5, 8, 12, 20]) {
    const tmax = refThetaMax(M, g).theta;
    for (const f of [0.1, 0.4, 0.7, 0.95]) {
      const th = f * tmax;
      const bw = betaFromThetaM(M, th, g);
      worst = Math.max(worst, Math.abs(refTheta(M, bw, g) - th) / th);
    }
  }
  out.push(mk("tbm-sweep", "external", "θ-β-M inverse residual, 28 (M, θ) cases", "max |θ_ref(β(θ,M)) − θ|/θ over M=1.5…20, θ=0.1…0.95 θmax", 0, worst, "Anderson Eq. 4.17 (re-derived)", 1, "closed-form", 1e-6));

  const tm2 = refThetaMax(2, g);
  out.push(mk("theta-max-2", "external", "Maximum attached deflection, M=2", "largest θ of Eq. 4.17 (brute-force β scan, ≈22.97°) vs the detachment bound used by betaFromThetaM", tm2.theta * RAD, detachAngle(2, g) * RAD, "Anderson Fig. 4.8 (scan of Eq. 4.17)", 3e-3, "independent-code"));

  // --- conservation across an oblique shock ---
  const ob = obliqueShock(8, 25 * DEG, g);
  const T01 = 1 + 0.5 * (g - 1) * 64;
  const T02 = ob.t2t1 * (1 + 0.5 * (g - 1) * ob.M2 * ob.M2);
  out.push(mk("obl-h0", "external", "Total enthalpy across oblique shock, M=8 β=25°", "T₀₂/T₀₁ = 1", 1, T02 / T01, "energy conservation", 1e-6, "invariant"));
  const ds = Math.log(ob.t2t1) * (g / (g - 1)) - Math.log(ob.p2p1);
  const dsPt = -Math.log((ob.p2p1 * isentropic(ob.M2, g).ptP) / isentropic(8, g).ptP);
  out.push(mk("obl-entropy", "external", "Entropy rise across oblique shock, M=8 β=25°", "Δs/R = γ/(γ−1) ln(T₂/T₁) − ln(p₂/p₁) must equal −ln(p₀₂/p₀₁) (and be > 0)", dsPt, ds > 0 ? ds : NaN, "second law / isentropic stagnation", 1e-9, "invariant"));

  // --- Prandtl–Meyer ---
  out.push(mk("pm-inf", "external", "Prandtl–Meyer ν(M→∞)", "ν_max = (π/2)(√((γ+1)/(γ−1)) − 1) = 130.454°", 90 * (Math.sqrt(6) - 1), prandtlMeyer(1e5, g) * RAD, "Anderson Eq. 4.44", 1e-3, "closed-form"));
  let pmWorst = 0;
  for (const M of [1.1, 1.5, 2, 3, 5, 10, 20]) pmWorst = Math.max(pmWorst, Math.abs(prandtlMeyer(M, g) - refNu(M, g)));
  out.push(mk("pm-sweep", "external", "ν(M) code vs independent formula, M=1.1…20", "max |Δν| (rad)", 0, pmWorst, "Anderson Eq. 4.44", 1, "closed-form", 1e-9));
  let invWorst = 0;
  for (const nuD of [0.5, 2, 5, 10, 20, 40, 60, 80, 100, 120]) {
    const m = fastInvPrandtlMeyer(nuD * DEG, g);
    invWorst = Math.max(invWorst, Math.abs(m - refInvNu(nuD * DEG, g)) / m);
  }
  out.push(mk("pm-inv", "external", "Inverse Prandtl–Meyer, ν=0.5…120°", "max |M_code − M_bisect|/M", 0, invWorst, "Anderson Eq. 4.44", 1, "closed-form", 5e-4));
  // 10° expansion from M=2
  const M2e = refInvNu(refNu(2, g) + 10 * DEG, g);
  const pr = isentropic(M2e, g).ptP ** -1 / isentropic(2, g).ptP ** -1; // p₂/p₁ = (p/p₀)(M₂)/(p/p₀)(M₁)
  const prCode = isentropic(fastInvPrandtlMeyer(prandtlMeyer(2, g) + 10 * DEG, g), g).p / isentropic(2, g).p;
  out.push(mk("pm-expand10", "external", "Expansion fan M=2, 10° turn, p₂/p₁", "isentropic p/p₀ at ν(M₁)+10°; M₂ by bisection on Eq. 4.44", pr, prCode, "Anderson §4.8", 1e-4, "closed-form"));

  // --- Newtonian Cp limit ---
  const gm = g;
  const Mh = 1e3;
  const p02p1 = Math.pow(((gm + 1) * (gm + 1) * Mh * Mh) / (4 * gm * Mh * Mh - 2 * (gm - 1)), gm / (gm - 1)) * ((1 - gm + 2 * gm * Mh * Mh) / (gm + 1));
  out.push(mk("cp-newton-lim", "external", "Newtonian Cp,max at M=10³ (Rayleigh pitot)", "[(p₀₂/p∞) − 1]/(½γM²), p₀₂ by Rayleigh formula → 1.8394 as M→∞", (p02p1 - 1) / (0.5 * gm * Mh * Mh), newtonianCpMax(Mh, gm), "Rayleigh pitot / Lees", 1e-6, "closed-form"));

  // --- Taylor–Maccoll vs independent RK4 (scripts/indep_taylor_maccoll.py, dθ = 0.001°) ---
  const cones: [number, number, number, number][] = [
    // M, cone°, β° , Cp   (python: IND lines)
    [8, 10, 12.98548, 0.0684743],
    [8, 5, 8.74437, 0.0198039],
    [8, 20, 23.17266, 0.2512042],
    [5, 10, 15.60828, 0.0747604],
    [2, 10, 31.20609, 0.1044709],
    [10, 15, 17.41265, 0.1440632],
    [3, 20, 29.61462, 0.2842698],
    [8, 1, 7.18740, 0.0014040],
    [8, 2, 7.28437, 0.0043305],
    [8, 30, 34.13783, 0.5312781],
  ];
  let bWorst = 0;
  let cWorst = 0;
  for (const [M, c, b, cp] of cones) {
    const bb = solveConeShock(M, c * DEG, g) * RAD;
    const st = coneSurfaceStateExact(M, c * DEG, g);
    bWorst = Math.max(bWorst, Math.abs(bb - b) / b);
    cWorst = Math.max(cWorst, Math.abs(st.cp - cp) / cp);
  }
  out.push(mk("tm-beta-10", "external", "Cone shock angle, 10 (M, θc) cases (M=2…10, θc=1°…30°)", "max rel. error vs independent RK4 (python, dθ=0.001°)", 0, bWorst, "Taylor–Maccoll; scripts/indep_taylor_maccoll.py", 1, "independent-code", 1e-4));
  out.push(mk("tm-cp-10", "external", "Cone surface Cp, same 10 cases", "max rel. error vs independent RK4 (python); includes 1° and 2° cones the old code clipped", 0, cWorst, "Taylor–Maccoll; scripts/indep_taylor_maccoll.py", 1, "independent-code", 1e-3));

  // --- atmosphere vs layered barometric integral ---
  let aw = 0;
  for (const h of [0, 5, 11, 15, 20, 25, 30, 40, 50, 60, 70]) {
    const a = atmosphere(h, 0, g);
    const r = refUs76(h);
    aw = Math.max(aw, Math.abs(a.T - r.T) / r.T, Math.abs(a.p - r.p) / r.p, Math.abs(a.rho - r.rho) / r.rho);
  }
  out.push(mk("us76-sweep", "gas", "US76 T, p, ρ at 11 altitudes 0–70 km", "max rel. error vs layered barometric integration (g₀=9.80665, R=287.0531)", 0, aw, "U.S. Standard Atmosphere 1976", 1, "independent-code", 1e-4));

  // --- skin friction ---
  out.push(mk("blasius", "external", "Laminar flat-plate Cf·√Re_x (low Mach)", "Blasius 0.664; code path = Eckert St → Cf = 2 St Pr^(2/3)", 0.664, laminarCfRe(0.2), "Blasius / Schlichting", 0.08, "closed-form"));
  const cfRef = refEckertTurbCf(1e7, 8, 226, 800, g);
  out.push(mk("vd2-vs-eckert", "external", "Turbulent Cf, M=8, Re=10⁷, Tw=800 K, Te=226 K", "van Driest II vs Eckert reference-temperature method (independent)", cfRef, vanDriestII(1e7, 8, 226, 800, g), "van Driest 1956 / Eckert 1955", 0.15, "independent-code"));
  const cfRef2 = refEckertTurbCf(1e7, 5, 220, 1500, g);
  out.push(mk("vd2-vs-eckert-hot", "external", "Turbulent Cf, M=5, Re=10⁷, Tw=1500 K", "van Driest II vs Eckert reference-temperature method (independent)", cfRef2, vanDriestII(1e7, 5, 220, 1500, g), "van Driest 1956 / Eckert 1955", 0.15, "independent-code"));
  const stT = eckertStanton(6e6, 220, 800, 4.0, 1.4, true);
  out.push(mk("st-cf-turb", "external", "Reynolds analogy, turbulent: Cf = 2 St Pr^(2/3)", "stantonToCf(St) / (2 St Pr^(2/3))", 1, stantonToCf(stT, true) / (2 * stT * Math.pow(0.71, 2 / 3)), "Reynolds analogy (Pr^(2/3) Colburn factor)", 1e-3, "closed-form"));

  // --- heating correlations ---
  const a30 = atmosphere(30, 8, g);
  const V = 8 * a30.a;
  const sg = suttonGraves(a30.rho, V, 0.05, 1);
  const dk = detraKempRiddell(a30.rho, V, 0.05, 1);
  out.push(mk("dkr-sg", "external", "Detra–Kemp–Riddell vs Sutton–Graves, M8, 30 km, Rn=5 cm", "q_DKR/q_SG (both ≈ same physics, scatter ~±30 %); the old DKR constant was 4000× low", 1, dk / sg, "Detra, Kemp, Riddell 1957 / SG 1971", 0.2, "consistency"));
  out.push(mk("sg-scale", "external", "Sutton–Graves scaling", "q(4ρ, Rn=1, V) / q(ρ, Rn=1, V) = 2 ; q(ρ, 4Rn)/q(ρ, Rn) = ½ ; q(ρ, 2V)/q(ρ,V) = 8", 2 + 0.5 + 8, suttonGraves(4, 1000, 1) / suttonGraves(1, 1000, 1) + suttonGraves(1, 1000, 4) / suttonGraves(1, 1000, 1) + suttonGraves(1, 2000, 1) / suttonGraves(1, 1000, 1), "Sutton & Graves 1971 form", 1e-9, "closed-form"));

  // --- hypersonic viscous interaction (formula transcription, flagged as such) ---
  out.push(mk("hp-weak", "external", "Weak-interaction p/p∞ at χ̄=1", "1 + 0.31 χ̄ + 0.05 χ̄² (Hayes–Probstein, cold wall)", 1.36, viscousInteractionPressure(1), "Hayes & Probstein 1959; transcription check only", 1e-9, "closed-form"));

  // --- inverse consistency of thetaFromBetaM ---
  out.push(mk("tbm-theta", "external", "thetaFromBetaM vs Eq. 4.17, M=8 β=20°", "θ = atan[2 cotβ (M²sin²β−1)/(M²(γ+cos2β)+2)]", refTheta(8, 20 * DEG, g) * RAD, thetaFromBetaM(8, 20 * DEG, g) * RAD, "Anderson Eq. 4.17", 1e-9, "closed-form"));

  return out;
}

function detachAngle(M: number, g: number): number {
  // largest θ for which betaFromThetaM returns a finite weak solution
  let lo = 0;
  let hi = 0.8;
  for (let i = 0; i < 60; i++) {
    const m = 0.5 * (lo + hi);
    const b = betaFromThetaM(M, m, g);
    if (Number.isFinite(b) && b > 0) lo = m;
    else hi = m;
  }
  return lo;
}

function laminarCfRe(M: number): number {
  const Rex = 1e6;
  const St = eckertStanton(Rex, 288, 288 * (1 + Math.sqrt(0.71) * 0.2 * M * M), M, 1.4, false);
  return stantonToCf(St, false) * Math.sqrt(Rex);
}
