/**
 * Lifting-surface (fin / wing / tail / canard) panel aerodynamics, "alone" (no body).
 *
 * Reference-frame: coefficients per PANEL, based on the exposed panel planform area S_e, unless the
 * function name says otherwise; assembly code multiplies by S_e / Sref.
 *
 * Lift-curve slope CLa (per rad, based on the area of the two exposed panels joined, aspect ratio
 * A = (2 b_e)^2 / (2 S_e)):
 *  - subsonic (M <= 0.9):  DATCOM 1976 Sec. 4.1.3.2 Eq. 4.1.3.2-a (straight tapered wing,
 *      Helmbold/Diederich form)  CLa = 2 pi A / (2 + sqrt(4 + A^2 beta^2/k^2 (1 + tan^2(L_c/2)/beta^2))),
 *      k = cl_alpha(section) / (2 pi) taken = 1 (thin section), L_c/2 = half-chord sweep.
 *  - supersonic (M >= 1.2):
 *      subsonic leading edge (m = beta cot(L_LE) < 1): triangular-wing exact linear result
 *        CLa = (pi A / 2) / E(sqrt(1 - m^2))  (Jones/Puckett; E = complete elliptic integral of the
 *        second kind; reduces to slender pi A/2 as m->0 and to Ackeret 4/beta at m=1 for a delta),
 *      supersonic leading edge: Ackeret 4/beta with the classical rectangular-wing tip-loss
 *        CLa = (4/beta) (1 - 1/(2 A beta)) for A beta >= 1, and a linear engineering interpolation
 *        A (pi/2 + (2 - pi/2) A beta) for A beta < 1 (NOT a published formula; flagged low/medium),
 *      smooth blend across 0.8 < m < 1.25 and capped at the 2-D Ackeret value.
 *  - 0.9 < M < 1.2: smoothstep interpolation between the two (transonic, LOW confidence).
 *
 * Non-linear normal force: supersonic: exact flat-plate shock-expansion section cn scaled by the
 * 3-D reduction CLa3D/(4/beta); subsonic/transonic: linear with soft stall plus a crossflow term
 * Cd_p sin|a| sin a cos a (flat plate normal drag, Hoerner, Fluid-Dynamic Drag, approximate).
 *
 * Section thickness wave drag (supersonic, linear theory / Ackeret): cd_w = (4/beta_n) I cos(L),
 * I = integral of the squared surface slope (diamond I=(t/c)^2, biconvex I=4/3 (t/c)^2),
 * beta_n from M cos(L) (infinite swept wing, Jones). Friction: reference-temperature Cf + Raymer
 * wing form factor. Leading-edge bluntness drag: Newtonian cylinder (2/3 Cp,max) on the normal
 * velocity component.
 * Trailing-edge flap effectiveness: thin-airfoil theory (Glauert), tau = 1 - (theta_h - sin theta_h)/pi,
 * theta_h = acos(2 cf/c - 1).
 */
import type { FinGeom } from "./geometry.ts";
import { sectionSlopeIntegral } from "./geometry.ts";
import type { CfOptions, Flow } from "./flow.ts";
import { skinFrictionCf } from "./flow.ts";
import { cpMaxNewton, flatPlateCn } from "./gasdyn.ts";
import { DEG, clamp, ellipticE, smoothstep } from "./util.ts";

export function subsonicCLa(g: FinGeom, M: number): number {
  const A = Math.max(g.aspect, 1e-3);
  const b2 = Math.max(1 - M * M, 1e-4);
  const t2 = g.tanHC * g.tanHC;
  return (2 * Math.PI * A) / (2 + Math.sqrt(4 + A * A * b2 * (1 + t2 / b2)));
}

export function supersonicCLa(g: FinGeom, M: number): number {
  const A = Math.max(g.aspect, 1e-3);
  const beta = Math.sqrt(Math.max(M * M - 1, 1e-6));
  const ack = 4 / beta;
  const tanLE = Math.max(g.tanLE, 1e-6);
  const m = beta / tanLE; // beta*cot(L_LE)
  // subsonic-LE branch
  const mm = clamp(m, 0, 1);
  const c1 = (Math.PI * A) / 2 / ellipticE(Math.sqrt(1 - mm * mm));
  // supersonic-LE branch
  const Ab = A * beta;
  const c2 = Ab >= 1 ? ack * (1 - 1 / (2 * Ab)) : A * (Math.PI / 2 + (2 - Math.PI / 2) * Ab);
  const w = smoothstep(0.8, 1.25, m);
  return Math.min((1 - w) * c1 + w * c2, ack);
}

/**
 * Lift-curve slope of the panel pair (per rad, on the joined exposed planform).
 * Transonic (0.8–1.2): fair the subsonic DATCOM value (evaluated at an effective Mach
 * capped by a Korn-like critical Mach estimate) into the M=1.2 supersonic linear value
 * with a C¹ smoothstep, and apply a soft sonic-peak attenuation of at most ~15% near M=1
 * (engineering device so CLa does not overshoot; NOT Lock's exact form). Confidence: low.
 *
 * Korn-like Mdd estimate used only to freeze the subsonic branch before the drag rise:
 *   Mdd ≈ 0.87 − 0.125 / cos(Λ_qc) − t/c   (Korn equation engineering reduction for a
 *   supercritical section with κ≈0.87; Mason / ESDU-style preliminary-design form).
 */
export function criticalMachEstimate(g: FinGeom): number {
  const cosL = 1 / Math.sqrt(1 + g.tanQC * g.tanQC);
  const t = g.set.thickness;
  return clamp(0.87 - 0.125 / Math.max(cosL, 0.3) - t, 0.55, 0.95);
}

export function finCLa(g: FinGeom, M: number): { CLa: number; regime: "subsonic" | "transonic" | "supersonic"; tag: string } {
  const A = Math.max(g.aspect, 1e-4);
  // Guard extreme AR: slender limit πA/2, high-AR → 2π (subsonic) / 4/β (supersonic)
  if (M <= 0.8) {
    let cla = subsonicCLa(g, Math.max(M, 0));
    if (A < 0.15) cla = Math.min(cla, (Math.PI * A) / 2 * 1.05);
    return { CLa: cla, regime: "subsonic", tag: "DATCOM subsonic straight-taper CLa (Eq. 4.1.3.2-a)" };
  }
  if (M >= 1.2) {
    let cla = supersonicCLa(g, M);
    if (A < 0.15) {
      const beta = Math.sqrt(M * M - 1);
      cla = Math.min(cla, (Math.PI * A) / 2 / Math.max(ellipticE(0), 1e-6) * 1.0);
      void beta;
    }
    return { CLa: cla, regime: "supersonic", tag: "supersonic linear theory (Ackeret/triangular-wing/tip-loss)" };
  }
  // Transonic fairing 0.8 → 1.2
  const Mdd = criticalMachEstimate(g);
  const Msub = Math.min(M, Math.max(Mdd, 0.8));
  const a = subsonicCLa(g, Msub);
  const b = supersonicCLa(g, 1.2);
  const w = smoothstep(0.8, 1.2, M);
  // soft sonic-peak attenuation (max 15% cut near M=1)
  const peak = 1 - 0.15 * Math.exp(-Math.pow((M - 1) / 0.12, 2));
  const cla = (a * (1 - w) + b * w) * peak;
  return {
    CLa: cla,
    regime: "transonic",
    tag: `transonic fairing 0.8–1.2 (Korn-like Mdd≈${Mdd.toFixed(2)}; soft sonic peak) — low confidence`,
  };
}

/** thin-airfoil flap effectiveness (Glauert) */
export function flapEffectiveness(cfoc: number): number {
  const E = clamp(cfoc, 0, 1);
  if (E >= 0.9999) return 1;
  if (E <= 1e-4) return 0;
  const th = Math.acos(2 * E - 1);
  return 1 - (th - Math.sin(th)) / Math.PI;
}

/** flat-plate normal-force drag coefficient vs aspect ratio (Hoerner, approximate). */
export function plateCrossflowCd(A: number): number {
  return 1.17 + 0.81 * clamp(1 - 1 / Math.max(A, 1), 0, 1);
}

/**
 * Panel normal-force coefficient (based on S_e) along the panel-normal direction for an effective
 * incidence aEff [rad] (signed).
 */
export function panelNormal(g: FinGeom, M: number, aEff: number, gamma: number): number {
  const { CLa } = finCLa(g, M);
  // Clamp incidence for the attached-flow branch; beyond ~90° use a sin-cos bluff model.
  const aRaw = Math.abs(aEff);
  const sgn = aEff < 0 ? -1 : 1;
  const a = Math.min(aRaw, Math.PI); // [0, π]
  const cdp = plateCrossflowCd(Math.max(g.aspect, 1e-3));
  let n: number;
  if (a > 70 * DEG) {
    // bluff / reverse-flow: CN ≈ Cd_p sin a |sin a|  (Hoerner flat-plate normal)
    n = cdp * Math.sin(a) * Math.abs(Math.sin(a));
  } else if (M < 1.2) {
    const aSat = 22 * DEG;
    const as = aSat * Math.tanh(a / aSat);
    n = CLa * as * Math.cos(a) + cdp * Math.sin(a) * Math.sin(a) * Math.cos(a);
  } else {
    const beta = Math.sqrt(Math.max(M * M - 1, 1e-6));
    const ratio = clamp(CLa / (4 / beta), 0, 1.0);
    const wnl = 1 - smoothstep(1.2, 3.5, M);
    const aSE = Math.min(a, 45 * DEG); // shock-expansion branch loses meaning past ~45°
    n = ratio * flatPlateCn(M, aSE, gamma) + wnl * cdp * Math.sin(a) * Math.sin(a) * Math.cos(Math.min(a, 90 * DEG));
  }
  return Number.isFinite(n) ? sgn * n : 0;
}

/** panel loading slope dN/da at a=0 (per rad, based on S_e) */
export function panelSlope(g: FinGeom, M: number): number {
  return finCLa(g, M).CLa;
}

/** chordwise centre of pressure of the alone panel from the nose [m] (strip centroid, see header notes). */
export function finXcp(g: FinGeom, M: number): number {
  const A = g.aspect;
  const fcSub = 0.25 + 0.25 * clamp((4 - A) / 2.5, 0, 1);
  const fc = fcSub + (0.5 - fcSub) * smoothstep(0.8, 1.2, M);
  const N = 40;
  let s = 0;
  let sc = 0;
  for (let i = 0; i < N; i++) {
    const eta = (i + 0.5) / N;
    const c = g.chordAt(eta);
    s += (g.xLEAt(eta) + fc * c) * c;
    sc += c;
  }
  return s / Math.max(sc, 1e-12);
}

export interface FinAxial {
  /** zero-lift axial force of ONE panel, coefficient on S_e */
  cdFric: number;
  cdWave: number;
  cdLE: number;
  cd0: number;
}

export function finAxial(g: FinGeom, flow: Flow, cf: CfOptions): FinAxial {
  const M = flow.mach;
  const t = g.set.thickness;
  const cfv = skinFrictionCf(flow, Math.max(g.mac, 1e-6), cf);
  const cosLm = 1 / Math.sqrt(1 + g.tanHC * g.tanHC);
  const xcm = g.set.section === "hex" || g.set.section === "flat" ? 0.5 : 0.3;
  const ffRaw = (1 + (0.6 / xcm) * t + 100 * Math.pow(t, 4)) * 1.34 * Math.pow(Math.max(M, 0.2), 0.18) * Math.pow(cosLm, 0.28);
  const ff = 1 + (Math.max(ffRaw, 1) - 1) * (1 - smoothstep(0.9, 1.2, M));
  const cdFric = cfv * 2 * (1 + 0.2 * t) * ff;
  // wave drag of the thickness distribution
  const I = sectionSlopeIntegral(g.set);
  const cosLe = 1 / Math.sqrt(1 + g.tanLE * g.tanLE);
  const cosL = 1 / Math.sqrt(1 + g.tanHC * g.tanHC);
  const waveAt = (mn: number) => (4 * I * cosL) / Math.sqrt(Math.max(mn * mn - 1, 1e-6));
  const Mn = M * cosL;
  let cdWave: number;
  if (Mn >= 1.2) cdWave = waveAt(Mn);
  else if (Mn <= 0.8) cdWave = 0;
  else {
    // Lock/Korn-style drag rise: Cd_w ≈ 20 (M − Mdd)^4 for M > Mdd (Korn/Lock engineering form
    // used in preliminary design; Lock 1951 / Korn equation family). Blended to the M=1.2
    // Ackeret value so the curve is continuous.
    const Mdd = criticalMachEstimate(g);
    const lock = Mn > Mdd ? 20 * Math.pow(Mn - Mdd, 4) * (1 + 5 * t) : 0;
    const ack12 = waveAt(1.2);
    const w = smoothstep(0.8, 1.2, Mn);
    cdWave = lock * (1 - w) + ack12 * w;
  }
  let cdLE = 0;
  const le = g.set.leRadius ?? 0;
  if (le > 0 && M > 1.2) {
    cdLE = ((2 / 3) * cpMaxNewton(M, flow.gamma) * (2 * le) * g.exposedSpan * cosLe * cosLe) / Math.max(g.area, 1e-12);
  }
  return { cdFric, cdWave, cdLE, cd0: cdFric + cdWave + cdLE };
}

/** roll-damping contribution of one panel: Clp (per rad of p b/2V) = -2 CLa cY2 / (Sref b^2) */
export function panelRollDamping(g: FinGeom, M: number, sRef: number, latRef: number): number {
  const { CLa } = finCLa(g, M);
  return (-2 * CLa * g.cY2) / (sRef * latRef * latRef);
}
