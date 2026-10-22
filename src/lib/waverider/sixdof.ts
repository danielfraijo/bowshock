import { DEG, RAD, clamp } from "./math";
import type { DesignParams, TriMesh } from "./types";
import type { Atmosphere } from "./atmosphere";
import type { MassProps } from "./mass";
import { panelAero, type PanelAero } from "./panel";

export interface StabDerivs {
  cla: number;
  cda: number;
  cma: number;
  cyb: number;
  cnb: number;
  clb: number;
  clq: number;
  cmq: number;
  cyp: number;
  clp: number;
  cnp: number;
  cnr: number;
  clr: number;
  cyr: number;
  staticMargin: number;
  staticMarginPct: number;
  trimAlpha: number;
  trimCm: number;
  longitudinallyStable: boolean;
  directionallyStable: boolean;
  rollStable: boolean;
}

export interface RigidMode {
  name: string;
  wn: number;
  fn: number;
  zeta: number;
  period: number;
  tHalf: number;
  stable: boolean;
  note: string;
}

export interface SixDofSample {
  t: number;
  alphaDeg: number;
  betaDeg: number;
  pDeg: number;
  qDeg: number;
  rDeg: number;
  phiDeg: number;
  thetaDeg: number;
  psiDeg: number;
  V: number;
  gammaDeg: number;
  hKm: number;
  xKm: number;
}

export interface SixDofResult {
  derivs: StabDerivs;
  modes: RigidMode[];
  samples: SixDofSample[];
  polar: { a: number; cl: number; cd: number; cm: number; ld: number }[];
  atAlpha: PanelAero;
  iterations: number;
  dt: number;
  notes: string[];
  skipped: boolean;
  method: string;
}

function emptyDerivs(): StabDerivs {
  return {
    cla: 0,
    cda: 0,
    cma: 0,
    cyb: 0,
    cnb: 0,
    clb: 0,
    clq: 0,
    cmq: 0,
    cyp: 0,
    clp: 0,
    cnp: 0,
    cnr: 0,
    clr: 0,
    cyr: 0,
    staticMargin: 0,
    staticMarginPct: 0,
    trimAlpha: 0,
    trimCm: 0,
    longitudinallyStable: false,
    directionallyStable: false,
    rollStable: false,
  };
}

function modeOf(name: string, wn: number, zeta: number, note: string, extraStable = true): RigidMode {
  const stable = extraStable && Number.isFinite(wn) && wn > 0 && zeta > -0.02;
  const period = wn > 1e-8 ? (2 * Math.PI) / wn : Infinity;
  const tHalf =
    Math.abs(zeta * wn) > 1e-8 ? (Math.LN2 / Math.abs(zeta * wn)) * (zeta >= 0 ? 1 : -1) : Infinity;
  return {
    name,
    wn: Number.isFinite(wn) ? wn : 0,
    fn: Number.isFinite(wn) ? wn / (2 * Math.PI) : 0,
    zeta: Number.isFinite(zeta) ? zeta : 0,
    period: Number.isFinite(period) ? period : 0,
    tHalf: Number.isFinite(tHalf) ? tHalf : 0,
    stable,
    note,
  };
}

/**
 * Rigid-body state in the panel frame (X aft, Y span, Z up). p, q, r are body rates about +X, +Y, +Z,
 * β > 0 sends the flow toward +Y (β_panel = −β_conventional). α, θ, γ are the conventional pitch-plane
 * angles (θ = α + γ in wings-level flight). Lengths m, speeds m/s, angles rad.
 */
export interface RigidState {
  alpha: number;
  q: number;
  theta: number;
  beta: number;
  p: number;
  r: number;
  phi: number;
  VV: number;
  gamma: number;
  h: number;
}

/**
 * Coefficients in the panel book: CL, CD wind axes; CY along +Y; Cl, Cm, Cn about +X, +Y, +Z, all three
 * moments on Sref·Lref (exactly what `panelAero` returns as cll, cm, cn).
 */
export interface RigidCoeffs {
  CL: number;
  CD: number;
  CY: number;
  Cl: number;
  Cm: number;
  Cn: number;
}

export interface RigidBody {
  S: number;
  /** Moment reference length for all three axes (panel Lref). */
  L: number;
  m: number;
  Ixx: number;
  Iyy: number;
  Izz: number;
  rho: (h: number) => number;
  /** Earth radius for inverse-square gravity and the path curvature term. */
  Re?: number;
  g0?: number;
}

export interface RigidRates {
  aDot: number;
  qDot: number;
  thDot: number;
  bDot: number;
  pDot: number;
  rDot: number;
  phiDot: number;
  vDot: number;
  gDot: number;
  hDot: number;
  xDot: number;
}

/**
 * State equations of the linear 6DOF time history (`solveSixDof`), exported so the audit can check them.
 * `guidance.ts` keeps its own copy because it also carries the bank-angle (Vinh) terms; the same two fixes are applied there.
 *
 * α̇ = q − γ̇ with the spherical point-mass γ̇ used by the 3DOF (lift, curvature V²/(R+h), inverse-square g),
 * so incidence stays θ − γ. That is main's equation (1812dcc), moved here unchanged.
 *
 * [accuracy-audit] Two further fixes:
 * - Roll / yaw moments are q·S·Lref·C, because `panelAero` normalises Cll and Cn by Sref·Lref. The old
 *   q·S·bref·C scaled them by b/L.
 * - β̇ = p sinα − r cosα − Y/(mV) in the panel frame (β_panel = −β_conventional, Y along +Y). The old + sign
 *   made side force feed sideslip instead of opposing it.
 * Energy: with CD = 0, ½V² − g0·Re²/(Re + h) is an exact invariant of these equations (lift does no work).
 */
export function rigidBodyRates(st: RigidState, c: RigidCoeffs, body: RigidBody): RigidRates {
  const Re = body.Re ?? 6371000;
  const g0 = body.g0 ?? 9.80665;
  const V = Math.max(st.VV, 1e-3);
  const qdyn = 0.5 * body.rho(st.h) * V * V;
  const Lift = qdyn * body.S * c.CL;
  const Drag = qdyn * body.S * c.CD;
  const Y = qdyn * body.S * c.CY;
  const gLoc = g0 * (Re / (Re + st.h)) ** 2;
  const gDot = V > 20 ? Lift / (body.m * V) + (V / (Re + st.h) - gLoc / V) * Math.cos(st.gamma) : 0;
  const qDot = (qdyn * body.S * body.L * c.Cm) / body.Iyy;
  const bDot = st.p * Math.sin(st.alpha) - st.r * Math.cos(st.alpha) - Y / (body.m * V);
  const pDot = (qdyn * body.S * body.L * c.Cl) / body.Ixx;
  const rDot = (qdyn * body.S * body.L * c.Cn) / body.Izz;
  const phiDot = st.p + Math.tan(st.theta) * (st.q * Math.sin(st.phi) + st.r * Math.cos(st.phi));
  const vDot = -Drag / body.m - gLoc * Math.sin(st.gamma);
  const hDot = V * Math.sin(st.gamma);
  const xDot = (V * Math.cos(st.gamma) * Re) / (Re + st.h);
  return { aDot: st.q - gDot, qDot, thDot: st.q, bDot, pDot, rDot, phiDot, vDot, gDot, hDot, xDot };
}

/** Specific mechanical energy, J/kg, for the inverse-square field used by `rigidBodyRates`. */
export function specificEnergy(V: number, h: number, Re = 6371000, g0 = 9.80665): number {
  return 0.5 * V * V - (g0 * Re * Re) / (Re + h);
}

/** One classical RK4 step of `rigidBodyRates` with the coefficients held over the step (zero-order hold). */
export function rk4Step<S extends RigidState & { x: number }>(st: S, dt: number, coeffs: (s: RigidState) => RigidCoeffs, body: RigidBody): S {
  const add = (s: S, k: RigidRates, f: number): S => ({
    ...s,
    alpha: s.alpha + f * k.aDot,
    q: s.q + f * k.qDot,
    theta: s.theta + f * k.thDot,
    beta: s.beta + f * k.bDot,
    p: s.p + f * k.pDot,
    r: s.r + f * k.rDot,
    phi: s.phi + f * k.phiDot,
    VV: s.VV + f * k.vDot,
    gamma: s.gamma + f * k.gDot,
    h: s.h + f * k.hDot,
    x: s.x + f * k.xDot,
  });
  const k1 = rigidBodyRates(st, coeffs(st), body);
  const s2 = add(st, k1, 0.5 * dt);
  const k2 = rigidBodyRates(s2, coeffs(s2), body);
  const s3 = add(st, k2, 0.5 * dt);
  const k3 = rigidBodyRates(s3, coeffs(s3), body);
  const s4 = add(st, k3, dt);
  const k4 = rigidBodyRates(s4, coeffs(s4), body);
  const w = (a: number, b: number, c: number, d: number) => (dt / 6) * (a + 2 * b + 2 * c + d);
  return {
    ...st,
    alpha: st.alpha + w(k1.aDot, k2.aDot, k3.aDot, k4.aDot),
    q: st.q + w(k1.qDot, k2.qDot, k3.qDot, k4.qDot),
    theta: st.theta + w(k1.thDot, k2.thDot, k3.thDot, k4.thDot),
    beta: st.beta + w(k1.bDot, k2.bDot, k3.bDot, k4.bDot),
    p: st.p + w(k1.pDot, k2.pDot, k3.pDot, k4.pDot),
    r: st.r + w(k1.rDot, k2.rDot, k3.rDot, k4.rDot),
    phi: st.phi + w(k1.phiDot, k2.phiDot, k3.phiDot, k4.phiDot),
    VV: st.VV + w(k1.vDot, k2.vDot, k3.vDot, k4.vDot),
    gamma: st.gamma + w(k1.gDot, k2.gDot, k3.gDot, k4.gDot),
    h: st.h + w(k1.hDot, k2.hDot, k3.hDot, k4.hDot),
    x: st.x + w(k1.xDot, k2.xDot, k3.xDot, k4.xDot),
  };
}

/** Etkin / Nelson linear modes + RK4 6DOF time history from panel derivatives. */
export function solveSixDof(
  mesh: TriMesh,
  params: DesignParams,
  atm: Atmosphere,
  mass: MassProps,
  sRef: number,
  lRef: number,
  full: boolean,
  baseline?: PanelAero,
): SixDofResult {
  const notes: string[] = [];
  const a0 = params.alphaDeg;
  const b0 = params.betaDeg;
  const p0 = baseline ?? panelAero(mesh, params, atm, a0, b0, sRef, lRef);
  if (!full) {
    return {
      derivs: { ...emptyDerivs(), trimAlpha: a0, trimCm: p0.cm },
      modes: [],
      samples: [],
      polar: [{ a: a0, cl: p0.cl, cd: p0.cd, cm: p0.cm, ld: p0.ld }],
      atAlpha: p0,
      iterations: 0,
      dt: 0,
      notes: ["6DOF is an external-flow tool — switch to External for rigid-body modes."],
      skipped: true,
      method: "skipped",
    };
  }

  const coef = (a: number, b: number, rate?: { p: number; q: number; r: number }) =>
    panelAero(mesh, params, atm, a, b, sRef, lRef, rate, true);
  const dA = 1.0;
  const pu = coef(a0 + dA, b0);
  const pd = coef(a0 - dA, b0);
  const pu2 = coef(a0 + 2 * dA, b0);
  const pd2 = coef(a0 - 2 * dA, b0);
  const pb = coef(a0, b0 + dA);
  const pb2 = coef(a0, b0 + 2 * dA);
  const pbm = coef(a0, b0 - dA);
  const pbm2 = coef(a0, b0 - 2 * dA);
  const hA = dA * DEG;
  const cla = (8 * (pu.cl - pd.cl) - (pu2.cl - pd2.cl)) / (12 * hA);
  const cda = (8 * (pu.cd - pd.cd) - (pu2.cd - pd2.cd)) / (12 * hA);
  const cma = (8 * (pu.cm - pd.cm) - (pu2.cm - pd2.cm)) / (12 * hA);
  const cnb = (8 * (pb.cn - pbm.cn) - (pb2.cn - pbm2.cn)) / (12 * hA);
  const cyb = (8 * (pb.cy - pbm.cy) - (pb2.cy - pbm2.cy)) / (12 * hA);
  const clb = (8 * (pb.cll - pbm.cll) - (pb2.cll - pbm2.cll)) / (12 * hA);

  const V = Math.max(50, atm.V);
  const L = Math.max(lRef, 1e-6);
  const bRef = Math.max(params.span, 1e-6);
  const qHat = 0.02;
  const pHat = 0.02;
  const rHat = 0.02;
  const qRate = (qHat * 2 * V) / L;
  const pRate = (pHat * 2 * V) / bRef;
  const rRate = (rHat * 2 * V) / bRef;
  const qP = coef(a0, b0, { p: 0, q: qRate, r: 0 });
  const qM = coef(a0, b0, { p: 0, q: -qRate, r: 0 });
  const pP = coef(a0, b0, { p: pRate, q: 0, r: 0 });
  const pM = coef(a0, b0, { p: -pRate, q: 0, r: 0 });
  const rP = coef(a0, b0, { p: 0, q: 0, r: rRate });
  const rM = coef(a0, b0, { p: 0, q: 0, r: -rRate });
  const cmq = (qP.cm - qM.cm) / (2 * qHat);
  const clq = (qP.cl - qM.cl) / (2 * qHat);
  const clp = (pP.cll - pM.cll) / (2 * pHat);
  const cnp = (pP.cn - pM.cn) / (2 * pHat);
  const cyp = (pP.cy - pM.cy) / (2 * pHat);
  const cnr = (rP.cn - rM.cn) / (2 * rHat);
  const clr = (rP.cll - rM.cll) / (2 * rHat);
  const cyr = (rP.cy - rM.cy) / (2 * rHat);

  const sm = cla !== 0 ? -cma / cla : 0;
  let trim = a0;
  let cmT = p0.cm;
  // [accuracy-audit] trimCm is now Cm evaluated AT trimAlpha (it was Cm at the previous Newton iterate).
  for (let i = 0; i < 6; i++) {
    const p = coef(trim, b0);
    cmT = p.cm;
    if (Math.abs(cmT) < 2e-4) break;
    const pp = coef(trim + 0.4, b0);
    const dcm = (pp.cm - p.cm) / 0.4;
    if (Math.abs(dcm) < 1e-6) break;
    const next = clamp(trim - p.cm / dcm, -8, 16);
    if (next === trim) break;
    trim = next;
    if (i === 5) cmT = coef(trim, b0).cm;
  }

  const polar: { a: number; cl: number; cd: number; cm: number; ld: number }[] = [];
  for (const a of [-4, -1, 2, 5, 8, 12]) {
    const p = coef(a, 0);
    polar.push({ a, cl: p.cl, cd: p.cd, cm: p.cm, ld: p.ld });
  }

  const derivs: StabDerivs = {
    cla,
    cda,
    cma,
    cyb,
    cnb,
    clb,
    clq,
    cmq,
    cyp,
    clp,
    cnp,
    cnr,
    clr,
    cyr,
    staticMargin: sm,
    staticMarginPct: sm * 100,
    trimAlpha: trim,
    trimCm: cmT,
    longitudinallyStable: cma < 0 && cla > 0,
    directionallyStable: cnb > 0,
    rollStable: clp < 0,
  };

  const qBar = Math.max(atm.q, 1);
  const S = Math.max(sRef, 1e-8);
  const m = Math.max(5, mass.mass);
  const Ixx = Math.max(1e-4, mass.Ixx);
  const Iyy = Math.max(1e-4, mass.Iyy);
  const Izz = Math.max(1e-4, mass.Izz);
  const g0 = 9.80665;

  const Z_a = -(qBar * S * cla) / m;
  const M_a = (qBar * S * L * cma) / Iyy;
  const M_q = (qBar * S * L * cmq * (L / (2 * V))) / Iyy;
  const ZaV = Z_a / V;
  const wnSp = Math.sqrt(Math.max(0, ZaV * M_q - M_a));
  const zetaSp = wnSp > 1e-8 ? -(ZaV + M_q) / (2 * wnSp) : 0;

  const ld = Math.max(0.4, p0.ld);
  const wnPh = (g0 * Math.sqrt(2)) / V;
  const zetaPh = 1 / (Math.sqrt(2) * ld);

  // [accuracy-audit] Panel Cn, Cll are on Sref·Lref, so the dimensional moments use L (was bRef → ×b/L).
  // Y_b is converted to the conventional β (CYβ_conv = −cyb_panel); the panel-frame signs of Cnβ, Clp, Cnr
  // already equal the conventional ones (see aerodb.ts derivsToConventional).
  const Y_b = -(qBar * S * cyb) / m;
  const N_b = (qBar * S * L * cnb) / Izz;
  const N_r = (qBar * S * L * cnr * (bRef / (2 * V))) / Izz;
  const L_p = (qBar * S * L * clp * (bRef / (2 * V))) / Ixx;
  const YbV = Y_b / V;
  const wnDr = Math.sqrt(Math.max(0, N_b + YbV * N_r));
  const zetaDr = wnDr > 1e-8 ? -(YbV + N_r) / (2 * wnDr) : 0;

  const tauRoll = L_p < -1e-6 ? -1 / L_p : 0;
  const lamS =
    (g0 / V) * ((clb * cnr - cnb * clr) / Math.max(1e-6, Math.abs(clp))) * Math.sign(-clp || -1);

  const modes: RigidMode[] = [
    modeOf(
      "Short period",
      wnSp,
      zetaSp,
      "Etkin 2nd ed. §6.3. ω² = Zα Mq/V − Mα. Needs Cmα < 0 (static margin > 0).",
      derivs.longitudinallyStable,
    ),
    modeOf(
      "Phugoid",
      wnPh,
      zetaPh,
      "Lanchester: ω ≈ g√2 / V, ζ ≈ 1/(√2 L/D). Energy interchange of height and speed.",
      zetaPh > 0,
    ),
    modeOf(
      "Dutch roll",
      wnDr,
      zetaDr,
      "Lateral-directional oscillation. ω² ≈ Nβ + Yβ Nr / V. Needs Cnβ > 0.",
      derivs.directionallyStable,
    ),
    {
      name: "Roll subsidence",
      wn: Math.abs(L_p),
      fn: Math.abs(L_p) / (2 * Math.PI),
      zeta: L_p < 0 ? 1 : -1,
      period: 0,
      tHalf: tauRoll,
      stable: L_p < 0,
      note: `First-order. λ = Lp = ${L_p.toExponential(3)} /s. Time to half ${tauRoll.toFixed(2)} s. Clp < 0 is damping.`,
    },
    {
      name: "Spiral",
      wn: Math.abs(lamS),
      fn: Math.abs(lamS) / (2 * Math.PI),
      zeta: lamS < 0 ? 1 : -1,
      period: 0,
      tHalf: Math.abs(lamS) > 1e-6 ? Math.LN2 / Math.abs(lamS) : 0,
      stable: lamS <= 0,
      note: "Slow non-oscillatory. Sign(Clβ Cnr − Cnβ Clr) with Clp damping. Screening estimate (Etkin §6.4).",
    },
  ];

  notes.push(
    "Static derivatives: 4th-order Richardson on α and β (±1°, ±2°). Mixed marches shock-expansion strips.",
  );
  notes.push(
    "Rotary derivatives: local velocity V∞ − ω × r_cg on every panel (Etkin). Nondim. q̂ = q L / 2V, p̂,r̂ = (p,r) b / 2V.",
  );
  notes.push(
    "Linear 6DOF RK4 integrates α, q, θ, β, p, r, φ about trim. Gravity in the pitch plane. Not a CFD DES.",
  );
  notes.push(
    `Inertias from Mirtich tetrahedra. Ixx=${Ixx.toFixed(2)} Iyy=${Iyy.toFixed(2)} Izz=${Izz.toFixed(2)} kg·m².`,
  );

  const dt = 0.02;
  // 16 s covers several short-period cycles and a visible arc of the path. The loop is the linear model, not a panel solve.
  const nStep = 800;
  const alpha0 = (a0 + 2) * DEG;
  let alpha = alpha0;
  let q = 0;
  let gamma = params.gammaDeg * DEG;
  // α = θ − γ at release. Pitch is attitude, not incidence.
  let theta = alpha0 + gamma;
  let beta = 0;
  let p = 0;
  let r = 0;
  let phi = 0;
  let psi = 0;
  let VV = V;
  let h = Math.max(500, params.altKm * 1000);
  let x = 0;
  const samples: SixDofSample[] = [];
  const Re = 6371000;
  const rho0 = Math.max(atm.rho, 1e-6);
  const hRef = h;
  const Hscale = 7200;

  const body: RigidBody = { S, L, m, Ixx, Iyy, Izz, rho: (hh) => rho0 * Math.exp(-(hh - hRef) / Hscale), Re, g0 };
  const coeffs = (st: RigidState): RigidCoeffs => {
    const qh = (st.q * L) / (2 * st.VV);
    const ph = (st.p * bRef) / (2 * st.VV);
    const rh = (st.r * bRef) / (2 * st.VV);
    const dAlpha = st.alpha - a0 * DEG;
    return {
      CL: p0.cl + cla * dAlpha + clq * qh,
      CD: Math.max(0.002, p0.cd + cda * dAlpha),
      CY: cyb * st.beta + cyp * ph + cyr * rh,
      Cl: clb * st.beta + clp * ph + clr * rh,
      Cm: p0.cm + cma * dAlpha + cmq * qh,
      Cn: cnb * st.beta + cnp * ph + cnr * rh,
    };
  };

  for (let k = 0; k <= nStep; k++) {
    if (k % 8 === 0) {
      samples.push({
        t: k * dt,
        alphaDeg: alpha * RAD,
        betaDeg: beta * RAD,
        pDeg: p * RAD,
        qDeg: q * RAD,
        rDeg: r * RAD,
        phiDeg: phi * RAD,
        thetaDeg: theta * RAD,
        psiDeg: psi * RAD,
        V: VV,
        gammaDeg: gamma * RAD,
        hKm: h / 1000,
        xKm: x / 1000,
      });
    }
    const next = rk4Step({ alpha, q, theta, beta, p, r, phi, VV, gamma, h, x }, dt, coeffs, body);
    alpha = next.alpha;
    q = next.q;
    theta = next.theta;
    beta = next.beta;
    p = next.p;
    r = next.r;
    phi = next.phi;
    VV = next.VV;
    gamma = next.gamma;
    h = next.h;
    x = next.x;
    psi += dt * ((q * Math.sin(phi) + r * Math.cos(phi)) / Math.max(Math.cos(theta), 0.2));
    alpha = clamp(alpha, -20 * DEG, 25 * DEG);
    beta = clamp(beta, -20 * DEG, 20 * DEG);
    q = clamp(q, -2, 2);
    p = clamp(p, -3, 3);
    r = clamp(r, -2, 2);
    VV = clamp(VV, 80, 12000);
    h = clamp(h, 0, 95000);
    if (h <= 0) break;
  }

  notes.push(`RK4 ${nStep} steps × dt=${dt}s from α₀+2°. Position (h, x) is in the same RK4 step. Density follows ρ e^{−Δh/H}, H = 7.2 km. α̇ = q − γ̇ so incidence stays θ − γ.`);

  return {
    derivs,
    modes,
    samples,
    polar,
    atAlpha: p0,
    iterations: nStep,
    dt,
    notes,
    skipped: false,
    method: "linear-panel + Etkin modes + RK4",
  };
}
