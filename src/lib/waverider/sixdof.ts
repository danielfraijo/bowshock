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
  for (let i = 0; i < 4; i++) {
    const p = coef(trim, b0);
    const pp = coef(trim + 0.4, b0);
    const dcm = (pp.cm - p.cm) / 0.4;
    if (Math.abs(dcm) < 1e-6) break;
    trim -= p.cm / dcm;
    trim = clamp(trim, -8, 16);
    cmT = p.cm;
    if (Math.abs(cmT) < 0.002) break;
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

  const Y_b = (qBar * S * cyb) / m;
  const N_b = (qBar * S * bRef * cnb) / Izz;
  const N_r = (qBar * S * bRef * cnr * (bRef / (2 * V))) / Izz;
  const L_p = (qBar * S * bRef * clp * (bRef / (2 * V))) / Ixx;
  const YbV = Y_b / V;
  const wnDr = Math.sqrt(Math.max(0, N_b));
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
      "Lateral-directional oscillation. ω ≈ √Nβ. Needs Cnβ > 0.",
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
  // α = θ − γ at release. Pitch is attitude, not incidence.
  let theta = alpha0 + params.gammaDeg * DEG;
  let beta = 0;
  let p = 0;
  let r = 0;
  let phi = 0;
  let psi = 0;
  let VV = V;
  let gamma = params.gammaDeg * DEG;
  let h = Math.max(500, params.altKm * 1000);
  let x = 0;
  const samples: SixDofSample[] = [];
  const Re = 6371000;
  const rho0 = Math.max(atm.rho, 1e-6);
  const hRef = h;
  const Hscale = 7200;

  const deriv = (st: {
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
  }) => {
    const qh = (st.q * L) / (2 * st.VV);
    const ph = (st.p * bRef) / (2 * st.VV);
    const rh = (st.r * bRef) / (2 * st.VV);
    const dAlpha = st.alpha - a0 * DEG;
    const CL = p0.cl + cla * dAlpha + clq * qh;
    const CD = Math.max(0.002, p0.cd + cda * dAlpha);
    const CY = cyb * st.beta + cyp * ph + cyr * rh;
    const Cl = clb * st.beta + clp * ph + clr * rh;
    const Cm = p0.cm + cma * dAlpha + cmq * qh;
    const Cn = cnb * st.beta + cnp * ph + cnr * rh;
    const rho = rho0 * Math.exp(-(st.h - hRef) / Hscale);
    const qdyn = 0.5 * rho * st.VV * st.VV;
    const Lift = qdyn * S * CL;
    const Drag = qdyn * S * CD;
    const Y = qdyn * S * CY;
    const gLoc = g0 * (Re / (Re + st.h)) ** 2;
    // α̇ = q − γ̇. γ̇ is the same spherical point-mass used by the 3DOF
    // (lift, curvature V²/(R+h), inverse-square g). A sin(α − (θ−γ))
    // gravity term is zero exactly when the attitude is consistent, so it
    // dropped (g/V − V/r) and α walked off θ − γ.
    const curv = (st.VV / (Re + st.h) - gLoc / st.VV) * Math.cos(st.gamma);
    const aDot = st.q - Lift / (m * st.VV) - (st.VV > 20 ? curv : 0);
    const qDot = (qdyn * S * L * Cm) / Iyy;
    const thDot = st.q;
    const bDot = st.p * Math.sin(st.alpha) - st.r * Math.cos(st.alpha) + Y / (m * st.VV);
    const pDot = (qdyn * S * bRef * Cl) / Ixx;
    const rDot = (qdyn * S * bRef * Cn) / Izz;
    const phiDot = st.p + Math.tan(st.theta) * (st.q * Math.sin(st.phi) + st.r * Math.cos(st.phi));
    const vDot = -Drag / m - gLoc * Math.sin(st.gamma);
    const gDot = st.VV > 20 ? Lift / (m * st.VV) + (st.VV / (Re + st.h) - gLoc / st.VV) * Math.cos(st.gamma) : 0;
    const hDot = st.VV * Math.sin(st.gamma);
    const xDot = (st.VV * Math.cos(st.gamma) * Re) / (Re + st.h);
    return { aDot, qDot, thDot, bDot, pDot, rDot, phiDot, vDot, gDot, hDot, xDot };
  };

  type St = {
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
    x: number;
  };
  const add = (st: St, k: ReturnType<typeof deriv>, s: number): St => ({
    alpha: st.alpha + s * k.aDot,
    q: st.q + s * k.qDot,
    theta: st.theta + s * k.thDot,
    beta: st.beta + s * k.bDot,
    p: st.p + s * k.pDot,
    r: st.r + s * k.rDot,
    phi: st.phi + s * k.phiDot,
    VV: st.VV + s * k.vDot,
    gamma: st.gamma + s * k.gDot,
    h: st.h + s * k.hDot,
    x: st.x + s * k.xDot,
  });

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
    const s0: St = { alpha, q, theta, beta, p, r, phi, VV, gamma, h, x };
    const k1 = deriv(s0);
    const k2 = deriv(add(s0, k1, 0.5 * dt));
    const k3 = deriv(add(s0, k2, 0.5 * dt));
    const k4 = deriv(add(s0, k3, dt));
    const step = (a: number, b: number, c: number, d: number) => (dt / 6) * (a + 2 * b + 2 * c + d);
    alpha += step(k1.aDot, k2.aDot, k3.aDot, k4.aDot);
    q += step(k1.qDot, k2.qDot, k3.qDot, k4.qDot);
    theta += step(k1.thDot, k2.thDot, k3.thDot, k4.thDot);
    beta += step(k1.bDot, k2.bDot, k3.bDot, k4.bDot);
    p += step(k1.pDot, k2.pDot, k3.pDot, k4.pDot);
    r += step(k1.rDot, k2.rDot, k3.rDot, k4.rDot);
    phi += step(k1.phiDot, k2.phiDot, k3.phiDot, k4.phiDot);
    VV += step(k1.vDot, k2.vDot, k3.vDot, k4.vDot);
    gamma += step(k1.gDot, k2.gDot, k3.gDot, k4.gDot);
    h += step(k1.hDot, k2.hDot, k3.hDot, k4.hDot);
    x += step(k1.xDot, k2.xDot, k3.xDot, k4.xDot);
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
