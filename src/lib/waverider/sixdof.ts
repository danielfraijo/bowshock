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
): SixDofResult {
  const notes: string[] = [];
  const a0 = params.alphaDeg;
  const b0 = params.betaDeg;
  const p0 = panelAero(mesh, params, atm, a0, b0, sRef, lRef);
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

  const dA = 1.0;
  const pu = panelAero(mesh, params, atm, a0 + dA, b0, sRef, lRef);
  const pd = panelAero(mesh, params, atm, a0 - dA, b0, sRef, lRef);
  const pu2 = panelAero(mesh, params, atm, a0 + 2 * dA, b0, sRef, lRef);
  const pd2 = panelAero(mesh, params, atm, a0 - 2 * dA, b0, sRef, lRef);
  const pb = panelAero(mesh, params, atm, a0, b0 + dA, sRef, lRef);
  const pb2 = panelAero(mesh, params, atm, a0, b0 + 2 * dA, sRef, lRef);
  const pbm = panelAero(mesh, params, atm, a0, b0 - dA, sRef, lRef);
  const pbm2 = panelAero(mesh, params, atm, a0, b0 - 2 * dA, sRef, lRef);
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
  const qPair = panelAero(mesh, params, atm, a0, b0, sRef, lRef, { p: 0, q: qRate, r: 0 });
  const pPair = panelAero(mesh, params, atm, a0, b0, sRef, lRef, { p: pRate, q: 0, r: 0 });
  const rPair = panelAero(mesh, params, atm, a0, b0, sRef, lRef, { p: 0, q: 0, r: rRate });
  const cmq = (qPair.cm - p0.cm) / qHat;
  const clq = (qPair.cl - p0.cl) / qHat;
  const clp = (pPair.cll - p0.cll) / pHat;
  const cnp = (pPair.cn - p0.cn) / pHat;
  const cyp = (pPair.cy - p0.cy) / pHat;
  const cnr = (rPair.cn - p0.cn) / rHat;
  const clr = (rPair.cll - p0.cll) / rHat;
  const cyr = (rPair.cy - p0.cy) / rHat;

  const sm = cla !== 0 ? -cma / cla : 0;
  let trim = a0;
  let cmT = p0.cm;
  for (let i = 0; i < 4; i++) {
    const p = panelAero(mesh, params, atm, trim, b0, sRef, lRef);
    const pp = panelAero(mesh, params, atm, trim + 0.4, b0, sRef, lRef);
    const dcm = (pp.cm - p.cm) / 0.4;
    if (Math.abs(dcm) < 1e-6) break;
    trim -= p.cm / dcm;
    trim = clamp(trim, -8, 16);
    cmT = p.cm;
    if (Math.abs(cmT) < 0.002) break;
  }

  const polar: { a: number; cl: number; cd: number; cm: number; ld: number }[] = [];
  for (const a of [-4, -1, 2, 5, 8, 12]) {
    const p = panelAero(mesh, params, atm, a, 0, sRef, lRef);
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
    "Static derivatives: 4th-order Richardson on α and β (±1°, ±2°). Mixed panel (CBAERO-class).",
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
  const nStep = 400;
  const alpha0 = (a0 + 2) * DEG;
  let alpha = alpha0;
  let q = 0;
  let theta = alpha0;
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
    const qdyn = 0.5 * atm.rho * st.VV * st.VV;
    const Lift = qdyn * S * CL;
    const Drag = qdyn * S * CD;
    const Y = qdyn * S * CY;
    const gLoc = g0 * (Re / (Re + h)) ** 2;
    const aDot = st.q - Lift / (m * st.VV) + (gLoc * Math.cos(st.gamma)) / st.VV * Math.sin(st.alpha - (st.theta - st.gamma));
    const qDot = (qdyn * S * L * Cm) / Iyy;
    const thDot = st.q;
    const bDot = st.p * Math.sin(st.alpha) - st.r * Math.cos(st.alpha) + Y / (m * st.VV);
    const pDot = (qdyn * S * bRef * Cl) / Ixx;
    const rDot = (qdyn * S * bRef * Cn) / Izz;
    const phiDot = st.p + Math.tan(st.theta) * (st.q * Math.sin(st.phi) + st.r * Math.cos(st.phi));
    const vDot = -Drag / m - gLoc * Math.sin(st.gamma);
    const gDot = st.VV > 20 ? Lift / (m * st.VV) + (st.VV / (Re + h) - gLoc / st.VV) * Math.cos(st.gamma) : 0;
    return { aDot, qDot, thDot, bDot, pDot, rDot, phiDot, vDot, gDot };
  };

  for (let k = 0; k <= nStep; k++) {
    if (k % 4 === 0) {
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
    const s0 = { alpha, q, theta, beta, p, r, phi, VV, gamma };
    const k1 = deriv(s0);
    const k2 = deriv({
      alpha: alpha + 0.5 * dt * k1.aDot,
      q: q + 0.5 * dt * k1.qDot,
      theta: theta + 0.5 * dt * k1.thDot,
      beta: beta + 0.5 * dt * k1.bDot,
      p: p + 0.5 * dt * k1.pDot,
      r: r + 0.5 * dt * k1.rDot,
      phi: phi + 0.5 * dt * k1.phiDot,
      VV: VV + 0.5 * dt * k1.vDot,
      gamma: gamma + 0.5 * dt * k1.gDot,
    });
    const k3 = deriv({
      alpha: alpha + 0.5 * dt * k2.aDot,
      q: q + 0.5 * dt * k2.qDot,
      theta: theta + 0.5 * dt * k2.thDot,
      beta: beta + 0.5 * dt * k2.bDot,
      p: p + 0.5 * dt * k2.pDot,
      r: r + 0.5 * dt * k2.rDot,
      phi: phi + 0.5 * dt * k2.phiDot,
      VV: VV + 0.5 * dt * k2.vDot,
      gamma: gamma + 0.5 * dt * k2.gDot,
    });
    const k4 = deriv({
      alpha: alpha + dt * k3.aDot,
      q: q + dt * k3.qDot,
      theta: theta + dt * k3.thDot,
      beta: beta + dt * k3.bDot,
      p: p + dt * k3.pDot,
      r: r + dt * k3.rDot,
      phi: phi + dt * k3.phiDot,
      VV: VV + dt * k3.vDot,
      gamma: gamma + dt * k3.gDot,
    });
    alpha += (dt / 6) * (k1.aDot + 2 * k2.aDot + 2 * k3.aDot + k4.aDot);
    q += (dt / 6) * (k1.qDot + 2 * k2.qDot + 2 * k3.qDot + k4.qDot);
    theta += (dt / 6) * (k1.thDot + 2 * k2.thDot + 2 * k3.thDot + k4.thDot);
    beta += (dt / 6) * (k1.bDot + 2 * k2.bDot + 2 * k3.bDot + k4.bDot);
    p += (dt / 6) * (k1.pDot + 2 * k2.pDot + 2 * k3.pDot + k4.pDot);
    r += (dt / 6) * (k1.rDot + 2 * k2.rDot + 2 * k3.rDot + k4.rDot);
    phi += (dt / 6) * (k1.phiDot + 2 * k2.phiDot + 2 * k3.phiDot + k4.phiDot);
    VV += (dt / 6) * (k1.vDot + 2 * k2.vDot + 2 * k3.vDot + k4.vDot);
    gamma += (dt / 6) * (k1.gDot + 2 * k2.gDot + 2 * k3.gDot + k4.gDot);
    const dh = VV * Math.sin(gamma);
    const dx = (VV * Math.cos(gamma) * Re) / (Re + h);
    h += dt * dh;
    x += dt * dx;
    psi += dt * (q * Math.sin(phi) + r * Math.cos(phi)) / Math.max(Math.cos(theta), 0.2);
    alpha = clamp(alpha, -20 * DEG, 25 * DEG);
    beta = clamp(beta, -20 * DEG, 20 * DEG);
    q = clamp(q, -2, 2);
    p = clamp(p, -3, 3);
    r = clamp(r, -2, 2);
    VV = clamp(VV, 80, 12000);
    h = clamp(h, 0, 95000);
  }

  notes.push(`RK4 ${nStep} steps × dt=${dt}s from α₀+2° pulse. Classic 4th-order, local truncation O(dt⁵).`);

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
