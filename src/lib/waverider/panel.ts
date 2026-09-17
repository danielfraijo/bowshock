import type { AeroMethod, DesignParams, TriMesh, Vec3 } from "./types";
import { SURFACE_ID } from "./types";
import { effectiveLeRadius } from "./blunt";
import {
  DEG,
  betaFromThetaM,
  clamp,
  coneSurfaceCp,
  fayRiddell,
  flowRegime,
  hypersonicTripped,
  invPrandtlMeyer,
  knudsen,
  leesHeatFactor,
  newtonianCpMax,
  obliqueShock,
  postShockT,
  prandtlMeyer,
  rarefactionWeight,
  suttonGraves,
  sweepHeatFactor,
  tauberLaminar,
  tauberSutton,
  tauberTurbulent,
  twEquilibrium,
  vacuumCp,
  vanDriestII,
  vcross,
  vlen,
  viscousChi,
  viscousInteractionPressure,
  vsub,
} from "./math";
import type { Atmosphere } from "./atmosphere";

export interface PanelAero {
  cl: number;
  cd: number;
  cy: number;
  cm: number;
  cn: number;
  cll: number;
  ld: number;
  ca: number;
  cnA: number;
  cdWave: number;
  cdFric: number;
  cdBase: number;
  cpMax: number;
  cp: Float32Array;
  heat: Float32Array;
  twEq: Float32Array;
  qStag: number;
  qMax: number;
  qMeanWind: number;
  qRad: number;
  twMax: number;
  qFay: number;
  qSG: number;
  kn: number;
  chiBar: number;
  gammaEq: number;
  regime: "continuum" | "slip" | "transitional" | "free-molecular";
  pVisc: number;
  lRef: number;
  sRef: number;
  cg: Vec3;
  force: Vec3;
  moment: Vec3;
  method: AeroMethod;
  notes: string[];
}

const CONE_FAMILIES = new Set(["cone", "osculating", "elliptic", "viscopt", "star"]);

function get(mesh: TriMesh, i: number): Vec3 {
  return [mesh.positions[i * 3], mesh.positions[i * 3 + 1], mesh.positions[i * 3 + 2]];
}

function flowDir(alpha: number, beta: number): Vec3 {
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);
  const cb = Math.cos(beta);
  const sb = Math.sin(beta);
  return [ca * cb, sb, sa * cb];
}

function rotateForce(F: Vec3, alpha: number, beta: number) {
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);
  const cb = Math.cos(beta);
  const sb = Math.sin(beta);
  const x = F[0];
  const y = F[1];
  const z = F[2];
  const x1 = cb * x + sb * y;
  const y1 = -sb * x + cb * y;
  const drag = ca * x1 + sa * z;
  const lift = -sa * x1 + ca * z;
  return { lift, drag, side: y1 };
}

function tangentWedgeCp(M: number, theta: number, gamma: number, cpMax: number): number {
  if (theta <= 1e-5) return 0;
  const beta = betaFromThetaM(M, theta, gamma);
  if (!Number.isFinite(beta)) return cpMax * Math.sin(theta) ** 2;
  const sh = obliqueShock(M, beta, gamma);
  const q = 0.5 * gamma * M * M;
  return (sh.p2p1 - 1) / q;
}

function windwardCp(
  M: number,
  theta: number,
  gamma: number,
  cpMax: number,
  method: AeroMethod,
  useCone: boolean,
): { cp: number; attached: boolean } {
  const sin2 = Math.sin(theta) ** 2;
  if (method === "newtonian") return { cp: cpMax * sin2, attached: true };
  const beta = betaFromThetaM(M, theta, gamma);
  const attached = Number.isFinite(beta);
  if (!attached) return { cp: cpMax * sin2, attached: false };
  if (method === "tangent" || method === "mixed") {
    const exact = useCone ? coneSurfaceCp(M, theta, gamma) : tangentWedgeCp(M, theta, gamma, cpMax);
    return { cp: exact, attached: true };
  }
  return { cp: cpMax * sin2, attached: attached };
}

function leewardCp(M: number, theta: number, gamma: number): number {
  const nu = prandtlMeyer(M, gamma);
  const nu2 = nu + theta;
  const nuMax = prandtlMeyer(20, gamma);
  if (nu2 >= nuMax * 0.98) return vacuumCp(M, gamma);
  const Mm = invPrandtlMeyer(nu2, gamma);
  const iso2 = 1 + 0.5 * (gamma - 1) * Mm * Mm;
  const iso1 = 1 + 0.5 * (gamma - 1) * M * M;
  const p2 = iso2 ** (-gamma / (gamma - 1)) / iso1 ** (-gamma / (gamma - 1));
  const q = 0.5 * gamma * M * M;
  return clamp((p2 - 1) / q, vacuumCp(M, gamma), 0.2);
}

export interface RateState {
  p: number;
  q: number;
  r: number;
}

export function panelAero(
  mesh: TriMesh,
  params: DesignParams,
  atm: Atmosphere,
  alphaDeg: number,
  betaDeg: number,
  sRef: number,
  lRef: number,
  rates?: RateState,
): PanelAero {
  const notes: string[] = [];
  const M = Math.max(1.05, params.lockFlight ? params.mach : params.flightMach);
  const g = params.gamma;
  const method = params.aeroMethod;
  const alpha = alphaDeg * DEG;
  const beta = betaDeg * DEG;
  const vhat = flowDir(alpha, beta);
  const cpMax = newtonianCpMax(M, g);
  const nt = mesh.indices.length / 3;
  const cp = new Float32Array(nt);
  const heat = new Float32Array(nt);
  const twEq = new Float32Array(nt);
  const cgX = clamp(params.cgFrac, 0.2, 0.85) * params.length;
  const cg: Vec3 = [cgX, 0, 0];
  let Fx = 0;
  let Fy = 0;
  let Fz = 0;
  let Mx = 0;
  let My = 0;
  let Mz = 0;
  let qStag = 0;
  let qMax = 0;
  let qWindSum = 0;
  let aWind = 0;
  let FxBase = 0;
  let nDetached = 0;
  const Rn = Math.max(effectiveLeRadius(params), 0.0015 * params.length);
  const Tw = Math.max(200, params.twK);
  const h0 = atm.T * (1 + 0.5 * (g - 1) * M * M) * 1004.7;
  const hw = Tw * 1004.7;
  const recovStag = clamp(1 - hw / Math.max(h0, 1), 0.05, 0.95);
  const useCone = CONE_FAMILIES.has(params.family);
  const kn = knudsen(atm.T, atm.p, Math.max(params.length, 1e-6));
  const wRare = rarefactionWeight(kn);
  const chiBar = viscousChi(M, Math.max(atm.ReL * params.length, 1), Tw, atm.T);
  const pVisc = viscousInteractionPressure(chiBar);
  const gammaEq = postShockT(M, atm.T, g).gammaEq;
  const regime = flowRegime(kn);
  const sweep = Math.atan2(params.length, Math.max(params.span / 2, 1e-6));
  const sweepF = sweepHeatFactor(sweep);
  const qSG = suttonGraves(atm.rho, atm.V, Rn, recovStag);
  const qFay = fayRiddell(atm.rho, atm.V, Rn, atm.T, atm.p, Tw, g);
  const qStag0 = qFay > 0 ? qFay : qSG;
  let transArea = 0;

  for (let t = 0; t < nt; t++) {
    const a = get(mesh, mesh.indices[t * 3]);
    const b = get(mesh, mesh.indices[t * 3 + 1]);
    const c = get(mesh, mesh.indices[t * 3 + 2]);
    const ab = vsub(b, a);
    const ac = vsub(c, a);
    const nraw = vcross(ab, ac);
    const mag = vlen(nraw);
    if (mag < 1e-16) continue;
    const area = 0.5 * mag;
    const n: Vec3 = [nraw[0] / mag, nraw[1] / mag, nraw[2] / mag];
    const cx = (a[0] + b[0] + c[0]) / 3;
    const cy = (a[1] + b[1] + c[1]) / 3;
    const cz = (a[2] + b[2] + c[2]) / 3;
    const rx = cx - cg[0];
    const ry = cy - cg[1];
    const rz = cz - cg[2];
    let vhx = vhat[0];
    let vhy = vhat[1];
    let vhz = vhat[2];
    if (rates && atm.V > 1) {
      const vx = atm.V * vhat[0] - (rates.q * rz - rates.r * ry);
      const vy = atm.V * vhat[1] - (rates.r * rx - rates.p * rz);
      const vz = atm.V * vhat[2] - (rates.p * ry - rates.q * rx);
      const vm = Math.hypot(vx, vy, vz) || 1;
      vhx = vx / vm;
      vhy = vy / vm;
      vhz = vz / vm;
    }
    const ndv = n[0] * vhx + n[1] * vhy + n[2] * vhz;
    const sinth = clamp(-ndv, 0, 1);
    const surf = mesh.surfaces[t] ?? 0;
    let cpi: number;
    if (surf === SURFACE_ID.base || surf === SURFACE_ID.nozzle) {
      cpi = -1 / (M * M);
    } else if (ndv >= 0) {
      const theta = Math.asin(clamp(ndv, 0, 1));
      cpi = method === "newtonian" ? 0 : leewardCp(M, theta, g);
    } else {
      const theta = Math.asin(sinth);
      const w = windwardCp(M, theta, g, cpMax, method, useCone);
      cpi = w.cp;
      if (!w.attached) nDetached++;
    }
    if (ndv < 0) cpi *= pVisc;
    if (wRare > 1e-4) {
      const fm = 2 * sinth * sinth;
      cpi = (1 - wRare) * cpi + wRare * fm;
    }
    cp[t] = cpi;
    const dFx = -cpi * area * n[0];
    const dFy = -cpi * area * n[1];
    const dFz = -cpi * area * n[2];
    Fx += dFx;
    Fy += dFy;
    Fz += dFz;
    if (surf === SURFACE_ID.base || surf === SURFACE_ID.nozzle) FxBase += dFx;
    Mx += ry * dFz - rz * dFy;
    My += rz * dFx - rx * dFz;
    Mz += rx * dFy - ry * dFx;

    const xRun = Math.max(Math.hypot(cx, 0.12 * cy), Rn);
    const ReX = atm.ReL * xRun;
    const tripped = hypersonicTripped(ReX, M);
    const rRec = tripped ? 0.89 : 0.84;
    const hrec = atm.T * 1004.7 * (1 + rRec * 0.5 * (g - 1) * M * M);
    const recov = clamp(1 - hw / Math.max(hrec, 1), 0.05, 0.95);
    const pRatio = clamp(cpi / Math.max(cpMax, 1e-6), 0.02, 1);

    if (surf === SURFACE_ID.leading) {
      heat[t] = qStag0 * sweepF * leesHeatFactor(pRatio, 1, tripped);
    } else if (ndv < -0.02 && surf !== SURFACE_ID.base && surf !== SURFACE_ID.nozzle) {
      const qLam = tauberLaminar(atm.rho, atm.V, xRun, recov, sinth);
      const qTurb = tauberTurbulent(atm.rho, atm.V, xRun, recov, sinth);
      const qW0 = tripped ? Math.max(qLam, qTurb) : qLam;
      const qLees = qStag0 * leesHeatFactor(pRatio, xRun / Math.max(Rn, 1e-8), tripped);
      heat[t] = 0.55 * qW0 + 0.45 * qLees;
    } else if (surf !== SURFACE_ID.base && surf !== SURFACE_ID.nozzle) {
      const qLam = tauberLaminar(atm.rho, atm.V, xRun, recov, Math.max(sinth, 0.05));
      heat[t] = 0.22 * qLam;
    }
    if (heat[t] > 0) {
      twEq[t] = twEquilibrium(heat[t]);
      if (heat[t] > qMax) qMax = heat[t];
      if (surf === SURFACE_ID.leading || ndv < -0.02) {
        qWindSum += heat[t] * area;
        aWind += area;
        if (tripped && ndv < -0.02) transArea += area;
      }
    }
  }

  qStag = qStag0;
  const qRad = tauberSutton(atm.rho, atm.V, Rn);
  if (qStag > qMax) qMax = qStag;
  const twMax = twEquilibrium(qMax);

  const q = atm.q || 1;
  const S = Math.max(sRef, 1e-8);
  const L = Math.max(lRef, 1e-8);
  const wind = rotateForce([Fx * q, Fy * q, Fz * q], alpha, beta);
  const ReBody = atm.ReL * L;
  const Cf =
    ReBody < 5e5
      ? 1.328 / Math.sqrt(Math.max(ReBody, 100))
      : vanDriestII(ReBody, M, atm.T, Tw, g);
  const swet = aWind > 0 ? aWind * 1.35 : S * 2.4;
  const Dfric = Cf * q * swet;
  const lift = wind.lift;
  const drag = wind.drag + Dfric;
  const cl = lift / (q * S);
  const cd = drag / (q * S);
  const Dbase = Math.abs(FxBase) * q;
  const cdBase = Dbase / (q * S);
  const cdFric = Dfric / (q * S);
  const cdWave = Math.max(0, cd - cdFric - cdBase);
  const cy = wind.side / (q * S);
  const cm = My / (S * L);
  const cn = Mz / (S * L);
  const cll = Mx / (S * L);
  const ld = cd > 1e-10 ? cl / cd : 0;
  const ca = (Fx * q) / (q * S);
  const cnA = (Fz * q) / (q * S);

  if (method === "mixed") {
    notes.push(
      useCone
        ? "Mixed: tangent-cone (Taylor–Maccoll) windward if attached, Modified Newtonian if detached; Prandtl–Meyer leeward; Love base; van Driest II Cf."
        : "Mixed: tangent-wedge (θ-β-M) windward if attached, Modified Newtonian if detached; Prandtl–Meyer leeward; Love base; van Driest II / Blasius Cf.",
    );
  } else if (method === "tangent") {
    notes.push(useCone ? "Tangent-cone (Sims / Taylor–Maccoll) on windward panels." : "Tangent-wedge (exact oblique shock) on windward panels.");
  } else {
    notes.push("Modified Newtonian (Lees) windward, shadow Cp = 0 leeward.");
  }
  if (nDetached > 12) notes.push(`${nDetached} windward panels have a detached shock — Newtonian used there.`);
  if (params.lockFlight) notes.push("Flight Mach locked to design Mach.");
  if (pVisc > 1.04) notes.push(`Viscous interaction χ̄=${chiBar.toFixed(2)} raises windward p by ${(pVisc - 1).toFixed(3)} (Hayes–Probstein).`);
  if (wRare > 0.02) notes.push(`Rarefaction Kn=${kn.toExponential(2)} (${regime}): Cp bridged toward free-molecular.`);
  if (aWind > 0 && transArea > 0) {
    notes.push(
      `Transition: ${(100 * transArea / aWind).toFixed(0)}% of windward area tripped (Reshotko Re_θ/M_e > 180). Heating is 0.55 Tauber + 0.45 Lees.`,
    );
  }

  return {
    cl,
    cd,
    cy,
    cm,
    cn,
    cll,
    ld,
    ca,
    cnA,
    cdWave,
    cdFric,
    cdBase,
    cpMax,
    cp,
    heat,
    twEq,
    qStag,
    qMax,
    qMeanWind: aWind > 0 ? qWindSum / aWind : 0,
    qRad,
    twMax,
    qFay,
    qSG,
    kn,
    chiBar,
    gammaEq,
    regime,
    pVisc,
    lRef: L,
    sRef: S,
    cg,
    force: [Fx * q, Fy * q, Fz * q],
    moment: [Mx * q, My * q, Mz * q],
    method,
    notes,
  };
}

export function finiteStab(
  mesh: TriMesh,
  params: DesignParams,
  atm: Atmosphere,
  sRef: number,
  lRef: number,
  full = true,
) {
  const a0 = params.alphaDeg;
  const b0 = params.betaDeg;
  const d = 1.5;
  const p0 = panelAero(mesh, params, atm, a0, b0, sRef, lRef);
  if (!full) {
    return {
      cla: 0,
      cma: 0,
      cnb: 0,
      cyb: 0,
      clb: 0,
      staticMargin: 0,
      staticMarginPct: 0,
      trimAlpha: a0,
      trimCm: p0.cm,
      longitudinallyStable: false,
      directionallyStable: false,
      polar: [{ a: a0, cl: p0.cl, cd: p0.cd, cm: p0.cm, ld: p0.ld }],
      atAlpha: p0,
    };
  }
  const pu = panelAero(mesh, params, atm, a0 + d, b0, sRef, lRef);
  const pd = panelAero(mesh, params, atm, a0 - d, b0, sRef, lRef);
  const pb = panelAero(mesh, params, atm, a0, b0 + d, sRef, lRef);
  const da = 2 * d * DEG;
  const cla = (pu.cl - pd.cl) / da;
  const cma = (pu.cm - pd.cm) / da;
  const cnb = (pb.cn - p0.cn) / (d * DEG);
  const cyb = (pb.cy - p0.cy) / (d * DEG);
  const clb = (pb.cll - p0.cll) / (d * DEG);
  const sm = cla !== 0 ? -cma / cla : 0;
  let trim = a0;
  let cmT = p0.cm;
  for (let i = 0; i < 6; i++) {
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
  return {
    cla,
    cma,
    cnb,
    cyb,
    clb,
    staticMargin: sm,
    staticMarginPct: sm * 100,
    trimAlpha: trim,
    trimCm: cmT,
    longitudinallyStable: cma < 0 && cla > 0,
    directionallyStable: cnb > 0,
    polar,
    atAlpha: p0,
  };
}
