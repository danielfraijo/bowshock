import type { AeroMethod, DesignParams, TriMesh, Vec3 } from "./types";
import { SURFACE_ID } from "./types";
import { effectiveLeRadius } from "./blunt";
import {
  DEG,
  airMu,
  betaFromThetaM,
  clamp,
  coneSurfaceCp,
  dahlemBuckCp,
  eckertStanton,
  fayRiddell,
  flowRegime,
  hypersonicTripped,
  invPrandtlMeyer,
  isentropic,
  knudsen,
  leesHeatFactor,
  machFromPRatio,
  newtonBusemannCp,
  newtonianCpMax,
  obliqueShock,
  postShockT,
  prandtlMeyer,
  rarefactionWeight,
  stantonToCf,
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
  zobyHeatWcm2,
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
  stanton: Float32Array;
  machE: Float32Array;
  cf: Float32Array;
  impact: Float32Array;
  xCp: number;
  qMeanLower: number;
  qMeanUpper: number;
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
  if (method === "cbaero") {
    const db = dahlemBuckCp(theta, cpMax);
    const nb = newtonBusemannCp(theta, cpMax, gamma);
    if (!attached) return { cp: 0.65 * cpMax * sin2 + 0.35 * nb, attached: false };
    const exact = useCone ? coneSurfaceCp(M, theta, gamma) : tangentWedgeCp(M, theta, gamma, cpMax);
    return { cp: 0.55 * exact + 0.25 * db + 0.2 * nb, attached: true };
  }
  if (!attached) return { cp: cpMax * sin2, attached: false };
  if (method === "tangent" || method === "mixed") {
    const exact = useCone ? coneSurfaceCp(M, theta, gamma) : tangentWedgeCp(M, theta, gamma, cpMax);
    return { cp: exact, attached: true };
  }
  return { cp: cpMax * sin2, attached: attached };
}

function eckertStantonLocal(
  rhoE: number,
  Ue: number,
  Te: number,
  s: number,
  Me: number,
  gamma: number,
  turbulent: boolean,
  Tw: number,
): number {
  const mu = airMu(Te);
  const ReS = (Math.max(rhoE, 1e-12) * Math.max(Ue, 1) * Math.max(s, 1e-6)) / Math.max(mu, 1e-10);
  return eckertStanton(ReS, Te, Tw, Me, gamma, turbulent);
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

/** Planform LE x so running length is s = x − x_LE(y), not just x (outboard LE was too cold). */
function xLeadingOf(params: DesignParams, y: number): number {
  const L = params.length;
  const s = Math.max(params.span / 2, 1e-9);
  const yn = clamp(Math.abs(y) / s, 0, 1);
  if (params.family === "liftbody" || params.family === "star") return 0;
  if (params.family === "caret") return L * yn;
  if (params.family === "cone" || params.family === "busemann") {
    const nose = clamp(params.captureFrac, 0.04, 0.45);
    const blunt = 1 - Math.sqrt(Math.max(0, 1 - yn * yn));
    return L * (nose * blunt * 0.65 + yn * 0.35);
  }
  const planform = params.planform;
  if (planform === "rect") return 0;
  if (planform === "spatular") {
    const nose = clamp(params.captureFrac, 0.04, 0.45);
    const blunt = 1 - Math.sqrt(Math.max(0, 1 - yn * yn));
    return L * (nose * blunt * 0.65 + yn * 0.35);
  }
  if (planform === "double") {
    const kink = 0.42;
    if (yn < kink) return L * 0.1 * (yn / kink);
    return L * (0.1 + 0.9 * ((yn - kink) / (1 - kink)) ** 1.05);
  }
  const pow = planform === "delta" ? 1 : clamp(params.planformPower, 0.6, 2.4);
  return L * yn ** pow;
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
  const stanton = new Float32Array(nt);
  const machE = new Float32Array(nt);
  const cfArr = new Float32Array(nt);
  const impact = new Float32Array(nt);
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
  const isoInf = isentropic(M, g);
  const p0inf = atm.p * isoInf.ptP;
  const Rgas = 287.05287;
  let transArea = 0;
  let qLower = 0;
  let aLower = 0;
  let qUpper = 0;
  let aUpper = 0;

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
    let surf = mesh.surfaces[t] ?? 0;
    // Aft-facing TE thickness / nicked base: Love, not a phantom windward panel.
    if (n[0] > 0.72 && surf !== SURFACE_ID.leading && surf !== SURFACE_ID.inlet) {
      surf = SURFACE_ID.base;
    }
    const windward = ndv < 0 && surf !== SURFACE_ID.base && surf !== SURFACE_ID.nozzle;
    impact[t] = -ndv;
    let cpi: number;
    if (surf === SURFACE_ID.base || surf === SURFACE_ID.nozzle) {
      cpi = -1 / (M * M);
    } else if (!windward) {
      const theta = Math.asin(clamp(ndv, 0, 1));
      cpi = method === "newtonian" ? 0 : leewardCp(M, theta, g);
    } else {
      const theta = Math.asin(sinth);
      const w = windwardCp(M, theta, g, cpMax, method, useCone);
      cpi = w.cp;
      if (!w.attached) nDetached++;
    }
    if (windward) cpi *= pVisc;
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

    const xLE = xLeadingOf(params, cy);
    const xRun = Math.max(cx - xLE, Rn);
    const qDyn = 0.5 * g * M * M;
    const pe = Math.max(atm.p * (1 + cpi * qDyn), 0.02 * atm.p);
    let Te = atm.T;
    let rhoe = atm.rho;
    let Ue = atm.V;
    let Me = M;
    const thetaW = Math.asin(sinth);
    if (windward && thetaW > 1e-4 && surf !== SURFACE_ID.base && surf !== SURFACE_ID.nozzle) {
      const betaS = betaFromThetaM(M, thetaW, g);
      if (Number.isFinite(betaS)) {
        const sh = obliqueShock(M, betaS, g);
        Te = atm.T * sh.t2t1;
        rhoe = atm.rho * sh.r2r1;
        Me = Math.max(0.2, sh.M2);
        Ue = Me * Math.sqrt(Math.max(g * Rgas * Te, 1));
      } else {
        Te = atm.T * (1 + 0.5 * (g - 1) * M * M) / (1 + 0.5 * (g - 1));
        rhoe = pe / (Rgas * Math.max(Te, 1));
        Me = 0.4;
        Ue = Me * Math.sqrt(Math.max(g * Rgas * Te, 1));
      }
    } else {
      const p_p0 = clamp(pe / Math.max(p0inf, 1e-8), 1e-8, 0.999);
      Me = Math.max(0.15, machFromPRatio(p_p0, g));
      Te = atm.T * (1 + 0.5 * (g - 1) * M * M) / (1 + 0.5 * (g - 1) * Me * Me);
      rhoe = pe / (Rgas * Math.max(Te, 1));
      Ue = Me * Math.sqrt(Math.max(g * Rgas * Te, 1));
    }
    machE[t] = Me;
    const muE = airMu(Te);
    const ReX = (rhoe * Ue * xRun) / Math.max(muE, 1e-10);
    const tripped = hypersonicTripped(ReX, Me);
    const rRec = tripped ? 0.89 : Math.sqrt(0.71);
    const hrec = Te * 1004.7 * (1 + rRec * 0.5 * (g - 1) * Me * Me);
    const recov = clamp(1 - hw / Math.max(hrec, 1), 0.05, 0.95);
    const pRatio = clamp(cpi / Math.max(cpMax, 1e-6), 0.01, 1);
    const qZ = zobyHeatWcm2(rhoe, Ue, Te, Tw, xRun, Me, g, tripped);
    const St = eckertStantonLocal(rhoe, Ue, Te, xRun, Me, g, tripped, Tw);
    stanton[t] = St;
    cfArr[t] = stantonToCf(St, tripped);

    if (surf === SURFACE_ID.leading) {
      heat[t] = qStag0 * sweepF * leesHeatFactor(Math.max(pRatio, 0.25), 1, tripped);
    } else if (surf === SURFACE_ID.base || surf === SURFACE_ID.nozzle) {
      heat[t] = 0.08 * qStag0 * Math.pow(Math.max(pe / Math.max(atm.p * (1 + cpMax * qDyn), 1), 0.01), 0.8);
    } else if (windward) {
      const impactS = Math.max(sinth, 0.02);
      const qLam = tauberLaminar(rhoe, Ue, xRun, recov, impactS);
      const qTurb = tauberTurbulent(rhoe, Ue, xRun, recov, impactS);
      const qW0 = tripped ? Math.max(qLam, qTurb) : qLam;
      const qLees = qStag0 * leesHeatFactor(pRatio, xRun / Math.max(Rn, 1e-8), tripped);
      heat[t] = 0.5 * qZ + 0.3 * qW0 + 0.2 * qLees;
    } else {
      // Prandtl–Meyer edge is cold; keep a small residual so the lid isn't a hole.
      heat[t] = Math.max(0.18 * qZ, 0.015 * qStag0 * Math.sqrt(Math.max(pRatio, 0.02)));
    }
    if (heat[t] > 0) {
      twEq[t] = twEquilibrium(heat[t]);
      if (heat[t] > qMax) qMax = heat[t];
      if (surf === SURFACE_ID.leading || windward) {
        qWindSum += heat[t] * area;
        aWind += area;
        if (tripped && windward) transArea += area;
      }
    }
    if (surf === SURFACE_ID.lower) {
      qLower += heat[t] * area;
      aLower += area;
    } else if (surf === SURFACE_ID.upper) {
      qUpper += heat[t] * area;
      aUpper += area;
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
  const xCp = Math.abs(Fz) > 1e-14 ? clamp(cg[0] - My / Fz, 0, L) : cg[0];

  if (method === "cbaero") {
    notes.push(
      "CBAERO-class: Dahlem–Buck + Newton–Busemann + tangent-wedge/cone windward; Prandtl–Meyer leeward; Love base. Heating is Eckert–Zoby (NASA TP-1374) on the post-shock edge state, so +α lights the belly and −α the lid.",
    );
  } else if (method === "mixed") {
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
  if (nDetached > 12) notes.push(`${nDetached} windward panels have a detached shock — Newtonian/Dahlem–Buck used there.`);
  if (params.lockFlight) notes.push("Flight Mach locked to design Mach.");
  if (pVisc > 1.04) notes.push(`Viscous interaction χ̄=${chiBar.toFixed(2)} raises windward p by ${(pVisc - 1).toFixed(3)} (Hayes–Probstein).`);
  if (wRare > 0.02) notes.push(`Rarefaction Kn=${kn.toExponential(2)} (${regime}): Cp bridged toward free-molecular.`);
  if (aWind > 0 && transArea > 0) {
    notes.push(
      `Transition: ${(100 * transArea / aWind).toFixed(0)}% of windward area tripped (Reshotko Re_θ/M_e > 180). Heating is Zoby + Tauber + Lees on the post-shock BL.`,
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
    stanton,
    machE,
    cf: cfArr,
    impact,
    xCp,
    qMeanLower: aLower > 0 ? qLower / aLower : 0,
    qMeanUpper: aUpper > 0 ? qUpper / aUpper : 0,
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
