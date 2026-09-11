import type { AeroMethod, DesignParams, TriMesh, Vec3 } from "./types";
import { SURFACE_ID } from "./types";
import {
  DEG,
  betaFromThetaM,
  clamp,
  newtonianCpMax,
  obliqueShock,
  prandtlMeyer,
  skinCf,
  tauberSutton,
  vacuumCp,
  vcross,
  vlen,
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
  qStag: number;
  qMax: number;
  qMeanWind: number;
  qRad: number;
  lRef: number;
  sRef: number;
  cg: Vec3;
  force: Vec3;
  moment: Vec3;
  method: AeroMethod;
  notes: string[];
}

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

function tangentCp(M: number, theta: number, gamma: number, cpMax: number): number {
  if (theta <= 1e-5) return 0;
  const beta = betaFromThetaM(M, theta, gamma);
  if (!Number.isFinite(beta)) return cpMax * Math.sin(theta) ** 2;
  const sh = obliqueShock(M, beta, gamma);
  const q = 0.5 * gamma * M * M;
  return (sh.p2p1 - 1) / q;
}

function leewardCp(M: number, theta: number, gamma: number): number {
  const nu = prandtlMeyer(M, gamma);
  const nu2 = nu + theta;
  const nuMax = prandtlMeyer(20, gamma);
  if (nu2 >= nuMax * 0.98) return vacuumCp(M, gamma);
  const iso = 1;
  const p1 = iso;
  const Ttratio = 1 + 0.5 * (gamma - 1) * M * M;
  const pt = p1 * Ttratio ** (gamma / (gamma - 1));
  const M2 = Math.max(M, 1.01);
  const k = Math.sqrt((gamma + 1) / (gamma - 1));
  let Mm = M2;
  for (let i = 0; i < 18; i++) {
    const n = prandtlMeyer(Mm, gamma) - nu2;
    const d = (prandtlMeyer(Mm + 1e-4, gamma) - prandtlMeyer(Mm, gamma)) / 1e-4;
    Mm = clamp(Mm - n / (d || 1), 1.01, 25);
  }
  const t2 = 1 + 0.5 * (gamma - 1) * Mm * Mm;
  const p2 = pt / t2 ** (gamma / (gamma - 1));
  const q = 0.5 * gamma * M * M;
  return clamp((p2 - 1) / q, vacuumCp(M, gamma), 0.2);
}

export function panelAero(
  mesh: TriMesh,
  params: DesignParams,
  atm: Atmosphere,
  alphaDeg: number,
  betaDeg: number,
  sRef: number,
  lRef: number,
): PanelAero {
  const notes: string[] = [];
  const M = Math.max(1.05, params.lockFlight ? params.mach : params.flightMach);
  const g = params.gamma;
  const method = params.aeroMethod;
  const alpha = alphaDeg * DEG;
  const beta = betaDeg * DEG;
  const vhat = flowDir(alpha, beta);
  const cpMax = newtonianCpMax(M, g);
  const cpVac = vacuumCp(M, g);
  const nt = mesh.indices.length / 3;
  const cp = new Float32Array(nt);
  const heat = new Float32Array(nt);
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
  const Rn = Math.max(params.leRadius, 0.0015 * params.length);
  const Tw = Math.max(200, params.twK);
  const h0 = atm.T * (1 + 0.5 * (g - 1) * M * M) * 1004.7;
  const hw = Tw * 1004.7;
  const recov = clamp(1 - hw / Math.max(h0, 1), 0.05, 0.95);

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
    const ndv = n[0] * vhat[0] + n[1] * vhat[1] + n[2] * vhat[2];
    const sinth = clamp(-ndv, 0, 1);
    const surf = mesh.surfaces[t] ?? 0;
    let cpi: number;
    if (surf === SURFACE_ID.base || surf === SURFACE_ID.nozzle) {
      cpi = -1 / (M * M);
    } else if (ndv >= 0) {
      const theta = Math.asin(clamp(ndv, 0, 1));
      cpi = method === "newtonian" ? 0.15 * cpVac : leewardCp(M, theta, g);
    } else {
      const theta = Math.asin(sinth);
      if (method === "newtonian") cpi = cpMax * sinth * sinth;
      else {
        const tw = tangentCp(M, theta, g, cpMax);
        cpi = method === "tangent" ? tw : 0.65 * tw + 0.35 * cpMax * sinth * sinth;
      }
    }
    cp[t] = cpi;
    const dFx = -cpi * area * n[0];
    const dFy = -cpi * area * n[1];
    const dFz = -cpi * area * n[2];
    Fx += dFx;
    Fy += dFy;
    Fz += dFz;
    if (surf === SURFACE_ID.base || surf === SURFACE_ID.nozzle) FxBase += dFx;
    const rx = cx - cg[0];
    const ry = cy - cg[1];
    const rz = cz - cg[2];
    Mx += ry * dFz - rz * dFy;
    My += rz * dFx - rx * dFz;
    Mz += rx * dFy - ry * dFx;

    if (ndv < -0.02 && surf !== SURFACE_ID.base) {
      const x = Math.max(cx, Rn);
      const qLam = 1.83e-4 * Math.sqrt(atm.rho / x) * atm.V ** 3 * recov * Math.pow(sinth, 1.15);
      const qTurb = 3.7e-5 * atm.rho ** 0.8 * atm.V ** 3.37 * x ** -0.2 * recov * Math.pow(sinth, 1.6);
      const ReX = atm.ReL * x;
      const qW = ReX > 5e5 ? Math.max(qLam, qTurb) : qLam;
      heat[t] = qW / 1e4;
      qWindSum += heat[t] * area;
      aWind += area;
      if (heat[t] > qMax) qMax = heat[t];
    }
  }

  qStag = (1.83e-4 * Math.sqrt(atm.rho / Rn) * atm.V ** 3 * recov) / 1e4;
  const qRad = tauberSutton(atm.rho, atm.V, Rn);
  if (qStag > qMax) qMax = qStag;

  const q = atm.q || 1;
  const S = Math.max(sRef, 1e-8);
  const L = Math.max(lRef, 1e-8);
  const wind = rotateForce([Fx * q, Fy * q, Fz * q], alpha, beta);
  const Cf = skinCf(atm.ReL * L);
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

  if (method === "mixed") notes.push("Mixed: tangent-wedge windward, Prandtl–Meyer leeward, Newtonian blend, Love base, van-Driest Cf.");
  if (params.lockFlight) notes.push("Flight Mach locked to design Mach.");

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
    qStag,
    qMax,
    qMeanWind: aWind > 0 ? qWindSum / aWind : 0,
    qRad,
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
