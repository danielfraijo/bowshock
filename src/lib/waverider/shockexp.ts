/**
 * Strip shock-expansion on the loft, tangent-wedge / Taylor–Maccoll at the
 * nose shock, Prandtl–Meyer on later turns. Caps (base, inlet lip, fins roots
 * that are not on a loft) stay on the local-inclination method.
 *
 * Flat caret strips recover the oblique-shock Cp and inviscid L/D = cot θ.
 * A pure tangent-wedge (CBAERO impact) recomputes each panel from freestream.
 * After the surface turns back, that misses the pressure the nose shock left behind.
 */
import type { Atmosphere } from "./atmosphere";
import {
  DEG,
  betaFromThetaM,
  clamp,
  coneSurfaceState,
  invPrandtlMeyer,
  newtonianCpMax,
  obliqueShock,
  prandtlMeyer,
  suttonGraves,
  tauberLaminar,
  tauberSutton,
  tauberTurbulent,
  vacuumCp,
  vanDriestII,
} from "./math";
import type { PanelAero, RateState } from "./panel";
import { SURFACE_ID, type AeroMethod, type DesignParams, type TriMesh, type Vec3 } from "./types";

const CONE_FAMILIES = new Set(["cone", "osculating", "elliptic", "viscopt", "star"]);

interface Face {
  area: number;
  n: Vec3;
  t: Vec3;
  c: Vec3;
  surf: number;
  grid: number;
  i: number;
  j: number;
  tris: number[];
}

interface FaceCache {
  strips: Face[][];
  loose: Face[];
}

const cache = new WeakMap<TriMesh, FaceCache>();

function decodeTag(tag: number): { g: number; i: number; j: number } | null {
  if (tag < 0) return null;
  return { j: tag & 4095, i: (tag >> 12) & 4095, g: tag >> 24 };
}

function get(mesh: TriMesh, i: number): Vec3 {
  return [mesh.positions[i * 3], mesh.positions[i * 3 + 1], mesh.positions[i * 3 + 2]];
}

function norm(a: Vec3): Vec3 {
  const m = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / m, a[1] / m, a[2] / m];
}

/**
 * Chord edge of the triangle, pointing downstream.
 * A quad is two triangles. On one of them the chord runs aft (dx > 0) and on
 * the other it runs forward, so the score uses |dx|. A signed dx lets the
 * diagonal win, and averaging that diagonal with the real chord reports ~20°
 * on an 8° wedge.
 */
function streamTangent(a: Vec3, b: Vec3, c: Vec3): Vec3 {
  const edges: Vec3[] = [
    [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
    [c[0] - b[0], c[1] - b[1], c[2] - b[2]],
    [a[0] - c[0], a[1] - c[1], a[2] - c[2]],
  ];
  let best = edges[0];
  let score = -Infinity;
  for (const e of edges) {
    const s = Math.abs(e[0]) - 2 * Math.abs(e[1]);
    if (s > score) {
      score = s;
      best = e;
    }
  }
  if (best[0] < 0) best = [-best[0], -best[1], -best[2]];
  return norm(best);
}

function buildFaces(mesh: TriMesh): FaceCache {
  const hit = cache.get(mesh);
  if (hit) return hit;
  const nt = mesh.indices.length / 3;
  const byTag = new Map<number, Face>();
  const loose: Face[] = [];
  for (let t = 0; t < nt; t++) {
    const ia = mesh.indices[t * 3];
    const ib = mesh.indices[t * 3 + 1];
    const ic = mesh.indices[t * 3 + 2];
    const a = get(mesh, ia);
    const b = get(mesh, ib);
    const c = get(mesh, ic);
    const abx = b[0] - a[0];
    const aby = b[1] - a[1];
    const abz = b[2] - a[2];
    const acx = c[0] - a[0];
    const acy = c[1] - a[1];
    const acz = c[2] - a[2];
    const nx = aby * acz - abz * acy;
    const ny = abz * acx - abx * acz;
    const nz = abx * acy - aby * acx;
    const mag = Math.hypot(nx, ny, nz);
    if (mag < 1e-16) continue;
    const area = 0.5 * mag;
    const n = norm([nx, ny, nz]);
    const ctr: Vec3 = [(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3];
    const tg = streamTangent(a, b, c);
    const surf = mesh.surfaces[t] ?? 0;
    const tag = mesh.tags ? mesh.tags[t] : -1;
    const dec = decodeTag(tag);
    if (!dec) {
      loose.push({ area, n, t: tg, c: ctr, surf, grid: -1, i: 0, j: 0, tris: [t] });
      continue;
    }
    const prev = byTag.get(tag);
    if (!prev) {
      byTag.set(tag, { area, n, t: tg, c: ctr, surf, grid: dec.g, i: dec.i, j: dec.j, tris: [t] });
    } else {
      const w = prev.area + area;
      prev.n = norm([
        prev.n[0] * prev.area + n[0] * area,
        prev.n[1] * prev.area + n[1] * area,
        prev.n[2] * prev.area + n[2] * area,
      ]);
      prev.t = norm([
        prev.t[0] * prev.area + tg[0] * area,
        prev.t[1] * prev.area + tg[1] * area,
        prev.t[2] * prev.area + tg[2] * area,
      ]);
      prev.c = [(prev.c[0] * prev.area + ctr[0] * area) / w, (prev.c[1] * prev.area + ctr[1] * area) / w, (prev.c[2] * prev.area + ctr[2] * area) / w];
      prev.area = w;
      prev.tris.push(t);
    }
  }
  const groups = new Map<string, Face[]>();
  for (const f of byTag.values()) {
    const k = `${f.grid}:${f.j}`;
    const list = groups.get(k);
    if (list) list.push(f);
    else groups.set(k, [f]);
  }
  const strips: Face[][] = [];
  for (const list of groups.values()) {
    list.sort((p, q) => p.i - q.i);
    strips.push(list);
  }
  const built = { strips, loose };
  cache.set(mesh, built);
  return built;
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
  const x1 = cb * F[0] + sb * F[1];
  const y1 = -sb * F[0] + cb * F[1];
  return { lift: -sa * x1 + ca * F[2], drag: ca * x1 + sa * F[2], side: y1 };
}

/** Signed angle from the freestream to the surface tangent. Positive compresses. */
function turning(tangent: Vec3, v: Vec3, n: Vec3): number {
  const tn = norm(tangent);
  const c = clamp(tn[0] * v[0] + tn[1] * v[1] + tn[2] * v[2], -1, 1);
  const ang = Math.acos(c);
  const wind = -(n[0] * v[0] + n[1] * v[1] + n[2] * v[2]);
  return wind >= 0 ? ang : -ang;
}

interface GasTables {
  gamma: number;
  mach: Float64Array;
  theta: Float64Array;
  p2: Float64Array;
  m2: Float64Array;
  nu: Float64Array;
  nuM: Float64Array;
}

const tables = new Map<number, GasTables>();

function gasTables(gamma: number): GasTables {
  const gKey = Math.round(gamma * 100);
  const hit = tables.get(gKey);
  if (hit) return hit;
  const nM = 81;
  const nT = 101;
  const mach = new Float64Array(nM);
  const theta = new Float64Array(nT);
  const p2 = new Float64Array(nM * nT);
  const m2 = new Float64Array(nM * nT);
  const nuM = new Float64Array(240);
  const nu = new Float64Array(240);
  for (let i = 0; i < nM; i++) mach[i] = 1.05 + (22 - 1.05) * (i / (nM - 1));
  for (let j = 0; j < nT; j++) theta[j] = ((50 * Math.PI) / 180) * (j / (nT - 1));
  for (let i = 0; i < nM; i++) {
    for (let j = 0; j < nT; j++) {
      const o = i * nT + j;
      if (theta[j] < 1e-6) {
        p2[o] = 1;
        m2[o] = mach[i];
        continue;
      }
      const beta = betaFromThetaM(mach[i], theta[j], gamma);
      if (!Number.isFinite(beta)) {
        p2[o] = -1;
        m2[o] = -1;
        continue;
      }
      const sh = obliqueShock(mach[i], beta, gamma);
      p2[o] = sh.p2p1;
      m2[o] = sh.M2;
    }
  }
  for (let i = 0; i < 240; i++) {
    nuM[i] = 1 + (28 * i) / 239;
    nu[i] = prandtlMeyer(nuM[i], gamma);
  }
  const tab = { gamma, mach, theta, p2, m2, nu, nuM };
  tables.set(gKey, tab);
  return tab;
}

function sample2(tab: GasTables, M: number, th: number): { p2p1: number; M2: number } | null {
  const { mach, theta, p2, m2 } = tab;
  if (th <= 1e-8) return { p2p1: 1, M2: M };
  if (M < mach[0] || th > theta[theta.length - 1]) return null;
  let i1 = 1;
  while (i1 < mach.length - 1 && mach[i1] < M) i1++;
  const i0 = i1 - 1;
  let j1 = 1;
  while (j1 < theta.length - 1 && theta[j1] < th) j1++;
  const j0 = j1 - 1;
  const tx = (M - mach[i0]) / (mach[i1] - mach[i0] || 1);
  const ty = (th - theta[j0]) / (theta[j1] - theta[j0] || 1);
  const nT = theta.length;
  const a = p2[i0 * nT + j0];
  const b = p2[i1 * nT + j0];
  const c = p2[i0 * nT + j1];
  const d = p2[i1 * nT + j1];
  if (a < 0 || b < 0 || c < 0 || d < 0) return null;
  const p =
    a * (1 - tx) * (1 - ty) + b * tx * (1 - ty) + c * (1 - tx) * ty + d * tx * ty;
  const a2 = m2[i0 * nT + j0];
  const b2 = m2[i1 * nT + j0];
  const c2 = m2[i0 * nT + j1];
  const d2 = m2[i1 * nT + j1];
  const mm = a2 * (1 - tx) * (1 - ty) + b2 * tx * (1 - ty) + c2 * (1 - tx) * ty + d2 * tx * ty;
  return { p2p1: p, M2: Math.max(mm, 1.01) };
}

function invNu(tab: GasTables, angle: number): number {
  const { nu, nuM } = tab;
  if (angle <= nu[0]) return nuM[0];
  if (angle >= nu[nu.length - 1]) return nuM[nuM.length - 1];
  let hi = 1;
  while (hi < nu.length - 1 && nu[hi] < angle) hi++;
  const lo = hi - 1;
  const u = (angle - nu[lo]) / (nu[hi] - nu[lo] || 1);
  return nuM[lo] * (1 - u) + nuM[hi] * u;
}

function pOverP0(M: number, gamma: number) {
  const t = 1 + 0.5 * (gamma - 1) * M * M;
  return t ** (-gamma / (gamma - 1));
}

function expand(tab: GasTables, M: number, dth: number, gamma: number): { p2p1: number; M2: number } {
  const nu1 = prandtlMeyer(Math.max(M, 1.001), gamma);
  const M2 = invNu(tab, nu1 + dth);
  const ratio = pOverP0(M2, gamma) / Math.max(1e-12, pOverP0(Math.max(M, 1.001), gamma));
  return { p2p1: ratio, M2 };
}

function paint(cp: Float32Array, face: Face, value: number) {
  for (const t of face.tris) cp[t] = value;
}

function localVelocity(face: Face, vhat: Vec3, atmV: number, rates: RateState | undefined, cg: Vec3): Vec3 {
  if (!rates || atmV <= 1) return vhat;
  const rx = face.c[0] - cg[0];
  const ry = face.c[1] - cg[1];
  const rz = face.c[2] - cg[2];
  const vx = atmV * vhat[0] - (rates.q * rz - rates.r * ry);
  const vy = atmV * vhat[1] - (rates.r * rx - rates.p * rz);
  const vz = atmV * vhat[2] - (rates.p * ry - rates.q * rx);
  return norm([vx, vy, vz]);
}

export function integrateShockExpansion(
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
  const qInf = 0.5 * g * M * M;
  const vac = vacuumCp(M, g);
  const tab = gasTables(g);
  const useCone = CONE_FAMILIES.has(params.family);
  const nt = mesh.indices.length / 3;
  const cp = new Float32Array(nt);
  const heat = new Float32Array(nt);
  const cgX = clamp(params.cgFrac, 0.2, 0.85) * params.length;
  const cg: Vec3 = [cgX, 0, 0];
  const faces = buildFaces(mesh);
  let Fx = 0;
  let Fy = 0;
  let Fz = 0;
  let Mx = 0;
  let My = 0;
  let Mz = 0;
  let qMax = 0;
  let qWindSum = 0;
  let aWind = 0;
  let FxBase = 0;
  let nDetached = 0;
  const Rn = Math.max(params.leRadius, 0.0015 * params.length);
  const Tw = Math.max(200, params.twK);
  const h0 = atm.T * (1 + 0.5 * (g - 1) * M * M) * 1004.7;
  const hw = Tw * 1004.7;
  const recovStag = clamp(1 - hw / Math.max(h0, 1), 0.05, 0.95);

  const accum = (face: Face, cpi: number, v: Vec3) => {
    paint(cp, face, cpi);
    const dFx = -cpi * face.area * face.n[0];
    const dFy = -cpi * face.area * face.n[1];
    const dFz = -cpi * face.area * face.n[2];
    Fx += dFx;
    Fy += dFy;
    Fz += dFz;
    if (face.surf === SURFACE_ID.base || face.surf === SURFACE_ID.nozzle) FxBase += dFx;
    const rx = face.c[0] - cg[0];
    const ry = face.c[1] - cg[1];
    const rz = face.c[2] - cg[2];
    Mx += ry * dFz - rz * dFy;
    My += rz * dFx - rx * dFz;
    Mz += rx * dFy - ry * dFx;
    const ndv = face.n[0] * v[0] + face.n[1] * v[1] + face.n[2] * v[2];
    if (ndv < -0.02 && face.surf !== SURFACE_ID.base) {
      const x = Math.max(face.c[0], Rn);
      const ReX = atm.ReL * x;
      const rRec = ReX > 5e5 ? 0.89 : 0.84;
      const hrec = atm.T * 1004.7 * (1 + rRec * 0.5 * (g - 1) * M * M);
      const recov = clamp(1 - hw / Math.max(hrec, 1), 0.05, 0.95);
      const sinth = clamp(-ndv, 0, 1);
      const qLam = tauberLaminar(atm.rho, atm.V, x, recov, sinth);
      const qTurb = tauberTurbulent(atm.rho, atm.V, x, recov, sinth);
      const qW = ReX > 5e5 ? Math.max(qLam, qTurb) : qLam;
      for (const tri of face.tris) heat[tri] = qW;
      qWindSum += qW * face.area;
      aWind += face.area;
      if (qW > qMax) qMax = qW;
    }
  };

  const impact = (inc: number, vM: number): { cp: number; attached: boolean } => {
    if (inc <= 1e-5) {
      if (method === "newtonian" || inc > -1e-5) return { cp: 0, attached: true };
      const nu = prandtlMeyer(vM, g);
      const nu2 = nu + -inc;
      if (nu2 >= prandtlMeyer(20, g) * 0.98) return { cp: vac, attached: true };
      const Mm = invPrandtlMeyer(nu2, g);
      const p2 = pOverP0(Mm, g) / pOverP0(vM, g);
      return { cp: clamp((p2 - 1) / qInf, vac, 0.2), attached: true };
    }
    if (method === "newtonian") return { cp: cpMax * Math.sin(inc) ** 2, attached: true };
    if (useCone) {
      const st = coneSurfaceState(M, inc, g);
      return { cp: st.cp, attached: true };
    }
    const sh = sample2(tab, vM, inc);
    if (!sh) return { cp: cpMax * Math.sin(inc) ** 2, attached: false };
    return { cp: (sh.p2p1 - 1) / (0.5 * g * vM * vM), attached: true };
  };

  const walk = (strip: Face[]) => {
    let Mloc = M;
    let pRatio = 1;
    let flow = 0;
    // "mixed" marches shock-expansion. "tangent" is CBAERO-class impact:
    // every panel is shocked or expanded from freestream, so an expanding
    // face keeps the freestream pressure of its own angle.
    let marching = method === "mixed";
    for (const face of strip) {
      const v = localVelocity(face, vhat, atm.V, rates, cg);
      const inc = turning(face.t, v, face.n);
      let cpi: number;
      if (!marching) {
        const hit = impact(inc, M);
        cpi = hit.cp;
        if (!hit.attached) nDetached++;
      } else if (face.surf === SURFACE_ID.base || face.surf === SURFACE_ID.nozzle) {
        cpi = -1 / (M * M);
      } else {
        const dth = inc - flow;
        if (dth > 0.15 * DEG) {
          const fromFree = Math.abs(flow) < 0.2 * DEG;
          if (useCone && fromFree) {
            const st = coneSurfaceState(M, Math.max(dth, 1e-4), g);
            pRatio = st.pRatio;
            Mloc = st.M;
            cpi = st.cp;
          } else {
            const sh = sample2(tab, Mloc, dth);
            if (!sh) {
              cpi = cpMax * Math.sin(Math.max(inc, 0)) ** 2;
              nDetached++;
              marching = false;
            } else {
              pRatio *= sh.p2p1;
              Mloc = sh.M2;
              cpi = (pRatio - 1) / qInf;
            }
          }
        } else if (dth < -0.15 * DEG) {
          const ex = expand(tab, Mloc, -dth, g);
          pRatio *= ex.p2p1;
          Mloc = ex.M2;
          cpi = clamp((pRatio - 1) / qInf, vac, 2);
        } else {
          cpi = (pRatio - 1) / qInf;
        }
        flow = inc;
      }
      accum(face, cpi, v);
    }
  };

  for (const strip of faces.strips) walk(strip);
  for (const face of faces.loose) {
    const v = localVelocity(face, vhat, atm.V, rates, cg);
    if (face.surf === SURFACE_ID.base || face.surf === SURFACE_ID.nozzle) {
      accum(face, -1 / (M * M), v);
      continue;
    }
    const inc = turning(face.t, v, face.n);
    const hit = impact(inc, M);
    if (!hit.attached) nDetached++;
    accum(face, hit.cp, v);
  }

  const qStag = suttonGraves(atm.rho, atm.V, Rn, recovStag);
  const qRad = tauberSutton(atm.rho, atm.V, Rn);
  if (qStag > qMax) qMax = qStag;
  const q = atm.q || 1;
  const S = Math.max(sRef, 1e-8);
  const Lref = Math.max(lRef, 1e-8);
  const wind = rotateForce([Fx * q, Fy * q, Fz * q], alpha, beta);
  const Cf = vanDriestII(atm.ReL * Lref, M, atm.T, Tw, g);
  const swet = aWind > 0 ? aWind * 1.35 : S * 2.4;
  const Dfric = Cf * q * swet;
  const lift = wind.lift;
  const drag = wind.drag + Dfric;
  const cl = lift / (q * S);
  const cd = drag / (q * S);
  const cdBase = (Math.abs(FxBase) * q) / (q * S);
  const cdFric = Dfric / (q * S);
  const cdWave = Math.max(0, cd - cdFric - cdBase);

  if (method === "newtonian") {
    notes.push("Modified Newtonian (Lees) windward, shadow Cp = 0 leeward.");
  } else if (method === "tangent") {
    notes.push(
      useCone
        ? "Tangent-cone: Taylor–Maccoll from freestream on every panel. Misses the pressure an upstream shock leaves on a face that turns back."
        : "Tangent-wedge: oblique shock from freestream on every panel. Misses the pressure an upstream shock leaves on a face that turns back.",
    );
  } else if (useCone) {
    notes.push(
      "Shock-expansion: Taylor–Maccoll cone at the nose of each strip, Prandtl–Meyer after that. Detached strips fall back to Modified Newtonian.",
    );
  } else {
    notes.push(
      "Shock-expansion: oblique shock at the nose of each strip, Prandtl–Meyer on later turns. Same gas model as the C kernel.",
    );
  }
  if (nDetached > 12) notes.push(`${nDetached} strips detached — Newtonian on those panels.`);
  if (params.lockFlight) notes.push("Flight Mach locked to design Mach.");

  return {
    cl,
    cd,
    cy: wind.side / (q * S),
    cm: My / (S * Lref),
    cn: Mz / (S * Lref),
    cll: Mx / (S * Lref),
    ld: cd > 1e-10 ? cl / cd : 0,
    ca: (Fx * q) / (q * S),
    cnA: (Fz * q) / (q * S),
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
    twEq: new Float32Array(nt),
    twMax: Tw,
    qFay: 0,
    qSG: qStag,
    kn: 0,
    chiBar: 0,
    gammaEq: g,
    regime: "continuum",
    pVisc: 0,
    lRef: Lref,
    sRef: S,
    cg,
    force: [Fx * q, Fy * q, Fz * q],
    moment: [Mx * q, My * q, Mz * q],
    method: method as AeroMethod,
    notes,
    stanton: new Float32Array(nt),
    machE: new Float32Array(nt),
    cf: new Float32Array(nt),
    impact: new Float32Array(nt),
    xCp: 0,
    qMeanLower: 0,
    qMeanUpper: 0,
  };
}
