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
  cpGas,
  fayRiddell,
  flowRegime,
  isentropic,
  knudsen,
  machFromPRatio,
  normalShock,
  postShockT,
  sweepHeatFactor,
  twEquilibrium,
  viscousChi,
  viscousInteractionPressure,
  R_AIR,
  fastInvPrandtlMeyer,
  invPrandtlMeyer,
  newtonianCpMax,
  obliqueShock,
  prandtlMeyer,
  suttonGraves,
  tauberSutton,
  vacuumCp,
  vanDriestII,
} from "./math";
import type { PanelAero, RateState } from "./panel";
import { effectiveLeRadius } from "./blunt";
import { bodyCf, surfaceHeat, xLeadingOf, type HeatCtx } from "./heating";
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
    const tg: Vec3 = [1, 0, 0];
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

/**
 * Signed flow turning at a facet: the angle between the local flow direction and the facet plane,
 * asin(−n·v̂). Positive compresses. For a planar facet in uniform flow this is exact (the oblique
 * shock turns the flow in the plane of v̂ and n), whatever the grid lines do. The earlier version
 * measured the angle to a grid-line tangent, which on a swept/skewed cell is not the flow
 * direction: it reported 15–20° on a 9° face (Cp ±30 %).
 */
function turning(v: Vec3, n: Vec3): number {
  return Math.asin(clamp(-(n[0] * v[0] + n[1] * v[1] + n[2] * v[2]), -1, 1));
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

/**
 * Isentropic turn through |dth| by Prandtl–Meyer. dth > 0 expands (ν grows),
 * dth < 0 compresses isentropically. ν is inverted with the Newton-refined
 * log table in math.ts (≤0.02 % in p) instead of the 240-point linear table
 * (up to 0.8 % near M = 1).
 */
function expand(_tab: GasTables, M: number, dth: number, gamma: number): { p2p1: number; M2: number } | null {
  const M1 = Math.max(M, 1.001);
  const nu2 = prandtlMeyer(M1, gamma) + dth;
  if (!(nu2 > 0)) return null;
  // Beyond ν(40) the flow is a near-vacuum: cap the marched Mach (it grew to 10⁹ at swept tips
  // and blew up the heating). Pressure there is already at the vacuum clamp.
  const M2 = Math.min(fastInvPrandtlMeyer(nu2, gamma), 40);
  const ratio = pOverP0(M2, gamma) / Math.max(1e-12, pOverP0(M1, gamma));
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
  const Rn = Math.max(effectiveLeRadius(params), 0.0015 * params.length);
  const Tw = Math.max(200, params.twK);
  const cpg = cpGas(g);
  const h0 = atm.T * (1 + 0.5 * (g - 1) * M * M) * cpg;
  const hw = Tw * cpg;
  const recovStag = clamp(1 - hw / Math.max(h0, 1), 0.05, 0.95);
  const T0 = atm.T * (1 + 0.5 * (g - 1) * M * M);
  const p0inf = atm.p * isentropic(M, g).ptP;
  const qSG = suttonGraves(atm.rho, atm.V, Rn, recovStag);
  const qFay = fayRiddell(atm.rho, atm.V, Rn, atm.T, atm.p, Tw, g);
  const qStag0 = qFay > 0 ? qFay : qSG;
  const sweepF = sweepHeatFactor(Math.atan2(params.length, Math.max(params.span / 2, 1e-6)));
  const heatCtx: HeatCtx = { pInf: atm.p, gamma: g, Tw, Rn, cpMax, qDyn: qInf, qStag0, sweepF };
  const twEq = new Float32Array(nt);
  const stanton = new Float32Array(nt);
  const machE = new Float32Array(nt);
  const cfArr = new Float32Array(nt);
  const impactArr = new Float32Array(nt);
  let qLower = 0;
  let aLower = 0;
  let qUpper = 0;
  let aUpper = 0;
  let aWet = 0;

  const accum = (face: Face, cpi: number, v: Vec3, Medge = NaN) => {
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
    const isBase = face.surf === SURFACE_ID.base || face.surf === SURFACE_ID.nozzle;
    if (!isBase && face.surf !== SURFACE_ID.inlet && face.surf !== SURFACE_ID.symmetry) aWet += face.area;
    const windward = ndv < -0.02 && !isBase;
    const sinth = clamp(-ndv, 0, 1);
    for (const tri of face.tris) impactArr[tri] = -ndv;
    // Edge state: adiabatic flow (T0 const), static p from Cp, Mach from the march
    // (or isentropically from p/p0 when the face has no marched Mach).
    const pe = Math.max(atm.p * (1 + cpi * qInf), 0.02 * atm.p);
    const Me = Number.isFinite(Medge) && Medge > 0.15 ? Medge : Math.max(0.15, machFromPRatio(clamp(pe / Math.max(p0inf, 1e-8), 1e-8, 0.999), g));
    const Te = T0 / (1 + 0.5 * (g - 1) * Me * Me);
    const rhoe = pe / (R_AIR * Math.max(Te, 1));
    const Ue = Me * Math.sqrt(Math.max(g * R_AIR * Te, 1));
    const xLE = xLeadingOf(params, face.c[1]);
    const xRun = Math.max(face.c[0] - xLE, Rn);
    const hr = surfaceHeat(heatCtx, {
      kind: face.surf === SURFACE_ID.leading ? "leading" : isBase ? "base" : windward ? "windward" : "leeward",
      cp: cpi,
      Me,
      Te,
      rhoe,
      Ue,
      pe,
      xRun,
      sinth,
    });
    const qh = hr.q;
    for (const tri of face.tris) {
      heat[tri] = qh;
      twEq[tri] = qh > 0 ? twEquilibrium(qh) : 0;
      stanton[tri] = hr.St;
      machE[tri] = Me;
      cfArr[tri] = hr.cf;
    }
    if (qh > qMax) qMax = qh;
    if (face.surf === SURFACE_ID.leading || windward) {
      qWindSum += qh * face.area;
      aWind += face.area;
    }
    if (face.surf === SURFACE_ID.lower) {
      qLower += qh * face.area;
      aLower += face.area;
    } else if (face.surf === SURFACE_ID.upper) {
      qUpper += qh * face.area;
      aUpper += face.area;
    }
  };

  const impact = (inc: number, vM: number): { cp: number; attached: boolean; Me?: number } => {
    if (inc <= 1e-5) {
      if (method === "newtonian" || inc > -1e-5) return { cp: 0, attached: true };
      const nu = prandtlMeyer(vM, g);
      const nu2 = nu + -inc;
      if (nu2 >= prandtlMeyer(20, g) * 0.98) return { cp: vac, attached: true };
      const Mm = invPrandtlMeyer(nu2, g);
      const p2 = pOverP0(Mm, g) / pOverP0(vM, g);
      return { cp: clamp((p2 - 1) / qInf, vac, 0.2), attached: true, Me: Mm };
    }
    if (method === "newtonian") return { cp: cpMax * Math.sin(inc) ** 2, attached: true };
    if (useCone) {
      const st = coneSurfaceState(M, inc, g);
      return { cp: st.cp, attached: true, Me: st.M };
    }
    const sh = sample2(tab, vM, inc);
    if (!sh) return { cp: cpMax * Math.sin(inc) ** 2, attached: false, Me: normalShock(M, g).M2 };
    return { cp: (sh.p2p1 - 1) / (0.5 * g * vM * vM), attached: true, Me: sh.M2 };
  };

  const walk = (strip: Face[]) => {
    let Mloc = M;
    let pRatio = 1;
    // Flow direction along the strip. It starts as the freestream and, after each facet, is the
    // projection of the previous direction onto that facet (the flow follows the surface).
    let vcur: Vec3 | null = null;
    let virgin = true;
    // "mixed" marches shock-expansion. "tangent" is CBAERO-class impact:
    // every panel is shocked or expanded from freestream, so an expanding
    // face keeps the freestream pressure of its own angle.
    let marching = method === "mixed";
    for (const face of strip) {
      const v = localVelocity(face, vhat, atm.V, rates, cg);
      const vf: Vec3 = marching && vcur ? vcur : v;
      const inc = turning(vf, face.n);
      let cpi: number;
      let Me = NaN;
      if (!marching) {
        const hit = impact(inc, M);
        cpi = hit.cp;
        Me = hit.Me ?? NaN;
        if (!hit.attached) nDetached++;
      } else if (face.surf === SURFACE_ID.base || face.surf === SURFACE_ID.nozzle) {
        cpi = -1 / (M * M);
      } else {
        const dth = inc;
        if (dth > 0.15 * DEG) {
          const fromFree = virgin;
          virgin = false;
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
              Me = normalShock(M, g).M2;
            } else {
              pRatio *= sh.p2p1;
              Mloc = sh.M2;
              cpi = (pRatio - 1) / qInf;
            }
          }
        } else {
          // Expansion (dth < 0) or a turn too small for a shock (|dth| ≤ 0.15°):
          // Prandtl–Meyer both ways. The old dead band dropped every turn under
          // 0.15° while still advancing `flow`, so a smoothly curved surface
          // (osculating, cone-derived lofts) never changed pressure along a strip.
          const ex = Math.abs(dth) > 1e-9 ? expand(tab, Mloc, -dth, g) : null;
          if (ex) {
            pRatio *= ex.p2p1;
            Mloc = ex.M2;
          }
          cpi = clamp((pRatio - 1) / qInf, vac, 2);
        }
        if (Math.abs(dth) > 0.15 * DEG) virgin = false;
        // follow the surface: remove the normal component of the flow direction
        const sn = -(vf[0] * face.n[0] + vf[1] * face.n[1] + vf[2] * face.n[2]);
        vcur = norm([vf[0] + sn * face.n[0], vf[1] + sn * face.n[1], vf[2] + sn * face.n[2]]);
        if (!Number.isFinite(Me)) Me = Mloc;
      }
      accum(face, cpi, v, Me);
    }
  };

  for (const strip of faces.strips) walk(strip);
  for (const face of faces.loose) {
    const v = localVelocity(face, vhat, atm.V, rates, cg);
    if (face.surf === SURFACE_ID.base || face.surf === SURFACE_ID.nozzle) {
      accum(face, -1 / (M * M), v);
      continue;
    }
    const inc = turning(v, face.n);
    const hit = impact(inc, M);
    if (!hit.attached) nDetached++;
    accum(face, hit.cp, v, hit.Me ?? NaN);
  }

  const qStag = qStag0;
  const qRad = tauberSutton(atm.rho, atm.V, Rn);
  if (qStag > qMax) qMax = qStag;
  const kn = knudsen(atm.T, atm.p, Math.max(params.length, 1e-6));
  const chiBar = viscousChi(M, Math.max(atm.ReL * params.length, 1), Tw, atm.T);
  const pVisc = viscousInteractionPressure(chiBar);
  const gammaEq = postShockT(M, atm.T, g).gammaEq;
  const regime = flowRegime(kn);
  const q = atm.q || 1;
  const S = Math.max(sRef, 1e-8);
  const Lref = Math.max(lRef, 1e-8);
  const wind = rotateForce([Fx * q, Fy * q, Fz * q], alpha, beta);
  const Cf = bodyCf(atm.ReL * Lref, M, atm.T, Tw, g);
  // Friction acts on the whole wetted surface (lee side included), not 1.35 × the windward area.
  const swet = aWet > 0 ? aWet : S * 2.4;
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
  notes.push("Heating: Eckert/Zoby + Tauber + Lees on the marched edge state (same model as the panel loop). Viscous interaction and rarefaction bridging are reported, not applied to Cp, in the marching method.");
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
    twEq,
    twMax: twEquilibrium(qMax),
    qFay,
    qSG,
    kn,
    chiBar,
    gammaEq,
    regime,
    pVisc,
    lRef: Lref,
    sRef: S,
    cg,
    force: [Fx * q, Fy * q, Fz * q],
    moment: [Mx * q, My * q, Mz * q],
    method: method as AeroMethod,
    notes,
    stanton,
    machE,
    cf: cfArr,
    impact: impactArr,
    xCp: Math.abs(Fz) > 1e-14 ? clamp(cg[0] - My / Fz, 0, Lref) : cg[0],
    qMeanLower: aLower > 0 ? qLower / aLower : 0,
    qMeanUpper: aUpper > 0 ? qUpper / aUpper : 0,
  };
}
