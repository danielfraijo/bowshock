/**
 * Analytic benchmark bodies with exact (or independently known) inviscid answers, used to
 * quantify how far each aero method is from the truth across shapes, Mach and alpha.
 *
 *   wedge / flat plate at alpha  — oblique shock (windward) + Prandtl–Meyer (leeward), exact in 2-D
 *   double wedge / ramp          — two oblique shocks (compression) or shock + PM (expansion), exact
 *   cone                         — Taylor–Maccoll at alpha = 0 (validated against SP-3004, see validate-ref.ts)
 *   sphere-cone                  — nose cap Newtonian reference; far-field cone Cp → sharp-cone value
 *
 * Reference formulas are re-derived here from NACA 1135 / Anderson, not taken from panel.ts.
 */
import { atmosphere } from "./atmosphere";
import { MeshBuilder, makeGrid, orientOutward, stitchGrid } from "./mesh";
import { panelAero, type PanelAero } from "./panel";
import { betaFromThetaM, coneSurfaceStateExact, DEG, invPrandtlMeyer, isentropic, newtonianCpMax, obliqueShock, prandtlMeyer } from "./math";
import { DEFAULT_PARAMS, SURFACE_ID, type AeroMethod, type DesignParams, type TriMesh } from "./types";

const GAMMA = 1.4;

export interface BenchCase {
  shape: "wedge" | "plate" | "ramp" | "cone" | "sphere" | "spherecone";
  M: number;
  /** body angle in degrees: wedge half-angle / cone half-angle / first ramp angle */
  thetaDeg: number;
  /** second ramp angle (ramp only) */
  theta2Deg?: number;
  alphaDeg: number;
  altKm: number;
}

export interface BenchBody {
  mesh: TriMesh;
  sRef: number;
  lRef: number;
  /** lower (windward) / upper (leeward) / side panel ids */
  lowerMask: (cx: number, cz: number) => boolean;
}

const SPAN = 0.2;

export function benchParams(c: BenchCase, method: AeroMethod): DesignParams {
  const conic = c.shape === "cone" || c.shape === "spherecone" || c.shape === "sphere";
  return { ...DEFAULT_PARAMS, family: conic ? "cone" : "caret", mach: c.M, flightMach: c.M, lockFlight: true, alphaDeg: c.alphaDeg, betaDeg: 0, length: 1, aeroMethod: method, altKm: c.altKm, gamma: GAMMA };
}

/** Extruded planar lofts: upper/lower profile z(x), span SPAN, unit chord. */
export function extrudedBody(lower: (x: number) => number, upper: (x: number) => number, nx: number): BenchBody {
  const ny = 4;
  const upperG = makeGrid("upper", nx, ny, (i, j) => [i / (nx - 1), (j / (ny - 1) - 0.5) * SPAN, upper(i / (nx - 1))]);
  const lowerG = makeGrid("lower", nx, ny, (i, j) => [i / (nx - 1), (j / (ny - 1) - 0.5) * SPAN, lower(i / (nx - 1))]);
  const b = new MeshBuilder(1);
  stitchGrid(b, upperG, "upper", false, 0);
  stitchGrid(b, lowerG, "lower", true, 1);
  return { mesh: b.finish(), sRef: SPAN, lRef: 1, lowerMask: () => true };
}

/** Body of revolution about +x with radius profile r(s) sampled at ni stations; open at the base. */
export function revolvedBody(profile: (u: number) => [number, number], ni: number, nj = 36): BenchBody {
  const g = makeGrid("lower", ni, nj + 1, (i, j) => {
    const [x, r] = profile(i / (ni - 1));
    const ph = (2 * Math.PI * j) / nj;
    return [x, r * Math.cos(ph), r * Math.sin(ph)];
  });
  const b = new MeshBuilder(1);
  stitchGrid(b, g, "lower", false, 0);
  const mesh = b.finish();
  // outward normals: flip any triangle whose normal points toward the axis
  const p = mesh.positions;
  for (let t = 0; t < mesh.indices.length / 3; t++) {
    const a = mesh.indices[t * 3] * 3;
    const bb = mesh.indices[t * 3 + 1] * 3;
    const c = mesh.indices[t * 3 + 2] * 3;
    const nx = (p[bb + 1] - p[a + 1]) * (p[c + 2] - p[a + 2]) - (p[bb + 2] - p[a + 2]) * (p[c + 1] - p[a + 1]);
    const ny = (p[bb + 2] - p[a + 2]) * (p[c] - p[a]) - (p[bb] - p[a]) * (p[c + 2] - p[a + 2]);
    const nz = (p[bb] - p[a]) * (p[c + 1] - p[a + 1]) - (p[bb + 1] - p[a + 1]) * (p[c] - p[a]);
    const my = (p[a + 1] + p[bb + 1] + p[c + 1]) / 3;
    const mz = (p[a + 2] + p[bb + 2] + p[c + 2]) / 3;
    if (ny * my + nz * mz < 0) {
      const tmp = mesh.indices[t * 3 + 1];
      mesh.indices[t * 3 + 1] = mesh.indices[t * 3 + 2];
      mesh.indices[t * 3 + 2] = tmp;
    }
    void nx;
  }
  let rmax = 0;
  for (let i = 0; i < p.length; i += 3) rmax = Math.max(rmax, Math.hypot(p[i + 1], p[i + 2]));
  return { mesh, sRef: Math.PI * rmax * rmax, lRef: 1, lowerMask: () => true };
}

export function buildBench(c: BenchCase): BenchBody {
  const t1 = Math.tan(c.thetaDeg * DEG);
  switch (c.shape) {
    case "wedge":
    case "plate":
      // lower surface is the inclined face; upper face lies in the z=0 plane (parallel to the body axis)
      return extrudedBody((x) => -x * t1, () => 0, 24);
    case "ramp": {
      const t2 = Math.tan((c.theta2Deg ?? c.thetaDeg) * DEG);
      const xk = 0.5;
      return extrudedBody((x) => (x <= xk ? -x * t1 : -xk * t1 - (x - xk) * t2), () => 0, 41);
    }
    case "cone":
      return revolvedBody((u) => [u, u * t1], 24);
    case "sphere": {
      const R = 0.5;
      return revolvedBody((u) => {
        const ph = u * (Math.PI / 2);
        return [R * (1 - Math.cos(ph)), R * Math.sin(ph)];
      }, 40);
    }
    case "spherecone": {
      const Rn = 0.05;
      const th = c.thetaDeg * DEG;
      const phT = Math.PI / 2 - th; // polar angle of the tangent point on the nose cap
      const xT = Rn * (1 - Math.cos(phT));
      const rT = Rn * Math.sin(phT);
      const nCap = 14;
      const nCone = 30;
      return revolvedBody((u) => {
        const k = u * (nCap + nCone);
        if (k <= nCap) {
          const ph = (k / nCap) * phT;
          return [Rn * (1 - Math.cos(ph)), Rn * Math.sin(ph)];
        }
        const s = (k - nCap) / nCone;
        const xEnd = 1;
        const x = xT + s * (xEnd - xT);
        return [x, rT + (x - xT) * t1];
      }, nCap + nCone + 1);
    }
  }
}

/* ------------------------- exact references ------------------------- */

const qOf = (M: number) => 0.5 * GAMMA * M * M;

/** Cp of a compression face at flow-relative deflection `theta` (rad): exact oblique shock; NaN if detached. */
export function exactCompressionCp(M: number, theta: number): number {
  if (theta <= 1e-9) return 0;
  const beta = betaFromThetaM(M, theta, GAMMA);
  if (!Number.isFinite(beta)) return NaN;
  return (obliqueShock(M, beta, GAMMA).p2p1 - 1) / qOf(M);
}

/** Cp of an expansion of `theta` (rad) from the freestream (isentropic Prandtl–Meyer); vacuum-limited. */
export function exactExpansionCp(M: number, theta: number): number {
  const nu = prandtlMeyer(M, GAMMA) + theta;
  const nuMax = (Math.PI / 2) * (Math.sqrt((GAMMA + 1) / (GAMMA - 1)) - 1);
  if (nu >= nuMax) return vacuumCpExact(M);
  const M2 = invPrandtlMeyer(nu, GAMMA);
  const p2p1 = isentropic(M2, GAMMA).p / isentropic(M, GAMMA).p;
  return (p2p1 - 1) / qOf(M);
}

export function vacuumCpExact(M: number): number {
  return -1 / (0.5 * GAMMA * M * M);
}

export interface PlateExact {
  cpLower: number;
  cpUpper: number;
  cl: number;
  cdWave: number;
}

/** Wedge/flat plate (lower face inclined theta to the body axis, upper face on the axis) at alpha. */
export function exactWedgePlate(c: BenchCase): PlateExact {
  const a = c.alphaDeg * DEG;
  const th = c.thetaDeg * DEG;
  const cpL = exactCompressionCp(c.M, th + a);
  const upper = a; // upper face lies on the axis; positive alpha turns it away from the flow
  let cpU: number;
  if (upper > 1e-9) {
    cpU = exactExpansionCp(c.M, upper);
  } else if (upper < -1e-9) cpU = exactCompressionCp(c.M, -upper);
  else cpU = 0;
  // force from pressure on projected areas: lower face (area/chord = 1/cos th), normal tilted by th
  const sref = SPAN;
  const fx = (cpL * Math.tan(th) * SPAN) /* axial from lower inclined face */;
  const fzL = cpL * SPAN; // normal force pointing up (+z) from lower face, per unit projected area
  const fzU = -cpU * SPAN; // upper face on axis: force along -z... (pressure pushes down when Cp>0)
  const fz = fzL + fzU;
  const ca = fx; // axial (drag-direction) force coefficient*S
  const cl = (fz * Math.cos(a) - ca * Math.sin(a)) / sref;
  const cd = (ca * Math.cos(a) + fz * Math.sin(a)) / sref;
  return { cpLower: cpL, cpUpper: cpU, cl, cdWave: cd };
}

/** Double-wedge / ramp aft-face Cp relative to freestream (alpha = 0). */
export function exactRampAft(M: number, th1Deg: number, th2Deg: number): number {
  const th1 = th1Deg * DEG;
  const b1 = betaFromThetaM(M, th1, GAMMA);
  if (!Number.isFinite(b1)) return NaN;
  const s1 = obliqueShock(M, b1, GAMMA);
  const d = (th2Deg - th1Deg) * DEG;
  if (Math.abs(d) < 1e-9) return (s1.p2p1 - 1) / qOf(M);
  if (d > 0) {
    const b2 = betaFromThetaM(s1.M2, d, GAMMA);
    if (!Number.isFinite(b2)) return NaN;
    const s2 = obliqueShock(s1.M2, b2, GAMMA);
    return (s1.p2p1 * s2.p2p1 - 1) / qOf(M);
  }
  const nu = prandtlMeyer(s1.M2, GAMMA) - d;
  const M3 = invPrandtlMeyer(nu, GAMMA);
  const p3p2 = isentropic(M3, GAMMA).p / isentropic(s1.M2, GAMMA).p;
  return (s1.p2p1 * p3p2 - 1) / qOf(M);
}

/* ------------------------- evaluation ------------------------- */

export interface BenchResult {
  c: BenchCase;
  method: AeroMethod;
  aero: PanelAero;
  cpLower: number;
  cpUpper: number;
  cpSide: number;
  cpAft: number;
}

function areaWeighted(mesh: TriMesh, cp: Float32Array, pick: (cx: number, cy: number, cz: number, nx: number, ny: number, nz: number, surf: number) => boolean): number {
  let s = 0;
  let w = 0;
  const p = mesh.positions;
  for (let t = 0; t < mesh.indices.length / 3; t++) {
    const a = mesh.indices[t * 3] * 3;
    const b = mesh.indices[t * 3 + 1] * 3;
    const c = mesh.indices[t * 3 + 2] * 3;
    const ux = p[b] - p[a];
    const uy = p[b + 1] - p[a + 1];
    const uz = p[b + 2] - p[a + 2];
    const vx = p[c] - p[a];
    const vy = p[c + 1] - p[a + 1];
    const vz = p[c + 2] - p[a + 2];
    const nx = uy * vz - uz * vy;
    const ny = uz * vx - ux * vz;
    const nz = ux * vy - uy * vx;
    const m = Math.hypot(nx, ny, nz);
    if (m < 1e-18) continue;
    const cx = (p[a] + p[b] + p[c]) / 3;
    const cy = (p[a + 1] + p[b + 1] + p[c + 1]) / 3;
    const cz = (p[a + 2] + p[b + 2] + p[c + 2]) / 3;
    if (!pick(cx, cy, cz, nx / m, ny / m, nz / m, mesh.surfaces[t])) continue;
    s += cp[t] * 0.5 * m;
    w += 0.5 * m;
  }
  return w > 0 ? s / w : NaN;
}

export function runBench(c: BenchCase, method: AeroMethod, body?: BenchBody): BenchResult {
  const bd = body ?? buildBench(c);
  const params = benchParams(c, method);
  const atm = atmosphere(c.altKm, c.M, GAMMA);
  const aero = panelAero(bd.mesh, params, atm, c.alphaDeg, 0, bd.sRef, bd.lRef);
  const cp = aero.cp;
  const lower = areaWeighted(bd.mesh, cp, (_x, _y, _z, _nx, _ny, nz, s) => s === SURFACE_ID.lower && nz < -0.5);
  const upper = areaWeighted(bd.mesh, cp, (_x, _y, _z, _nx, _ny, nz, s) => s === SURFACE_ID.upper && nz > 0.5);
  const side = areaWeighted(bd.mesh, cp, (x) => x > 0.15);
  const aft = areaWeighted(bd.mesh, cp, (x, _y, _z, _nx, _ny, nz, s) => s === SURFACE_ID.lower && x > 0.62 && nz < -0.5);
  return { c, method, aero, cpLower: lower, cpUpper: upper, cpSide: side, cpAft: aft };
}

/** Sharp cone at alpha = 0: exact Taylor–Maccoll surface Cp. */
export function exactConeCp(M: number, thetaDeg: number): number {
  return coneSurfaceStateExact(M, thetaDeg * DEG, GAMMA).cp;
}

export function newtonianCone(M: number, thetaDeg: number): number {
  return newtonianCpMax(M, GAMMA) * Math.sin(thetaDeg * DEG) ** 2;
}
