/**
 * Exact-gas anchors for the strip solver.
 * A flat wedge must recover the oblique shock. A wedge that then turns back
 * to the freestream must keep the residual pressure; tangent-wedge sets that
 * face to Cp = 0.
 */
import { atmosphere } from "./atmosphere";
import {
  betaFromThetaM,
  DEG,
  invPrandtlMeyer,
  isentropic,
  obliqueShock,
  prandtlMeyer,
} from "./math";
import { makeGrid, MeshBuilder, stitchGrid } from "./mesh";
import { panelAero } from "./panel";
import { DEFAULT_PARAMS, type AeroMethod, type DesignParams, type TriMesh } from "./types";

const M = 8;
const GAMMA = 1.4;
const SPAN = 0.2;

function design(method: AeroMethod): DesignParams {
  return {
    ...DEFAULT_PARAMS,
    family: "caret",
    mach: M,
    flightMach: M,
    lockFlight: true,
    alphaDeg: 0,
    betaDeg: 0,
    length: 1,
    aeroMethod: method,
  };
}

function extruded(slope: (x: number) => number, nx: number): TriMesh {
  const ny = 6;
  const upper = makeGrid("upper", nx, ny, (i, j) => [i / (nx - 1), (j / (ny - 1) - 0.5) * SPAN, 0]);
  const lower = makeGrid("lower", nx, ny, (i, j) => {
    const x = i / (nx - 1);
    return [x, (j / (ny - 1) - 0.5) * SPAN, slope(x)];
  });
  const builder = new MeshBuilder(1);
  stitchGrid(builder, upper, "upper", false, 0);
  stitchGrid(builder, lower, "lower", true, 1);
  return builder.finish();
}

function meanLower(mesh: TriMesh, cp: Float32Array, pick: (cx: number) => boolean): number {
  let sum = 0;
  let w = 0;
  const nt = mesh.indices.length / 3;
  const p = mesh.positions;
  for (let t = 0; t < nt; t++) {
    if (mesh.surfaces[t] !== 1) continue;
    const ia = mesh.indices[t * 3];
    const ib = mesh.indices[t * 3 + 1];
    const ic = mesh.indices[t * 3 + 2];
    const ax = p[ia * 3];
    const ay = p[ia * 3 + 1];
    const az = p[ia * 3 + 2];
    const bx = p[ib * 3];
    const by = p[ib * 3 + 1];
    const bz = p[ib * 3 + 2];
    const cx = p[ic * 3];
    const cy = p[ic * 3 + 1];
    const cz = p[ic * 3 + 2];
    const x = (ax + bx + cx) / 3;
    if (!pick(x)) continue;
    const nx = (by - ay) * (cz - az) - (bz - az) * (cy - ay);
    const ny = (bz - az) * (cx - ax) - (cz - az) * (bx - ax);
    const nz = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    const a = 0.5 * Math.hypot(nx, ny, nz);
    sum += cp[t] * a;
    w += a;
  }
  return w > 0 ? sum / w : 0;
}

export function obliqueCp(mach: number, thetaRad: number, gamma = GAMMA): number {
  const q = 0.5 * gamma * mach * mach;
  const beta = betaFromThetaM(mach, thetaRad, gamma);
  const shock = obliqueShock(mach, beta, gamma);
  return (shock.p2p1 - 1) / q;
}

/** Nose shock, then a Prandtl–Meyer turn down to thetaAft. */
export function analyticStrip(thetaNose: number, thetaAft: number, mach = M, gamma = GAMMA) {
  const q = 0.5 * gamma * mach * mach;
  const beta = betaFromThetaM(mach, thetaNose, gamma);
  const shock = obliqueShock(mach, beta, gamma);
  const noseCp = (shock.p2p1 - 1) / q;
  const turn = thetaNose - thetaAft;
  const M3 = invPrandtlMeyer(prandtlMeyer(shock.M2, gamma) + turn, gamma);
  const p3p2 = isentropic(M3, gamma).p / isentropic(shock.M2, gamma).p;
  const p3p1 = p3p2 * shock.p2p1;
  return { noseCp, aftCp: (p3p1 - 1) / q, p2p1: shock.p2p1, p3p1, M2: shock.M2, M3 };
}

export interface FlatWedge {
  cp: number;
  exactCp: number;
  ldWave: number;
  cot: number;
}

export function flatWedgeAnchor(): FlatWedge {
  const theta = 8 * DEG;
  const mesh = extruded((x) => -x * Math.tan(theta), 20);
  const aero = panelAero(mesh, design("mixed"), atmosphere(30, M, GAMMA), 0, 0, SPAN, 1);
  return {
    cp: meanLower(mesh, aero.cp, () => true),
    exactCp: obliqueCp(M, theta),
    ldWave: aero.cdWave > 1e-8 ? aero.cl / aero.cdWave : 0,
    cot: 1 / Math.tan(theta),
  };
}

export interface RampAnchor {
  noseCp: number;
  aftCp: number;
  tangentAft: number;
  exactNose: number;
  exactAft: number;
  exactP3: number;
}

/** 30° nose over the front half, then a face aligned with the freestream. */
export function expansionAnchor(): RampAnchor {
  const theta = 30 * DEG;
  const kink = 0.5;
  const zK = -kink * Math.tan(theta);
  const mesh = extruded((x) => (x <= kink ? -x * Math.tan(theta) : zK), 21);
  const atm = atmosphere(30, M, GAMMA);
  const mixed = panelAero(mesh, design("mixed"), atm, 0, 0, SPAN, 1);
  const tangent = panelAero(mesh, design("tangent"), atm, 0, 0, SPAN, 1);
  const exact = analyticStrip(theta, 0);
  return {
    noseCp: meanLower(mesh, mixed.cp, (x) => x < 0.4),
    aftCp: meanLower(mesh, mixed.cp, (x) => x > 0.62),
    tangentAft: meanLower(mesh, tangent.cp, (x) => x > 0.62),
    exactNose: exact.noseCp,
    exactAft: exact.aftCp,
    exactP3: exact.p3p1,
  };
}
