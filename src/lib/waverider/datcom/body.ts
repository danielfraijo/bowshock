/**
 * Body-of-revolution aerodynamics.
 *
 * Axial force (all in coefficient form on Sref):
 *  - wave drag, M >= 1.2: tangent-cone pressure on forward-facing segments (Taylor–Maccoll via
 *    ../math.ts, Sims NASA SP-3004), Prandtl–Meyer expansion on rearward-facing segments bounded
 *    by the base pressure (flow separation limit), modified-Newtonian fallback when a cone
 *    shock detaches (Lees 1955). D/(q Sref) = (1/Sref) * sum Cp_i * dA_i with dA_i = pi (r1^2 - r0^2).
 *  - wave drag, M < 1.2: no wave drag below the body critical Mach, smooth rise to the M=1.2
 *    tangent-cone value (engineering interpolation, LOW confidence; Mcr = 1 - 0.6/sqrt(fineness) is
 *    an engineering heuristic, not a published formula).
 *  - skin friction: reference-temperature Cf (flow.ts) x wetted area, with Hoerner/Raymer fuselage
 *    form factor FF = 1 + 60/f^3 + 0.0025 f below M~0.9, faded out by M=1.2.
 *  - base drag: Cpb = -(0.12 + 0.13 M^2) (M<=1; Hoerner, Fluid-Dynamic Drag ch.3 trend),
 *    -0.25 for 1<M<2, -1/M^2 for M>=2 (classical engineering approximation).
 *
 * Normal force / moment:
 *  - slender-body potential term CN = (2 S_b/Sref) sin(a) cos(a) acting at x_cp = x_b - Vol/S_b
 *    (Munk 1924 / Allen & Perkins NACA Rep. 1048),
 *  - viscous crossflow term eta * Cdc(Mc) * (S_p/Sref) * sin^2(a) acting at the planform
 *    centroid (Allen & Perkins, NACA Rep. 1048, 1951),
 *  - hypersonic: numerically integrated modified-Newtonian impact pressure on the actual profile,
 *    blended in between M=3 and M=8 in 'auto' mode.
 * Dynamic: slender-body (apparent mass) pitch damping.
 */
import type { BodyMethod } from "./types.ts";
import type { BodyIntegrals, BodyProfile } from "./geometry.ts";
import type { CfOptions, Flow } from "./flow.ts";
import { skinFrictionCf } from "./flow.ts";
import { cpMaxNewton, cpSurface2D } from "./gasdyn.ts";
import { coneSurfaceCp } from "../math.ts";
import { DEG, clamp, interp1, smoothstep } from "./util.ts";

export interface BodyCtx {
  prof: BodyProfile;
  ints: BodyIntegrals;
  sRef: number;
  lRef: number;
  latRef: number;
  xcg: number;
  flow: Flow;
  method: BodyMethod;
  cf: CfOptions;
  exitDiameter?: number;
  baseDrag?: boolean;
}

/** base pressure coefficient (see header) */
export function basePressureCp(M: number): number {
  if (M <= 1) return -(0.12 + 0.13 * M * M);
  if (M < 2) return -0.25;
  return -1 / (M * M);
}

const CONE_ANGLES = [0, 1, 2, 3, 4, 5, 6, 8, 10, 12.5, 15, 17.5, 20, 25, 30, 35, 40, 50, 60, 70, 90];
const coneTables = new Map<string, number[]>();

/** Tangent-cone Cp for a forward-facing surface of local slope delta [rad] (table-interpolated TM). */
function cpTangentCone(M: number, delta: number, g: number): number {
  const key = `${M.toFixed(3)}:${g}`;
  let tab = coneTables.get(key);
  if (!tab) {
    tab = CONE_ANGLES.map((d) => (d === 0 ? 0 : coneSurfaceCp(M, d * DEG, g)));
    // enforce monotone non-decreasing (TM solver noise near detachment)
    for (let i = 1; i < tab.length; i++) if (tab[i] < tab[i - 1]) tab[i] = tab[i - 1];
    coneTables.set(key, tab);
  }
  return interp1(CONE_ANGLES, tab, Math.abs(delta) / DEG);
}

/** Local-surface Cp for a body segment of slope delta (rad), supersonic. */
function cpSegment(M: number, delta: number, g: number): number {
  if (delta >= 0) return cpTangentCone(M, delta, g);
  const cpb = basePressureCp(M);
  const cp = cpSurface2D(M, delta, g).cp; // Prandtl-Meyer expansion
  return Math.max(cp, cpb);
}

export function noseWaveDragSupersonic(ctx: BodyCtx, M: number): number {
  const { prof, sRef } = ctx;
  const g = ctx.flow.gamma;
  let sum = 0;
  for (let i = 0; i < prof.x.length - 1; i++) {
    const dx = prof.x[i + 1] - prof.x[i];
    const dr = prof.r[i + 1] - prof.r[i];
    if (dx <= 1e-12) {
      // vertical step (e.g. blunt shoulder): full stagnation pressure on the annulus
      if (dr > 0) sum += cpMaxNewton(M, g) * Math.PI * (prof.r[i + 1] ** 2 - prof.r[i] ** 2);
      continue;
    }
    const delta = Math.atan2(dr, dx);
    const dA = Math.PI * (prof.r[i + 1] ** 2 - prof.r[i] ** 2);
    sum += cpSegment(M, delta, g) * dA;
  }
  return sum / sRef;
}

function etaFinite(fineness: number): number {
  // finite-cylinder drag ratio vs crossflow cylinder (Allen & Perkins use Hoerner's data).
  // Recalled approximate values, +-0.05, NOT verified against the original figure.
  return interp1([1, 2, 5, 10, 20, 40], [0.6, 0.64, 0.74, 0.82, 0.91, 0.98], fineness);
}
/** Crossflow drag coefficient of a circular cylinder vs crossflow Mach (approximate trend, not verified data). */
export function crossflowCd(Mc: number): number {
  return interp1([0, 0.4, 1.0, 2.0, 5.0, 10], [1.2, 1.2, 1.45, 1.4, 1.3, 1.25], Mc);
}

export interface BodyAxial {
  CAwave: number;
  CAfric: number;
  CAbase: number;
  CA0: number;
  tag: string;
  notes: string[];
  cfValue: number;
}

export function bodyAxial(ctx: BodyCtx): BodyAxial {
  const { prof, ints, sRef, flow } = ctx;
  const M = flow.mach;
  const notes: string[] = [];
  const f = prof.length / Math.max(2 * prof.rMax, 1e-9);
  // wave
  let CAwave = 0;
  let tag = "";
  if (M >= 1.2) {
    CAwave = noseWaveDragSupersonic(ctx, M);
    tag = "tangent-cone/Prandtl-Meyer wave drag";
  } else {
    const w12 = noseWaveDragSupersonic(ctx, 1.2);
    const mcr = clamp(1 - 0.6 / Math.sqrt(Math.max(f, 1)), 0.5, 0.95);
    CAwave = w12 * smoothstep(mcr, 1.2, M);
    if (M > mcr) {
      tag = "subsonic/transonic wave-drag interpolation";
      notes.push(`Mach ${mcr.toFixed(2)}-1.20 body wave drag is an interpolation to the M=1.2 tangent-cone value (low confidence)`);
    } else tag = "subsonic (no wave drag)";
  }
  // friction
  const cf = skinFrictionCf(flow, Math.max(prof.length, 1e-6), ctx.cf);
  const ff = 1 + 60 / Math.pow(Math.max(f, 1), 3) + 0.0025 * f;
  const ffEff = 1 + (ff - 1) * (1 - smoothstep(0.9, 1.2, M));
  const CAfric = (cf * ffEff * ints.wetted) / sRef;
  // base
  let abEff = 0;
  if (ctx.exitDiameter !== undefined) {
    abEff = Math.max(ints.baseArea - (Math.PI * ctx.exitDiameter * ctx.exitDiameter) / 4, 0);
  } else if (ctx.baseDrag) abEff = ints.baseArea;
  const CAbase = (-basePressureCp(M) * abEff) / sRef;
  return { CAwave, CAfric, CAbase, CA0: CAwave + CAfric + CAbase, tag, notes, cfValue: cf };
}

export interface BodyNormal {
  /** normal force coefficient along the crossflow direction (>=0 for alphaT>=0) */
  N: number;
  /** first moment of the normal force about the nose: sum(F x) / (q Sref)  [m] */
  Nx: number;
  /** axial force increment due to alpha (added to CA0) */
  dCA: number;
  tag: string;
  notes: string[];
}

function newtonBody(ctx: BodyCtx, alphaT: number): { N: number; Nx: number; CA: number } {
  const { prof, sRef } = ctx;
  const g = ctx.flow.gamma;
  const cpm = cpMaxNewton(ctx.flow.mach, g);
  const sa = Math.sin(alphaT);
  const ca = Math.cos(alphaT);
  const NW = 48;
  let N = 0;
  let Nx = 0;
  let CA = 0;
  for (let i = 0; i < prof.x.length - 1; i++) {
    const dx = prof.x[i + 1] - prof.x[i];
    const dr = prof.r[i + 1] - prof.r[i];
    if (dx <= 1e-12) continue;
    const delta = Math.atan2(dr, dx);
    const rm = 0.5 * (prof.r[i] + prof.r[i + 1]);
    const dl = Math.hypot(dx, dr);
    const sd = Math.sin(delta);
    const cd = Math.cos(delta);
    const xm = 0.5 * (prof.x[i] + prof.x[i + 1]);
    let fc = 0;
    let fx = 0;
    for (let k = 0; k < NW; k++) {
      const om = ((k + 0.5) * Math.PI) / NW;
      const sn = sd * ca + sa * cd * Math.cos(om);
      if (sn > 0) {
        const cp = cpm * sn * sn;
        fc += cp * cd * Math.cos(om);
        fx += cp * sd;
      }
    }
    const dom = (2 * Math.PI) / NW; // both halves
    const area = rm * dl * dom;
    N += fc * area;
    Nx += fc * area * xm;
    CA += fx * area;
  }
  return { N: N / sRef, Nx: Nx / sRef, CA: CA / sRef };
}

export function newtonWeight(ctx: BodyCtx): number {
  const M = ctx.flow.mach;
  if (ctx.method === "slender") return 0;
  if (ctx.method === "newtonian") return M > 1.2 ? 1 : 0;
  return smoothstep(3, 8, M);
}

export function bodyNormal(ctx: BodyCtx, alphaT: number): BodyNormal {
  const { prof, ints, sRef, flow } = ctx;
  const notes: string[] = [];
  // Fold angles into [0, π] for the magnitude model (sin/cos based)
  let a = Math.abs(alphaT);
  if (a > Math.PI) a = a % (2 * Math.PI);
  if (a > Math.PI) a = 2 * Math.PI - a;
  a = Math.min(a, Math.PI);
  const M = flow.mach;
  const sgn = alphaT < 0 ? -1 : 1;
  const sa = Math.sin(a);
  const ca = Math.cos(a);
  // slender body potential + Allen-Perkins viscous crossflow
  const Sb = ints.baseArea;
  const xb = prof.x[prof.x.length - 1];
  const xcpPot = Sb > 1e-12 ? xb - ints.volume / Sb : 0.5 * xb;
  const CNpot = ((2 * Sb) / sRef) * sa * ca;
  const f = prof.length / Math.max(2 * prof.rMax, 1e-9);
  const eta = etaFinite(f);
  const Cdc = crossflowCd(M * sa);
  const CNvis = (eta * Cdc * ints.planform * sa * sa) / sRef;
  const Nslender = CNpot + CNvis;
  const Nxslender = CNpot * xcpPot + CNvis * ints.xPlanform;
  const dCAslender = 0;
  const w = newtonWeight(ctx);
  let N = Nslender;
  let Nx = Nxslender;
  let dCA = dCAslender;
  let tag = "slender-body + Allen-Perkins crossflow";
  if (w > 0) {
    const nw = newtonBody(ctx, a);
    const cache = ctx as BodyCtx & { _n0?: { N: number; Nx: number; CA: number } };
    const n0 = cache._n0 ?? (cache._n0 = newtonBody(ctx, 0));
    N = (1 - w) * Nslender + w * nw.N;
    Nx = (1 - w) * Nxslender + w * nw.Nx;
    dCA = w * (nw.CA - n0.CA);
    tag = w >= 0.999 ? "modified-Newtonian body integration" : "slender/Newtonian blend";
    if (w < 0.999) notes.push("Mach 3-8 body: blend of slender-body+crossflow and modified Newtonian (medium confidence)");
  }
  if (M < 0.3) notes.push("body: incompressible limit, slender-body theory only");
  return { N: sgn * N, Nx: sgn * Nx, dCA, tag, notes };
}

/** Slender-body pitch-damping Cmq (per rad of q*Lref/2V) referenced to Sref and `len`. */
export function bodyDamping(ctx: BodyCtx, len: number): number {
  const { prof, ints, sRef, xcg } = ctx;
  const xb = prof.x[prof.x.length - 1];
  const Sb = ints.baseArea;
  let integral = 0;
  for (let i = 0; i < prof.x.length - 1; i++) {
    const dx = prof.x[i + 1] - prof.x[i];
    const xm = 0.5 * (prof.x[i] + prof.x[i + 1]);
    const S = Math.PI * (0.5 * (prof.r[i] + prof.r[i + 1])) ** 2;
    integral += S * (xcg - xm) * dx;
  }
  // Munk apparent-mass result derived in the module notes: Cmq = -4 [Sb (xcg-xb)^2 + int S (xcg - x) dx] / (Sref len^2)
  return (-4 * (Sb * (xcg - xb) ** 2 + integral)) / (sRef * len * len);
}
