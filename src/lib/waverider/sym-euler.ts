/**
 * Symmetry-plane Euler (Rust → wasm32).
 * A coarse HLLC solve on the centerline, between CBAERO's local-inclination
 * panel method and a Cart3D volume Euler. The nose and the centerline use
 * the captured shock; outboard panels stay on CBAERO.
 */
import type { Atmosphere } from "./atmosphere";
import { effectiveLeRadius } from "./blunt";
import { EULER_WASM_B64 } from "./euler-wasm";
import type { PanelAero } from "./panel";
import { SURFACE_ID, type DesignParams, type SurfaceGrid, type TriMesh, type Vec3 } from "./types";

export interface Centerline {
  x: Float64Array;
  z: Float64Array;
}

interface Walls {
  upper: Centerline;
  lower: Centerline;
  span: number;
}

const wallsOf = new WeakMap<TriMesh, Walls>();

function monotone(xIn: Float64Array, zIn: Float64Array): Centerline {
  const x: number[] = [];
  const z: number[] = [];
  for (let i = 0; i < xIn.length; i++) {
    if (x.length && xIn[i] <= x[x.length - 1] + 1e-7) continue;
    x.push(xIn[i]);
    z.push(zIn[i]);
  }
  if (x.length < 4) {
    return { x: xIn.slice(), z: zIn.slice() };
  }
  return { x: Float64Array.from(x), z: Float64Array.from(z) };
}

function centerlineOf(g: SurfaceGrid): Centerline {
  let jBest = 0;
  let best = Infinity;
  for (let j = 0; j < g.nj; j++) {
    let y = 0;
    for (let i = 0; i < g.ni; i++) y += Math.abs(g.xyz[(i * g.nj + j) * 3 + 1]);
    y /= Math.max(g.ni, 1);
    if (y < best) {
      best = y;
      jBest = j;
    }
  }
  const x = new Float64Array(g.ni);
  const z = new Float64Array(g.ni);
  for (let i = 0; i < g.ni; i++) {
    const o = (i * g.nj + jBest) * 3;
    x[i] = g.xyz[o];
    z[i] = g.xyz[o + 2];
  }
  return monotone(x, z);
}

export function bindCenterline(mesh: TriMesh, upper: SurfaceGrid, lower: SurfaceGrid, span: number) {
  wallsOf.set(mesh, { upper: centerlineOf(upper), lower: centerlineOf(lower), span: Math.max(span, 1e-6) });
}

type Kern = {
  alloc: (n: number) => number;
  solve: (ptr: number, n: number, mach: number, gamma: number, alpha: number) => number;
  memory: WebAssembly.Memory;
  ptr: number;
  cap: number;
};

let kern: Kern | null = null;
const solved = new Map<string, Float64Array>();

function kernel(): Kern {
  if (kern) return kern;
  const bin = atob(EULER_WASM_B64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  const instance = new WebAssembly.Instance(new WebAssembly.Module(bytes), {});
  const exp = instance.exports as {
    euler_alloc: (n: number) => number;
    euler_solve: (ptr: number, n: number, mach: number, gamma: number, alpha: number) => number;
    memory: WebAssembly.Memory;
  };
  kern = { alloc: exp.euler_alloc, solve: exp.euler_solve, memory: exp.memory, ptr: 0, cap: 0 };
  return kern;
}

function solveCp(wall: Centerline, mach: number, gamma: number, alpha: number, tag: string): Float64Array {
  const key = `${tag}|${mach.toFixed(2)}|${gamma.toFixed(3)}|${alpha.toFixed(3)}|${wall.x.length}|${wall.x[0].toFixed(4)}|${wall.z[wall.z.length >> 1].toFixed(4)}|${wall.x[wall.x.length - 1].toFixed(4)}`;
  const hit = solved.get(key);
  if (hit) return hit;
  const k = kernel();
  const n = wall.x.length;
  const need = (n * 3 + 16) * 8;
  if (k.cap < need) {
    k.ptr = k.alloc(need);
    k.cap = need;
  }
  const view = new Float64Array(k.memory.buffer, k.ptr, n * 2);
  for (let i = 0; i < n; i++) {
    view[i * 2] = wall.x[i];
    view[i * 2 + 1] = wall.z[i];
  }
  k.solve(k.ptr, n, mach, gamma, alpha);
  const out = new Float64Array(new Float64Array(k.memory.buffer, k.ptr + n * 16, n));
  solved.set(key, out);
  if (solved.size > 24) {
    const first = solved.keys().next().value;
    if (first) solved.delete(first);
  }
  return out;
}

function interp(wall: Centerline, cp: Float64Array, x: number): number {
  const xs = wall.x;
  if (x <= xs[0]) return cp[0];
  const last = xs.length - 1;
  if (x >= xs[last]) return cp[last];
  let lo = 0;
  let hi = last;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (xs[mid] <= x) lo = mid;
    else hi = mid;
  }
  const dx = xs[hi] - xs[lo];
  const t = dx > 1e-12 ? (x - xs[lo]) / dx : 0;
  return cp[lo] + (cp[hi] - cp[lo]) * t;
}

/** Local dz/dx of the bound centreline at x (finite difference of the bracketing segment). */
function wallSlope(wall: Centerline, x: number): number {
  const xs = wall.x;
  const last = xs.length - 1;
  let lo = 0;
  let hi = last;
  if (x <= xs[0]) hi = 1;
  else if (x >= xs[last]) lo = last - 1;
  else {
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (xs[mid] <= x) lo = mid;
      else hi = mid;
    }
  }
  const dx = xs[hi] - xs[lo];
  return dx > 1e-12 ? (wall.z[hi] - wall.z[lo]) / dx : 0;
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

/** Replace centerline / nose Cp with the Rust Euler field and recompute the coefficients. */
export function withSymmetryEuler(
  mesh: TriMesh,
  params: DesignParams,
  atm: Atmosphere,
  base: PanelAero,
  alphaDeg: number,
  betaDeg: number,
): PanelAero {
  const walls = wallsOf.get(mesh);
  if (!walls || walls.upper.x.length < 4) {
    return {
      ...base,
      notes: [...base.notes, "Symmetry-plane Euler needs an upper/lower centerline — CBAERO coefficients kept."],
    };
  }
  const mach = Math.max(1.05, params.lockFlight ? params.mach : params.flightMach);
  const alpha = (alphaDeg * Math.PI) / 180;
  let up: Float64Array;
  let lo: Float64Array;
  try {
    up = solveCp(walls.upper, mach, params.gamma, alpha, "u");
    lo = solveCp(walls.lower, mach, params.gamma, alpha, "l");
  } catch (err) {
    return { ...base, notes: [...base.notes, `Euler kernel failed (${err instanceof Error ? err.message : "wasm"}). CBAERO kept.`] };
  }
  const Rn = Math.max(effectiveLeRadius(params), 0.006 * params.length);
  const span = walls.span;
  const cp = new Float32Array(base.cp);
  const pos = mesh.positions;
  const ind = mesh.indices;
  const nt = ind.length / 3;
  const alphaR = alpha;
  const betaR = (betaDeg * Math.PI) / 180;
  const vhat = flowDir(alphaR, betaR);
  const cg = base.cg;
  let Fx = 0;
  let Fy = 0;
  let Fz = 0;
  let Mx = 0;
  let My = 0;
  let Mz = 0;
  let FxBase = 0;
  for (let t = 0; t < nt; t++) {
    const ia = ind[t * 3] * 3;
    const ib = ind[t * 3 + 1] * 3;
    const ic = ind[t * 3 + 2] * 3;
    const x1 = pos[ia];
    const y1 = pos[ia + 1];
    const z1 = pos[ia + 2];
    const x2 = pos[ib];
    const y2 = pos[ib + 1];
    const z2 = pos[ib + 2];
    const x3 = pos[ic];
    const y3 = pos[ic + 1];
    const z3 = pos[ic + 2];
    const nx = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    const ny = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    const nz = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    const mag = Math.hypot(nx, ny, nz);
    if (mag < 1e-16) continue;
    const area = 0.5 * mag;
    const inv = 1 / mag;
    const n0 = nx * inv;
    const n1 = ny * inv;
    const n2 = nz * inv;
    const cx = (x1 + x2 + x3) / 3;
    const cy = (y1 + y2 + y3) / 3;
    const cz = (z1 + z2 + z3) / 3;
    let surf = mesh.surfaces[t] ?? 0;
    if (n0 > 0.72 && surf !== SURFACE_ID.leading && surf !== SURFACE_ID.inlet) surf = SURFACE_ID.base;
    const basePanel = surf === SURFACE_ID.base || surf === SURFACE_ID.nozzle;
    if (!basePanel) {
      const wall = cz >= 0 ? walls.upper : walls.lower;
      const field = cz >= 0 ? up : lo;
      const eCp = interp(wall, field, cx);
      const yN = Math.abs(cy) / (0.5 * span);
      // PHYSICS-ACCURACY CHANGE (Dina, flagged shared edit): the 2-D centreline solution is the pressure on the
      // *keel line*. On an anhedral caret the lower faces are planar facets 33° out of the symmetry plane
      // whose inclination to the flow (8.7°) is not the keel's, so blending it in cut the lower-face Cp from the
      // exact oblique-shock 0.0787 to 0.0195 (CL −75 %). Euler now only carries a panel whose normal agrees with
      // the normal of the wall it was solved on (Gaussian in the angle, 6° for the span blend, 29° for the nose
      // cap where the surface turns quickly); elsewhere the base method (exact on attached facets) stays.
      const sl = wallSlope(wall, cx);
      const inv = 1 / Math.sqrt(1 + sl * sl);
      const sgn = cz >= 0 ? 1 : -1;
      const dotN = sgn * (-sl * inv * n0 + inv * n2);
      const dAng = Math.acos(Math.max(-1, Math.min(1, dotN)));
      const gCenter = Math.exp(-((dAng / 0.105) ** 2));
      const gNose = Math.exp(-((dAng / 0.5) ** 2));
      const center = Math.exp(-((yN / 0.34) ** 2)) * gCenter;
      const nose = cx < 3.2 * Rn ? Math.exp(-((yN / 0.7) ** 2)) * gNose : 0;
      const w = Math.max(center, nose);
      // Physical bound (Dina, flagged): a steady wall pressure cannot exceed the pitot (post-normal-shock stagnation) Cp.
      // The centreline solve gave up to 2.9 (pitot 1.7) at M3, α = −30° on the lift body.
      cp[t] = Math.min(base.cp[t] * (1 - w) + eCp * w, Math.max(base.cpMax, base.cp[t]));
    }
    const cpi = cp[t];
    const dFx = -cpi * area * n0;
    const dFy = -cpi * area * n1;
    const dFz = -cpi * area * n2;
    Fx += dFx;
    Fy += dFy;
    Fz += dFz;
    if (basePanel) FxBase += dFx;
    const rx = cx - cg[0];
    const ry = cy - cg[1];
    const rz = cz - cg[2];
    Mx += ry * dFz - rz * dFy;
    My += rz * dFx - rx * dFz;
    Mz += rx * dFy - ry * dFx;
    void vhat;
  }
  const q = atm.q || 1;
  const S = Math.max(base.sRef, 1e-8);
  const L = Math.max(base.lRef, 1e-8);
  const wind = rotateForce([Fx * q, Fy * q, Fz * q], alphaR, betaR);
  const Dfric = base.cdFric * q * S;
  const lift = wind.lift;
  const drag = wind.drag + Dfric;
  const cl = lift / (q * S);
  const cd = drag / (q * S);
  const cdBase = Math.abs(FxBase) / S;
  const cdFric = base.cdFric;
  const cdWave = Math.max(0, cd - cdFric - cdBase);
  const ld = cd > 1e-10 ? cl / cd : 0;
  const xCp = Math.abs(Fz) > 1e-14 ? Math.max(0, Math.min(L, cg[0] - My / Fz)) : cg[0];
  return {
    ...base,
    cl,
    cd,
    cy: wind.side / (q * S),
    cm: My / (S * L),
    cn: Mz / (S * L),
    cll: Mx / (S * L),
    ld,
    ca: Fx / S,
    cnA: Fz / S,
    cdWave,
    cdBase,
    cp,
    xCp,
    force: [Fx * q, Fy * q, Fz * q],
    moment: [Mx * q, My * q, Mz * q],
    method: "euler",
    notes: [
      ...base.notes.filter((n) => !n.startsWith("CBAERO-class") && !n.startsWith("Tangent") && !n.startsWith("Modified")),
      "Strip Euler: Rust HLLC on the symmetry plane (bow shock captured), blended into CBAERO off the centerline. Between a CBAERO panel run and Cart3D — not a volume mesh.",
    ],
  };
}
