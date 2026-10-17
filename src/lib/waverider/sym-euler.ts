/**
 * Symmetry-plane Euler (Rust → wasm32).
 * A coarse HLLC solve on the centerline, between CBAERO's local-inclination
 * panel method and a Cart3D volume Euler. The nose and the centerline use
 * the captured shock; outboard panels stay on CBAERO.
 */
import type { Atmosphere } from "./atmosphere";
import { effectiveLeRadius } from "./blunt";
import { qualitySettings } from "../perf/quality";
import { cachedWallCp, eulerCache, kernelFlags, hasCachedWallCp, solveWallCp, solveWallEx, storeCachedWallCp } from "../perf/euler-kernel";
import type { PanelAero } from "./panel";
// [mesh-refinement]
import { LEGACY, estimateShockHeights, isBaseline, shockFractionFromHeight, stripDims, type RefinementSettings } from "./refinement";
import { solveStrip, solveStripAsync, stripDomain, stripGridOf } from "./strip-euler";
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

/** [perf] read access to the bound upper/lower walls (bench + parity tests). */
export function wallsOfMesh(mesh: TriMesh): { upper: Centerline; lower: Centerline; span: number } | undefined {
  return wallsOf.get(mesh);
}

export function bindCenterline(mesh: TriMesh, upper: SurfaceGrid, lower: SurfaceGrid, span: number) {
  wallsOf.set(mesh, { upper: centerlineOf(upper), lower: centerlineOf(lower), span: Math.max(span, 1e-6) });
}

/** [mesh-refinement] Grid-refined solve (wasm euler_solve_ex, TypeScript fallback in strip-euler.ts). Used for every setting except the baseline. */
export function refinedPlan(wall: Centerline, mach: number, gamma: number, alpha: number, ref: RefinementSettings, noseR: number) {
  const v = ref.volume;
  const d = stripDims(v);
  const dom = stripDomain(wall.x, wall.z);
  const shockOn = v.shockClustering > 0 || v.adaptive;
  let shockS: Float64Array | null = null;
  if (shockOn) {
    const xs = new Float64Array(d.ni + 1);
    for (let i = 0; i <= d.ni; i++) xs[i] = dom.xmin + ((dom.xmax - dom.xmin) * i) / d.ni;
    const h = estimateShockHeights(wall.x, wall.z, xs, Math.max(1.05, mach), gamma, alpha, dom.sign as 1 | -1, noseR);
    shockS = new Float64Array(d.ni + 1);
    for (let i = 0; i <= d.ni; i++) shockS[i] = shockFractionFromHeight(h[i], dom.hFar, LEGACY.strip.wallExp);
  }
  return { v, d, shockS };
}

function solveCpRefined(wall: Centerline, mach: number, gamma: number, alpha: number, ref: RefinementSettings, noseR: number): Float64Array {
  const { v, d, shockS } = refinedPlan(wall, mach, gamma, alpha, ref, noseR);
  const adaptive = v.adaptive;
  // The wasm entry point (euler_solve_ex: first-order, tolerance stop + grid sequencing) runs the static
  // clustered grid; MUSCL and the adaptive re-cluster pass run in TypeScript. Any failure or an old binary
  // without the export falls through to the TypeScript solver (cross-checked against the Rust tests).
  if (!adaptive && (v.order ?? 1) === 1) {
    try {
      const out = solveWallEx(wall.x, wall.z, mach, gamma, alpha, {
        ni: d.ni,
        nj: d.nj,
        iters: d.iters,
        wallAmp: v.wallClustering,
        shockAmp: shockS ? v.shockClustering : 0,
        shockWidth: v.shockWidth,
        tol: v.tol ?? 0,
        sequence: v.sequence ?? 0,
        cfl: 0,
        shockS,
      });
      if (out) return out;
    } catch {
      /* fall through to TypeScript */
    }
  }
  return solveStrip(wall.x, wall.z, mach, gamma, alpha, stripGridOf(v, shockS)).cp;
}

function refKey(ref?: RefinementSettings): string {
  if (!ref || isBaseline(ref)) return "";
  const v = ref.volume;
  return `|R${v.nStream}x${v.nCross}x${v.iters}w${v.wallClustering}s${v.shockClustering}u${v.shockWidth}a${v.adaptive ? 1 : 0}t${v.tol ?? 0}o${v.order ?? 1}q${v.sequence ?? 0}`;
}

/** Clears the solve cache (tests / memory pressure). */
export function clearSymmetryEulerCache() {
  eulerCache.clear();
}

/** Test hook: the wall solve behind `withSymmetryEuler` (tier-aware, cached). */
export function solveCpForTest(wall: Centerline, mach: number, gamma: number, alpha: number): Float64Array {
  return solveCp(wall, mach, gamma, alpha, "t", undefined, 0.02);
}

/** [perf] exact-key LRU (full wall hash + exact floats) + buffer reuse live in perf/euler-kernel. */
function solveCp(wall: Centerline, mach: number, gamma: number, alpha: number, _tag: string, ref?: RefinementSettings, noseR = 0): Float64Array {
  // an explicit (non-baseline) refinement wins; otherwise the active quality tier (legacy = shipped kernel)
  const eff = ref && !isBaseline(ref) ? ref : qualitySettings(kernelFlags.quality);
  if (eff) {
    return cachedWallCp(wall.x, wall.z, mach, gamma, alpha, `${noseR}${refKey(eff)}`, () => solveCpRefined(wall, mach, gamma, alpha, eff, noseR));
  }
  return solveWallCp(wall.x, wall.z, mach, gamma, alpha);
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
  const machIn = params.lockFlight ? params.mach : params.flightMach;
  if (!Number.isFinite(machIn) || !Number.isFinite(alphaDeg) || !Number.isFinite(params.gamma) || params.gamma <= 1) {
    return { ...base, notes: [...base.notes, "Symmetry-plane Euler skipped: Mach / alpha / gamma is not a finite, valid number. CBAERO kept."] };
  }
  const mach = Math.max(1.05, machIn);
  const alpha = (alphaDeg * Math.PI) / 180;
  let up: Float64Array;
  let lo: Float64Array;
  try {
    const noseR = Math.max(effectiveLeRadius(params), 0.006 * params.length);
    up = solveCp(walls.upper, mach, params.gamma, alpha, "u", params.refinement, noseR);
    lo = solveCp(walls.lower, mach, params.gamma, alpha, "l", params.refinement, noseR);
    if (!up.every(Number.isFinite) || !lo.every(Number.isFinite)) throw new Error("non-finite Cp");
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
      const center = Math.exp(-((yN / 0.34) ** 2));
      const nose = cx < 3.2 * Rn ? Math.exp(-((yN / 0.7) ** 2)) : 0;
      const w = Math.max(center, nose);
      cp[t] = base.cp[t] * (1 - w) + eCp * w;
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
      ...(params.refinement && !isBaseline(params.refinement)
        ? [
            `Mesh refinement "${params.refinement.preset}": symmetry-plane grid ${stripDims(params.refinement.volume).ni}x${stripDims(params.refinement.volume).nj}x${stripDims(params.refinement.volume).iters}${params.refinement.volume.shockClustering > 0 || params.refinement.volume.wallClustering > 0 ? " with wall/shock clustering" : ""}${params.refinement.volume.adaptive ? " + adaptive re-cluster" : ""} (TypeScript solver; the legacy kernel is the 88x28x42 baseline, refined tiers run euler_solve_ex).`,
          ]
        : []),
    ],
  };
}

/**
 * [mesh-refinement] Pre-solve both walls of a built vehicle into the solve cache without blocking the UI thread
 * (the refined solver is chunked and yields to the event loop every `chunk` sweeps). A later synchronous
 * `withSymmetryEuler` for the same geometry / flight condition / refinement is then a cache hit.
 * Resolves to false when there is nothing to warm (baseline = the wasm run is already ~20 ms, or no centerline).
 */
export async function warmSymmetryEuler(
  mesh: TriMesh,
  params: DesignParams,
  alphaDeg: number,
  opts: { chunk?: number; signal?: { aborted: boolean }; onProgress?: (p: number) => void } = {},
): Promise<boolean> {
  const walls = wallsOf.get(mesh);
  const ref = params.refinement;
  if (!walls || walls.upper.x.length < 4 || !ref || isBaseline(ref)) return false;
  const machIn = params.lockFlight ? params.mach : params.flightMach;
  if (!Number.isFinite(machIn) || !Number.isFinite(alphaDeg) || !Number.isFinite(params.gamma) || params.gamma <= 1) return false;
  const mach = Math.max(1.05, machIn);
  const alpha = (alphaDeg * Math.PI) / 180;
  const noseR = Math.max(effectiveLeRadius(params), 0.006 * params.length);
  let done = 0;
  for (const [, wall] of [["u", walls.upper], ["l", walls.lower]] as const) {
    const extra = `${noseR}${refKey(ref)}`;
    if (hasCachedWallCp(wall.x, wall.z, mach, params.gamma, alpha, extra)) {
      done++;
      continue;
    }
    const { v, shockS } = refinedPlan(wall, mach, params.gamma, alpha, ref, noseR);
    const r = await solveStripAsync(wall.x, wall.z, mach, params.gamma, alpha, stripGridOf(v, shockS), {
      chunk: opts.chunk,
      signal: opts.signal,
      onProgress: (p) => opts.onProgress?.((done + p) / 2),
    });
    if (r.cp.every(Number.isFinite)) storeCachedWallCp(wall.x, wall.z, mach, params.gamma, alpha, extra, r.cp);
    done++;
  }
  return true;
}
