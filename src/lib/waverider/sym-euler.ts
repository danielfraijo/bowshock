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
// [mesh-refinement]
import { LEGACY, MEDIUM, estimateShockHeights, isBaseline, previewOf, shockFractionFromHeight, stripDims, sweepOf, type RefinementSettings } from "./refinement";
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

export function bindCenterline(mesh: TriMesh, upper: SurfaceGrid, lower: SurfaceGrid, span: number) {
  wallsOf.set(mesh, { upper: centerlineOf(upper), lower: centerlineOf(lower), span: Math.max(span, 1e-6) });
}

type Kern = {
  alloc: (n: number) => number;
  solve: (ptr: number, n: number, mach: number, gamma: number, alpha: number) => number;
  /** [mesh-refinement] present only in a wasm rebuilt from the updated crate; the shipped binary has no `euler_solve_ex`. */
  solveEx?: (ptr: number, n: number, mach: number, gamma: number, alpha: number, ni: number, nj: number, iters: number, wallAmp: number, shockAmp: number, shockWidth: number, tol: number, sequence: number, cfl: number) => number;
  memory: WebAssembly.Memory;
  ptr: number;
  cap: number;
};

let kern: Kern | null = null;
const solved = new Map<string, Float64Array>();
const stats = new Map<string, WallSolveInfo>();

function kernel(): Kern {
  if (kern) return kern;
  const bin = atob(EULER_WASM_B64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  const instance = new WebAssembly.Instance(new WebAssembly.Module(bytes), {});
  const exp = instance.exports as {
    euler_alloc: (n: number) => number;
    euler_solve: (ptr: number, n: number, mach: number, gamma: number, alpha: number) => number;
    euler_solve_ex?: Kern["solveEx"];
    memory: WebAssembly.Memory;
  };
  kern = { alloc: exp.euler_alloc, solve: exp.euler_solve, solveEx: exp.euler_solve_ex, memory: exp.memory, ptr: 0, cap: 0 };
  return kern;
}

/** [mesh-refinement] Grid-refined solve in TypeScript (see strip-euler.ts). Used for every setting except the baseline. */
function refinedPlan(wall: Centerline, mach: number, gamma: number, alpha: number, ref: RefinementSettings, noseR: number) {
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

/** [mesh-refinement] What one wall solve did (shown in the UI as the convergence readout). */
export interface WallSolveInfo {
  /** Sweeps run on the finest level of the last pass. */
  iters: number;
  /** Iteration cap that applied. */
  cap: number;
  /** Mass-flux residual (RMS d(rho)/dt) at the last sweep. */
  residual: number;
  converged: boolean;
  stop: string;
  cflFallback: boolean;
  /** Cell updates over all levels / passes. */
  work: number;
  grid: string;
  ms: number;
}

function solveCpRefined(wall: Centerline, mach: number, gamma: number, alpha: number, ref: RefinementSettings, noseR: number): { cp: Float64Array; info: WallSolveInfo } {
  const t0 = performance.now();
  const { v, d, shockS } = refinedPlan(wall, mach, gamma, alpha, ref, noseR);
  const adaptive = v.adaptive;
  // Rebuilt wasm (has euler_solve_ex): run the clustered static grid there. Not exercised by the shipped
  // binary; any failure falls through to the TypeScript solver, which is cross-checked against the Rust tests.
  // the (rebuilt) wasm entry point does first-order solves with the tolerance stop + sequencing; MUSCL and the adaptive re-cluster run in TypeScript
  if (!adaptive && (v.order ?? 1) === 1) {
    try {
      const k = kernel();
      if (k.solveEx) {
        const n = wall.x.length;
        const need = (n * 3 + d.ni + 1 + 16) * 8;
        if (k.cap < need) {
          k.ptr = k.alloc(need);
          k.cap = need;
        }
        const view = new Float64Array(k.memory.buffer, k.ptr, n * 3 + d.ni + 1);
        for (let i = 0; i < n; i++) {
          view[i * 2] = wall.x[i];
          view[i * 2 + 1] = wall.z[i];
        }
        for (let i = 0; i <= d.ni; i++) view[n * 3 + i] = shockS ? shockS[i] : 0.3;
        const res = k.solveEx(k.ptr, n, mach, gamma, alpha, d.ni, d.nj, d.iters, v.wallClustering, shockS ? v.shockClustering : 0, v.shockWidth, v.tol ?? 0, v.sequence ?? 0, 0);
        if (res >= 0 && Number.isFinite(res))
          return {
            cp: new Float64Array(new Float64Array(k.memory.buffer, k.ptr + n * 16, n)),
            info: { iters: d.iters, cap: d.iters, residual: res, converged: !!(v.tol && v.tol > 0), stop: "wasm", cflFallback: false, work: 0, grid: `${d.ni}x${d.nj}`, ms: performance.now() - t0 },
          };
      }
    } catch {
      /* fall through to TypeScript */
    }
  }
  const r = solveStrip(wall.x, wall.z, mach, gamma, alpha, stripGridOf(v, shockS));
  return { cp: r.cp, info: infoOf(r, d, performance.now() - t0) };
}

function infoOf(r: ReturnType<typeof solveStrip>, d: { ni: number; nj: number; iters: number }, ms: number): WallSolveInfo {
  return { iters: r.grid.itersUsed, cap: d.iters, residual: r.residual, converged: r.converged, stop: r.stopReason, cflFallback: r.cflFallback, work: r.work, grid: `${r.grid.ni}x${r.grid.nj}`, ms };
}

function refKey(ref?: RefinementSettings): string {
  if (!ref || isBaseline(ref)) return "";
  const v = ref.volume;
  return `|R${v.nStream}x${v.nCross}x${v.iters}w${v.wallClustering}s${v.shockClustering}u${v.shockWidth}a${v.adaptive ? 1 : 0}t${v.tol ?? 0}o${v.order ?? 1}q${v.sequence ?? 0}`;
}

/** [mesh-refinement] FNV-1a over the exact bytes of both wall arrays: the cache key sees the whole geometry, not three samples of it. */
function wallHash(wall: Centerline): string {
  let h = 0x811c9dc5;
  for (const arr of [wall.x, wall.z]) {
    const bytes = new Uint8Array(arr.buffer, arr.byteOffset, arr.byteLength);
    for (let i = 0; i < bytes.length; i++) {
      h ^= bytes[i];
      h = Math.imul(h, 0x01000193) >>> 0;
    }
  }
  return h.toString(16);
}

/** [mesh-refinement] true LRU (a hit moves the entry to the back); entries are tiny (one Cp array per wall). */
const SOLVE_CACHE_MAX = 96;
function remember(key: string, value: Float64Array) {
  solved.delete(key);
  solved.set(key, value);
  while (solved.size > SOLVE_CACHE_MAX) {
    const first = solved.keys().next().value;
    if (first === undefined) break;
    solved.delete(first);
    stats.delete(first);
  }
}

/** Cache size / capacity / keys oldest-first (tests). */
export function symmetryEulerCacheInfo() {
  return { size: solved.size, max: SOLVE_CACHE_MAX, keys: [...solved.keys()] };
}

/** Clears the solve cache (tests / memory pressure). */
export function clearSymmetryEulerCache() {
  solved.clear();
  stats.clear();
}

// ---------------------------------------------------------------------------------------------------------------------
// [mesh-refinement] Deferred solves, preview fidelity, and the convergence report.
//
// A study evaluates the Euler method at many angles of attack (derivatives, polar). Solving them synchronously inside a
// React render froze the page for 4-11 s. `beginEulerCollect` turns a cache MISS of a refined solve into a recorded job
// (the call returns the CBAERO result with a note); `solveEulerJobs` then solves the jobs in chunks that yield to the
// event loop, after which re-running the study is a pure cache hit. Nothing here changes a number: a completed
// collect -> solve -> re-run cycle equals the plain synchronous call exactly (tested).
// ---------------------------------------------------------------------------------------------------------------------
export class EulerDeferred extends Error {
  constructor() {
    super("Euler solve deferred");
    this.name = "EulerDeferred";
  }
}

export interface EulerJob {
  key: string;
  wall: Centerline;
  mach: number;
  gamma: number;
  alpha: number;
  ref: RefinementSettings;
  noseR: number;
  tag: string;
  primary: boolean;
}

let collector: Map<string, EulerJob> | null = null;
let fidelity: "full" | "preview" = "full";
let primaryAlpha: number | null = null;

/** Start recording missed refined solves. `primaryAlphaDeg`: the angle of attack the user asked for (others are sweep points). */
export function beginEulerCollect(opts: { primaryAlphaDeg?: number; preview?: boolean } = {}) {
  collector = new Map();
  fidelity = opts.preview ? "preview" : "full";
  primaryAlpha = opts.primaryAlphaDeg !== undefined && Number.isFinite(opts.primaryAlphaDeg) ? (opts.primaryAlphaDeg * Math.PI) / 180 : null;
}

/** Stop recording; returns the jobs (primary angle first). */
export function endEulerCollect(): EulerJob[] {
  const jobs = collector ? [...collector.values()] : [];
  collector = null;
  fidelity = "full";
  primaryAlpha = null;
  return jobs.sort((a, b) => Number(b.primary) - Number(a.primary));
}

function effectiveRef(ref: RefinementSettings, alpha: number): { ref: RefinementSettings; primary: boolean } {
  const primary = primaryAlpha === null || Math.abs(alpha - primaryAlpha) < 1e-9;
  if (isBaseline(ref)) return { ref, primary };
  if (fidelity === "preview") return { ref: previewOf(ref), primary };
  return { ref: primary ? ref : sweepOf(ref), primary };
}

let lastReport: EulerReport | null = null;
let lastKey = "";
export interface EulerReport {
  preset: string;
  /** The preview (coarse) fidelity was used instead of the selected preset. */
  preview: boolean;
  /** Some solves are still pending (CBAERO numbers shown for them). */
  pending: boolean;
  alphaDeg: number;
  upper?: WallSolveInfo;
  lower?: WallSolveInfo;
  /** Sweep points (derivatives / polar) were solved with a cheaper setting than the selected one. */
  sweepCheaper: boolean;
}
/** The convergence readout of the most recent primary Euler evaluation (null before any). */
export function symmetryEulerReport(): EulerReport | null {
  return lastReport;
}

function cpKey(wall: Centerline, mach: number, gamma: number, alpha: number, tag: string, ref?: RefinementSettings, noseR = 0): string {
  return `${tag}|${mach.toFixed(3)}|${gamma.toFixed(4)}|${alpha.toFixed(5)}|${wall.x.length}|${wallHash(wall)}|${noseR.toFixed(5)}${refKey(ref)}`;
}

function solveCp(wall: Centerline, mach: number, gamma: number, alpha: number, tag: string, refIn?: RefinementSettings, noseR = 0): Float64Array {
  const eff = refIn ? effectiveRef(refIn, alpha) : { ref: undefined, primary: true };
  const ref = eff.ref;
  const key = cpKey(wall, mach, gamma, alpha, tag, ref, noseR);
  lastKey = key;
  const hit = solved.get(key);
  if (hit) {
    remember(key, hit);
    return hit;
  }
  if (ref && !isBaseline(ref)) {
    if (collector) {
      collector.set(key, { key, wall, mach, gamma, alpha, ref, noseR, tag, primary: eff.primary });
      throw new EulerDeferred();
    }
    const { cp, info } = solveCpRefined(wall, mach, gamma, alpha, ref, noseR);
    remember(key, cp);
    stats.set(key, info);
    return cp;
  }
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
  remember(key, out);
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
  const machIn = params.lockFlight ? params.mach : params.flightMach;
  if (!Number.isFinite(machIn) || !Number.isFinite(alphaDeg) || !Number.isFinite(params.gamma) || params.gamma <= 1) {
    return { ...base, notes: [...base.notes, "Symmetry-plane Euler skipped: Mach / alpha / gamma is not a finite, valid number. CBAERO kept."] };
  }
  const mach = Math.max(1.05, machIn);
  const alpha = (alphaDeg * Math.PI) / 180;
  let up!: Float64Array;
  let lo!: Float64Array;
  const selected = params.refinement ?? MEDIUM;
  const primary = primaryAlpha === null || Math.abs(alpha - primaryAlpha) < 1e-9;
  try {
    const noseR = Math.max(effectiveLeRadius(params), 0.006 * params.length);
    // both walls are attempted so a deferred pass records BOTH jobs (one study pass, not one per wall)
    let deferred = false;
    let ku = "";
    let kl = "";
    try {
      up = solveCp(walls.upper, mach, params.gamma, alpha, "u", selected, noseR);
      ku = lastKey;
    } catch (e) {
      if (!(e instanceof EulerDeferred)) throw e;
      deferred = true;
    }
    try {
      lo = solveCp(walls.lower, mach, params.gamma, alpha, "l", selected, noseR);
      kl = lastKey;
    } catch (e) {
      if (!(e instanceof EulerDeferred)) throw e;
      deferred = true;
    }
    if (deferred) throw new EulerDeferred();
    if (!up.every(Number.isFinite) || !lo.every(Number.isFinite)) throw new Error("non-finite Cp");
    if (primary)
      lastReport = {
        preset: selected.preset,
        preview: fidelity === "preview" && !isBaseline(selected),
        pending: false,
        alphaDeg: alphaDeg,
        upper: stats.get(ku),
        lower: stats.get(kl),
        sweepCheaper: false,
      };
  } catch (err) {
    if (err instanceof EulerDeferred) {
      if (primary) lastReport = { preset: selected.preset, preview: fidelity === "preview", pending: true, alphaDeg, sweepCheaper: false };
      return { ...base, notes: [...base.notes, "Symmetry-plane Euler solve pending - CBAERO shown until it finishes."] };
    }
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
      ...(!isBaseline(params.refinement ?? MEDIUM)
        ? [
            `Mesh refinement "${(params.refinement ?? MEDIUM).preset}": symmetry-plane grid ${stripDims((params.refinement ?? MEDIUM).volume).ni}x${stripDims((params.refinement ?? MEDIUM).volume).nj}${(params.refinement ?? MEDIUM).volume.tol ? ` (converged to ${(params.refinement ?? MEDIUM).volume.tol} of peak residual, <= ${(params.refinement ?? MEDIUM).volume.iters} sweeps)` : ` x ${(params.refinement ?? MEDIUM).volume.iters} sweeps`}${(params.refinement ?? MEDIUM).volume.shockClustering > 0 || (params.refinement ?? MEDIUM).volume.wallClustering > 0 ? ", wall/shock clustering" : ""}${(params.refinement ?? MEDIUM).volume.adaptive ? " + adaptive re-cluster" : ""}${(params.refinement ?? MEDIUM).volume.order === 2 ? ", MUSCL" : ""} (TypeScript solver; choose "legacy" for the unconverged 88x28x42 wasm run of b5fdccc).`,
          ]
        : []),
    ],
  };
}

/** Sweeps per chunk so one chunk takes about `ms` milliseconds (measured ~4.5 M cell-updates/s in the TS kernel). */
export function chunkFor(cells: number, ms = 10): number {
  return Math.max(1, Math.min(400, Math.round((ms * 4500) / Math.max(cells, 1))));
}

/**
 * [mesh-refinement] Solve recorded jobs without blocking the UI thread: each job runs in chunks of about 10 ms that yield to the
 * event loop; `signal.aborted` cancels between chunks (rejects with AbortError). Results land in the LRU cache.
 */
export async function solveEulerJobs(
  jobs: EulerJob[],
  opts: { signal?: { aborted: boolean }; onProgress?: (done: number, total: number) => void; chunkMs?: number } = {},
): Promise<void> {
  let done = 0;
  for (const job of jobs) {
    if (opts.signal?.aborted) throw new DOMException("Euler solve aborted", "AbortError");
    if (!solved.has(job.key)) {
      const t0 = performance.now();
      const { v, d, shockS } = refinedPlan(job.wall, job.mach, job.gamma, job.alpha, job.ref, job.noseR);
      const r = await solveStripAsync(job.wall.x, job.wall.z, job.mach, job.gamma, job.alpha, stripGridOf(v, shockS), {
        chunk: chunkFor(d.ni * d.nj, opts.chunkMs ?? 10),
        signal: opts.signal,
        onProgress: (p) => opts.onProgress?.(done + p, jobs.length),
      });
      if (!r.cp.every(Number.isFinite)) throw new Error("non-finite Cp");
      remember(job.key, r.cp);
      stats.set(job.key, infoOf(r, d, performance.now() - t0));
    }
    done++;
    opts.onProgress?.(done, jobs.length);
  }
}

/**
 * [mesh-refinement] Pre-solve both walls of a built vehicle into the solve cache without blocking the UI thread. A later
 * synchronous `withSymmetryEuler` for the same geometry / flight condition / refinement is then a cache hit.
 * Resolves to false when there is nothing to warm (legacy = the wasm run is already ~20 ms, or no centerline).
 */
export async function warmSymmetryEuler(
  mesh: TriMesh,
  params: DesignParams,
  alphaDeg: number,
  opts: { chunk?: number; signal?: { aborted: boolean }; onProgress?: (p: number) => void } = {},
): Promise<boolean> {
  const walls = wallsOf.get(mesh);
  const ref = params.refinement ?? MEDIUM;
  if (!walls || walls.upper.x.length < 4 || isBaseline(ref)) return false;
  const machIn = params.lockFlight ? params.mach : params.flightMach;
  if (!Number.isFinite(machIn) || !Number.isFinite(alphaDeg) || !Number.isFinite(params.gamma) || params.gamma <= 1) return false;
  const mach = Math.max(1.05, machIn);
  const alpha = (alphaDeg * Math.PI) / 180;
  const noseR = Math.max(effectiveLeRadius(params), 0.006 * params.length);
  const jobs: EulerJob[] = [];
  for (const [tag, wall] of [["u", walls.upper], ["l", walls.lower]] as const) {
    const key = cpKey(wall, mach, params.gamma, alpha, tag, ref, noseR);
    if (!solved.has(key)) jobs.push({ key, wall, mach, gamma: params.gamma, alpha, ref, noseR, tag, primary: true });
  }
  await solveEulerJobs(jobs, { signal: opts.signal, onProgress: (d, t) => opts.onProgress?.(t ? d / t : 1) });
  return true;
}
