/**
 * User-selectable mesh refinement: presets, validation, cost readout and the
 * grid-clustering functions shared by the surface mesh and the Euler volume grids.
 *
 * Fidelity target: above a CBAERO panel run, below a Cart3D volume Euler.
 * Nothing here changes a physics model — only where grid points go and how many.
 *
 * Defaults: `medium` reproduces the pre-refinement behaviour exactly
 * (surface nx/ny = DEFAULT_PARAMS 40x28, wedge-anchor Euler 72x24x280,
 * symmetry-plane kernel 88x28x42, no clustering). See refinement.test.ts.
 */
import { betaFromThetaM } from "./math";

export type RefinementPreset = "coarse" | "medium" | "fine" | "custom";

export interface SurfaceRefinement {
  /** Streamwise points per loft sheet. */
  nx: number;
  /** Spanwise points per loft sheet. */
  ny: number;
  /** 0 = today's stations (cosine-clustered). 1 = strong extra clustering toward the leading edge / nose. */
  leClustering: number;
}

export interface VolumeRefinement {
  /** Streamwise cells of the 72x24x280-class wedge anchor. */
  nStream: number;
  /** Cells across the shock layer (wall to far field). */
  nCross: number;
  /** Pseudo-time iterations (local time step). */
  iters: number;
  /** 0 = legacy wall spacing. >0 piles cells toward the wall (monitor-function amplitude). */
  wallClustering: number;
  /** 0 = legacy spacing. >0 piles cells toward the predicted shock (monitor-function amplitude). */
  shockClustering: number;
  /** Gaussian half-width of the shock cluster, as a fraction of the cross-flow extent. */
  shockWidth: number;
  /** Re-cluster toward the computed density jump after a first solve, then re-solve. */
  adaptive: boolean;
}

export interface RefinementSettings {
  preset: RefinementPreset;
  surface: SurfaceRefinement;
  volume: VolumeRefinement;
}

/** Legacy grids this module must reproduce at `medium`. */
export const LEGACY = {
  surface: { nx: 40, ny: 28 },
  wedge: { ni: 72, nj: 24, iters: 280 },
  /** native/cuspis-euler NI, NJ, ITERS (wasm already shipped in euler-wasm.ts). */
  strip: { ni: 88, nj: 28, iters: 42, wallExp: 1.55 },
} as const;

export const REFINEMENT_LIMITS = {
  surface: { nx: [12, 160], ny: [10, 120], leClustering: [0, 1], maxPoints: 12000 },
  volume: {
    nStream: [24, 400],
    nCross: [8, 160],
    iters: [20, 4000],
    wallClustering: [0, 12],
    shockClustering: [0, 30],
    shockWidth: [0.02, 0.3],
    /** Hard cap on cells of one 2-D grid. Memory is ~140 B/cell, so this is far inside browser limits. */
    maxCells: 60000,
    /** Hard cap on cell-updates (cells x iterations): bounds wall-clock time (~10-15 s at the measured TypeScript rates). */
    maxWork: 5e7,
  },
} as const;

/**
 * Rough throughput in cell-updates per second, measured on this box (see NOTES.md; your CPU will differ):
 * TypeScript wedge anchor ~3.4 M/s, TypeScript symmetry-plane solver ~5.5 M/s, shipped wasm ~8 M/s
 * (measured on the legacy 88x28x42 run only - a small sample).
 */
export const COST_MODEL = {
  jsRateWedge: 3.4e6,
  jsRateStrip: 5.5e6,
  wasmRateStrip: 8e6,
} as const;

export const DEFAULT_SHOCK_WIDTH = 0.09;
/** Warn (not clamp) when adjacent cell sizes in the cross-flow direction differ by more than this. The presets stay under it. */
export const MAX_SMOOTH_STRETCH = 2;

const PRESETS: Record<Exclude<RefinementPreset, "custom">, RefinementSettings> = {
  coarse: {
    preset: "coarse",
    surface: { nx: 24, ny: 16, leClustering: 0 },
    volume: {
      nStream: 48,
      nCross: 16,
      iters: 400,
      wallClustering: 0,
      shockClustering: 0,
      shockWidth: DEFAULT_SHOCK_WIDTH,
      adaptive: false,
    },
  },
  medium: {
    preset: "medium",
    surface: { nx: LEGACY.surface.nx, ny: LEGACY.surface.ny, leClustering: 0 },
    volume: {
      nStream: LEGACY.wedge.ni,
      nCross: LEGACY.wedge.nj,
      iters: LEGACY.wedge.iters,
      wallClustering: 0,
      shockClustering: 0,
      shockWidth: DEFAULT_SHOCK_WIDTH,
      adaptive: false,
    },
  },
  fine: {
    preset: "fine",
    surface: { nx: 96, ny: 32, leClustering: 0.6 },
    volume: {
      nStream: 96,
      nCross: 32,
      iters: 1500,
      wallClustering: 2,
      shockClustering: 6,
      shockWidth: DEFAULT_SHOCK_WIDTH,
      adaptive: false,
    },
  },
};

export function presetSettings(id: Exclude<RefinementPreset, "custom">): RefinementSettings {
  const p = PRESETS[id];
  return { preset: p.preset, surface: { ...p.surface }, volume: { ...p.volume } };
}

export const MEDIUM: RefinementSettings = presetSettings("medium");

/** Effective settings for a design: refinement (or medium) with the surface counts mirrored from params.nx / params.ny (those sliders are the source of truth). */
export function refinementOf(p: {
  refinement?: RefinementSettings;
  nx: number;
  ny: number;
}): RefinementSettings {
  const r = p.refinement ?? MEDIUM;
  return { ...r, surface: { ...r.surface, nx: p.nx, ny: p.ny } };
}

export interface ValidatedRefinement {
  settings: RefinementSettings;
  /** Human-readable notes: every clamp, rounding or cap that changed a requested value. */
  warnings: string[];
  clamped: boolean;
}

type Partial2<T> = { [K in keyof T]?: T[K] };
export interface RefinementInput {
  preset?: RefinementPreset;
  surface?: Partial2<SurfaceRefinement>;
  volume?: Partial2<VolumeRefinement>;
}

function num(v: unknown, fallback: number): number {
  return typeof v === "number" && Number.isFinite(v) ? v : fallback;
}

/**
 * Clamp and cap anything: unknown preset -> medium, NaN -> default, ranges clamped,
 * integers rounded, then the total-cell / total-work caps enforced (cells are reduced
 * first, keeping aspect, then iterations). A named preset ignores custom numbers.
 */
export function validateRefinement(input?: RefinementInput | null): ValidatedRefinement {
  const warnings: string[] = [];
  const L = REFINEMENT_LIMITS;
  const id = input?.preset;
  if (id === "coarse" || id === "medium" || id === "fine") {
    return { settings: presetSettings(id), warnings, clamped: false };
  }
  if (id !== undefined && id !== "custom")
    warnings.push(`Unknown preset "${String(id)}" - using medium.`);
  if (id !== "custom") return { settings: presetSettings("medium"), warnings, clamped: false };

  const base = PRESETS.medium;
  const s = input?.surface ?? {};
  const v = input?.volume ?? {};
  let clamped = false;
  const fit = (
    label: string,
    raw: unknown,
    def: number,
    lo: number,
    hi: number,
    integer: boolean,
  ) => {
    let x = num(raw, def);
    if (typeof raw !== "number" || !Number.isFinite(raw)) {
      if (raw !== undefined) {
        warnings.push(`${label}: not a finite number, using ${def}.`);
        clamped = true;
      }
    }
    if (integer) {
      const r = Math.round(x);
      if (r !== x) {
        x = r;
      }
    }
    if (x < lo) {
      warnings.push(`${label}: ${x} below minimum ${lo}, raised.`);
      x = lo;
      clamped = true;
    } else if (x > hi) {
      warnings.push(`${label}: ${x} above maximum ${hi}, lowered.`);
      x = hi;
      clamped = true;
    }
    return x;
  };

  const surface: SurfaceRefinement = {
    nx: fit("surface nx", s.nx, base.surface.nx, L.surface.nx[0], L.surface.nx[1], true),
    ny: fit("surface ny", s.ny, base.surface.ny, L.surface.ny[0], L.surface.ny[1], true),
    leClustering: fit(
      "LE clustering",
      s.leClustering,
      0,
      L.surface.leClustering[0],
      L.surface.leClustering[1],
      false,
    ),
  };
  if (surface.nx * surface.ny > L.surface.maxPoints) {
    const k = Math.sqrt(L.surface.maxPoints / (surface.nx * surface.ny));
    surface.nx = Math.max(L.surface.nx[0], Math.floor(surface.nx * k));
    surface.ny = Math.max(L.surface.ny[0], Math.floor(surface.ny * k));
    warnings.push(
      `Surface grid capped at ${L.surface.maxPoints} points per sheet: now ${surface.nx} x ${surface.ny}.`,
    );
    clamped = true;
  }

  const volume: VolumeRefinement = {
    nStream: fit(
      "volume cells streamwise",
      v.nStream,
      base.volume.nStream,
      L.volume.nStream[0],
      L.volume.nStream[1],
      true,
    ),
    nCross: fit(
      "volume cells across shock layer",
      v.nCross,
      base.volume.nCross,
      L.volume.nCross[0],
      L.volume.nCross[1],
      true,
    ),
    iters: fit(
      "Euler iterations",
      v.iters,
      base.volume.iters,
      L.volume.iters[0],
      L.volume.iters[1],
      true,
    ),
    wallClustering: fit(
      "wall clustering",
      v.wallClustering,
      0,
      L.volume.wallClustering[0],
      L.volume.wallClustering[1],
      false,
    ),
    shockClustering: fit(
      "shock clustering",
      v.shockClustering,
      0,
      L.volume.shockClustering[0],
      L.volume.shockClustering[1],
      false,
    ),
    shockWidth: fit(
      "shock cluster width",
      v.shockWidth,
      DEFAULT_SHOCK_WIDTH,
      L.volume.shockWidth[0],
      L.volume.shockWidth[1],
      false,
    ),
    adaptive: v.adaptive === true,
  };
  if (volume.nStream * volume.nCross > L.volume.maxCells) {
    const k = Math.sqrt(L.volume.maxCells / (volume.nStream * volume.nCross));
    volume.nStream = Math.max(L.volume.nStream[0], Math.floor(volume.nStream * k));
    volume.nCross = Math.max(L.volume.nCross[0], Math.floor(volume.nCross * k));
    warnings.push(
      `Volume grid capped at ${L.volume.maxCells} cells: now ${volume.nStream} x ${volume.nCross}.`,
    );
    clamped = true;
  }
  if (volume.wallClustering > 0 || volume.shockClustering > 0) {
    const ratio = spacingStats(
      clusterCoordinates(volume.nCross, {
        wall: volume.wallClustering,
        shock: volume.shockClustering,
        shockPos: 0.3,
        shockWidth: volume.shockWidth,
      }),
    ).maxAdjacentRatio;
    if (ratio > MAX_SMOOTH_STRETCH) {
      warnings.push(
        `Aggressive clustering: neighbouring cell sizes differ by up to ${ratio.toFixed(1)}x (> ${MAX_SMOOTH_STRETCH}). Add cells across the layer or lower the clustering strength, or expect a noisier shock.`,
      );
    }
  }
  // Work cap applies to the larger of the wedge-anchor march and the two symmetry-plane wall solves.
  const workOf = (v: VolumeRefinement) => {
    const c = estimateCostOf(v);
    return Math.max(c.wedgeWork, c.stripWork);
  };
  let work = workOf(volume);
  if (work > L.volume.maxWork) {
    const k = L.volume.maxWork / work;
    const it = Math.max(L.volume.iters[0], Math.floor(volume.iters * k));
    warnings.push(
      `Work cap ${(L.volume.maxWork / 1e6).toFixed(0)} M cell-updates: iterations lowered ${volume.iters} -> ${it}.`,
    );
    volume.iters = it;
    clamped = true;
    work = workOf(volume);
    while (
      work > L.volume.maxWork &&
      (volume.nStream > L.volume.nStream[0] || volume.nCross > L.volume.nCross[0])
    ) {
      volume.nStream = Math.max(L.volume.nStream[0], Math.floor(volume.nStream * 0.9));
      volume.nCross = Math.max(L.volume.nCross[0], Math.floor(volume.nCross * 0.9));
      work = workOf(volume);
    }
    warnings.push(
      `Work cap: grid reduced to ${volume.nStream} x ${volume.nCross}, ${volume.iters} iterations.`,
    );
  }
  return { settings: { preset: "custom", surface, volume }, warnings, clamped };
}

/** True when these settings must reproduce the legacy grids bit for bit. */
export function isBaseline(r?: RefinementSettings | null): boolean {
  if (!r) return true;
  const m = MEDIUM;
  const v = r.volume;
  return (
    v.nStream === m.volume.nStream &&
    v.nCross === m.volume.nCross &&
    v.iters === m.volume.iters &&
    v.wallClustering === 0 &&
    v.shockClustering === 0 &&
    !v.adaptive
  );
}

export function isSurfaceBaseline(r?: RefinementSettings | null): boolean {
  return !r || r.surface.leClustering === 0;
}

/**
 * Symmetry-plane (Rust kernel class) grid for a setting.
 * - Baseline (`medium`): exactly the shipped kernel, 88x28 cells x 42 iterations. That run is far from
 *   converged (see NOTES.md); it is kept only so the default behaviour does not change.
 * - Anything else: cells scale from the 72x24 wedge anchor (x 88/72, x 28/24) and the iteration count
 *   is the user's `iters` (the march needs several domain crossings to settle).
 */
export function stripDims(v: VolumeRefinement): { ni: number; nj: number; iters: number } {
  const a = LEGACY.wedge;
  const s = LEGACY.strip;
  if (isBaseline({ preset: "medium", surface: MEDIUM.surface, volume: v }))
    return { ni: s.ni, nj: s.nj, iters: s.iters };
  return {
    ni: Math.max(8, Math.round((s.ni * v.nStream) / a.ni)),
    nj: Math.max(6, Math.round((s.nj * v.nCross) / a.nj)),
    iters: v.iters,
  };
}

/**
 * Spanwise points above this trip pre-existing non-manifold aft-tip edges in generate.ts / mesh.ts weldAft
 * (tolerance 8e-4 L is larger than the tip spacing). Measured on b5fdccc: caret is manifold at ny <= 32 and
 * not at ny >= 34. Streamwise points (nx) are not affected. See NOTES.md.
 */
export const SPAN_MANIFOLD_SAFE_NY = 32;

export function surfaceWarnings(r: RefinementSettings): string[] {
  const out: string[] = [];
  if (r.surface.ny > SPAN_MANIFOLD_SAFE_NY) {
    out.push(
      `Spanwise points ${r.surface.ny} > ${SPAN_MANIFOLD_SAFE_NY}: several families currently report non-manifold aft-tip edges at this density (existing weld tolerance; still watertight). Check the CAD tab.`,
    );
  }
  return out;
}

export interface CostEstimate {
  surfacePointsPerSheet: number;
  /** Rough triangle count of the closed vehicle (two loft sheets + LE + caps). Measured counts are in NOTES.md. */
  surfaceTrianglesApprox: number;
  wedgeCells: number;
  wedgeWork: number;
  wedgeSeconds: number;
  stripCells: number;
  stripWork: number;
  /** Two wall solves (upper + lower) per aero evaluation (wasm for the baseline, TypeScript otherwise). */
  stripSeconds: number;
  withinCaps: boolean;
}

function estimateCostOf(v: VolumeRefinement) {
  const passes = v.adaptive ? 1.5 : 1;
  const wedgeCells = v.nStream * v.nCross;
  const wedgeWork = wedgeCells * v.iters * passes;
  const d = stripDims(v);
  const stripCells = d.ni * d.nj;
  const stripWork = 2 * stripCells * d.iters * passes;
  return { wedgeCells, wedgeWork, stripCells, stripWork };
}

export function estimateCost(r: RefinementSettings): CostEstimate {
  const { wedgeCells, wedgeWork, stripCells, stripWork } = estimateCostOf(r.volume);
  const L = REFINEMENT_LIMITS;
  return {
    surfacePointsPerSheet: r.surface.nx * r.surface.ny,
    surfaceTrianglesApprox: Math.round(4 * (r.surface.nx - 1) * (r.surface.ny - 1) * 1.04),
    wedgeCells,
    wedgeWork,
    wedgeSeconds: wedgeWork / COST_MODEL.jsRateWedge,
    stripCells,
    stripWork,
    stripSeconds: stripWork / (isBaseline(r) ? COST_MODEL.wasmRateStrip : COST_MODEL.jsRateStrip),
    withinCaps:
      r.surface.nx * r.surface.ny <= L.surface.maxPoints &&
      wedgeCells <= L.volume.maxCells &&
      Math.max(wedgeWork, stripWork) <= L.volume.maxWork,
  };
}

export function describeCost(r: RefinementSettings): string {
  const c = estimateCost(r);
  const sec = (x: number) =>
    x < 0.095 ? "<0.1 s" : x < 10 ? `${x.toFixed(1)} s` : `${Math.round(x)} s`;
  const d = stripDims(r.volume);
  return `Surface ${r.surface.nx}x${r.surface.ny} (~${c.surfaceTrianglesApprox.toLocaleString("en-US")} tris) · Euler ${d.ni}x${d.nj}x${d.iters} on the symmetry plane (~${sec(c.stripSeconds)}), wedge anchor ${r.volume.nStream}x${r.volume.nCross}x${r.volume.iters} (${c.wedgeCells.toLocaleString("en-US")} cells, ~${sec(c.wedgeSeconds)})`;
}

// ---------------------------------------------------------------------------
// Clustering
// ---------------------------------------------------------------------------

export interface ClusterSpec {
  /** Monitor amplitude at the wall (s = 0). 0 = off. */
  wall: number;
  /** Monitor amplitude at the shock (s = shockPos). 0 = off. */
  shock: number;
  /** Shock position in [0, 1]. */
  shockPos: number;
  /** Gaussian width in s units. */
  shockWidth: number;
  /** Exponential decay length of the wall term in s units. */
  wallDecay?: number;
}

const CLUSTER_SAMPLES = 1024;

/**
 * Equidistribute the monitor m(s) = 1 + wall*exp(-s/lw) + shock*exp(-((s-c)/w)^2).
 * Returns n+1 strictly increasing coordinates from exactly 0 to exactly 1; local cell
 * size is proportional to 1/m, so spacing is smallest at the wall and at the shock.
 * With wall = shock = 0 this is exactly j/n. The same algorithm is implemented in
 * native/cuspis-euler (cluster_coordinates) and cross-checked in the tests.
 */
export function clusterCoordinates(n: number, spec: ClusterSpec): Float64Array {
  const out = new Float64Array(n + 1);
  if (!(spec.wall > 0) && !(spec.shock > 0)) {
    for (let j = 0; j <= n; j++) out[j] = j / n;
    return out;
  }
  const M = Math.max(CLUSTER_SAMPLES, 8 * n);
  const lw = spec.wallDecay ?? 0.12;
  const c = Math.min(Math.max(spec.shockPos, 0), 1);
  const w = Math.max(spec.shockWidth, 1e-3);
  const cum = new Float64Array(M + 1);
  let prev = monitor(0, spec.wall, lw, spec.shock, c, w);
  for (let k = 1; k <= M; k++) {
    const m = monitor(k / M, spec.wall, lw, spec.shock, c, w);
    cum[k] = cum[k - 1] + (0.5 * (prev + m)) / M;
    prev = m;
  }
  const total = cum[M];
  let k = 0;
  out[0] = 0;
  for (let j = 1; j < n; j++) {
    const target = (total * j) / n;
    while (k < M - 1 && cum[k + 1] < target) k++;
    const span = cum[k + 1] - cum[k];
    const t = span > 0 ? (target - cum[k]) / span : 0;
    out[j] = (k + t) / M;
  }
  out[n] = 1;
  return out;
}

function monitor(s: number, wall: number, lw: number, shock: number, c: number, w: number): number {
  const d = (s - c) / w;
  return 1 + wall * Math.exp(-s / lw) + shock * Math.exp(-d * d);
}

/** Largest ratio of adjacent spacings (smoothness) and smallest / largest spacing of a coordinate set. */
export function spacingStats(x: ArrayLike<number>): {
  strictlyIncreasing: boolean;
  minSpacing: number;
  maxSpacing: number;
  maxAdjacentRatio: number;
} {
  let inc = true;
  let mn = Infinity;
  let mx = 0;
  let ratio = 1;
  for (let i = 1; i < x.length; i++) {
    const d = x[i] - x[i - 1];
    if (!(d > 0)) inc = false;
    mn = Math.min(mn, d);
    mx = Math.max(mx, d);
    if (i > 1) {
      const p = x[i - 1] - x[i - 2];
      if (p > 0 && d > 0) ratio = Math.max(ratio, d / p, p / d);
    }
  }
  return { strictlyIncreasing: inc, minSpacing: mn, maxSpacing: mx, maxAdjacentRatio: ratio };
}

/**
 * Smooth one-sided stretching of a monotone fraction set toward 0 (the leading edge / nose).
 * Endpoints are kept exactly; the index map u(t) = (e^{at}-1)/(e^{a}-1) is smooth and strictly
 * increasing, and the fractions are re-read through it, so the result stays strictly increasing
 * whenever the input is. strength 0 returns the input untouched (same array).
 */
export function clusterFractionsToLE(fracs: Float64Array, strength: number): Float64Array {
  const k = fracs.length - 1;
  if (!(strength > 0) || k < 2) return fracs;
  const a = 2.6 * Math.min(Math.max(strength, 0), 1);
  const den = Math.expm1(a);
  const out = new Float64Array(fracs.length);
  out[0] = fracs[0];
  for (let i = 1; i < k; i++) {
    const u = (Math.expm1((a * i) / k) / den) * k;
    const lo = Math.min(Math.floor(u), k - 1);
    const t = u - lo;
    out[i] = fracs[lo] + (fracs[lo + 1] - fracs[lo]) * t;
  }
  out[k] = fracs[k];
  return out;
}

// ---------------------------------------------------------------------------
// Shock position estimate (for static shock-aligned clustering)
// ---------------------------------------------------------------------------

/** Billig (1967) sphere stand-off / nose radius, M-dependent: 0.143 exp(3.24/M^2). */
export function billigStandoffOverR(mach: number): number {
  return 0.143 * Math.exp(3.24 / Math.max(mach * mach, 1));
}

/**
 * Predicted shock height above a wall polyline, measured in z at each query x.
 * `side` is +1 when the flow domain lies above the wall (upper surface), -1 below.
 * Straight-wedge theory from the nose with the mean wall angle from the nose as the
 * turning angle (exact for a wedge, an estimate for curved walls), plus the Billig
 * nose stand-off decaying over ~12 nose radii. Detached cases use a normal-shock-like
 * slope. This is an estimate used only to place grid points, never to set the flow.
 */
export function estimateShockHeights(
  wallX: ArrayLike<number>,
  wallZ: ArrayLike<number>,
  queryX: ArrayLike<number>,
  mach: number,
  gamma: number,
  alpha: number,
  side: 1 | -1,
  noseRadius: number,
): Float64Array {
  const n = wallX.length;
  const x0 = wallX[0];
  const z0 = wallZ[0];
  const out = new Float64Array(queryX.length);
  const zAt = (x: number) => {
    if (x <= wallX[0]) return wallZ[0];
    if (x >= wallX[n - 1]) return wallZ[n - 1];
    let lo = 0;
    let hi = n - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (wallX[mid] <= x) lo = mid;
      else hi = mid;
    }
    const dx = wallX[hi] - wallX[lo];
    const t = dx > 1e-14 ? (x - wallX[lo]) / dx : 0;
    return wallZ[lo] + (wallZ[hi] - wallZ[lo]) * t;
  };
  const dn = billigStandoffOverR(mach) * Math.max(noseRadius, 0);
  for (let q = 0; q < queryX.length; q++) {
    const x = queryX[q];
    const run = x - x0;
    if (run <= 1e-9) {
      out[q] = dn;
      continue;
    }
    const slope = (zAt(x) - z0) / run;
    const theta = Math.max(side * (Math.atan(slope) - alpha), 1e-3);
    let beta = betaFromThetaM(mach, theta, gamma);
    if (!Number.isFinite(beta))
      beta = Math.min(1.45, Math.asin(Math.min(1, 1 / mach)) + 1.6 * theta);
    const hw = side * (Math.tan(alpha + side * beta) - Math.tan(alpha + side * theta)) * run;
    const nose = dn * Math.exp(-run / (12 * Math.max(noseRadius, 1e-6)));
    out[q] = Math.max(hw, 0) + nose;
  }
  return out;
}

/** Convert a shock height to the clustering coordinate s for a legacy wall-stretch exponent p (eta = s^p). */
export function shockFractionFromHeight(height: number, farField: number, wallExp: number): number {
  const eta = Math.min(Math.max(height / Math.max(farField, 1e-12), 0.04), 0.92);
  return Math.pow(eta, 1 / wallExp);
}

/**
 * Per-column shock position (s in [0,1]) from a converged-ish cell field: the cell row with
 * the steepest density jump in each column, refined to sub-cell position by the half-jump
 * crossing, then smoothed along the columns. Returns NaN for columns with no clear jump
 * (caller falls back to the static estimate there).
 * rho is cell-centred, indexed j*ni + i; cell centres in s are (j+0.5)/nj for a uniform
 * map; `sOfRow` maps the row index to the actual s of the cell centre in that column.
 */
export function detectShockFractions(
  rho: ArrayLike<number>,
  ni: number,
  nj: number,
  sOfRow: (i: number, j: number) => number,
  rhoInf: number,
  minJump = 0.35,
): Float64Array {
  const raw = new Float64Array(ni).fill(NaN);
  for (let i = 0; i < ni; i++) {
    // Scan from the far field (j = nj-1, freestream) toward the wall, find where density first rises.
    const rhoMax = (() => {
      let m = 0;
      for (let j = 0; j < nj; j++) m = Math.max(m, rho[j * ni + i]);
      return m;
    })();
    if (rhoMax < rhoInf * (1 + minJump)) continue;
    const half = rhoInf + 0.5 * (rhoMax - rhoInf);
    for (let j = nj - 2; j >= 0; j--) {
      if (rho[j * ni + i] >= half && rho[(j + 1) * ni + i] < half) {
        const a = rho[j * ni + i];
        const b = rho[(j + 1) * ni + i];
        const t = (a - half) / Math.max(a - b, 1e-12);
        const s0 = sOfRow(i, j);
        const s1 = sOfRow(i, j + 1);
        raw[i] = s0 + (s1 - s0) * t;
        break;
      }
    }
  }
  return raw;
}

/** Fill NaN gaps by nearest valid neighbour, then smooth with a 3-point filter (end values held). */
export function smoothFractions(
  raw: Float64Array,
  fallback: ArrayLike<number>,
  passes = 6,
): Float64Array {
  const n = raw.length;
  const out = new Float64Array(n);
  for (let i = 0; i < n; i++) out[i] = Number.isFinite(raw[i]) ? raw[i] : fallback[i];
  for (let p = 0; p < passes; p++) {
    const t = out.slice();
    for (let i = 1; i < n - 1; i++) out[i] = 0.25 * t[i - 1] + 0.5 * t[i] + 0.25 * t[i + 1];
  }
  return out;
}
