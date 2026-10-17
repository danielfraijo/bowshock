/**
 * Grid-convergence study for the refinement presets, scored against the exact oblique shock.
 *
 *   bun run src/lib/waverider/refinement-study.ts            # prints the tables
 *
 * 1) Wedge anchor (euler2d.ts): post-shock pressure ratio and shock angle, M=8, 8 deg and 30 deg.
 * 2) Symmetry-plane kernel (strip-euler.ts / the shipped wasm for medium) on the same wedges.
 * 3) Surface mesh: triangles + watertight / manifold per preset for a family.
 */
import { buildVehicle } from "./generate";
import { eulerWedge } from "./euler2d";
import { DEG, betaFromThetaM, obliqueShock } from "./math";
import { analyzeMesh } from "./mesh";
import {
  LEGACY,
  estimateShockHeights,
  presetSettings,
  shockFractionFromHeight,
  stripDims,
  type RefinementPreset,
  type RefinementSettings,
  type VolumeRefinement,
} from "./refinement";
import { solveStrip, stripDomain, stripGridOf } from "./strip-euler";
import { DEFAULT_PARAMS, type WaveriderFamily } from "./types";

export interface WedgeRow {
  preset: RefinementPreset;
  thetaDeg: number;
  grid: string;
  cells: number;
  ms: number;
  p2p1: number;
  exactP: number;
  pErrPct: number;
  shockDeg: number;
  exactShockDeg: number;
  shockErrDeg: number;
  residual: number;
}

export function wedgeStudy(
  mach = 8,
  thetas = [8, 30],
  ids: Exclude<RefinementPreset, "custom">[] = ["coarse", "legacy", "medium", "fine"],
): WedgeRow[] {
  const rows: WedgeRow[] = [];
  for (const thetaDeg of thetas) {
    for (const id of ids) {
      const s = presetSettings(id);
      // Fresh solve every time so the timing is real (the module cache would hide it).
      const r = eulerWedge(mach, thetaDeg, 1.4, id === "legacy" ? undefined : s.volume);
      rows.push({
        preset: id,
        thetaDeg,
        grid: `${r.ni ?? LEGACY.wedge.ni}x${r.nj ?? LEGACY.wedge.nj}x${r.iters}`,
        cells: (r.ni ?? LEGACY.wedge.ni) * (r.nj ?? LEGACY.wedge.nj),
        ms: r.ms ?? NaN,
        p2p1: r.p2p1,
        exactP: r.exactP,
        pErrPct: r.relErr * 100,
        shockDeg: r.shockAngleDeg ?? NaN,
        exactShockDeg: r.exactShockAngleDeg ?? NaN,
        shockErrDeg: (r.shockAngleDeg ?? NaN) - (r.exactShockAngleDeg ?? NaN),
        residual: r.residual,
      });
    }
  }
  return rows;
}

export interface StripRow {
  preset: RefinementPreset;
  thetaDeg: number;
  grid: string;
  cells: number;
  ms: number;
  cp: number;
  exactCp: number;
  cpErrPct: number;
  shockDeg: number;
  exactShockDeg: number;
  residual: number;
}

export interface StripCaseResult {
  grid: string;
  cells: number;
  ms: number;
  cp: number;
  exactCp: number;
  cpErrPct: number;
  shockDeg: number;
  exactShockDeg: number;
  shockErrDeg: number;
  residual: number;
  iterations: number;
  converged: boolean;
  stopReason: string;
  /** Cell updates (all levels / passes). */
  work: number;
}

/**
 * One straight-wedge wall (L = 1, alpha 0): mean wall Cp over 0.3 < x < 0.8 and the fitted shock angle
 * vs the exact oblique shock, for any volume refinement (the same mapping the app uses).
 */
export function stripCase(mach: number, thetaDeg: number, vol: VolumeRefinement): StripCaseResult {
  const n = 80;
  const x = Array.from({ length: n }, (_, i) => i / (n - 1));
  const z = x.map((v) => v * Math.tan(thetaDeg * DEG));
  const beta = betaFromThetaM(mach, thetaDeg * DEG);
  const exactCp = (obliqueShock(mach, beta).p2p1 - 1) / (0.5 * 1.4 * mach * mach);
  const d = stripDims(vol);
  const dom = stripDomain(x, z);
  let shockS: Float64Array | null = null;
  if (vol.shockClustering > 0 || vol.adaptive) {
    const xs = new Float64Array(d.ni + 1).map((_, i) => dom.xmin + ((dom.xmax - dom.xmin) * i) / d.ni);
    const h = estimateShockHeights(x, z, xs, mach, 1.4, 0, 1, 0);
    shockS = h.map((v) => shockFractionFromHeight(v, dom.hFar, LEGACY.strip.wallExp));
  }
  const grid = stripGridOf(vol, shockS);
  const t0 = performance.now();
  const r = solveStrip(x, z, mach, 1.4, 0, grid);
  const ms = performance.now() - t0;
  let sum = 0;
  let c = 0;
  for (let i = 0; i < n; i++) {
    if (x[i] > 0.3 && x[i] < 0.8) {
      sum += r.cp[i];
      c++;
    }
  }
  const xs: number[] = [];
  const ys: number[] = [];
  for (let i = 0; i < r.shock.x.length; i++) {
    const xx = r.shock.x[i];
    if (xx > 0.3 && xx < 0.85 && Number.isFinite(r.shock.height[i])) {
      xs.push(xx);
      ys.push(r.shock.height[i] + xx * Math.tan(thetaDeg * DEG));
    }
  }
  let ang = NaN;
  if (xs.length > 3) {
    const mx = xs.reduce((a, b) => a + b) / xs.length;
    const my = ys.reduce((a, b) => a + b) / ys.length;
    let sxy = 0;
    let sxx = 0;
    xs.forEach((v, k) => {
      sxy += (v - mx) * (ys[k] - my);
      sxx += (v - mx) ** 2;
    });
    ang = Math.atan(sxy / sxx) / DEG;
  }
  return {
    grid: `${d.ni}x${d.nj}x${d.iters}`,
    cells: d.ni * d.nj,
    ms,
    cp: sum / c,
    exactCp,
    cpErrPct: (Math.abs(sum / c - exactCp) / exactCp) * 100,
    shockDeg: ang,
    exactShockDeg: beta / DEG,
    shockErrDeg: ang - beta / DEG,
    residual: r.residual,
    iterations: r.grid.itersUsed,
    converged: r.converged,
    stopReason: r.stopReason,
    work: r.work,
  };
}

/** Straight wedge wall, L=1, M given, alpha 0, for the named presets. */
export function stripStudy(
  mach = 8,
  thetas = [8, 30],
  ids: Exclude<RefinementPreset, "custom">[] = ["coarse", "legacy", "medium", "fine"],
): StripRow[] {
  const rows: StripRow[] = [];
  for (const thetaDeg of thetas) {
    for (const id of ids) {
      const set: RefinementSettings = presetSettings(id);
      const c = stripCase(mach, thetaDeg, set.volume);
      rows.push({
        preset: id,
        thetaDeg,
        grid: c.grid,
        cells: c.cells,
        ms: c.ms,
        cp: c.cp,
        exactCp: c.exactCp,
        cpErrPct: c.cpErrPct,
        shockDeg: c.shockDeg,
        exactShockDeg: c.exactShockDeg,
        residual: c.residual,
      });
    }
  }
  return rows;
}

export interface SurfaceRow {
  preset: RefinementPreset;
  family: WaveriderFamily;
  nx: number;
  ny: number;
  triangles: number;
  vertices: number;
  watertight: boolean;
  manifold: boolean;
  openEdges: number;
  nonManifoldEdges: number;
}

export function surfaceStudy(
  families: WaveriderFamily[] = ["caret"],
  ids: Exclude<RefinementPreset, "custom">[] = ["coarse", "medium", "fine"],
): SurfaceRow[] {
  const rows: SurfaceRow[] = [];
  for (const family of families) {
    for (const id of ids) {
      const r = presetSettings(id);
      const b = buildVehicle({
        ...DEFAULT_PARAMS,
        family,
        nx: r.surface.nx,
        ny: r.surface.ny,
        refinement: r,
      });
      const q = analyzeMesh(b.mesh);
      rows.push({
        preset: id,
        family,
        nx: r.surface.nx,
        ny: r.surface.ny,
        triangles: q.triangles,
        vertices: q.vertices,
        watertight: q.watertight,
        manifold: q.manifold,
        openEdges: q.openEdges,
        nonManifoldEdges: q.nonManifoldEdges,
      });
    }
  }
  return rows;
}

export function formatTables(): string {
  const f = (v: number, d = 2) => (Number.isFinite(v) ? v.toFixed(d) : "n/a");
  const lines: string[] = [];
  lines.push(
    "Wedge anchor (euler2d.ts), M=8, gamma=1.4 - pressure ratio and shock angle vs exact oblique shock",
  );
  lines.push(
    "theta preset  grid          cells  time_ms  p2/p1   exact   p_err%  beta_deg  exact  beta_err_deg",
  );
  for (const r of wedgeStudy())
    lines.push(
      `${String(r.thetaDeg).padEnd(5)} ${r.preset.padEnd(7)} ${r.grid.padEnd(13)} ${String(r.cells).padEnd(6)} ${f(r.ms, 0).padEnd(8)} ${f(r.p2p1, 3).padEnd(7)} ${f(r.exactP, 3).padEnd(7)} ${f(r.pErrPct).padEnd(7)} ${f(r.shockDeg).padEnd(9)} ${f(r.exactShockDeg).padEnd(6)} ${f(r.shockErrDeg)}`,
    );
  lines.push("");
  lines.push(
    "Symmetry-plane kernel on a straight wedge wall (medium = shipped 88x28x42), M=8: mean wall Cp 0.3<x<0.8 and shock angle",
  );
  lines.push("theta preset  grid          cells  time_ms  Cp      exact   Cp_err% beta_deg  exact");
  for (const r of stripStudy())
    lines.push(
      `${String(r.thetaDeg).padEnd(5)} ${r.preset.padEnd(7)} ${r.grid.padEnd(13)} ${String(r.cells).padEnd(6)} ${f(r.ms, 0).padEnd(8)} ${f(r.cp, 4).padEnd(7)} ${f(r.exactCp, 4).padEnd(7)} ${f(r.cpErrPct, 1).padEnd(7)} ${f(r.shockDeg).padEnd(9)} ${f(r.exactShockDeg)}`,
    );
  lines.push("");
  lines.push("Surface mesh, caret M8 default design");
  lines.push("preset  nx x ny  triangles  vertices  watertight  manifold  open  nonManifold");
  for (const r of surfaceStudy())
    lines.push(
      `${r.preset.padEnd(7)} ${`${r.nx}x${r.ny}`.padEnd(8)} ${String(r.triangles).padEnd(10)} ${String(r.vertices).padEnd(9)} ${String(r.watertight).padEnd(11)} ${String(r.manifold).padEnd(9)} ${String(r.openEdges).padEnd(5)} ${r.nonManifoldEdges}`,
    );
  return lines.join("\n");
}

if (import.meta.main) console.log(formatTables());
