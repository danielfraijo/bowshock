/**
 * Euler quality tiers for the symmetry-plane solver. Tier names are what the live pipeline and the worker use;
 * each maps onto the mesh-refinement kernel (`euler_solve_ex`: wall + shock clustering, convergence stop, grid
 * sequencing). `legacy` is the original shipped kernel (88x28 cells, 42 fixed sweeps: unconverged, 33-93 % wall-Cp
 * error on wedges) and stays the library default so existing bit-for-bit baselines keep holding; the app's
 * pipeline selects a tier per request mode. Measurements: NOTES.md "Iteration default".
 */
import { presetSettings, type RefinementSettings, type VolumeRefinement } from "../waverider/refinement";
import type { EulerQuality } from "./euler-kernel";

const v = (nStream: number, nCross: number, iters: number, tol: number, sequence: number): VolumeRefinement => ({
  nStream,
  nCross,
  iters,
  wallClustering: 2,
  shockClustering: 6,
  shockWidth: 0.09,
  adaptive: false,
  tol,
  sequence,
});

/** volume settings per tier (nStream/nCross are the 72x24-anchor units that stripDims scales to the 88x28 kernel). */
export const QUALITY_VOLUMES: Record<Exclude<EulerQuality, "legacy">, VolumeRefinement> = {
  /** 35 x 11 cells, tol 1e-2: live drag preview, ~10-12 ms per wall solve in wasm */
  preview: v(29, 9, 400, 1e-2, 1),
  /** 88 x 28 cells, tol 1e-2, one coarse level: the default after a release (~4.5 % mean wall-Cp vs reference, ~90 ms/solve) */
  standard: v(72, 24, 1500, 1e-2, 1),
  /** 88 x 28 cells, the app's `converged` preset */
  converged: presetSettings("converged").volume,
  /** 117 x 37 cells, tight tolerance: ground truth for the studies */
  reference: v(96, 32, 4000, 1e-4, 2),
};

const cache = new Map<string, RefinementSettings>();

/** Settings for a tier, or undefined for `legacy` (= no refinement object, the shipped kernel). */
export function qualitySettings(q: EulerQuality): RefinementSettings | undefined {
  if (q === "legacy") return undefined;
  let s = cache.get(q);
  if (!s) {
    s = { preset: "custom", surface: presetSettings("medium").surface, volume: { ...QUALITY_VOLUMES[q] } };
    cache.set(q, s);
  }
  return s;
}
