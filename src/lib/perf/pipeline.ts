/**
 * Staged, content-addressed compute pipeline.
 *
 *   params ──► [geometry]  key = hash(all params except ANALYSIS_ONLY)   → BuiltVehicle   (LRU)
 *          └─► [study]     key = hash(all params) + mode + kernel version → StudyResult    (LRU)
 *                 └─ inside: panelAero memo, Euler wall-Cp LRU (perf/euler-kernel)
 *
 * Moving Mach / alpha / altitude / CG therefore never rebuilds geometry; a cosmetic/view option is
 * not a DesignParams field at all, so it can never trigger a recompute. The geometry key is built
 * by DENY-list (fields known not to be read by buildVehicle). A new DesignParams field added by a
 * sibling is treated as geometric until someone proves otherwise, so a stale mesh is impossible.
 * perf.test.ts proves the deny-list: perturbing each listed field leaves buildVehicle's output
 * bit-identical.
 */
import { buildVehicle } from "../waverider/generate";
import { studyVehicle, type StudyResult } from "../waverider/study";
import { bindCenterline } from "../waverider/sym-euler";
import type { BuiltVehicle, DesignParams } from "../waverider/types";
import { Hasher } from "./hash";
import { KERNEL_VERSION, kernelFlags, type EulerQuality } from "./euler-kernel";
import { LRU } from "./lru";
import { withoutCosmetic } from "./cosmetic";

/** Fields that buildVehicle never reads (verified in perf.test.ts). */
export const ANALYSIS_ONLY: readonly (keyof DesignParams)[] = [
  "name",
  "unit",
  "lockFlight",
  "flightMach",
  "alphaDeg",
  "betaDeg",
  "gammaDeg",
  "altKm",
  "cgFrac",
  "aeroMethod",
  "twK",
  "fuel",
  "phi",
  "rhoKgM3",
  "massKg",
];

/**
 * preview / quick: reduced study at the `preview` Euler tier (drag). full: complete study at `standard`.
 * refine: complete study at `converged` (background refinement after a release).
 */
export type StudyMode = "preview" | "quick" | "full" | "refine";

export const MODE_QUALITY: Record<StudyMode, EulerQuality> = { preview: "preview", quick: "preview", full: "standard", refine: "converged" };

export function geometryKey(p: DesignParams): string {
  const skip = new Set<string>(ANALYSIS_ONLY as readonly string[]);
  const o: Record<string, unknown> = {};
  for (const k of Object.keys(p)) if (!skip.has(k)) o[k] = (p as unknown as Record<string, unknown>)[k];
  return new Hasher().str("geom").value(o).digest();
}

export function studyKey(p: DesignParams, mode: StudyMode): string {
  return new Hasher().str("study").str(KERNEL_VERSION).str(mode).str(MODE_QUALITY[mode]).value(withoutCosmetic(p)).digest();
}

function builtBytes(b: BuiltVehicle): number {
  let n = b.mesh.positions.byteLength + b.mesh.indices.byteLength + b.mesh.surfaces.byteLength + (b.mesh.tags?.byteLength ?? 0);
  for (const g of b.grids) n += g.xyz.byteLength;
  for (const g of b.shockGrids) n += g.xyz.byteLength;
  return n;
}

function studyBytes(s: StudyResult): number {
  const a = s.aero;
  let n = 0;
  for (const k of ["cp", "heat", "twEq", "stanton", "machE", "cf", "impact"] as const) n += (a[k] as Float32Array | undefined)?.byteLength ?? 0;
  return n + 4096;
}

export interface StudyOutcome {
  geomKey: string;
  studyKey: string;
  mode: StudyMode;
  built: BuiltVehicle;
  study: StudyResult;
  /** true when the stage was served from cache */
  hit: { geometry: boolean; study: boolean };
  ms: { geometry: number; study: number; total: number };
}

export class Pipeline {
  readonly geometry = new LRU<BuiltVehicle>(12, 96 * 1024 * 1024);
  readonly studies = new LRU<StudyResult>(24, 96 * 1024 * 1024);

  /** Force one Euler tier for every mode (benchmarks, the CLI `--quality`); undefined = MODE_QUALITY. */
  constructor(private quality?: EulerQuality) {}

  build(params: DesignParams): { built: BuiltVehicle; key: string; hit: boolean } {
    const key = geometryKey(params);
    const cached = this.geometry.get(key);
    if (cached) return { built: cached.params === params ? cached : { ...cached, params }, key, hit: true };
    const built = buildVehicle(params);
    this.geometry.set(key, built, builtBytes(built));
    return { built, key, hit: false };
  }

  run(params: DesignParams, mode: StudyMode): StudyOutcome {
    const t0 = performance.now();
    const g = this.build(params);
    const t1 = performance.now();
    const sk = studyKey(params, mode);
    let study = this.studies.get(sk);
    const studyHit = !!study;
    if (!study) {
      const prev = kernelFlags.quality;
      kernelFlags.quality = this.quality ?? MODE_QUALITY[mode];
      try {
        study = studyVehicle(g.built, { quick: mode === "preview" || mode === "quick" });
      } finally {
        kernelFlags.quality = prev;
      }
      this.studies.set(sk, study, studyBytes(study));
    }
    const t2 = performance.now();
    return {
      geomKey: g.key,
      studyKey: sk,
      mode,
      built: g.built,
      study,
      hit: { geometry: g.hit, study: studyHit },
      ms: { geometry: t1 - t0, study: t2 - t1, total: t2 - t0 },
    };
  }

  clear() {
    this.geometry.clear();
    this.studies.clear();
  }

  stats() {
    return { geometry: this.geometry.stats(), studies: this.studies.stats() };
  }
}

export const defaultPipeline = new Pipeline();

/**
 * A BuiltVehicle that crossed a structured-clone boundary lost its (WeakMap-keyed) symmetry-plane
 * centerline binding; this restores it so main-thread studies/exports on the clone keep the Euler wall.
 */
export function rebindCenterline(built: BuiltVehicle): BuiltVehicle {
  const upper = built.grids.find((g) => g.name === "upper" || g.name === "cowl");
  const lower = built.grids.find((g) => g.name === "lower");
  if (upper && lower) bindCenterline(built.mesh, upper, lower, built.params.span);
  return built;
}
