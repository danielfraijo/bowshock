/**
 * [mesh-refinement] Progressive, non-blocking study driver.
 *
 * `studyVehicle` with the converged Euler method needs two wall solves per angle of attack and a full study visits ~20+
 * angles (derivatives, polar). Run synchronously inside a React render that froze the page for 4-11 s (measured in headless
 * Chrome, see NOTES.md). This driver gives the same numbers without the freeze:
 *
 *   round 0  study with Euler solves *deferred* -> CBAERO-quality numbers immediately (reported as a preview)
 *   solve    the deferred wall solves, in ~10 ms chunks that yield to the event loop, cancellable between chunks
 *   round 1+ study again: every solve is now a cache hit (sweep points can reveal new angles, so repeat, <= maxRounds)
 *
 * The final result equals the synchronous `studyVehicle` result exactly when the selected preset is also used for the sweep
 * points; for `fine` / expensive `custom` the sweep points (not the angle of attack you asked for) use the `medium` solver and
 * the report says so.
 */
import { isBaseline, MEDIUM, sweepOf } from "./refinement";
import { studyVehicle, type StudyResult } from "./study";
import { buildVehicle } from "./generate";
import { DEFAULT_PARAMS, type BuiltVehicle, type DesignParams } from "./types";
import { beginEulerCollect, endEulerCollect, solveEulerJobs, symmetryEulerReport, type EulerReport } from "./sym-euler";

export type StudyStage = "preview" | "solving" | "final" | "fallback";

export interface StudyStatus {
  stage: StudyStage;
  /** 0..1 over the pending wall solves of the current round. */
  fraction: number;
  /** Convergence readout of the primary angle of attack (null when Euler is not in use). */
  report: EulerReport | null;
  /** Human-readable problems that were absorbed (solver failed -> CBAERO; sweep cheaper than selection; ...). */
  warnings: string[];
  rounds: number;
}

export interface ProgressiveOptions {
  /** Quick study (no six-DOF sweeps / glide): used while dragging. */
  quick?: boolean;
  /** Use the coarse live-preview solver instead of the selected preset (only the primary angle is solved in quick mode). */
  preview?: boolean;
  signal?: { aborted: boolean };
  onUpdate?: (study: StudyResult, status: StudyStatus) => void;
  maxRounds?: number;
}

export function usesEuler(built: BuiltVehicle): boolean {
  return built.params.aeroMethod === "euler" && !isBaseline(built.params.refinement ?? MEDIUM);
}

function abortError() {
  return new DOMException("study aborted", "AbortError");
}

/** `studyVehicle` that never throws: on failure it reruns with CBAERO and says so. */
export function safeStudy(built: BuiltVehicle, quick: boolean): { study: StudyResult; warning?: string } {
  try {
    return { study: studyVehicle(built, { quick }) };
  } catch (err) {
    const msg = err instanceof Error ? err.message : String(err);
    const fallback: BuiltVehicle = { ...built, params: { ...built.params, aeroMethod: "cbaero" } };
    // if even CBAERO fails, let it throw: the caller's error boundary shows a message
    return { study: studyVehicle(fallback, { quick }), warning: `Analysis failed with the ${built.params.aeroMethod} method (${msg}); CBAERO numbers are shown.` };
  }
}

export async function studyProgressive(built: BuiltVehicle, opts: ProgressiveOptions = {}): Promise<{ study: StudyResult; status: StudyStatus }> {
  const quick = !!opts.quick;
  const warnings: string[] = [];
  const selected = built.params.refinement ?? MEDIUM;
  if (!usesEuler(built)) {
    const { study, warning } = safeStudy(built, quick);
    if (warning) warnings.push(warning);
    const status: StudyStatus = { stage: warning ? "fallback" : "final", fraction: 1, report: null, warnings, rounds: 1 };
    opts.onUpdate?.(study, status);
    return { study, status };
  }
  const cheaperSweep = !quick && !opts.preview && JSON.stringify(sweepOf(selected).volume) !== JSON.stringify(selected.volume);
  if (cheaperSweep)
    warnings.push("Stability derivatives and the polar use the medium solver; the angle of attack you set uses the selected preset.");

  const maxRounds = opts.maxRounds ?? 8;
  let study: StudyResult | null = null;
  let rounds = 0;
  for (;;) {
    if (opts.signal?.aborted) throw abortError();
    rounds++;
    beginEulerCollect({ primaryAlphaDeg: built.params.alphaDeg, preview: !!opts.preview });
    let jobs: ReturnType<typeof endEulerCollect> = [];
    let warning: string | undefined;
    try {
      const r = safeStudy(built, quick);
      study = r.study;
      warning = r.warning;
    } finally {
      jobs = endEulerCollect();
    }
    if (warning) {
      warnings.push(warning);
      const status: StudyStatus = { stage: "fallback", fraction: 1, report: null, warnings, rounds };
      opts.onUpdate?.(study!, status);
      return { study: study!, status };
    }
    const report = symmetryEulerReport();
    if (jobs.length === 0 || rounds > maxRounds) {
      if (jobs.length > 0) warnings.push("Some sweep points were not solved (iteration limit); CBAERO values stand in for them.");
      const status: StudyStatus = { stage: "final", fraction: 1, report: report && { ...report, preview: !!opts.preview && !isBaseline(selected), sweepCheaper: cheaperSweep }, warnings, rounds };
      opts.onUpdate?.(study!, status);
      return { study: study!, status };
    }
    opts.onUpdate?.(study!, { stage: rounds === 1 ? "preview" : "solving", fraction: 0, report, warnings, rounds });
    try {
      await solveEulerJobs(jobs, {
        signal: opts.signal,
        onProgress: (d, t) => opts.onUpdate?.(study!, { stage: "solving", fraction: t ? d / t : 1, report, warnings, rounds }),
      });
    } catch (err) {
      if (err instanceof DOMException && err.name === "AbortError") throw err;
      // a solver failure is absorbed: CBAERO stands, the user is told
      const msg = err instanceof Error ? err.message : String(err);
      warnings.push(`Euler solver failed (${msg}); CBAERO numbers are shown.`);
      const fb = safeStudy({ ...built, params: { ...built.params, aeroMethod: "cbaero" } }, quick).study;
      const status: StudyStatus = { stage: "fallback", fraction: 1, report: null, warnings, rounds };
      opts.onUpdate?.(fb, status);
      return { study: fb, status };
    }
  }
}

/**
 * Instant, never-throwing study for the first paint of a change: the Euler solves are deferred (CBAERO numbers, ~100 ms for a
 * full study, ~3 ms quick) and nothing is cached or left recording. Follow with `studyProgressive` for the converged result.
 */
export function instantStudy(built: BuiltVehicle, quick: boolean): { study: StudyResult; warning?: string } {
  if (!usesEuler(built)) return safeStudy(built, quick);
  beginEulerCollect({ primaryAlphaDeg: built.params.alphaDeg, preview: true });
  try {
    return safeStudy(built, quick);
  } finally {
    endEulerCollect();
  }
}

/**
 * `buildVehicle` that never throws: a shape the generator rejects keeps the last good vehicle on screen (or the default
 * design if there is none yet) and returns the message for the UI.
 */
export function safeBuild(params: DesignParams, lastGood?: BuiltVehicle | null): { built: BuiltVehicle; error?: string } {
  try {
    return { built: buildVehicle(params) };
  } catch (err) {
    const error = `This shape could not be built (${err instanceof Error ? err.message : String(err)}).`;
    return { built: lastGood ?? buildVehicle({ ...DEFAULT_PARAMS, refinement: params.refinement }), error };
  }
}
