// [mesh-refinement] Non-blocking study hook: instant CBAERO-quality result, then the converged Euler solve in the background.
import { useEffect, useMemo, useRef, useState } from "react";
import { instantStudy, studyProgressive, type StudyStatus } from "../lib/waverider/progressive-study";
import type { StudyResult } from "../lib/waverider/study";
import type { BuiltVehicle } from "../lib/waverider/types";

export interface UseStudy {
  study: StudyResult;
  /** "preview": numbers on screen are not yet the converged solve. */
  status: StudyStatus & { busy: boolean; error?: string };
}

/** Debounce before a background solve starts (ms): a slider tick that is followed by another never starts a solve. */
export const DRAG_DEBOUNCE_MS = 60;
export const SETTLE_DEBOUNCE_MS = 140;

export function useStudy(built: BuiltVehicle, deps: readonly unknown[], dragging: boolean, hydrated: boolean, liveOnly = false): UseStudy {
  const quick = dragging || !hydrated;
  // eslint-disable-next-line react-hooks/exhaustive-deps
  const instant = useMemo(() => instantStudy(built, quick), [built, quick, ...deps]);
  const [res, setRes] = useState<{ for: unknown; study: StudyResult; status: StudyStatus; error?: string } | null>(null);
  const gen = useRef(0);

  useEffect(() => {
    if (!hydrated) return;
    const token = { aborted: false };
    const my = ++gen.current;
    const t = setTimeout(
      async () => {
        try {
          const { study, status } = await studyProgressive(built, { quick: dragging, preview: dragging || liveOnly, signal: token });
          if (!token.aborted && my === gen.current) setRes({ for: instant, study, status });
        } catch (err) {
          if (token.aborted || (err instanceof DOMException && err.name === "AbortError")) return;
          // never let a background failure reach the render: keep the instant result and say so
          if (my === gen.current)
            setRes({ for: instant, study: instant.study, status: { stage: "fallback", fraction: 1, report: null, warnings: [], rounds: 0 }, error: err instanceof Error ? err.message : String(err) });
        }
      },
      dragging ? DRAG_DEBOUNCE_MS : SETTLE_DEBOUNCE_MS,
    );
    return () => {
      token.aborted = true;
      clearTimeout(t);
    };
  }, [instant, built, dragging, hydrated, liveOnly]);

  const fresh = res && res.for === instant ? res : null;
  const warnings = fresh ? fresh.status.warnings : instant.warning ? [instant.warning] : [];
  return {
    study: fresh ? fresh.study : instant.study,
    status: {
      stage: fresh ? fresh.status.stage : "preview",
      fraction: fresh ? fresh.status.fraction : 0,
      report: fresh ? fresh.status.report : null,
      warnings,
      rounds: fresh ? fresh.status.rounds : 0,
      busy: !fresh && hydrated,
      error: fresh?.error,
    },
  };
}
