// [mesh-refinement] Non-blocking study hook: instant CBAERO-quality result, then the converged Euler solve in the background.
import { useEffect, useMemo, useRef, useState } from "react";
import { instantStudy, studyProgressive, usesEuler, type StudyStatus } from "../lib/waverider/progressive-study";
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
  const euler = usesEuler(built);
  // eslint-disable-next-line react-hooks/exhaustive-deps
  const instant = useMemo(() => instantStudy(built, quick), [built, quick, ...deps]);
  const [res, setRes] = useState<{ for: unknown; study: StudyResult; status: StudyStatus; error?: string } | null>(null);
  const gen = useRef(0);

  useEffect(() => {
    if (!hydrated) return;
    // Panel methods already finished inside the render. A second identical study
    // only delayed the numbers and kept the status line on "working".
    if (!euler) return;
    const token = { aborted: false };
    const my = ++gen.current;
    const t = setTimeout(
      async () => {
        try {
          let lastPaint = 0;
          const { study, status } = await studyProgressive(built, {
            quick: dragging,
            preview: dragging || liveOnly,
            signal: token,
            onUpdate: (next, st) => {
              if (token.aborted || my !== gen.current) return;
              const now = performance.now();
              const done = st.stage === "final" || st.stage === "fallback";
              if (!done && lastPaint !== 0 && now - lastPaint < 150) return;
              lastPaint = now;
              setRes({ for: instant, study: next, status: st });
            },
          });
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
  }, [instant, built, dragging, hydrated, liveOnly, euler]);

  const fresh = euler && res && res.for === instant ? res : null;
  const warnings = fresh ? fresh.status.warnings : instant.warning ? [instant.warning] : [];
  return {
    study: fresh ? fresh.study : instant.study,
    status: {
      stage: fresh ? fresh.status.stage : instant.warning ? "fallback" : euler ? "preview" : "final",
      fraction: fresh ? fresh.status.fraction : euler ? 0 : 1,
      report: fresh ? fresh.status.report : null,
      warnings,
      rounds: fresh ? fresh.status.rounds : euler ? 0 : 1,
      busy: euler && hydrated && (!fresh || (fresh.status.stage !== "final" && fresh.status.stage !== "fallback")),
      error: fresh?.error,
    },
  };
}
