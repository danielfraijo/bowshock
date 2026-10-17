/**
 * React hook: live geometry + study for the Designer, computed off the main thread.
 *
 *   const { built, study, busy, quality } = useLiveVehicle(params, { dragging, hydrated });
 *
 * - First render (SSR and hydration) is computed synchronously, exactly like before, so server and
 *   client markup agree.
 * - After hydration every change is sent to the worker: while dragging, a reduced "preview" study at the
 *   coarse Euler tier on the fast lane (latest-wins); when released, the full study at the standard tier on
 *   the slow lane, then a converged-tier refinement on the preemptible refine lane. Geometry is only rebuilt
 *   when a geometry field changed (pipeline.geometryKey); flight changes reuse the cached mesh.
 * - {built, study} always come from the same job, so per-triangle arrays always match the mesh.
 *   While a newer job is running the previous coherent pair stays on screen.
 * - Where Workers are unavailable the same code runs in-process (loopback lane).
 */
import { useEffect, useMemo, useRef, useState } from "react";
import type { StudyResult } from "../waverider/study";
import type { BuiltVehicle, DesignParams } from "../waverider/types";
import { getComputeClient, SupersededError } from "./client";
import { Hasher } from "./hash";
import { defaultPipeline, rebindCenterline, type StudyMode } from "./pipeline";

export interface LiveVehicle {
  built: BuiltVehicle;
  study: StudyResult;
  /** a newer job is in flight (preview / full) */
  busy: boolean;
  /** the converged background refinement of the shown design is running */
  refining: boolean;
  /** mode of the study currently shown */
  quality: StudyMode;
  /** worker round-trip timings of the last applied result (ms) */
  lastMs: number;
}

export function useLiveVehicle(params: DesignParams, opts: { dragging: boolean; hydrated: boolean }): LiveVehicle {
  const initial = useMemo(() => {
    const out = defaultPipeline.run(params, "quick");
    return { built: out.built, study: out.study, geomKey: out.geomKey, quality: "quick" as StudyMode, lastMs: out.ms.total };
  }, []); // eslint-disable-line react-hooks/exhaustive-deps
  const [state, setState] = useState(initial);
  const [busy, setBusy] = useState(false);
  const [refining, setRefining] = useState(false);
  const held = useRef(initial.geomKey);
  const latest = useRef(0);
  const key = useMemo(() => new Hasher().value(params).digest(), [params]);
  const mode: StudyMode = opts.dragging ? "preview" : "full";

  useEffect(() => {
    if (!opts.hydrated) return;
    const client = getComputeClient();
    const ticket = ++latest.current;
    let alive = true;
    client.cancelRefine(); // any new interaction kills the old background refinement
    setRefining(false);
    setBusy(true);
    const t0 = performance.now();
    client
      .study(params, mode, held.current)
      .then((r) => {
        if (!alive || ticket !== latest.current) return;
        held.current = r.geomKey;
        setState((prev) => {
          const built = r.built ? rebindCenterline({ ...r.built, params }) : { ...prev.built, params };
          return { built, study: r.study, geomKey: r.geomKey, quality: r.mode, lastMs: performance.now() - t0 };
        });
        setBusy(false);
        if (r.mode === "full") {
          // progressive: converged-tier refinement of the released design in the background (latest-wins, preemptible)
          setRefining(true);
          client
            .study(params, "refine", held.current)
            .then((rr) => {
              if (!alive || ticket !== latest.current) return;
              setState((prev) => ({ ...prev, study: rr.study, quality: rr.mode, lastMs: performance.now() - t0 }));
              setRefining(false);
            })
            .catch((e) => {
              if (!(e instanceof SupersededError)) console.warn("refinement failed; keeping the standard result", e);
              if (alive && ticket === latest.current) setRefining(false);
            });
        }
      })
      .catch((e) => {
        if (e instanceof SupersededError) return; // a newer request owns `busy`
        if (alive && ticket === latest.current) {
          console.warn("compute worker failed, computing on the main thread", e);
          const out = defaultPipeline.run(params, mode === "preview" ? "quick" : "full");
          held.current = out.geomKey;
          setState({ built: out.built, study: out.study, geomKey: out.geomKey, quality: out.mode, lastMs: out.ms.total });
          setBusy(false);
        }
      });
    return () => {
      alive = false;
    };
  }, [key, mode, opts.hydrated]); // eslint-disable-line react-hooks/exhaustive-deps

  return { built: state.built, study: state.study, busy, refining, quality: state.quality, lastMs: state.lastMs };
}
