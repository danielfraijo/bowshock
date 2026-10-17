/**
 * React hook: live geometry + study for the Designer, computed off the main thread.
 *
 *   const { built, study, busy, quality } = useLiveVehicle(params, { dragging, hydrated });
 *
 * - First render (SSR and hydration) is computed synchronously, exactly like before, so server and
 *   client markup agree.
 * - After hydration every change is sent to the worker: while dragging, a coarse "preview" study on the
 *   fast lane (latest-wins); when released, the full study on the slow lane. Geometry is only rebuilt
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
  /** a newer job is in flight */
  busy: boolean;
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
  const held = useRef(initial.geomKey);
  const latest = useRef(0);
  const key = useMemo(() => new Hasher().value(params).digest(), [params]);
  const mode: StudyMode = opts.dragging ? "preview" : "full";

  useEffect(() => {
    if (!opts.hydrated) return;
    const client = getComputeClient();
    const ticket = ++latest.current;
    let alive = true;
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

  return { built: state.built, study: state.study, busy, quality: state.quality, lastMs: state.lastMs };
}
