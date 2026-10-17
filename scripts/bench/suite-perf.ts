/**
 * Benchmarks that only exist once the perf layer is present (new APIs): staged pipeline with
 * dependency tracking, preview/full modes, cache hits and the async client round trip.
 * They complement (never replace) the like-for-like entries in suite.ts.
 */
import type { BenchSpec } from "./harness";
import { paramsFor, slide } from "./suite";
import { Pipeline } from "../../src/lib/perf/pipeline";
import { ComputeClient } from "../../src/lib/perf/client";
import type { WaveriderFamily } from "../../src/lib/waverider/types";

export function perfSuite(): BenchSpec[] {
  const specs: BenchSpec[] = [];
  for (const fam of ["caret", "osculating", "liftbody", "scramjet"] as WaveriderFamily[]) {
    specs.push({
      name: `pipeline/geom-slider-drag/${fam}`,
      group: "pipeline",
      note: "Pipeline.run(preview): geometry rebuild + coarse-Euler study (what the worker does while a geometry slider drags)",
      maxRuns: 40,
      setup: () => {
        const pl = new Pipeline();
        return (i) => pl.run(paramsFor(fam, { span: slide(i, 1.6, 2.6) }), "preview");
      },
    });
    specs.push({
      name: `pipeline/geom-slider-release/${fam}`,
      group: "pipeline",
      note: "Pipeline.run(full): geometry + full study on release",
      maxRuns: 15,
      setup: () => {
        const pl = new Pipeline();
        return (i) => pl.run(paramsFor(fam, { span: slide(i, 1.6, 2.6) }), "full");
      },
    });
    specs.push({
      name: `pipeline/flight-slider-drag/${fam}`,
      group: "pipeline",
      note: "Pipeline.run(preview) with only alpha changing: geometry stage served from cache",
      maxRuns: 40,
      setup: () => {
        const pl = new Pipeline();
        pl.run(paramsFor(fam), "preview");
        return (i) => pl.run(paramsFor(fam, { alphaDeg: slide(i, 0, 6) }), "preview");
      },
    });
    specs.push({
      name: `pipeline/flight-slider-release/${fam}`,
      group: "pipeline",
      note: "Pipeline.run(full), alpha only: cached geometry, full study",
      maxRuns: 20,
      setup: () => {
        const pl = new Pipeline();
        pl.run(paramsFor(fam), "full");
        return (i) => pl.run(paramsFor(fam, { alphaDeg: slide(i, 0, 6) }), "full");
      },
    });
  }
  specs.push({
    name: "pipeline/repeat-identical(full,caret)",
    group: "pipeline",
    note: "same params again (e.g. toggling back, undo): full study served from the content-addressed cache",
    maxRuns: 200,
    setup: () => {
      const pl = new Pipeline();
      pl.run(paramsFor("caret"), "full");
      return () => pl.run(paramsFor("caret"), "full");
    },
  });
  specs.push({
    name: "pipeline/cosmetic-change(caret)",
    group: "pipeline",
    note: "name/unit change: nothing recomputed (geometry + study hits)",
    maxRuns: 200,
    setup: () => {
      const pl = new Pipeline();
      pl.run(paramsFor("caret"), "full");
      return (i) => pl.run(paramsFor("caret", { name: `n${i}` }), "full");
    },
  });
  specs.push({
    name: "client/roundtrip-preview(caret,structured-clone)",
    group: "client",
    note: "ComputeClient.study(preview) over the full protocol incl. structuredClone of request+reply (worker transfer cost model)",
    maxRuns: 40,
    setup: () => {
      const c = new ComputeClient({ runner: "inline", clone: true });
      return async (i) => {
        const r = await c.study(paramsFor("caret", { span: slide(i, 1.6, 2.6) }), "preview");
        return r.study;
      };
    },
  });
  return specs;
}
