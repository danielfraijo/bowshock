/**
 * Benchmarks for the integrated app (topology + mesh refinement + physics data pack + DATCOM). They use only APIs that
 * exist on origin/main, so the same file runs on the unoptimised tree (before) and on feat/performance (after).
 * Inputs vary per iteration so caches in the optimised tree cannot turn compute into lookup.
 */
import type { BenchSpec } from "./harness";
import { paramsFor, slide } from "./suite";
import { buildVehicle } from "../../src/lib/waverider/generate";
import { badEdges, topoMetrics, worstDihedrals } from "../../src/lib/waverider/geom-quality";
import { analyzeMesh } from "../../src/lib/waverider/mesh";
import { edgeDefects, triangleQuality } from "../../src/lib/waverider/mesh-repair";
import { flipForQuality, orientConsistent, removeDoubleFaces } from "../../src/lib/waverider/topology";
import { presetSettings } from "../../src/lib/waverider/refinement";
import { computeDataPack } from "../../src/lib/waverider/datapack";
import { runCases } from "../../src/lib/waverider/datcom/model";
import { PRESETS } from "../../src/lib/waverider/datcom/presets";
import { runValidation } from "../../src/lib/waverider/validate";
import { studyVehicle } from "../../src/lib/waverider/study";
import { buildZip, utf8 } from "../../src/lib/waverider/zip";
import { packFiles, packToJson } from "../../src/lib/waverider/datapack";
import type { WaveriderFamily } from "../../src/lib/waverider/types";

export function combinedSuite(): BenchSpec[] {
  const specs: BenchSpec[] = [];
  const fams: WaveriderFamily[] = ["caret", "osculating", "liftbody", "scramjet"];

  // ---- topology / geometry quality (Jobi)
  for (const fam of fams) {
    specs.push({
      name: `topo/quality-metrics/${fam}`,
      group: "topo",
      note: "geom-quality.topoMetrics + badEdges + worstDihedrals + mesh-repair edge/triangle checks on a built mesh",
      maxRuns: 60,
      setup: () => {
        const b = buildVehicle(paramsFor(fam, { nx: 64, ny: 48 }));
        return () => {
          topoMetrics(b.mesh);
          badEdges(b.mesh);
          worstDihedrals(b.mesh, 10);
          edgeDefects(b.mesh);
          return triangleQuality(b.mesh);
        };
      },
    });
    specs.push({
      name: `topo/analyze-mesh/${fam}`,
      group: "topo",
      maxRuns: 80,
      setup: () => {
        const b = buildVehicle(paramsFor(fam, { nx: 64, ny: 48 }));
        return () => analyzeMesh(b.mesh);
      },
    });
    specs.push({
      name: `topo/orient+flip+dedupe/${fam}`,
      group: "topo",
      maxRuns: 60,
      setup: () => {
        const b = buildVehicle(paramsFor(fam, { nx: 64, ny: 48 }));
        return () => {
          const m = { ...b.mesh, indices: b.mesh.indices.slice() };
          orientConsistent(m);
          flipForQuality(m, {});
          return removeDoubleFaces(m);
        };
      },
    });
  }

  // ---- refined meshes (Roman): surface + volume presets through the whole build
  for (const preset of ["coarse", "medium", "fine", "converged"] as const) {
    for (const fam of ["caret", "osculating"] as WaveriderFamily[]) {
      specs.push({
        name: `mesh/build/${preset}/${fam}`,
        group: "mesh",
        maxRuns: 40,
        setup: () => (i) => buildVehicle(paramsFor(fam, { refinement: presetSettings(preset), span: 2 + (i % 13) * 0.01 })),
      });
    }
    specs.push({
      name: `mesh/study-quick/${preset}/caret`,
      group: "mesh",
      note: "buildVehicle + studyVehicle(quick) with the refinement preset on (euler aero)",
      maxRuns: preset === "fine" || preset === "converged" ? 6 : 15,
      setup: () => (i) => studyVehicle(buildVehicle(paramsFor("caret", { aeroMethod: "euler", refinement: presetSettings(preset), alphaDeg: slide(i, 0, 5) })), { quick: true }),
    });
  }

  // ---- physics data pack (Dina)
  for (const method of ["cbaero", "euler"] as const) {
    specs.push({
      name: `datapack/compute/${method}`,
      group: "datapack",
      note: "computeDataPack from a changed parameter (all sweeps + validation + provenance)",
      maxRuns: 6,
      setup: () => (i) => computeDataPack(paramsFor("caret", { aeroMethod: method, alphaDeg: slide(i, 0, 5), flightMach: 8 + (i % 5) * 0.1 })),
    });
  }
  specs.push({
    name: "datapack/live-recompute-flight",
    group: "datapack",
    note: "Data tab live recompute after a flight-condition tweak (geometry unchanged)",
    maxRuns: 6,
    setup: () => (i) => computeDataPack(paramsFor("caret", { altKm: slide(i, 20, 40) })),
  });
  specs.push({
    name: "datapack/serialise",
    group: "datapack",
    maxRuns: 20,
    setup: () => {
      const pack = computeDataPack(paramsFor("caret"));
      return () => {
        packToJson(pack, true);
        return buildZip(packFiles(pack).map((f) => ({ name: f.name, data: utf8(f.content) })));
      };
    },
  });
  specs.push({
    name: "validate/runValidation",
    group: "datapack",
    maxRuns: 20,
    setup: () => () => runValidation(),
  });

  // ---- DATCOM sweeps (Stirke)
  for (const p of PRESETS) {
    specs.push({
      name: `datcom/runCases/${p.id}`,
      group: "datcom",
      maxRuns: 40,
      setup: () => (i) => runCases(p.config, { ...p.flight, altKm: (p.flight.altKm ?? 20) + (i % 7) * 0.05 }),
    });
  }
  return specs;
}
