// Run: bun scripts/topology-report.ts [--json out.json] [--stl dir]
// Prints a per-family geometry-quality table (manifold / slivers / G1 jumps).
import { writeFileSync, mkdirSync } from "node:fs";
import { buildVehicle } from "../src/lib/waverider/generate";
import { topoMetrics } from "../src/lib/waverider/geom-quality";
import { meshToBinaryStl } from "../src/lib/waverider/export-stl";
import { DEFAULT_PARAMS, type DesignParams, type WaveriderFamily } from "../src/lib/waverider/types";

export const FAMILIES: WaveriderFamily[] = ["caret", "cone", "osculating", "viscopt", "elliptic", "wedgecone", "star", "liftbody", "ramjet", "scramjet", "busemann", "inward", "integrated"];

export interface Case { label: string; patch: Partial<DesignParams> }
export const CASES: Case[] = [
  { label: "M8 default", patch: {} },
  { label: "M5 R=.5%", patch: { mach: 5, leRadius: 0.02, nx: 40, ny: 28 } },
  { label: "M12 fine", patch: { mach: 12, nx: 80, ny: 56, leRadius: 0.01 } },
  { label: "noLE", patch: { leBlunt: false } },
  { label: "bigR wide", patch: { leRadius: 0.1, span: 3.2 } },
  { label: "coarse", patch: { nx: 20, ny: 12 } },
];

export function build(family: WaveriderFamily, patch: Partial<DesignParams>) {
  const params: DesignParams = {
    ...DEFAULT_PARAMS,
    family,
    flowThrough: family === "ramjet" || family === "scramjet",
    ...patch,
  };
  return buildVehicle(params);
}

if (import.meta.main) {
  const args = process.argv.slice(2);
  const jsonOut = args.includes("--json") ? args[args.indexOf("--json") + 1] : null;
  const stlDir = args.includes("--stl") ? args[args.indexOf("--stl") + 1] : null;
  if (stlDir) mkdirSync(stlDir, { recursive: true });
  const rows: Record<string, unknown>[] = [];
  const hdr = "family      case         tris   open nonM incon dupV degen minAng  sliv  aspect  jumpSame jumpLE jumpNose  vol";
  console.log(hdr);
  for (const f of FAMILIES) {
    for (const c of CASES) {
      const built = build(f, c.patch);
      const m = topoMetrics(built.mesh);
      const flow = f === "ramjet" || f === "scramjet";
      rows.push({ family: f, case: c.label, flowThrough: flow, ...m });
      console.log(
        `${f.padEnd(11)} ${c.label.padEnd(12)} ${String(m.triangles).padStart(5)} ${String(m.openEdges).padStart(5)} ${String(m.nonManifoldEdges).padStart(4)} ${String(m.inconsistentEdges).padStart(5)} ${String(m.duplicateVerts).padStart(4)} ${String(m.degenerateTris).padStart(5)} ${m.minAngleDeg.toFixed(2).padStart(6)} ${String(m.slivers).padStart(5)} ${m.maxAspect.toFixed(1).padStart(7)} ${m.maxJumpSame.toFixed(1).padStart(8)} ${m.maxJumpLE.toFixed(1).padStart(6)} ${m.maxJumpNose.toFixed(1).padStart(8)} ${m.signedVolume.toFixed(3).padStart(7)}`,
      );
      if (stlDir) writeFileSync(`${stlDir}/${f}_${c.label.replace(/\W+/g, "_")}.stl`, Buffer.from(meshToBinaryStl(built.mesh, f)));
    }
  }
  if (jsonOut) writeFileSync(jsonOut, JSON.stringify(rows, null, 1));
}
