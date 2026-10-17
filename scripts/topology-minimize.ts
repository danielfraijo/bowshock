// Run: bun scripts/topology-minimize.ts <fuzz.json> <caseId> [cause]
// Delta-minimises a fuzz case: resets each parameter to its default while the defect persists.
import { readFileSync } from "node:fs";
import { runCase } from "./topology-fuzz";
import { DEFAULT_PARAMS, type DesignParams } from "../src/lib/waverider/types";
const [file, idS, cause] = process.argv.slice(2);
const j = JSON.parse(readFileSync(file, "utf8"));
const defs = j.defects.filter((d: any) => d.id === +idS && (!cause || d.cause === cause));
if (!defs.length) throw new Error("no such defect");
const d = defs[0];
const cur: Partial<DesignParams> = { ...d.params };
const has = (p: Partial<DesignParams>) => runCase({ id: +idS, family: d.family, params: p, exports: true }, 1e9).some((x) => x.cause === d.cause);
if (!has(cur)) throw new Error("not reproducible");
for (const k of Object.keys(cur) as (keyof DesignParams)[]) {
  if (k === "family" || k === "flowThrough") continue;
  const keep = cur[k];
  (cur as any)[k] = (DEFAULT_PARAMS as any)[k];
  if (!has(cur)) (cur as any)[k] = keep;
}
const diff: any = {};
for (const k of Object.keys(cur) as (keyof DesignParams)[]) if ((cur as any)[k] !== (DEFAULT_PARAMS as any)[k]) diff[k] = (cur as any)[k];
console.log(d.family, d.cause, JSON.stringify(diff));
