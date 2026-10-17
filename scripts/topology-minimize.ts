// Run: bun scripts/topology-minimize.ts <fuzz.json> <caseId|all:K> [cause]
// Delta-minimises a fuzz case: resets each parameter to its default while the defect persists.
// `all:K` minimises up to K cases per cause and prints a grouped summary of the surviving parameters.
import { readFileSync } from "node:fs";
import { runCase } from "./topology-fuzz";
import { DEFAULT_PARAMS, type DesignParams } from "../src/lib/waverider/types";
const [file, idS, cause] = process.argv.slice(2);
const j = JSON.parse(readFileSync(file, "utf8"));

function minimise(d: any): Record<string, unknown> | null {
  const withExports = String(d.cause).startsWith("export");
  const cur: Partial<DesignParams> = { ...d.params };
  const has = (p: Partial<DesignParams>) => runCase({ id: d.id, family: d.family, params: p, exports: withExports }, 1e9).some((x) => x.cause === d.cause);
  if (!has(cur)) return null;
  for (const k of Object.keys(cur) as (keyof DesignParams)[]) {
    if (k === "family" || k === "flowThrough") continue;
    const keep = cur[k];
    (cur as any)[k] = (DEFAULT_PARAMS as any)[k];
    if (!has(cur)) (cur as any)[k] = keep;
  }
  const diff: Record<string, unknown> = {};
  for (const k of Object.keys(cur) as (keyof DesignParams)[]) if (k !== "family" && k !== "flowThrough" && (cur as any)[k] !== (DEFAULT_PARAMS as any)[k]) diff[k] = typeof (cur as any)[k] === "number" ? +(cur as any)[k].toPrecision(4) : (cur as any)[k];
  return diff;
}

if (idS.startsWith("all:")) {
  const K = +idS.slice(4);
  const causes = cause ? [cause] : [...new Set<string>(j.defects.map((d: any) => d.cause))];
  for (const c of causes) {
    const ds = j.defects.filter((d: any) => d.cause === c);
    const step = Math.max(1, Math.floor(ds.length / K));
    const groups = new Map<string, number>();
    for (let q = 0; q < ds.length && q / step < K; q += step) {
      const d = ds[q];
      const diff = minimise(d);
      const key = diff ? `${d.family} ${JSON.stringify(diff)}` : `${d.family} (not reproducible)`;
      groups.set(key, (groups.get(key) ?? 0) + 1);
    }
    console.log(`== ${c} (${ds.length} records)`);
    for (const [k, v] of groups) console.log(`  ${v}x ${k}`);
  }
} else {
  const d = j.defects.find((x: any) => x.id === +idS && (!cause || x.cause === cause));
  if (!d) throw new Error("no such defect");
  const diff = minimise(d);
  console.log(d.family, d.cause, diff ? JSON.stringify(diff) : "not reproducible");
}
