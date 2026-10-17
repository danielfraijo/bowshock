/**
 * Bowshock benchmark runner.
 *   npm run bench                         full suite -> bench-results.json
 *   npm run bench -- --quick              ~1/4 of the sampling budget
 *   npm run bench -- --filter geom/       only names containing "geom/"
 *   npm run bench -- --out file.json      choose output path
 *   npm run bench -- --compare before.json  print speedup table vs an earlier run
 */
import { readFileSync, writeFileSync, mkdirSync } from "node:fs";
import { dirname } from "node:path";
import { machineInfo, runBench, type BenchResult } from "./bench/harness";
import { buildSuite } from "./bench/suite";

const args = process.argv.slice(2);
const flag = (n: string) => args.includes(`--${n}`);
const val = (n: string) => {
  const i = args.indexOf(`--${n}`);
  return i >= 0 ? args[i + 1] : undefined;
};

const scale = flag("quick") ? 0.25 : Number(val("scale") ?? 1);
const filter = val("filter");
const out = val("out") ?? "bench-results.json";
const cmp = val("compare");

let specs = buildSuite();
let extra: typeof specs = [];
try {
  // Optional: the optimisation layer registers extra (new-API) benchmarks here.
  const mod = await import("./bench/suite-perf");
  extra = mod.perfSuite();
} catch {
  /* baseline checkout has no perf layer */
}
specs = [...specs, ...extra];
if (filter) specs = specs.filter((s) => s.name.includes(filter));

const results: BenchResult[] = [];
const info = machineInfo();
console.log(`# bowshock bench  node ${info.node}  ${info.cpu} x${info.cores}  commit ${info.commit}  scale ${scale}`);
console.log("name".padEnd(58), "n".padStart(4), "median ms".padStart(11), "p95 ms".padStart(10));
for (const spec of specs) {
  try {
    const r = await runBench(spec, scale);
    results.push(r);
    console.log(spec.name.padEnd(58), String(r.n).padStart(4), r.median.toFixed(3).padStart(11), r.p95.toFixed(3).padStart(10));
  } catch (err) {
    console.log(spec.name.padEnd(58), "FAILED", err instanceof Error ? err.message : String(err));
  }
}
mkdirSync(dirname(out) || ".", { recursive: true });
writeFileSync(out, JSON.stringify({ machine: info, scale, results }, null, 2));
console.log(`wrote ${out}`);

if (cmp) {
  const before = JSON.parse(readFileSync(cmp, "utf8")) as { results: BenchResult[] };
  const map = new Map(before.results.map((r) => [r.name, r]));
  console.log("\nname".padEnd(59), "before med".padStart(11), "after med".padStart(10), "x".padStart(7), "before p95".padStart(11), "after p95".padStart(10), "x".padStart(7));
  for (const r of results) {
    const b = map.get(r.name);
    if (!b) continue;
    console.log(
      r.name.padEnd(58),
      b.median.toFixed(3).padStart(11),
      r.median.toFixed(3).padStart(10),
      (b.median / r.median).toFixed(2).padStart(7),
      b.p95.toFixed(3).padStart(11),
      r.p95.toFixed(3).padStart(10),
      (b.p95 / r.p95).toFixed(2).padStart(7),
    );
  }
}
