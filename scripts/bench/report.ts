/**
 * Markdown before/after table from two bench JSON files.
 *   tsx scripts/bench/report.ts before.json after.json > table.md
 */
import { readFileSync } from "node:fs";
interface R { name: string; group: string; n: number; median: number; p95: number }
const [bf, af] = process.argv.slice(2);
const before = JSON.parse(readFileSync(bf, "utf8")) as { results: R[]; machine: Record<string, unknown> };
const after = JSON.parse(readFileSync(af, "utf8")) as { results: R[]; machine: Record<string, unknown> };
const bm = new Map(before.results.map((r) => [r.name, r]));
const fmt = (x: number) => (x >= 100 ? x.toFixed(0) : x >= 10 ? x.toFixed(1) : x >= 0.1 ? x.toFixed(2) : x.toFixed(3));
const x = (b: number, a: number) => (a > 0 ? (b / a >= 10 ? (b / a).toFixed(0) : (b / a).toFixed(2)) + "x" : "n/a");
console.log("| benchmark | before median (ms) | after median (ms) | speedup | before p95 | after p95 | p95 speedup |");
console.log("|---|---:|---:|---:|---:|---:|---:|");
for (const a of after.results) {
  const b = bm.get(a.name);
  if (!b) console.log(`| ${a.name} | new | ${fmt(a.median)} | - | new | ${fmt(a.p95)} | - |`);
  else console.log(`| ${a.name} | ${fmt(b.median)} | ${fmt(a.median)} | ${b.median < 0.002 ? "-" : x(b.median, a.median)} | ${fmt(b.p95)} | ${fmt(a.p95)} | ${b.p95 < 0.002 ? "-" : x(b.p95, a.p95)} |`);
}
