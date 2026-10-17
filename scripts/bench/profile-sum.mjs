// Summarise a .cpuprofile by self time per function: node scripts/bench/profile-sum.mjs file.cpuprofile [top]
import { readFileSync } from "node:fs";
const prof = JSON.parse(readFileSync(process.argv[2], "utf8"));
const top = Number(process.argv[3] ?? 18);
const self = new Map();
const byId = new Map(prof.nodes.map((n) => [n.id, n]));
const dt = prof.timeDeltas;
let total = 0;
prof.samples.forEach((id, i) => {
  const n = byId.get(id);
  const cf = n.callFrame;
  const key = `${cf.functionName || "(anon)"}  ${cf.url.replace(/^.*\/(src|scripts)\//, "$1/")}:${cf.lineNumber + 1}`;
  self.set(key, (self.get(key) ?? 0) + dt[i]);
  total += dt[i];
});
const rows = [...self.entries()].sort((a, b) => b[1] - a[1]).slice(0, top);
for (const [k, v] of rows) console.log(`${(v / 1000).toFixed(0).padStart(7)} ms ${((100 * v) / total).toFixed(1).padStart(5)} %  ${k}`);
console.log(`total ${(total / 1000).toFixed(0)} ms`);
