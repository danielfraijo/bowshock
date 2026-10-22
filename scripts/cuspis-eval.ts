/**
 * Headless design evaluation for OpenMDAO, scipy, or a shell loop.
 *
 *   node --import ./scripts/ts-register.mjs scripts/cuspis-eval.ts design.json
 *
 * design.json is either a DesignParams object or { "params": { ... } }.
 * Prints one JSON object: { ld, rangeKm, qStag, mass, staticMargin, volume, cd, cm, cl, method }.
 * Euler is evaluated as CBAERO — same rule as the in-browser optimizer.
 */
import { readFileSync, statSync } from "node:fs";
import { evaluateDesign } from "../src/lib/waverider/mdo";
import { sanitizeParams } from "../src/lib/waverider/presets";

const file = process.argv[2];
if (!file) {
  console.error("usage: node --import ./scripts/ts-register.mjs scripts/cuspis-eval.ts design.json");
  process.exit(2);
}
const st = statSync(file);
if (!st.isFile() || st.size > 2_000_000) {
  console.error("design file must be a JSON file under 2 MB");
  process.exit(2);
}
const raw = JSON.parse(readFileSync(file, "utf8"), (key, value) =>
  key === "__proto__" || key === "constructor" || key === "prototype" ? undefined : value,
) as { params?: unknown };
const params = sanitizeParams(raw.params ?? raw);
const ev = evaluateDesign(params);
if (ev.error) {
  console.error(ev.error);
  process.exit(1);
}
process.stdout.write(JSON.stringify(ev.metrics, null, 2) + "\n");
