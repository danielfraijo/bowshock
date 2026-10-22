/**
 * Headless design evaluation for OpenMDAO, scipy, or a shell loop.
 *
 *   node --import ./scripts/ts-register.mjs scripts/cuspis-eval.ts design.json
 *
 * design.json is either a DesignParams object or { "params": { ... } }.
 * Prints one JSON object: { ld, rangeKm, qStag, mass, staticMargin, volume, cd, cm, cl, method }.
 * Euler is evaluated as CBAERO — same rule as the in-browser optimizer.
 */
import { readFileSync } from "node:fs";
import { evaluateDesign } from "../src/lib/waverider/mdo";
import { sanitizeParams } from "../src/lib/waverider/presets";

const file = process.argv[2];
if (!file) {
  console.error("usage: node --import ./scripts/ts-register.mjs scripts/cuspis-eval.ts design.json");
  process.exit(2);
}
const raw = JSON.parse(readFileSync(file, "utf8")) as { params?: unknown };
const params = sanitizeParams(raw.params ?? raw);
const ev = evaluateDesign(params);
if (ev.error) {
  console.error(ev.error);
  process.exit(1);
}
process.stdout.write(JSON.stringify(ev.metrics, null, 2) + "\n");
