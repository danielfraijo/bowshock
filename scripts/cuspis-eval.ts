/**
 * Headless design evaluation and batch runs for OpenMDAO, scipy, or a shell loop.
 *
 *   node --import ./scripts/ts-register.mjs scripts/cuspis-eval.ts design.json
 *     design.json is either a DesignParams object or { "params": { ... } }.
 *     Prints one JSON object: { ld, rangeKm, qStag, mass, staticMargin, volume, cd, cm, cl, method }.
 *     Euler is evaluated as CBAERO — same rule as the in-browser optimizer.
 *
 *   npm run aerodb -- --config cfg.json --out dir [--mode aerodb|eval|mdo|sweep|sensitivity|fly]
 *     Batch CLI over generateAeroDatabase, mdo and guidance. See docs/aerodb.md.
 *
 * Logic lives in src/lib/waverider/headless-cli.ts (typechecked); this file wires process I/O and signals.
 * Input JSON (design or config) must be a regular file under 2 MB; __proto__/constructor/prototype keys are dropped.
 */
import { runCli } from "../src/lib/waverider/headless-cli";

const ac = new AbortController();
let interrupts = 0;
process.on("SIGINT", () => {
  interrupts++;
  if (interrupts > 1) process.exit(130);
  process.stderr.write("\naerodb: SIGINT — cancelling (press Ctrl-C again to force)\n");
  ac.abort(new DOMException("interrupted by SIGINT", "AbortError"));
});
process.on("SIGTERM", () => ac.abort(new DOMException("terminated by SIGTERM", "AbortError")));

process.exitCode = await runCli(process.argv.slice(2), { stderr: (s) => process.stderr.write(s), stdout: (s) => process.stdout.write(s) }, ac.signal);
