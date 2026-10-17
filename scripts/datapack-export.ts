/**
 * Export the Bowshock data package for a design.
 *   node --import ./scripts/ts-register.mjs scripts/datapack-export.ts [family] [outDir] [key=value …]
 * e.g. … caret ./out aeroMethod=mixed alphaDeg=2 flightMach=8 lockFlight=false
 */
import { mkdirSync, writeFileSync } from "node:fs";
import { dirname } from "node:path";
import { execSync } from "node:child_process";
import { computeDataPack, packFiles } from "../src/lib/waverider/datapack";
import { DEFAULT_PARAMS, type DesignParams } from "../src/lib/waverider/types";

const [family = "caret", out = "./datapack-out", ...kv] = process.argv.slice(2);
const params: Record<string, unknown> = { ...DEFAULT_PARAMS, family };
for (const a of kv) {
  const [k, v] = a.split("=");
  const cur = (DEFAULT_PARAMS as unknown as Record<string, unknown>)[k];
  params[k] = typeof cur === "number" ? Number(v) : typeof cur === "boolean" ? v === "true" : v;
}
let gitHash = "";
try {
  gitHash = execSync("git rev-parse --short HEAD", { stdio: ["ignore", "pipe", "ignore"] }).toString().trim();
} catch {
  /* not a git checkout */
}
const t0 = Date.now();
const pack = computeDataPack(params as unknown as DesignParams, { gitHash });
for (const f of packFiles(pack, ".")) {
  const path = `${out}/${f.name}`;
  mkdirSync(dirname(path), { recursive: true });
  writeFileSync(path, f.content);
}
console.log(`${family}: ${pack.tables.length} tables, key ${pack.key}, ${Date.now() - t0} ms, validation ${pack.validation.pass}/${pack.validation.n} → ${out}`);
