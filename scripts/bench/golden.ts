/**
 * Output goldens for the optimised hot paths. Uses ONLY APIs that exist on the pre-perf tree, so the fixture can be
 * (re)generated there and then checked on the optimised tree:
 *   tsx scripts/bench/golden.ts --write src/lib/perf/fixtures/golden.json     (run on the unoptimised commit)
 *   tsx scripts/bench/golden.ts --check src/lib/perf/fixtures/golden.json     (npm run test:regress)
 * Every entry is a SHA-256 over the full numeric output (geometry arrays, study arrays, tables), so "equal" means
 * bit-for-bit equal; a failing key is reported with its name.
 */
import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";
import { buildVehicle } from "../../src/lib/waverider/generate";
import { badEdges, topoMetrics, worstDihedrals } from "../../src/lib/waverider/geom-quality";
import { analyzeMesh } from "../../src/lib/waverider/mesh";
import { edgeDefects, triangleQuality } from "../../src/lib/waverider/mesh-repair";
import { presetSettings } from "../../src/lib/waverider/refinement";
import { computeDataPack, packToJson } from "../../src/lib/waverider/datapack";
import { runCases } from "../../src/lib/waverider/datcom/model";
import { PRESETS } from "../../src/lib/waverider/datcom/presets";
import { runValidation } from "../../src/lib/waverider/validate";
import { studyVehicle } from "../../src/lib/waverider/study";
import { DEFAULT_PARAMS, type BuiltVehicle, type WaveriderFamily } from "../../src/lib/waverider/types";
import { FAMILIES, paramsFor } from "./suite";

const sha = (...parts: (string | ArrayBufferView)[]) => {
  const h = createHash("sha256");
  for (const p of parts) h.update(typeof p === "string" ? p : new Uint8Array(p.buffer, p.byteOffset, p.byteLength));
  return h.digest("hex").slice(0, 24);
};
const json = (v: unknown) => JSON.stringify(v, (_k, x) => (typeof x === "number" && !Number.isFinite(x) ? String(x) : x instanceof Float64Array || x instanceof Float32Array ? Array.from(x) : x));
/**
 * Canonical text with every float rounded to 12 significant digits. Used where the optimised tree is allowed to
 * differ from the original by ~1 ulp: the wedge-anchor Euler check (validation "euler-p") now runs the Rust march
 * (same algorithm, 2e-15 relative difference; perf.test.ts), which flows into studies / validation / the data pack.
 */
const r12 = (v: unknown) => json(v).replace(/-?\d+\.\d+(?:e[+-]?\d+)?/gi, (m) => Number(m).toPrecision(12));
const meshHash = (b: BuiltVehicle) => sha(b.mesh.positions, b.mesh.indices, b.mesh.surfaces, json(b.aero));

const out: Record<string, string> = {};
function put(k: string, f: () => string) {
  try {
    out[k] = f();
  } catch (e) {
    out[k] = "ERR " + (e instanceof Error ? e.message : String(e));
  }
}

for (const fam of FAMILIES) put(`geom/${fam}`, () => meshHash(buildVehicle(paramsFor(fam))));
for (const fam of ["caret", "osculating"] as WaveriderFamily[]) {
  put(`geom/${fam}@96x72`, () => meshHash(buildVehicle(paramsFor(fam, { nx: 96, ny: 72 }))));
  for (const preset of ["coarse", "fine"] as const) put(`geom/${fam}/${preset}`, () => meshHash(buildVehicle(paramsFor(fam, { refinement: presetSettings(preset) }))));
}
for (const fam of ["caret", "osculating", "liftbody", "scramjet"] as WaveriderFamily[]) {
  const b = buildVehicle(paramsFor(fam, { nx: 64, ny: 48 }));
  put(`topo/${fam}`, () => sha(json({ t: topoMetrics(b.mesh), e: badEdges(b.mesh), d: worstDihedrals(b.mesh, 10), q: analyzeMesh(b.mesh), x: edgeDefects(b.mesh), y: triangleQuality(b.mesh) })));
}
for (const p of PRESETS) put(`datcom/${p.id}`, () => sha(json(runCases(p.config, p.flight))));
for (const method of ["cbaero", "mixed", "euler"] as const)
  for (const fam of ["caret", "osculating", "liftbody"] as WaveriderFamily[])
    put(`study/${method}/${fam}`, () => {
      const s = studyVehicle(buildVehicle(paramsFor(fam, { aeroMethod: method })), {});
      const q = studyVehicle(buildVehicle(paramsFor(fam, { aeroMethod: method, alphaDeg: 3.3, flightMach: 9.1 })), { quick: true });
      return sha(r12({ ...s, elapsedMs: 0 }), r12({ ...q, elapsedMs: 0 }));
    });
put("validation", () => sha(r12(runValidation())));
for (const method of ["cbaero", "euler"] as const)
  put(`datapack/${method}`, () => {
    const pack = computeDataPack({ ...DEFAULT_PARAMS, aeroMethod: method });
    return sha(r12(packToJson(pack).replace(/\d{4}-\d\d-\d\dT[\d:.]+Z/g, "T")));
  });

const [mode, file] = process.argv.slice(2);
if (mode === "--write") {
  writeFileSync(file, JSON.stringify(out, null, 1) + "\n");
  console.log(`wrote ${Object.keys(out).length} goldens to ${file}`);
} else if (mode === "--check") {
  const want = JSON.parse(readFileSync(file, "utf8")) as Record<string, string>;
  let bad = 0;
  for (const k of Object.keys(want)) {
    if (out[k] !== want[k]) {
      bad++;
      console.log(`MISMATCH ${k}: ${out[k]} != ${want[k]}`);
    }
  }
  console.log(`${Object.keys(want).length - bad}/${Object.keys(want).length} goldens identical`);
  if (bad) process.exit(1);
} else console.log(JSON.stringify(out, null, 1));
