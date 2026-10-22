/**
 * [aerodb-perf] generateAeroDatabase benchmark: TS (forcesOnly, i.e. heating skipped, so the speedup is not
 * inflated by heating), the original e586e26 TS loop (heating on, optional), wasm on the calling thread, and the
 * worker_threads pool. Also a kernel-only per-point timing (no geometry rebuilds).
 *
 *   npm run bench:aerodb [-- --quick] [-- --legacy /path/to/e586e26/checkout]
 *
 * Writes bench/aerodb-perf.json and bench/aerodb-perf.md.
 */
import { writeFileSync, mkdirSync, existsSync } from "node:fs";
import { cpus } from "node:os";
import { execSync } from "node:child_process";
import { buildVehicle } from "../src/lib/waverider/generate";
import { DEFAULT_PARAMS, type AeroMethod, type WaveriderFamily } from "../src/lib/waverider/types";
import { generateAeroDatabase, gridCount, type AeroDbSpec } from "../src/lib/waverider/aerodb";
import { createNodeAeroPool } from "../src/lib/waverider/aerodb-pool-node";
import { evalCaseTs, evalCaseWasm, packPoints, type BatchPoint } from "../src/lib/waverider/aerodb-batch";
import { PANEL_WASM_BYTES, panelKernel } from "../src/lib/waverider/panel-kernel";

const args = process.argv.slice(2);
const quick = args.includes("--quick");
const legacyDir = args.includes("--legacy") ? args[args.indexOf("--legacy") + 1] : null;
const nCpu = cpus().length;
const poolSize = Math.max(1, nCpu - 1);

const grid = (machMin: number, machMax: number, machStep: number, aMin: number, aMax: number, aStep: number, bMin: number, bMax: number, bStep: number, dMin: number, dMax: number, dStep: number, method: AeroMethod): AeroDbSpec => ({
  machMin, machMax, machStep, alphaMin: aMin, alphaMax: aMax, alphaStep: aStep, betaMin: bMin, betaMax: bMax, betaStep: bStep, deltaMin: dMin, deltaMax: dMax, deltaStep: dStep,
  method, eulerSymmetry: false, dynamicDerivatives: true, altKm: 30,
});

const GRIDS = {
  "480": (m: AeroMethod) => grid(4, 12, 2, -4, 10, 2, -4, 8, 4, -10, 10, 10, m),
  "2016": (m: AeroMethod) => grid(2, 14, 2, -6, 16, 2, -4, 8, 4, -15, 10, 5, m),
  "7875": (m: AeroMethod) => grid(2, 22, 1, -8, 20, 2, -6, 6, 3, -20, 20, 10, m),
};

interface Row {
  family: string;
  method: string;
  triangles: number;
  grid: string;
  points: number;
  tsLegacyMs?: number;
  tsMs?: number;
  wasmMs: number;
  poolMs: number;
  poolColdMs?: number;
}

async function time<T>(f: () => Promise<T>): Promise<[number, T]> {
  const t0 = performance.now();
  const r = await f();
  return [performance.now() - t0, r];
}

const legacy = legacyDir && existsSync(`${legacyDir}/src/lib/waverider/aerodb.ts`) ? ((await import(`${legacyDir}/src/lib/waverider/aerodb.ts`)) as typeof import("../src/lib/waverider/aerodb")) : null;
const legacyGen = legacyDir && legacy ? ((await import(`${legacyDir}/src/lib/waverider/generate.ts`)) as typeof import("../src/lib/waverider/generate")) : null;

const rows: Row[] = [];
const kernelRows: { family: string; method: string; triangles: number; tsUsPerPt: number; wasmUsPerPt: number }[] = [];
const pool = createNodeAeroPool(poolSize);
try {
  const families: [WaveriderFamily, AeroMethod[]][] = quick ? [["caret", ["cbaero"]]] : [["caret", ["cbaero", "mixed", "newtonian"]], ["osculating", ["cbaero", "mixed"]], ["boostglide", ["tangent"]]];
  for (const [family, methods] of families) {
    const built = buildVehicle({ ...DEFAULT_PARAMS, family, mach: 8 });
    const nt = built.mesh.indices.length / 3;
    for (const method of methods) {
      // kernel-only: one mesh, 256 points, warm caches on both sides
      const pts: BatchPoint[] = [];
      for (const mach of [3, 5, 8, 12]) for (let a = -4; a <= 12; a += 2) for (const beta of [0, 2, 4, 6, -3, -6, 1, 3]) pts.push({ mach, alpha: a, beta });
      const c = { mesh: built.mesh, params: { ...built.params, aeroMethod: method }, altKm: 30, sRef: built.aero.planformArea, lRef: built.params.length };
      const packed = packPoints(pts);
      const k = panelKernel()!;
      evalCaseTs(c, packed);
      evalCaseWasm(k, c, packed);
      const [tTs] = await time(async () => evalCaseTs(c, packed));
      const [tW] = await time(async () => evalCaseWasm(k, c, packed));
      kernelRows.push({ family, method, triangles: nt, tsUsPerPt: (tTs * 1000) / pts.length, wasmUsPerPt: (tW * 1000) / pts.length });
      console.log(`kernel ${family} ${method} nt ${nt}: ts ${((tTs * 1000) / pts.length).toFixed(0)} us/pt, wasm ${((tW * 1000) / pts.length).toFixed(0)} us/pt (x${(tTs / tW).toFixed(2)})`);

      const grids = quick ? ["480"] : family === "caret" && method === "cbaero" ? ["480", "2016", "7875"] : ["480", "2016"];
      for (const g of grids) {
        const spec = GRIDS[g as keyof typeof GRIDS](method);
        const n = gridCount(spec, built.params.mach, built.params.alphaDeg).points;
        const row: Row = { family, method, triangles: nt, grid: g, points: n, wasmMs: 0, poolMs: 0 };
        // fresh workers for the cold number (includes worker start + TS loader + wasm compile)
        if (g === "480" && method === methods[0]) {
          pool.terminate();
          [row.poolColdMs] = await time(() => generateAeroDatabase(built, spec, { engine: "pool", pool }));
        }
        const runTs = g !== "7875" || !quick;
        if (runTs) [row.tsMs] = await time(() => generateAeroDatabase(built, spec, { engine: "ts", pointCap: 1e9 }));
        if (legacy && legacyGen && g === "480") {
          const lb = legacyGen.buildVehicle({ ...DEFAULT_PARAMS, family, mach: 8 });
          [row.tsLegacyMs] = await time(() => legacy.generateAeroDatabase(lb, spec));
        }
        [row.wasmMs] = await time(() => generateAeroDatabase(built, spec, { engine: "wasm" }));
        [row.poolMs] = await time(() => generateAeroDatabase(built, spec, { engine: "pool", pool }));
        rows.push(row);
        const f = (v?: number) => (v === undefined ? "-" : `${(v / 1000).toFixed(2)} s`);
        console.log(
          `${family} ${method} ${n} pts: ts-legacy ${f(row.tsLegacyMs)}  ts ${f(row.tsMs)}  wasm ${f(row.wasmMs)}  pool×${poolSize} ${f(row.poolMs)}  (cold ${f(row.poolColdMs)})`,
        );
      }
    }
  }
} finally {
  pool.terminate();
}

let sha = "unknown";
try {
  sha = execSync("git rev-parse --short HEAD").toString().trim();
} catch {
  /* not a checkout */
}
const meta = { sha, node: process.version, cpus: nCpu, cpuModel: cpus()[0]?.model ?? "?", poolSize, wasmBytes: PANEL_WASM_BYTES, date: new Date().toISOString(), legacyDir: legacy ? legacyDir : null };
mkdirSync("bench", { recursive: true });
writeFileSync("bench/aerodb-perf.json", JSON.stringify({ meta, kernel: kernelRows, generateAeroDatabase: rows }, null, 2) + "\n");
const s = (v?: number) => (v === undefined ? "–" : (v / 1000).toFixed(2));
const x = (a?: number, b?: number) => (a === undefined || b === undefined || b === 0 ? "–" : `×${(a / b).toFixed(1)}`);
const md = [
  `# aerodb-perf benchmark`,
  ``,
  `${meta.date} · ${sha} · Node ${meta.node} · ${meta.cpus}× ${meta.cpuModel} · pool ${poolSize} worker_threads · cuspis-panel wasm ${meta.wasmBytes} B`,
  ``,
  `TS = generateAeroDatabase engine "ts" (panelAero forcesOnly: heating skipped). TS-legacy = e586e26's loop (heating on). Times include the elevon geometry rebuilds and the dynamic-derivative points. Pool cold = fresh workers (start, TS loader, wasm compile).`,
  ``,
  `## generateAeroDatabase (seconds)`,
  ``,
  `| family | method | tris | points | TS-legacy | TS | wasm | pool | pool cold | wasm vs TS | pool vs TS |`,
  `|---|---|---|---|---|---|---|---|---|---|---|`,
  ...rows.map((r) => `| ${r.family} | ${r.method} | ${r.triangles} | ${r.points} | ${s(r.tsLegacyMs)} | ${s(r.tsMs)} | ${s(r.wasmMs)} | ${s(r.poolMs)} | ${s(r.poolColdMs)} | ${x(r.tsMs, r.wasmMs)} | ${x(r.tsMs, r.poolMs)} |`),
  ``,
  `## Kernel only (µs per point, one mesh, warm caches)`,
  ``,
  `| family | method | tris | TS | wasm | speedup |`,
  `|---|---|---|---|---|---|`,
  ...kernelRows.map((k) => `| ${k.family} | ${k.method} | ${k.triangles} | ${k.tsUsPerPt.toFixed(0)} | ${k.wasmUsPerPt.toFixed(0)} | ×${(k.tsUsPerPt / k.wasmUsPerPt).toFixed(2)} |`),
  ``,
].join("\n");
writeFileSync("bench/aerodb-perf.md", md);
console.log("wrote bench/aerodb-perf.json, bench/aerodb-perf.md");
