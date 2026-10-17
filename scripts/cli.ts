/**
 * Bowshock headless CLI: the same kernels as the web app (same TS code, same cuspis_euler.wasm).
 *
 *   npm run cli -- study  [--family caret] [--mach 8] [--alpha 2] [--alt 28] [--method euler|cbaero|mixed|...] [--mode full|quick|preview] [--json]
 *   npm run cli -- wedge  --mach 6 --theta 10          2-D HLLC wedge vs exact oblique shock
 *   npm run cli -- check                                Checks-tab identities
 *   npm run cli -- export --kind stl|step-nurbs|iges|plot3d|obj|kit [--family ..] [--out dir]
 *   npm run cli -- bench  [--quick] [--filter geom/] [--out file.json] [--compare before.json]
 *   npm run cli -- info                                 kernel / wasm / machine info
 * Common: --wasm path/to/cuspis_euler.wasm  (default: public/cuspis_euler.wasm if present, else the embedded copy)
 * Build a standalone single-file bundle (no tsx needed): npm run build:cli  ->  dist-cli/bowshock-cli.mjs
 */
import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { join, dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { DEFAULT_PARAMS, type DesignParams, type WaveriderFamily } from "../src/lib/waverider/types";
import { eulerFromBytes, eulerSync, KERNEL_VERSION, wasmSource } from "../src/lib/perf/euler-kernel";
import { Pipeline, type StudyMode } from "../src/lib/perf/pipeline";
import { WorkerCore } from "../src/lib/perf/worker-core";
import { studyJson } from "../src/lib/waverider/study";
import { solveEulerWedge } from "../src/lib/waverider/euler2d";
import { runValidation, validationSummary } from "../src/lib/waverider/validate";
import type { WorkerReply } from "../src/lib/perf/protocol";

const here = dirname(fileURLToPath(import.meta.url));
const root = resolve(here, "..");
const argv = process.argv.slice(2);
const cmd = argv[0] ?? "help";
const rest = argv.slice(1);
const has = (n: string) => rest.includes(`--${n}`);
const opt = (n: string, d?: string) => {
  const i = rest.indexOf(`--${n}`);
  return i >= 0 && i + 1 < rest.length ? rest[i + 1] : d;
};
const num = (n: string, d: number) => (opt(n) !== undefined ? Number(opt(n)) : d);

async function loadWasm() {
  const cands = [opt("wasm"), join(here, "cuspis_euler.wasm"), join(root, "public", "cuspis_euler.wasm")].filter(Boolean) as string[];
  const p = cands.find((c) => existsSync(c));
  if (p) await eulerFromBytes(readFileSync(p));
  else eulerSync();
}

function paramsFromArgs(): DesignParams {
  const family = (opt("family", "caret") ?? "caret") as WaveriderFamily;
  return {
    ...DEFAULT_PARAMS,
    family,
    flowThrough: family === "ramjet" || family === "scramjet",
    mach: num("design-mach", num("mach", DEFAULT_PARAMS.mach)),
    flightMach: num("mach", DEFAULT_PARAMS.flightMach),
    alphaDeg: num("alpha", DEFAULT_PARAMS.alphaDeg),
    altKm: num("alt", DEFAULT_PARAMS.altKm),
    aeroMethod: (opt("method") ?? DEFAULT_PARAMS.aeroMethod) as DesignParams["aeroMethod"],
    nx: num("nx", DEFAULT_PARAMS.nx),
    ny: num("ny", DEFAULT_PARAMS.ny),
  };
}

async function main() {
  if (cmd === "bench") {
    // delegate: same harness/suite as `npm run bench`
    process.argv = [process.argv[0], join(here, "bench.ts"), ...rest];
    await import("./bench");
    return;
  }
  await loadWasm();
  if (cmd === "info") {
    console.log(JSON.stringify({ kernel: KERNEL_VERSION, wasm: wasmSource, node: process.version, platform: process.platform, arch: process.arch }, null, 2));
  } else if (cmd === "study") {
    const p = paramsFromArgs();
    const mode = (opt("mode", "full") ?? "full") as StudyMode;
    const pl = new Pipeline();
    const t0 = performance.now();
    const out = pl.run(p, mode);
    const ms = performance.now() - t0;
    if (has("json")) console.log(studyJson(out.built, out.study));
    else {
      const a = out.study.aero;
      console.log(`${p.family} M${p.flightMach} α${p.alphaDeg}° h${p.altKm} km  method=${p.aeroMethod}  mode=${mode}`);
      console.log(`CL ${a.cl.toFixed(5)}  CD ${a.cd.toFixed(5)}  L/D ${a.ld.toFixed(3)}   [geometry ${out.ms.geometry.toFixed(1)} ms, study ${out.ms.study.toFixed(1)} ms, total ${ms.toFixed(1)} ms; wasm=${wasmSource}]`);
    }
  } else if (cmd === "wedge") {
    const mach = num("mach", 6);
    const theta = num("theta", 10);
    const r = solveEulerWedge(mach, theta, num("gamma", 1.4));
    console.log(JSON.stringify(r, null, 2));
  } else if (cmd === "check") {
    const checks = runValidation();
    const s = validationSummary(checks);
    for (const c of checks) if (!c.pass) console.log("FAIL", c.name ?? JSON.stringify(c));
    console.log(`checks ${s.pass}/${s.n} ${s.ok ? "OK" : "FAILED"}`);
    process.exitCode = s.ok ? 0 : 1;
  } else if (cmd === "export") {
    const kind = opt("kind", "stl") ?? "stl";
    const outDir = resolve(opt("out", "bowshock-out") ?? "bowshock-out");
    mkdirSync(outDir, { recursive: true });
    const read = (p: string) => (existsSync(join(root, p)) ? readFileSync(join(root, p), "utf8") : "");
    const files: { name: string; blob: Blob }[] = [];
    let err: string | undefined;
    const core = new WorkerCore((r: WorkerReply) => {
      if (r.type === "export") files.push(...r.files);
      if (r.type === "error") err = r.message;
    });
    await core.handle({
      type: "export",
      id: 1,
      kind,
      params: paramsFromArgs(),
      sources: {
        python: read("public/waverider_cad.py"),
        cSrc: read("public/bowshock_aero.c"),
        cppSrc: read("public/bowshock.cpp"),
        kernelSource: read("native/bowshock_kernel.c"),
      },
    });
    if (err) throw new Error(err);
    for (const f of files) {
      const path = join(outDir, f.name);
      mkdirSync(dirname(path), { recursive: true });
      writeFileSync(path, Buffer.from(await f.blob.arrayBuffer()));
      console.log(`wrote ${path} (${f.blob.size} bytes)`);
    }
  } else {
    console.log(readFileSync(fileURLToPath(import.meta.url), "utf8").split("*/")[0].replace(/^\/\*\*\n?/, "").replace(/^ \* ?/gm, ""));
  }
}

await main();
