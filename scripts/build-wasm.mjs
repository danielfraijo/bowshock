#!/usr/bin/env node
/**
 * Build the Rust/wasm kernels and regenerate their embedded base64 modules.
 *
 *   npm run build:wasm                 # native/cuspis-panel -> src/lib/waverider/panel-wasm.ts
 *   npm run build:wasm -- cuspis-euler # native/cuspis-euler -> src/lib/waverider/euler-wasm.ts
 *   npm run build:wasm -- all
 *
 * Pattern from feat/performance (PR #12, scripts/build-wasm.mjs at a110aba): cargo --release for
 * wasm32-unknown-unknown, optional `wasm-opt -O3` when binaryen is on PATH, base64 into a .ts module.
 * Needs a rustc with the wasm32-unknown-unknown target; set CUSPIS_RUSTC=/path/to/rustc to pick one.
 * cuspis-euler is only rebuilt when asked: the shipped euler-wasm.ts is the reference binary for its tests.
 */
import { execFileSync, spawnSync } from "node:child_process";
import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const CRATES = {
  "cuspis-panel": {
    artifact: "cuspis_panel.wasm",
    out: join(root, "src", "lib", "waverider", "panel-wasm.ts"),
    header:
      "/** Packed cuspis-panel wasm (wasm32, no imports). Built from native/cuspis-panel by scripts/build-wasm.mjs (npm run build:wasm). Batch panel / shock-expansion force evaluator for the aero database. */\nexport const PANEL_WASM_B64 =",
  },
  "cuspis-euler": {
    artifact: "cuspis_euler.wasm",
    out: join(root, "src", "lib", "waverider", "euler-wasm.ts"),
    header: "/** Packed cuspis-euler wasm (wasm32, no std I/O). Built from native/cuspis-euler by scripts/build-wasm.mjs. */\nexport const EULER_WASM_B64 =",
  },
};

const args = process.argv.slice(2);
const names = args.length === 0 ? ["cuspis-panel"] : args.includes("all") ? Object.keys(CRATES) : args;
const env = { ...process.env };
if (process.env.CUSPIS_RUSTC) env.RUSTC = process.env.CUSPIS_RUSTC;
const hasOpt = spawnSync("wasm-opt", ["--version"], { stdio: "ignore" }).status === 0;

for (const name of names) {
  const c = CRATES[name];
  if (!c) throw new Error(`unknown crate ${name} (known: ${Object.keys(CRATES).join(", ")})`);
  const crate = join(root, "native", name);
  execFileSync("cargo", ["build", "--release", "--target", "wasm32-unknown-unknown", "--offline", "--lib"], { cwd: crate, env, stdio: "inherit" });
  const built = join(crate, "target", "wasm32-unknown-unknown", "release", c.artifact);
  if (hasOpt) {
    execFileSync("wasm-opt", ["-O3", "--enable-bulk-memory", "--enable-nontrapping-float-to-int", "--enable-sign-ext", "--enable-mutable-globals", built, "-o", built], { stdio: "inherit" });
  }
  const bytes = readFileSync(built);
  // Validate before shipping.
  new WebAssembly.Module(bytes);
  writeFileSync(c.out, `${c.header}\n  "${bytes.toString("base64")}";\n`);
  console.log(`${name}: ${bytes.length} bytes${hasOpt ? " (wasm-opt -O3)" : " (no wasm-opt on PATH: rustc/LTO output as is)"} -> ${c.out.slice(root.length + 1)}`);
}
