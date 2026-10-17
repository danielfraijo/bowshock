#!/usr/bin/env node
/**
 * Build native/cuspis-euler -> public/cuspis_euler.wasm and regenerate the embedded base64
 * fallback (src/lib/waverider/euler-wasm.ts).
 *
 *   npm run build:wasm
 *
 * Needs a Rust toolchain with the wasm32-unknown-unknown target (rustup target add
 * wasm32-unknown-unknown). Set CUSPIS_RUSTC=/path/to/rustc to use a specific compiler.
 * If `wasm-opt` (binaryen) is on PATH it is run with -O3 afterwards. SIMD: set CUSPIS_SIMD=1
 * to build with +simd128 (kernels fall back to scalar code automatically when unsupported).
 */
import { execFileSync, spawnSync } from "node:child_process";
import { copyFileSync, readFileSync, writeFileSync, statSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const crate = join(root, "native", "cuspis-euler");
const env = { ...process.env };
if (process.env.CUSPIS_RUSTC) env.RUSTC = process.env.CUSPIS_RUSTC;
if (process.env.CUSPIS_SIMD === "1") env.RUSTFLAGS = `${env.RUSTFLAGS ?? ""} -C target-feature=+simd128`.trim();

execFileSync("cargo", ["build", "--release", "--target", "wasm32-unknown-unknown", "--offline"], {
  cwd: crate,
  env,
  stdio: "inherit",
});
const built = join(crate, "target", "wasm32-unknown-unknown", "release", "cuspis_euler.wasm");
const out = join(root, "public", "cuspis_euler.wasm");
copyFileSync(built, out);

const opt = spawnSync("wasm-opt", ["--version"], { stdio: "ignore" });
if (opt.status === 0) {
  execFileSync("wasm-opt", ["-O3", "--enable-simd", "--enable-bulk-memory", "--enable-nontrapping-float-to-int", "--enable-sign-ext", "--enable-mutable-globals", out, "-o", out], { stdio: "inherit" });
  console.log("wasm-opt -O3 applied");
} else {
  console.log("wasm-opt not found: shipping the rustc/LTO output as is");
}

const bytes = readFileSync(out);
const b64 = bytes.toString("base64");
writeFileSync(
  join(root, "src", "lib", "waverider", "euler-wasm.ts"),
  `/** Packed cuspis-euler wasm (wasm32, no std I/O). Built from native/cuspis-euler by scripts/build-wasm.mjs. */\nexport const EULER_WASM_B64 =\n  "${b64}";\n`,
);
console.log(`public/cuspis_euler.wasm ${statSync(out).size} bytes; euler-wasm.ts regenerated`);
