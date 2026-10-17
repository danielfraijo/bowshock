// Bundle scripts/cli.ts (+ kernels + embedded wasm) into ONE node-runnable file: dist-cli/bowshock-cli.mjs
// Run:  npm run build:cli && node dist-cli/bowshock-cli.mjs study --family caret --mach 8
import { build } from "esbuild";
import { copyFileSync, mkdirSync, existsSync } from "node:fs";
await build({
  entryPoints: ["scripts/cli.ts"],
  outfile: "dist-cli/bowshock-cli.mjs",
  bundle: true,
  platform: "node",
  format: "esm",
  target: "node20",
  minify: true,
  legalComments: "none",
  // the bench command pulls the suite in lazily via a runtime import(); keep it external-free by bundling it too
  banner: { js: "import{createRequire as __cr}from'node:module';const require=__cr(import.meta.url);" },
});
mkdirSync("dist-cli", { recursive: true });
if (existsSync("public/cuspis_euler.wasm")) copyFileSync("public/cuspis_euler.wasm", "dist-cli/cuspis_euler.wasm");
console.log("wrote dist-cli/bowshock-cli.mjs (+ cuspis_euler.wasm next to it)");
