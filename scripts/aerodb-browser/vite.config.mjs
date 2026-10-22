// [aerodb-perf] Builds the browser check page (scripts/aerodb-browser/run.mjs builds and serves it).
import { fileURLToPath } from "node:url";
import { defineConfig } from "vite";
const here = fileURLToPath(new URL(".", import.meta.url));
export default defineConfig({
  root: here,
  resolve: { alias: { "@": fileURLToPath(new URL("../../src", import.meta.url)) } },
  worker: { format: "es" },
  build: { outDir: fileURLToPath(new URL("../../.cache/aerodb-browser", import.meta.url)), emptyOutDir: true },
  logLevel: "warn",
});
