#!/usr/bin/env node
/**
 * Wrapper that prefers a Node >= 22 binary (for --experimental-strip-types)
 * and forwards to src/lib/waverider/datcom/cli.ts.
 */
import { spawnSync } from "node:child_process";
import { existsSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, "..");
const cli = join(root, "src/lib/waverider/datcom/cli.ts");

const candidates = [
  process.env.NODE22,
  join(root, "../tools/node_modules/.bin/node"),
  join(root, "node_modules/.bin/node"),
  "node",
].filter(Boolean);

let node = "node";
for (const c of candidates) {
  if (c === "node" || existsSync(c)) {
    const v = spawnSync(c, ["--version"], { encoding: "utf8" });
    if (v.status === 0 && /^v(2[2-9]|[3-9])/.test((v.stdout || "").trim())) {
      node = c;
      break;
    }
  }
}

const r = spawnSync(node, ["--experimental-strip-types", cli, ...process.argv.slice(2)], {
  stdio: "inherit",
  env: process.env,
});
process.exit(r.status ?? 1);
