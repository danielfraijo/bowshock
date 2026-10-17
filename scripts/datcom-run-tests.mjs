#!/usr/bin/env node
import { spawnSync } from "node:child_process";
import { existsSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, "..");
const tests = [
  join(root, "src/lib/waverider/datcom/validate-datcom.test.ts"),
  join(root, "src/lib/waverider/datcom/stress.test.ts"),
];
const candidates = [
  process.env.NODE22,
  join(root, "../tools/node_modules/.bin/node"),
  "/workspace/tools/node_modules/.bin/node",
  "node",
].filter(Boolean);
let node = null;
for (const c of candidates) {
  if (c !== "node" && !existsSync(c)) continue;
  const v = spawnSync(c, ["--version"], { encoding: "utf8" });
  if (v.status === 0 && /^v(2[2-9]|[3-9])/.test((v.stdout || "").trim())) {
    node = c;
    break;
  }
}
if (!node) {
  console.error("test:datcom requires Node >= 22 for --experimental-strip-types");
  process.exit(1);
}
const r = spawnSync(node, ["--experimental-strip-types", "--test", ...tests], { stdio: "inherit" });
process.exit(r.status ?? 1);
