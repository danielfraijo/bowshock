#!/usr/bin/env node
/**
 * [aerodb-perf] Real-browser check of the aero-database worker pool:
 *   node scripts/aerodb-browser/run.mjs        (CHROME=/path/to/chrome to pick the browser)
 * Builds scripts/aerodb-browser with Vite, serves it on 127.0.0.1, runs generateAeroDatabase with the TS engine and
 * with the default engine (Web Worker pool) in headless Chrome, and prints: engine, cap, worst relative difference,
 * timings, the longest main-thread frame gap while the pool ran, and whether AbortSignal cancelled the run.
 */
import { execFileSync } from "node:child_process";
import { existsSync, readFileSync } from "node:fs";
import { createServer } from "node:http";
import { extname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { chromium } from "playwright";

const repo = fileURLToPath(new URL("../..", import.meta.url));
const dist = join(repo, ".cache", "aerodb-browser");
execFileSync(join(repo, "node_modules", ".bin", "vite"), ["build", "--config", join(repo, "scripts", "aerodb-browser", "vite.config.mjs")], { cwd: repo, stdio: "inherit" });
const srv = createServer((req, res) => {
  const p = join(dist, req.url === "/" ? "index.html" : (req.url ?? "/").split("?")[0]);
  if (!p.startsWith(dist) || !existsSync(p)) {
    res.writeHead(404);
    return res.end();
  }
  res.writeHead(200, { "content-type": extname(p) === ".js" ? "text/javascript" : "text/html" });
  res.end(readFileSync(p));
}).listen(0, "127.0.0.1");
await new Promise((r) => srv.once("listening", r));
const port = srv.address().port;
const browser = await chromium.launch({ executablePath: process.env.CHROME || "/usr/bin/google-chrome", headless: true });
try {
  const page = await browser.newPage();
  page.on("pageerror", (e) => console.log("pageerror:", e.message));
  await page.goto(`http://127.0.0.1:${port}/`);
  await page.waitForFunction(() => document.getElementById("out")?.textContent !== "running", null, { timeout: 180000 });
  const res = JSON.parse((await page.textContent("#out")) ?? "{}");
  console.log(JSON.stringify(res, null, 2));
  if (!res.ok || !String(res.engine).startsWith("pool:") || res.worst > 1e-6 || res.aborted !== "AbortError") process.exitCode = 1;
} finally {
  await browser.close();
  srv.close();
}
