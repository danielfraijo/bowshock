// Browser smoke for the perf layer: loads the app in headless Chrome, checks that the compute worker(s)
// start, the wasm loads, there are no console errors, and measures main-thread long tasks while the
// design is being recomputed. Usage: node scripts/web/smoke.mjs http://localhost:8080 [--inline]
import { chromium } from "playwright-core";
const url = process.argv[2] ?? "http://localhost:8080";
const browser = await chromium.launch({ executablePath: process.env.CHROME ?? "/usr/bin/google-chrome", headless: true, args: ["--no-sandbox"] });
const page = await browser.newPage({ viewport: { width: 1500, height: 950 } });
const errors = [];
const workers = [];
const reqs = [];
page.on("console", (m) => {
  if (m.type() === "error" || m.type() === "warning") errors.push(`${m.type()}: ${m.text()}`);
});
page.on("pageerror", (e) => errors.push("pageerror: " + e.message));
page.on("worker", (w) => workers.push(w.url()));
page.context().on("response", (r) => {
  const u = r.url();
  if (/wasm|worker/i.test(u)) reqs.push({ url: u.replace(/^https?:\/\/[^/]+/, ""), status: r.status(), type: r.headers()["content-type"], cache: r.headers()["cache-control"] });
});
await page.addInitScript(() => {
  window.__long = [];
  new PerformanceObserver((l) => l.getEntries().forEach((e) => window.__long.push(e.duration))).observe({ entryTypes: ["longtask"] });
});
await page.goto(url, { waitUntil: "networkidle" });
await page.waitForTimeout(2500);
console.log("workers:", workers);
console.log("wasm/worker requests:", JSON.stringify(reqs, null, 1));
// idle baseline: long tasks with no interaction (headless software WebGL rendering shows up here, not compute)
await page.evaluate(() => (window.__long.length = 0));
await page.waitForTimeout(3000);
console.log("idle 3 s:", JSON.stringify(await page.evaluate(() => ({ n: window.__long.length, sum: Math.round(window.__long.reduce((a, b) => a + b, 0)), max: Math.max(0, ...window.__long) }))));
// drag a slider: find first range input
const sliders = await page.$$('[role="slider"]');
console.log("sliders:", sliders.length);
await page.evaluate(() => (window.__long.length = 0));
const box = sliders.length ? await sliders[0].boundingBox() : null;
let frames = 0;
if (box) {
  await page.evaluate(() => {
    window.__frames = [];
    let last = performance.now();
    const tick = () => {
      const n = performance.now();
      window.__frames.push(n - last);
      last = n;
      window.__raf = requestAnimationFrame(tick);
    };
    tick();
  });
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await page.mouse.down();
  for (let i = 0; i < 40; i++) {
    await page.mouse.move(box.x + box.width / 2 + (i % 20) * 3, box.y + box.height / 2);
    await page.waitForTimeout(16);
  }
  await page.mouse.up();
  await page.waitForTimeout(2000);
  const r = await page.evaluate(() => {
    cancelAnimationFrame(window.__raf);
    const f = [...window.__frames].sort((a, b) => a - b);
    return { frames: f.length, p50: f[Math.floor(f.length / 2)], p95: f[Math.floor(f.length * 0.95)], max: f[f.length - 1], long: window.__long };
  });
  console.log("drag:", JSON.stringify(r));
}
// heavy tabs: the Data tab tables and a DATCOM run must not block the main thread (they run on the bulk worker lane)
async function phase(name, fn) {
  await page.evaluate(() => (window.__long.length = 0));
  const t0 = Date.now();
  await fn();
  const r = await page.evaluate(() => window.__long.slice());
  console.log(`${name}: ${Date.now() - t0} ms wall, long tasks ${r.length} (max ${Math.max(0, ...r).toFixed(0)} ms, sum ${r.reduce((a, b) => a + b, 0).toFixed(0)} ms)`);
}
if ((await page.getByRole("button", { name: "Data", exact: true }).count()) === 0) console.log("Data tab: not present in this domain (external); skipped");
else await phase("Data tab (datapack in worker)", async () => {
  await page.getByRole("button", { name: "Data", exact: true }).click();
  await page.waitForFunction(() => /Validation/.test(document.body.innerText) && /pass/.test(document.body.innerText), null, { timeout: 60000 }).catch(() => console.log("  (data pack text not seen)"));
  await page.waitForTimeout(500);
});
await phase("DATCOM run (worker)", async () => {
  await page.getByRole("button", { name: "DATCOM", exact: true }).click();
  const run = page.getByRole("button", { name: /Run DATCOM/ });
  await run.click();
  await page.waitForTimeout(1500);
});
console.log("workers started:", workers.length, workers.map((w) => w.replace(/^https?:\/\/[^/]+/, "")));
console.log("worker stats:", JSON.stringify(await page.evaluate(() => window.__bowshockCompute?.stats())));
console.log("errors:", errors.length ? errors : "none");
await browser.close();
