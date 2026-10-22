#!/usr/bin/env node
// [p7-project] Usage: node scripts/ui-redesign/p7-shots.mjs [baseUrl]
// Drives the Projects panel end to end (save, snapshot, Iterate aero DB, variant, compare) and screenshots it.
import { chromium } from "playwright";
const base = process.argv[2] ?? "http://localhost:8081/";
const out = process.env.SHOT_DIR ?? "/workspace/bowshock-patches/p7-project/screens";
const browser = await chromium.launch({
  executablePath: process.env.CHROME ?? "/usr/bin/google-chrome",
  args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"],
});
const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
const errors = [];
page.on("pageerror", (e) => { errors.push(e.message); console.log("pageerror", e.message); });
page.on("dialog", async (d) => { await d.accept(d.type() === "prompt" ? (d.defaultValue() || "variant-b") : undefined); });
await page.goto(base, { waitUntil: "networkidle" });
await page.evaluate(async () => { localStorage.clear(); await new Promise((r) => { const q = indexedDB.deleteDatabase("bowshock-projects"); q.onsuccess = q.onerror = q.onblocked = () => r(); }); });
await page.reload({ waitUntil: "networkidle" });
await page.waitForTimeout(2500);
const shot = async (name) => { await page.waitForTimeout(600); await page.screenshot({ path: `${out}/${name}.png` }); console.log("saved", name); };
const dirty = () => page.getByTestId("unsaved-indicator").getAttribute("data-dirty");
const panel = () => page.getByTestId("projects-panel");
const openPanel = async () => { if (!(await panel().isVisible().catch(() => false))) await page.getByTestId("projects-open").click(); };
const closePanel = async () => { await page.keyboard.press("Escape"); await page.waitForTimeout(200); };
const pane = async (label) => panel().locator(".hud-seg button").filter({ hasText: new RegExp(`^${label}`) }).click();
const snapshot = async (msg) => { await openPanel(); await pane("History"); await panel().getByPlaceholder("Snapshot message").fill(msg); await page.getByTestId("snapshot-btn").click(); await page.waitForTimeout(400); };
const tab = async (group, label) => {
  const aside = page.locator("aside").last();
  const sel = (t) => aside.locator("[role=tab]").filter({ hasText: new RegExp(`^${t}$`) }).first();
  if (!(await sel(label).isVisible().catch(() => false))) await sel(group).click();
  await sel(label).click();
};
const buildDb = async () => {
  await tab("Iterate", "Iterate");
  await page.getByRole("button", { name: "Build aero database" }).click();
  await page.waitForFunction(() => !/Building table/.test(document.body.innerText), null, { timeout: 180000 });
  await page.waitForTimeout(500);
};

await shot("p7-01-header-new-project");
await openPanel();
await page.getByTestId("project-name").fill("Caret M8 study");
await panel().getByRole("button", { name: /^Save$/ }).click();
await page.waitForTimeout(500);
console.log("dirty after save:", await dirty());
await shot("p7-02-project-pane-saved");
await snapshot("baseline caret M8");
await closePanel();

await buildDb();
console.log("dirty after aero DB:", await dirty());
await shot("p7-03-iterate-aerodb-unsaved");
await snapshot("baseline + aero DB");

// variant B: Caret M5 preset, own aero DB
await closePanel();
await page.getByRole("button", { name: "Presets" }).click();
await page.getByRole("button", { name: /Caret M5/ }).click();
await page.waitForTimeout(1500);
await buildDb();
await snapshot("variant B: caret M5");
// variant C: Star 4-fin, no aero DB
await closePanel();
await page.getByRole("button", { name: /Star 4-fin/ }).click();
await page.waitForTimeout(1500);
await snapshot("variant C: star 4-fin (no DB)");
await shot("p7-04-history-list");

// branch from the baseline, then restore variant B
await panel().getByRole("listitem").filter({ hasText: "baseline caret M8" }).getByRole("button", { name: "Branch" }).click();
await page.waitForTimeout(500);
await shot("p7-05-history-branched");
await panel().getByRole("listitem").filter({ hasText: "variant B" }).getByRole("button", { name: "Restore" }).click();
await page.waitForTimeout(800);
console.log("dirty after restore:", await dirty());

await pane("Compare");
for (const t of ["baseline + aero DB", "variant B", "variant C"]) await page.getByTestId("project-compare").locator("button").filter({ hasText: t }).first().click();
await page.getByRole("button", { name: "Diff + metrics" }).click();
await page.waitForFunction(() => !/Evaluating/.test(document.body.innerText), null, { timeout: 180000 });
await shot("p7-06-compare-diff-metrics");
await panel().locator("div.overflow-y-auto").first().evaluate((el) => (el.scrollTop = el.scrollHeight));
await shot("p7-07-compare-aerodb-overlay");
await closePanel();
await shot("p7-08-header-unsaved-after-restore");
console.log("errors:", errors.length);
await browser.close();
