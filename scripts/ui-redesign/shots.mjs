#!/usr/bin/env node
// Usage: node scripts/ui-redesign/shots.mjs <before|after> [baseUrl]
// Captures the lab at 1440x900 for the views used to review the UI redesign.
import { chromium } from "playwright";
const prefix = process.argv[2] ?? "after";
const base = process.argv[3] ?? "http://localhost:8081/";
const out = process.env.SHOT_DIR ?? "/workspace/bowshock-patches/ui-redesign/screens";
const browser = await chromium.launch({
  executablePath: process.env.CHROME ?? "/usr/bin/google-chrome",
  args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"],
});
const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
page.on("pageerror", (e) => console.log("pageerror", e.message));
await page.goto(base, { waitUntil: "networkidle" });
await page.evaluate(() => localStorage.clear());
await page.reload({ waitUntil: "networkidle" });
await page.waitForTimeout(2500);
const only = process.env.ONLY ? new Set(process.env.ONLY.split(",")) : null; // e.g. ONLY=18-internal-data
const shot = async (name) => { if (only && !only.has(name)) return; await page.waitForTimeout(700); await page.screenshot({ path: `${out}/${prefix}-${name}.png` }); console.log("saved", name); };
// Works with both the old flat tab bar (buttons) and the new two-tier [role=tab] bar.
const GROUP_OF = { Geom: "Design", Flight: "Design", Aero: "Analyze", Heat: "Analyze", Frontier: "Analyze", Stab: "Analyze", Cycle: "Analyze", Shocks: "Analyze", "6DOF": "Dynamics", Traj: "Dynamics", DATCOM: "Verify", Checks: "Verify", Data: "Output", CAD: "Output" };
const tab = async (label) => {
  // scoped to the right-hand panel: the viewport colour-mode chip "HEAT" must not match the Heat tab
  const sel = (t) => page.locator("aside").last().locator(`button, [role=tab]`).filter({ hasText: new RegExp(`^${t}$`) }).first();
  if (!(await sel(label).isVisible().catch(() => false))) await sel(GROUP_OF[label]).click();
  await sel(label).click();
  if (process.env.DEBUG_TABS) console.log("tab", label, await page.locator("[role=tab][aria-selected=true]").allInnerTexts());
};
await shot("01-lab-geom");
for (const [label, name] of [["Flight","02-flight"],["Aero","03-aero"],["Heat","04-heat"],["Frontier","05-frontier"],["Stab","06-stab"],["6DOF","07-6dof"],["Traj","08-traj"],["Checks","09-checks"]]) {
  await tab(label); await shot(name);
}
await tab("DATCOM"); await shot("10-datcom-setup");
await page.getByRole("button", { name: /Run DATCOM/ }).click().catch((e) => console.log("run datcom fail", e.message));
await page.waitForTimeout(4000);
await page.locator("aside").last().locator("div.overflow-y-auto, div[class*=overflow-y-auto]").first().evaluate((el) => (el.scrollTop = 520)).catch(() => {});
await shot("11-datcom-result");
await page.locator("aside").last().locator("div[class*=overflow-y-auto]").first().evaluate((el) => (el.scrollTop = 1400)).catch(() => {});
await shot("12-datcom-result-lower");
await tab("CAD"); await shot("13-cad");
// Internal toolkit
await page.getByRole("button", { name: "Internal", exact: true }).click();
await page.waitForTimeout(1500);
await shot("14-internal-geom");
for (const [label, name] of [["Cycle","15-internal-cycle"],["Shocks","16-internal-shocks"],["Heat","17-internal-heat"],["Data","18-internal-data"]]) {
  await tab(label);
  if (label === "Data") {
    await page.waitForTimeout(2000);
    await page.waitForFunction(() => !/computing/i.test(document.body.innerText), null, { timeout: 120000 }).catch(() => console.log("data pack still computing"));
  }
  await shot(name);
}
await browser.close();
