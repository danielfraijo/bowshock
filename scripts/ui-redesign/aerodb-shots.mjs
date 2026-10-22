#!/usr/bin/env node
// Usage: node scripts/ui-redesign/aerodb-shots.mjs [baseUrl]
// Iterate → Aero DB: idle, list axes, then after a run (summary, plots, exports). Same browser setup as shots.mjs.
import { chromium } from "playwright";
const base = process.argv[2] ?? "http://localhost:8081/";
const out = process.env.SHOT_DIR ?? "/workspace/bowshock-patches/aerodb-ui/screens";
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
const aside = page.locator("aside").last();
const scroller = aside.locator("div[class*=overflow-y-auto]").first();
const shot = async (name) => { await page.mouse.move(2, 2); await page.waitForTimeout(700); await page.screenshot({ path: `${out}/${name}.png` }); console.log("saved", name); };
const scrollTo = async (text) => {
  await aside.getByText(text, { exact: true }).first().evaluate((el) => el.scrollIntoView({ block: "start" }));
};
await aside.locator("button, [role=tab]").filter({ hasText: /^Iterate$/ }).first().click();
await page.waitForTimeout(800);
await shot("01-aerodb-idle");
await aside.getByRole("button", { name: "List", exact: true }).click();
const lists = aside.locator("input[type=text][inputmode=decimal]");
await lists.nth(1).fill("-4, 0, 2, 4, 8, 12");
await lists.nth(3).fill("-5, 0, 5");
await shot("02-aerodb-list-axes");
await aside.getByRole("button", { name: /Build aero database/ }).click();
await page.waitForTimeout(400);
await shot("03-aerodb-running");
await aside.getByRole("button", { name: /Build aero database/ }).waitFor({ timeout: 240000 });
await page.waitForTimeout(500);
await scrollTo("Points");
await shot("04-aerodb-summary");
await scrollTo("Coefficient ranges");
await shot("05-aerodb-ranges");
await scroller.evaluate((el) => (el.scrollTop = el.scrollHeight));
await shot("06-aerodb-plots-exports");
await browser.close();
