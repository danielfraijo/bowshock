#!/usr/bin/env node
// Rendered-DOM audit: for visible text elements compute WCAG contrast of computed colour vs the nearest
// opaque ancestor background, report anything < 4.5:1 (3:1 for large text), plus a keyboard focus-ring check.
// Usage: node scripts/ui-redesign/audit-dom.mjs [baseUrl]
import { chromium } from "playwright";
const base = process.argv[2] ?? "http://localhost:8081/";
const browser = await chromium.launch({ executablePath: process.env.CHROME ?? "/usr/bin/google-chrome", args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"] });
const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
await page.goto(base, { waitUntil: "networkidle" });
await page.evaluate(() => localStorage.clear());
await page.reload({ waitUntil: "networkidle" });
await page.waitForTimeout(2500);
const GROUP = { Geom: "Design", Flight: "Design", Aero: "Analyze", Heat: "Analyze", Frontier: "Analyze", Stab: "Analyze", "6DOF": "Dynamics", Traj: "Dynamics", DATCOM: "Verify", Checks: "Verify", CAD: "Output" };
const tab = async (label) => {
  const sel = (t) => page.locator(`[role=tab]:text-is("${t}")`).first();
  if (!(await sel(label).isVisible().catch(() => false))) await sel(GROUP[label]).click();
  await sel(label).click(); await page.waitForTimeout(600);
};
const audit = () => page.evaluate(() => {
  const parse = (c) => { const m = c.match(/rgba?\(([^)]+)\)/); if (!m) return null; const p = m[1].split(/[ ,/]+/).filter(Boolean).map(Number); return { r: p[0], g: p[1], b: p[2], a: p[3] ?? 1 }; };
  const lin = (v) => { v /= 255; return v <= 0.03928 ? v / 12.92 : ((v + 0.055) / 1.055) ** 2.4; };
  const L = (c) => 0.2126 * lin(c.r) + 0.7152 * lin(c.g) + 0.0722 * lin(c.b);
  const over = (f, b) => ({ r: f.r * f.a + b.r * (1 - f.a), g: f.g * f.a + b.g * (1 - f.a), b: f.b * f.a + b.b * (1 - f.a), a: 1 });
  const bgOf = (el) => { const stack = []; for (let e = el; e; e = e.parentElement) { const c = parse(getComputedStyle(e).backgroundColor); if (c && c.a > 0) { stack.push(c); if (c.a >= 1) break; } } let base = { r: 5, g: 8, b: 11, a: 1 }; for (const c of stack.reverse()) base = over(c, base); return base; };
  const out = [];
  const walker = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT);
  const seen = new Set();
  while (walker.nextNode()) {
    const t = walker.currentNode; if (!t.textContent.trim()) continue;
    const el = t.parentElement; if (!el || seen.has(el)) continue; seen.add(el);
    const r = el.getBoundingClientRect(); if (r.width < 1 || r.height < 1) continue;
    const cs = getComputedStyle(el); if (cs.visibility === "hidden" || cs.display === "none") continue;
    if (el.closest("canvas, svg, [aria-hidden=true], script, style") ) continue;
    if (el.closest(":disabled") || el.closest("[data-nodisabled]")) continue;
    let fg = parse(cs.color); if (!fg) continue;
    const bg = bgOf(el); fg = over({ ...fg, a: fg.a * parseFloat(cs.opacity) }, bg);
    const a = L(fg), b = L(bg); const ratio = (Math.max(a, b) + 0.05) / (Math.min(a, b) + 0.05);
    const size = parseFloat(cs.fontSize), bold = parseInt(cs.fontWeight) >= 700;
    const need = size >= 24 || (size >= 18.66 && bold) ? 3 : 4.5;
    out.push({ text: t.textContent.trim().slice(0, 28), ratio: +ratio.toFixed(2), need, cls: (el.className?.toString?.() ?? "").slice(0, 50), size });
  }
  return out;
});
let total = 0, fails = [];
const views = [["Geom"], ["Flight"], ["Aero"], ["Heat"], ["Frontier"], ["Stab"], ["6DOF"], ["Traj"], ["Checks"], ["DATCOM"], ["CAD"]];
for (const [v] of views) {
  await tab(v);
  if (v === "DATCOM") { await page.getByRole("button", { name: /Run DATCOM/ }).click(); await page.waitForTimeout(3500); }
  const r = await audit(); total += r.length;
  for (const x of r) if (x.ratio < x.need) fails.push({ view: v, ...x });
  console.log(`${v}: ${r.length} text elements, min ratio ${Math.min(...r.map((x) => x.ratio))}`);
}
console.log(`\n${total} text elements checked; ${fails.length} below AA`);
for (const f of fails.slice(0, 40)) console.log(" FAIL", JSON.stringify(f));
// focus ring check: tab through first 40 focusables, make sure each shows a non-none outline
await page.reload({ waitUntil: "networkidle" }); await page.waitForTimeout(1500);
let bad = [];
for (let i = 0; i < 40; i++) {
  await page.keyboard.press("Tab");
  const info = await page.evaluate(() => { const e = document.activeElement; if (!e || e === document.body) return null; const cs = getComputedStyle(e); return { tag: e.tagName, text: (e.textContent || e.getAttribute("aria-label") || e.className || "").toString().trim().slice(0, 24), outlineStyle: cs.outlineStyle, outlineWidth: cs.outlineWidth, outlineColor: cs.outlineColor, role: e.getAttribute("role") }; });
  if (info && (info.outlineStyle === "none" || parseFloat(info.outlineWidth) < 2)) bad.push(info);
}
console.log(`\nfocus ring: ${bad.length} of 40 tab stops without a >=2px outline`); for (const b of bad) console.log(" ", JSON.stringify(b));
await browser.close();
