// Verifies prefers-reduced-motion: scanline sweep + LED pulse animations are off; also shots at narrow widths.
import { chromium } from "playwright";
const b = await chromium.launch({ executablePath: "/usr/bin/google-chrome", args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"] });
for (const rm of ["no-preference", "reduce"]) {
  const ctx = await b.newContext({ viewport: { width: 1440, height: 900 }, reducedMotion: rm });
  const p = await ctx.newPage();
  await p.goto(process.argv[2] ?? "http://localhost:8081/", { waitUntil: "networkidle" }); await p.waitForTimeout(1500);
  const r = await p.evaluate(() => {
    const q = (sel, pseudo) => { const e = document.querySelector(sel); if (!e) return null; const cs = getComputedStyle(e, pseudo); return { animationName: cs.animationName, display: cs.display, animationDuration: cs.animationDuration }; };
    return { sweep: q(".hud-scan", "::after"), led: q(".hud-led"), tabTransition: getComputedStyle(document.querySelector("[role=tab]")).transitionDuration };
  });
  console.log(rm, JSON.stringify(r));
  await ctx.close();
}
const out = process.env.SHOT_DIR ?? "/workspace/bowshock-patches/ui-redesign/screens";
for (const [w, h, n] of [[1024, 768, "tablet-1024"], [390, 844, "mobile-390"]]) {
  const ctx = await b.newContext({ viewport: { width: w, height: h } }); const p = await ctx.newPage();
  await p.goto(process.argv[2] ?? "http://localhost:8081/", { waitUntil: "networkidle" }); await p.waitForTimeout(2500);
  await p.screenshot({ path: `${out}/after-resp-${n}.png` }); console.log("saved", n); await ctx.close();
}
await b.close();
