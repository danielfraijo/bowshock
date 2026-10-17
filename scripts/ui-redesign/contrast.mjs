#!/usr/bin/env node
// WCAG 2.x contrast check for the HUD design tokens. Parses @theme colours from src/styles.css.
// Usage: node scripts/ui-redesign/contrast.mjs [--md]   (exit 1 on any AA failure of a required pair)
import { readFileSync } from "node:fs";
const css = readFileSync(new URL("../../src/styles.css", import.meta.url), "utf8");
const tok = {};
for (const m of css.matchAll(/--color-([a-z0-9-]+):\s*(#[0-9a-fA-F]{6})/g)) tok[m[1]] = m[2];
const lum = (hex) => {
  const c = [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), 16) / 255).map((v) => (v <= 0.03928 ? v / 12.92 : ((v + 0.055) / 1.055) ** 2.4));
  return 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2];
};
const ratio = (a, b) => { const [x, y] = [lum(a), lum(b)].sort((p, q) => q - p); return (x + 0.05) / (y + 0.05); };
const mix = (fg, bg, a) => "#" + [1, 3, 5].map((i) => Math.round(parseInt(fg.slice(i, i + 2), 16) * a + parseInt(bg.slice(i, i + 2), 16) * (1 - a)).toString(16).padStart(2, "0")).join("");
const T = tok;
const accentTintBg = mix(T.accent, T["surface-2"], 0.2); // selected-card gradient start (20% accent over surface)
// [label, fg, bg, min, kind]
const pairs = [
  ["Body text on bg", T.fg, T.bg, 4.5, "text"],
  ["Body text on surface", T.fg, T.surface, 4.5, "text"],
  ["Body text on surface-2 (cards/inputs)", T.fg, T["surface-2"], 4.5, "text"],
  ["Body text on surface-3 (hover)", T.fg, T["surface-3"], 4.5, "text"],
  ["Muted text on bg", T.muted, T.bg, 4.5, "text"],
  ["Muted text on surface", T.muted, T.surface, 4.5, "text"],
  ["Muted text on surface-2", T.muted, T["surface-2"], 4.5, "text"],
  ["Muted text on surface-3 (hover)", T.muted, T["surface-3"], 4.5, "text"],
  ["Subtle (tertiary) text on bg", T.subtle, T.bg, 4.5, "text"],
  ["Subtle text on surface", T.subtle, T.surface, 4.5, "text"],
  ["Subtle text on surface-2", T.subtle, T["surface-2"], 4.5, "text"],
  ["Accent text on bg (tab labels, readouts)", T.accent, T.bg, 4.5, "text"],
  ["Accent text on surface", T.accent, T.surface, 4.5, "text"],
  ["Accent text on surface-2", T.accent, T["surface-2"], 4.5, "text"],
  ["Accent text on selected-card tint", T.accent, accentTintBg, 4.5, "text"],
  ["Muted text on selected-card tint", T.muted, accentTintBg, 4.5, "text"],
  ["Dark text on accent fill (active chips, CTA)", T["accent-fg"], T.accent, 4.5, "text"],
  ["Dark text at 70% on accent fill (card sub-label)", mix(T["accent-fg"], T.accent, 0.7), T.accent, 4.5, "text"],
  ["Warn text on bg", T.warn, T.bg, 4.5, "text"],
  ["Warn text on surface-2", T.warn, T["surface-2"], 4.5, "text"],
  ["Warn text on surface", T.warn, T.surface, 4.5, "text"],
  ["Chart axis tick (muted) on surface-2", T.muted, T["surface-2"], 4.5, "text"],
  ["Chart series 1 (accent) on bg", T["chart-1"], T.bg, 3, "graphic"],
  ["Chart series 2 (steel white) on bg", T["chart-2"], T.bg, 3, "graphic"],
  ["Chart series 3 (grey-blue, dotted) on bg", T["chart-3"], T.bg, 3, "graphic"],
  ["Control outline (line) on bg", T.line, T.bg, 3, "graphic"],
  ["Control outline (line) on surface", T.line, T.surface, 3, "graphic"],
  ["Control outline (line) on surface-2", T.line, T["surface-2"], 3, "graphic"],
  ["Focus ring (accent) on bg", T.accent, T.bg, 3, "graphic"],
  ["Focus ring (accent) on surface-2", T.accent, T["surface-2"], 3, "graphic"],
  ["Focus ring (dark) inside accent fill", T["accent-fg"], T.accent, 3, "graphic"],
  ["Decorative hairline (border) on bg — informational", T.border, T.bg, 1.2, "decor"],
  ["Disabled text on surface-2 — exempt (WCAG 1.4.3), informational", T.disabled, T["surface-2"], 1, "exempt"],
];
let fail = 0;
const rows = pairs.map(([l, f, b, min, kind]) => {
  const r = ratio(f, b);
  const ok = r >= min;
  if (!ok) fail++;
  return { l, f, b, r, min, ok, kind };
});
if (process.argv.includes("--md")) {
  console.log("| Pair | FG | BG | Ratio | Required | Result |\n|---|---|---|---|---|---|");
  for (const x of rows) console.log(`| ${x.l} | \`${x.f}\` | \`${x.b}\` | ${x.r.toFixed(2)}:1 | ${x.min}:1 (${x.kind}) | ${x.ok ? "PASS" : "**FAIL**"} |`);
} else {
  for (const x of rows) console.log(`${x.ok ? "PASS" : "FAIL"}  ${x.r.toFixed(2).padStart(6)}:1  (>= ${x.min})  ${x.l}   ${x.f} on ${x.b}`);
}
console.log(fail ? `\n${fail} FAILURE(S)` : "\nAll required pairs pass WCAG AA.");
process.exit(fail ? 1 : 0);
