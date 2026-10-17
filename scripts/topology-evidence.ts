// Run: bun scripts/topology-evidence.ts <outDir> [label]
// Writes SVG evidence of the LE fillet: a y=const section of the nose region, and a plan view of the nose mesh.
import { mkdirSync, writeFileSync } from "node:fs";
import { buildVehicle } from "../src/lib/waverider/generate";
import { DEFAULT_PARAMS, type TriMesh } from "../src/lib/waverider/types";

const out = process.argv[2] ?? "evidence";
const label = process.argv[3] ?? "after";
mkdirSync(out, { recursive: true });
const built = buildVehicle({ ...DEFAULT_PARAMS, family: "caret", mach: 12, nx: 80, ny: 56, leRadius: 0.01 });
const mesh: TriMesh = built.mesh;

function slice(y0: number): [number, number, number, number][] {
  const p = mesh.positions;
  const segs: [number, number, number, number][] = [];
  for (let t = 0; t < mesh.indices.length / 3; t++) {
    const v = [0, 1, 2].map((k) => mesh.indices[t * 3 + k] * 3);
    const pts: [number, number][] = [];
    for (let k = 0; k < 3; k++) {
      const a = v[k];
      const b = v[(k + 1) % 3];
      const da = p[a + 1] - y0;
      const db = p[b + 1] - y0;
      if (da * db < 0) {
        const u = da / (da - db);
        pts.push([p[a] + u * (p[b] - p[a]), p[a + 2] + u * (p[b + 2] - p[a + 2])]);
      }
    }
    if (pts.length === 2) segs.push([pts[0][0], pts[0][1], pts[1][0], pts[1][1]]);
  }
  return segs;
}

function svg(title: string, w: number, h: number, body: string) {
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}"><rect width="100%" height="100%" fill="white"/><text x="8" y="16" font-family="monospace" font-size="12">${title}</text>${body}</svg>`;
}

// 1. section normal to y at mid-span, zoomed on the LE (x in [x0, x0+dx])
{
  const y0 = 0.45 * (built.params.span / 2);
  const segs = slice(y0);
  let xmin = Infinity;
  for (const s of segs) xmin = Math.min(xmin, s[0], s[2]);
  const dx = 0.06;
  const sc = 4000;
  const W = 760;
  const H = 300;
  const body = segs
    .filter((s) => s[0] < xmin + dx && s[2] < xmin + dx)
    .map((s) => `<line x1="${20 + (s[0] - xmin) * sc}" y1="${H / 2 - s[1] * sc}" x2="${20 + (s[2] - xmin) * sc}" y2="${H / 2 - s[3] * sc}" stroke="black" stroke-width="1.2"/>`)
    .join("");
  writeFileSync(`${out}/le-section-${label}.svg`, svg(`${label}: y=${y0.toFixed(3)} section, caret M12 nx80 ny56 R=0.01 (x from ${xmin.toFixed(3)} m, 1 px = 0.25 mm)`, W, H, body));
}
// 2. plan view of the nose mesh (upper-surface edges projected on x-y)
{
  const p = mesh.positions;
  const sc = 2600;
  const W = 760;
  const H = 520;
  const lines: string[] = [];
  for (let t = 0; t < mesh.indices.length / 3; t++) {
    const v = [0, 1, 2].map((k) => mesh.indices[t * 3 + k] * 3);
    if (v.some((a) => p[a] > 0.3 || Math.abs(p[a + 1]) > 0.14)) continue;
    const col = mesh.surfaces[t] === 3 ? "#c00" : mesh.surfaces[t] === 1 ? "#99c" : "#222";
    const pts = v.map((a) => `${(20 + p[a] * sc).toFixed(1)},${(H / 2 - p[a + 1] * sc).toFixed(1)}`).join(" ");
    lines.push(`<polygon points="${pts}" fill="none" stroke="${col}" stroke-width="0.5"/>`);
  }
  writeFileSync(`${out}/nose-plan-${label}.svg`, svg(`${label}: nose plan view (x 0..0.3 m, |y|<0.14 m); red = LE fillet strips`, W, H, lines.join("")));
}
console.log(`wrote ${out}/le-section-${label}.svg and nose-plan-${label}.svg`);
