import type { SurfaceGrid, Vec3 } from "./types";
import { gridPoint } from "./mesh";
import { surfacePatches } from "./export-plot3d";

function pad80(s: string) {
  return (s.length >= 80 ? s.slice(0, 80) : s + " ".repeat(80 - s.length));
}

function deLine(type: number, paramStart: number, form: number, de: number, extra = 0) {
  const seq = de * 2 - 1;
  const a = `${String(type).padStart(8)}${String(paramStart).padStart(8)}${String(0).padStart(8)}${String(0).padStart(8)}${String(0).padStart(8)}${String(0).padStart(8)}${String(0).padStart(8)}${String(0).padStart(8)}`;
  const b = `${String(type).padStart(8)}${String(0).padStart(8)}${String(0).padStart(8)}${String(extra).padStart(8)}${String(form).padStart(8)}        ${String(0).padStart(8)}${String(seq + 1).padStart(8)}`;
  return [pad80(a.slice(0, 72) + "D" + String(seq).padStart(7)), pad80(b.slice(0, 72) + "D" + String(seq + 1).padStart(7))];
}

function paramLines(type: number, values: Array<number | string>, de: number, startP: number): string[] {
  const payload = `${type},${values.join(",")};`;
  const chunks: string[] = [];
  let i = 0;
  const width = 64;
  while (i < payload.length) {
    chunks.push(payload.slice(i, i + width));
    i += width;
  }
  return chunks.map((c, k) => {
    const pnum = startP + k;
    const body = c + " ".repeat(Math.max(0, 64 - c.length));
    return pad80(`${body}${String(de).padStart(8)}P${String(pnum).padStart(7)}`);
  });
}

function knotsFor(nPoles: number, degree: number): number[] {
  const p = Math.min(degree, nPoles - 1);
  const knots: number[] = [];
  for (let i = 0; i < p + 1; i++) knots.push(0);
  const internal = nPoles - p - 1;
  for (let i = 1; i <= internal; i++) knots.push(i / (internal + 1));
  for (let i = 0; i < p + 1; i++) knots.push(1);
  return knots;
}

function downsample(g: SurfaceGrid, capU = 18, capV = 16): Vec3[][] {
  const du = Math.max(1, Math.floor((g.ni - 1) / Math.min(capU, g.ni - 1)));
  const dv = Math.max(1, Math.floor((g.nj - 1) / Math.min(capV, g.nj - 1)));
  const poles: Vec3[][] = [];
  const is: number[] = [];
  for (let i = 0; i < g.ni; i += du) is.push(i);
  if (is[is.length - 1] !== g.ni - 1) is.push(g.ni - 1);
  const js: number[] = [];
  for (let j = 0; j < g.nj; j += dv) js.push(j);
  if (js[js.length - 1] !== g.nj - 1) js.push(g.nj - 1);
  for (const i of is) {
    const row: Vec3[] = [];
    for (const j of js) row.push(gridPoint(g, i, j));
    poles.push(row);
  }
  return poles;
}

export function gridsToIges(grids: SurfaceGrid[], name: string): string {
  const start = new Date().toISOString();
  const s: string[] = [];
  s.push(pad80(`, Bowshock waverider CAD, ${name}`.slice(0, 72) + "S" + "1".padStart(7)));
  const g1 = pad80(
    `1H,,1H;,${Math.min(8, name.length)}H${name.slice(0, 8)},7HIGES5.3,8HBowshock,8HBowshock,32,38,6,308,15,4HBowshock,1.0,1,4HINCH,32768,0.0,${start},0.000001,1000.0,7HUnknown,7HUnknown,11,0,0;` +
      "G" +
      "1".padStart(7),
  );
  // Global section is finicky; write a conservative 4-line block.
  const globals = [
    `1H,,1H;,8H${(name + "        ").slice(0, 8)},7HIGES5.3,`,
    `8HBowshock,8HBowshock,32,38,6,308,15,`,
    `8HBowshock,1.,2,2HMM,32768,0.,15H${start.replace(/[-:TZ]/g, "").slice(0, 15)},`,
    `1.E-6,1000.,7HUnknown,7HUnknown,11,0,0;`,
  ];
  globals.forEach((line, i) => s.push(pad80(line + " ".repeat(Math.max(0, 72 - line.length)) + "G" + String(i + 1).padStart(7))));

  const surfaces = surfacePatches(grids).filter((g) => g.ni >= 2 && g.nj >= 2).slice(0, 12);
  const dLines: string[] = [];
  const pLines: string[] = [];
  let pCursor = 1;
  let de = 1;

  for (const g of surfaces) {
    const poles = downsample(g);
    const k1 = poles.length - 1;
    const k2 = poles[0].length - 1;
    const m1 = Math.min(3, k1);
    const m2 = Math.min(3, k2);
    const su = knotsFor(k1 + 1, m1);
    const sv = knotsFor(k2 + 1, m2);
    const vals: Array<number | string> = [k1, k2, m1, m2, 0, 0, 0, 0, 0];
    for (const k of su) vals.push(k);
    for (const k of sv) vals.push(k);
    for (let i = 0; i <= k1; i++) for (let j = 0; j <= k2; j++) vals.push(1);
    for (let i = 0; i <= k1; i++) {
      for (let j = 0; j <= k2; j++) {
        const p = poles[i][j];
        vals.push(p[0], p[1], p[2]);
      }
    }
    vals.push(0, 1, 0, 1);
    const pl = paramLines(128, vals, de, pCursor);
    const [d1, d2] = deLine(128, pCursor, 0, (de + 1) / 2);
    // deLine uses de index as entity number; keep sequential DE pointers 1,3,5...
    void d1;
    void d2;
    const seq = de;
    const dirA = `${String(128).padStart(8)}${String(pCursor).padStart(8)}${String(0).padStart(8)}${String(0).padStart(8)}${String(0).padStart(8)}${String(0).padStart(8)}${String(0).padStart(8)}${String(0).padStart(8)}`;
    const dirB = `${String(128).padStart(8)}${String(0).padStart(8)}${String(pl.length).padStart(8)}${String(0).padStart(8)}${String(0).padStart(8)}        ${String(0).padStart(8)}`;
    dLines.push(pad80(dirA.slice(0, 72) + "D" + String(seq).padStart(7)));
    dLines.push(pad80(dirB.slice(0, 72) + "D" + String(seq + 1).padStart(7)));
    pLines.push(...pl);
    pCursor += pl.length;
    de += 2;
  }

  s.push(...dLines);
  s.push(...pLines);
  const t = `S${String(1).padStart(7)}G${String(globals.length).padStart(7)}D${String(dLines.length).padStart(7)}P${String(pLines.length).padStart(7)}`;
  s.push(pad80(t + " ".repeat(Math.max(0, 72 - t.length)) + "T" + "1".padStart(7)));
  return s.join("\n") + "\n";
}
