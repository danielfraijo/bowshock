import type { LengthUnit, SurfaceGrid } from "./types";
import { surfacePatches } from "./export-plot3d";
import { gridNet, interpolateNet } from "./export-nurbs";

/** Fixed-format IGES 5.3 writer: one Rational B-Spline Surface (type 128) per patch. */

const UNIT_FLAG: Record<LengthUnit, { flag: number; name: string }> = {
  m: { flag: 6, name: "1HM" },
  mm: { flag: 2, name: "2HMM" },
  in: { flag: 1, name: "2HIN" },
};

/** One 80-column record: text in cols 1-72, section letter in col 73, sequence number in cols 74-80. */
function rec(text: string, section: "S" | "G" | "D" | "P" | "T", seq: number): string {
  if (text.length > 72) throw new Error(`IGES record too long: ${text.length}`);
  return text.padEnd(72, " ") + section + String(seq).padStart(7, " ");
}

function hollerith(s: string): string {
  const clean = s.replace(/[^\x20-\x7e,;]/g, "_").replace(/[,;]/g, "_");
  return `${clean.length}H${clean}`;
}

/** Real number in IGES free format (always with a decimal point). */
function real(v: number): string {
  if (!Number.isFinite(v) || Math.abs(v) < 1e-300) return "0.0";
  const t = Number(v.toPrecision(15));
  let out = t.toString();
  if (/e/i.test(out)) {
    const [m, e] = out.toLowerCase().split("e");
    out = `${m.includes(".") ? m : `${m}.0`}E${e.replace("+", "")}`;
  } else if (!out.includes(".")) out += ".0";
  return out;
}

/** Pack free-format fields into lines of <= width columns without splitting a field. */
function pack(fields: string[], width: number, terminator: string): string[] {
  const lines: string[] = [];
  let cur = "";
  fields.forEach((f, i) => {
    const piece = f + (i === fields.length - 1 ? terminator : ",");
    if (cur.length + piece.length > width && cur.length) {
      lines.push(cur);
      cur = "";
    }
    cur += piece;
  });
  if (cur) lines.push(cur);
  return lines;
}

function stamp(d: Date): string {
  const p = (n: number, w = 2) => String(n).padStart(w, "0");
  return `${p(d.getUTCFullYear(), 4)}${p(d.getUTCMonth() + 1)}${p(d.getUTCDate())}.${p(d.getUTCHours())}${p(d.getUTCMinutes())}${p(d.getUTCSeconds())}`;
}

export function gridsToIges(grids: SurfaceGrid[], name: string, unit: LengthUnit = "m"): string {
  const when = stamp(new Date());
  const u = UNIT_FLAG[unit] ?? UNIT_FLAG.m;
  const start = [rec(`Cuspis waverider CAD: ${name}`.slice(0, 72), "S", 1)];

  const patches = surfacePatches(grids).filter((g) => g.ni >= 2 && g.nj >= 2);
  let extent = 1;
  for (const g of patches) for (let k = 0; k < g.xyz.length; k++) extent = Math.max(extent, Math.abs(g.xyz[k]));
  const gfields = [
    "1H,",
    "1H;",
    hollerith(name.slice(0, 40) || "waverider"),
    hollerith(`${name.slice(0, 40) || "waverider"}.igs`),
    hollerith("Cuspis"),
    hollerith("Cuspis 1.0"),
    "32",
    "38",
    "6",
    "308",
    "15",
    hollerith(name.slice(0, 40) || "waverider"),
    "1.0",
    String(u.flag),
    u.name,
    "1",
    "0.01",
    `15H${when}`,
    "1.0E-6",
    real(Math.ceil(extent * 10)),
    hollerith("Cuspis"),
    hollerith("Cuspis"),
    "11",
    "0",
    `15H${when}`,
  ];
  const glines = pack(gfields, 72, ";");
  const global = glines.map((l, i) => rec(l, "G", i + 1));

  const dEntries: string[] = [];
  const pLines: string[] = [];
  let deSeq = 1;
  let pSeq = 1;
  for (const g of patches) {
    const net = interpolateNet(gridNet(g, 200), 3);
    const k1 = net.ctrl.length - 1;
    const k2 = net.ctrl[0].length - 1;
    const fields: string[] = ["128", String(k1), String(k2), String(net.p), String(net.q), "0", "0", "1", "0", "0"];
    for (const k of net.U) fields.push(real(k));
    for (const k of net.V) fields.push(real(k));
    for (let j = 0; j <= k2; j++) for (let i = 0; i <= k1; i++) fields.push("1.0");
    // control points: first index (i, the U direction) varies fastest
    for (let j = 0; j <= k2; j++)
      for (let i = 0; i <= k1; i++) {
        const c = net.ctrl[i][j];
        fields.push(real(c[0]), real(c[1]), real(c[2]));
      }
    fields.push("0.0", "1.0", "0.0", "1.0");
    const lines = pack(fields, 64, ";").map((l, k) => rec(l.padEnd(64, " ") + String(deSeq).padStart(8, " "), "P", pSeq + k));
    const f8 = (v: number | string) => String(v).padStart(8, " ");
    const line1 = [128, pSeq, 0, 0, 0, 0, 0, 0, "00000000"].map(f8).join("");
    const line2 = [128, 0, 0, lines.length, 0, 0, 0, "", 0].map(f8).join("");
    dEntries.push(rec(line1, "D", deSeq), rec(line2, "D", deSeq + 1));
    pLines.push(...lines);
    pSeq += lines.length;
    deSeq += 2;
  }
  const t = `S${String(start.length).padStart(7, "0")}G${String(global.length).padStart(7, "0")}D${String(dEntries.length).padStart(7, "0")}P${String(pLines.length).padStart(7, "0")}`;
  return [...start, ...global, ...dEntries, ...pLines, rec(t, "T", 1)].join("\n") + "\n";
}
