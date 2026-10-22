/**
 * Summary, plot series, and bundle export over the `aerodb.ts` table.
 *
 * Pure functions, no DOM. Trim and static margin are read on the anchor slice
 * the table already uses for its plots: β nearest 0, δ nearest 0.
 *
 * Trim α is the first Cm = 0 crossing along α, linearly interpolated between
 * breakpoints. Static margin is −Cmα/CLα. Cm is about the CG and
 * nondimensional on Sref·Lref, so the ratio is a fraction of Lref (the
 * geometric length) and × Lref gives x_np − x_cg in metres. Cmα and CLα come
 * from the table's own central differences when the node has a two-sided
 * stencil; otherwise a one-sided difference of the neighbouring breakpoints.
 */
import { aeroDbToCsv, aeroDbToJson, aeroDbToSimulink, type AeroDatabase, type AeroDbPoint } from "./aerodb";
import { buildZip, utf8, type ZipEntry } from "./zip";

export const AERODB_COEFS = ["cl", "cd", "cy", "cm", "cn", "cll", "ca", "ld"] as const;
export type AeroDbCoef = (typeof AERODB_COEFS)[number];
export const COEF_LABEL: Record<AeroDbCoef, string> = { cl: "CL", cd: "CD", cy: "CY", cm: "Cm", cn: "Cn", cll: "Cll", ca: "CA", ld: "LD" };

export interface CoefRange {
  min: number;
  max: number;
}

export interface TrimRow {
  mach: number;
  /** Trim α in degrees, or null when Cm does not cross zero inside the α range. */
  alphaTrim: number | null;
  /** dCm/dα at the crossing, per degree. Negative is a stable trim. */
  dCmdAlphaPerDeg: number | null;
}

export interface MarginRow {
  mach: number;
  /** α the derivatives were taken at (trim α, or the node nearest the design α when untrimmed). */
  alpha_deg: number;
  claPerRad: number;
  cmaPerRad: number;
  /** −Cmα/CLα as a fraction of Lref. Positive = neutral point aft of the CG. */
  smFrac: number | null;
  /** smFrac × Lref, metres. */
  sm_m: number | null;
  source: "table derivative" | "finite difference";
}

export interface AeroDbExtraSummary {
  ranges: Record<AeroDbCoef, CoefRange | null>;
  anchor: { beta_deg: number; delta_deg: number };
  trim: TrimRow[];
  margin: MarginRow[];
}

const DEG = Math.PI / 180;

function nearestOf(axis: number[], value: number): number {
  let best = axis[0] ?? 0;
  for (const v of axis) if (Math.abs(v - value) < Math.abs(best - value)) best = v;
  return best;
}

export function coefRanges(points: AeroDbPoint[]): Record<AeroDbCoef, CoefRange | null> {
  const out = {} as Record<AeroDbCoef, CoefRange | null>;
  for (const c of AERODB_COEFS) {
    let min = Infinity;
    let max = -Infinity;
    for (const p of points) {
      const v = p[c];
      if (!Number.isFinite(v)) continue;
      if (v < min) min = v;
      if (v > max) max = v;
    }
    out[c] = min === Infinity ? null : { min, max };
  }
  return out;
}

/** First Cm = 0 crossing along α (linear interpolation). An exact zero counts. */
export function trimCrossing(alpha: number[], cm: number[]): { alpha: number; slope: number } | null {
  for (let i = 0; i < alpha.length; i++) {
    const c0 = cm[i];
    if (c0 === 0) {
      const j = i + 1 < alpha.length ? i + 1 : i - 1;
      const slope = j >= 0 && j !== i ? (cm[j] - c0) / (alpha[j] - alpha[i]) : NaN;
      return { alpha: alpha[i], slope };
    }
    if (i + 1 < alpha.length) {
      const c1 = cm[i + 1];
      if (c0 * c1 < 0) {
        const t = c0 / (c0 - c1);
        return { alpha: alpha[i] + t * (alpha[i + 1] - alpha[i]), slope: (c1 - c0) / (alpha[i + 1] - alpha[i]) };
      }
    }
  }
  return null;
}

/** Anchor-slice points for one Mach, sorted by α. */
function alphaRow(db: AeroDatabase, mach: number, beta: number, delta: number): AeroDbPoint[] {
  return db.points.filter((p) => p.mach === mach && p.beta_deg === beta && p.delta_deg === delta).sort((a, b) => a.alpha_deg - b.alpha_deg);
}

export function summarizeAeroDb(db: AeroDatabase, designAlphaDeg = 0): AeroDbExtraSummary {
  const beta = nearestOf(db.axes.beta_deg, 0);
  const delta = nearestOf(db.axes.delta_deg, 0);
  const lRef = db.meta.lRef_m;
  const trim: TrimRow[] = [];
  const margin: MarginRow[] = [];
  for (const mach of db.axes.mach) {
    const row = alphaRow(db, mach, beta, delta);
    if (!row.length) continue;
    const a = row.map((p) => p.alpha_deg);
    const cm = row.map((p) => p.cm);
    const t = trimCrossing(a, cm);
    trim.push({ mach, alphaTrim: t ? t.alpha : null, dCmdAlphaPerDeg: t && Number.isFinite(t.slope) ? t.slope : null });
    if (row.length < 2) continue;
    const aRef = t ? t.alpha : nearestOf(a, designAlphaDeg);
    // node nearest the reference α; prefer its two-sided table derivative
    let k = 0;
    for (let i = 1; i < row.length; i++) if (Math.abs(a[i] - aRef) < Math.abs(a[k] - aRef)) k = i;
    const node = row[k];
    let cla: number;
    let cma: number;
    let source: MarginRow["source"];
    let at: number;
    if (k > 0 && k < row.length - 1 && node.cla !== 0) {
      cla = node.cla;
      cma = node.cma;
      source = "table derivative";
      at = node.alpha_deg;
    } else {
      let i = 0;
      while (i < row.length - 2 && a[i + 1] < aRef) i++;
      const span = (a[i + 1] - a[i]) * DEG;
      cla = (row[i + 1].cl - row[i].cl) / span;
      cma = (row[i + 1].cm - row[i].cm) / span;
      source = "finite difference";
      at = aRef;
    }
    const smFrac = Math.abs(cla) > 1e-9 ? -cma / cla : null;
    margin.push({ mach, alpha_deg: at, claPerRad: cla, cmaPerRad: cma, smFrac, sm_m: smFrac === null ? null : smFrac * lRef, source });
  }
  return { ranges: coefRanges(db.points), anchor: { beta_deg: beta, delta_deg: delta }, trim, margin };
}

export const machKey = (m: number) => `M${Math.round(m * 100) / 100}`;

export type SeriesRow = { x: number } & Record<string, number | null>;

/** A coefficient vs α on the anchor slice, one key per Mach. */
export function coefVsAlphaByMach(db: AeroDatabase, coef: AeroDbCoef): { rows: SeriesRow[]; keys: string[] } {
  const beta = nearestOf(db.axes.beta_deg, 0);
  const delta = nearestOf(db.axes.delta_deg, 0);
  const keys = db.axes.mach.map(machKey);
  const rows = db.axes.alpha_deg.map((x) => {
    const r = { x } as SeriesRow;
    for (const m of db.axes.mach) {
      const p = db.points.find((q) => q.mach === m && q.alpha_deg === x && q.beta_deg === beta && q.delta_deg === delta);
      r[machKey(m)] = p && Number.isFinite(p[coef]) ? p[coef] : null;
    }
    return r;
  });
  return { rows, keys };
}

/** A coefficient vs δ at β nearest 0 and α nearest `alphaDeg`, one key per Mach. */
export function coefVsDeltaByMach(db: AeroDatabase, coef: AeroDbCoef, alphaDeg: number): { rows: SeriesRow[]; keys: string[]; alpha_deg: number } {
  const beta = nearestOf(db.axes.beta_deg, 0);
  const alpha = nearestOf(db.axes.alpha_deg, alphaDeg);
  const keys = db.axes.mach.map(machKey);
  const rows = db.axes.delta_deg.map((x) => {
    const r = { x } as SeriesRow;
    for (const m of db.axes.mach) {
      const p = db.points.find((q) => q.mach === m && q.alpha_deg === alpha && q.beta_deg === beta && q.delta_deg === x);
      r[machKey(m)] = p && Number.isFinite(p[coef]) ? p[coef] : null;
    }
    return r;
  });
  return { rows, keys, alpha_deg: alpha };
}

/** "−4, 0, 4, 8" → [-4, 0, 4, 8]. Blank → []. Throws on a token that is not a number. */
export function parseAxisList(text: string): number[] {
  const tokens = text
    .split(/[,;\s]+/)
    .map((t) => t.trim().replace(/\u2212/g, "-"))
    .filter(Boolean);
  return tokens.map((t) => {
    const v = Number(t);
    if (!Number.isFinite(v)) throw new Error(`"${t}" is not a number`);
    return v;
  });
}

function num(v: number): string {
  return Number.isFinite(v) ? Number(v.toPrecision(8)).toString() : "";
}

/**
 * One CSV per coefficient. Rows are (mach, alpha, beta); columns are the δ breakpoints.
 * A Mach × α block at fixed β is a 2-D Lookup Table per δ column.
 */
export function aeroDbCoefficientCsvs(db: AeroDatabase): { name: string; text: string }[] {
  const { axes, meta } = db;
  const nA = axes.alpha_deg.length;
  const nB = axes.beta_deg.length;
  const nD = axes.delta_deg.length;
  return AERODB_COEFS.map((c) => {
    const lines = [
      `# cuspis aero table ${COEF_LABEL[c]}  ${meta.format}  method ${meta.method}`,
      `# sref_m2 ${meta.sRef_m2}  lref_m ${meta.lRef_m}  bref_m ${meta.bRef_m}  cg_m ${meta.cg_m.join(" ")}`,
      `# angles deg; rows mach, alpha_deg, beta_deg; one column per delta_deg breakpoint`,
      ["mach", "alpha_deg", "beta_deg", ...axes.delta_deg.map((d) => `delta_${num(d)}`)].join(","),
    ];
    for (let im = 0; im < axes.mach.length; im++) {
      for (let ia = 0; ia < nA; ia++) {
        for (let ib = 0; ib < nB; ib++) {
          const cells = [num(axes.mach[im]), num(axes.alpha_deg[ia]), num(axes.beta_deg[ib])];
          for (let id = 0; id < nD; id++) {
            const p = db.points[((im * nA + ia) * nB + ib) * nD + id];
            cells.push(p ? num(p[c]) : "");
          }
          lines.push(cells.join(","));
        }
      }
    }
    return { name: `coefficients/${COEF_LABEL[c]}.csv`, text: lines.join("\n") + "\n" };
  });
}

function fmtN(v: number | null | undefined, d: number): string {
  if (v === null || v === undefined || !Number.isFinite(v)) return "—";
  return (Math.abs(v) < 1e-9 ? 0 : v).toFixed(d);
}

export function aeroDbSummaryText(db: AeroDatabase, s: AeroDbExtraSummary): string {
  const lines = [
    `${db.meta.name} (${db.meta.family}) aero database, ${db.meta.method}, ${db.points.length} points`,
    `Max L/D ${fmtN(db.summary.maxLd, 3)} at M ${db.summary.maxLdAt.mach}, α ${db.summary.maxLdAt.alpha_deg}°, β ${db.summary.maxLdAt.beta_deg}°, δ ${db.summary.maxLdAt.delta_deg}°`,
    "Ranges:",
    ...AERODB_COEFS.map((c) => `  ${COEF_LABEL[c]}: ${fmtN(s.ranges[c]?.min, 5)} … ${fmtN(s.ranges[c]?.max, 5)}`),
    `Trim and static margin on β = ${s.anchor.beta_deg}°, δ = ${s.anchor.delta_deg}° (SM = −Cmα/CLα, fraction of Lref ${db.meta.lRef_m} m):`,
    ...s.trim.map((t) => {
      const m = s.margin.find((r) => r.mach === t.mach);
      const trimS = t.alphaTrim === null ? "none in range" : `${t.alphaTrim.toFixed(2)}°`;
      const smS = m && m.smFrac !== null ? `${(m.smFrac * 100).toFixed(2)}% Lref (${fmtN(m.sm_m, 4)} m) at α ${m.alpha_deg.toFixed(2)}°, ${m.source}` : "—";
      return `  M ${t.mach}: trim α ${trimS}; SM ${smS}`;
    }),
  ];
  return lines.join("\n") + "\n";
}

/** Zip: long CSV, JSON, Simulink .m, one CSV per coefficient, and a summary README. */
export function aeroDbZip(db: AeroDatabase, designAlphaDeg = 0): { blob: Blob; entries: string[] } {
  const s = summarizeAeroDb(db, designAlphaDeg);
  const files = [
    { name: "aerodb.csv", text: aeroDbToCsv(db) },
    { name: "aerodb.json", text: aeroDbToJson(db) },
    { name: "cuspis_aero_db.m", text: aeroDbToSimulink(db) },
    ...aeroDbCoefficientCsvs(db),
    { name: "README.txt", text: aeroDbSummaryText(db, s) },
  ];
  const entries: ZipEntry[] = files.map((f) => ({ name: f.name, data: utf8(f.text) }));
  return { blob: buildZip(entries), entries: files.map((f) => f.name) };
}
