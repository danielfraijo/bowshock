/**
 * [p7-project] Variant compare: 2–4 versions (snapshots, branch tips or the live working state) side by side.
 * Pure data builder; the metric evaluator is injected (the UI passes a wrapper over mdo.ts evaluateDesign, the
 * same cheap panel evaluation the Iterate lab uses), so this module stays DOM-free and testable.
 */
import type { AeroDatabase } from "../waverider/aerodb.ts";
import type { DesignParams } from "../waverider/types.ts";
import { diffStates } from "./diff.ts";
import { aeroDbStale, canonicalJson, type ProjectDoc, type ProjectState } from "./schema.ts";
import { listSnapshots } from "./history.ts";

export interface CompareEntry {
  id: string;
  label: string;
  state: ProjectState;
}

export interface VariantMetrics {
  ld: number;
  cl: number;
  cd: number;
  cm: number;
  staticMargin: number;
  volume: number;
  sRef: number;
  volEff: number;
  mass: number;
  qStag: number;
  rangeKm: number;
  method?: string;
  error?: string;
}

export type MetricKey = Exclude<keyof VariantMetrics, "method" | "error">;

export const COMPARE_METRICS: { key: MetricKey; label: string; unit: string; better: "max" | "min" | null; digits: number }[] = [
  { key: "ld", label: "L/D", unit: "", better: "max", digits: 3 },
  { key: "cl", label: "CL", unit: "", better: null, digits: 4 },
  { key: "cd", label: "CD", unit: "", better: "min", digits: 4 },
  { key: "cm", label: "Cm", unit: "", better: null, digits: 4 },
  { key: "staticMargin", label: "Static margin", unit: "L", better: null, digits: 3 },
  { key: "volume", label: "Volume", unit: "m³", better: "max", digits: 3 },
  { key: "sRef", label: "Sref (planform)", unit: "m²", better: null, digits: 3 },
  { key: "volEff", label: "η vol (V^⅔/S)", unit: "", better: "max", digits: 3 },
  { key: "mass", label: "Mass", unit: "kg", better: "min", digits: 0 },
  { key: "qStag", label: "q̇ stag", unit: "W/cm²", better: "min", digits: 1 },
  { key: "rangeKm", label: "Glide range", unit: "km", better: "max", digits: 0 },
];

export interface CompareParamRow {
  path: string;
  values: unknown[];
}

export interface CompareMetricRow {
  key: MetricKey;
  label: string;
  unit: string;
  digits: number;
  values: (number | null)[];
  /** Index of the best value (per `better`), or null. */
  best: number | null;
  /** Relative spread (max−min)/|mean|, for highlighting. */
  spread: number;
}

export interface CurvePoint {
  alpha: number;
  cl: number;
  cd: number;
  ld: number;
}

export interface CompareCurves {
  /** Target Mach (first variant with a database decides); each series reports the Mach it actually used. */
  mach: number;
  series: { entryId: string; label: string; mach: number; points: CurvePoint[]; stale: boolean }[];
  /** Entries without a stored aero database. */
  missing: string[];
  /** Entries whose stored database was generated for a different design (carried over after a param change). */
  stale: string[];
  /** Union of α breakpoints, sorted, with one row per α and a column per series (recharts-ready). */
  rows: Record<string, number | null>[];
}

export interface CompareData {
  entries: { id: string; label: string }[];
  /** Params (and aero-DB spec) that differ between ANY of the entries. Database results are not diffed here. */
  params: CompareParamRow[];
  metrics: CompareMetricRow[] | null;
  curves: CompareCurves | null;
}

function get(obj: unknown, path: string): unknown {
  let cur: unknown = obj;
  for (const k of path.split(".")) {
    if (!cur || typeof cur !== "object") return undefined;
    cur = (cur as Record<string, unknown>)[k];
  }
  return cur;
}

/** Selectable versions: live working state, each branch tip, then every snapshot (newest first). */
export function compareCandidates(doc: ProjectDoc, working?: ProjectState): CompareEntry[] {
  const out: CompareEntry[] = [];
  if (working) out.push({ id: "working", label: "Working copy", state: working });
  for (const b of doc.history.branches) {
    const tip = listSnapshots(doc, b)[0];
    if (tip) out.push({ id: `tip:${b}`, label: `${b} (tip)`, state: tip.state });
  }
  for (const s of listSnapshots(doc)) out.push({ id: s.id, label: `${s.message} · ${s.branch}`, state: s.state });
  return out;
}

/** Nearest-value pick on an axis. */
function nearest(axis: number[], v: number): number {
  let best = axis[0]!;
  for (const a of axis) if (Math.abs(a - v) < Math.abs(best - v)) best = a;
  return best;
}

/** CL/CD/(L/D) vs α at β≈0, δ≈0 and the axis Mach nearest `mach`. */
export function alphaCurve(db: AeroDatabase, mach: number): { mach: number; points: CurvePoint[] } {
  const ms = db.axes?.mach?.length ? db.axes.mach : [...new Set(db.points.map((p) => p.mach))];
  const m = nearest(ms, mach);
  const betas = [...new Set(db.points.filter((p) => p.mach === m).map((p) => p.beta_deg))];
  const b = betas.length ? nearest(betas, 0) : 0;
  const deltas = [...new Set(db.points.filter((p) => p.mach === m && p.beta_deg === b).map((p) => p.delta_deg))];
  const d = deltas.length ? nearest(deltas, 0) : 0;
  const byAlpha = new Map<number, CurvePoint>();
  for (const p of db.points) {
    if (p.mach !== m || p.beta_deg !== b || p.delta_deg !== d) continue;
    if (!byAlpha.has(p.alpha_deg)) byAlpha.set(p.alpha_deg, { alpha: p.alpha_deg, cl: p.cl, cd: p.cd, ld: p.ld ?? (p.cd ? p.cl / p.cd : 0) });
  }
  return { mach: m, points: [...byAlpha.values()].sort((x, y) => x.alpha - y.alpha) };
}

export function buildCompare(
  entries: CompareEntry[],
  opts: { evaluate?: (p: DesignParams) => VariantMetrics; mach?: number; allParams?: boolean } = {},
): CompareData {
  if (entries.length < 2 || entries.length > 4) throw new Error(`Pick 2 to 4 versions to compare (got ${entries.length})`);
  const ids = new Set(entries.map((e) => e.id));
  if (ids.size !== entries.length) throw new Error("The same version was picked twice");

  // param rows: union of paths that differ between entry 0 and any other entry, plus between any pair
  const paths = new Set<string>();
  for (let i = 0; i < entries.length; i++)
    for (let j = i + 1; j < entries.length; j++)
      for (const c of diffStates(entries[i]!.state, entries[j]!.state)) if (c.path !== "aeroDb.database") paths.add(c.path);
  if (opts.allParams) for (const k of Object.keys(entries[0]!.state.params)) paths.add(`params.${k}`);
  const params: CompareParamRow[] = [...paths].sort().map((path) => ({
    path,
    values: entries.map((e) => get({ params: e.state.params, aeroDb: e.state.aeroDb ?? null }, path)),
  }));

  let metrics: CompareMetricRow[] | null = null;
  if (opts.evaluate) {
    // identical designs evaluate once
    const cache = new Map<string, VariantMetrics>();
    const evals = entries.map((e) => {
      const k = canonicalJson(e.state.params);
      if (!cache.has(k)) cache.set(k, opts.evaluate!(e.state.params));
      return cache.get(k)!;
    });
    metrics = COMPARE_METRICS.map((m) => {
      const values = evals.map((ev) => (ev.error || !Number.isFinite(ev[m.key]) ? null : ev[m.key]));
      const finite = values.filter((v): v is number => v !== null);
      let best: number | null = null;
      if (m.better && finite.length > 1) {
        const target = m.better === "max" ? Math.max(...finite) : Math.min(...finite);
        if (Math.max(...finite) !== Math.min(...finite)) best = values.indexOf(target);
      }
      const mean = finite.length ? finite.reduce((a, b) => a + b, 0) / finite.length : 0;
      const spread = finite.length > 1 && Math.abs(mean) > 1e-12 ? (Math.max(...finite) - Math.min(...finite)) / Math.abs(mean) : 0;
      return { key: m.key, label: m.label, unit: m.unit, digits: m.digits, values, best, spread };
    });
  }

  let curves: CompareCurves | null = null;
  const withDb = entries.filter((e) => e.state.aeroDb?.database?.points?.length);
  if (withDb.length) {
    const first = withDb[0]!.state.aeroDb!.database!;
    const mach = opts.mach ?? first.slices?.design?.mach ?? first.meta?.designMach ?? first.axes.mach[0]!;
    const series = withDb.map((e) => ({
      entryId: e.id,
      label: e.label,
      ...alphaCurve(e.state.aeroDb!.database!, mach),
      stale: aeroDbStale(e.state.aeroDb, e.state.params),
    }));
    const alphas = [...new Set(series.flatMap((s) => s.points.map((p) => p.alpha)))].sort((a, b) => a - b);
    const rows = alphas.map((alpha) => {
      const r: Record<string, number | null> = { alpha };
      series.forEach((s, i) => {
        const p = s.points.find((q) => q.alpha === alpha);
        r[`cl${i}`] = p ? p.cl : null;
        r[`cd${i}`] = p ? p.cd : null;
        r[`ld${i}`] = p ? p.ld : null;
      });
      return r;
    });
    curves = {
      mach,
      series,
      missing: entries.filter((e) => !withDb.includes(e)).map((e) => e.label),
      stale: series.filter((s) => s.stale).map((s) => s.label),
      rows,
    };
  }
  return { entries: entries.map((e) => ({ id: e.id, label: e.label })), params, metrics, curves };
}
