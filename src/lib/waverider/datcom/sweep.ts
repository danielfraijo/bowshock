/**
 * Parameter sweeps: Mach / alpha / roll / fin deflection.
 * Returns a tidy table structure suitable for plotting and CSV export.
 */
import type { FlightSpec, RunResult, VehicleConfig } from "./types.ts";
import { runCases, buildModel, machState, evalPoint } from "./model.ts";
import { flowConditions } from "./flow.ts";
import { cloneConfig } from "./presets.ts";

export interface SweepRow {
  mach: number;
  alpha: number;
  beta: number;
  phi: number;
  delta: number;
  CN: number;
  CA: number;
  Cm: number;
  CY: number;
  Cn: number;
  Cl: number;
  CL: number;
  CD: number;
  LD: number;
  xcp: number;
  CNa: number;
  Cma: number;
  CYb: number;
  Cnb: number;
  Clb: number;
  Clp: number;
  Cmq: number;
  CNd: number;
  Cmd: number;
  Cld: number;
  staticMargin: number;
  confidence: string;
  method: string;
}

export interface SweepResult {
  rows: SweepRow[];
  warnings: string[];
  run?: RunResult;
}

function toRows(r: RunResult, delta = 0): SweepRow[] {
  const rows: SweepRow[] = [];
  for (const c of r.cases) {
    for (const p of c.points) {
      rows.push({
        mach: c.mach,
        alpha: p.alpha,
        beta: p.beta,
        phi: p.phi,
        delta,
        CN: p.CN,
        CA: p.CA,
        Cm: p.Cm,
        CY: p.CY,
        Cn: p.Cn,
        Cl: p.Cl,
        CL: p.CL,
        CD: p.CD,
        LD: p.LD,
        xcp: p.xcp,
        CNa: p.derivs.CNa,
        Cma: p.derivs.Cma,
        CYb: p.derivs.CYb,
        Cnb: p.derivs.Cnb,
        Clb: p.derivs.Clb,
        Clp: p.derivs.Clp,
        Cmq: p.derivs.Cmq,
        CNd: p.derivs.CNd,
        Cmd: p.derivs.Cmd,
        Cld: p.derivs.Cld,
        staticMargin: p.derivs.staticMargin,
        confidence: p.validity.confidence,
        method: p.validity.method,
      });
    }
  }
  return rows;
}

export function sweepMachAlpha(cfg: VehicleConfig, flight: FlightSpec): SweepResult {
  const run = runCases(cfg, flight);
  return { rows: toRows(run), warnings: run.warnings, run };
}

export function sweepPhi(
  cfg: VehicleConfig,
  base: FlightSpec,
  phis: number[],
): SweepResult {
  const rows: SweepRow[] = [];
  const warnings: string[] = [];
  for (const phi of phis) {
    const r = runCases(cfg, { ...base, phi, beta: 0 });
    rows.push(...toRows(r));
    warnings.push(...r.warnings);
  }
  return { rows, warnings };
}

/**
 * Sweep a single fin-set's collective deflection (pitch mixer: delta_i = d * sin(phiF_i)).
 * setIndex defaults to the first canard/tail/fin set.
 */
export function sweepDeflection(
  cfg: VehicleConfig,
  base: FlightSpec,
  deltas: number[],
  setIndex?: number,
): SweepResult {
  const rows: SweepRow[] = [];
  const warnings: string[] = [];
  const model = buildModel(cfg);
  const si =
    setIndex ??
    Math.max(
      0,
      cfg.sets.findIndex((s) => s.role === "canard" || s.role === "tail" || s.role === "fin" || s.role === "htail"),
    );
  if (si < 0 || si >= cfg.sets.length) return { rows, warnings: ["no fin set to deflect"] };
  for (const d of deltas) {
    const c2 = cloneConfig(cfg);
    const set = c2.sets[si];
    const phis = set.phiF ?? model.fins[si]?.panelPhi ?? [];
    set.deflection = phis.map((p) => d * Math.sin((p * Math.PI) / 180));
    const r = runCases(c2, base);
    rows.push(...toRows(r, d));
    warnings.push(...r.warnings);
  }
  return { rows, warnings };
}

/** Convenience: evaluate a single (M, alpha, beta, phi) point. */
export function singlePoint(cfg: VehicleConfig, mach: number, alpha: number, beta = 0, phi = 0, altKm = 0) {
  const model = buildModel(cfg);
  const flow = flowConditions({ machs: [mach], alphas: [alpha], beta, phi, altKm }, 0);
  const ms = machState(model, flow);
  return evalPoint(ms, alpha, beta, phi);
}

export function sweepToCSV(rows: SweepRow[]): string {
  if (!rows.length) return "";
  const keys = Object.keys(rows[0]) as (keyof SweepRow)[];
  return [keys.join(","), ...rows.map((r) => keys.map((k) => r[k]).join(","))].join("\n");
}
