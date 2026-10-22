/**
 * Browser-side multidisciplinary search.
 *
 * Each evaluation is one geometry build plus one panel solve (CBAERO if the
 * selected method is Euler — Euler is too slow to sit inside a population).
 * Range is the equilibrium-glide integral at the current speed. Static margin
 * is (x_cp − x_cg) / Lref, positive when the center of pressure is aft.
 * Heating is the panel stagnation flux.
 *
 * The search is NSGA-II (Deb): non-dominated sorting, crowding distance,
 * constraint-domination. It is gradient-free and happy with a noisy panel
 * code. Objectives are minimized internally; a "max" goal is negated.
 *
 * OpenMDAO / an external script should not scrape the UI. Call
 * `evaluateDesign` from `scripts/cuspis-eval.ts` (Node) and parse the JSON.
 * The same function is the one the genetic algorithm calls.
 */
import { atmosphere } from "./atmosphere";
import { equilibriumGlideRange } from "./flight-science";
import { buildVehicle } from "./generate";
import { panelAero } from "./panel";
import type { AeroMethod, DesignParams } from "./types";

export const MDO_METRICS = ["ld", "rangeKm", "qStag", "mass", "staticMargin", "volume", "cd", "cm"] as const;
export type MdoMetric = (typeof MDO_METRICS)[number];

export interface MdoMetrics {
  ld: number;
  rangeKm: number;
  /** W/cm² */
  qStag: number;
  mass: number;
  /** Positive: CP aft of CG. */
  staticMargin: number;
  volume: number;
  cd: number;
  cm: number;
  cl: number;
  method: AeroMethod;
}

export interface OptVariable {
  key: keyof DesignParams;
  min: number;
  max: number;
}

export interface OptObjective {
  metric: MdoMetric;
  goal: "min" | "max";
  weight: number;
}

export interface OptConstraint {
  metric: MdoMetric;
  op: ">=" | "<=";
  value: number;
}

export interface OptSpec {
  variables: OptVariable[];
  objectives: OptObjective[];
  constraints: OptConstraint[];
  population: number;
  generations: number;
  seed: number;
}

export interface DesignEval {
  params: DesignParams;
  metrics: MdoMetrics;
  violation: number;
  feasible: boolean;
  error?: string;
}

export interface OptResult {
  pareto: DesignEval[];
  history: { generation: number; bestLd: number; feasible: number }[];
  evaluations: number;
  notes: string[];
}

const NUMERIC_KEYS: (keyof DesignParams)[] = [
  "mach",
  "length",
  "span",
  "height",
  "shockDeg",
  "coneDeg",
  "superN",
  "planformPower",
  "fins",
  "wedgeFrac",
  "captureFrac",
  "dihedralDeg",
  "camber",
  "elevonDeg",
  "finHeight",
  "cgFrac",
  "leRadius",
  "alphaDeg",
  "altKm",
];

export function numericDesignKeys(): (keyof DesignParams)[] {
  return NUMERIC_KEYS;
}

function methodOf(p: DesignParams): AeroMethod {
  return p.aeroMethod === "euler" ? "cbaero" : p.aeroMethod;
}

export function evaluateDesign(params: DesignParams): DesignEval {
  try {
    const built = buildVehicle(params);
    const method = methodOf(params);
    const flight = { ...params, aeroMethod: method };
    const M = params.lockFlight ? params.mach : params.flightMach;
    const atm = atmosphere(params.altKm, M, params.gamma);
    const sRef = Math.max(built.aero.planformArea, 1e-6);
    const aero = panelAero(built.mesh, flight, atm, params.alphaDeg, params.betaDeg, sRef, params.length, undefined, true);
    const rho = Math.max(20, params.rhoKgM3 || 160);
    const mass = params.massKg > 0 ? params.massKg : rho * Math.max(built.quality.volume, 0);
    const sm = (aero.xCp - aero.cg[0]) / Math.max(aero.lRef, 1e-6);
    const V = Math.max(200, atm.V);
    const rangeKm = equilibriumGlideRange(Math.max(aero.ld, 0.15), V, 0.45 * V) / 1000;
    return {
      params,
      metrics: {
        ld: aero.ld,
        rangeKm,
        qStag: aero.qStag,
        mass,
        staticMargin: sm,
        volume: built.quality.volume,
        cd: aero.cd,
        cm: aero.cm,
        cl: aero.cl,
        method,
      },
      violation: 0,
      feasible: true,
    };
  } catch (err) {
    return {
      params,
      metrics: { ld: 0, rangeKm: 0, qStag: 1e6, mass: 1e9, staticMargin: -1, volume: 0, cd: 5, cm: 0, cl: 0, method: "cbaero" },
      violation: 1e6,
      feasible: false,
      error: err instanceof Error ? err.message : String(err),
    };
  }
}

function metricValue(m: MdoMetrics, key: MdoMetric): number {
  return m[key];
}

export function constraintViolation(metrics: MdoMetrics, constraints: OptConstraint[]): number {
  let v = 0;
  for (const c of constraints) {
    const x = metricValue(metrics, c.metric);
    if (!Number.isFinite(x)) {
      v += 1;
      continue;
    }
    if (c.op === ">=") v += Math.max(0, c.value - x);
    else v += Math.max(0, x - c.value);
  }
  return v;
}

function costVector(metrics: MdoMetrics, objectives: OptObjective[]): number[] {
  return objectives.map((o) => {
    const x = metricValue(metrics, o.metric);
    const w = Math.max(1e-6, o.weight);
    const signed = o.goal === "max" ? -x : x;
    return signed / w;
  });
}

function dominates(a: number[], av: number, b: number[], bv: number): boolean {
  const aFeas = av <= 1e-8;
  const bFeas = bv <= 1e-8;
  if (aFeas && !bFeas) return true;
  if (!aFeas && bFeas) return false;
  if (!aFeas && !bFeas) return av < bv - 1e-12;
  let better = false;
  for (let i = 0; i < a.length; i++) {
    if (a[i] > b[i] + 1e-12) return false;
    if (a[i] < b[i] - 1e-12) better = true;
  }
  return better;
}

/** Constraint-domination (Deb). Costs are minimized. */
export function dominatesPoint(aCost: number[], aViol: number, bCost: number[], bViol: number): boolean {
  return dominates(aCost, aViol, bCost, bViol);
}

interface Ind {
  genes: number[];
  eval: DesignEval;
  cost: number[];
  rank: number;
  crowd: number;
}

function decode(base: DesignParams, vars: OptVariable[], genes: number[]): DesignParams {
  const next = { ...base };
  vars.forEach((v, i) => {
    const g = Math.max(0, Math.min(1, genes[i] ?? 0.5));
    const value = v.min + (v.max - v.min) * g;
    (next as unknown as Record<string, number>)[v.key] = value;
  });
  return next;
}

function encode(base: DesignParams, vars: OptVariable[]): number[] {
  return vars.map((v) => {
    const x = Number(base[v.key]);
    const span = v.max - v.min;
    if (!Number.isFinite(x) || span === 0) return 0.5;
    return Math.max(0, Math.min(1, (x - v.min) / span));
  });
}

function mulberry32(seed: number) {
  let a = seed >>> 0;
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function nonDominated(pop: Ind[]): void {
  const n = pop.length;
  const S: number[][] = Array.from({ length: n }, () => []);
  const domCount = new Array(n).fill(0);
  const fronts: number[][] = [[]];
  for (let p = 0; p < n; p++) {
    for (let q = 0; q < n; q++) {
      if (p === q) continue;
      if (dominates(pop[p].cost, pop[p].eval.violation, pop[q].cost, pop[q].eval.violation)) S[p].push(q);
      else if (dominates(pop[q].cost, pop[q].eval.violation, pop[p].cost, pop[p].eval.violation)) domCount[p]++;
    }
    if (domCount[p] === 0) {
      pop[p].rank = 0;
      fronts[0].push(p);
    }
  }
  let i = 0;
  while (fronts[i] && fronts[i].length) {
    const next: number[] = [];
    for (const p of fronts[i]) {
      for (const q of S[p]) {
        domCount[q]--;
        if (domCount[q] === 0) {
          pop[q].rank = i + 1;
          next.push(q);
        }
      }
    }
    i++;
    fronts[i] = next;
  }
  for (const ind of pop) ind.crowd = 0;
  const m = pop[0]?.cost.length ?? 1;
  for (const front of fronts) {
    if (!front || front.length === 0) continue;
    for (let k = 0; k < m; k++) {
      const ids = [...front].sort((a, b) => pop[a].cost[k] - pop[b].cost[k]);
      pop[ids[0]].crowd = Infinity;
      pop[ids[ids.length - 1]].crowd = Infinity;
      const lo = pop[ids[0]].cost[k];
      const hi = pop[ids[ids.length - 1]].cost[k];
      const span = hi - lo || 1;
      for (let j = 1; j < ids.length - 1; j++) {
        pop[ids[j]].crowd += (pop[ids[j + 1]].cost[k] - pop[ids[j - 1]].cost[k]) / span;
      }
    }
  }
}

function better(a: Ind, b: Ind): boolean {
  if (a.rank !== b.rank) return a.rank < b.rank;
  return a.crowd > b.crowd;
}

function sbx(rng: () => number, a: number, b: number, eta = 12): [number, number] {
  if (Math.abs(a - b) < 1e-12 || rng() > 0.9) return [a, b];
  const u = rng();
  const beta = u <= 0.5 ? (2 * u) ** (1 / (eta + 1)) : (1 / (2 * (1 - u))) ** (1 / (eta + 1));
  let c1 = 0.5 * ((a + b) - beta * Math.abs(b - a));
  let c2 = 0.5 * ((a + b) + beta * Math.abs(b - a));
  c1 = Math.max(0, Math.min(1, c1));
  c2 = Math.max(0, Math.min(1, c2));
  return [c1, c2];
}

function mutate(rng: () => number, g: number, eta = 18): number {
  if (rng() > 1 / 3) return g;
  const u = rng();
  const delta = u < 0.5 ? (2 * u) ** (1 / (eta + 1)) - 1 : 1 - (2 * (1 - u)) ** (1 / (eta + 1));
  return Math.max(0, Math.min(1, g + delta));
}

export interface OptProgress {
  generation: number;
  generations: number;
  evaluations: number;
  label: string;
}

export async function runOptimizer(
  base: DesignParams,
  spec: OptSpec,
  opts?: { signal?: AbortSignal; onProgress?: (p: OptProgress) => void },
): Promise<OptResult> {
  const vars = spec.variables.filter((v) => NUMERIC_KEYS.includes(v.key) && v.max > v.min);
  const objectives = spec.objectives.length ? spec.objectives : [{ metric: "ld" as const, goal: "max" as const, weight: 1 }];
  if (!vars.length) throw new Error("Pick at least one design variable with a range.");
  const popN = Math.max(4, Math.min(24, Math.round(spec.population || 8)));
  const gens = Math.max(1, Math.min(20, Math.round(spec.generations || 5)));
  const rng = mulberry32(spec.seed || 1);
  const notes = [
    "NSGA-II, real-coded SBX and polynomial mutation. Constraint-domination (Deb).",
    "Each evaluation: buildVehicle + one panel solve. Euler is replaced by CBAERO inside the loop.",
    "Range is equilibrium glide from the current speed down to 45% of that speed. Not a closed-loop trajectory.",
    "Static margin is (x_cp − x_cg) / L. Positive means the center of pressure is aft of the CG.",
  ];
  let evaluations = 0;
  const make = async (genes: number[], label: string): Promise<Ind> => {
    if (opts?.signal?.aborted) throw new DOMException("Optimization cancelled", "AbortError");
    const params = decode(base, vars, genes);
    const ev = evaluateDesign(params);
    ev.violation = ev.violation + constraintViolation(ev.metrics, spec.constraints);
    ev.feasible = ev.violation <= 1e-8 && !ev.error;
    evaluations++;
    opts?.onProgress?.({ generation: 0, generations: gens, evaluations, label });
    await new Promise((r) => setTimeout(r, 0));
    return { genes, eval: ev, cost: costVector(ev.metrics, objectives), rank: 0, crowd: 0 };
  };

  let pop: Ind[] = [];
  const seedGenes = encode(base, vars);
  pop.push(await make(seedGenes, "baseline"));
  while (pop.length < popN) {
    const genes = vars.map(() => rng());
    pop.push(await make(genes, `initial ${pop.length + 1}/${popN}`));
  }
  const history: OptResult["history"] = [];
  for (let g = 0; g < gens; g++) {
    if (opts?.signal?.aborted) throw new DOMException("Optimization cancelled", "AbortError");
    nonDominated(pop);
    const children: Ind[] = [];
    while (children.length < popN) {
      const tour = () => {
        const i = Math.floor(rng() * pop.length);
        const j = Math.floor(rng() * pop.length);
        return better(pop[i], pop[j]) ? pop[i] : pop[j];
      };
      const dad = tour();
      const mom = tour();
      const c1: number[] = [];
      const c2: number[] = [];
      for (let k = 0; k < vars.length; k++) {
        const [a, b] = sbx(rng, dad.genes[k], mom.genes[k]);
        c1.push(mutate(rng, a));
        c2.push(mutate(rng, b));
      }
      children.push(await make(c1, `gen ${g + 1} eval`));
      if (children.length < popN) children.push(await make(c2, `gen ${g + 1} eval`));
    }
    const pool = pop.concat(children);
    nonDominated(pool);
    pool.sort((a, b) => (a.rank - b.rank) || (b.crowd - a.crowd));
    pop = pool.slice(0, popN);
    const feas = pop.filter((p) => p.eval.feasible);
    const bestLd = pop.reduce((m, p) => Math.max(m, p.eval.metrics.ld), 0);
    history.push({ generation: g + 1, bestLd, feasible: feas.length });
    opts?.onProgress?.({ generation: g + 1, generations: gens, evaluations, label: `generation ${g + 1}` });
  }
  nonDominated(pop);
  const pareto = pop
    .filter((p) => p.rank === 0)
    .map((p) => p.eval)
    .sort((a, b) => b.metrics.ld - a.metrics.ld);
  return { pareto, history, evaluations, notes };
}

export interface SweepRow {
  value: number;
  metrics: MdoMetrics;
  error?: string;
}

export async function parameterSweep(
  base: DesignParams,
  key: keyof DesignParams,
  values: number[],
  opts?: { signal?: AbortSignal; onProgress?: (done: number, total: number) => void },
): Promise<SweepRow[]> {
  const rows: SweepRow[] = [];
  for (let i = 0; i < values.length; i++) {
    if (opts?.signal?.aborted) throw new DOMException("Sweep cancelled", "AbortError");
    const params = { ...base, [key]: values[i] } as DesignParams;
    const ev = evaluateDesign(params);
    rows.push({ value: values[i], metrics: ev.metrics, error: ev.error });
    opts?.onProgress?.(i + 1, values.length);
    await new Promise((r) => setTimeout(r, 0));
  }
  return rows;
}

export interface SensitivityRow {
  key: string;
  base: number;
  plus: number;
  minus: number;
  /** d(metric)/d(variable) from a central difference. */
  dMetric: number;
  /** (dJ/J) / (dx/x) */
  elasticity: number;
}

export async function sensitivity(
  base: DesignParams,
  keys: (keyof DesignParams)[],
  metric: MdoMetric,
  frac = 0.05,
  opts?: { signal?: AbortSignal },
): Promise<{ baseline: number; rows: SensitivityRow[] }> {
  const mid = evaluateDesign(base);
  const j0 = metricValue(mid.metrics, metric);
  const rows: SensitivityRow[] = [];
  for (const key of keys) {
    if (opts?.signal?.aborted) throw new DOMException("Sensitivity cancelled", "AbortError");
    const x = Number(base[key]);
    if (!Number.isFinite(x)) continue;
    const dx = Math.max(Math.abs(x) * frac, 1e-4);
    const up = evaluateDesign({ ...base, [key]: x + dx } as DesignParams);
    const dn = evaluateDesign({ ...base, [key]: x - dx } as DesignParams);
    const ju = metricValue(up.metrics, metric);
    const jd = metricValue(dn.metrics, metric);
    const dMetric = (ju - jd) / (2 * dx);
    const elasticity = Math.abs(j0) > 1e-8 && Math.abs(x) > 1e-8 ? (dMetric * x) / j0 : 0;
    rows.push({ key: String(key), base: x, plus: ju, minus: jd, dMetric, elasticity });
    await new Promise((r) => setTimeout(r, 0));
  }
  return { baseline: j0, rows };
}
