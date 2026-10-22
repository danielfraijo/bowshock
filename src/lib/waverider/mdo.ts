/**
 * Browser-side multidisciplinary search.
 *
 * Each evaluation is one geometry build plus one panel solve (CBAERO if the
 * selected method is Euler — Euler is too slow to sit inside a population).
 * Range is the equilibrium-glide integral at the current speed. Static margin
 * is (x_cp − x_cg) / Lref, positive when the center of pressure is aft.
 * Heating is the panel stagnation flux.
 *
 * The default search is NSGA-II (Deb): non-dominated sorting, crowding distance,
 * constraint-domination. It is gradient-free and happy with a noisy panel
 * code. Objectives are minimized internally; a "max" goal is negated.
 * `OptSpec.algorithm` can instead pick a single-objective method from
 * mdo-solvers.ts (CMA-ES, DE, Nelder–Mead, pattern search) on the weighted sum
 * Σ weight·(±metric). DOE (LHS, full factorial) and global sensitivity
 * (Morris, Sobol/Saltelli) over design variables are below `sensitivity`.
 * Every method also runs on any `UnitProblem` (unit cube, cost vector plus
 * violation), which is how the analytic benchmarks in mdo.test.ts drive them.
 * See docs/optimize.md for assumptions and limits.
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
import {
  type Rng,
  type UnitEval,
  type UnitPoint,
  type UnitProblem,
  FEAS_TOL,
  SearchStop,
  UnitEvaluator,
  clamp01,
  constrainedDominates,
  mulberry32,
  paretoDominates,
} from "./mdo-core";
import { cmaes, differentialEvolution, nelderMead, patternSearch } from "./mdo-solvers";
import { factorialUnit, maximinLhs, morrisScreening, sobolIndices } from "./mdo-sampling";

export * from "./mdo-core";
export * from "./mdo-solvers";
export * from "./mdo-sampling";

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

export const OPT_ALGORITHMS = ["nsga2", "cmaes", "de", "nelder-mead", "pattern-search"] as const;
export type OptAlgorithm = (typeof OPT_ALGORITHMS)[number];

export interface OptSpec {
  variables: OptVariable[];
  objectives: OptObjective[];
  constraints: OptConstraint[];
  population: number;
  generations: number;
  seed: number;
  /**
   * Default "nsga2". The single-objective methods minimize Σ weight·(±metric)
   * with Deb feasibility rules and get a budget of population·(generations+1)
   * evaluations, the same as NSGA-II.
   */
  algorithm?: OptAlgorithm;
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
    const aero = panelAero(built.mesh, flight, atm, params.alphaDeg, params.betaDeg, sRef, params.length);
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

/** Constraint-domination (Deb). Costs are minimized. */
export function dominatesPoint(aCost: number[], aViol: number, bCost: number[], bViol: number): boolean {
  return constrainedDominates(aCost, aViol, bCost, bViol);
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

/* ------------------------------------------------------------------ NSGA-II (the only one) */

interface Ind<T> {
  p: UnitPoint<T>;
  rank: number;
  crowd: number;
}

function nonDominated<T>(pop: Ind<T>[]): void {
  const n = pop.length;
  const S: number[][] = Array.from({ length: n }, () => []);
  const domCount = new Array(n).fill(0);
  const fronts: number[][] = [[]];
  for (let p = 0; p < n; p++) {
    for (let q = 0; q < n; q++) {
      if (p === q) continue;
      if (constrainedDominates(pop[p].p.cost, pop[p].p.violation, pop[q].p.cost, pop[q].p.violation)) S[p].push(q);
      else if (constrainedDominates(pop[q].p.cost, pop[q].p.violation, pop[p].p.cost, pop[p].p.violation)) domCount[p]++;
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
  const m = pop[0]?.p.cost.length ?? 1;
  for (const front of fronts) {
    if (!front || front.length === 0) continue;
    for (let k = 0; k < m; k++) {
      const ids = [...front].sort((a, b) => pop[a].p.cost[k] - pop[b].p.cost[k]);
      pop[ids[0]].crowd = Infinity;
      pop[ids[ids.length - 1]].crowd = Infinity;
      const lo = pop[ids[0]].p.cost[k];
      const hi = pop[ids[ids.length - 1]].p.cost[k];
      const span = hi - lo || 1;
      for (let j = 1; j < ids.length - 1; j++) {
        pop[ids[j]].crowd += (pop[ids[j + 1]].p.cost[k] - pop[ids[j - 1]].p.cost[k]) / span;
      }
    }
  }
}

function better<T>(a: Ind<T>, b: Ind<T>): boolean {
  if (a.rank !== b.rank) return a.rank < b.rank;
  return a.crowd > b.crowd;
}

/**
 * Bounded simulated binary crossover (Deb & Agrawal 1995; Deb's NSGA-II reference code) on a pair of
 * unit-cube vectors: crossover with probability pc per pair, each gene with probability 1/2, spread
 * distribution truncated to the box, children swapped at random per gene.
 */
function sbx(rng: Rng, a: number[], b: number[], eta: number, pc: number): [number[], number[]] {
  const c1 = [...a];
  const c2 = [...b];
  if (rng() > pc) return [c1, c2];
  for (let i = 0; i < a.length; i++) {
    if (rng() > 0.5) continue;
    const y1 = Math.min(a[i], b[i]);
    const y2 = Math.max(a[i], b[i]);
    if (y2 - y1 < 1e-14) continue;
    const r = rng();
    const spread = (beta: number) => {
      const alpha = 2 - Math.pow(beta, -(eta + 1));
      return r <= 1 / alpha ? Math.pow(r * alpha, 1 / (eta + 1)) : Math.pow(1 / (2 - r * alpha), 1 / (eta + 1));
    };
    let v1 = 0.5 * (y1 + y2 - spread(1 + (2 * y1) / (y2 - y1)) * (y2 - y1));
    let v2 = 0.5 * (y1 + y2 + spread(1 + (2 * (1 - y2)) / (y2 - y1)) * (y2 - y1));
    v1 = clamp01(v1);
    v2 = clamp01(v2);
    if (rng() < 0.5) [v1, v2] = [v2, v1];
    c1[i] = v1;
    c2[i] = v2;
  }
  return [c1, c2];
}

/** Polynomial mutation (Deb & Goyal 1996), per gene with probability pm, clipped to [0,1]. */
function mutate(rng: Rng, g: number, eta: number, pm: number): number {
  if (rng() >= pm) return g;
  const u = rng();
  const delta = u < 0.5 ? (2 * u) ** (1 / (eta + 1)) - 1 : 1 - (2 * (1 - u)) ** (1 / (eta + 1));
  return clamp01(g + delta);
}

/** Feasible, non-dominated, de-duplicated subset (sorted by the first cost). */
export function paretoFilter<T>(points: UnitPoint<T>[]): UnitPoint<T>[] {
  const feas = points.filter((p) => p.violation <= FEAS_TOL && p.cost.every(Number.isFinite));
  feas.sort((a, b) => {
    for (let k = 0; k < a.cost.length; k++) if (a.cost[k] !== b.cost[k]) return a.cost[k] - b.cost[k];
    return a.id - b.id;
  });
  const out: UnitPoint<T>[] = [];
  const seen = new Set<string>();
  for (const p of feas) {
    const key = p.cost.join(",");
    if (seen.has(key) || out.some((o) => paretoDominates(o.cost, p.cost))) continue;
    seen.add(key);
    out.push(p);
  }
  return out;
}

export interface Nsga2Options<P = unknown> {
  popSize?: number;
  generations?: number;
  /** SBX probability per pair (default 0.9). */
  crossoverProb?: number;
  /** SBX distribution index (default 20). */
  etaC?: number;
  /** Mutation distribution index (default 18). */
  etaM?: number;
  /** Per-gene mutation probability (default 1/n). */
  mutationProb?: number;
  /** Seed members in the unit cube (the rest are uniform random). */
  initial?: number[][];
  /** Keep a non-dominated archive of every evaluation (default true). */
  archive?: boolean;
  signal?: AbortSignal;
  /** Called after each survival step with the new population. */
  onGeneration?: (generation: number, population: UnitPoint<P>[], evaluations: number) => void | Promise<void>;
}

export interface Nsga2Result<P = unknown> {
  /** Rank-0 members of the final population. */
  front: UnitPoint<P>[];
  /** Feasible non-dominated set over every evaluation (empty if archive = false or nothing was feasible). */
  archive: UnitPoint<P>[];
  population: UnitPoint<P>[];
  evaluations: number;
  generations: number;
}

/**
 * NSGA-II (Deb, Pratap, Agarwal & Meyarivan 2002) on a unit-cube problem: fast non-dominated sort with
 * constraint-domination, crowding distance, binary crowded tournament, bounded SBX + polynomial mutation,
 * (μ+λ) elitist survival. runOptimizer is this with the design problem.
 */
export async function nsga2<P>(problem: UnitProblem<P>, rng: Rng, o: Nsga2Options<P> = {}): Promise<Nsga2Result<P>> {
  const n = problem.dim;
  const N = Math.max(4, o.popSize ?? 100);
  const G = Math.max(0, o.generations ?? 100);
  const pc = o.crossoverProb ?? 0.9;
  const etaC = o.etaC ?? 20;
  const etaM = o.etaM ?? 18;
  const pm = o.mutationProb ?? 1 / Math.max(1, n);
  const ev = new UnitEvaluator(problem, { signal: o.signal });
  const all: UnitPoint<P>[] = [];
  const make = async (u: number[]): Promise<Ind<P>> => {
    const p = await ev.eval(u);
    if (o.archive !== false) all.push(p);
    return { p, rank: 0, crowd: 0 };
  };
  let pop: Ind<P>[] = [];
  for (const u0 of o.initial ?? []) if (pop.length < N) pop.push(await make(u0));
  while (pop.length < N) pop.push(await make(Array.from({ length: n }, () => rng())));
  for (let g = 0; g < G; g++) {
    if (o.signal?.aborted) throw new SearchStop("aborted");
    nonDominated(pop);
    const children: Ind<P>[] = [];
    const tour = () => {
      const i = Math.floor(rng() * pop.length);
      const j = Math.floor(rng() * pop.length);
      return better(pop[i], pop[j]) ? pop[i] : pop[j];
    };
    while (children.length < N) {
      const [a, b] = sbx(rng, tour().p.u, tour().p.u, etaC, pc);
      children.push(await make(a.map((x) => mutate(rng, x, etaM, pm))));
      if (children.length < N) children.push(await make(b.map((x) => mutate(rng, x, etaM, pm))));
    }
    const pool = pop.concat(children);
    nonDominated(pool);
    pool.sort((x, y) => x.rank - y.rank || y.crowd - x.crowd);
    pop = pool.slice(0, N);
    await o.onGeneration?.(g + 1, pop.map((m) => m.p), ev.evaluations);
  }
  nonDominated(pop);
  return {
    front: pop.filter((m) => m.rank === 0).map((m) => m.p),
    archive: o.archive === false ? [] : paretoFilter(all),
    population: pop.map((m) => m.p),
    evaluations: ev.evaluations,
    generations: G,
  };
}

/* ------------------------------------------------------------------ design problem */

export interface DesignProblemOptions {
  /** Single-objective view: cost = [Σ weight·(±metric)] instead of the per-objective vector. */
  scalar?: boolean;
  signal?: AbortSignal;
  /** Called after each evaluation (UI progress). */
  onEval?: (ev: DesignEval, evaluations: number) => void;
  /** Yield to the event loop after each evaluation (default true) so the UI and AbortSignal stay live. */
  yieldEach?: boolean;
}

/**
 * The vehicle study as a unit-cube problem: u → decode onto the variable bounds → evaluateDesign →
 * cost (minimized) + constraint violation. This is the hook for a CLI / OpenMDAO driver that wants to
 * run any mdo method headless (scripts/cuspis-eval.ts can call it; no CLI is added here).
 */
export function designProblem(
  base: DesignParams,
  variables: OptVariable[],
  objectives: OptObjective[],
  constraints: OptConstraint[],
  opts: DesignProblemOptions = {},
): UnitProblem<DesignEval> & { variables: OptVariable[]; decode(u: number[]): DesignParams; encode(p: DesignParams): number[] } {
  const vars = variables.filter((v) => NUMERIC_KEYS.includes(v.key) && v.max > v.min);
  if (!vars.length) throw new Error("Pick at least one design variable with a range.");
  const objs = objectives.length ? objectives : [{ metric: "ld" as const, goal: "max" as const, weight: 1 }];
  let count = 0;
  return {
    dim: vars.length,
    variables: vars,
    decode: (u) => decode(base, vars, u),
    encode: (p) => encode(p, vars),
    async evaluate(u: number[]): Promise<UnitEval<DesignEval>> {
      if (opts.signal?.aborted) throw new DOMException("Optimization cancelled", "AbortError");
      const ev = evaluateDesign(decode(base, vars, u));
      ev.violation = ev.violation + constraintViolation(ev.metrics, constraints);
      ev.feasible = ev.violation <= FEAS_TOL && !ev.error;
      count++;
      opts.onEval?.(ev, count);
      if (opts.yieldEach !== false) await new Promise((r) => setTimeout(r, 0));
      const cost = opts.scalar
        ? [objs.reduce((s, o) => s + Math.max(1e-6, o.weight) * (o.goal === "max" ? -1 : 1) * metricValue(ev.metrics, o.metric), 0)]
        : costVector(ev.metrics, objs);
      return { cost: cost.map((c) => (Number.isFinite(c) ? c : 1e12)), violation: ev.violation, data: ev };
    },
  };
}

export interface OptProgress {
  generation: number;
  generations: number;
  evaluations: number;
  label: string;
}

const ALGO_NOTES: Record<OptAlgorithm, string> = {
  nsga2: "NSGA-II, bounded SBX (pc 0.9/pair) and polynomial mutation (1/n per gene). Constraint-domination (Deb).",
  cmaes: "CMA-ES (IPOP restarts) on Σ weight·(±metric), Deb feasibility ranking. Local-to-regional; one design out.",
  de: "Differential evolution DE/rand/1/bin with F dither on Σ weight·(±metric), Deb feasibility selection.",
  "nelder-mead": "Nelder–Mead (adaptive) on Σ weight·(±metric), Deb ranking. Local only; starts at the baseline.",
  "pattern-search": "Compass pattern search on Σ weight·(±metric), Deb ranking. Local only; poor on active nonlinear constraints.",
};

export async function runOptimizer(
  base: DesignParams,
  spec: OptSpec,
  opts?: { signal?: AbortSignal; onProgress?: (p: OptProgress) => void },
): Promise<OptResult> {
  const objectives = spec.objectives.length ? spec.objectives : [{ metric: "ld" as const, goal: "max" as const, weight: 1 }];
  const popN = Math.max(4, Math.min(24, Math.round(spec.population || 8)));
  const gens = Math.max(1, Math.min(20, Math.round(spec.generations || 5)));
  const algorithm: OptAlgorithm = spec.algorithm && OPT_ALGORITHMS.includes(spec.algorithm) ? spec.algorithm : "nsga2";
  const rng = mulberry32(spec.seed || 1);
  const notes = [
    ALGO_NOTES[algorithm],
    "Each evaluation: buildVehicle + one panel solve. Euler is replaced by CBAERO inside the loop.",
    "Range is equilibrium glide from the current speed down to 45% of that speed. Not a closed-loop trajectory.",
    "Static margin is (x_cp − x_cg) / L. Positive means the center of pressure is aft of the CG.",
  ];
  let generation = 0;
  let evaluations = 0;
  const label = () => (generation === 0 ? `initial ${evaluations}/${popN}` : `gen ${generation} eval`);
  const problem = designProblem(base, spec.variables, objectives, spec.constraints, {
    scalar: algorithm !== "nsga2",
    signal: opts?.signal,
    onEval: (_ev, k) => {
      evaluations = k;
      opts?.onProgress?.({ generation: 0, generations: gens, evaluations, label: evaluations === 1 ? "baseline" : label() });
    },
  });
  const seedGenes = problem.encode(base);
  const history: OptResult["history"] = [];
  const record = (g: number, pts: UnitPoint<DesignEval>[]) => {
    const evs = pts.map((p) => p.data!).filter(Boolean);
    history.push({ generation: g, bestLd: evs.reduce((m, e) => Math.max(m, e.metrics.ld), 0), feasible: evs.filter((e) => e.feasible).length });
    opts?.onProgress?.({ generation: g, generations: gens, evaluations, label: `generation ${g}` });
  };
  const byLd = (a: DesignEval, b: DesignEval) => b.metrics.ld - a.metrics.ld;
  const rethrow = (err: unknown) => {
    if (err instanceof SearchStop && err.reason === "aborted") throw new DOMException("Optimization cancelled", "AbortError");
    throw err;
  };

  if (algorithm === "nsga2") {
    const r = await nsga2(problem, rng, {
      popSize: popN,
      generations: gens,
      initial: [seedGenes],
      signal: opts?.signal,
      onGeneration: (g, pop) => {
        generation = g;
        record(g, pop);
      },
    }).catch(rethrow);
    // The archive keeps every feasible non-dominated design seen, not just the survivors of the last generation.
    const set = r.archive.length ? paretoFilter([...r.archive, ...r.front]) : r.front;
    return { pareto: set.map((p) => p.data!).sort(byLd), history, evaluations: r.evaluations, notes };
  }

  const budget = popN * (gens + 1);
  // Window the single-objective run into "generations" of popN evaluations for the history chart.
  let window: UnitPoint<DesignEval>[] = [];
  const onEval = (p: UnitPoint) => {
    window.push(p as UnitPoint<DesignEval>);
    if (window.length === popN) {
      generation++;
      if (generation <= gens) record(generation, window);
      window = [];
    }
  };
  const common = { maxEvals: budget, signal: opts?.signal, onEval };
  const run =
    algorithm === "cmaes"
      ? cmaes(problem, rng, { ...common, x0: seedGenes, popSize: Math.max(4, Math.min(popN, 4 + Math.floor(3 * Math.log(problem.dim)) * 2)), restarts: 9 })
      : algorithm === "de"
        ? differentialEvolution(problem, rng, { ...common, popSize: Math.max(5, popN), generations: gens, initial: [seedGenes] })
        : algorithm === "nelder-mead"
          ? nelderMead(problem, { ...common, x0: seedGenes, restarts: 9 })
          : patternSearch(problem, { ...common, x0: seedGenes });
  const r = await run.catch(rethrow);
  notes.push(`${r.message}; ${r.evaluations} evaluations (budget ${budget}).`);
  return { pareto: r.best.data ? [r.best.data] : [], history, evaluations: r.evaluations, notes };
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

/* ------------------------------------------------------------------ DOE over design variables */

export interface DoeRow {
  /** Physical values of the swept variables. */
  values: Partial<Record<keyof DesignParams, number>>;
  metrics: MdoMetrics;
  feasible: boolean;
  error?: string;
}

interface DoeOptions {
  signal?: AbortSignal;
  onProgress?: (done: number, total: number) => void;
}

async function runDoe(base: DesignParams, vars: OptVariable[], points: number[][], opts?: DoeOptions): Promise<DoeRow[]> {
  const rows: DoeRow[] = [];
  for (let i = 0; i < points.length; i++) {
    if (opts?.signal?.aborted) throw new DOMException("Sweep cancelled", "AbortError");
    const params = decode(base, vars, points[i]);
    const ev = evaluateDesign(params);
    const values: DoeRow["values"] = {};
    for (const v of vars) values[v.key] = Number(params[v.key]);
    rows.push({ values, metrics: ev.metrics, feasible: !ev.error, error: ev.error });
    opts?.onProgress?.(i + 1, points.length);
    await new Promise((r) => setTimeout(r, 0));
  }
  return rows;
}

function rangedVars(variables: OptVariable[]): OptVariable[] {
  const vars = variables.filter((v) => NUMERIC_KEYS.includes(v.key) && v.max > v.min);
  if (!vars.length) throw new Error("Pick at least one design variable with a range.");
  return vars;
}

/**
 * Latin hypercube over the variable box (uniform marginals). `candidates` > 1 keeps the maximin-best
 * of that many seeded designs. Other parameters stay at `base`.
 */
export async function lhsSweep(
  base: DesignParams,
  variables: OptVariable[],
  samples: number,
  opts?: DoeOptions & { seed?: number; candidates?: number },
): Promise<DoeRow[]> {
  const vars = rangedVars(variables);
  const pts = maximinLhs(mulberry32(opts?.seed ?? 1), Math.max(1, Math.round(samples)), vars.length, opts?.candidates ?? 1);
  return runDoe(base, vars, pts, opts);
}

/**
 * Full-factorial grid: `levels` equally spaced values per variable (a number for all, or one per variable).
 * The first variable varies slowest. Cost is Π levels; capped at 1e6 points.
 */
export async function factorialSweep(
  base: DesignParams,
  variables: OptVariable[],
  levels: number | number[],
  opts?: DoeOptions,
): Promise<DoeRow[]> {
  const vars = rangedVars(variables);
  const lv = vars.map((_, i) => (Array.isArray(levels) ? (levels[i] ?? 3) : levels));
  return runDoe(base, vars, factorialUnit(lv), opts);
}

/* ------------------------------------------------------------------ global sensitivity over design variables */

function metricFn(base: DesignParams, vars: OptVariable[], metric: MdoMetric) {
  return (u: number[]) => {
    const ev = evaluateDesign(decode(base, vars, u));
    return ev.error ? NaN : metricValue(ev.metrics, metric);
  };
}

export interface MorrisRow {
  key: string;
  /** Mean elementary effect over the full variable range. */
  mu: number;
  /** Mean |effect| (screening rank). */
  muStar: number;
  /** Spread of the effects: nonlinearity or interaction. */
  sigma: number;
}

/** Morris screening of one metric over the variable box: r·(d+1) evaluations. Effects are per full range. */
export async function morrisSensitivity(
  base: DesignParams,
  variables: OptVariable[],
  metric: MdoMetric,
  opts?: { trajectories?: number; levels?: number; seed?: number; signal?: AbortSignal },
): Promise<{ rows: MorrisRow[]; evaluations: number }> {
  const vars = rangedVars(variables);
  const f = metricFn(base, vars, metric);
  const r = await morrisScreening(
    async (u) => {
      const y = f(u);
      await new Promise((res) => setTimeout(res, 0));
      return y;
    },
    vars.length,
    mulberry32(opts?.seed ?? 1),
    opts,
  );
  return { rows: vars.map((v, i) => ({ key: String(v.key), ...r.indices[i] })), evaluations: r.evaluations };
}

export interface SobolRow {
  key: string;
  /** First-order index: share of variance from this variable alone. */
  S1: number;
  /** Total index: including all its interactions. */
  ST: number;
  S1_ci?: [number, number];
  ST_ci?: [number, number];
}

/**
 * Sobol indices of one metric over the variable box (inputs uniform and independent): N·(d+2) evaluations.
 * Failed builds are dropped from the estimate (`used` < `samples` reports it).
 */
export async function sobolSensitivity(
  base: DesignParams,
  variables: OptVariable[],
  metric: MdoMetric,
  opts?: { samples?: number; bootstrap?: number; seed?: number; signal?: AbortSignal },
): Promise<{ rows: SobolRow[]; variance: number; samples: number; used: number; evaluations: number }> {
  const vars = rangedVars(variables);
  const f = metricFn(base, vars, metric);
  let k = 0;
  const r = await sobolIndices(
    async (u) => {
      const y = f(u);
      if (++k % 8 === 0) await new Promise((res) => setTimeout(res, 0));
      return y;
    },
    vars.length,
    mulberry32(opts?.seed ?? 1),
    { samples: opts?.samples ?? 64, bootstrap: opts?.bootstrap, signal: opts?.signal },
  );
  return {
    rows: vars.map((v, i) => ({ key: String(v.key), ...r.indices[i] })),
    variance: r.variance,
    samples: r.samples,
    used: r.used,
    evaluations: r.evaluations,
  };
}
