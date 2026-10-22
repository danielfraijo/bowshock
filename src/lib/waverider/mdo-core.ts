/**
 * Problem-agnostic plumbing shared by every search method in mdo.ts.
 *
 * All methods work in the unit hypercube u ∈ [0,1]^n (mdo.ts decodes u onto the
 * OptVariable bounds). A problem returns a cost vector (minimized) and a total
 * constraint violation ≥ 0 (0 = feasible). Ranking uses Deb's
 * constraint-domination everywhere: feasible beats infeasible, two infeasible
 * points compare by violation, two feasible points by Pareto dominance (or by
 * the scalar cost for single-objective methods).
 */

/** Uniform [0,1) generator. All stochastic methods draw only from one, so a run is reproducible per seed. */
export type Rng = () => number;

/** mulberry32: small, fast, platform-independent 32-bit PRNG (the one runOptimizer has always used). */
export function mulberry32(seed: number): Rng {
  let a = seed >>> 0;
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Standard normal via Box–Muller (one draw per call; the pair's second value is discarded). */
export function gaussian(rng: Rng): number {
  let u = 0;
  while (u <= 1e-300) u = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rng());
}

export function randInt(rng: Rng, n: number): number {
  return Math.min(n - 1, Math.floor(rng() * n));
}

/** Fisher–Yates permutation of 0..n−1. */
export function permutation(rng: Rng, n: number): number[] {
  const p = Array.from({ length: n }, (_, i) => i);
  for (let i = n - 1; i > 0; i--) {
    const j = randInt(rng, i + 1);
    [p[i], p[j]] = [p[j], p[i]];
  }
  return p;
}

export const clamp01 = (v: number) => (v < 0 ? 0 : v > 1 ? 1 : Number.isFinite(v) ? v : 0.5);

/** Feasibility tolerance on the total violation (same as runOptimizer). */
export const FEAS_TOL = 1e-8;

/** Pareto dominance on minimized cost vectors (1e-12 slack, as runOptimizer has always used). */
export function paretoDominates(a: number[], b: number[]): boolean {
  let better = false;
  for (let i = 0; i < a.length; i++) {
    if (a[i] > b[i] + 1e-12) return false;
    if (a[i] < b[i] - 1e-12) better = true;
  }
  return better;
}

/** Constraint-domination (Deb 2000). Costs are minimized; violation ≥ 0, feasible when ≤ FEAS_TOL. */
export function constrainedDominates(a: number[], av: number, b: number[], bv: number): boolean {
  const aFeas = av <= FEAS_TOL;
  const bFeas = bv <= FEAS_TOL;
  if (aFeas && !bFeas) return true;
  if (!aFeas && bFeas) return false;
  if (!aFeas && !bFeas) return av < bv - 1e-12;
  return paretoDominates(a, b);
}

/** Result of one evaluation. `data` carries problem-specific payload (e.g. the DesignEval). */
export interface UnitEval<T = unknown> {
  cost: number[];
  violation: number;
  data?: T;
}

export interface UnitPoint<T = unknown> extends UnitEval<T> {
  u: number[];
  /** 0-based evaluation index. */
  id: number;
}

export interface UnitProblem<T = unknown> {
  dim: number;
  /** Evaluate a point of the unit cube. May be async (the design loop yields to the UI). */
  evaluate(u: number[]): UnitEval<T> | Promise<UnitEval<T>>;
}

/** How single-objective methods rank points. "domination" = Deb feasibility rules; "penalty" = f + ρ·violation. */
export type ConstraintHandling = "domination" | "penalty";

/** Scalar cost used by single-objective methods: Σ cost (callers pre-weight the vector). */
export function scalarCost(cost: number[]): number {
  let s = 0;
  for (const c of cost) s += c;
  return Number.isFinite(s) ? s : Infinity;
}

/** < 0 when a is better. */
export function makeComparator(handling: ConstraintHandling = "domination", rho = 1e3) {
  return (a: UnitEval, b: UnitEval): number => {
    const fa = scalarCost(a.cost);
    const fb = scalarCost(b.cost);
    if (handling === "penalty") {
      const pa = fa + rho * a.violation;
      const pb = fb + rho * b.violation;
      return pa === pb ? 0 : pa < pb || !Number.isFinite(pb) ? -1 : 1;
    }
    const aF = a.violation <= FEAS_TOL;
    const bF = b.violation <= FEAS_TOL;
    if (aF !== bF) return aF ? -1 : 1;
    if (!aF) return a.violation - b.violation;
    if (fa === fb) return 0;
    if (!Number.isFinite(fb)) return -1;
    if (!Number.isFinite(fa)) return 1;
    return fa - fb;
  };
}

export class SearchStop extends Error {
  constructor(public reason: "aborted" | "budget") {
    super(reason === "aborted" ? "Optimization cancelled" : "evaluation budget exhausted");
  }
}

export interface SearchOptions {
  /** Hard cap on problem evaluations. */
  maxEvals?: number;
  signal?: AbortSignal;
  handling?: ConstraintHandling;
  /** Penalty factor ρ when handling = "penalty" (default 1e3). */
  penalty?: number;
  /** Called after every evaluation. */
  onEval?: (p: UnitPoint, evaluations: number) => void;
}

/**
 * Budget/abort/best-tracking wrapper all methods evaluate through. Inputs are clamped to the cube.
 * Throws SearchStop("budget") at the cap; the methods catch it and return the best point so far.
 */
export class UnitEvaluator<T = unknown> {
  evaluations = 0;
  best: UnitPoint<T> | null = null;
  readonly compare: (a: UnitEval, b: UnitEval) => number;
  private readonly maxEvals: number;
  constructor(
    readonly problem: UnitProblem<T>,
    private readonly opts: SearchOptions = {},
  ) {
    this.maxEvals = opts.maxEvals ?? Infinity;
    this.compare = makeComparator(opts.handling, opts.penalty);
  }
  get remaining() {
    return this.maxEvals - this.evaluations;
  }
  async eval(uIn: number[]): Promise<UnitPoint<T>> {
    if (this.opts.signal?.aborted) throw new SearchStop("aborted");
    if (this.evaluations >= this.maxEvals) throw new SearchStop("budget");
    const u = uIn.map(clamp01);
    const r = await this.problem.evaluate(u);
    const violation = Number.isFinite(r.violation) ? Math.max(0, r.violation) : 1e6;
    const p: UnitPoint<T> = { u, cost: r.cost, violation, data: r.data, id: this.evaluations++ };
    if (!this.best || this.compare(p, this.best) < 0) this.best = p;
    this.opts.onEval?.(p as UnitPoint, this.evaluations);
    return p;
  }
}

export interface SearchResult<T = unknown> {
  best: UnitPoint<T>;
  evaluations: number;
  iterations: number;
  message: string;
  /** Final population for population methods. */
  population?: UnitPoint<T>[];
}

/* ------------------------------------------------------------------ Pareto metrics */

/** Exact hypervolume dominated by `points` w.r.t. `ref` (minimization). 2-D sweep; n-D by slicing (small fronts). */
export function hypervolume(points: number[][], ref: number[]): number {
  const pts = points.filter((p) => p.every((v, i) => v < ref[i]));
  if (!pts.length) return 0;
  const m = ref.length;
  if (m === 1) return ref[0] - Math.min(...pts.map((p) => p[0]));
  if (m === 2) {
    const s = [...pts].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    let hv = 0;
    let yPrev = ref[1];
    for (const p of s) {
      if (p[1] < yPrev) {
        hv += (ref[0] - p[0]) * (yPrev - p[1]);
        yPrev = p[1];
      }
    }
    return hv;
  }
  const s = [...pts].sort((a, b) => a[m - 1] - b[m - 1]);
  let hv = 0;
  for (let i = 0; i < s.length; i++) {
    const zNext = i + 1 < s.length ? s[i + 1][m - 1] : ref[m - 1];
    const depth = zNext - s[i][m - 1];
    if (depth <= 0) continue;
    hv += depth * hypervolume(s.slice(0, i + 1).map((p) => p.slice(0, m - 1)), ref.slice(0, m - 1));
  }
  return hv;
}

/** Mean Euclidean distance from each point to its nearest reference point (GD). Swap the arguments for IGD. */
export function generationalDistance(points: number[][], reference: number[][]): number {
  if (!points.length || !reference.length) return Infinity;
  let s = 0;
  for (const p of points) {
    let best = Infinity;
    for (const r of reference) {
      let d = 0;
      for (let i = 0; i < p.length; i++) d += (p[i] - r[i]) ** 2;
      if (d < best) best = d;
    }
    s += Math.sqrt(best);
  }
  return s / points.length;
}
