/**
 * Design-of-experiments and global sensitivity for mdo.ts, in the unit cube (each input uniform over its bounds).
 *
 * - Full-factorial grids and Latin hypercube samples (optionally maximin-improved over seeded candidates).
 * - Morris elementary effects: μ, μ* (Campolongo et al. 2007) and σ, r trajectories on a p-level grid, r·(d+1) runs.
 * - Sobol indices from Saltelli's A/B/AB_i scheme, N·(d+2) runs: first order by Saltelli (2010), total by
 *   Jansen (1999), with seeded bootstrap 95 % intervals.
 * The response function may be async; points that return a non-finite value are dropped from the estimate.
 */
import { type Rng, permutation, randInt } from "./mdo-core";

export type ScalarFn = (u: number[]) => number | Promise<number>;

/** n equally spaced levels in [0,1] (n = 1 → the centre). */
export function unitLevels(n: number): number[] {
  const m = Math.max(1, Math.round(n));
  return m === 1 ? [0.5] : Array.from({ length: m }, (_, i) => i / (m - 1));
}

/** Full-factorial grid in the unit cube. The first dimension varies slowest. Capped at 1e6 points. */
export function factorialUnit(levels: number[]): number[][] {
  const lv = levels.map(unitLevels);
  const total = lv.reduce((p, l) => p * l.length, 1);
  if (total > 1_000_000) throw new Error(`factorial grid of ${total} points exceeds the 1e6 limit`);
  const out: number[][] = [];
  for (let k = 0; k < total; k++) {
    const u = new Array<number>(lv.length);
    let r = k;
    for (let d = lv.length - 1; d >= 0; d--) {
      u[d] = lv[d][r % lv[d].length];
      r = Math.floor(r / lv[d].length);
    }
    out.push(u);
  }
  return out;
}

/** Latin hypercube: n samples × d dims, one per stratum per dimension, random position inside the stratum. */
export function lhsUnit(rng: Rng, n: number, d: number): number[][] {
  const out = Array.from({ length: n }, () => new Array<number>(d).fill(0));
  for (let j = 0; j < d; j++) {
    const perm = permutation(rng, n);
    for (let i = 0; i < n; i++) out[i][j] = (perm[i] + rng()) / n;
  }
  return out;
}

export function minPairDistance(pts: number[][]): number {
  let m = Infinity;
  for (let i = 0; i < pts.length; i++)
    for (let j = i + 1; j < pts.length; j++) {
      let s = 0;
      for (let k = 0; k < pts[i].length; k++) s += (pts[i][k] - pts[j][k]) ** 2;
      if (s < m) m = s;
    }
  return Math.sqrt(m);
}

/** LHS keeping the maximin-best of `candidates` seeded designs (candidates = 1: plain LHS). */
export function maximinLhs(rng: Rng, n: number, d: number, candidates = 1): number[][] {
  let best = lhsUnit(rng, n, d);
  let bestD = candidates > 1 ? minPairDistance(best) : 0;
  for (let c = 1; c < candidates; c++) {
    const t = lhsUnit(rng, n, d);
    const dist = minPairDistance(t);
    if (dist > bestD) {
      best = t;
      bestD = dist;
    }
  }
  return best;
}

/* ------------------------------------------------------------------ Morris */

export interface MorrisOptions {
  trajectories?: number;
  /** Grid levels p (default 4; Δ = p / (2(p − 1))). */
  levels?: number;
  signal?: AbortSignal;
}

export interface MorrisIndex {
  /** Mean elementary effect (per unit-cube Δ, i.e. per full variable range). */
  mu: number;
  /** Mean |effect|: the screening measure. */
  muStar: number;
  /** Std. dev. of the effects: nonlinearity / interaction. */
  sigma: number;
}

export async function morrisScreening(f: ScalarFn, dim: number, rng: Rng, o: MorrisOptions = {}) {
  const r = o.trajectories ?? 10;
  const p = Math.max(2, o.levels ?? 4);
  const delta = p / (2 * (p - 1));
  const effects: number[][] = Array.from({ length: dim }, () => []);
  let evaluations = 0;
  const call = async (u: number[]) => {
    if (o.signal?.aborted) throw new DOMException("Sensitivity cancelled", "AbortError");
    evaluations++;
    return f(u);
  };
  for (let t = 0; t < r; t++) {
    // base point on the lower half of the grid so +Δ stays inside
    const u = Array.from({ length: dim }, () => randInt(rng, Math.ceil(p / 2)) / (p - 1));
    let prev = await call([...u]);
    for (const i of permutation(rng, dim)) {
      const step = u[i] + delta <= 1 + 1e-12 ? delta : -delta;
      u[i] += step;
      const cur = await call([...u]);
      const ee = (cur - prev) / step;
      if (Number.isFinite(ee)) effects[i].push(ee);
      prev = cur;
    }
  }
  const indices: MorrisIndex[] = effects.map((e) => {
    if (!e.length) return { mu: NaN, muStar: NaN, sigma: NaN };
    const mu = e.reduce((a, b) => a + b, 0) / e.length;
    const muStar = e.reduce((a, b) => a + Math.abs(b), 0) / e.length;
    const sigma = e.length > 1 ? Math.sqrt(e.reduce((a, b) => a + (b - mu) ** 2, 0) / (e.length - 1)) : 0;
    return { mu, muStar, sigma };
  });
  return { trajectories: r, levels: p, delta, indices, evaluations };
}

/* ------------------------------------------------------------------ Sobol / Saltelli */

export interface SobolOptions {
  /** Base sample size N (cost N·(d+2)), default 1024. */
  samples?: number;
  /** Bootstrap resamples for 95 % intervals (0 = none, default 100). */
  bootstrap?: number;
  sampling?: "random" | "lhs";
  signal?: AbortSignal;
}

export interface SobolIndex {
  S1: number;
  ST: number;
  S1_ci?: [number, number];
  ST_ci?: [number, number];
}

function saltelliEstimate(fA: number[], fB: number[], fAB: number[][], rows: number[]) {
  const n = rows.length;
  let mean = 0;
  for (const r of rows) mean += fA[r] + fB[r];
  mean /= 2 * n;
  let V = 0;
  for (const r of rows) V += (fA[r] - mean) ** 2 + (fB[r] - mean) ** 2;
  V /= 2 * n - 1;
  const S1: number[] = [];
  const ST: number[] = [];
  for (const col of fAB) {
    let s1 = 0;
    let st = 0;
    for (const r of rows) {
      s1 += fB[r] * (col[r] - fA[r]);
      st += (fA[r] - col[r]) ** 2;
    }
    S1.push(V > 0 ? s1 / n / V : NaN);
    ST.push(V > 0 ? st / (2 * n) / V : NaN);
  }
  return { V, S1, ST };
}

export async function sobolIndices(f: ScalarFn, dim: number, rng: Rng, o: SobolOptions = {}) {
  const N = Math.max(2, o.samples ?? 1024);
  let evaluations = 0;
  const call = async (u: number[]) => {
    if (o.signal?.aborted) throw new DOMException("Sensitivity cancelled", "AbortError");
    evaluations++;
    return f(u);
  };
  const gen = () => (o.sampling === "random" ? Array.from({ length: N }, () => Array.from({ length: dim }, () => rng())) : lhsUnit(rng, N, dim));
  const A = gen();
  const B = gen();
  const fA: number[] = [];
  const fB: number[] = [];
  for (const u of A) fA.push(await call(u));
  for (const u of B) fB.push(await call(u));
  const fAB: number[][] = [];
  for (let j = 0; j < dim; j++) {
    const col: number[] = [];
    for (let i = 0; i < N; i++) col.push(await call(A[i].map((v, k) => (k === j ? B[i][j] : v))));
    fAB.push(col);
  }
  const rows: number[] = [];
  for (let i = 0; i < N; i++) if (Number.isFinite(fA[i]) && Number.isFinite(fB[i]) && fAB.every((c) => Number.isFinite(c[i]))) rows.push(i);
  const est = saltelliEstimate(fA, fB, fAB, rows);
  const nBoot = o.bootstrap ?? 100;
  const boots: { S1: number[]; ST: number[] }[] = [];
  for (let b = 0; b < nBoot && rows.length > 1; b++) boots.push(saltelliEstimate(fA, fB, fAB, rows.map(() => rows[randInt(rng, rows.length)])));
  const ci = (vals: number[]): [number, number] => {
    const s = vals.filter(Number.isFinite).sort((a, b) => a - b);
    return s.length ? [s[Math.floor(0.025 * (s.length - 1))], s[Math.ceil(0.975 * (s.length - 1))]] : [NaN, NaN];
  };
  const indices: SobolIndex[] = est.S1.map((S1, j) => ({
    S1,
    ST: est.ST[j],
    ...(boots.length ? { S1_ci: ci(boots.map((b) => b.S1[j])), ST_ci: ci(boots.map((b) => b.ST[j])) } : {}),
  }));
  return { samples: N, used: rows.length, variance: est.V, indices, evaluations };
}
