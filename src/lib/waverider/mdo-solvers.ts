/**
 * Single-objective, gradient-free searches for mdo.ts (sibling of the NSGA-II in mdo.ts).
 *
 * - CMA-ES: (μ/μ_w, λ) with IPOP restarts (Hansen 2016 tutorial).
 * - DE: DE/rand/1/bin or DE/best/1/bin with F dither (Storn & Price 1997; Das et al. 2005).
 * - Nelder–Mead: adaptive coefficients (Gao & Han 2012), optional restarts.
 * - Pattern search: compass poll with a pattern move and step contraction (Torczon 1997).
 *
 * All work in the unit cube and rank with the evaluator comparator, which defaults to Deb's
 * constraint-domination (feasible first, then violation, then cost). Multiple costs are summed
 * (callers pre-weight them). Every method stops cleanly at `maxEvals` and returns the best point
 * seen. Box handling: CMA-ES/NM/PS clip trial points to [0,1]; DE uses midpoint bounce-back.
 */
import {
  type Rng,
  type SearchOptions,
  type SearchResult,
  type UnitPoint,
  type UnitProblem,
  SearchStop,
  UnitEvaluator,
  clamp01,
  gaussian,
  randInt,
} from "./mdo-core";

async function guarded<T>(
  ev: UnitEvaluator<T>,
  body: () => Promise<{ iterations: number; message: string; population?: UnitPoint<T>[] }>,
  name: string,
): Promise<SearchResult<T>> {
  let out: { iterations: number; message: string; population?: UnitPoint<T>[] };
  try {
    out = await body();
  } catch (err) {
    if (!(err instanceof SearchStop) || err.reason !== "budget") throw err;
    out = { iterations: -1, message: "evaluation budget exhausted" };
  }
  if (!ev.best) throw new Error(`${name}: no evaluations (budget 0?)`);
  return { best: ev.best, evaluations: ev.evaluations, iterations: out.iterations, message: `${name}: ${out.message}`, population: out.population };
}

const center = (n: number, x0?: number[]) => (x0 && x0.length === n ? x0.map(clamp01) : new Array<number>(n).fill(0.5));

/* ------------------------------------------------------------------ CMA-ES */

export interface CmaesOptions extends SearchOptions {
  /** Start in the unit cube (default the centre). */
  x0?: number[];
  /** Initial step in unit-cube units (default 0.3). */
  sigma0?: number;
  popSize?: number;
  maxIter?: number;
  tolX?: number;
  tolFun?: number;
  /** IPOP restarts with doubled λ from a random mean (default 0). */
  restarts?: number;
}

/** Cyclic Jacobi eigen-decomposition of a symmetric matrix (columns of `vectors` are eigenvectors). */
export function symEig(Ain: number[][]): { values: number[]; vectors: number[][] } {
  const n = Ain.length;
  const A = Ain.map((r) => [...r]);
  const V = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j): number => (i === j ? 1 : 0)));
  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += A[p][q] * A[p][q];
    if (off < 1e-30) break;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        if (Math.abs(A[p][q]) < 1e-300) continue;
        const theta = (A[q][q] - A[p][p]) / (2 * A[p][q]);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < n; k++) {
          const akp = A[k][p];
          const akq = A[k][q];
          A[k][p] = c * akp - s * akq;
          A[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < n; k++) {
          const apk = A[p][k];
          const aqk = A[q][k];
          A[p][k] = c * apk - s * aqk;
          A[q][k] = s * apk + c * aqk;
        }
        for (let k = 0; k < n; k++) {
          const vkp = V[k][p];
          const vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq;
          V[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  return { values: A.map((r, i) => r[i]), vectors: V };
}

export async function cmaes<T>(problem: UnitProblem<T>, rng: Rng, o: CmaesOptions = {}): Promise<SearchResult<T>> {
  const ev = new UnitEvaluator(problem, o);
  const n = problem.dim;
  return guarded(
    ev,
    async () => {
      const maxIter = o.maxIter ?? 1000;
      const tolX = o.tolX ?? 1e-11;
      const tolFun = o.tolFun ?? 1e-12;
      const fit = (p: UnitPoint<T>) => p.cost.reduce((s, v) => s + v, 0) + 1e3 * p.violation;
      let lambda = o.popSize ?? 4 + Math.floor(3 * Math.log(n));
      let mean = center(n, o.x0);
      let it = 0;
      let msg = "max iterations";
      for (let r = 0; r <= (o.restarts ?? 0); r++) {
        const mu = Math.floor(lambda / 2);
        const wRaw = Array.from({ length: mu }, (_, i) => Math.log(mu + 0.5) - Math.log(i + 1));
        const wSum = wRaw.reduce((s, v) => s + v, 0);
        const w = wRaw.map((v) => v / wSum);
        const mueff = 1 / w.reduce((s, v) => s + v * v, 0);
        const cc = (4 + mueff / n) / (n + 4 + (2 * mueff) / n);
        const cs = (mueff + 2) / (n + mueff + 5);
        const c1 = 2 / ((n + 1.3) ** 2 + mueff);
        const cmu = Math.min(1 - c1, (2 * (mueff - 2 + 1 / mueff)) / ((n + 2) ** 2 + mueff));
        const damps = 1 + 2 * Math.max(0, Math.sqrt((mueff - 1) / (n + 1)) - 1) + cs;
        const chiN = Math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n));
        let sigma = o.sigma0 ?? 0.3;
        let C = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j): number => (i === j ? 1 : 0)));
        const pc = new Array<number>(n).fill(0);
        const ps = new Array<number>(n).fill(0);
        const hist: number[] = [];
        msg = "max iterations";
        for (let gen = 0; it < maxIter; it++, gen++) {
          const eig = symEig(C);
          const B = eig.vectors;
          const D = eig.values.map((v) => Math.sqrt(Math.max(v, 1e-30)));
          const pop: UnitPoint<T>[] = [];
          for (let k = 0; k < lambda; k++) {
            const z = Array.from({ length: n }, () => gaussian(rng));
            const y = new Array<number>(n).fill(0);
            for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) y[i] += B[i][j] * D[j] * z[j];
            pop.push(await ev.eval(mean.map((m, i) => clamp01(m + sigma * y[i]))));
          }
          pop.sort(ev.compare);
          const old = mean;
          mean = new Array<number>(n).fill(0);
          for (let k = 0; k < mu; k++) for (let i = 0; i < n; i++) mean[i] += w[k] * pop[k].u[i];
          const dm = mean.map((m, i) => (m - old[i]) / sigma);
          const bt = new Array<number>(n).fill(0);
          for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) bt[j] += B[i][j] * dm[i];
          const invs = new Array<number>(n).fill(0);
          for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) invs[i] += (B[i][j] * bt[j]) / D[j];
          const csn = Math.sqrt(cs * (2 - cs) * mueff);
          for (let i = 0; i < n; i++) ps[i] = (1 - cs) * ps[i] + csn * invs[i];
          const psNorm = Math.sqrt(ps.reduce((s, v) => s + v * v, 0));
          const hsig = psNorm / Math.sqrt(1 - (1 - cs) ** (2 * (gen + 1))) / chiN < 1.4 + 2 / (n + 1) ? 1 : 0;
          const ccn = Math.sqrt(cc * (2 - cc) * mueff);
          for (let i = 0; i < n; i++) pc[i] = (1 - cc) * pc[i] + hsig * ccn * dm[i];
          const ys = pop.slice(0, mu).map((p) => p.u.map((v, i) => (v - old[i]) / sigma));
          const Cn = C.map((row) => [...row]);
          for (let i = 0; i < n; i++)
            for (let j = 0; j < n; j++) {
              let rankMu = 0;
              for (let k = 0; k < mu; k++) rankMu += w[k] * ys[k][i] * ys[k][j];
              Cn[i][j] = (1 - c1 - cmu) * C[i][j] + c1 * (pc[i] * pc[j] + (1 - hsig) * cc * (2 - cc) * C[i][j]) + cmu * rankMu;
            }
          for (let i = 0; i < n; i++) for (let j = 0; j < i; j++) Cn[i][j] = Cn[j][i] = 0.5 * (Cn[i][j] + Cn[j][i]);
          C = Cn;
          sigma = Math.min(1, sigma * Math.exp((cs / damps) * (psNorm / chiN - 1)));
          hist.push(fit(pop[0]));
          if (sigma * Math.max(...D) < tolX) {
            msg = "converged (tolX)";
            it++;
            break;
          }
          const win = 10 + Math.ceil((30 * n) / lambda);
          if (hist.length >= win) {
            const recent = hist.slice(-win);
            const spread = Math.max(...recent) - Math.min(...recent);
            if (spread < tolFun && Math.abs(fit(pop[pop.length - 1]) - fit(pop[0])) < tolFun) {
              msg = "converged (tolFun)";
              it++;
              break;
            }
          }
        }
        if (it >= maxIter) break;
        lambda *= 2;
        mean = Array.from({ length: n }, () => rng());
      }
      return { iterations: it, message: msg };
    },
    "CMA-ES",
  );
}

/* ------------------------------------------------------------------ Differential evolution */

export interface DeOptions extends SearchOptions {
  popSize?: number;
  generations?: number;
  F?: number;
  CR?: number;
  strategy?: "rand1bin" | "best1bin";
  /** Per-generation F dither, uniform in [F, 1] (default true). */
  dither?: boolean;
  /** Seed members (unit cube). */
  initial?: number[][];
}

export async function differentialEvolution<T>(problem: UnitProblem<T>, rng: Rng, o: DeOptions = {}): Promise<SearchResult<T>> {
  const ev = new UnitEvaluator(problem, o);
  const n = problem.dim;
  return guarded(
    ev,
    async () => {
      const N = Math.max(5, o.popSize ?? Math.max(20, 10 * n));
      const G = o.generations ?? 100;
      const F = o.F ?? 0.5;
      const CR = o.CR ?? 0.9;
      const U: number[][] = [];
      for (const u0 of o.initial ?? []) if (U.length < N) U.push(u0.map(clamp01));
      while (U.length < N) U.push(Array.from({ length: n }, () => rng()));
      const E: UnitPoint<T>[] = [];
      for (const u of U) E.push(await ev.eval(u));
      let g = 1;
      for (; g <= G; g++) {
        const Fg = o.dither === false ? F : F + (1 - F) * rng();
        let bi = 0;
        for (let i = 1; i < N; i++) if (ev.compare(E[i], E[bi]) < 0) bi = i;
        const trials: number[][] = [];
        for (let i = 0; i < N; i++) {
          const pick = (...not: number[]) => {
            let k: number;
            do k = randInt(rng, N);
            while (k === i || not.includes(k));
            return k;
          };
          let r1 = pick();
          const r2 = pick(r1);
          const r3 = pick(r1, r2);
          if (o.strategy === "best1bin") r1 = bi;
          const jr = randInt(rng, n);
          trials.push(
            U[i].map((ui, j) => {
              if (j !== jr && rng() >= CR) return ui;
              const v = U[r1][j] + Fg * (U[r2][j] - U[r3][j]);
              return v < 0 ? 0.5 * ui : v > 1 ? 0.5 * (ui + 1) : v;
            }),
          );
        }
        for (let i = 0; i < N; i++) {
          const e = await ev.eval(trials[i]);
          if (ev.compare(e, E[i]) <= 0) {
            U[i] = trials[i];
            E[i] = e;
          }
        }
      }
      return { iterations: g - 1, message: `${o.strategy ?? "rand1bin"}, ${G} generations × ${N}`, population: E };
    },
    "DE",
  );
}

/* ------------------------------------------------------------------ Nelder–Mead */

export interface NelderMeadOptions extends SearchOptions {
  x0?: number[];
  /** Initial simplex edge (unit cube, default 0.1). */
  step?: number;
  maxIter?: number;
  xtol?: number;
  ftol?: number;
  restarts?: number;
  adaptive?: boolean;
}

export async function nelderMead<T>(problem: UnitProblem<T>, o: NelderMeadOptions = {}): Promise<SearchResult<T>> {
  const ev = new UnitEvaluator(problem, o);
  const n = problem.dim;
  return guarded(
    ev,
    async () => {
      const adaptive = o.adaptive ?? true;
      const ge = adaptive ? 1 + 2 / n : 2;
      const c = adaptive ? 0.75 - 1 / (2 * n) : 0.5;
      const s = adaptive ? 1 - 1 / n : 0.5;
      const maxIter = o.maxIter ?? 200 * n;
      const xtol = o.xtol ?? 1e-8;
      const ftol = o.ftol ?? 1e-10;
      const step = o.step ?? 0.1;
      const lt = (p: UnitPoint<T>, q: UnitPoint<T>) => ev.compare(p, q) < 0;
      const fit = (p: UnitPoint<T>) => p.cost.reduce((a, v) => a + v, 0) + 1e3 * p.violation;
      let start = center(n, o.x0);
      let it = 0;
      let msg = "max iterations";
      for (let r = 0; r <= (o.restarts ?? 0); r++) {
        let simplex: number[][] = [start];
        for (let i = 0; i < n; i++) {
          const v = [...start];
          v[i] = v[i] + step <= 1 ? v[i] + step : v[i] - step;
          simplex.push(v.map(clamp01));
        }
        let E: UnitPoint<T>[] = [];
        for (const u of simplex) E.push(await ev.eval(u));
        msg = "max iterations";
        for (; it < maxIter; it++) {
          const ord = E.map((_, i) => i).sort((p, q) => ev.compare(E[p], E[q]));
          simplex = ord.map((i) => simplex[i]);
          E = ord.map((i) => E[i]);
          let diam = 0;
          for (let i = 1; i <= n; i++) for (let j = 0; j < n; j++) diam = Math.max(diam, Math.abs(simplex[i][j] - simplex[0][j]));
          const fs = Math.abs(fit(E[n]) - fit(E[0]));
          if ((diam < xtol && (fs < ftol || !Number.isFinite(fs))) || diam < 1e-13) {
            msg = "converged (xtol/ftol)";
            break;
          }
          const cen = new Array<number>(n).fill(0);
          for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) cen[j] += simplex[i][j] / n;
          const pt = (t: number) => cen.map((cj, j) => clamp01(cj + t * (simplex[n][j] - cj)));
          const xr = pt(-1);
          const er = await ev.eval(xr);
          if (lt(er, E[0])) {
            const xe = pt(-ge);
            const ee = await ev.eval(xe);
            [simplex[n], E[n]] = lt(ee, er) ? [xe, ee] : [xr, er];
          } else if (lt(er, E[n - 1])) {
            [simplex[n], E[n]] = [xr, er];
          } else {
            const outside = lt(er, E[n]);
            const xc = outside ? pt(-c) : pt(c);
            const ec = await ev.eval(xc);
            if (lt(ec, outside ? er : E[n])) {
              [simplex[n], E[n]] = [xc, ec];
            } else {
              for (let i = 1; i <= n; i++) {
                simplex[i] = simplex[i].map((v, j) => clamp01(simplex[0][j] + s * (v - simplex[0][j])));
                E[i] = await ev.eval(simplex[i]);
              }
            }
          }
        }
        let bi = 0;
        for (let i = 1; i <= n; i++) if (lt(E[i], E[bi])) bi = i;
        start = simplex[bi];
        if (it >= maxIter) break;
      }
      return { iterations: it, message: msg };
    },
    "Nelder–Mead",
  );
}

/* ------------------------------------------------------------------ Pattern search */

export interface PatternSearchOptions extends SearchOptions {
  x0?: number[];
  /** Initial poll step (unit cube, default 0.25). */
  step?: number;
  minStep?: number;
  contraction?: number;
  expansion?: number;
  maxIter?: number;
}

export async function patternSearch<T>(problem: UnitProblem<T>, o: PatternSearchOptions = {}): Promise<SearchResult<T>> {
  const ev = new UnitEvaluator(problem, o);
  const n = problem.dim;
  return guarded(
    ev,
    async () => {
      let step = o.step ?? 0.25;
      const minStep = o.minStep ?? 1e-7;
      const shrink = o.contraction ?? 0.5;
      const grow = o.expansion ?? 1;
      const maxIter = o.maxIter ?? 1000 * n;
      let u = center(n, o.x0);
      let best = await ev.eval(u);
      let it = 0;
      let msg = "max iterations";
      for (; it < maxIter; it++) {
        if (step < minStep) {
          msg = "converged (step < minStep)";
          break;
        }
        let improved = false;
        for (let i = 0; i < n && !improved; i++) {
          for (const sgn of [1, -1]) {
            const t = [...u];
            t[i] = clamp01(t[i] + sgn * step);
            if (t[i] === u[i]) continue;
            const e = await ev.eval(t);
            if (ev.compare(e, best) < 0) {
              const t2 = [...t];
              t2[i] = clamp01(t[i] + sgn * step);
              const e2 = t2[i] !== t[i] ? await ev.eval(t2) : e;
              [u, best] = ev.compare(e2, e) < 0 ? [t2, e2] : [t, e];
              improved = true;
              break;
            }
          }
        }
        step = improved ? step * grow : step * shrink;
      }
      return { iterations: it, message: msg };
    },
    "Pattern search",
  );
}
