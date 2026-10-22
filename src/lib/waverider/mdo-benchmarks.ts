/**
 * Analytic verification problems for the mdo.ts searches, posed in the unit cube (scaled onto each problem's box).
 * - sphere, Rosenbrock (n-D): known minimum 0.
 * - G06 (Floudas & Pardalos 1987): two nonlinear inequality constraints, f* = −6961.81388 at (14.095, 0.84296).
 * - ZDT1 / ZDT2 (Zitzler, Deb & Thiele 2000): 2-objective, analytic Pareto fronts.
 * - Ishigami (Ishigami & Homma 1990): analytic Sobol indices.
 */
import type { UnitProblem } from "./mdo-core";

const scale = (u: number[], lo: number[], hi: number[]) => u.map((v, i) => lo[i] + (hi[i] - lo[i]) * v);

export interface Benchmark extends UnitProblem {
  name: string;
  lower: number[];
  upper: number[];
  /** Physical point from the unit cube. */
  toX(u: number[]): number[];
  known?: { x?: number[]; f?: number };
}

function box(name: string, lower: number[], upper: number[], fn: (x: number[]) => { cost: number[]; violation: number }, known?: Benchmark["known"]): Benchmark {
  const toX = (u: number[]) => scale(u, lower, upper);
  return { name, dim: lower.length, lower, upper, toX, evaluate: (u) => fn(toX(u)), known };
}

export function sphere(n = 5, shift = 0.3): Benchmark {
  return box(`sphere-${n}`, new Array(n).fill(-5), new Array(n).fill(5), (x) => ({ cost: [x.reduce((s, v) => s + (v - shift) ** 2, 0)], violation: 0 }), {
    x: new Array(n).fill(shift),
    f: 0,
  });
}

export function rosenbrock(n = 2): Benchmark {
  return box(
    `rosenbrock-${n}`,
    new Array(n).fill(-2),
    new Array(n).fill(2),
    (x) => {
      let f = 0;
      for (let i = 0; i < n - 1; i++) f += 100 * (x[i + 1] - x[i] * x[i]) ** 2 + (1 - x[i]) ** 2;
      return { cost: [f], violation: 0 };
    },
    { x: new Array(n).fill(1), f: 0 },
  );
}

export const G06_OPT = -6961.81388;

/** G06: min (x1−10)³ + (x2−20)³ s.t. (x1−5)² + (x2−5)² ≥ 100, (x1−6)² + (x2−5)² ≤ 82.81, x1∈[13,100], x2∈[0,100]. */
export function g06(): Benchmark {
  return box(
    "g06",
    [13, 0],
    [100, 100],
    ([x1, x2]) => {
      const h1 = (x1 - 5) ** 2 + (x2 - 5) ** 2;
      const h2 = (x1 - 6) ** 2 + (x2 - 5) ** 2;
      return { cost: [(x1 - 10) ** 3 + (x2 - 20) ** 3], violation: Math.max(0, 100 - h1) + Math.max(0, h2 - 82.81) };
    },
    { x: [14.095, 0.84296], f: G06_OPT },
  );
}

/**
 * `shift` > 0 moves the optimum of x2..xn from the lower bound to the interior point `shift`
 * (g uses |x_i − shift| / max(shift, 1 − shift)); the Pareto front is unchanged. This guards
 * against operators that only look good because they pile genes onto the bounds.
 */
function zdt(which: 1 | 2, n: number, shift = 0): Benchmark {
  const tag = shift ? `-shift${shift}` : "";
  return box(`zdt${which}-${n}${tag}`, new Array(n).fill(0), new Array(n).fill(1), (x) => {
    let s = 0;
    const w = Math.max(shift, 1 - shift);
    for (let i = 1; i < n; i++) s += shift ? Math.abs(x[i] - shift) / w : x[i];
    const g = 1 + (9 * s) / (n - 1);
    const r = x[0] / g;
    return { cost: [x[0], g * (1 - (which === 1 ? Math.sqrt(r) : r * r))], violation: 0 };
  });
}
/** ZDT1: convex front f2 = 1 − √f1. */
export const zdt1 = (n = 30, shift = 0) => zdt(1, n, shift);
/** ZDT2: non-convex front f2 = 1 − f1². */
export const zdt2 = (n = 30, shift = 0) => zdt(2, n, shift);

export function zdtFront(which: "zdt1" | "zdt2", m = 1000): number[][] {
  return Array.from({ length: m }, (_, i) => {
    const f1 = i / (m - 1);
    return [f1, which === "zdt1" ? 1 - Math.sqrt(f1) : 1 - f1 * f1];
  });
}

/** Exact hypervolume of the true ZDT front w.r.t. (ref, ref), ref ≥ 1: ref − ∫f2 + (ref − 1)·ref. */
export function zdtTrueHypervolume(which: "zdt1" | "zdt2", ref = 1.1): number {
  return ref - (which === "zdt1" ? 1 / 3 : 2 / 3) + (ref - 1) * ref;
}

/** Ishigami y = sin x1 + a sin² x2 + b x3⁴ sin x1 on [−π, π]³, as a function of the unit cube. */
export function ishigami(a = 7, b = 0.1): (u: number[]) => number {
  return (u) => {
    const [x1, x2, x3] = u.map((v) => -Math.PI + 2 * Math.PI * v);
    return Math.sin(x1) + a * Math.sin(x2) ** 2 + b * x3 ** 4 * Math.sin(x1);
  };
}

/** Analytic first-order and total Sobol indices of Ishigami. */
export function ishigamiIndices(a = 7, b = 0.1) {
  const V1 = 0.5 * (1 + (b * Math.PI ** 4) / 5) ** 2;
  const V2 = (a * a) / 8;
  const V13 = (b * b * Math.PI ** 8 * 8) / 225;
  const V = V1 + V2 + V13;
  return { V, S1: [V1 / V, V2 / V, 0], ST: [(V1 + V13) / V, V2 / V, V13 / V] };
}
