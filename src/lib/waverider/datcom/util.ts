/** Small numeric helpers for the DATCOM-style estimator (pure TS, no DOM, no path aliases). */
export const DEG = Math.PI / 180;
export const RAD = 180 / Math.PI;

export function clamp(v: number, a: number, b: number): number {
  return v < a ? a : v > b ? b : v;
}
export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}
/** C1 smoothstep from 0 (x<=e0) to 1 (x>=e1). */
export function smoothstep(e0: number, e1: number, x: number): number {
  if (e1 === e0) return x < e0 ? 0 : 1;
  const t = clamp((x - e0) / (e1 - e0), 0, 1);
  return t * t * (3 - 2 * t);
}
/** Piecewise-linear interpolation, clamped at the ends. xs ascending. */
export function interp1(xs: readonly number[], ys: readonly number[], x: number): number {
  const n = xs.length;
  if (n === 0) return 0;
  if (x <= xs[0]) return ys[0];
  if (x >= xs[n - 1]) return ys[n - 1];
  let i = 1;
  while (i < n - 1 && xs[i] < x) i++;
  const t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
  return ys[i - 1] + t * (ys[i] - ys[i - 1]);
}
/** Composite Simpson integration of f over [a,b] with n (even) panels. */
export function simpson(f: (x: number) => number, a: number, b: number, n = 64): number {
  if (n % 2) n++;
  const h = (b - a) / n;
  let s = f(a) + f(b);
  for (let i = 1; i < n; i++) s += f(a + i * h) * (i % 2 ? 4 : 2);
  return (s * h) / 3;
}
/**
 * Complete elliptic integral of the 2nd kind E(k) (modulus k), via the
 * arithmetic-geometric mean (Abramowitz & Stegun 17.6).
 */
export function ellipticE(k: number): number {
  k = clamp(k, 0, 1);
  if (k >= 1) return 1;
  let a = 1;
  let b = Math.sqrt(1 - k * k);
  let c = k;
  let sum = 0.5 * c * c;
  let pow = 0.5;
  for (let i = 0; i < 40 && Math.abs(c) > 1e-15; i++) {
    const an = 0.5 * (a + b);
    c = 0.5 * (a - b);
    b = Math.sqrt(a * b);
    a = an;
    pow *= 2;
    sum += pow * c * c;
  }
  const K = Math.PI / (2 * a);
  return K * (1 - sum);
}
export function fin(x: number): boolean {
  return Number.isFinite(x);
}
export function sign(x: number): number {
  return x < 0 ? -1 : 1;
}
/** Round to n significant digits for stable text output. */
export function sig(x: number, n = 5): number {
  if (!Number.isFinite(x) || x === 0) return x;
  return Number(x.toPrecision(n));
}
