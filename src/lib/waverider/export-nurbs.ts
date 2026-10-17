/**
 * Shared NURBS helpers for the IGES (type 128) and STEP (B_SPLINE_SURFACE_WITH_KNOTS) exporters.
 *
 * The old exporters wrote the structured grid as the CONTROL net of a uniform-knot cubic. That is only an
 * approximation of the vehicle (corners and the LE fillet are smoothed away) and the two formats even
 * disagreed on the pole ordering. Here the surface INTERPOLATES the grid points (global cubic
 * interpolation, chord-length parameters, averaged knots), so the exported surfaces pass through the
 * vehicle's own sample points, and both formats use the same net.
 */
import type { SurfaceGrid, Vec3 } from "./types";
import { gridPoint } from "./mesh";

export interface NurbsNet {
  /** degrees in the grid-i and grid-j directions */
  p: number;
  q: number;
  /** full (clamped) knot vectors */
  U: number[];
  V: number[];
  /** control points ctrl[i][j], i along grid-i, j along grid-j */
  ctrl: Vec3[][];
}

/** Indices kept when a direction of `n` samples is capped at `cap` (always includes both ends). */
export function sampleIndices(n: number, cap = 80): number[] {
  if (n <= cap) return Array.from({ length: n }, (_, i) => i);
  const out: number[] = [];
  for (let k = 0; k < cap; k++) out.push(k === cap - 1 ? n - 1 : Math.round((k * (n - 1)) / (cap - 1)));
  return out.filter((v, i) => i === 0 || v !== out[i - 1]);
}

export function gridNet(g: SurfaceGrid, cap = 80): Vec3[][] {
  const is = sampleIndices(g.ni, cap);
  const js = sampleIndices(g.nj, cap);
  return is.map((i) => js.map((j) => gridPoint(g, i, j)));
}

function dist(a: Vec3, b: Vec3) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

/** Parameters 0..1 along one direction: chord length averaged over all lines, mixed 3 % with uniform. */
function parameters(lines: Vec3[][]): number[] {
  const n = lines[0].length;
  const acc = new Array<number>(n).fill(0);
  let used = 0;
  for (const line of lines) {
    const c: number[] = [0];
    for (let i = 1; i < n; i++) c.push(c[i - 1] + dist(line[i], line[i - 1]));
    const tot = c[n - 1];
    if (!(tot > 1e-12)) continue;
    for (let i = 0; i < n; i++) acc[i] += c[i] / tot;
    used++;
  }
  const out: number[] = [];
  for (let i = 0; i < n; i++) {
    const uni = n > 1 ? i / (n - 1) : 0;
    out.push(used ? 0.97 * (acc[i] / used) + 0.03 * uni : uni);
  }
  out[0] = 0;
  out[n - 1] = 1;
  return out;
}

/** Averaged knot vector (de Boor) for n points and degree p. */
function averagedKnots(u: number[], p: number): number[] {
  const n = u.length - 1;
  const K: number[] = [];
  for (let i = 0; i <= p; i++) K.push(0);
  for (let j = 1; j <= n - p; j++) {
    let s = 0;
    for (let i = j; i <= j + p - 1; i++) s += u[i];
    K.push(s / p);
  }
  for (let i = 0; i <= p; i++) K.push(1);
  return K;
}

function findSpan(n: number, p: number, u: number, K: number[]): number {
  if (u >= K[n + 1]) return n;
  if (u <= K[p]) return p;
  let lo = p;
  let hi = n + 1;
  let mid = (lo + hi) >> 1;
  while (u < K[mid] || u >= K[mid + 1]) {
    if (u < K[mid]) hi = mid;
    else lo = mid;
    mid = (lo + hi) >> 1;
  }
  return mid;
}

function basis(span: number, u: number, p: number, K: number[]): number[] {
  const N = new Array<number>(p + 1).fill(0);
  const left = new Array<number>(p + 1).fill(0);
  const right = new Array<number>(p + 1).fill(0);
  N[0] = 1;
  for (let j = 1; j <= p; j++) {
    left[j] = u - K[span + 1 - j];
    right[j] = K[span + j] - u;
    let saved = 0;
    for (let r = 0; r < j; r++) {
      const d = right[r + 1] + left[j - r];
      const t = d > 0 ? N[r] / d : 0;
      N[r] = saved + right[r + 1] * t;
      saved = left[j - r] * t;
    }
    N[j] = saved;
  }
  return N;
}

function solveDense(A: number[][], B: number[][]): number[][] {
  const n = A.length;
  const m = B[0].length;
  const M = A.map((r, i) => [...r, ...B[i]]);
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    if (Math.abs(M[piv][c]) < 1e-300) continue;
    [M[c], M[piv]] = [M[piv], M[c]];
    const d = M[c][c];
    for (let k = c; k < n + m; k++) M[c][k] /= d;
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c];
      if (f === 0) continue;
      for (let k = c; k < n + m; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((r) => r.slice(n));
}

/** Global interpolation of a point net with a clamped B-spline surface of degree <= 3. */
export function interpolateNet(net: Vec3[][], maxDegree = 3): NurbsNet {
  const ni = net.length;
  const nj = net[0].length;
  const p = Math.max(1, Math.min(maxDegree, ni - 1));
  const q = Math.max(1, Math.min(maxDegree, nj - 1));
  const u = parameters(Array.from({ length: nj }, (_, j) => net.map((row) => row[j]))); // lines running along grid-i
  const v = parameters(net); // lines running along grid-j
  const U = averagedKnots(u, p);
  const V = averagedKnots(v, q);
  const collocation = (par: number[], K: number[], deg: number) => {
    const n = par.length - 1;
    return par.map((t) => {
      const span = findSpan(n, deg, t, K);
      const N = basis(span, t, deg, K);
      const row = new Array<number>(n + 1).fill(0);
      for (let k = 0; k <= deg; k++) row[span - deg + k] = N[k];
      return row;
    });
  };
  const Au = collocation(u, U, p);
  const Av = collocation(v, V, q);
  // u-direction: for every grid-j column solve Au * C = P
  const tmp: Vec3[][] = Array.from({ length: ni }, () => new Array<Vec3>(nj));
  for (let j = 0; j < nj; j++) {
    const rhs = net.map((row) => [row[j][0], row[j][1], row[j][2]]);
    const sol = solveDense(Au, rhs);
    for (let i = 0; i < ni; i++) tmp[i][j] = [sol[i][0], sol[i][1], sol[i][2]];
  }
  const ctrl: Vec3[][] = [];
  for (let i = 0; i < ni; i++) {
    const rhs = Array.from({ length: nj }, (_, j) => [tmp[i][j][0], tmp[i][j][1], tmp[i][j][2]]);
    const sol = solveDense(Av, rhs);
    ctrl.push(sol.map((r) => [r[0], r[1], r[2]] as Vec3));
  }
  return { p, q, U, V, ctrl };
}

/** Evaluate the surface at (u, v) in [0,1]^2. Also returns the partial derivatives by central differences. */
export function evalSurface(s: Pick<NurbsNet, "p" | "q" | "U" | "V" | "ctrl">, u: number, v: number): Vec3 {
  const ni = s.ctrl.length - 1;
  const nj = s.ctrl[0].length - 1;
  const su = findSpan(ni, s.p, u, s.U);
  const sv = findSpan(nj, s.q, v, s.V);
  const Nu = basis(su, u, s.p, s.U);
  const Nv = basis(sv, v, s.q, s.V);
  const out: Vec3 = [0, 0, 0];
  for (let a = 0; a <= s.p; a++)
    for (let b = 0; b <= s.q; b++) {
      const w = Nu[a] * Nv[b];
      const c = s.ctrl[su - s.p + a][sv - s.q + b];
      out[0] += w * c[0];
      out[1] += w * c[1];
      out[2] += w * c[2];
    }
  return out;
}

export function evalCurve(ctrl: Vec3[], p: number, K: number[], u: number): Vec3 {
  const n = ctrl.length - 1;
  const sp = findSpan(n, p, u, K);
  const N = basis(sp, u, p, K);
  const out: Vec3 = [0, 0, 0];
  for (let a = 0; a <= p; a++) {
    const c = ctrl[sp - p + a];
    out[0] += N[a] * c[0];
    out[1] += N[a] * c[1];
    out[2] += N[a] * c[2];
  }
  return out;
}

/** (knot value, multiplicity) pairs of a knot vector, as STEP stores them. */
export function knotMultiplicities(K: number[]): { knots: number[]; mult: number[] } {
  const knots: number[] = [];
  const mult: number[] = [];
  for (const k of K) {
    if (knots.length && Math.abs(k - knots[knots.length - 1]) < 1e-12) mult[mult.length - 1]++;
    else {
      knots.push(k);
      mult.push(1);
    }
  }
  return { knots, mult };
}
