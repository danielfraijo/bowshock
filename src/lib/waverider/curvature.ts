/**
 * Curvature-adaptive 1-D sampling of a space curve with sharp-corner detection.
 *
 * Given a curve P(t), t in [0,1], pick n parameter values such that
 *   - the end points and every detected sharp corner (turning angle >= cornerDeg) are samples
 *     exactly, so corners are never cut or straddled by a chord;
 *   - the remaining points equidistribute a monitor  rho = 1 + A * curvature (arc-length
 *     curvature, scaled by the curve length) plus a short graded bump next to each corner, so
 *     tight arcs get dense points and smooth / straight stretches stay smooth;
 *   - the result is strictly increasing, deterministic and never NaN, for any finite input.
 * strength = 0 is NOT handled here (callers skip the module); strength is clamped to [0, 1].
 */
export type Pt3 = readonly [number, number, number];

export interface CurvatureOptions {
  /** 0..1: how strongly curvature pulls points in. */
  strength: number;
  /** Turning angle (deg) between adjacent chords that counts as a sharp corner. Default 25. */
  cornerDeg?: number;
  /** Maximum number of corners that are honoured (default n/3). */
  maxCorners?: number;
}

export interface CurvatureSampling {
  t: Float64Array;
  /** Parameter values of the corners that were honoured (subset of t, ascending). */
  corners: number[];
  /** Max turning angle (deg) found between adjacent chords of the dense polyline. */
  maxTurnDeg: number;
}

const clamp = (x: number, a: number, b: number) => Math.min(Math.max(x, a), b);

export function curvatureAdaptiveParams(
  point: (t: number) => Pt3,
  n: number,
  opts: CurvatureOptions,
): CurvatureSampling {
  const N = Math.max(2, Math.floor(Number.isFinite(n) ? n : 2));
  const uniform = (): CurvatureSampling => ({
    t: Float64Array.from({ length: N }, (_, i) => i / (N - 1)),
    corners: [],
    maxTurnDeg: 0,
  });
  const A = 4 * clamp(Number.isFinite(opts.strength) ? opts.strength : 0, 0, 1);
  if (N < 3 || A <= 0) return uniform();
  const cornerRad = (clamp(opts.cornerDeg ?? 25, 5, 120) * Math.PI) / 180;
  const M = Math.max(16 * N, 256);

  // dense polyline
  const px = new Float64Array(M + 1);
  const py = new Float64Array(M + 1);
  const pz = new Float64Array(M + 1);
  for (let i = 0; i <= M; i++) {
    const p = point(i / M);
    if (!Number.isFinite(p[0] + p[1] + p[2])) return uniform();
    px[i] = p[0];
    py[i] = p[1];
    pz[i] = p[2];
  }
  const ds = new Float64Array(M);
  let total = 0;
  for (let k = 0; k < M; k++) {
    ds[k] = Math.hypot(px[k + 1] - px[k], py[k + 1] - py[k], pz[k + 1] - pz[k]);
    total += ds[k];
  }
  if (!(total > 1e-14)) return uniform();

  // turning angle at interior vertices (between chord k-1 and k)
  const turn = new Float64Array(M + 1);
  let maxTurn = 0;
  for (let i = 1; i < M; i++) {
    const a = ds[i - 1];
    const b = ds[i];
    if (a < 1e-14 * total || b < 1e-14 * total) continue;
    const c =
      ((px[i] - px[i - 1]) * (px[i + 1] - px[i]) +
        (py[i] - py[i - 1]) * (py[i + 1] - py[i]) +
        (pz[i] - pz[i - 1]) * (pz[i + 1] - pz[i])) /
      (a * b);
    turn[i] = Math.acos(clamp(c, -1, 1));
    if (turn[i] > maxTurn) maxTurn = turn[i];
  }

  // corners = local maxima of the turning angle above the threshold (a corner may be smeared
  // over 2 adjacent vertices when it falls between samples: sum a +-1 window to measure it).
  const win = (i: number) => turn[i] + (i > 1 ? turn[i - 1] : 0) + (i < M - 1 ? turn[i + 1] : 0);
  const cand: { i: number; w: number }[] = [];
  for (let i = 1; i < M; i++) {
    const w = win(i);
    if (w >= cornerRad && turn[i] >= turn[i - 1] && turn[i] > turn[i + 1] - 1e-15) {
      if (cand.length && i - cand[cand.length - 1].i < 2) {
        if (w > cand[cand.length - 1].w) cand[cand.length - 1] = { i, w };
      } else cand.push({ i, w });
    }
  }
  const maxCorners = Math.max(0, Math.min(opts.maxCorners ?? Math.floor(N / 3), N - 2));
  cand.sort((a, b) => b.w - a.w);
  const kept = cand.slice(0, maxCorners).sort((a, b) => a.i - b.i);
  const cornerVerts = kept.map((c) => c.i);

  // monitor per chord: arc-length curvature (rad per unit length * total), smoothed, capped
  const rho = new Float64Array(M);
  const isCornerVert = new Uint8Array(M + 1);
  for (const i of cornerVerts) isCornerVert[i] = 1;
  const kap = new Float64Array(M);
  for (let k = 0; k < M; k++) {
    const t0 = isCornerVert[k] ? 0 : turn[k];
    const t1 = isCornerVert[k + 1] ? 0 : turn[k + 1];
    const len = Math.max(ds[k], 1e-14 * total);
    kap[k] = ((0.5 * (t0 + t1)) / len) * total;
  }
  const sm = new Float64Array(M);
  for (let k = 0; k < M; k++) {
    let s = 0;
    let w = 0;
    for (let d = -2; d <= 2; d++) {
      const j = k + d;
      if (j < 0 || j >= M) continue;
      const wt = 3 - Math.abs(d);
      s += kap[j] * wt;
      w += wt;
    }
    sm[k] = s / w;
  }
  const KCAP = 40;
  let cumLen = 0;
  const cornerS = cornerVerts.map((i) => {
    let s = 0;
    for (let k = 0; k < i; k++) s += ds[k];
    return s;
  });
  for (let k = 0; k < M; k++) {
    const sMid = cumLen + 0.5 * ds[k];
    let bump = 0;
    for (const cs of cornerS) bump += Math.exp(-Math.abs(sMid - cs) / (0.03 * total));
    rho[k] = 1 + A * (Math.min(sm[k], KCAP) / 4 + 0.5 * bump);
    cumLen += ds[k];
  }
  // cumulative mass M(t_i)
  const mass = new Float64Array(M + 1);
  for (let k = 0; k < M; k++) mass[k + 1] = mass[k] + rho[k] * ds[k];

  // Refine each corner to sub-sample accuracy: with a short probe h, the turning angle of the
  // chords (P(t-h), P(t)) -> (P(t), P(t+h)) peaks exactly when t sits on the kink.
  const cornerParam = cornerVerts.map((i) => {
    const h = 0.25 / M;
    const ang = (t: number) => {
      const a = point(clamp(t - h, 0, 1));
      const b = point(t);
      const c = point(clamp(t + h, 0, 1));
      const ux = b[0] - a[0];
      const uy = b[1] - a[1];
      const uz = b[2] - a[2];
      const vx = c[0] - b[0];
      const vy = c[1] - b[1];
      const vz = c[2] - b[2];
      const lu = Math.hypot(ux, uy, uz);
      const lv = Math.hypot(vx, vy, vz);
      if (lu < 1e-300 || lv < 1e-300) return 0;
      return Math.acos(clamp((ux * vx + uy * vy + uz * vz) / (lu * lv), -1, 1));
    };
    let lo = Math.max(0, (i - 1) / M);
    let hi = Math.min(1, (i + 1) / M);
    for (let it = 0; it < 50; it++) {
      const m1 = lo + (hi - lo) / 3;
      const m2 = hi - (hi - lo) / 3;
      if (ang(m1) < ang(m2)) lo = m1;
      else hi = m2;
    }
    return clamp(0.5 * (lo + hi), 1e-9, 1 - 1e-9);
  });

  const bounds = [0, ...cornerVerts, M];
  const nPieces = bounds.length - 1;
  const interior = N - (nPieces + 1);
  const massAt = (t: number) => {
    const f = t * M;
    const k = Math.min(Math.floor(f), M - 1);
    return mass[k] + (mass[k + 1] - mass[k]) * (f - k);
  };
  const bT = [0, ...cornerParam, 1];
  const bM = bT.map(massAt);
  const pieceMass = bM.slice(1).map((m, p) => m - bM[p]);
  const totalMass = pieceMass.reduce((a, b) => a + b, 0);
  // largest-remainder allocation of interior points
  const exact = pieceMass.map((m) => (interior * m) / totalMass);
  const cnt = exact.map(Math.floor);
  let left = interior - cnt.reduce((a, b) => a + b, 0);
  const order = exact.map((e, i) => ({ i, r: e - Math.floor(e) })).sort((a, b) => b.r - a.r || a.i - b.i);
  for (let q = 0; left > 0; q = (q + 1) % nPieces, left--) cnt[order[q].i]++;

  const invert = (target: number): number => {
    // find chord with mass[k] <= target <= mass[k+1] (binary search), linear inside
    let lo = 0;
    let hi = M;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (mass[mid] <= target) lo = mid;
      else hi = mid;
    }
    const dm = mass[lo + 1] - mass[lo];
    const f = dm > 0 ? (target - mass[lo]) / dm : 0;
    return (lo + clamp(f, 0, 1)) / M;
  };
  const out: number[] = [0];
  const cornerT: number[] = [];
  for (let p = 0; p < nPieces; p++) {
    const m0 = bM[p];
    const m1 = bM[p + 1];
    const m = cnt[p];
    for (let q = 1; q <= m; q++) out.push(invert(m0 + ((m1 - m0) * q) / (m + 1)));
    const tEnd = bT[p + 1];
    out.push(tEnd);
    if (p < nPieces - 1) cornerT.push(tEnd);
  }
  const t = Float64Array.from(out);
  t[0] = 0;
  t[t.length - 1] = 1;
  // guarantee strict monotonicity (never relied on at present, defensive for degenerate masses)
  for (let i = 1; i < t.length; i++) if (!(t[i] > t[i - 1])) t[i] = Math.min(1, t[i - 1] + 1e-9);
  return { t, corners: cornerT, maxTurnDeg: (maxTurn * 180) / Math.PI };
}

/**
 * Re-distribute spanwise loft stations by curvature of the leading-edge / keel curve.
 *
 * `ys` is the existing (ascending, cosine-clustered) station list: [-yMax .. 0 .. yMax] for a full model or [0 .. yMax] for a
 * half model. The stations are re-sampled in the *index* parameter of that list (piecewise linear), so strength 0 returns the
 * very same stations, the end stations (y = 0 and ±yMax, which stay off the zero-chord pole) are never moved, y = 0 is kept
 * on a full model, a sharp corner of the curve is always a station, and the result is mirror-symmetric and strictly ascending.
 * Station count is unchanged. Any non-finite input falls back to `ys`.
 */
export function curvatureStations(ys: number[], strength: number, curveAt: (y: number) => Pt3): number[] {
  const n = ys.length;
  if (!(strength > 0) || n < 5 || !ys.every(Number.isFinite)) return ys;
  const full = ys[0] < 0;
  const i0 = full ? (n - 1) >> 1 : 0;
  const half = ys.slice(i0); // ascending from 0 .. yMax
  const m = half.length;
  if (m < 4 || !(half[m - 1] > half[0])) return ys;
  const yAt = (u: number) => {
    const x = Math.min(Math.max(u, 0), 1) * (m - 1);
    const k = Math.min(Math.floor(x), m - 2);
    return half[k] + (half[k + 1] - half[k]) * (x - k);
  };
  let s: CurvatureSampling;
  try {
    s = curvatureAdaptiveParams((u) => curveAt(yAt(u)), m, { strength });
  } catch {
    return ys;
  }
  const out = Array.from(s.t, (u) => yAt(u));
  out[0] = half[0];
  out[m - 1] = half[m - 1];
  for (let k = 1; k < m; k++) if (!(out[k] > out[k - 1])) return ys;
  if (!full) return out;
  const res = new Array<number>(n);
  for (let k = 0; k < m; k++) {
    res[i0 + k] = out[k];
    res[i0 - k] = -out[k];
  }
  return res;
}
