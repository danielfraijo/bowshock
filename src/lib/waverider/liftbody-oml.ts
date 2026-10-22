/**
 * Lifting-body outer mold line.
 *
 * Nose: osculating paraboloid r² = 2 R x. Curvature at the pole is 1/R, which is
 * the radius in Sutton–Graves and Fay–Riddell. A hemisphere is not used — its
 * equator has zero slope, so the loft reads as a ball glued on the body.
 *
 * The parabola runs out to a point where a power-law fade of the slope,
 *   m(t) = m_body + (m_nose − m_body) (1−t)^γ
 * hits the aft section with a monotone-decreasing slope (no shoulder) and the
 * right radius. γ is fixed by the secant (the integral of m has to land on the
 * body). Fritsch–Carlson cubics were not used: they stay monotone in r but can
 * still re-steepen. If the requested R cannot do this inside the OML, R is
 * reduced and reported.
 */

export interface LiftOmlSpec {
  length: number;
  span: number;
  height: number;
  /** Requested nose radius of curvature [m]. */
  leRadius: number;
  planform: string;
  planformPower: number;
}

export interface LiftOml {
  length: number;
  /** Radius of curvature actually built. */
  rn: number;
  requested: number;
  a: (x: number) => number;
  upper: (x: number) => number;
  lower: (x: number) => number;
}

function clamp(v: number, a: number, b: number): number {
  return Math.max(a, Math.min(b, v));
}

/**
 * One semi-axis: osculating parabola of radius `rn` until it has room to fade,
 * with a monotone-decreasing slope, into a crest, then a short boattail.
 * Slope never increases, so there is no spherical shoulder.
 */
function axisOf(rn: number, rMax: number, L: number): (x: number) => number {
  const rPeak = Math.max(rMax, 1e-4);
  // Deepest cap that still sits under 70% of the crest and inside the first 40% of length.
  let xCap = Math.min((0.62 * rPeak) ** 2 / (2 * rn), 0.28 * L, 0.85 * rn);
  xCap = Math.max(xCap, Math.min(0.35 * rn, 0.12 * L));
  let p0 = Math.sqrt(2 * rn * xCap);
  if (p0 > 0.78 * rPeak) {
    p0 = 0.78 * rPeak;
    xCap = (p0 * p0) / (2 * rn);
  }
  const m0 = rn / Math.max(p0, 1e-9);
  const xCrest = clamp(Math.max(xCap * 2.4, 0.34 * L), xCap + 0.08 * L, 0.7 * L);
  const dx = xCrest - xCap;
  let m1 = (0.04 * rPeak) / Math.max(L, 1e-6);
  let sec = (rPeak - p0) / dx;
  if (sec <= m1) m1 = sec * 0.35;
  if (!(m0 > sec)) {
    // Requested radius is too sharp to need a fade; ride the parabola up to the crest.
    const xHit = Math.min(xCrest, (rPeak * rPeak) / (2 * rn));
    return (x) => {
      if (x <= 1e-12) return 0;
      if (x <= xHit) return Math.sqrt(2 * rn * x);
      const u = clamp((x - xHit) / Math.max(L - xHit, 1e-9), 0, 1);
      const rBase = 0.9 * rPeak;
      return rPeak + (rBase - rPeak) * u * u;
    };
  }
  const gamma = (m0 - m1) / Math.max(sec - m1, 1e-9) - 1;
  const g = clamp(gamma, 0.2, 14);
  const rBase = 0.9 * rPeak;
  return (x: number) => {
    if (x <= 1e-12) return 0;
    if (x <= xCap) return Math.sqrt(2 * rn * x);
    if (x >= L) return rBase;
    if (x >= xCrest) {
      const u = (x - xCrest) / Math.max(L - xCrest, 1e-9);
      return rPeak + (rBase - rPeak) * u * u;
    }
    const t = (x - xCap) / dx;
    const integ = m1 * t + (sec - m1) * (1 - Math.pow(1 - t, g + 1));
    return Math.max(0, p0 + dx * integ);
  };
}

/** Shared OML for the CAD loft and the DATCOM ELLBOD stations. */
export function liftOml(spec: LiftOmlSpec): LiftOml {
  const L = Math.max(spec.length, 1e-3);
  const s = Math.max(spec.span / 2, 1e-3);
  const h = Math.max(spec.height, 1e-3);
  const bu = 0.58 * h;
  const bl = 0.42 * h;
  // Largest R whose parabola still fits under every semi-axis inside the forebody.
  const rMin = Math.min(s, bu, bl);
  const rnMax = (0.7 * rMin) ** 2 / (2 * Math.max(0.12 * L, 1e-4));
  const requested = clamp(Math.max(spec.leRadius, 0.004 * L), 0.003 * L, Math.max(rnMax, 0.004 * L));
  const rn = requested;
  return {
    length: L,
    rn,
    requested: spec.leRadius,
    a: axisOf(rn, s, L),
    upper: axisOf(rn, bu, L),
    lower: axisOf(rn, bl, L),
  };
}

/** Streamwise stations. Cosine on the nose, smoothstep through the fairing, uniform aft. */
export function liftXs(oml: LiftOml, nx: number): number[] {
  const L = oml.length;
  const n = Math.max(64, nx);
  const nNose = Math.max(20, Math.round(n * 0.28));
  const nBlend = Math.max(20, Math.round(n * 0.34));
  const nAft = Math.max(22, n - nNose - nBlend);
  let xCap = Math.min(0.5 * oml.rn, 0.06 * L);
  const probe = Math.sqrt(Math.max(2 * oml.rn * xCap, 0));
  if (probe > 0 && Math.abs(oml.upper(xCap) - probe) / probe > 0.03) {
    let lo = 1e-5 * L;
    let hi = Math.min(0.15 * L, oml.rn);
    for (let k = 0; k < 28; k++) {
      const mid = 0.5 * (lo + hi);
      const pr = Math.sqrt(2 * oml.rn * mid);
      const err = Math.abs(oml.upper(mid) - pr) / Math.max(pr, 1e-9);
      if (err < 0.015) lo = mid;
      else hi = mid;
    }
    xCap = Math.max(lo, 1e-4 * oml.rn);
  }
  const xJoin = Math.min(0.5 * L, Math.max(0.14 * L, 8 * oml.rn));
  const xs: number[] = [];
  for (let i = 0; i < nNose; i++) {
    const phi = (Math.PI / 2) * (i / Math.max(nNose - 1, 1));
    xs.push(xCap * (1 - Math.cos(phi)));
  }
  for (let i = 1; i <= nBlend; i++) {
    const t = i / nBlend;
    const u = t * t * (3 - 2 * t);
    xs.push(xCap + (xJoin - xCap) * u);
  }
  for (let i = 1; i <= nAft; i++) xs.push(xJoin + ((L - xJoin) * i) / nAft);
  xs[0] = 0;
  xs[xs.length - 1] = L;
  for (let i = 1; i < xs.length; i++) if (xs[i] <= xs[i - 1]) xs[i] = Math.min(L, xs[i - 1] + 1e-6 * L);
  xs[xs.length - 1] = L;
  return xs;
}
