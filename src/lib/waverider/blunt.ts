/**
 * Circular leading-edge fillet for Pointwise-grade CAD.
 *
 * Sharp inverse-design waveriders meet at a knife-edge. CFD / automatic
 * surface mesh needs a finite nose radius: a circular arc in the local
 * XZ (constant-y) plane, G1 to upper and lower. Spanwise y is locked so
 * the loft stays a structured constant-y station — no diagonal cuts.
 *
 * Radius tapers where the local chord cannot hold the requested R.
 * Tips keep a tiny finite chord (the span list never sits on y = ±s)
 * so the wingtip is not a pole and is not a rectangular chop.
 *
 * References: Bowcutt viscous waveriders; Takashima / Lewis blunt LE;
 * standard 2-D rolling-ball fillet.
 */
import type { DesignParams, SurfaceGrid, Vec3 } from "./types";
import { clamp, vadd, vcross, vdot, vlen, vlerp, vnorm, vscale, vsub } from "./math";
import { gridPoint, makeGrid, setGridPoint } from "./mesh";

/** Max facet turning on the fillet arc (deg). Sets the arc point count (9..21). */
const LE_MAX_SEG = 10 * (Math.PI / 180);
const ALPHA_MIN = 3.5 * (Math.PI / 180);
/** Longest span edge a fillet quad may have, in units of the arc size. */
const FILLET_ASPECT = 12;
/** Tangent length of the blend never exceeds this many radii (bounds the material removed). */
const S_CAP = 4;
/** G2 blends need a much longer transition than a circle to turn the same angle with zero end curvature. */
const S_CAP_G2 = 40;
/** Blend footprint never exceeds this fraction of the local row length. */
const CHORD_FRAC = 0.12;
/** The tangent length may change by at most this much per unit LE length (smooth radius law). */
const S_LIP = 0.5;
/** Longest LE edge over shortest arc facet the fillet strips may have. */
const LE_ASPECT = 400;
/** Arcs smaller than this fraction of the body length are not tessellated (they leave micro holes between unwelded grids). */
const LE_MIN_SCALE = 2e-4;

export type LeBlend = "circular" | "g2";

export interface FilletOptions {
  /** circular (default): README contract. g2: quintic blend with zero end curvature (curvature-continuous). */
  blend?: LeBlend;
}

export function effectiveLeRadius(params: DesignParams): number {
  if (params.leBlunt === false) return 0;
  const L = Math.max(params.length, 1e-6);
  const requested = params.leRadius > 0 ? params.leRadius : 0.005 * L;
  return clamp(requested, 0.001 * L, 0.04 * L);
}

function rowOf(g: SurfaceGrid, j: number): Vec3[] {
  const pts: Vec3[] = [];
  for (let i = 0; i < g.ni; i++) pts.push(gridPoint(g, i, j));
  return pts;
}

function writeRow(g: SurfaceGrid, j: number, pts: Vec3[]) {
  const n = Math.min(g.ni, pts.length);
  for (let i = 0; i < n; i++) setGridPoint(g, i, j, pts[i]);
}

function chordLen(pts: Vec3[]): number {
  let s = 0;
  for (let i = 1; i < pts.length; i++) s += vlen(vsub(pts[i], pts[i - 1]));
  return s;
}

function pointAtLength(pts: Vec3[], sWant: number): { p: Vec3; k: number } {
  let acc = 0;
  for (let i = 0; i < pts.length - 1; i++) {
    const d = vlen(vsub(pts[i + 1], pts[i]));
    if (acc + d >= sWant || i === pts.length - 2) {
      const u = d > 1e-15 ? clamp((sWant - acc) / d, 0, 1) : 0;
      return { p: vlerp(pts[i], pts[i + 1], u), k: i + 1 };
    }
    acc += d;
  }
  return { p: pts[pts.length - 1], k: pts.length - 1 };
}

/**
 * Re-place the points of a row on the polyline `pts` (the part of the row behind the fillet tangent
 * point) keeping the ORIGINAL relative arclength spacing `fr` (0..1). The grid's own cell grading is
 * preserved, so the first cell behind the fillet is as wide as the loft intended (no sliver).
 */
function resampleFrac(pts: Vec3[], fr: number[]): Vec3[] {
  const n = fr.length;
  if (pts.length === 0) return pts;
  if (pts.length === 1) return Array.from({ length: n }, () => pts[0]);
  const acc: number[] = [0];
  for (let i = 1; i < pts.length; i++) acc.push(acc[i - 1] + vlen(vsub(pts[i], pts[i - 1])));
  const total = acc[acc.length - 1];
  if (total < 1e-15) return Array.from({ length: n }, () => pts[0]);
  const out: Vec3[] = [];
  let k = 0;
  for (let i = 0; i < n; i++) {
    const sPos = clamp(fr[i], 0, 1) * total;
    while (k < acc.length - 2 && acc[k + 1] < sPos) k++;
    const span = acc[k + 1] - acc[k];
    const t = span > 1e-15 ? (sPos - acc[k]) / span : 0;
    out.push(vlerp(pts[k], pts[k + 1], clamp(t, 0, 1)));
  }
  out[0] = pts[0];
  out[n - 1] = pts[pts.length - 1];
  return out;
}

function arcFractions(row: Vec3[]): number[] {
  const acc: number[] = [0];
  for (let i = 1; i < row.length; i++) acc.push(acc[i - 1] + vlen(vsub(row[i], row[i - 1])));
  const tot = acc[acc.length - 1] || 1;
  return acc.map((v) => v / tot);
}

function slerpUnit(a: Vec3, b: Vec3, t: number): Vec3 {
  const d = clamp(vdot(a, b), -1, 1);
  const om = Math.acos(d);
  if (om < 1e-8) return vnorm(vadd(a, vscale(vsub(b, a), t)));
  const s = Math.sin(om);
  return vnorm(vadd(vscale(a, Math.sin((1 - t) * om) / s), vscale(b, Math.sin(t * om) / s)));
}

/**
 * Quintic Bezier blend between the two tangent lines with zero curvature at both ends
 * (P0,P1,P2 and P3,P4,P5 are collinear), so it meets planar faces with G2 contact.
 * Control offsets (fractions of the tangent length s measured from the apex V):
 * P0 = s, P1 = 0.40 s, P2 = 0 (the apex), mirrored on the other side.
 */
const G2_A1 = 0.4;
function bezier5(c: Vec3[], t: number): Vec3 {
  const u = 1 - t;
  const w = [u ** 5, 5 * t * u ** 4, 10 * t * t * u ** 3, 10 * t ** 3 * u * u, 5 * t ** 4 * u, t ** 5];
  let x = 0;
  let y = 0;
  let z = 0;
  for (let k = 0; k < 6; k++) {
    x += w[k] * c[k][0];
    y += w[k] * c[k][1];
    z += w[k] * c[k][2];
  }
  return [x, y, z];
}

/** Apex radius of the unit-tangent-length G2 blend for half-angle h (radians). */
export function g2ApexRadiusUnit(h: number): number {
  const uL: Vec3 = [Math.cos(h), -Math.sin(h), 0];
  const uU: Vec3 = [Math.cos(h), Math.sin(h), 0];
  const c = g2Controls([0, 0, 0], uL, uU, 1);
  const d = 1e-3;
  const p0 = bezier5(c, 0.5 - d);
  const p1 = bezier5(c, 0.5);
  const p2 = bezier5(c, 0.5 + d);
  const ax = (p2[0] - p0[0]) / (2 * d);
  const ay = (p2[1] - p0[1]) / (2 * d);
  const bx = (p2[0] - 2 * p1[0] + p0[0]) / (d * d);
  const by = (p2[1] - 2 * p1[1] + p0[1]) / (d * d);
  const k = Math.abs(ax * by - ay * bx) / Math.pow(ax * ax + ay * ay, 1.5);
  return 1 / Math.max(k, 1e-9);
}

function g2Controls(V: Vec3, uL: Vec3, uU: Vec3, s: number): Vec3[] {
  const at = (u: Vec3, f: number): Vec3 => vadd(V, vscale(u, s * f));
  return [at(uL, 1), at(uL, G2_A1), at(uL, 0), at(uU, 0), at(uU, G2_A1), at(uU, 1)];
}

function planar(v: Vec3, y: number): Vec3 {
  return [v[0], y, v[2]];
}

/** In-plane unit vector perpendicular to T (removes the LE-tangent component). */
function perpTo(v: Vec3, T: Vec3): Vec3 {
  return vnorm(vsub(v, vscale(T, vdot(v, T))));
}

/** Vector from the LE point to where the y = const plane meets a normal-plane offset D. */
function toSection(D: Vec3, T: Vec3): Vec3 {
  if (Math.abs(T[1]) < 0.02) return [D[0], 0, D[2]];
  const k = D[1] / T[1];
  return [D[0] - k * T[0], 0, D[2] - k * T[2]];
}

/** LE tangent at each station from the LE polyline; planform corners use the +span side. */
function leTangents(P: Vec3[]): Vec3[] {
  const n = P.length;
  const out: Vec3[] = [];
  const dir = (a: Vec3, b: Vec3): Vec3 | null => {
    const d = vsub(b, a);
    return vlen(d) > 1e-12 ? vnorm(d) : null;
  };
  for (let j = 0; j < n; j++) {
    const f = j + 1 < n ? dir(P[j], P[j + 1]) : null;
    const b = j > 0 ? dir(P[j - 1], P[j]) : null;
    let T: Vec3;
    if (f && b) {
      if (vdot(f, b) > Math.cos((55 * Math.PI) / 180)) T = vnorm(vadd(f, b));
      else T = P[j][1] >= 0 ? f : b; // corner (e.g. the delta nose): take the side away from the centerline
    } else T = f ?? b ?? [0, 1, 0];
    out.push(T);
  }
  return out;
}

interface Frame {
  ok: boolean;
  P: Vec3;
  T: Vec3;
  uU: Vec3;
  uL: Vec3;
  h: number;
  alpha: number;
  U: Vec3[];
  Lo: Vec3[];
  wU: Vec3;
  wL: Vec3;
  chord: number;
  s: number;
}

interface Arc {
  pts: Vec3[];
  tu: Vec3;
  tl: Vec3;
  uRest: Vec3[];
  lRest: Vec3[];
  scale: number;
  R: number;
}

function stationFrame(U: Vec3[], Lo: Vec3[], T: Vec3, sgnUp: number, Rreq: number): Frame {
  const P = vlerp(U[0], Lo[0], 0.5);
  const bad = (): Frame => ({ ok: false, P, T, uU: T, uL: T, h: 0, alpha: 0, U, Lo, wU: T, wL: T, chord: 0, s: 0 });
  if (vlen(vsub(U[0], Lo[0])) > 0.6 * Math.max(Rreq, 1e-9)) return bad();
  const chordU = chordLen(U);
  const chordL = chordLen(Lo);
  const chord = Math.max(Math.min(chordU, chordL), 1e-12);
  const probe = (row: Vec3[], c: number): Vec3 => {
    const first = vlen(vsub(row[1], row[0]));
    const q = pointAtLength(row, Math.max(first, 0.02 * c)).p;
    return vsub(q, P);
  };
  const rU = probe(U, chordU);
  const rL = probe(Lo, chordL);
  if (vlen(rU) < 1e-12 || vlen(rL) < 1e-12) return bad();
  let uU = perpTo(rU, T);
  let uL = perpTo(rL, T);
  if (!(vlen(uU) > 0.5) || !(vlen(uL) > 0.5)) return bad();
  let alpha = Math.acos(clamp(vdot(uU, uL), -1, 1));
  if (alpha < ALPHA_MIN) {
    // open a near-zero wedge symmetrically about its bisector so s = R / tan(h) stays finite
    let bis = vadd(uU, uL);
    bis = vlen(bis) < 0.2 ? perpTo(vsub(rU, rL), T) : vnorm(bis);
    let thick = perpTo(vcross(T, bis), T);
    if (vdot(thick, [0, 0, sgnUp]) < 0) thick = vscale(thick, -1);
    const hh = ALPHA_MIN / 2;
    uU = vnorm(vadd(vscale(bis, Math.cos(hh)), vscale(thick, Math.sin(hh))));
    uL = vnorm(vadd(vscale(bis, Math.cos(hh)), vscale(thick, -Math.sin(hh))));
    alpha = ALPHA_MIN;
  }
  const h = Math.min(0.5 * alpha, (80 * Math.PI) / 180);
  const wU = toSection(uU, T);
  const wL = toSection(uL, T);
  return { ok: true, P, T, uU, uL, h, alpha, U, Lo, wU, wL, chord, s: 0 };
}

/** Points of the LE blend between the two tangent lines (apex P, unit directions uL/uU, tangent length s). */
function blendPoints(P: Vec3, uL: Vec3, uU: Vec3, s: number, h: number, nArc: number, blend: LeBlend): { raw: Vec3[]; R: number } {
  const raw: Vec3[] = [];
  if (blend === "g2") {
    const ctrl = g2Controls(P, uL, uU, s);
    for (let k = 0; k < nArc; k++) raw.push(bezier5(ctrl, k / (nArc - 1)));
    return { raw, R: s * g2ApexRadiusUnit(h) };
  }
  const R = s * Math.tan(h);
  const bis = vnorm(vadd(uU, uL));
  const AU = vadd(P, vscale(uU, s));
  const AL = vadd(P, vscale(uL, s));
  const d = R / Math.sin(h);
  const C = vadd(P, vscale(bis, d));
  const aL = vnorm(vsub(AL, C));
  const aU = vnorm(vsub(AU, C));
  const front = vscale(bis, -1);
  for (let k = 0; k < nArc; k++) {
    const t = k / (nArc - 1);
    const dir = t <= 0.5 ? slerpUnit(aL, front, t * 2) : slerpUnit(front, aU, t * 2 - 1);
    raw.push(vadd(C, vscale(dir, R)));
  }
  return { raw, R };
}

/**
 * 2-D LE profile for a wedge of half-angle `halfAngle` (rad): the blend between the lower and upper tangent
 * lines, n points, apex at the origin pointing to -x, tangent length 1. Exposed for the continuity tests.
 */
export function leBlendProfile(halfAngle: number, blend: LeBlend, n = 201): { pts: Vec3[]; radius: number; uL: Vec3; uU: Vec3 } {
  const uU: Vec3 = [Math.cos(halfAngle), 0, Math.sin(halfAngle)];
  const uL: Vec3 = [Math.cos(halfAngle), 0, -Math.sin(halfAngle)];
  const { raw, R } = blendPoints([0, 0, 0], uL, uU, 1, halfAngle, n, blend);
  return { pts: raw, radius: R, uL, uU };
}

function buildArc(f: Frame, s: number, y: number, nArc: number, blend: LeBlend): Arc {
  const h = f.h;
  const { raw, R } = blendPoints(f.P, f.uL, f.uU, s, h, nArc, blend);
  const pts = raw.map((q) => {
    const D = vsub(q, f.P);
    const d = toSection(D, f.T);
    return planar(vadd(f.P, d), y);
  });
  const lenU = vlen(toSection(vscale(f.uU, s), f.T));
  const lenL = vlen(toSection(vscale(f.uL, s), f.T));
  const cutU = pointAtLength(f.U, lenU);
  const cutL = pointAtLength(f.Lo, lenL);
  const tu = planar(cutU.p, y);
  const tl = planar(cutL.p, y);
  // land the arc exactly on the grid rows (curved rows differ from the planar model by O(kappa s^2))
  const eL = vsub(tl, pts[0]);
  const eU = vsub(tu, pts[nArc - 1]);
  for (let k = 0; k < nArc; k++) {
    const t = k / (nArc - 1);
    const wU = t * t * (3 - 2 * t);
    const wL = 1 - wU;
    pts[k] = planar(vadd(pts[k], vadd(vscale(eL, wL * wL), vscale(eU, wU * wU))), y);
  }
  pts[0] = tl;
  pts[nArc - 1] = tu;
  const front = pts[(nArc - 1) >> 1];
  const mid = vlerp(tl, tu, 0.5);
  const scale = Math.max(vlen(vsub(tu, tl)), 2 * vlen(vsub(front, mid)));
  return {
    pts,
    tu,
    tl,
    uRest: [tu, ...f.U.slice(Math.max(cutU.k, 1))],
    lRest: [tl, ...f.Lo.slice(Math.max(cutL.k, 1))],
    scale,
    R,
  };
}

/**
 * Per-span rolling-ball fillet. Returns a leading-edge surface (i along the arc,
 * j along span) or null if R = 0 or the arc cannot be resolved on this span lattice.
 *
 * The ball rolls in the plane NORMAL to the leading edge (not the constant-y plane),
 * so a swept edge gets a true radius R; each y = const station then carries the exact
 * section of that fillet. Tangent length is bounded by S_CAP radii, by the local row
 * length, and by the local thickness, and varies at most S_LIP per unit LE length so
 * the radius tapers smoothly toward the tips instead of switching off station by station.
 */
export function applyLeadingFillet(
  upper: SurfaceGrid,
  lower: SurfaceGrid,
  radius: number,
  length: number,
  opts: FilletOptions = {},
): SurfaceGrid | null {
  if (!(radius > 1e-9)) return null;
  const blend = opts.blend ?? "circular";
  const L = Math.max(length, 1e-6);
  const nj = Math.min(upper.nj, lower.nj);
  if (nj < 3) return null;
  const Us: Vec3[][] = [];
  const Ls: Vec3[][] = [];
  const P: Vec3[] = [];
  let up = 0;
  for (let j = 0; j < nj; j++) {
    Us.push(rowOf(upper, j));
    Ls.push(rowOf(lower, j));
    P.push(vlerp(Us[j][0], Ls[j][0], 0.5));
    up += Us[j][Us[j].length - 1][2] - Ls[j][Ls[j].length - 1][2];
  }
  const sgnUp = up >= 0 ? 1 : -1;
  const Rreq = Math.min(radius, 0.08 * L);
  const Ts = leTangents(P);
  const frames: Frame[] = [];
  for (let j = 0; j < nj; j++) frames.push(stationFrame(Us[j], Ls[j], Ts[j], sgnUp, Rreq));

  // nominal tangent length from R, then cap by thickness / row length / S_CAP
  const apexUnit = (h: number) => (blend === "g2" ? g2ApexRadiusUnit(h) : Math.tan(h));
  for (const f of frames) {
    if (!f.ok) continue;
    let s = Math.min(Rreq / apexUnit(f.h), (blend === "g2" ? S_CAP_G2 : S_CAP) * Rreq);
    const muU = Math.max(vlen(f.wU), 1e-9);
    const muL = Math.max(vlen(f.wL), 1e-9);
    s = Math.min(s, (CHORD_FRAC * chordLen(f.U)) / muU, (CHORD_FRAC * chordLen(f.Lo)) / muL);
    const sinH = Math.sin(f.h);
    for (let it = 0; it < 3 && s > 0; it++) {
      const tu = pointAtLength(f.U, s * muU).p;
      const tl = pointAtLength(f.Lo, s * muL).p;
      const gap = vlen(vsub(tu, tl));
      const sThick = (0.95 * gap) / Math.max(2 * sinH, 1e-9);
      if (sThick >= s) break;
      s = sThick;
    }
    f.s = Math.max(s, 0);
  }
  // Lipschitz-limit the tangent length along the LE so the radius law is smooth
  const step = (j: number) => vlen(vsub(P[j], P[j - 1]));
  for (let j = 1; j < nj; j++) frames[j].s = Math.min(frames[j].s, (frames[j - 1].ok ? frames[j - 1].s : 0) + S_LIP * step(j));
  for (let j = nj - 2; j >= 0; j--) frames[j].s = Math.min(frames[j].s, (frames[j + 1].ok ? frames[j + 1].s : 0) + S_LIP * step(j + 1));

  // Arc point count. Angle rule: facets turn at most LE_MAX_SEG (bounds the seam normal jump to half of it).
  // Aspect rule: the longest LE edge over the shortest arc facet stays <= LE_ASPECT, which can lower the count.
  let maxTurn = 0;
  for (const f of frames) if (f.ok && f.s > 0) maxTurn = Math.max(maxTurn, Math.PI - f.alpha);
  const nAngle = clamp(Math.ceil(maxTurn / LE_MAX_SEG) + 1, 7, 21);
  const dyOf = (j: number) => {
    let d = 0;
    if (j > 0) d = Math.max(d, vlen(vsub(P[j], P[j - 1])));
    if (j + 1 < nj) d = Math.max(d, vlen(vsub(P[j], P[j + 1])));
    return d;
  };
  const probe = frames.map((f, j) => (f.ok && f.s > 1e-9 * L ? buildArc(f, f.s, P[j][1], nAngle, blend) : null));
  const arcLen = (a: Arc) => {
    let l = 0;
    for (let k = 1; k < a.pts.length; k++) l += vlen(vsub(a.pts[k], a.pts[k - 1]));
    return l;
  };
  const okProbe = probe.map((a, j) => !!a && a.scale > LE_MIN_SCALE * L && dyOf(j) <= FILLET_ASPECT * a.scale && dyOf(j) <= (LE_ASPECT * arcLen(a)) / 6);
  let nFit = nAngle;
  for (let j = 0; j < nj; j++) {
    if (!okProbe[j]) continue;
    const fit = Math.floor((LE_ASPECT * arcLen(probe[j]!)) / Math.max(dyOf(j), 1e-12)) + 1;
    if (fit < nFit) nFit = fit;
  }
  const nArc = clamp(nFit, 7, nAngle) | 1;
  const arcs: (Arc | null)[] = nArc === nAngle ? probe : frames.map((f, j) => (f.ok && f.s > 1e-9 * L ? buildArc(f, f.s, P[j][1], nArc, blend) : null));
  const live = arcs.map((a, j) => !!a && okProbe[j] && a.scale > LE_MIN_SCALE * L && dyOf(j) <= FILLET_ASPECT * a.scale);
  let nLive = 0;
  for (const v of live) if (v) nLive++;
  if (nLive < 3) return null;

  const lead = makeGrid("leading", nArc, nj, (i, j) => (live[j] ? arcs[j]!.pts[i] : P[j]));
  for (let j = 0; j < nj; j++) {
    const y = P[j][1];
    if (!live[j]) {
      // unresolved: the arc collapses to the seam point; the row stays sharp there
      const m = planar(P[j], y);
      setGridPoint(upper, 0, j, m);
      setGridPoint(lower, 0, j, m);
      continue;
    }
    const a = arcs[j]!;
    writeRow(upper, j, resampleFrac(a.uRest.map((q) => planar(q, q[1])), arcFractions(Us[j])));
    writeRow(lower, j, resampleFrac(a.lRest.map((q) => planar(q, q[1])), arcFractions(Ls[j])));
    setGridPoint(upper, 0, j, a.tu);
    setGridPoint(lower, 0, j, a.tl);
    for (let k = 0; k < nArc; k++) setGridPoint(lead, k, j, a.pts[k]);
    setGridPoint(lead, 0, j, a.tl);
    setGridPoint(lead, nArc - 1, j, a.tu);
  }
  return lead;
}

/** D-shaped cap at a fillet tip so the arc is not a hanging edge. */
export function filletTipCap(lead: SurfaceGrid, j: number, name: string): SurfaceGrid | null {
  const n = lead.ni;
  if (n < 3 || j < 0 || j >= lead.nj) return null;
  const tl = gridPoint(lead, 0, j);
  const tu = gridPoint(lead, n - 1, j);
  let bow = 0;
  for (let k = 0; k < n; k++) {
    const t = n <= 1 ? 0 : k / (n - 1);
    bow = Math.max(bow, vlen(vsub(gridPoint(lead, k, j), vlerp(tl, tu, t))));
  }
  if (bow < 1e-10) return null;
  return makeGrid(name, 2, n, (i, k) => {
    const p = gridPoint(lead, k, j);
    if (i === 0) return p;
    return vlerp(tl, tu, n <= 1 ? 0 : k / (n - 1));
  });
}

/** Spherical nose on a star body so the pole is not a zero-area spike. */
export function bluntStarNose(grids: SurfaceGrid[], radius: number, length: number) {
  const R = Math.min(radius, 0.12 * Math.max(length, 1e-6));
  if (!(R > 1e-9)) return;
  const xJoin = R;
  for (const g of grids) {
    for (let i = 0; i < g.ni; i++) {
      for (let j = 0; j < g.nj; j++) {
        const p = gridPoint(g, i, j);
        if (p[0] >= xJoin) continue;
        const yz = Math.hypot(p[1], p[2]);
        const x = clamp(p[0], 0, xJoin);
        const rSph = Math.sqrt(Math.max(0, R * R - (x - R) * (x - R)));
        if (yz < 1e-12) {
          setGridPoint(g, i, j, [x, 0, 0]);
          continue;
        }
        const s = rSph / yz;
        setGridPoint(g, i, j, [x, p[1] * s, p[2] * s]);
      }
    }
  }
}

/** External cowl-lip radius: quarter-cylinder along the span at x=0. */
export function applyCowlLip(cowl: SurfaceGrid, radius: number, length: number): SurfaceGrid | null {
  const L = Math.max(length, 1e-6);
  const R = Math.min(radius, 0.06 * L);
  if (!(R > 1e-9) || cowl.ni < 3) return null;
  const nj = cowl.nj;
  const nArc = 9;
  const lip = makeGrid("cowl_lip", nArc, nj, () => [0, 0, 0]);
  for (let j = 0; j < nj; j++) {
    const p0 = gridPoint(cowl, 0, j);
    const C: Vec3 = [p0[0] + R, p0[1], p0[2]];
    for (let k = 0; k < nArc; k++) {
      const phi = (Math.PI / 2) * (1 - k / (nArc - 1));
      setGridPoint(lip, k, j, [C[0] - R * Math.sin(phi), p0[1], C[2] + R * (1 - Math.cos(phi))]);
    }
    setGridPoint(lip, nArc - 1, j, p0);
  }
  return lip;
}
