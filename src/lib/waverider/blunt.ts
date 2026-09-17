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

const N_ARC = 13;
const ALPHA_MIN = 3.5 * (Math.PI / 180);

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

function tangentAt(pts: Vec3[], from = 0): Vec3 {
  for (let i = from; i < pts.length - 1; i++) {
    const t = vsub(pts[i + 1], pts[i]);
    if (vlen(t) > 1e-12) return vnorm(t);
  }
  return [1, 0, 0];
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

function resample(pts: Vec3[], n: number, power = 1.4): Vec3[] {
  if (pts.length === 0) return pts;
  if (n <= 1) return [pts[0]];
  const acc: number[] = [0];
  for (let i = 1; i < pts.length; i++) acc.push(acc[i - 1] + vlen(vsub(pts[i], pts[i - 1])));
  const total = acc[acc.length - 1];
  if (total < 1e-15) return Array.from({ length: n }, () => pts[0]);
  const out: Vec3[] = [];
  for (let i = 0; i < n; i++) {
    const u = n <= 1 ? 0 : i / (n - 1);
    const s = Math.pow(u, power) * total;
    let k = 0;
    while (k < acc.length - 2 && acc[k + 1] < s) k++;
    const span = acc[k + 1] - acc[k];
    const t = span > 1e-15 ? (s - acc[k]) / span : 0;
    out.push(vlerp(pts[k], pts[k + 1], clamp(t, 0, 1)));
  }
  out[0] = pts[0];
  out[n - 1] = pts[pts.length - 1];
  return out;
}

function slerpUnit(a: Vec3, b: Vec3, t: number): Vec3 {
  const d = clamp(vdot(a, b), -1, 1);
  const om = Math.acos(d);
  if (om < 1e-8) return vnorm(vadd(a, vscale(vsub(b, a), t)));
  const s = Math.sin(om);
  return vnorm(vadd(vscale(a, Math.sin((1 - t) * om) / s), vscale(b, Math.sin(t * om) / s)));
}

type Station = { ok: boolean; arc: Vec3[]; tu: Vec3; tl: Vec3; uRow: Vec3[] | null; lRow: Vec3[] | null };

function planar(v: Vec3, y: number): Vec3 {
  return [v[0], y, v[2]];
}

function xzNorm(v: Vec3): Vec3 {
  const n = Math.hypot(v[0], v[2]);
  if (n < 1e-15) return [1, 0, 0];
  return [v[0] / n, 0, v[2] / n];
}

function lockY(st: Station, y: number) {
  for (const p of st.arc) p[1] = y;
  st.tu[1] = y;
  st.tl[1] = y;
  if (st.uRow) st.uRow[0][1] = y;
  if (st.lRow) st.lRow[0][1] = y;
}

function openTangents(tU: Vec3, tL: Vec3): { tU: Vec3; tL: Vec3; alpha: number } {
  tU = xzNorm(tU);
  tL = xzNorm(tL);
  let alpha = Math.acos(clamp(vdot(tU, tL), -0.999, 0.999));
  if (alpha >= ALPHA_MIN) return { tU, tL, alpha };
  let bis = vadd(tU, tL);
  if (vlen(bis) < 0.2) bis = [1, 0, 0];
  bis = xzNorm(bis);
  let thick: Vec3 = [0, 0, 1];
  if (Math.abs(bis[2]) > 0.92) thick = [1, 0, 0];
  thick = xzNorm(vcross([0, 1, 0], bis));
  if (vlen(thick) < 0.2) thick = [0, 0, 1];
  const h = ALPHA_MIN / 2;
  return {
    tU: xzNorm(vadd(vscale(bis, Math.cos(h)), vscale(thick, Math.sin(h)))),
    tL: xzNorm(vadd(vscale(bis, Math.cos(h)), vscale(thick, -Math.sin(h)))),
    alpha: ALPHA_MIN,
  };
}

function stationFillet(U: Vec3[], Lo: Vec3[], radius: number, L: number): Station {
  const y = U[0][1];
  const P = planar(vlerp(U[0], Lo[0], 0.5), y);
  const opening = vlen(vsub(U[0], Lo[0]));
  const fail = (p: Vec3): Station => ({
    ok: false,
    arc: Array.from({ length: N_ARC }, () => planar(p, y)),
    tu: planar(p, y),
    tl: planar(p, y),
    uRow: U.map((q) => [q[0], q[1], q[2]] as Vec3),
    lRow: Lo.map((q) => [q[0], q[1], q[2]] as Vec3),
  });
  if (opening > 1.6 * Math.max(radius, 1e-9)) return fail(P);
  const opened = openTangents(tangentAt(U), tangentAt(Lo));
  const tU = opened.tU;
  const tL = opened.tL;
  const alpha = opened.alpha;
  const half = 0.5 * alpha;
  const chord = Math.max(chordLen(U), chordLen(Lo), 1e-9);
  let R = Math.min(radius, 0.08 * L, 0.28 * chord * Math.tan(half));
  const tProbe = Math.min(0.16 * chord, Math.max(4 * radius, 1e-6));
  const tLoc = vlen(vsub(pointAtLength(U, tProbe).p, pointAtLength(Lo, tProbe).p));
  R = Math.min(R, 0.42 * tLoc, 0.12 * chord);
  if (R < 2e-5 * L && chord > 4e-4 * L) R = Math.min(2e-5 * L, 0.18 * chord);
  if (!(R > 1e-12)) return fail(P);
  let sUse = R / Math.tan(half);
  sUse = Math.min(sUse, 2.4 * R, 0.1 * chord);
  R = sUse * Math.tan(half);
  const d = R / Math.sin(half);
  const bis = xzNorm(vadd(tU, tL));
  if (vlen(bis) < 0.2) return fail(P);
  const C = planar(vadd(P, vscale(bis, d)), y);
  const uCut = pointAtLength(U, sUse);
  const lCut = pointAtLength(Lo, sUse);
  let TU = planar(uCut.p, y);
  let TL = planar(lCut.p, y);
  if (vlen(vsub(TU, TL)) < 4e-4 * L) {
    TU = planar(vadd(P, vscale(tU, sUse)), y);
    TL = planar(vadd(P, vscale(tL, sUse)), y);
  }
  const F = planar(vsub(C, vscale(bis, R)), y);
  const aTL = xzNorm(vsub(TL, C));
  const aTU = xzNorm(vsub(TU, C));
  const aF = xzNorm(vsub(F, C));
  if (vlen(aTL) < 0.5 || vlen(aTU) < 0.5 || vlen(aF) < 0.5) return fail(P);
  const arc: Vec3[] = [];
  for (let k = 0; k < N_ARC; k++) {
    const t = k / (N_ARC - 1);
    const dir = t <= 0.5 ? slerpUnit(aTL, aF, t * 2) : slerpUnit(aF, aTU, t * 2 - 1);
    arc.push(planar(vadd(C, vscale(xzNorm(dir), R)), y));
  }
  arc[0] = TL;
  arc[N_ARC - 1] = TU;
  const uRest = [TU, ...U.slice(Math.max(uCut.k, 1))];
  const lRest = [TL, ...Lo.slice(Math.max(lCut.k, 1))];
  const st: Station = {
    ok: true,
    arc,
    tu: TU,
    tl: TL,
    uRow: resample(uRest, U.length, 1.45),
    lRow: resample(lRest, Lo.length, 1.45),
  };
  lockY(st, y);
  return st;
}

function lerpStation(a: Station, b: Station, t: number): Station {
  const arc = a.arc.map((p, k) => vlerp(p, b.arc[k], t));
  const tu = vlerp(a.tu, b.tu, t);
  const tl = vlerp(a.tl, b.tl, t);
  arc[0] = tl;
  arc[arc.length - 1] = tu;
  const uRow = a.uRow && b.uRow ? a.uRow.map((p, i) => vlerp(p, b.uRow![i], t)) : a.uRow ?? b.uRow;
  const lRow = a.lRow && b.lRow ? a.lRow.map((p, i) => vlerp(p, b.lRow![i], t)) : a.lRow ?? b.lRow;
  if (uRow) uRow[0] = tu;
  if (lRow) lRow[0] = tl;
  return { ok: true, arc, tu, tl, uRow: uRow ?? null, lRow: lRow ?? null };
}

/**
 * Per-span circular fillet. Returns a leading-edge surface
 * (i along the arc, j along span) or null if R = 0.
 */
export function applyLeadingFillet(
  upper: SurfaceGrid,
  lower: SurfaceGrid,
  radius: number,
  length: number,
): SurfaceGrid | null {
  if (!(radius > 1e-9)) return null;
  const L = Math.max(length, 1e-6);
  const nj = Math.min(upper.nj, lower.nj);
  const nArc = N_ARC;
  const stations: Station[] = [];
  const yj: number[] = [];
  for (let j = 0; j < nj; j++) {
    const U = rowOf(upper, j);
    yj.push(U[0][1]);
    stations.push(stationFillet(U, rowOf(lower, j), radius, L));
  }

  const nOk = stations.filter((s) => s.ok).length;
  if (nOk < 3) return null;

  for (let j = 0; j < nj; j++) {
    if (stations[j].ok) continue;
    const keepU = stations[j].uRow;
    const keepL = stations[j].lRow;
    let lo = j - 1;
    while (lo >= 0 && !stations[lo].ok) lo--;
    let hi = j + 1;
    while (hi < nj && !stations[hi].ok) hi++;
    if (lo >= 0 && hi < nj) {
      stations[j] = lerpStation(stations[lo], stations[hi], (j - lo) / Math.max(hi - lo, 1));
    } else if (lo >= 0) {
      stations[j] = lerpStation(stations[lo], stations[lo], 0);
    } else if (hi < nj) {
      stations[j] = lerpStation(stations[hi], stations[hi], 0);
    }
    stations[j].uRow = keepU;
    stations[j].lRow = keepL;
    if (keepU) keepU[0] = stations[j].tu;
    if (keepL) keepL[0] = stations[j].tl;
  }

  const lead = makeGrid("leading", nArc, nj, (i, j) => stations[j].arc[i] ?? stations[j].tu);
  for (let j = 0; j < nj; j++) {
    const st = stations[j];
    lockY(st, yj[j]);
    if (st.uRow) writeRow(upper, j, st.uRow);
    if (st.lRow) writeRow(lower, j, st.lRow);
    setGridPoint(upper, 0, j, st.tu);
    setGridPoint(lower, 0, j, st.tl);
    for (let k = 0; k < nArc; k++) setGridPoint(lead, k, j, st.arc[k]);
    setGridPoint(lead, 0, j, st.tl);
    setGridPoint(lead, nArc - 1, j, st.tu);
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
