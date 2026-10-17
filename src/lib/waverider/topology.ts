/**
 * Mesh topology repair / quality passes used by generate.ts.
 *   orientConsistent   — coherent winding by edge propagation + outward by signed volume
 *   flipForQuality     — constrained edge flips that raise the minimum angle inside a
 *                        smooth patch (same surface id, small dihedral). Vertices never move, so
 *                        watertightness, area, and volume (to second order) are preserved.
 */
import type { TriMesh } from "./types";

function triNormalRaw(p: Float64Array, a: number, b: number, c: number): [number, number, number] {
  const ax = p[a * 3], ay = p[a * 3 + 1], az = p[a * 3 + 2];
  const abx = p[b * 3] - ax, aby = p[b * 3 + 1] - ay, abz = p[b * 3 + 2] - az;
  const acx = p[c * 3] - ax, acy = p[c * 3 + 1] - ay, acz = p[c * 3 + 2] - az;
  return [aby * acz - abz * acy, abz * acx - abx * acz, abx * acy - aby * acx];
}

/** Smallest interior angle (radians) of triangle (a,b,c). 0 when degenerate. */
export function minAngleOf(p: Float64Array, a: number, b: number, c: number): number {
  const d = (i: number, j: number) => Math.hypot(p[i * 3] - p[j * 3], p[i * 3 + 1] - p[j * 3 + 1], p[i * 3 + 2] - p[j * 3 + 2]);
  const la = d(b, c), lb = d(c, a), lc = d(a, b);
  const n = triNormalRaw(p, a, b, c);
  const area2 = Math.hypot(n[0], n[1], n[2]);
  if (!(area2 > 0)) return 0;
  const ang = (x: number, y: number, z: number) => Math.acos(Math.max(-1, Math.min(1, (y * y + z * z - x * x) / (2 * y * z))));
  return Math.min(ang(la, lb, lc), ang(lb, lc, la), ang(lc, la, lb));
}

/**
 * Make triangle winding coherent. Every connected component (through manifold edges)
 * is propagated from a seed; a component whose signed volume is negative is flipped so
 * normals point outward. Open components (flow-through ducts) use the same volume sign
 * of the surface integral, which is the outward convention used for closed solids.
 * Returns the number of triangles that were reversed.
 */
export function orientConsistent(mesh: TriMesh): number {
  const nt = mesh.indices.length / 3;
  if (nt === 0) return 0;
  const nv = mesh.positions.length / 3;
  const K = nv + 1;
  const idx = mesh.indices;
  const edgeTris = new Map<number, number[]>();
  for (let t = 0; t < nt; t++) {
    for (let k = 0; k < 3; k++) {
      const u = idx[t * 3 + k];
      const v = idx[t * 3 + ((k + 1) % 3)];
      const key = Math.min(u, v) * K + Math.max(u, v);
      const a = edgeTris.get(key);
      if (a) a.push(t);
      else edgeTris.set(key, [t]);
    }
  }
  const comp = new Int32Array(nt).fill(-1);
  const flip = new Uint8Array(nt);
  let nComp = 0;
  const stack: number[] = [];
  const dirOf = (t: number, u: number, v: number): number => {
    for (let k = 0; k < 3; k++) {
      const a = idx[t * 3 + k];
      const b = idx[t * 3 + ((k + 1) % 3)];
      if (a === u && b === v) return flip[t] ? -1 : 1;
      if (a === v && b === u) return flip[t] ? 1 : -1;
    }
    return 0;
  };
  for (let s = 0; s < nt; s++) {
    if (comp[s] >= 0) continue;
    comp[s] = nComp;
    stack.push(s);
    while (stack.length) {
      const t = stack.pop()!;
      for (let k = 0; k < 3; k++) {
        const u = idx[t * 3 + k];
        const v = idx[t * 3 + ((k + 1) % 3)];
        const lst = edgeTris.get(Math.min(u, v) * K + Math.max(u, v))!;
        if (lst.length !== 2) continue; // only propagate through manifold edges
        const o = lst[0] === t ? lst[1] : lst[0];
        if (comp[o] >= 0) continue;
        // t uses u->v (after flips); o must use v->u
        const dt = dirOf(t, u, v);
        const dOraw = dirOf(o, u, v) * (flip[o] ? -1 : 1); // raw direction of o on u->v
        // want effective direction of o on u->v to be -dt
        flip[o] = dOraw === -dt ? 0 : 1;
        comp[o] = nComp;
        stack.push(o);
      }
    }
    nComp++;
  }
  // signed volume per component, in the (possibly flipped) orientation
  const vol = new Float64Array(nComp);
  const p = mesh.positions;
  for (let t = 0; t < nt; t++) {
    let a = idx[t * 3], b = idx[t * 3 + 1];
    const c = idx[t * 3 + 2];
    if (flip[t]) [a, b] = [b, a];
    const ax = p[a * 3], ay = p[a * 3 + 1], az = p[a * 3 + 2];
    const bx = p[b * 3], by = p[b * 3 + 1], bz = p[b * 3 + 2];
    const cx = p[c * 3], cy = p[c * 3 + 1], cz = p[c * 3 + 2];
    vol[comp[t]] += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6;
  }
  let changed = 0;
  for (let t = 0; t < nt; t++) {
    const f = flip[t] ^ (vol[comp[t]] < 0 ? 1 : 0);
    if (f) {
      const b = idx[t * 3 + 1];
      idx[t * 3 + 1] = idx[t * 3 + 2];
      idx[t * 3 + 2] = b;
      changed++;
    }
  }
  return changed;
}

export interface FlipOptions {
  /** Do not flip across edges whose two faces differ by more than this (degrees). */
  maxDihedralDeg?: number;
  /** A flip must raise the local minimum angle by at least this (degrees). */
  minGainDeg?: number;
  maxPasses?: number;
}

/**
 * Constrained edge flips. Only edges shared by two triangles of the same surface id
 * (and the same loft grid when tags exist) whose normals agree within maxDihedralDeg
 * are candidates, so creases, seams, fillet tangent lines and surface boundaries
 * are never touched. Returns the number of flips performed.
 */
export function flipForQuality(mesh: TriMesh, opt: FlipOptions = {}): number {
  const maxD = Math.cos(((opt.maxDihedralDeg ?? 12) * Math.PI) / 180);
  const gain = ((opt.minGainDeg ?? 0.5) * Math.PI) / 180;
  const passes = opt.maxPasses ?? 12;
  const nt = mesh.indices.length / 3;
  const nv = mesh.positions.length / 3;
  const K = nv + 1;
  const idx = mesh.indices;
  const P = mesh.positions;
  const tags = mesh.tags;
  const edges = new Map<number, number[]>();
  const addEdges = (t: number) => {
    for (let k = 0; k < 3; k++) {
      const u = idx[t * 3 + k];
      const v = idx[t * 3 + ((k + 1) % 3)];
      const key = Math.min(u, v) * K + Math.max(u, v);
      const a = edges.get(key);
      if (a) a.push(t);
      else edges.set(key, [t]);
    }
  };
  const removeEdge = (t: number, u: number, v: number) => {
    const key = Math.min(u, v) * K + Math.max(u, v);
    const a = edges.get(key);
    if (!a) return;
    const i = a.indexOf(t);
    if (i >= 0) a.splice(i, 1);
    if (!a.length) edges.delete(key);
  };
  for (let t = 0; t < nt; t++) addEdges(t);
  const gridOf = (t: number) => (tags && tags[t] >= 0 ? tags[t] >>> 24 : -1);
  let flips = 0;
  for (let pass = 0; pass < passes; pass++) {
    let did = 0;
    const keys = [...edges.keys()];
    for (const key of keys) {
      const lst = edges.get(key);
      if (!lst || lst.length !== 2) continue;
      const [t1, t2] = lst;
      if (mesh.surfaces[t1] !== mesh.surfaces[t2]) continue;
      if (tags && (tags[t1] < 0 || tags[t2] < 0 || gridOf(t1) !== gridOf(t2))) continue;
      const u = Math.floor(key / K);
      const v = key % K;
      const opp = (t: number) => {
        for (let k = 0; k < 3; k++) {
          const w = idx[t * 3 + k];
          if (w !== u && w !== v) return w;
        }
        return -1;
      };
      const a = opp(t1);
      const b = opp(t2);
      if (a < 0 || b < 0 || a === b) continue;
      // the new diagonal a-b must not already exist
      if (edges.has(Math.min(a, b) * K + Math.max(a, b))) continue;
      const n1 = triNormalRaw(P, idx[t1 * 3], idx[t1 * 3 + 1], idx[t1 * 3 + 2]);
      const n2 = triNormalRaw(P, idx[t2 * 3], idx[t2 * 3 + 1], idx[t2 * 3 + 2]);
      const m1 = Math.hypot(n1[0], n1[1], n1[2]);
      const m2 = Math.hypot(n2[0], n2[1], n2[2]);
      if (!(m1 > 0 && m2 > 0)) continue;
      const cosd = (n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]) / (m1 * m2);
      if (cosd < maxD) continue;
      // orientation of t1 as (x,y,z) cyclic with edge u->v direction
      let uu = u, vv = v;
      // find directed edge in t1
      let fwd = false;
      for (let k = 0; k < 3; k++) if (idx[t1 * 3 + k] === u && idx[t1 * 3 + ((k + 1) % 3)] === v) fwd = true;
      if (!fwd) [uu, vv] = [v, u];
      // t1 = (uu, vv, a); t2 = (vv, uu, b). New: (a, uu, b) and (a, b, vv)
      const nA = triNormalRaw(P, a, uu, b);
      const nB = triNormalRaw(P, a, b, vv);
      const mA = Math.hypot(nA[0], nA[1], nA[2]);
      const mB = Math.hypot(nB[0], nB[1], nB[2]);
      if (!(mA > 0 && mB > 0)) continue;
      const avg = [n1[0] / m1 + n2[0] / m2, n1[1] / m1 + n2[1] / m2, n1[2] / m1 + n2[2] / m2];
      const al = Math.hypot(avg[0], avg[1], avg[2]);
      const dA = (nA[0] * avg[0] + nA[1] * avg[1] + nA[2] * avg[2]) / (mA * al);
      const dB = (nB[0] * avg[0] + nB[1] * avg[1] + nB[2] * avg[2]) / (mB * al);
      if (dA < 0.9 || dB < 0.9) continue; // flip would fold (non-convex quad)
      const before = Math.min(minAngleOf(P, uu, vv, a), minAngleOf(P, vv, uu, b));
      const after = Math.min(minAngleOf(P, a, uu, b), minAngleOf(P, a, b, vv));
      if (after < before + gain) continue;
      // commit: t1 <- (a, uu, b); t2 <- (a, b, vv)
      for (const t of [t1, t2]) for (let k = 0; k < 3; k++) removeEdge(t, idx[t * 3 + k], idx[t * 3 + ((k + 1) % 3)]);
      idx[t1 * 3] = a; idx[t1 * 3 + 1] = uu; idx[t1 * 3 + 2] = b;
      idx[t2 * 3] = a; idx[t2 * 3 + 1] = b; idx[t2 * 3 + 2] = vv;
      addEdges(t1);
      addEdges(t2);
      did++;
    }
    flips += did;
    if (!did) break;
  }
  return flips;
}

// ---------------------------------------------------------------------------
// Trailing-edge tidy (grid level). Replaces matchTrailingEdge + fairTrailingEdge
// + weldAft in the generate pipeline: those moved y (shearing the last cell column
// at the tips) and welded vertices by *distance*, which pinched non-adjacent
// vertices together and produced non-manifold / folded tip triangles.
// ---------------------------------------------------------------------------
import type { SurfaceGrid } from "./types";

function gp(g: SurfaceGrid, i: number, j: number): [number, number, number] {
  const o = (i * g.nj + j) * 3;
  return [g.xyz[o], g.xyz[o + 1], g.xyz[o + 2]];
}
function sp(g: SurfaceGrid, i: number, j: number, p: [number, number, number]) {
  const o = (i * g.nj + j) * 3;
  g.xyz[o] = p[0];
  g.xyz[o + 1] = p[1];
  g.xyz[o + 2] = p[2];
}

/**
 * Share the TE x between upper and lower, fair it along the span (x only; y and z are
 * left alone so the last cell column keeps its shape), clamp x <= L, and zip the TE
 * of every station whose local thickness is below `thinTol` (and both tips) onto the
 * mid-surface. Zipping is pairwise by grid index, so it can never pinch unrelated verts.
 */
export function tidyTrailingEdge(upper: SurfaceGrid, lower: SurfaceGrid, length: number, thinTolRel = 0.0006) {
  const niU = upper.ni;
  const niL = lower.ni;
  const nj = Math.min(upper.nj, lower.nj);
  if (niU < 3 || niL < 3 || nj < 2) return;
  const L = Math.max(length, 1e-6);
  const xs = new Float64Array(nj);
  for (let j = 0; j < nj; j++) xs[j] = 0.5 * (gp(upper, niU - 1, j)[0] + gp(lower, niL - 1, j)[0]);
  if (nj >= 3) {
    for (let pass = 0; pass < 2; pass++) {
      const x2 = xs.slice();
      for (let j = 1; j < nj - 1; j++) x2[j] = 0.2 * xs[j - 1] + 0.6 * xs[j] + 0.2 * xs[j + 1];
      xs.set(x2);
    }
  }
  const thin = thinTolRel * L;
  for (let j = 0; j < nj; j++) {
    const pu = gp(upper, niU - 2, j);
    const pl = gp(lower, niL - 2, j);
    const x = Math.min(L, Math.max(xs[j], Math.max(pu[0], pl[0]) + 2e-6 * L));
    const u = gp(upper, niU - 1, j);
    const l = gp(lower, niL - 1, j);
    u[0] = x;
    l[0] = x;
    const tip = j === 0 || j === nj - 1;
    if (tip || Math.abs(u[2] - l[2]) < thin) {
      const m: [number, number, number] = [x, 0.5 * (u[1] + l[1]), 0.5 * (u[2] + l[2])];
      sp(upper, niU - 1, j, m);
      sp(lower, niL - 1, j, m);
    } else {
      sp(upper, niU - 1, j, u);
      sp(lower, niL - 1, j, l);
    }
  }
}

/**
 * Collapse the LE-fillet row at a wing tip that has degenerated to (almost) a point, together with the
 * upper/lower seam points that are tied to it. Without this, a millimetre-sized arc at the tip leaves
 * near-coincident but unwelded vertices from three grids, i.e. a tiny open hole.
 */
export function collapseTipRows(upper: SurfaceGrid, lower: SurfaceGrid, leading: SurfaceGrid | null, length: number, tolRel = 0.0008) {
  if (!leading) return;
  const nj = Math.min(upper.nj, lower.nj, leading.nj);
  const tol = tolRel * Math.max(length, 1e-6);
  for (const j of [0, nj - 1]) {
    const pts: [number, number, number][] = [];
    for (let i = 0; i < leading.ni; i++) pts.push(gp(leading, i, j));
    pts.push(gp(upper, 0, j), gp(lower, 0, j));
    const lo = [Infinity, Infinity, Infinity];
    const hi = [-Infinity, -Infinity, -Infinity];
    const c = [0, 0, 0];
    for (const q of pts)
      for (let k = 0; k < 3; k++) {
        lo[k] = Math.min(lo[k], q[k]);
        hi[k] = Math.max(hi[k], q[k]);
        c[k] += q[k];
      }
    if (Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) > tol) continue;
    const m: [number, number, number] = [c[0] / pts.length, c[1] / pts.length, c[2] / pts.length];
    for (let i = 0; i < leading.ni; i++) sp(leading, i, j, m);
    sp(upper, 0, j, m);
    sp(lower, 0, j, m);
  }
}

/**
 * Remove back-to-back duplicate faces: two triangles on the same three (welded) vertices. They appear where
 * the upper and lower sheets have been zipped together into a zero-thickness strip at a wing tip; leaving
 * both makes every edge of the strip shared by four faces (non-manifold). Removing both closes the knife.
 * Returns the same mesh when nothing was removed.
 */
export function removeDoubleFaces(mesh: TriMesh): TriMesh {
  const nt = mesh.indices.length / 3;
  const nv = mesh.positions.length / 3;
  const K = BigInt(nv + 1);
  const seen = new Map<bigint, number>();
  const kill = new Uint8Array(nt);
  let n = 0;
  for (let t = 0; t < nt; t++) {
    const v = [mesh.indices[t * 3], mesh.indices[t * 3 + 1], mesh.indices[t * 3 + 2]].sort((a, b) => a - b);
    const key = (BigInt(v[0]) * K + BigInt(v[1])) * K + BigInt(v[2]);
    const o = seen.get(key);
    if (o === undefined) seen.set(key, t);
    else if (o >= 0) {
      kill[o] = 1;
      kill[t] = 1;
      seen.set(key, -1);
      n += 2;
    } else {
      kill[t] = 1; // third copy: drop as well
      n++;
    }
  }
  if (!n) return mesh;
  const keep = nt - n;
  const indices = new Uint32Array(keep * 3);
  const surfaces = new Uint8Array(keep);
  const tags = mesh.tags ? new Int32Array(keep) : undefined;
  let w = 0;
  for (let t = 0; t < nt; t++) {
    if (kill[t]) continue;
    indices.set(mesh.indices.subarray(t * 3, t * 3 + 3), w * 3);
    surfaces[w] = mesh.surfaces[t];
    if (tags) tags[w] = mesh.tags![t];
    w++;
  }
  return { positions: mesh.positions, indices, surfaces, tags };
}
