/**
 * Mesh robustness helpers for the refinement work (new file so mesh.ts stays untouched):
 *  - edge-defect counting (open / non-manifold) with typed arrays,
 *  - duplicate / zero-volume triangle-pair removal,
 *  - tolerance-aware aft welding: try the legacy tolerance first (so every mesh that is already
 *    closed and manifold is returned unchanged), and only shrink it when the result is not.
 *  - triangle quality (min angle, aspect ratio, sliver count) for tests and diagnostics.
 */
import { weldAft } from "./mesh";
import type { TriMesh } from "./types";

export interface EdgeDefects {
  /** Edges used by exactly one triangle. */
  open: number;
  /** Edges used by three or more triangles. */
  nonManifold: number;
}

/** Edge-use counts via a sorted numeric key (no string allocation). */
export function edgeDefects(mesh: TriMesh): EdgeDefects {
  const nt = mesh.indices.length / 3;
  const nv = Math.max(mesh.positions.length / 3, 1);
  const keys = new Float64Array(nt * 3);
  for (let t = 0; t < nt; t++) {
    for (let k = 0; k < 3; k++) {
      const a = mesh.indices[t * 3 + k];
      const b = mesh.indices[t * 3 + ((k + 1) % 3)];
      keys[t * 3 + k] = a < b ? a * nv + b : b * nv + a;
    }
  }
  keys.sort();
  let open = 0;
  let nonManifold = 0;
  for (let i = 0; i < keys.length; ) {
    let j = i + 1;
    while (j < keys.length && keys[j] === keys[i]) j++;
    const c = j - i;
    if (c === 1) open++;
    else if (c > 2) nonManifold++;
    i = j;
  }
  return { open, nonManifold };
}

/**
 * Remove triangle pairs that occupy the same three vertices with opposite orientation (a
 * zero-thickness pocket left behind when two sheets are welded together) and exact duplicates.
 * Only whole pairs are removed, so a closed mesh stays closed.
 */
export function removeCollapsedPairs(mesh: TriMesh): TriMesh {
  const nt = mesh.indices.length / 3;
  const nv = Math.max(mesh.positions.length / 3, 1);
  const seen = new Map<number, number[]>();
  const key = (a: number, b: number, c: number) => {
    const s = [a, b, c].sort((x, y) => x - y);
    return (s[0] * nv + s[1]) * nv + s[2];
  };
  for (let t = 0; t < nt; t++) {
    const k = key(mesh.indices[t * 3], mesh.indices[t * 3 + 1], mesh.indices[t * 3 + 2]);
    const list = seen.get(k);
    if (list) list.push(t);
    else seen.set(k, [t]);
  }
  const drop = new Uint8Array(nt);
  let any = false;
  for (const list of seen.values()) {
    if (list.length < 2) continue;
    const pairs = list.length - (list.length % 2);
    for (let i = 0; i < pairs; i++) drop[list[i]] = 1;
    any = true;
  }
  if (!any) return mesh;
  const idx: number[] = [];
  const surf: number[] = [];
  const tags: number[] = [];
  for (let t = 0; t < nt; t++) {
    if (drop[t]) continue;
    idx.push(mesh.indices[t * 3], mesh.indices[t * 3 + 1], mesh.indices[t * 3 + 2]);
    surf.push(mesh.surfaces[t] ?? 0);
    if (mesh.tags) tags.push(mesh.tags[t] ?? -1);
  }
  return {
    positions: mesh.positions,
    indices: Uint32Array.from(idx),
    surfaces: Uint8Array.from(surf),
    ...(mesh.tags ? { tags: Int32Array.from(tags) } : {}),
  };
}

/** Candidate weld tolerances as fractions of the legacy 8e-4 L (1 first: unchanged behaviour). */
export const WELD_TOL_LADDER = [1, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625, 0.0078125, 0.00390625];

export interface WeldReport {
  /** Multiplier of the legacy tolerance that was finally used. */
  scale: number;
  defects: EdgeDefects;
  /** Number of candidates evaluated (1 = legacy tolerance was already fine). */
  tried: number;
}

/**
 * Drop-in replacement for `weldAft(mesh, length)`. The legacy tolerance (8e-4 L) is evaluated
 * first; if the welded mesh has open or non-manifold edges, smaller tolerances (and the
 * collapsed-pair clean-up) are tried and the first defect-free result wins, otherwise the one
 * with the fewest defects. Meshes that are already closed + manifold come back bit-identical.
 */
export function weldAftRobust(mesh: TriMesh, length: number, report?: (r: WeldReport) => void): TriMesh {
  const L = Math.max(length, 1e-6);
  const defectsOf = (m: TriMesh) => {
    const d = edgeDefects(m);
    return { d, score: d.open + d.nonManifold };
  };
  const base = weldAft(mesh, length);
  let best = base;
  let bestScore = defectsOf(base);
  let bestScale = 1;
  let tried = 1;
  if (bestScore.score > 0) {
    for (let i = 0; i < WELD_TOL_LADDER.length && bestScore.score > 0; i++) {
      const scale = WELD_TOL_LADDER[i];
      for (const clean of [false, true]) {
        const w = i === 0 && !clean ? base : weldAft(mesh, length, 8e-4 * L * scale);
        const cand = clean ? removeCollapsedPairs(w) : w;
        if (i === 0 && !clean) continue;
        tried++;
        const s = defectsOf(cand);
        if (s.score < bestScore.score) {
          best = cand;
          bestScore = s;
          bestScale = scale;
        }
        if (bestScore.score === 0) break;
      }
    }
  }
  report?.({ scale: bestScale, defects: bestScore.d, tried });
  return best;
}

export interface TriangleQuality {
  triangles: number;
  /** Smallest interior angle over all triangles (degrees). */
  minAngleDeg: number;
  /** Largest edge-length / (2 sqrt(3) inradius) ratio (1 = equilateral). */
  maxAspect: number;
  /** Triangles with min angle < sliverDeg or area < areaTol. */
  slivers: number;
  /** Zero-area triangles (area <= 1e-18 L^2). */
  degenerate: number;
  /** Median of the per-triangle min angle (degrees). */
  medianMinAngleDeg: number;
}

export function triangleQuality(mesh: TriMesh, length = 1, sliverDeg = 1): TriangleQuality {
  const nt = mesh.indices.length / 3;
  const p = mesh.positions;
  const areaTol = 1e-18 * length * length;
  let minAng = 180;
  let maxAspect = 1;
  let slivers = 0;
  let degenerate = 0;
  const mins = new Float64Array(nt);
  for (let t = 0; t < nt; t++) {
    const a = mesh.indices[t * 3] * 3;
    const b = mesh.indices[t * 3 + 1] * 3;
    const c = mesh.indices[t * 3 + 2] * 3;
    const ab = Math.hypot(p[b] - p[a], p[b + 1] - p[a + 1], p[b + 2] - p[a + 2]);
    const bc = Math.hypot(p[c] - p[b], p[c + 1] - p[b + 1], p[c + 2] - p[b + 2]);
    const ca = Math.hypot(p[a] - p[c], p[a + 1] - p[c + 1], p[a + 2] - p[c + 2]);
    const ux = p[b] - p[a];
    const uy = p[b + 1] - p[a + 1];
    const uz = p[b + 2] - p[a + 2];
    const vx = p[c] - p[a];
    const vy = p[c + 1] - p[a + 1];
    const vz = p[c + 2] - p[a + 2];
    const area = 0.5 * Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
    if (area <= areaTol) {
      degenerate++;
      slivers++;
      mins[t] = 0;
      minAng = 0;
      continue;
    }
    const longest = Math.max(ab, bc, ca);
    const shortest = Math.min(ab, bc, ca);
    // min angle is opposite the shortest edge: sin(A) = 2 area / (e1 e2)
    const other = ab + bc + ca - longest - shortest;
    const sinMin = Math.min(1, (2 * area) / Math.max(longest * other, 1e-300));
    let ang = (Math.asin(sinMin) * 180) / Math.PI;
    // asin returns the acute branch; the min angle is always acute, so this is exact.
    if (!Number.isFinite(ang)) ang = 0;
    mins[t] = ang;
    if (ang < minAng) minAng = ang;
    const s = 0.5 * (ab + bc + ca);
    const inr = area / s;
    const asp = longest / (2 * Math.sqrt(3) * inr);
    if (asp > maxAspect) maxAspect = asp;
    if (ang < sliverDeg) slivers++;
  }
  const sorted = mins.slice().sort();
  return {
    triangles: nt,
    minAngleDeg: nt ? minAng : 0,
    maxAspect,
    slivers,
    degenerate,
    medianMinAngleDeg: nt ? sorted[Math.floor(nt / 2)] : 0,
  };
}

/**
 * Safety net for pipelines that already close the aft tips at grid level (topology tidy): returns the mesh
 * unchanged (same object) when it is closed and manifold, otherwise tries `weldAftRobust` and keeps its
 * result only if it has strictly fewer defects than the input.
 */
export function repairAftIfDefective(mesh: TriMesh, length: number): TriMesh {
  const d0 = edgeDefects(mesh);
  const s0 = d0.open + d0.nonManifold;
  if (s0 === 0) return mesh;
  const welded = weldAftRobust(mesh, length);
  const d1 = edgeDefects(welded);
  return d1.open + d1.nonManifold < s0 ? welded : mesh;
}
