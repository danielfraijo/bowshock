import type { MeshQuality, SurfaceGrid, SurfaceKind, TriMesh, Vec3 } from "./types";
import { SURFACE_ID } from "./types";
import { vcross, vdot, vlen, vsub } from "./math";

function dist2(pos: number[], a: number, b: number) {
  const ax = pos[a * 3] - pos[b * 3];
  const ay = pos[a * 3 + 1] - pos[b * 3 + 1];
  const az = pos[a * 3 + 2] - pos[b * 3 + 2];
  return ax * ax + ay * ay + az * az;
}

export class MeshBuilder {
  private keyToIndex = new Map<string, number>();
  private pos: number[] = [];
  private idx: number[] = [];
  private surf: number[] = [];
  skipped = 0;
  private q: number;
  private areaMin: number;

  constructor(length = 1) {
    const L = Math.max(length, 1e-6);
    // ~10 nm relative weld — coincident LE/TE zipper verts merge; distinct grid pts do not.
    this.q = 1e8 / L;
    this.areaMin = 1e-18 * L * L;
  }

  vert(x: number, y: number, z: number): number {
    const key = `${Math.round(x * this.q)}:${Math.round(y * this.q)}:${Math.round(z * this.q)}`;
    const existing = this.keyToIndex.get(key);
    if (existing !== undefined) return existing;
    const i = this.pos.length / 3;
    this.keyToIndex.set(key, i);
    this.pos.push(x, y, z);
    return i;
  }

  vertP(p: Vec3): number {
    return this.vert(p[0], p[1], p[2]);
  }

  tri(a: number, b: number, c: number, surface: SurfaceKind) {
    if (a === b || b === c || c === a) {
      this.skipped++;
      return;
    }
    const ax = this.pos[a * 3];
    const ay = this.pos[a * 3 + 1];
    const az = this.pos[a * 3 + 2];
    const bx = this.pos[b * 3];
    const by = this.pos[b * 3 + 1];
    const bz = this.pos[b * 3 + 2];
    const cx = this.pos[c * 3];
    const cy = this.pos[c * 3 + 1];
    const cz = this.pos[c * 3 + 2];
    const abx = bx - ax;
    const aby = by - ay;
    const abz = bz - az;
    const acx = cx - ax;
    const acy = cy - ay;
    const acz = cz - az;
    const nx = aby * acz - abz * acy;
    const ny = abz * acx - abx * acz;
    const nz = abx * acy - aby * acx;
    const mag = Math.hypot(nx, ny, nz);
    if (mag < this.areaMin) {
      this.skipped++;
      return;
    }
    this.idx.push(a, b, c);
    this.surf.push(SURFACE_ID[surface]);
  }

  /** Split on the shorter diagonal so sliver quads do not fan into hanging spikes. */
  quad(a: number, b: number, c: number, d: number, surface: SurfaceKind) {
    if (a === b && b === c && c === d) {
      this.skipped++;
      return;
    }
    const ac = dist2(this.pos, a, c);
    const bd = dist2(this.pos, b, d);
    if (ac <= bd) {
      this.tri(a, b, c, surface);
      this.tri(a, c, d, surface);
    } else {
      this.tri(a, b, d, surface);
      this.tri(b, c, d, surface);
    }
  }

  fan(loop: number[], surface: SurfaceKind, reverse = false) {
    if (loop.length < 3) return;
    const pts = reverse ? [...loop].reverse() : loop;
    const c = pts[0];
    for (let i = 1; i < pts.length - 1; i++) this.tri(c, pts[i], pts[i + 1], surface);
  }

  finish(): TriMesh {
    const mesh: TriMesh = {
      positions: Float64Array.from(this.pos),
      indices: Uint32Array.from(this.idx),
      surfaces: Uint8Array.from(this.surf),
    };
    orientOutward(mesh);
    return mesh;
  }
}

function centroid(mesh: TriMesh): Vec3 {
  const nv = mesh.positions.length / 3;
  let x = 0;
  let y = 0;
  let z = 0;
  for (let i = 0; i < nv; i++) {
    x += mesh.positions[i * 3];
    y += mesh.positions[i * 3 + 1];
    z += mesh.positions[i * 3 + 2];
  }
  return [x / Math.max(nv, 1), y / Math.max(nv, 1), z / Math.max(nv, 1)];
}

export function orientOutward(mesh: TriMesh) {
  const c = centroid(mesh);
  const nt = mesh.indices.length / 3;
  let zUpper = 0;
  let nUpper = 0;
  let zLower = 0;
  let nLower = 0;
  for (let t = 0; t < nt; t++) {
    const a = mesh.indices[t * 3];
    const b = mesh.indices[t * 3 + 1];
    const ic = mesh.indices[t * 3 + 2];
    const mz =
      (mesh.positions[a * 3 + 2] + mesh.positions[b * 3 + 2] + mesh.positions[ic * 3 + 2]) / 3;
    if (mesh.surfaces[t] === SURFACE_ID.upper) {
      zUpper += mz;
      nUpper++;
    } else if (mesh.surfaces[t] === SURFACE_ID.lower) {
      zLower += mz;
      nLower++;
    }
  }
  const meanU = nUpper ? zUpper / nUpper : 1;
  const meanL = nLower ? zLower / nLower : -1;
  const lowerIsDown = meanL <= meanU;

  for (let t = 0; t < nt; t++) {
    const a = mesh.indices[t * 3];
    const b = mesh.indices[t * 3 + 1];
    const ic = mesh.indices[t * 3 + 2];
    const ax = mesh.positions[a * 3];
    const ay = mesh.positions[a * 3 + 1];
    const az = mesh.positions[a * 3 + 2];
    const bx = mesh.positions[b * 3];
    const by = mesh.positions[b * 3 + 1];
    const bz = mesh.positions[b * 3 + 2];
    const cx = mesh.positions[ic * 3];
    const cy = mesh.positions[ic * 3 + 1];
    const cz = mesh.positions[ic * 3 + 2];
    const abx = bx - ax;
    const aby = by - ay;
    const abz = bz - az;
    const acx = cx - ax;
    const acy = cy - ay;
    const acz = cz - az;
    const nx = aby * acz - abz * acy;
    const ny = abz * acx - abx * acz;
    const nz = abx * acy - aby * acx;
    const mx = (ax + bx + cx) / 3 - c[0];
    const my = (ay + by + cy) / 3 - c[1];
    const mz = (az + bz + cz) / 3 - c[2];
    const surf = mesh.surfaces[t] ?? 0;
    let flip = nx * mx + ny * my + nz * mz < 0;
    if (surf === SURFACE_ID.base || surf === SURFACE_ID.nozzle) flip = nx < 0;
    else if (surf === SURFACE_ID.leading || surf === SURFACE_ID.inlet) flip = nx > 0;
    else if (surf === SURFACE_ID.lower) flip = lowerIsDown ? nz > 0 : nz < 0;
    else if (surf === SURFACE_ID.upper) flip = lowerIsDown ? nz < 0 : nz > 0;
    if (flip) {
      mesh.indices[t * 3 + 1] = ic;
      mesh.indices[t * 3 + 2] = b;
    }
  }
}

export function gridPoint(g: SurfaceGrid, i: number, j: number): Vec3 {
  const o = (i * g.nj + j) * 3;
  return [g.xyz[o], g.xyz[o + 1], g.xyz[o + 2]];
}

export function setGridPoint(g: SurfaceGrid, i: number, j: number, p: Vec3) {
  const o = (i * g.nj + j) * 3;
  g.xyz[o] = p[0];
  g.xyz[o + 1] = p[1];
  g.xyz[o + 2] = p[2];
}

export function makeGrid(name: string, ni: number, nj: number, sample: (i: number, j: number) => Vec3): SurfaceGrid {
  const xyz = new Float64Array(ni * nj * 3);
  for (let i = 0; i < ni; i++) {
    for (let j = 0; j < nj; j++) {
      const p = sample(i, j);
      const o = (i * nj + j) * 3;
      xyz[o] = p[0];
      xyz[o + 1] = p[1];
      xyz[o + 2] = p[2];
    }
  }
  return { name, ni, nj, xyz };
}

export function stitchGrid(b: MeshBuilder, g: SurfaceGrid, surface: SurfaceKind, flip = false) {
  for (let i = 0; i < g.ni - 1; i++) {
    for (let j = 0; j < g.nj - 1; j++) {
      const a = b.vertP(gridPoint(g, i, j));
      const c1 = b.vertP(gridPoint(g, i + 1, j));
      const c2 = b.vertP(gridPoint(g, i + 1, j + 1));
      const d = b.vertP(gridPoint(g, i, j + 1));
      if (flip) b.quad(a, d, c2, c1, surface);
      else b.quad(a, c1, c2, d, surface);
    }
  }
}

/**
 * Snap already-sharp seams to bitwise-identical vertices and collapse
 * zero-chord stations (delta tips) to a point. Does not flatten blunt
 * noses or open inlets — those have a real LE gap.
 */
export function zipperSharpEdges(upper: SurfaceGrid, lower: SurfaceGrid, length: number) {
  const L = Math.max(length, 1e-6);
  const eps = 1e-7 * L;
  const ni = Math.min(upper.ni, lower.ni);
  const nj = Math.min(upper.nj, lower.nj);
  const dist = (a: Vec3, b: Vec3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  const mid = (a: Vec3, b: Vec3): Vec3 => [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5];
  for (let j = 0; j < nj; j++) {
    const uLE = gridPoint(upper, 0, j);
    const lLE = gridPoint(lower, 0, j);
    if (dist(uLE, lLE) <= eps) {
      const m = mid(uLE, lLE);
      setGridPoint(upper, 0, j, m);
      setGridPoint(lower, 0, j, m);
    }
    const uTE = gridPoint(upper, ni - 1, j);
    const chord = dist(gridPoint(upper, 0, j), uTE);
    if (chord <= 10 * eps) {
      const p = gridPoint(upper, 0, j);
      for (let i = 0; i < ni; i++) {
        setGridPoint(upper, i, j, p);
        setGridPoint(lower, i, j, p);
      }
      continue;
    }
    for (let i = 0; i < ni; i++) {
      const u = gridPoint(upper, i, j);
      const l = gridPoint(lower, i, j);
      if (dist(u, l) <= eps) {
        const m = mid(u, l);
        setGridPoint(upper, i, j, m);
        setGridPoint(lower, i, j, m);
      }
    }
  }
}

export function analyzeMesh(mesh: TriMesh, skipped = 0): MeshQuality {
  const nv = mesh.positions.length / 3;
  const nt = mesh.indices.length / 3;
  const edgeCount = new Map<string, number>();
  let volume = 0;
  let area = 0;
  let minEdge = Infinity;
  let maxEdge = 0;
  const min: Vec3 = [Infinity, Infinity, Infinity];
  const max: Vec3 = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < nv; i++) {
    const x = mesh.positions[i * 3];
    const y = mesh.positions[i * 3 + 1];
    const z = mesh.positions[i * 3 + 2];
    if (x < min[0]) min[0] = x;
    if (y < min[1]) min[1] = y;
    if (z < min[2]) min[2] = z;
    if (x > max[0]) max[0] = x;
    if (y > max[1]) max[1] = y;
    if (z > max[2]) max[2] = z;
  }
  const get = (i: number): Vec3 => [
    mesh.positions[i * 3],
    mesh.positions[i * 3 + 1],
    mesh.positions[i * 3 + 2],
  ];
  const addEdge = (a: number, b: number) => {
    const i0 = Math.min(a, b);
    const i1 = Math.max(a, b);
    const k = `${i0}:${i1}`;
    edgeCount.set(k, (edgeCount.get(k) ?? 0) + 1);
  };
  for (let t = 0; t < nt; t++) {
    const a = mesh.indices[t * 3];
    const b = mesh.indices[t * 3 + 1];
    const c = mesh.indices[t * 3 + 2];
    const pa = get(a);
    const pb = get(b);
    const pc = get(c);
    const ab = vsub(pb, pa);
    const ac = vsub(pc, pa);
    const n = vcross(ab, ac);
    const mag = vlen(n);
    area += 0.5 * mag;
    volume += vdot(pa, n) / 6;
    const e1 = vlen(ab);
    const e2 = vlen(vsub(pc, pb));
    const e3 = vlen(vsub(pa, pc));
    minEdge = Math.min(minEdge, e1, e2, e3);
    maxEdge = Math.max(maxEdge, e1, e2, e3);
    addEdge(a, b);
    addEdge(b, c);
    addEdge(c, a);
  }
  let open = 0;
  let nonMan = 0;
  for (const n of edgeCount.values()) {
    if (n === 1) open++;
    else if (n !== 2) nonMan++;
  }
  if (volume < 0) volume = -volume;
  return {
    vertices: nv,
    triangles: nt,
    volume,
    area,
    watertight: open === 0 && nt > 0,
    manifold: open === 0 && nonMan === 0 && nt > 0,
    openEdges: open,
    nonManifoldEdges: nonMan,
    skippedDegenerate: skipped,
    minEdge: Number.isFinite(minEdge) ? minEdge : 0,
    maxEdge,
    bbox: { min, max },
  };
}

export function compactMesh(mesh: TriMesh): TriMesh {
  const nv = mesh.positions.length / 3;
  const used = new Uint8Array(nv);
  for (let t = 0; t < mesh.indices.length; t++) used[mesh.indices[t]] = 1;
  let keep = 0;
  for (let i = 0; i < nv; i++) if (used[i]) keep++;
  if (keep === nv) return mesh;
  const remap = new Int32Array(nv);
  const positions = new Float64Array(keep * 3);
  let w = 0;
  for (let i = 0; i < nv; i++) {
    if (!used[i]) {
      remap[i] = -1;
      continue;
    }
    remap[i] = w;
    positions[w * 3] = mesh.positions[i * 3];
    positions[w * 3 + 1] = mesh.positions[i * 3 + 1];
    positions[w * 3 + 2] = mesh.positions[i * 3 + 2];
    w++;
  }
  const indices = new Uint32Array(mesh.indices.length);
  for (let t = 0; t < mesh.indices.length; t++) indices[t] = remap[mesh.indices[t]];
  return { positions, indices, surfaces: mesh.surfaces };
}

/** Merge aft vertices closer than `tol`. Closes micro TE-tip slits without eating the LE fillet. */
export function weldAft(mesh: TriMesh, length: number, tol?: number): TriMesh {
  const L = Math.max(length, 1e-6);
  const t = tol ?? 8e-4 * L;
  const t2 = t * t;
  const nv = mesh.positions.length / 3;
  let yMax = 0;
  let xMax = 0;
  for (let i = 0; i < nv; i++) {
    const ax = Math.abs(mesh.positions[i * 3]);
    const ay = Math.abs(mesh.positions[i * 3 + 1]);
    if (ay > yMax) yMax = ay;
    if (ax > xMax) xMax = ax;
  }
  const xCut = xMax - Math.max(4e-3 * L, t * 4);
  const yCut = yMax * 0.9;
  const remap = new Int32Array(nv);
  for (let i = 0; i < nv; i++) remap[i] = i;
  const aft: number[] = [];
  for (let i = 0; i < nv; i++) {
    if (mesh.positions[i * 3] >= xCut && Math.abs(mesh.positions[i * 3 + 1]) >= yCut) aft.push(i);
  }
  for (let a = 0; a < aft.length; a++) {
    const i = aft[a];
    if (remap[i] !== i) continue;
    const ix = mesh.positions[i * 3];
    const iy = mesh.positions[i * 3 + 1];
    const iz = mesh.positions[i * 3 + 2];
    for (let b = a + 1; b < aft.length; b++) {
      const j = aft[b];
      if (remap[j] !== j) continue;
      const dx = mesh.positions[j * 3] - ix;
      const dy = mesh.positions[j * 3 + 1] - iy;
      const dz = mesh.positions[j * 3 + 2] - iz;
      if (dx * dx + dy * dy + dz * dz <= t2) remap[j] = i;
    }
  }
  const idx = new Uint32Array(mesh.indices.length);
  for (let k = 0; k < mesh.indices.length; k++) {
    let i = mesh.indices[k];
    while (remap[i] !== i) i = remap[i];
    idx[k] = i;
  }
  const keep: number[] = [];
  const keepS: number[] = [];
  const nt = idx.length / 3;
  for (let t = 0; t < nt; t++) {
    const a = idx[t * 3];
    const b = idx[t * 3 + 1];
    const c = idx[t * 3 + 2];
    if (a === b || b === c || c === a) continue;
    keep.push(a, b, c);
    keepS.push(mesh.surfaces[t] ?? 0);
  }
  const out: TriMesh = {
    positions: mesh.positions,
    indices: Uint32Array.from(keep),
    surfaces: Uint8Array.from(keepS),
  };
  return compactMesh(out);
}

export function scaleMesh(mesh: TriMesh, s: number): TriMesh {
  if (s === 1) return mesh;
  const positions = new Float64Array(mesh.positions.length);
  for (let i = 0; i < mesh.positions.length; i++) positions[i] = mesh.positions[i] * s;
  return { positions, indices: mesh.indices, surfaces: mesh.surfaces };
}

export function scaleGrids(grids: SurfaceGrid[], s: number): SurfaceGrid[] {
  if (s === 1) return grids;
  return grids.map((g) => {
    const xyz = new Float64Array(g.xyz.length);
    for (let i = 0; i < g.xyz.length; i++) xyz[i] = g.xyz[i] * s;
    return { ...g, xyz };
  });
}

/** Share TE (x, y) between the two sheets and spanwise-smooth the edge — kills nicks. */
export function matchTrailingEdge(upper: SurfaceGrid, lower: SurfaceGrid, length: number) {
  const niU = upper.ni;
  const niL = lower.ni;
  const nj = Math.min(upper.nj, lower.nj);
  if (niU < 2 || niL < 2 || nj < 2) return;
  const xs = new Float64Array(nj);
  const ys = new Float64Array(nj);
  const zU = new Float64Array(nj);
  const zL = new Float64Array(nj);
  for (let j = 0; j < nj; j++) {
    const u = gridPoint(upper, niU - 1, j);
    const l = gridPoint(lower, niL - 1, j);
    xs[j] = 0.5 * (u[0] + l[0]);
    ys[j] = 0.5 * (u[1] + l[1]);
    zU[j] = u[2];
    zL[j] = l[2];
  }
  for (let pass = 0; pass < 3; pass++) {
    const x2 = xs.slice();
    const y2 = ys.slice();
    for (let j = 1; j < nj - 1; j++) {
      x2[j] = 0.15 * xs[j - 1] + 0.7 * xs[j] + 0.15 * xs[j + 1];
      y2[j] = 0.15 * ys[j - 1] + 0.7 * ys[j] + 0.15 * ys[j + 1];
    }
    xs.set(x2);
    ys.set(y2);
  }
  for (let j = 0; j < nj; j++) {
    setGridPoint(upper, niU - 1, j, [xs[j], ys[j], zU[j]]);
    setGridPoint(lower, niL - 1, j, [xs[j], ys[j], zL[j]]);
  }
}

/**
 * Fair the trailing edge as a G0 curve, clamp x ≤ L, collapse only the tip
 * vertices. Stops the aft spike / indent that a min-chord pad used to leave.
 */
export function fairTrailingEdge(upper: SurfaceGrid, lower: SurfaceGrid, length: number) {
  const niU = upper.ni;
  const niL = lower.ni;
  const nj = Math.min(upper.nj, lower.nj);
  if (niU < 3 || niL < 3 || nj < 3) return;
  const L = Math.max(length, 1e-6);
  const xs = new Float64Array(nj);
  const ys = new Float64Array(nj);
  const zU = new Float64Array(nj);
  const zL = new Float64Array(nj);
  for (let j = 0; j < nj; j++) {
    const u = gridPoint(upper, niU - 1, j);
    const l = gridPoint(lower, niL - 1, j);
    xs[j] = 0.5 * (u[0] + l[0]);
    ys[j] = 0.5 * (u[1] + l[1]);
    zU[j] = u[2];
    zL[j] = l[2];
  }
  for (let pass = 0; pass < 2; pass++) {
    const x2 = xs.slice();
    for (let j = 1; j < nj - 1; j++) x2[j] = 0.2 * xs[j - 1] + 0.6 * xs[j] + 0.2 * xs[j + 1];
    xs.set(x2);
  }
  for (let j = 0; j < nj; j++) {
    const prevU = gridPoint(upper, niU - 2, j);
    xs[j] = clampF(xs[j], prevU[0] + 2e-6 * L, L);
    setGridPoint(upper, niU - 1, j, [xs[j], ys[j], zU[j]]);
    setGridPoint(lower, niL - 1, j, [xs[j], ys[j], zL[j]]);
  }
  for (const j of [0, nj - 1]) {
    const u = gridPoint(upper, niU - 1, j);
    const l = gridPoint(lower, niL - 1, j);
    const m: Vec3 = [0.5 * (u[0] + l[0]), 0.5 * (u[1] + l[1]), 0.5 * (u[2] + l[2])];
    m[0] = Math.min(m[0], L);
    setGridPoint(upper, niU - 1, j, m);
    setGridPoint(lower, niL - 1, j, m);
  }
}

/** Keep streamwise x monotone so TE quads cannot fold back on themselves. */
export function enforceMonotoneChord(g: SurfaceGrid) {
  for (let j = 0; j < g.nj; j++) {
    for (let i = 1; i < g.ni; i++) {
      const a = gridPoint(g, i - 1, j);
      const b = gridPoint(g, i, j);
      if (b[0] + 1e-12 < a[0]) setGridPoint(g, i, j, [a[0], b[1], b[2]]);
    }
  }
}

function lerp3(a: Vec3, b: Vec3, t: number): Vec3 {
  return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
}

function bilerp(g: SurfaceGrid, u: number, v: number): Vec3 {
  const iu = clampF(u, 0, g.ni - 1);
  const jv = clampF(v, 0, g.nj - 1);
  const i0 = Math.min(Math.floor(iu), g.ni - 2);
  const j0 = Math.min(Math.floor(jv), g.nj - 2);
  const su = smooth01(iu - i0);
  const sv = smooth01(jv - j0);
  const p00 = gridPoint(g, i0, j0);
  const p10 = gridPoint(g, i0 + 1, j0);
  const p01 = gridPoint(g, i0, j0 + 1);
  const p11 = gridPoint(g, i0 + 1, j0 + 1);
  const a = lerp3(p00, p10, su);
  const b = lerp3(p01, p11, su);
  return lerp3(a, b, sv);
}

function clampF(v: number, a: number, b: number) {
  return Math.max(a, Math.min(b, v));
}

function smooth01(t: number) {
  const u = clampF(t, 0, 1);
  return u * u * (3 - 2 * u);
}

/** Smoothstep-bilinear upsample. Use before IGES / STEP / Plot3D so Pointwise sees dense patches. */
export function refineGrid(g: SurfaceGrid, fi = 2, fj = 2): SurfaceGrid {
  const ni = (g.ni - 1) * Math.max(1, fi) + 1;
  const nj = (g.nj - 1) * Math.max(1, fj) + 1;
  if (ni === g.ni && nj === g.nj) return g;
  return makeGrid(g.name, ni, nj, (i, j) => {
    const u = (i / Math.max(ni - 1, 1)) * (g.ni - 1);
    const v = (j / Math.max(nj - 1, 1)) * (g.nj - 1);
    return bilerp(g, u, v);
  });
}

export function refineGrids(grids: SurfaceGrid[], fi = 2, fj = 2): SurfaceGrid[] {
  return grids.map((g) => {
    if (g.ni < 2 || g.nj < 2) return g;
    if (g.name === "leading" || g.name === "cowl_lip" || g.name.startsWith("tip") || g.name === "base") {
      return refineGrid(g, Math.max(1, fi), Math.max(1, fj));
    }
    return refineGrid(g, fi, fj);
  });
}

