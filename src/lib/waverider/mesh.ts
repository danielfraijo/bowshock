import type { MeshQuality, SurfaceGrid, SurfaceKind, TriMesh, Vec3 } from "./types";
import { SURFACE_ID } from "./types";
import { vcross, vdot, vlen, vsub } from "./math";

export class MeshBuilder {
  private keyToIndex = new Map<string, number>();
  private pos: number[] = [];
  private idx: number[] = [];
  private surf: number[] = [];
  skipped = 0;
  private q: number;

  constructor(length = 1) {
    this.q = 1e10 / Math.max(length, 1e-6);
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
    if (mag < 1e-16) {
      this.skipped++;
      return;
    }
    this.idx.push(a, b, c);
    this.surf.push(SURFACE_ID[surface]);
  }

  quad(a: number, b: number, c: number, d: number, surface: SurfaceKind) {
    this.tri(a, b, c, surface);
    this.tri(a, c, d, surface);
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
    if (nx * mx + ny * my + nz * mz < 0) {
      mesh.indices[t * 3 + 1] = ic;
      mesh.indices[t * 3 + 2] = b;
    }
  }
}

export function gridPoint(g: SurfaceGrid, i: number, j: number): Vec3 {
  const o = (i * g.nj + j) * 3;
  return [g.xyz[o], g.xyz[o + 1], g.xyz[o + 2]];
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
