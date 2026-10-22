/**
 * Analytic test meshes for the accuracy audit (`npm run test:audit`).
 * Every mesh is closed and wound with outward normals ((b − a) × (c − a) points out), which is
 * the convention `panelAero` and `massProperties` assume. Frame: X aft, Y span, Z up.
 */
import { SURFACE_ID, type TriMesh } from "./types";

class Builder {
  pos: number[] = [];
  ind: number[] = [];
  surf: number[] = [];
  v(x: number, y: number, z: number) {
    this.pos.push(x, y, z);
    return this.pos.length / 3 - 1;
  }
  tri(a: number, b: number, c: number, s: number) {
    this.ind.push(a, b, c);
    this.surf.push(s);
  }
  quad(a: number, b: number, c: number, d: number, s: number) {
    this.tri(a, b, c, s);
    this.tri(a, c, d, s);
  }
  mesh(): TriMesh {
    return { positions: new Float64Array(this.pos), indices: new Uint32Array(this.ind), surfaces: new Uint8Array(this.surf) };
  }
}

/** Axis-aligned box a×b×c centred at o. */
export function boxMesh(a: number, b: number, c: number, o: [number, number, number] = [0, 0, 0], surf = SURFACE_ID.upper): TriMesh {
  const B = new Builder();
  for (let i = 0; i < 8; i++) B.v(o[0] + (i & 1 ? a / 2 : -a / 2), o[1] + (i & 2 ? b / 2 : -b / 2), o[2] + (i & 4 ? c / 2 : -c / 2));
  // faces with outward normals: -z, +z, -y, +y, -x, +x
  const F = [
    [0, 2, 3, 1],
    [4, 5, 7, 6],
    [0, 1, 5, 4],
    [2, 6, 7, 3],
    [0, 4, 6, 2],
    [1, 3, 7, 5],
  ];
  for (const f of F) B.quad(f[0], f[1], f[2], f[3], surf);
  return B.mesh();
}

/**
 * Thin flat plate in the X–Y plane (x ∈ [0, L], y ∈ ±b/2, z = z0 ± t/2), split into nx × ny cells on each face.
 * Edge faces are tagged "base"-free upper so their tiny area only adds friction.
 */
export function flatPlateMesh(L: number, b: number, t: number, nx = 8, ny = 4, z0 = 0): TriMesh {
  const B = new Builder();
  const lo: number[][] = [];
  const hi: number[][] = [];
  for (let i = 0; i <= nx; i++) {
    lo.push([]);
    hi.push([]);
    for (let j = 0; j <= ny; j++) {
      const x = (L * i) / nx;
      const y = -b / 2 + (b * j) / ny;
      lo[i].push(B.v(x, y, z0 - t / 2));
      hi[i].push(B.v(x, y, z0 + t / 2));
    }
  }
  for (let i = 0; i < nx; i++) {
    for (let j = 0; j < ny; j++) {
      // lower face, outward −z
      B.quad(lo[i][j], lo[i][j + 1], lo[i + 1][j + 1], lo[i + 1][j], SURFACE_ID.lower);
      // upper face, outward +z
      B.quad(hi[i][j], hi[i + 1][j], hi[i + 1][j + 1], hi[i][j + 1], SURFACE_ID.upper);
    }
  }
  // perimeter (thin) — leading edge x = 0 is tagged leading so the base heuristic (nx > 0.72) cannot flip it
  for (let j = 0; j < ny; j++) {
    B.quad(lo[0][j], hi[0][j], hi[0][j + 1], lo[0][j + 1], SURFACE_ID.leading);
    B.quad(lo[nx][j], lo[nx][j + 1], hi[nx][j + 1], hi[nx][j], SURFACE_ID.base);
  }
  for (let i = 0; i < nx; i++) {
    B.quad(lo[i][0], lo[i + 1][0], hi[i + 1][0], hi[i][0], SURFACE_ID.upper);
    B.quad(lo[i][ny], hi[i][ny], hi[i + 1][ny], lo[i + 1][ny], SURFACE_ID.upper);
  }
  return B.mesh();
}

/** Rotate a mesh by +90° about X (y → z, z → −y): the plate becomes a vertical fin with normals ±Y. */
export function rotateX90(m: TriMesh): TriMesh {
  const p = new Float64Array(m.positions.length);
  for (let i = 0; i < p.length; i += 3) {
    p[i] = m.positions[i];
    p[i + 1] = -m.positions[i + 2];
    p[i + 2] = m.positions[i + 1];
  }
  return { positions: p, indices: m.indices, surfaces: m.surfaces };
}

/** Translate a mesh. */
export function translate(m: TriMesh, d: [number, number, number]): TriMesh {
  const p = new Float64Array(m.positions.length);
  for (let i = 0; i < p.length; i += 3) {
    p[i] = m.positions[i] + d[0];
    p[i + 1] = m.positions[i + 1] + d[1];
    p[i + 2] = m.positions[i + 2] + d[2];
  }
  return { positions: p, indices: m.indices, surfaces: m.surfaces };
}

/**
 * UV sphere of radius R centred at (cx, 0, 0). nLat bands from the nose (x = cx − R) to the tail.
 * Forward hemisphere is tagged lower/upper by sign of z; panelAero reclassifies aft-facing panels (n_x > 0.72) as base.
 */
export function sphereMesh(R: number, cx: number, nLat = 48, nLon = 64): TriMesh {
  const B = new Builder();
  const rings: number[][] = [];
  const nose = B.v(cx - R, 0, 0);
  for (let i = 1; i < nLat; i++) {
    const th = (Math.PI * i) / nLat; // polar angle from −X
    const ring: number[] = [];
    for (let j = 0; j < nLon; j++) {
      const ph = (2 * Math.PI * j) / nLon;
      ring.push(B.v(cx - R * Math.cos(th), R * Math.sin(th) * Math.cos(ph), R * Math.sin(th) * Math.sin(ph)));
    }
    rings.push(ring);
  }
  const tail = B.v(cx + R, 0, 0);
  const s = (j: number) => (Math.sin((2 * Math.PI * (j + 0.5)) / nLon) >= 0 ? SURFACE_ID.upper : SURFACE_ID.lower);
  for (let j = 0; j < nLon; j++) {
    const j1 = (j + 1) % nLon;
    B.tri(nose, rings[0][j1], rings[0][j], s(j));
    for (let i = 0; i < rings.length - 1; i++) B.quad(rings[i][j], rings[i][j1], rings[i + 1][j1], rings[i + 1][j], s(j));
    B.tri(tail, rings[rings.length - 1][j], rings[rings.length - 1][j1], s(j));
  }
  return B.mesh();
}

/** Outward-normal check: signed volume > 0 when every face winds outward. */
export function signedVolume(m: TriMesh): number {
  const p = m.positions;
  const I = m.indices;
  let v = 0;
  for (let t = 0; t < I.length; t += 3) {
    const a = I[t] * 3;
    const b = I[t + 1] * 3;
    const c = I[t + 2] * 3;
    v += p[a] * (p[b + 1] * p[c + 2] - p[b + 2] * p[c + 1]) - p[a + 1] * (p[b] * p[c + 2] - p[b + 2] * p[c]) + p[a + 2] * (p[b] * p[c + 1] - p[b + 1] * p[c]);
  }
  return v / 6;
}
