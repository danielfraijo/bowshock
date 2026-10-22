/**
 * Import a tessellated CAD shell (STL or OBJ) and hand it to the panel book.
 *
 * STEP and IGES are surfaces, not triangles. Tessellating NURBS in the browser
 * is not this file — export an STL from the CAD tool. The shell is welded,
 * oriented outward, and placed in the Cuspis frame: +X aft, +Y span, +Z up,
 * most-forward point at the origin, spanwise bbox centered.
 *
 * Reference area is half the absolute projected area on the XY plane, which is
 * the planform of a closed shell (upper and lower both project). Nose radius
 * is a sphere fit through the nose vertex, used only by the heating estimate.
 */
import { analyzeMesh, MeshBuilder } from "./mesh";
import { orientConsistent, removeDoubleFaces } from "./topology";
import type { AeroEstimate, MeshQuality, TriMesh } from "./types";

export type CadAxis = "cuspis" | "y-up" | "nose-plus-x";

export interface CadImport {
  name: string;
  mesh: TriMesh;
  quality: MeshQuality;
  length: number;
  span: number;
  height: number;
  /** Projected planform, m². */
  sRef: number;
  /** Sphere fit at the nose, m. Heating only. */
  leRadius: number;
  notes: string[];
}

const MAX_BYTES = 25_000_000;
const MAX_TRIS = 60_000;

export function parseCad(data: ArrayBuffer, filename: string, opts: { scale: number; axis: CadAxis }): CadImport {
  if (data.byteLength > MAX_BYTES) throw new Error("That file is over 25 MB. Export a coarser STL.");
  if (data.byteLength < 16) throw new Error("That file is empty.");
  const scale = Number.isFinite(opts.scale) && opts.scale > 0 ? opts.scale : 1;
  const raw = readTriangles(data, filename);
  if (raw.count < 1) throw new Error("No triangles in that file.");
  if (raw.count > MAX_TRIS) {
    throw new Error(`That mesh has ${raw.count.toLocaleString()} triangles. The cap is ${MAX_TRIS.toLocaleString()} so the panel solve stays responsive — export a coarser STL.`);
  }

  const mapped: number[] = [];
  let minX = Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  let minZ = Infinity;
  let maxZ = -Infinity;
  let maxX = -Infinity;
  for (let i = 0; i < raw.pos.length; i += 3) {
    const m = mapAxis(raw.pos[i] * scale, raw.pos[i + 1] * scale, raw.pos[i + 2] * scale, opts.axis);
    mapped.push(m[0], m[1], m[2]);
    if (m[0] < minX) minX = m[0];
    if (m[0] > maxX) maxX = m[0];
    if (m[1] < minY) minY = m[1];
    if (m[1] > maxY) maxY = m[1];
    if (m[2] < minZ) minZ = m[2];
    if (m[2] > maxZ) maxZ = m[2];
  }
  const length = maxX - minX;
  const span = maxY - minY;
  const height = maxZ - minZ;
  if (!(length > 1e-6) || !(span > 1e-9 && height > 1e-9)) {
    throw new Error("The shell has no extent. Check the unit (m, mm, in) and that the file is a triangle mesh.");
  }
  const yMid = 0.5 * (minY + maxY);
  const tol = 1e-5 * Math.max(length, span, height);
  const q = (v: number) => Math.round(v / tol) * tol;
  const L = Math.max(length, 1e-3);
  const b = new MeshBuilder(L);
  for (let t = 0; t < raw.count; t++) {
    const o = t * 9;
    const a = b.vert(q(mapped[o] - minX), q(mapped[o + 1] - yMid), q(mapped[o + 2]));
    const c = b.vert(q(mapped[o + 3] - minX), q(mapped[o + 4] - yMid), q(mapped[o + 5]));
    const d = b.vert(q(mapped[o + 6] - minX), q(mapped[o + 7] - yMid), q(mapped[o + 8]));
    b.tri(a, c, d, "upper");
  }
  let mesh = removeDoubleFaces(b.finish());
  orientConsistent(mesh);
  const quality = analyzeMesh(mesh, b.skipped);
  const sRef = projectedPlanform(mesh);
  const leRadius = estimateNoseRadius(mesh, length);
  const name = safeName(filename);
  const notes = [
    `Imported ${name} (${raw.kind.toUpperCase()}). ${quality.triangles.toLocaleString()} triangles after welding. Frame: nose at x = 0, span centered, +Z up.`,
    raw.kind === "stl"
      ? "STL has no units. The file was scaled with the unit selected in the CAD tab."
      : "OBJ was scaled with the unit selected in the CAD tab.",
    opts.axis === "y-up"
      ? "Y-up CAD was rotated to Z-up, right-handed."
      : opts.axis === "nose-plus-x"
        ? "Nose-at-+X was flipped so the nose is the most-forward point."
        : "Coordinates were read as Cuspis: +X aft, +Y span, +Z up.",
    quality.watertight
      ? "The shell is closed, so volume and the panel book are meaningful."
      : `${quality.openEdges} open edges remain. Coefficients are still integrated, but volume and base pressure are less trustworthy. Export a watertight STL if you can.`,
    `Sref is half the projected XY area (${sRef.toFixed(4)} m²), the planform of a closed shell. Nose radius for heating is a sphere fit, R = ${(leRadius * 1000).toFixed(1)} mm.`,
    "Symmetry-plane Euler is not run on an imported shell. The solve is the panel method.",
  ];
  if (b.skipped) notes.push(`${b.skipped} degenerate triangles were dropped while welding.`);
  return { name, mesh, quality, length, span, height, sRef, leRadius, notes };
}

export function cadAero(imp: CadImport): AeroEstimate {
  const vol = Math.max(imp.quality.volume, 0);
  const s = Math.max(imp.sRef, 1e-8);
  return {
    thetaDeg: 0,
    betaDeg: 0,
    muDeg: 0,
    coneDeg: 0,
    pressureRatio: 1,
    temperatureRatio: 1,
    densityRatio: 1,
    m2: 0,
    planformArea: s,
    wettedArea: imp.quality.area,
    volume: vol,
    cl: 0,
    cd: 0,
    ld: 0,
    volumetricEfficiency: vol / Math.max(s * imp.length, 1e-8),
    attached: false,
    notes: imp.notes,
  };
}

function projectedPlanform(mesh: TriMesh): number {
  const p = mesh.positions;
  const idx = mesh.indices;
  let proj = 0;
  for (let t = 0; t < idx.length; t += 3) {
    const a = idx[t] * 3;
    const b = idx[t + 1] * 3;
    const c = idx[t + 2] * 3;
    const cross = (p[b] - p[a]) * (p[c + 1] - p[a + 1]) - (p[b + 1] - p[a + 1]) * (p[c] - p[a]);
    proj += 0.5 * Math.abs(cross);
  }
  return Math.max(proj * 0.5, 1e-8);
}

/** R = (dx² + r²) / (2 dx) for points just aft of the nose, median. */
function estimateNoseRadius(mesh: TriMesh, length: number): number {
  const p = mesh.positions;
  const nv = p.length / 3;
  let nose = 0;
  let bestX = Infinity;
  for (let i = 0; i < nv; i++) {
    if (p[i * 3] < bestX) {
      bestX = p[i * 3];
      nose = i;
    }
  }
  const nx = p[nose * 3];
  const ny = p[nose * 3 + 1];
  const nz = p[nose * 3 + 2];
  const xCut = nx + 0.08 * Math.max(length, 1e-6);
  const samples: number[] = [];
  for (let i = 0; i < nv; i++) {
    const dx = p[i * 3] - nx;
    if (!(dx > 1e-4 * length) || p[i * 3] > xCut) continue;
    const r = Math.hypot(p[i * 3 + 1] - ny, p[i * 3 + 2] - nz);
    if (!(r > 1e-6)) continue;
    const R = (dx * dx + r * r) / (2 * dx);
    if (R > 0 && R < 0.5 * length) samples.push(R);
  }
  if (!samples.length) return Math.max(1e-4, 0.001 * length);
  samples.sort((a, b) => a - b);
  return samples[Math.floor(samples.length / 2)];
}

function mapAxis(x: number, y: number, z: number, axis: CadAxis): [number, number, number] {
  if (axis === "y-up") return [x, -z, y];
  if (axis === "nose-plus-x") return [-x, -y, z];
  return [x, y, z];
}

function safeName(filename: string): string {
  const base = filename.replace(/^.*[/\\]/, "").replace(/\.(stl|obj)$/i, "");
  const clean = base.replace(/[^\w.\- ]+/g, "").trim().slice(0, 48);
  return clean || "imported";
}

function readTriangles(data: ArrayBuffer, filename: string): { pos: Float64Array; count: number; kind: "stl" | "obj" } {
  if (binaryStlCount(data) > 0) {
    const n = binaryStlCount(data);
    const pos = new Float64Array(n * 9);
    const dv = new DataView(data);
    let o = 84;
    for (let t = 0; t < n; t++) {
      o += 12;
      for (let k = 0; k < 9; k++) {
        pos[t * 9 + k] = dv.getFloat32(o, true);
        o += 4;
      }
      o += 2;
    }
    return { pos, count: n, kind: "stl" };
  }
  const text = new TextDecoder("utf-8", { fatal: false }).decode(data);
  const head = text.slice(0, 400).toLowerCase();
  if (head.includes("facet") && head.includes("vertex")) return { ...parseAsciiStl(text), kind: "stl" };
  if (/\.(obj)$/i.test(filename) || /^v\s/m.test(text)) return { ...parseObj(text), kind: "obj" };
  throw new Error("This is not an STL or OBJ triangle mesh. STEP and IGES surfaces are not tessellated here — export an STL from the CAD tool and import that.");
}

function binaryStlCount(data: ArrayBuffer): number {
  if (data.byteLength < 84) return 0;
  const n = new DataView(data).getUint32(80, true);
  if (n < 1 || n > MAX_TRIS * 4) return 0;
  if (data.byteLength !== 84 + n * 50) return 0;
  return n;
}

function parseAsciiStl(text: string): { pos: Float64Array; count: number } {
  const nums: number[] = [];
  const re = /vertex\s+([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)\s+([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)\s+([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(text))) nums.push(Number(m[1]), Number(m[2]), Number(m[3]));
  if (nums.length < 9 || nums.length % 9 !== 0) throw new Error("ASCII STL did not contain complete triangles.");
  return { pos: Float64Array.from(nums), count: nums.length / 9 };
}

function parseObj(text: string): { pos: Float64Array; count: number } {
  const verts: number[] = [];
  const tris: number[] = [];
  for (const line of text.split(/\r?\n/)) {
    const s = line.trim();
    if (s.startsWith("v ")) {
      const p = s.split(/\s+/);
      const x = Number(p[1]);
      const y = Number(p[2]);
      const z = Number(p[3]);
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
      verts.push(x, y, z);
    } else if (s.startsWith("f ")) {
      const ids = s
        .split(/\s+/)
        .slice(1)
        .map((tok) => {
          const i = parseInt(tok.split("/")[0] || "", 10);
          if (!Number.isFinite(i) || i === 0) return -1;
          return i > 0 ? i - 1 : verts.length / 3 + i;
        })
        .filter((i) => i >= 0);
      for (let k = 1; k < ids.length - 1; k++) tris.push(ids[0], ids[k], ids[k + 1]);
    }
  }
  if (!tris.length) throw new Error("OBJ has vertices but no faces.");
  const pos = new Float64Array(tris.length * 3);
  const nv = verts.length / 3;
  for (let i = 0; i < tris.length; i++) {
    const id = tris[i];
    if (id < 0 || id >= nv) throw new Error("OBJ face index is outside the vertex list.");
    pos[i * 3] = verts[id * 3];
    pos[i * 3 + 1] = verts[id * 3 + 1];
    pos[i * 3 + 2] = verts[id * 3 + 2];
  }
  return { pos, count: tris.length / 3 };
}
