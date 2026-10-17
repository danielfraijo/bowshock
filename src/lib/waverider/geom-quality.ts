/**
 * Geometry-quality measurements for the topology / watertightness work.
 * Pure functions over TriMesh + exported text/binary — used by
 * topology.test.ts and scripts/topology-report.ts. Not imported by the app.
 */
import { evalSurface, type NurbsNet } from "./export-nurbs";
import type { TriMesh } from "./types";
import { SURFACE_ID } from "./types";

export interface TopoMetrics {
  triangles: number;
  vertices: number;
  openEdges: number;
  nonManifoldEdges: number;
  /** Directed edge used twice (two faces disagree on orientation) => flipped neighbour. */
  inconsistentEdges: number;
  duplicateVerts: number;
  degenerateTris: number;
  minAngleDeg: number;
  /** Triangles with min angle below `sliverDeg`. */
  slivers: number;
  /** Longest edge / shortest altitude (>= 2/sqrt3 ~ 1.155 for equilateral). */
  maxAspect: number;
  /** Same three measures over non-fillet triangles only (the LE fillet strips are anisotropic by construction). */
  bodyMinAngleDeg: number;
  bodySlivers: number;
  /** Longest edge / shortest edge over non-fillet triangles (the metric fidelity.test.ts bounds). */
  bodyEdgeRatio: number;
  /** Signed volume (positive = outward normals). */
  signedVolume: number;
  /** Max dihedral (normal-to-normal) angle across edges shared by two faces of the same surface id. */
  maxJumpSame: number;
  /** Max dihedral across upper|lower <-> leading seams (G1 fillet measure). */
  maxJumpLE: number;
  /** Max dihedral across any interior edge with both faces forward of x = noseX. */
  maxJumpNose: number;
  /** Number of edges whose dihedral exceeds 60deg. */
  sharpEdges: number;
}

export interface TopoOptions {
  sliverDeg?: number;
  /** Relative weld tolerance (fraction of bbox diagonal). */
  weldRel?: number;
  /** Streamwise x extent considered "nose" for maxJumpNose. */
  noseX?: number;
  /** |y| beyond this fraction of the max |y| is the wing-tip zone (excluded from G1 maxima). */
  tipFrac?: number;
}

function tri(mesh: TriMesh, t: number) {
  const p = mesh.positions;
  const i = mesh.indices;
  const a = i[t * 3] * 3;
  const b = i[t * 3 + 1] * 3;
  const c = i[t * 3 + 2] * 3;
  return [
    [p[a], p[a + 1], p[a + 2]],
    [p[b], p[b + 1], p[b + 2]],
    [p[c], p[c + 1], p[c + 2]],
  ] as const;
}

export function triNormal(mesh: TriMesh, t: number): [number, number, number, number] {
  const [a, b, c] = tri(mesh, t);
  const abx = b[0] - a[0], aby = b[1] - a[1], abz = b[2] - a[2];
  const acx = c[0] - a[0], acy = c[1] - a[1], acz = c[2] - a[2];
  const nx = aby * acz - abz * acy;
  const ny = abz * acx - abx * acz;
  const nz = abx * acy - aby * acx;
  const m = Math.hypot(nx, ny, nz);
  return m > 0 ? [nx / m, ny / m, nz / m, m] : [0, 0, 0, 0];
}

export function triAngles(mesh: TriMesh, t: number): { minAngle: number; aspect: number; area: number } {
  const [a, b, c] = tri(mesh, t);
  const d = (p: readonly number[], q: readonly number[]) => Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
  const la = d(b, c), lb = d(c, a), lc = d(a, b);
  const n = triNormal(mesh, t);
  const area = 0.5 * n[3];
  const longest = Math.max(la, lb, lc);
  if (!(area > 0) || longest <= 0) return { minAngle: 0, aspect: Infinity, area: 0 };
  const alt = (2 * area) / longest;
  const ang = (x: number, y: number, z: number) => Math.acos(Math.max(-1, Math.min(1, (y * y + z * z - x * x) / (2 * y * z))));
  const minAngle = Math.min(ang(la, lb, lc), ang(lb, lc, la), ang(lc, la, lb));
  return { minAngle: (minAngle * 180) / Math.PI, aspect: longest / alt, area };
}

export function topoMetrics(mesh: TriMesh, opt: TopoOptions = {}): TopoMetrics {
  const sliverDeg = opt.sliverDeg ?? 3;
  const nt = mesh.indices.length / 3;
  const nv = mesh.positions.length / 3;
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < nv; i++) {
    for (let k = 0; k < 3; k++) {
      const v = mesh.positions[i * 3 + k];
      if (v < min[k]) min[k] = v;
      if (v > max[k]) max[k] = v;
    }
  }
  const diag = nv ? Math.hypot(max[0] - min[0], max[1] - min[1], max[2] - min[2]) : 1;
  const weld = (opt.weldRel ?? 1e-9) * diag;
  const noseX = opt.noseX ?? 0.08 * (max[0] - min[0]);
  // tip zone: planform tips are knife edges by design (collapsed to the mid-surface); G1 stats skip them
  const tipY = (opt.tipFrac ?? 0.96) * Math.max(Math.abs(min[1]), Math.abs(max[1]));

  // duplicate (unwelded) verts: distinct indices at the same position
  const q = 1 / Math.max(weld, 1e-300);
  const seen = new Map<string, number>();
  let dup = 0;
  const used = new Uint8Array(nv);
  for (let t = 0; t < nt * 3; t++) used[mesh.indices[t]] = 1;
  for (let i = 0; i < nv; i++) {
    if (!used[i]) continue;
    const key = `${Math.round(mesh.positions[i * 3] * q)}:${Math.round(mesh.positions[i * 3 + 1] * q)}:${Math.round(mesh.positions[i * 3 + 2] * q)}`;
    if (seen.has(key)) dup++;
    else seen.set(key, i);
  }

  type E = { n: number; t0: number; t1: number; f: number; b: number; lo: number; hi: number };
  const edges = new Map<number, E>();
  const K = nv + 1;
  let minAngle = 180, bodyMin = 180, bodySliv = 0, bodyER = 0;
  let slivers = 0;
  let maxAspect = 1;
  let degenerate = 0;
  let vol = 0;
  for (let t = 0; t < nt; t++) {
    const ia = mesh.indices[t * 3], ib = mesh.indices[t * 3 + 1], ic = mesh.indices[t * 3 + 2];
    const ta = triAngles(mesh, t);
    if (!(ta.area > 1e-20 * diag * diag)) {
      degenerate++;
      continue;
    }
    if (mesh.surfaces[t] !== SURFACE_ID.leading) {
      if (ta.minAngle < bodyMin) bodyMin = ta.minAngle;
      if (ta.minAngle < sliverDeg) bodySliv++;
      const el = [0, 1, 2].map((k) => Math.hypot(...([0, 1, 2].map((c) => tri(mesh, t)[k][c] - tri(mesh, t)[(k + 1) % 3][c]) as [number, number, number])));
      const er = Math.max(...el) / Math.max(Math.min(...el), 1e-300);
      if (er > bodyER) bodyER = er;
    }
    if (ta.minAngle < minAngle) minAngle = ta.minAngle;
    if (ta.minAngle < sliverDeg) slivers++;
    if (ta.aspect > maxAspect) maxAspect = ta.aspect;
    const [a, b, c] = tri(mesh, t);
    vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
    const ring = [ia, ib, ic];
    for (let k = 0; k < 3; k++) {
      const u = ring[k];
      const v = ring[(k + 1) % 3];
      const lo = Math.min(u, v);
      const hi = Math.max(u, v);
      const key = lo * K + hi;
      const e = edges.get(key);
      const fwd = u < v;
      if (e) {
        e.n++;
        e.t1 = t;
        if (fwd) e.f++;
        else e.b++;
      } else edges.set(key, { n: 1, t0: t, t1: -1, f: fwd ? 1 : 0, b: fwd ? 0 : 1, lo, hi });
    }
  }
  let open = 0, nonMan = 0, incons = 0;
  let jumpSame = 0, jumpLE = 0, jumpNose = 0, sharp = 0;
  const S = SURFACE_ID;
  for (const e of edges.values()) {
    if (e.n === 1) open++;
    else if (e.n > 2) nonMan++;
    else {
      if (e.f !== 1 || e.b !== 1) incons++;
      const n0 = triNormal(mesh, e.t0);
      const n1 = triNormal(mesh, e.t1);
      if (n0[3] === 0 || n1[3] === 0) continue;

      const ang = (Math.acos(Math.max(-1, Math.min(1, n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2]))) * 180) / Math.PI;
      const s0 = mesh.surfaces[e.t0];
      const s1 = mesh.surfaces[e.t1];
      if (ang > 60) sharp++;
      const inTip = Math.abs(mesh.positions[e.lo * 3 + 1]) > tipY && Math.abs(mesh.positions[e.hi * 3 + 1]) > tipY;
      if (inTip) continue;
      if (s0 === s1 && ang > jumpSame) jumpSame = ang;
      const le = (s0 === S.leading && (s1 === S.upper || s1 === S.lower)) || (s1 === S.leading && (s0 === S.upper || s0 === S.lower));
      if (le && ang > jumpLE) jumpLE = ang;
      const cx = (t: number) => {
        const [a, b, c] = tri(mesh, t);
        return (a[0] + b[0] + c[0]) / 3;
      };
      if (cx(e.t0) < noseX && cx(e.t1) < noseX && (s0 === S.upper || s0 === S.lower || s0 === S.leading) && (s1 === S.upper || s1 === S.lower || s1 === S.leading) && ang > jumpNose) jumpNose = ang;
    }
  }
  return {
    triangles: nt,
    vertices: nv,
    openEdges: open,
    nonManifoldEdges: nonMan,
    inconsistentEdges: incons,
    duplicateVerts: dup,
    degenerateTris: degenerate,
    minAngleDeg: nt ? minAngle : 0,
    bodyMinAngleDeg: nt ? bodyMin : 0,
    bodySlivers: bodySliv,
    bodyEdgeRatio: bodyER,
    slivers,
    maxAspect,
    signedVolume: vol,
    maxJumpSame: jumpSame,
    maxJumpLE: jumpLE,
    maxJumpNose: jumpNose,
    sharpEdges: sharp,
  };
}

// ---------- export validators ----------

export interface ExportCheck {
  ok: boolean;
  errors: string[];
  info: Record<string, number | string>;
}

export function checkBinaryStl(buf: ArrayBuffer, expectTris: number): ExportCheck {
  const errors: string[] = [];
  const view = new DataView(buf);
  const n = buf.byteLength >= 84 ? view.getUint32(80, true) : -1;
  if (buf.byteLength !== 84 + 50 * n) errors.push(`size ${buf.byteLength} != 84+50*${n}`);
  if (n !== expectTris) errors.push(`triangle count ${n} != ${expectTris}`);
  let bad = 0;
  let flipped = 0;
  for (let t = 0; t < Math.min(n, Math.floor((buf.byteLength - 84) / 50)); t++) {
    const o = 84 + t * 50;
    const f = (k: number) => view.getFloat32(o + k * 4, true);
    for (let k = 0; k < 12; k++) if (!Number.isFinite(f(k))) bad++;
    const ax = f(3), ay = f(4), az = f(5), bx = f(6), by = f(7), bz = f(8), cx = f(9), cy = f(10), cz = f(11);
    const nx = (by - ay) * (cz - az) - (bz - az) * (cy - ay);
    const ny = (bz - az) * (cx - ax) - (bx - ax) * (cz - az);
    const nz = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (nx * f(0) + ny * f(1) + nz * f(2) < 0) flipped++;
  }
  if (bad) errors.push(`${bad} non-finite floats`);
  if (flipped) errors.push(`${flipped} facet normals disagree with winding`);
  return { ok: errors.length === 0, errors, info: { tris: n } };
}

/** Minimal 80-column fixed-format IGES section/sequence validation. */
export function checkIges(text: string): ExportCheck {
  const errors: string[] = [];
  const lines = text.split("\n").filter((l) => l.length);
  const cnt: Record<string, number> = { S: 0, G: 0, D: 0, P: 0, T: 0 };
  const lastSeq: Record<string, number> = {};
  for (const l of lines) {
    if (l.length !== 80) errors.push(`line length ${l.length}: ${l.slice(0, 20)}`);
    const sec = l[72];
    cnt[sec] = (cnt[sec] ?? 0) + 1;
    const seq = parseInt(l.slice(73, 80), 10);
    if (sec !== "T" && seq !== cnt[sec]) errors.push(`${sec} seq ${seq} expected ${cnt[sec]}`);
    lastSeq[sec] = seq;
    if (errors.length > 8) break;
  }
  const t = lines[lines.length - 1] ?? "";
  if (t[72] !== "T") errors.push("missing T record");
  else {
    const get = (s: string, k: number) => parseInt(t.slice(k, k + 8).replace(/\D/g, ""), 10);
    if (get("S", 1) !== cnt.S) errors.push(`T S=${get("S", 1)} vs ${cnt.S}`);
    if (get("G", 9) !== cnt.G) errors.push(`T G=${get("G", 9)} vs ${cnt.G}`);
    if (get("D", 17) !== cnt.D) errors.push(`T D=${get("D", 17)} vs ${cnt.D}`);
    if (get("P", 25) !== cnt.P) errors.push(`T P=${get("P", 25)} vs ${cnt.P}`);
  }
  // each directory entry pair must point at a P line, and PD pointers resolve
  const dLines = lines.filter((l) => l[72] === "D");
  const pLines = lines.filter((l) => l[72] === "P");
  let surfaces = 0;
  for (let k = 0; k + 1 < dLines.length; k += 2) {
    const type = parseInt(dLines[k].slice(0, 8), 10);
    const pstart = parseInt(dLines[k].slice(8, 16), 10);
    const pcount = parseInt(dLines[k + 1].slice(24, 32), 10);
    if (type !== 128) errors.push(`D${k + 1}: unexpected type ${type}`);
    if (!(pstart >= 1 && pstart + pcount - 1 <= pLines.length)) errors.push(`D${k + 1}: P range ${pstart}+${pcount} outside ${pLines.length}`);
    else {
      const payload = pLines
        .slice(pstart - 1, pstart - 1 + pcount)
        .map((l) => l.slice(0, 64))
        .join("")
        .replace(/\s+$/, "");
      if (!payload.endsWith(";")) errors.push(`D${k + 1}: parameter record not terminated`);
      const v = payload.slice(0, -1).split(",");
      const k1 = +v[1], k2 = +v[2], m1 = +v[3], m2 = +v[4];
      const nu = k1 + 1, nv = k2 + 1;
      const expect = 1 + 9 + (k1 + m1 + 2) + (k2 + m2 + 2) + nu * nv + 3 * nu * nv + 4;
      if (v.length !== expect) errors.push(`D${k + 1}: ${v.length} params, expected ${expect}`);
      for (const tok of v) if (!Number.isFinite(+tok)) {
        errors.push(`D${k + 1}: non-numeric token ${tok}`);
        break;
      }
    }
    surfaces++;
  }
  return { ok: errors.length === 0, errors: errors.slice(0, 10), info: { surfaces, D: cnt.D, P: cnt.P } };
}

/** Parse a Plot3D formatted multi-block XYZ file and check counts + finiteness. */
export function checkPlot3d(text: string): ExportCheck {
  const errors: string[] = [];
  const toks = text.split(/\s+/).filter(Boolean);
  let o = 0;
  const nb = parseInt(toks[o++], 10);
  if (!(nb >= 1)) return { ok: false, errors: ["bad block count"], info: {} };
  const dims: number[][] = [];
  for (let b = 0; b < nb; b++) dims.push([+toks[o++], +toks[o++], +toks[o++]]);
  let need = 0;
  for (const d of dims) need += 3 * d[0] * d[1] * d[2];
  if (toks.length - o !== need) errors.push(`value count ${toks.length - o} != ${need}`);
  for (let k = o; k < toks.length; k++) if (!Number.isFinite(+toks[k])) {
    errors.push(`non-finite value at ${k}`);
    break;
  }
  return { ok: errors.length === 0, errors, info: { blocks: nb } };
}

interface StepEnt { id: number; type: string; body: string }

export function parseStep(text: string): Map<number, StepEnt> {
  const data = text.slice(text.indexOf("DATA;") + 5, text.lastIndexOf("ENDSEC;"));
  const map = new Map<number, StepEnt>();
  for (const m of data.matchAll(/#(\d+)\s*=\s*([\s\S]*?);\s*(?=#\d+\s*=|$)/g)) {
    const body = m[2].trim();
    const tm = /^\(?\s*([A-Z_0-9]+)/.exec(body);
    map.set(+m[1], { id: +m[1], type: tm ? tm[1] : "?", body });
  }
  return map;
}

/**
 * Structural STEP checks: refs resolve, CLOSED_SHELL edges each used by exactly two face loops, traversed in
 * opposite directions (loop direction = ORIENTED_EDGE sense x FACE_BOUND sense; same_sense only concerns the normal).
 */
export function checkStep(text: string): ExportCheck {
  const errors: string[] = [];
  if (!text.startsWith("ISO-10303-21;") || !text.trimEnd().endsWith("END-ISO-10303-21;")) errors.push("bad envelope");
  const ents = parseStep(text);
  let dangling = 0;
  for (const e of ents.values()) {
    for (const r of e.body.matchAll(/#(\d+)/g)) if (!ents.has(+r[1])) dangling++;
  }
  if (dangling) errors.push(`${dangling} dangling #refs`);
  const types = new Map<string, number>();
  for (const e of ents.values()) types.set(e.type, (types.get(e.type) ?? 0) + 1);
  const info: Record<string, number | string> = { entities: ents.size };
  const shell = [...ents.values()].find((e) => e.type === "CLOSED_SHELL");
  if (!shell) errors.push("no CLOSED_SHELL");
  else if (types.has("EDGE_CURVE")) {
    const faces = [...shell.body.matchAll(/#(\d+)/g)].map((m) => +m[1]);
    const use = new Map<number, number[]>();
    for (const f of faces) {
      const face = ents.get(f);
      if (!face) continue;
      const bounds = [...face.body.matchAll(/#(\d+)/g)].map((m) => ents.get(+m[1])).filter((x) => x && x.type === "FACE_OUTER_BOUND");
      for (const bd of bounds) {
        const loop = ents.get(+/#(\d+)/.exec(bd!.body)![1]);
        if (!loop) continue;
        for (const m of loop.body.matchAll(/#(\d+)/g)) {
          const oe = ents.get(+m[1]);
          if (!oe || oe.type !== "ORIENTED_EDGE") continue;
          const edgeId = +/#(\d+),\.[TF]\.\)$/.exec(oe.body)![1];
          const sense = (/,\.T\.\)$/.test(oe.body) ? 1 : -1) * (/,\.T\.\)$/.test(bd!.body) ? 1 : -1);
          const arr = use.get(edgeId) ?? [];
          arr.push(sense);
          use.set(edgeId, arr);
        }
      }
    }
    let open = 0, nonMan = 0, same = 0;
    for (const arr of use.values()) {
      if (arr.length === 1) open++;
      else if (arr.length !== 2) nonMan++;
      else if (arr[0] === arr[1]) same++;
    }
    info.faces = faces.length;
    info.edges = use.size;
    info.openEdges = open;
    info.nonManifoldEdges = nonMan;
    info.sameSenseEdges = same;
    if (open) errors.push(`${open} edges used by only one face`);
    if (nonMan) errors.push(`${nonMan} edges used by >2 faces`);
    if (same) errors.push(`${same} edges traversed in the same direction by both faces`);
  } else info.facetedOrTess = "yes";
  return { ok: errors.length === 0, errors, info };
}

/** Locations of problem edges (for debugging / reports). */
export function badEdges(mesh: TriMesh): { kind: "open" | "nonManifold" | "inconsistent"; a: number[]; b: number[]; surfaces: number[] }[] {
  const nt = mesh.indices.length / 3;
  const nv = mesh.positions.length / 3;
  const K = nv + 1;
  const map = new Map<number, { tris: number[]; dirs: boolean[] }>();
  for (let t = 0; t < nt; t++) {
    for (let k = 0; k < 3; k++) {
      const u = mesh.indices[t * 3 + k];
      const v = mesh.indices[t * 3 + ((k + 1) % 3)];
      const key = Math.min(u, v) * K + Math.max(u, v);
      const e = map.get(key) ?? { tris: [], dirs: [] };
      e.tris.push(t);
      e.dirs.push(u < v);
      map.set(key, e);
    }
  }
  const out: { kind: "open" | "nonManifold" | "inconsistent"; a: number[]; b: number[]; surfaces: number[] }[] = [];
  for (const [key, e] of map) {
    const lo = Math.floor(key / K);
    const hi = key % K;
    const a = [mesh.positions[lo * 3], mesh.positions[lo * 3 + 1], mesh.positions[lo * 3 + 2]];
    const b = [mesh.positions[hi * 3], mesh.positions[hi * 3 + 1], mesh.positions[hi * 3 + 2]];
    const surfaces = e.tris.map((t) => mesh.surfaces[t]);
    if (e.tris.length === 1) out.push({ kind: "open", a, b, surfaces });
    else if (e.tris.length > 2) out.push({ kind: "nonManifold", a, b, surfaces });
    else if (e.dirs[0] === e.dirs[1]) out.push({ kind: "inconsistent", a, b, surfaces });
  }
  return out;
}

/** Edges (both faces present) sorted by dihedral angle, with location + surface ids. For debugging. */
export function worstDihedrals(mesh: TriMesh, n = 10, filter?: (s0: number, s1: number) => boolean) {
  const nt = mesh.indices.length / 3;
  const nv = mesh.positions.length / 3;
  const K = nv + 1;
  const map = new Map<number, number[]>();
  for (let t = 0; t < nt; t++)
    for (let k = 0; k < 3; k++) {
      const u = mesh.indices[t * 3 + k];
      const v = mesh.indices[t * 3 + ((k + 1) % 3)];
      const key = Math.min(u, v) * K + Math.max(u, v);
      const a = map.get(key);
      if (a) a.push(t);
      else map.set(key, [t]);
    }
  const out: { ang: number; a: number[]; b: number[]; s: number[] }[] = [];
  for (const [key, ts] of map) {
    if (ts.length !== 2) continue;
    const s0 = mesh.surfaces[ts[0]], s1 = mesh.surfaces[ts[1]];
    if (filter && !filter(s0, s1)) continue;
    const n0 = triNormal(mesh, ts[0]);
    const n1 = triNormal(mesh, ts[1]);
    if (!n0[3] || !n1[3]) continue;
    const ang = (Math.acos(Math.max(-1, Math.min(1, n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2]))) * 180) / Math.PI;
    const lo = Math.floor(key / K), hi = key % K;
    out.push({ ang, a: [mesh.positions[lo * 3], mesh.positions[lo * 3 + 1], mesh.positions[lo * 3 + 2]], b: [mesh.positions[hi * 3], mesh.positions[hi * 3 + 1], mesh.positions[hi * 3 + 2]], s: [s0, s1] });
  }
  out.sort((x, y) => y.ang - x.ang);
  return out.slice(0, n);
}


// ---------------------------------------------------------------------------------------------
// Geometric export checks: read the B-spline surfaces back and compare with the mesh.
// ---------------------------------------------------------------------------------------------

/** Parse every Rational B-Spline Surface (128) of an IGES file back into control nets (i = U index, fastest). */
export function parseIgesSurfaces(text: string): NurbsNet[] {
  const lines = text.split("\n").filter((l) => l.length >= 73);
  const dLines = lines.filter((l) => l[72] === "D");
  const pLines = lines.filter((l) => l[72] === "P");
  const out: NurbsNet[] = [];
  for (let k = 0; k + 1 < dLines.length; k += 2) {
    const pstart = parseInt(dLines[k].slice(8, 16), 10);
    const pcount = parseInt(dLines[k + 1].slice(24, 32), 10);
    const payload = pLines
      .slice(pstart - 1, pstart - 1 + pcount)
      .map((l) => l.slice(0, 64))
      .join("")
      .trim();
    const v = payload.slice(0, -1).split(",").map((t) => +t.trim());
    const k1 = v[1], k2 = v[2], m1 = v[3], m2 = v[4];
    let o = 10;
    const U = v.slice(o, o + k1 + m1 + 2);
    o += U.length;
    const V = v.slice(o, o + k2 + m2 + 2);
    o += V.length;
    o += (k1 + 1) * (k2 + 1); // weights
    const ctrl: [number, number, number][][] = Array.from({ length: k1 + 1 }, () => new Array(k2 + 1));
    for (let j = 0; j <= k2; j++)
      for (let i = 0; i <= k1; i++) {
        ctrl[i][j] = [v[o], v[o + 1], v[o + 2]];
        o += 3;
      }
    out.push({ p: m1, q: m2, U, V, ctrl });
  }
  return out;
}

/** Parse the B_SPLINE_SURFACE_WITH_KNOTS faces of a STEP file: net, same_sense flag, face name. */
export function parseStepSurfaces(text: string): { net: NurbsNet; sameSense: boolean; name: string }[] {
  const ents = parseStep(text);
  const out: { net: NurbsNet; sameSense: boolean; name: string }[] = [];
  const pointOf = (id: number): [number, number, number] => {
    const m = /\(\s*([^,()]+),([^,()]+),([^,()]+)\)\s*\)\s*$/.exec(ents.get(id)!.body)!;
    return [+m[1], +m[2], +m[3]];
  };
  const expand = (mult: number[], kn: number[]) => {
    const K: number[] = [];
    mult.forEach((m, i) => { for (let r = 0; r < m; r++) K.push(kn[i]); });
    return K;
  };
  for (const e of ents.values()) {
    if (e.type !== "ADVANCED_FACE") continue;
    const sameSense = /,\.T\.\)$/.test(e.body);
    const name = /^ADVANCED_FACE\('([^']*)'/.exec(e.body)?.[1] ?? "";
    const surfId = [...e.body.matchAll(/#(\d+)/g)].map((m) => +m[1]).map((id) => ents.get(id)).find((x) => x && x.type === "B_SPLINE_SURFACE_WITH_KNOTS");
    if (!surfId) continue;
    const b = surfId.body;
    const head = /^B_SPLINE_SURFACE_WITH_KNOTS\('[^']*',(\d+),(\d+),\((.*?)\),\.[A-Z_]+\.,\.[TF]\.,\.[TF]\.,\.[TF]\.,\(([\d,]+)\),\(([\d,]+)\),\(([^)]*)\),\(([^)]*)\),\.[A-Z_]+\.\)$/s.exec(b);
    if (!head) continue;
    const rows = [...head[3].matchAll(/\(([^()]*)\)/g)].map((m) => [...m[1].matchAll(/#(\d+)/g)].map((r) => pointOf(+r[1])));
    const mu = head[4].split(",").map(Number), mv = head[5].split(",").map(Number);
    const ku = head[6].split(",").map(Number), kv = head[7].split(",").map(Number);
    out.push({ net: { p: +head[1], q: +head[2], U: expand(mu, ku), V: expand(mv, kv), ctrl: rows }, sameSense, name });
  }
  return out;
}

/** Sample surfaces on an n x n parameter grid. */
export function sampleSurfaces(nets: NurbsNet[], n = 12): [number, number, number][] {
  const pts: [number, number, number][] = [];
  for (const net of nets) for (let a = 0; a <= n; a++) for (let b = 0; b <= n; b++) pts.push(evalSurface(net, a / n, b / n));
  return pts;
}

/** Largest distance from any sample point to the closest mesh triangle (brute force; keep meshes small). */
export function maxDistanceToMesh(points: [number, number, number][], mesh: TriMesh): number {
  const nt = mesh.indices.length / 3;
  const T: number[][] = [];
  for (let t = 0; t < nt; t++) {
    const [a, b, c] = tri(mesh, t);
    T.push([a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]]);
  }
  let worst = 0;
  for (const p of points) {
    let best = Infinity;
    for (const q of T) {
      const d = pointTriDist(p, q);
      if (d < best) best = d;
      if (best < 1e-12) break;
    }
    if (best > worst) worst = best;
  }
  return worst;
}

function pointTriDist(p: number[], t: number[]): number {
  // Ericson, Real-Time Collision Detection: closest point on triangle
  const ab = [t[3] - t[0], t[4] - t[1], t[5] - t[2]];
  const ac = [t[6] - t[0], t[7] - t[1], t[8] - t[2]];
  const ap = [p[0] - t[0], p[1] - t[1], p[2] - t[2]];
  const dot = (u: number[], v: number[]) => u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  const d1 = dot(ab, ap), d2 = dot(ac, ap);
  let cp: number[];
  if (d1 <= 0 && d2 <= 0) cp = [t[0], t[1], t[2]];
  else {
    const bp = [p[0] - t[3], p[1] - t[4], p[2] - t[5]];
    const d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) cp = [t[3], t[4], t[5]];
    else {
      const vc = d1 * d4 - d3 * d2;
      if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const v = d1 / (d1 - d3);
        cp = [t[0] + v * ab[0], t[1] + v * ab[1], t[2] + v * ab[2]];
      } else {
        const cpv = [p[0] - t[6], p[1] - t[7], p[2] - t[8]];
        const d5 = dot(ab, cpv), d6 = dot(ac, cpv);
        if (d6 >= 0 && d5 <= d6) cp = [t[6], t[7], t[8]];
        else {
          const vb = d5 * d2 - d1 * d6;
          if (vb <= 0 && d2 >= 0 && d6 <= 0) {
            const w = d2 / (d2 - d6);
            cp = [t[0] + w * ac[0], t[1] + w * ac[1], t[2] + w * ac[2]];
          } else {
            const va = d3 * d6 - d5 * d4;
            if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
              const w = (d4 - d3) / (d4 - d3 + (d5 - d6));
              cp = [t[3] + w * (t[6] - t[3]), t[4] + w * (t[7] - t[4]), t[5] + w * (t[8] - t[5])];
            } else {
              const den = 1 / (va + vb + vc);
              const v = vb * den, w = vc * den;
              cp = [t[0] + ab[0] * v + ac[0] * w, t[1] + ab[1] * v + ac[1] * w, t[2] + ab[2] * v + ac[2] * w];
            }
          }
        }
      }
    }
  }
  return Math.hypot(p[0] - cp[0], p[1] - cp[1], p[2] - cp[2]);
}

/** Enclosed volume of the STEP B-rep from its surfaces and same_sense flags (positive = outward). */
export function stepBrepVolume(text: string, n = 20): number {
  let vol = 0;
  for (const f of parseStepSurfaces(text)) {
    let sum = 0;
    const h = 1e-5;
    for (let a = 0; a < n; a++)
      for (let b = 0; b < n; b++) {
        const u = (a + 0.5) / n, v = (b + 0.5) / n;
        const P = evalSurface(f.net, u, v);
        const u1 = Math.min(1, u + h), u0 = Math.max(0, u - h), v1 = Math.min(1, v + h), v0 = Math.max(0, v - h);
        const A1 = evalSurface(f.net, u1, v), A0 = evalSurface(f.net, u0, v), B1 = evalSurface(f.net, u, v1), B0 = evalSurface(f.net, u, v0);
        const Su = [(A1[0] - A0[0]) / (u1 - u0), (A1[1] - A0[1]) / (u1 - u0), (A1[2] - A0[2]) / (u1 - u0)];
        const Sv = [(B1[0] - B0[0]) / (v1 - v0), (B1[1] - B0[1]) / (v1 - v0), (B1[2] - B0[2]) / (v1 - v0)];
        const nx = Su[1] * Sv[2] - Su[2] * Sv[1], ny = Su[2] * Sv[0] - Su[0] * Sv[2], nz = Su[0] * Sv[1] - Su[1] * Sv[0];
        sum += (P[0] * nx + P[1] * ny + P[2] * nz) / 3;
      }
    vol += (f.sameSense ? 1 : -1) * (sum / (n * n));
  }
  return vol;
}

/**
 * Volume of the flow path of a flow-through duct (between the cowl and the floor grids), by cell-wise mean
 * height times projected (x, y) area. The duct mesh itself is open at inlet and nozzle, so its divergence volume
 * is not this number; the STEP solid (capped by planar inlet / nozzle faces) should match it.
 */
export function ductFlowVolume(grids: { name: string; ni: number; nj: number; xyz: Float64Array | number[] }[]): number {
  const up = grids.find((g) => g.name === "cowl");
  const lo = grids.find((g) => g.name === "lower");
  if (!up || !lo || up.ni !== lo.ni || up.nj !== lo.nj) return NaN;
  const P = (g: { nj: number; xyz: Float64Array | number[] }, i: number, j: number) => (i * g.nj + j) * 3;
  let v = 0;
  for (let i = 0; i < up.ni - 1; i++)
    for (let j = 0; j < up.nj - 1; j++) {
      const idx = [P(up, i, j), P(up, i + 1, j), P(up, i + 1, j + 1), P(up, i, j + 1)];
      const lidx = [P(lo, i, j), P(lo, i + 1, j), P(lo, i + 1, j + 1), P(lo, i, j + 1)];
      let area = 0;
      for (let k = 0; k < 4; k++) {
        const a = idx[k], b = idx[(k + 1) % 4];
        area += up.xyz[a] * up.xyz[b + 1] - up.xyz[b] * up.xyz[a + 1];
      }
      area = Math.abs(area) / 2;
      let h = 0;
      for (let k = 0; k < 4; k++) h += up.xyz[idx[k] + 2] - lo.xyz[lidx[k] + 2];
      v += (h / 4) * area;
    }
  return v;
}
