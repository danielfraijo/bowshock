/**
 * Geometry-quality measurements for the topology / watertightness work.
 * Pure functions over TriMesh + exported text/binary — used by
 * topology.test.ts and scripts/topology-report.ts. Not imported by the app.
 */
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
  let minAngle = 180;
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
  let lastSeq: Record<string, number> = {};
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

/** Structural STEP checks: refs resolve, CLOSED_SHELL edges each used by exactly two oriented faces (opposite sense). */
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
      const faceSense = /,\.T\.\)$/.test(face.body) ? 1 : -1;
      const bounds = [...face.body.matchAll(/#(\d+)/g)].map((m) => ents.get(+m[1])).filter((x) => x && x.type === "FACE_OUTER_BOUND");
      for (const bd of bounds) {
        const loop = ents.get(+/#(\d+)/.exec(bd!.body)![1]);
        if (!loop) continue;
        for (const m of loop.body.matchAll(/#(\d+)/g)) {
          const oe = ents.get(+m[1]);
          if (!oe || oe.type !== "ORIENTED_EDGE") continue;
          const edgeId = +/#(\d+),\.[TF]\.\)$/.exec(oe.body)![1];
          const sense = (/,\.T\.\)$/.test(oe.body) ? 1 : -1) * faceSense;
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
