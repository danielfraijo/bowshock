import type { LengthUnit, SurfaceGrid, TriMesh, Vec3 } from "./types";
import { vcross, vnorm, vsub } from "./math";
import { surfacePatches } from "./export-plot3d";
import { evalCurve, evalSurface, gridNet, interpolateNet, knotMultiplicities, type NurbsNet } from "./export-nurbs";

function s(n: number): string {
  if (!Number.isFinite(n) || Math.abs(n) < 1e-15) return "0.";
  const v = Number(n.toPrecision(14));
  let out = v.toString();
  if (!out.includes(".") && !out.toLowerCase().includes("e")) out += ".";
  return out;
}

class StepDoc {
  n = 0;
  lines: string[] = [];
  add(entity: string): number {
    const id = ++this.n;
    this.lines.push(`#${id} = ${entity};`);
    return id;
  }
}

function stepHeader(name: string) {
  const now = new Date().toISOString().slice(0, 19);
  return `ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Cuspis watertight waverider NURBS blunt LE'),'2;1');
FILE_NAME('${name}.step','${now}',('Cuspis'),('Cuspis Roma'),'Cuspis CAD','Cuspis','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
ENDSEC;
DATA;`;
}

function unitBlock(doc: StepDoc, unit: LengthUnit, uncertainty = 1e-8) {
  const app = doc.add("APPLICATION_CONTEXT('automotive design')");
  doc.add(`APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2010,#${app})`);
  const pctx = doc.add(`PRODUCT_CONTEXT('',#${app},'mechanical')`);
  const dctx = doc.add(`PRODUCT_DEFINITION_CONTEXT('',#${app},'design')`);
  const prod = doc.add(`PRODUCT('waverider','Cuspis waverider','inverse-design waverider',(#${pctx}))`);
  const pdf = doc.add(`PRODUCT_DEFINITION_FORMATION('','',#${prod})`);
  const pd = doc.add(`PRODUCT_DEFINITION('design','',#${pdf},#${dctx})`);
  const pds = doc.add(`PRODUCT_DEFINITION_SHAPE('','',#${pd})`);

  let lengthUnit: number;
  if (unit === "mm") {
    lengthUnit = doc.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))");
  } else if (unit === "in") {
    const metre = doc.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.))");
    const exp = doc.add("DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.)");
    lengthUnit = doc.add(`(LENGTH_UNIT() NAMED_UNIT(#${exp}) CONVERSION_BASED_UNIT('INCH',#${metre}))`);
  } else {
    lengthUnit = doc.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.))");
  }
  const angle = doc.add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
  const solid = doc.add("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())");
  const uncVal = doc.add(`LENGTH_MEASURE(${s(uncertainty)})`);
  const unc = doc.add(
    `UNCERTAINTY_MEASURE_WITH_UNIT(#${uncVal},#${lengthUnit},'distance_accuracy_value','closure')`,
  );
  const ctx = doc.add(
    `(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#${unc})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#${lengthUnit},#${angle},#${solid})) REPRESENTATION_CONTEXT('3D',' '))`,
  );
  return { pds, ctx };
}

function get(mesh: TriMesh, i: number): Vec3 {
  return [mesh.positions[i * 3], mesh.positions[i * 3 + 1], mesh.positions[i * 3 + 2]];
}

export function meshToFacetedStep(mesh: TriMesh, name: string, unit: LengthUnit): string {
  const doc = new StepDoc();
  const { pds, ctx } = unitBlock(doc, unit);

  const pointIds: number[] = [];
  const nv = mesh.positions.length / 3;
  for (let i = 0; i < nv; i++) {
    const p = get(mesh, i);
    pointIds.push(doc.add(`CARTESIAN_POINT('',(${s(p[0])},${s(p[1])},${s(p[2])}))`));
  }

  const faceIds: number[] = [];
  const nt = mesh.indices.length / 3;
  for (let t = 0; t < nt; t++) {
    const ia = mesh.indices[t * 3];
    const ib = mesh.indices[t * 3 + 1];
    const ic = mesh.indices[t * 3 + 2];
    const a = get(mesh, ia);
    const b = get(mesh, ib);
    const c = get(mesh, ic);
    const n = vnorm(vcross(vsub(b, a), vsub(c, a)));
    if (n[0] === 0 && n[1] === 0 && n[2] === 0) continue;
    let ref: Vec3 = vnorm(vsub(b, a));
    if (Math.abs(ref[0]) + Math.abs(ref[1]) + Math.abs(ref[2]) < 1e-12) ref = [1, 0, 0];
    const loop = doc.add(`POLY_LOOP('',(#${pointIds[ia]},#${pointIds[ib]},#${pointIds[ic]}))`);
    const bound = doc.add(`FACE_OUTER_BOUND('',#${loop},.T.)`);
    const dirN = doc.add(`DIRECTION('',(${s(n[0])},${s(n[1])},${s(n[2])}))`);
    const dirR = doc.add(`DIRECTION('',(${s(ref[0])},${s(ref[1])},${s(ref[2])}))`);
    const ax = doc.add(`AXIS2_PLACEMENT_3D('',#${pointIds[ia]},#${dirN},#${dirR})`);
    const plane = doc.add(`PLANE('',#${ax})`);
    faceIds.push(doc.add(`ADVANCED_FACE('',(#${bound}),#${plane},.T.)`));
  }

  const shell = doc.add(`CLOSED_SHELL('',(${faceIds.map((id) => `#${id}`).join(",")}))`);
  const solid = doc.add(`MANIFOLD_SOLID_BREP('Waverider',#${shell})`);
  const repr = doc.add(`ADVANCED_BREP_SHAPE_REPRESENTATION('',(#${solid}),#${ctx})`);
  doc.add(`SHAPE_DEFINITION_REPRESENTATION(#${pds},#${repr})`);

  return `${stepHeader(name)}\n${doc.lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
}

function emitBSplineSurface(doc: StepDoc, net: NurbsNet): number {
  const ids = net.ctrl.map((row) => row.map((p) => doc.add(`CARTESIAN_POINT('',(${s(p[0])},${s(p[1])},${s(p[2])}))`)));
  const ku = knotMultiplicities(net.U);
  const kv = knotMultiplicities(net.V);
  const grid = ids.map((row) => `(${row.map((id) => `#${id}`).join(",")})`).join(",");
  return doc.add(
    `B_SPLINE_SURFACE_WITH_KNOTS('',${net.p},${net.q},(${grid}),.UNSPECIFIED.,.F.,.F.,.F.,(${ku.mult.join(",")}),(${kv.mult.join(",")}),(${ku.knots.map(s).join(",")}),(${kv.knots.map(s).join(",")}),.UNSPECIFIED.)`,
  );
}

interface EdgeGeom {
  /** control points / degree / knots of the boundary iso-curve (exactly on the surface) */
  ctrl: Vec3[];
  p: number;
  K: number[];
}

function pointAt(e: EdgeGeom, t: number): Vec3 {
  return evalCurve(e.ctrl, e.p, e.K, t);
}

function close3(a: Vec3, b: Vec3, tol: number) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) <= tol;
}

interface PatchFace {
  name: string;
  net: NurbsNet;
  /** canonical CCW (in u,v) boundary: v=0, u=max, v=max reversed, u=0 reversed */
  sides: { geom: EdgeGeom; reversed: boolean }[];
}

function patchFace(name: string, net: NurbsNet, tol: number): PatchFace | null {
  const ni = net.ctrl.length;
  const nj = net.ctrl[0].length;
  const col = (j: number) => net.ctrl.map((row) => row[j]);
  const raw: { geom: EdgeGeom; reversed: boolean }[] = [
    { geom: { ctrl: col(0), p: net.p, K: net.U }, reversed: false },
    { geom: { ctrl: net.ctrl[ni - 1], p: net.q, K: net.V }, reversed: false },
    { geom: { ctrl: col(nj - 1), p: net.p, K: net.U }, reversed: true },
    { geom: { ctrl: net.ctrl[0], p: net.q, K: net.V }, reversed: true },
  ];
  const sides = raw.filter((e) => !close3(pointAt(e.geom, 0), pointAt(e.geom, 1), tol) || !close3(pointAt(e.geom, 0), pointAt(e.geom, 0.5), tol));
  if (sides.length < 2) return null;
  return { name, net, sides };
}

/** Surface integral of P.(Su x Sv)/3 over the patch (positive when the loop-consistent normal points away from the origin). */
function patchVolumeTerm(net: NurbsNet, n = 24): number {
  let sum = 0;
  const h = 1e-5;
  for (let a = 0; a < n; a++) {
    for (let b = 0; b < n; b++) {
      const u = (a + 0.5) / n;
      const v = (b + 0.5) / n;
      const P = evalSurface(net, u, v);
      const Pu1 = evalSurface(net, Math.min(1, u + h), v);
      const Pu0 = evalSurface(net, Math.max(0, u - h), v);
      const Pv1 = evalSurface(net, u, Math.min(1, v + h));
      const Pv0 = evalSurface(net, u, Math.max(0, v - h));
      const du = Math.min(1, u + h) - Math.max(0, u - h);
      const dv = Math.min(1, v + h) - Math.max(0, v - h);
      const Su: Vec3 = [(Pu1[0] - Pu0[0]) / du, (Pu1[1] - Pu0[1]) / du, (Pu1[2] - Pu0[2]) / du];
      const Sv: Vec3 = [(Pv1[0] - Pv0[0]) / dv, (Pv1[1] - Pv0[1]) / dv, (Pv1[2] - Pv0[2]) / dv];
      const nrm = vcross(Su, Sv);
      sum += (P[0] * nrm[0] + P[1] * nrm[1] + P[2] * nrm[2]) / 3;
    }
  }
  return sum / (n * n);
}

/**
 * Sewn NURBS B-rep: interpolating degree-3 B-spline faces, one shared EDGE_CURVE per coincident boundary,
 * CLOSED_SHELL + MANIFOLD_SOLID_BREP. Face loops are oriented consistently (every edge is traversed once
 * in each direction) and outward (positive enclosed volume); ADVANCED_FACE.same_sense follows the loop
 * direction. Boundaries are matched by geometry (end points + dense polyline distance), and the file's
 * global distance uncertainty is the worst matched deviation actually measured (not a hopeful constant).
 */
export function gridsToNurbsStep(grids: SurfaceGrid[], name: string, unit: LengthUnit): string {
  // Flow-through ducts export the flow-path solid: cowl, floor, side walls and planar inlet / nozzle caps. The
  // rounded cowl lip hangs outside that volume (it would be a free sheet), so it is left out of the B-rep.
  const hasCaps = grids.some((g) => g.name === "inlet") && grids.some((g) => g.name === "nozzle");
  const usable = surfacePatches(grids).filter((g) => g.ni >= 2 && g.nj >= 2 && !(hasCaps && g.name === "cowl_lip"));
  const empty = () =>
    meshToFacetedStep({ positions: new Float64Array(), indices: new Uint32Array(), surfaces: new Uint8Array() }, name, unit);
  if (!usable.length) return empty();

  let extent = 1e-9;
  for (const g of usable) for (let k = 0; k < g.xyz.length; k++) extent = Math.max(extent, Math.abs(g.xyz[k]));
  const tol = 2e-5 * extent;

  const faces: PatchFace[] = [];
  for (const g of usable) {
    const f = patchFace(g.name || "surface", interpolateNet(gridNet(g, 200), 3), 1e-9 * extent);
    if (f) faces.push(f);
  }
  if (!faces.length) return empty();

  // unique edges: same end points (either direction) and the same curve (dense polyline distance both ways)
  interface Edge {
    geom: EdgeGeom;
    poly: Vec3[];
    uses: { face: number; side: number; dir: 1 | -1 }[];
  }
  const polyOf = (e: EdgeGeom, n = 96) => Array.from({ length: n + 1 }, (_, k) => pointAt(e, k / n));
  const segDist = (p: Vec3, a: Vec3, b: Vec3) => {
    const ab: Vec3 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const l2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    const t = l2 > 0 ? Math.max(0, Math.min(1, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1] + (p[2] - a[2]) * ab[2]) / l2)) : 0;
    return Math.hypot(p[0] - a[0] - t * ab[0], p[1] - a[1] - t * ab[1], p[2] - a[2] - t * ab[2]);
  };
  const toPoly = (p: Vec3, poly: Vec3[]) => {
    let d = Infinity;
    for (let k = 0; k + 1 < poly.length; k++) d = Math.min(d, segDist(p, poly[k], poly[k + 1]));
    return d;
  };
  const curveGap = (A: Vec3[], B: Vec3[]) => {
    let g = 0;
    for (let k = 0; k < A.length; k += 4) g = Math.max(g, toPoly(A[k], B));
    for (let k = 0; k < B.length; k += 4) g = Math.max(g, toPoly(B[k], A));
    return g;
  };
  const matchTol = 2.5e-3 * extent; // boundary iso-curves of neighbouring interpolants differ by the parametrisation (nose clustering)
  let worstMatch = 0;
  const edges: Edge[] = [];
  const edgeOf: { edge: number; dir: 1 | -1 }[][] = faces.map(() => []);
  faces.forEach((f, fi) => {
    f.sides.forEach((sd, si) => {
      const geomDir: EdgeGeom = sd.reversed ? { ctrl: [...sd.geom.ctrl].reverse(), p: sd.geom.p, K: sd.geom.K.map((k) => 1 - k).reverse() } : sd.geom;
      const poly = polyOf(geomDir);
      const a = poly[0];
      const b = poly[poly.length - 1];
      let found = -1;
      let dir: 1 | -1 = 1;
      for (let e = 0; e < edges.length && found < 0; e++) {
        const ea = edges[e].poly[0];
        const eb = edges[e].poly[edges[e].poly.length - 1];
        const fwd = close3(a, ea, tol) && close3(b, eb, tol);
        const rev = close3(a, eb, tol) && close3(b, ea, tol);
        if (!fwd && !rev) continue;
        const gap = curveGap(poly, edges[e].poly);
        if (process.env.DBGSTEP && gap > matchTol) console.log('endpoints match, curve gap', (gap / extent).toExponential(2), 'faces', faces[fi].name, edges[e].uses.map((u) => faces[u.face].name));
        if (gap > matchTol) continue;
        worstMatch = Math.max(worstMatch, gap);
        found = e;
        dir = fwd ? 1 : -1;
      }
      if (found < 0) {
        edges.push({ geom: geomDir, poly, uses: [] });
        found = edges.length - 1;
        dir = 1;
      }
      edges[found].uses.push({ face: fi, side: si, dir });
      edgeOf[fi][si] = { edge: found, dir };
    });
  });
  if (process.env.DBGSTEP) for (const e of edges) if (e.uses.length !== 2) console.log('edge uses', e.uses.length, e.uses.map((u) => faces[u.face].name + ':' + u.side), pointAt(e.geom, 0).map((v) => v.toFixed(4)).join(','), '->', pointAt(e.geom, 1).map((v) => v.toFixed(4)).join(','));
  // the file declares the accuracy it actually achieves: worst edge-to-edge deviation of matched boundaries
  const declared = Math.max(1e-6 * extent, 1.5 * worstMatch, tol * 0.5);

  // consistent orientation by propagation over shared edges: flip_a*dir_a = -(flip_b*dir_b)
  const flip = new Array<number>(faces.length).fill(0);
  const comp = new Array<number>(faces.length).fill(-1);
  let nComp = 0;
  for (let s0 = 0; s0 < faces.length; s0++) {
    if (flip[s0]) continue;
    flip[s0] = 1;
    comp[s0] = nComp;
    const queue = [s0];
    while (queue.length) {
      const f = queue.pop()!;
      for (let si = 0; si < faces[f].sides.length; si++) {
        const { edge, dir } = edgeOf[f][si];
        for (const u of edges[edge].uses) {
          if (u.face === f || flip[u.face]) continue;
          flip[u.face] = -flip[f] * dir * u.dir;
          comp[u.face] = nComp;
          queue.push(u.face);
        }
      }
    }
    nComp++;
  }
  // outward: per connected component, positive enclosed volume
  const volTerm = faces.map((f) => patchVolumeTerm(f.net));
  for (let c = 0; c < nComp; c++) {
    let vol = 0;
    faces.forEach((_, i) => {
      if (comp[i] === c) vol += flip[i] * volTerm[i];
    });
    if (vol < 0) faces.forEach((_, i) => comp[i] === c && (flip[i] = -flip[i]));
  }

  const doc = new StepDoc();
  const { pds, ctx } = unitBlock(doc, unit, declared);
  const verts: { p: Vec3; pt: number; vx: number }[] = [];
  const vertex = (p: Vec3) => {
    for (const v of verts) if (close3(v.p, p, tol)) return v;
    const pt = doc.add(`CARTESIAN_POINT('',(${s(p[0])},${s(p[1])},${s(p[2])}))`);
    const vx = doc.add(`VERTEX_POINT('',#${pt})`);
    const rec = { p, pt, vx };
    verts.push(rec);
    return rec;
  };
  const edgeIds = edges.map((e) => {
    const v0 = vertex(pointAt(e.geom, 0));
    const v1 = vertex(pointAt(e.geom, 1));
    const ptIds = e.geom.ctrl.map((pt) => doc.add(`CARTESIAN_POINT('',(${s(pt[0])},${s(pt[1])},${s(pt[2])}))`));
    const km = knotMultiplicities(e.geom.K);
    const curve = doc.add(
      `B_SPLINE_CURVE_WITH_KNOTS('',${e.geom.p},(${ptIds.map((id) => `#${id}`).join(",")}),.UNSPECIFIED.,.F.,.F.,(${km.mult.join(",")}),(${km.knots.map(s).join(",")}),.UNSPECIFIED.)`,
    );
    return doc.add(`EDGE_CURVE('',#${v0.vx},#${v1.vx},#${curve},.T.)`);
  });

  const faceIds: number[] = [];
  faces.forEach((f, fi) => {
    const surf = emitBSplineSurface(doc, f.net);
    // loop in canonical order; reversed (order and every sense) when the face is flipped
    let seq = f.sides.map((_, si) => ({ id: edgeIds[edgeOf[fi][si].edge], same: edgeOf[fi][si].dir === 1 }));
    if (flip[fi] < 0) seq = seq.reverse().map((o) => ({ ...o, same: !o.same }));
    const os = seq.map((o) => doc.add(`ORIENTED_EDGE('',*,*,#${o.id},${o.same ? ".T." : ".F."})`));
    const loop = doc.add(`EDGE_LOOP('',(${os.map((id) => `#${id}`).join(",")}))`);
    const bound = doc.add(`FACE_OUTER_BOUND('',#${loop},.T.)`);
    faceIds.push(doc.add(`ADVANCED_FACE('${f.name}',(#${bound}),#${surf},${flip[fi] > 0 ? ".T." : ".F."})`));
  });

  const shell = doc.add(`CLOSED_SHELL('',(${faceIds.map((id) => `#${id}`).join(",")}))`);
  const solid = doc.add(`MANIFOLD_SOLID_BREP('Cuspis',#${shell})`);
  const repr = doc.add(`ADVANCED_BREP_SHAPE_REPRESENTATION('',(#${solid}),#${ctx})`);
  doc.add(`SHAPE_DEFINITION_REPRESENTATION(#${pds},#${repr})`);
  return `${stepHeader(name)}\n${doc.lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
}

/**
 * AP242 tessellated solid — triangles, not control points.
 * Pointwise 18.2+ / FreeCAD / many CAD tools import this as a mesh.
 */
export function meshToTessellatedStep(mesh: TriMesh, name: string, unit: LengthUnit): string {
  const doc = new StepDoc();
  const { pds, ctx } = unitBlock(doc, unit);
  const nv = mesh.positions.length / 3;
  const nt = mesh.indices.length / 3;
  const chunks: string[] = [];
  const chunkSize = 80;
  for (let start = 0; start < nv; start += chunkSize) {
    const end = Math.min(nv, start + chunkSize);
    const pts: string[] = [];
    for (let i = start; i < end; i++) {
      pts.push(`(${s(mesh.positions[i * 3])},${s(mesh.positions[i * 3 + 1])},${s(mesh.positions[i * 3 + 2])})`);
    }
    chunks.push(`(${pts.join(",")})`);
  }
  const coords = doc.add(`COORDINATES_LIST('',${nv},(${Array.from({ length: nv }, (_, i) => `(${s(mesh.positions[i * 3])},${s(mesh.positions[i * 3 + 1])},${s(mesh.positions[i * 3 + 2])})`).join(",")}))`);
  const triIdx: number[] = [];
  for (let t = 0; t < nt; t++) {
    triIdx.push(mesh.indices[t * 3] + 1, mesh.indices[t * 3 + 1] + 1, mesh.indices[t * 3 + 2] + 1);
  }
  const face = doc.add(`TRIANGULATED_FACE('${name}',(),#${coords},.F.,(${triIdx.join(",")}),$,$)`);
  const shell = doc.add(`TESSELLATED_SHELL('',(#${face}),$)`);
  const solid = doc.add(`TESSELLATED_SOLID('${name}',(#${shell}),$)`);
  const repr = doc.add(`TESSELLATED_SHAPE_REPRESENTATION('',(#${solid}),#${ctx})`);
  doc.add(`SHAPE_DEFINITION_REPRESENTATION(#${pds},#${repr})`);
  void chunks;
  const header = `ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Cuspis tessellated waverider'),'2;1');
FILE_NAME('${name}.step','${new Date().toISOString().slice(0, 19)}',('Cuspis'),('Cuspis Roma'),'Cuspis CAD','Cuspis','');
FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF'));
ENDSEC;
DATA;`;
  return `${header}\n${doc.lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
}
