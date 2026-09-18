import type { LengthUnit, SurfaceGrid, TriMesh, Vec3 } from "./types";
import { vcross, vnorm, vsub } from "./math";
import { gridPoint } from "./mesh";
import { surfacePatches } from "./export-plot3d";

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

function unitBlock(doc: StepDoc, unit: LengthUnit) {
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
  const uncVal = doc.add("LENGTH_MEASURE(1.E-8)");
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

function splineKnots(nPoles: number, degree: number): { knots: number[]; mult: number[] } {
  const p = Math.min(degree, Math.max(1, nPoles - 1));
  const knots: number[] = [];
  const mult: number[] = [];
  knots.push(0);
  mult.push(p + 1);
  const internal = nPoles - p - 1;
  for (let i = 1; i <= internal; i++) {
    knots.push(i / (internal + 1));
    mult.push(1);
  }
  knots.push(1);
  mult.push(p + 1);
  return { knots, mult };
}

function emitBSplineSurface(doc: StepDoc, poles: Vec3[][], degree = 3): number {
  const ni = poles.length;
  const nj = poles[0].length;
  const p1 = Math.min(degree, ni - 1);
  const p2 = Math.min(degree, nj - 1);
  const ids: number[][] = [];
  for (let i = 0; i < ni; i++) {
    const row: number[] = [];
    for (let j = 0; j < nj; j++) {
      const p = poles[i][j];
      row.push(doc.add(`CARTESIAN_POINT('',(${s(p[0])},${s(p[1])},${s(p[2])}))`));
    }
    ids.push(row);
  }
  const ku = splineKnots(ni, p1);
  const kv = splineKnots(nj, p2);
  const grid = ids.map((row) => `(${row.map((id) => `#${id}`).join(",")})`).join(",");
  return doc.add(
    `B_SPLINE_SURFACE_WITH_KNOTS('',${p1},${p2},(${grid}),.UNSPECIFIED.,.F.,.F.,.F.,(${ku.mult.join(",")}),(${kv.mult.join(",")}),(${ku.knots.map(s).join(",")}),(${kv.knots.map(s).join(",")}),.UNSPECIFIED.)`,
  );
}

function emitBSplineCurve(doc: StepDoc, poles: Vec3[], degree = 3): number {
  const n = poles.length;
  const p = Math.min(degree, n - 1);
  const ids = poles.map((pt) => doc.add(`CARTESIAN_POINT('',(${s(pt[0])},${s(pt[1])},${s(pt[2])}))`));
  const k = splineKnots(n, p);
  return doc.add(
    `B_SPLINE_CURVE_WITH_KNOTS('',${p},(${ids.map((id) => `#${id}`).join(",")}),.UNSPECIFIED.,.F.,.F.,(${k.mult.join(",")}),(${k.knots.map(s).join(",")}),.UNSPECIFIED.)`,
  );
}

function keepDenseU(g: SurfaceGrid): boolean {
  return (
    g.ni <= 4 ||
    g.name === "leading" ||
    g.name === "cowl_lip" ||
    g.name === "tip_l" ||
    g.name === "tip_r" ||
    g.name.startsWith("side") ||
    g.name.startsWith("base") ||
    g.name.startsWith("inlet") ||
    g.name.startsWith("nozzle")
  );
}

function gridToPoles(g: SurfaceGrid, capU = 80, capV = 64): Vec3[][] {
  const nu = keepDenseU(g) ? g.ni : Math.min(g.ni, capU);
  const nv = Math.min(g.nj, capV);
  const poles: Vec3[][] = [];
  for (let iu = 0; iu < nu; iu++) {
    const i = iu === nu - 1 ? g.ni - 1 : Math.round((iu * (g.ni - 1)) / Math.max(nu - 1, 1));
    const row: Vec3[] = [];
    for (let jv = 0; jv < nv; jv++) {
      const j = jv === nv - 1 ? g.nj - 1 : Math.round((jv * (g.nj - 1)) / Math.max(nv - 1, 1));
      row.push(gridPoint(g, i, j));
    }
    poles.push(row);
  }
  return poles;
}

function qk(p: Vec3): string {
  return `${Math.round(p[0] * 1e7)}:${Math.round(p[1] * 1e7)}:${Math.round(p[2] * 1e7)}`;
}

function curveKey(poles: Vec3[]): { key: string; reversed: boolean } {
  const a = qk(poles[0]);
  const b = qk(poles[poles.length - 1]);
  const m = qk(poles[Math.floor(poles.length / 2)]);
  if (a < b || (a === b && m <= qk(poles[Math.floor((poles.length - 1) / 2)]))) {
    return { key: `${a}|${b}|${m}`, reversed: false };
  }
  return { key: `${b}|${a}|${m}`, reversed: true };
}

function vertex(doc: StepDoc, p: Vec3, cache: Map<string, { pt: number; vx: number }>): { pt: number; vx: number } {
  const k = qk(p);
  const hit = cache.get(k);
  if (hit) return hit;
  const pt = doc.add(`CARTESIAN_POINT('',(${s(p[0])},${s(p[1])},${s(p[2])}))`);
  const vx = doc.add(`VERTEX_POINT('',#${pt})`);
  const rec = { pt, vx };
  cache.set(k, rec);
  return rec;
}

function emitEdge(
  doc: StepDoc,
  poles: Vec3[],
  verts: Map<string, { pt: number; vx: number }>,
  edges: Map<string, { id: number }>,
): { id: number; same: boolean } {
  const { key, reversed } = curveKey(poles);
  const hit = edges.get(key);
  if (hit) return { id: hit.id, same: !reversed };
  const pts = reversed ? [...poles].reverse() : poles;
  const v0 = vertex(doc, pts[0], verts);
  const v1 = vertex(doc, pts[pts.length - 1], verts);
  const curve = emitBSplineCurve(doc, pts, Math.min(3, pts.length - 1));
  const id = doc.add(`EDGE_CURVE('',#${v0.vx},#${v1.vx},#${curve},.T.)`);
  edges.set(key, { id });
  return { id, same: !reversed };
}

function emitSewnFace(
  doc: StepDoc,
  poles: Vec3[][],
  name: string,
  verts: Map<string, { pt: number; vx: number }>,
  edges: Map<string, { id: number }>,
): number | null {
  const ni = poles.length;
  const nj = poles[0].length;
  const surf = emitBSplineSurface(doc, poles, 3);
  const raw: Vec3[][] = [
    poles.map((row) => row[0]),
    poles[ni - 1],
    poles.map((row) => row[nj - 1]).reverse(),
    [...poles[0]].reverse(),
  ];
  const kept = raw.filter((c) => {
    let len = 0;
    for (let i = 1; i < c.length; i++) {
      const d = Math.hypot(c[i][0] - c[i - 1][0], c[i][1] - c[i - 1][1], c[i][2] - c[i - 1][2]);
      len += d;
    }
    return len > 1e-10;
  });
  if (kept.length < 2) return null;
  const oriented = kept.map((c) => emitEdge(doc, c, verts, edges));
  const os = oriented.map((e) => doc.add(`ORIENTED_EDGE('',*,*,#${e.id},${e.same ? ".T." : ".F."})`));
  const loop = doc.add(`EDGE_LOOP('',(${os.map((id) => `#${id}`).join(",")}))`);
  const bound = doc.add(`FACE_OUTER_BOUND('',#${loop},.T.)`);
  return doc.add(`ADVANCED_FACE('${name}',(#${bound}),#${surf},.T.)`);
}

/**
 * Sewn NURBS B-rep: degree-3 B-splines, shared EDGE_CURVE on coincident
 * boundaries, CLOSED_SHELL + MANIFOLD_SOLID_BREP. Pointwise / SolidWorks /
 * FreeCAD import this as surfaces (not a control-point cloud). Dense
 * degree-1 poles were the cyan scatter.
 */
export function gridsToNurbsStep(grids: SurfaceGrid[], name: string, unit: LengthUnit): string {
  const usable = surfacePatches(grids).filter((g) => g.ni >= 2 && g.nj >= 2);
  if (!usable.length) {
    return meshToFacetedStep(
      { positions: new Float64Array(), indices: new Uint32Array(), surfaces: new Uint8Array() },
      name,
      unit,
    );
  }

  const doc = new StepDoc();
  const { pds, ctx } = unitBlock(doc, unit);
  const verts = new Map<string, { pt: number; vx: number }>();
  const edges = new Map<string, { id: number }>();
  const faces: number[] = [];
  for (const g of usable) {
    const face = emitSewnFace(doc, gridToPoles(g), g.name || "surface", verts, edges);
    if (face != null) faces.push(face);
  }

  if (!faces.length) {
    return meshToFacetedStep(
      { positions: new Float64Array(), indices: new Uint32Array(), surfaces: new Uint8Array() },
      name,
      unit,
    );
  }

  const shell = doc.add(`CLOSED_SHELL('',(${faces.map((id) => `#${id}`).join(",")}))`);
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
