import type { LengthUnit, SurfaceGrid, TriMesh, Vec3 } from "./types";
import { vcross, vnorm, vsub } from "./math";
import { gridPoint } from "./mesh";

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
FILE_DESCRIPTION(('Bowshock watertight waverider'),'2;1');
FILE_NAME('${name}.step','${now}',('Bowshock'),('Bowshock'),'Bowshock CAD','Bowshock','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
ENDSEC;
DATA;`;
}

function unitBlock(doc: StepDoc, unit: LengthUnit) {
  const app = doc.add("APPLICATION_CONTEXT('automotive design')");
  doc.add(`APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2010,#${app})`);
  const pctx = doc.add(`PRODUCT_CONTEXT('',#${app},'mechanical')`);
  const dctx = doc.add(`PRODUCT_DEFINITION_CONTEXT('',#${app},'design')`);
  const prod = doc.add(`PRODUCT('waverider','Waverider','inverse-design waverider',(#${pctx}))`);
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

function gridToPoles(g: SurfaceGrid, capU = 16, capV = 14): Vec3[][] {
  const nu = Math.min(g.ni, capU);
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

function vertex(doc: StepDoc, p: Vec3): { pt: number; vx: number } {
  const pt = doc.add(`CARTESIAN_POINT('',(${s(p[0])},${s(p[1])},${s(p[2])}))`);
  const vx = doc.add(`VERTEX_POINT('',#${pt})`);
  return { pt, vx };
}

function emitFace(doc: StepDoc, poles: Vec3[][], name: string): number {
  const ni = poles.length;
  const nj = poles[0].length;
  const surf = emitBSplineSurface(doc, poles, 3);
  const v00 = vertex(doc, poles[0][0]);
  const v10 = vertex(doc, poles[ni - 1][0]);
  const v11 = vertex(doc, poles[ni - 1][nj - 1]);
  const v01 = vertex(doc, poles[0][nj - 1]);
  const c0 = emitBSplineCurve(doc, poles.map((row) => row[0]), 1);
  const c1 = emitBSplineCurve(doc, poles[ni - 1], 1);
  const c2 = emitBSplineCurve(doc, poles.map((row) => row[nj - 1]).reverse(), 1);
  const c3 = emitBSplineCurve(doc, [...poles[0]].reverse(), 1);
  const e0 = doc.add(`EDGE_CURVE('',#${v00.vx},#${v10.vx},#${c0},.T.)`);
  const e1 = doc.add(`EDGE_CURVE('',#${v10.vx},#${v11.vx},#${c1},.T.)`);
  const e2 = doc.add(`EDGE_CURVE('',#${v11.vx},#${v01.vx},#${c2},.T.)`);
  const e3 = doc.add(`EDGE_CURVE('',#${v01.vx},#${v00.vx},#${c3},.T.)`);
  const o0 = doc.add(`ORIENTED_EDGE('',*,*,#${e0},.T.)`);
  const o1 = doc.add(`ORIENTED_EDGE('',*,*,#${e1},.T.)`);
  const o2 = doc.add(`ORIENTED_EDGE('',*,*,#${e2},.T.)`);
  const o3 = doc.add(`ORIENTED_EDGE('',*,*,#${e3},.T.)`);
  const loop = doc.add(`EDGE_LOOP('',(#${o0},#${o1},#${o2},#${o3}))`);
  const bound = doc.add(`FACE_OUTER_BOUND('',#${loop},.T.)`);
  return doc.add(`ADVANCED_FACE('${name}',(#${bound}),#${surf},.T.)`);
}

/**
 * Untrimmed NURBS faces as a SHELL_BASED_SURFACE_MODEL.
 * Degree-3 B-splines, ~16×14 poles per patch — Pointwise File > Import > STEP
 * reads surfaces. Dense degree-1 poles look like the cyan point cloud.
 */
export function gridsToNurbsStep(grids: SurfaceGrid[], name: string, unit: LengthUnit): string {
  const usable = grids.filter((g) => g.ni >= 2 && g.nj >= 2);
  if (!usable.length) {
    return meshToFacetedStep(
      { positions: new Float64Array(), indices: new Uint32Array(), surfaces: new Uint8Array() },
      name,
      unit,
    );
  }

  const doc = new StepDoc();
  const { pds, ctx } = unitBlock(doc, unit);
  const faces: number[] = [];
  for (const g of usable) faces.push(emitFace(doc, gridToPoles(g), g.name || "surface"));

  const upper = usable.find((g) => g.name === "upper" || g.name === "cowl") ?? usable[0];
  const lower = usable.find((g) => g.name === "lower") ?? usable[usable.length - 1];
  if (upper && lower && upper !== lower) {
    const uP = gridToPoles(upper);
    const lP = gridToPoles(lower);
    const nj = Math.min(uP[0].length, lP[0].length);
    const mid = Math.floor(nj / 2);
    const teU = uP[uP.length - 1].slice(0, nj);
    const teL = lP[lP.length - 1].slice(0, nj);
    const leU = uP[0].slice(0, nj);
    const leL = lP[0].slice(0, nj);
    const teDist = Math.hypot(teU[mid][0] - teL[mid][0], teU[mid][1] - teL[mid][1], teU[mid][2] - teL[mid][2]);
    const leDist = Math.hypot(leU[mid][0] - leL[mid][0], leU[mid][1] - leL[mid][1], leU[mid][2] - leL[mid][2]);
    if (teDist > 1e-7) faces.push(emitFace(doc, [teU, teL], "nozzle_or_base"));
    if (leDist > 1e-7) faces.push(emitFace(doc, [leU, leL], "inlet"));
  }

  const shell = doc.add(`OPEN_SHELL('',(${faces.map((id) => `#${id}`).join(",")}))`);
  const model = doc.add(`SHELL_BASED_SURFACE_MODEL('Bowshock',(#${shell}))`);
  const repr = doc.add(`MANIFOLD_SURFACE_SHAPE_REPRESENTATION('',(#${model}),#${ctx})`);
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
FILE_DESCRIPTION(('Bowshock tessellated waverider'),'2;1');
FILE_NAME('${name}.step','${new Date().toISOString().slice(0, 19)}',('Bowshock'),('Bowshock'),'Bowshock CAD','Bowshock','');
FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF'));
ENDSEC;
DATA;`;
  return `${header}\n${doc.lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
}
