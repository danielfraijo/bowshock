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

function gridToPoles(g: SurfaceGrid, capU = 24, capV = 20): Vec3[][] {
  const du = Math.max(1, Math.floor((g.ni - 1) / Math.min(capU, g.ni - 1)));
  const dv = Math.max(1, Math.floor((g.nj - 1) / Math.min(capV, g.nj - 1)));
  const poles: Vec3[][] = [];
  for (let i = 0; i < g.ni; i += du) {
    const row: Vec3[] = [];
    for (let j = 0; j < g.nj; j += dv) row.push(gridPoint(g, i, j));
    if ((g.nj - 1) % dv !== 0) row.push(gridPoint(g, i, g.nj - 1));
    poles.push(row);
  }
  if ((g.ni - 1) % du !== 0) {
    const row: Vec3[] = [];
    for (let j = 0; j < g.nj; j += dv) row.push(gridPoint(g, g.ni - 1, j));
    if ((g.nj - 1) % dv !== 0) row.push(gridPoint(g, g.ni - 1, g.nj - 1));
    poles.push(row);
  }
  return poles;
}

function vertex(doc: StepDoc, p: Vec3): { pt: number; vx: number } {
  const pt = doc.add(`CARTESIAN_POINT('',(${s(p[0])},${s(p[1])},${s(p[2])}))`);
  const vx = doc.add(`VERTEX_POINT('',#${pt})`);
  return { pt, vx };
}

export function gridsToNurbsStep(grids: SurfaceGrid[], name: string, unit: LengthUnit): string {
  const upper = grids.find((g) => g.name === "upper") ?? grids[0];
  const lower = grids.find((g) => g.name === "lower") ?? grids[1] ?? grids[0];
  if (!upper || !lower || upper.ni < 2) {
    return meshToFacetedStep(
      { positions: new Float64Array(), indices: new Uint32Array(), surfaces: new Uint8Array() },
      name,
      unit,
    );
  }

  const doc = new StepDoc();
  const { pds, ctx } = unitBlock(doc, unit);

  const uPoles = gridToPoles(upper);
  const lPoles = gridToPoles(lower);

  const uSurf = emitBSplineSurface(doc, uPoles);
  const lSurf = emitBSplineSurface(doc, lPoles);

  const leU = uPoles[0];
  const teU = uPoles[uPoles.length - 1];
  const teL = lPoles[lPoles.length - 1];
  const baseSurf = emitBSplineSurface(doc, [teU, teL]);

  const vL = vertex(doc, leU[0]);
  const vR = vertex(doc, leU[leU.length - 1]);

  const cLE = emitBSplineCurve(doc, leU);
  const cUTE = emitBSplineCurve(doc, teU);
  const cLTE = emitBSplineCurve(doc, teL);

  const eLE = doc.add(`EDGE_CURVE('',#${vL.vx},#${vR.vx},#${cLE},.T.)`);
  const eUTE = doc.add(`EDGE_CURVE('',#${vL.vx},#${vR.vx},#${cUTE},.T.)`);
  const eLTE = doc.add(`EDGE_CURVE('',#${vL.vx},#${vR.vx},#${cLTE},.T.)`);

  const oe1 = doc.add(`ORIENTED_EDGE('',*,*,#${eLE},.T.)`);
  const oe2 = doc.add(`ORIENTED_EDGE('',*,*,#${eUTE},.F.)`);
  const loopU = doc.add(`EDGE_LOOP('',(#${oe1},#${oe2}))`);
  const oe3 = doc.add(`ORIENTED_EDGE('',*,*,#${eLE},.F.)`);
  const oe4 = doc.add(`ORIENTED_EDGE('',*,*,#${eLTE},.T.)`);
  const loopL = doc.add(`EDGE_LOOP('',(#${oe3},#${oe4}))`);
  const oe5 = doc.add(`ORIENTED_EDGE('',*,*,#${eUTE},.T.)`);
  const oe6 = doc.add(`ORIENTED_EDGE('',*,*,#${eLTE},.F.)`);
  const loopB = doc.add(`EDGE_LOOP('',(#${oe5},#${oe6}))`);

  const fU = doc.add(`ADVANCED_FACE('upper',(#${doc.add(`FACE_OUTER_BOUND('',#${loopU},.T.)`)}),#${uSurf},.T.)`);
  const fL = doc.add(`ADVANCED_FACE('lower',(#${doc.add(`FACE_OUTER_BOUND('',#${loopL},.T.)`)}),#${lSurf},.T.)`);
  const fB = doc.add(`ADVANCED_FACE('base',(#${doc.add(`FACE_OUTER_BOUND('',#${loopB},.T.)`)}),#${baseSurf},.T.)`);

  const shell = doc.add(`CLOSED_SHELL('',(#${fU},#${fL},#${fB}))`);
  const solid = doc.add(`MANIFOLD_SOLID_BREP('Waverider',#${shell})`);
  const repr = doc.add(`ADVANCED_BREP_SHAPE_REPRESENTATION('',(#${solid}),#${ctx})`);
  doc.add(`SHAPE_DEFINITION_REPRESENTATION(#${pds},#${repr})`);

  return `${stepHeader(name)}\n${doc.lines.join("\n")}\nENDSEC;\nEND-ISO-10303-21;\n`;
}
