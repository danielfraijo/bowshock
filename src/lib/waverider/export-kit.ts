import type { BuiltVehicle } from "./types";
import { meshToAsciiStl, meshToBinaryStl } from "./export-stl";
import { gridsToNurbsStep, meshToFacetedStep, meshToTessellatedStep } from "./export-step";
import { gridsToIges } from "./export-iges";
import { gridsToPlot3d, meshToAsciiStlRegions, meshToObj, meshToVtk, pointwiseGlyph } from "./export-plot3d";
import { scaleGrids, scaleMesh } from "./mesh";
import { unitScale } from "./math";
import { designJson, pythonRunner, readmeFor } from "./kit-text";
import { buildZip, utf8 } from "./zip";

export function scaledClone(built: BuiltVehicle) {
  const s = unitScale(built.params.unit);
  return {
    mesh: scaleMesh(built.mesh, s),
    grids: scaleGrids(built.grids, s),
    shock: scaleGrids(built.shockGrids, s),
    unit: built.params.unit,
    name: built.params.name.replace(/[^\w.-]+/g, "_") || "waverider",
  };
}

export function fileBlobs(built: BuiltVehicle, analysis?: { cp?: Float32Array; heat?: Float32Array; twEq?: Float32Array }) {
  const { mesh, grids, name, unit } = scaledClone(built);
  const hasNurbs = grids.some((g) => g.ni >= 2 && g.nj >= 2);
  let faceted: string | undefined;
  const facet = () => (faceted ??= meshToFacetedStep(mesh, name, unit));
  return {
    name,
    get stlBin() {
      return new Blob([meshToBinaryStl(mesh, name)], { type: "model/stl" });
    },
    get stlAscii() {
      return new Blob([meshToAsciiStl(mesh, name)], { type: "model/stl" });
    },
    get stlRegions() {
      return new Blob([meshToAsciiStlRegions(mesh, name)], { type: "model/stl" });
    },
    get stepFacet() {
      return new Blob([facet()], { type: "application/step" });
    },
    get stepNurbs() {
      return new Blob([hasNurbs ? gridsToNurbsStep(grids, name, unit) : facet()], { type: "application/step" });
    },
    get stepTess() {
      return new Blob([meshToTessellatedStep(mesh, name, unit)], { type: "application/step" });
    },
    get iges() {
      return new Blob([gridsToIges(grids, name)], { type: "model/iges" });
    },
    get plot3d() {
      return new Blob([gridsToPlot3d(grids)], { type: "text/plain" });
    },
    get obj() {
      return new Blob([meshToObj(mesh)], { type: "model/obj" });
    },
    get vtk() {
      return new Blob([meshToVtk(mesh, analysis?.cp, analysis?.heat, analysis?.twEq)], { type: "text/plain" });
    },
    get glyph() {
      return new Blob([pointwiseGlyph(name)], { type: "text/plain" });
    },
    get json() {
      return new Blob([designJson(built.params)], { type: "application/json" });
    },
    get readme() {
      return new Blob([readmeFor(built)], { type: "text/plain" });
    },
    get runner() {
      return new Blob([pythonRunner(built.params)], { type: "text/x-python" });
    },
  };
}

export async function cfdZip(
  built: BuiltVehicle,
  pythonSource: string,
  cSource = "",
  analysis = "",
  cppSource = "",
  fields?: { cp?: Float32Array; heat?: Float32Array; twEq?: Float32Array },
): Promise<Blob> {
  const f = fileBlobs(built, fields);
  const n = f.name;
  const enc = new TextEncoder();
  const bin = new Uint8Array(await f.stlBin.arrayBuffer());
  const files = [
    { name: `${n}/${n}.stl`, data: bin },
    { name: `${n}/${n}_ascii.stl`, data: utf8(await f.stlAscii.text()) },
    { name: `${n}/${n}_regions.stl`, data: utf8(await f.stlRegions.text()) },
    { name: `${n}/${n}_nurbs.step`, data: utf8(await f.stepNurbs.text()) },
    { name: `${n}/${n}_tess.step`, data: utf8(await f.stepTess.text()) },
    { name: `${n}/${n}_faceted.step`, data: utf8(await f.stepFacet.text()) },
    { name: `${n}/${n}.igs`, data: utf8(await f.iges.text()) },
    { name: `${n}/${n}.x`, data: utf8(await f.plot3d.text()) },
    { name: `${n}/${n}.obj`, data: utf8(await f.obj.text()) },
    { name: `${n}/${n}.vtk`, data: utf8(await f.vtk.text()) },
    { name: `${n}/${n}.glf`, data: utf8(await f.glyph.text()) },
    { name: `${n}/POINTWISE.txt`, data: utf8(pointwiseHowto(n)) },
    { name: `${n}/design.json`, data: utf8(await f.json.text()) },
    { name: `${n}/README.txt`, data: utf8(await f.readme.text()) },
    { name: `${n}/run_case.py`, data: utf8(await f.runner.text()) },
    { name: `${n}/waverider_cad.py`, data: enc.encode(pythonSource) },
  ];
  if (cSource) files.push({ name: `${n}/bowshock_aero.c`, data: enc.encode(cSource) });
  if (cppSource) files.push({ name: `${n}/bowshock.cpp`, data: enc.encode(cppSource) });
  if (analysis) files.push({ name: `${n}/analysis.json`, data: utf8(analysis) });
  return buildZip(files);
}

function pointwiseHowto(name: string): string {
  return `POINTWISE — do not import STEP as a point cloud
================================================
The cyan point cloud is what you get if Plot3D is opened as XYZ scatter
or if STEP control points are imported as a point list. Use these instead.

1) RECOMMENDED (unstructured, automatic mesh)
   File > Import > STL
   Pick ${name}.stl  (binary, watertight triangles, blunt LE)
   Assemble a domain from the database. T-Rex off the walls.
   The nose is a circular fillet of finite radius — no knife-edge.

2) STRUCTURED (database surface patches)
   File > Import > Plot3D
     Dimension: 3-D
     Format:    Formatted (ASCII)
     IBLANK:    off
     File:      ${name}.x     << extension .x — NOT .xyz
   Blocks are upper, lower, leading (the circular LE), base, side walls.
   Combine coincident connectors, then extrude.

3) IGES NURBS (Pointwise's most reliable CAD surfaces)
   File > Import > IGES  →  ${name}.igs
   Type-128 B-splines, one face per patch including the LE strip.

4) NURBS STEP (sewn B-rep — SolidWorks, FreeCAD, Pointwise Database)
   File > Import > STEP  →  ${name}_nurbs.step
   Degree-3 B-splines with SHARED edges and a CLOSED_SHELL solid.
   Circular leading-edge patch is a real face, not a knife-edge.
   If an older Pointwise build still lists poles, use STL or IGES.

5) Glyph
   File > Glyph > Execute  →  ${name}.glf
   Imports the STL, then IGES, then Plot3D automatically.

Frame: X streamwise (nose / cowl lip at origin), Y span, Z up.
Ramjet/scramjet flow-through: inlet at x=0 and nozzle at x=L are OPEN
faces in the Plot3D blocks named inlet / nozzle. Set those as inflow/outflow.
`;
}