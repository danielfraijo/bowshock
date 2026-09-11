import type { BuiltVehicle } from "./types";
import { meshToAsciiStl, meshToBinaryStl } from "./export-stl";
import { gridsToNurbsStep, meshToFacetedStep } from "./export-step";
import { gridsToIges } from "./export-iges";
import { gridsToPlot3d, meshToObj } from "./export-plot3d";
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

export function fileBlobs(built: BuiltVehicle) {
  const { mesh, grids, name, unit } = scaledClone(built);
  const hasNurbs = grids.some((g) => g.name === "upper") && grids.some((g) => g.name === "lower");
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
    get stepFacet() {
      return new Blob([facet()], { type: "application/step" });
    },
    get stepNurbs() {
      return new Blob([hasNurbs ? gridsToNurbsStep(grids, name, unit) : facet()], { type: "application/step" });
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

export async function cfdZip(built: BuiltVehicle, pythonSource: string, cSource = "", analysis = ""): Promise<Blob> {
  const f = fileBlobs(built);
  const n = f.name;
  const enc = new TextEncoder();
  const bin = new Uint8Array(await f.stlBin.arrayBuffer());
  const files = [
    { name: `${n}/${n}.stl`, data: bin },
    { name: `${n}/${n}_ascii.stl`, data: utf8(await f.stlAscii.text()) },
    { name: `${n}/${n}_faceted.step`, data: utf8(await f.stepFacet.text()) },
    { name: `${n}/${n}_nurbs.step`, data: utf8(await f.stepNurbs.text()) },
    { name: `${n}/${n}.igs`, data: utf8(await f.iges.text()) },
    { name: `${n}/${n}.xyz`, data: utf8(await f.plot3d.text()) },
    { name: `${n}/${n}.obj`, data: utf8(await f.obj.text()) },
    { name: `${n}/design.json`, data: utf8(await f.json.text()) },
    { name: `${n}/README.txt`, data: utf8(await f.readme.text()) },
    { name: `${n}/run_case.py`, data: utf8(await f.runner.text()) },
    { name: `${n}/waverider_cad.py`, data: enc.encode(pythonSource) },
  ];
  if (cSource) files.push({ name: `${n}/bowshock_aero.c`, data: enc.encode(cSource) });
  if (analysis) files.push({ name: `${n}/analysis.json`, data: utf8(analysis) });
  return buildZip(files);
}
