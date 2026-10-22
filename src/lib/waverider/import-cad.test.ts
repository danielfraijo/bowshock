/**
 * CAD import: STL/OBJ round-trip into the panel book.
 * Run: node --import ./scripts/ts-register.mjs src/lib/waverider/import-cad.test.ts
 */
import assert from "node:assert/strict";
import { atmosphere } from "./atmosphere";
import { meshToBinaryStl, meshToAsciiStl } from "./export-stl";
import { buildVehicle } from "./generate";
import { parseCad } from "./import-cad";
import { MeshBuilder } from "./mesh";
import { panelAero } from "./panel";
import { DEFAULT_PARAMS } from "./types";

function fileBytes(text: string): ArrayBuffer {
  const u = new TextEncoder().encode(text);
  return u.buffer.slice(u.byteOffset, u.byteOffset + u.byteLength);
}

function boxMesh() {
  const b = new MeshBuilder(1);
  const v = (x: number, y: number, z: number) => b.vert(x, y, z);
  const p000 = v(0, 0, 0);
  const p100 = v(1, 0, 0);
  const p110 = v(1, 1, 0);
  const p010 = v(0, 1, 0);
  const p001 = v(0, 0, 1);
  const p101 = v(1, 0, 1);
  const p111 = v(1, 1, 1);
  const p011 = v(0, 1, 1);
  b.quad(p000, p010, p110, p100, "lower");
  b.quad(p001, p101, p111, p011, "upper");
  b.quad(p000, p100, p101, p001, "base");
  b.quad(p010, p011, p111, p110, "upper");
  b.quad(p000, p001, p011, p010, "leading");
  b.quad(p100, p110, p111, p101, "base");
  return b.finish();
}

{
  const built = buildVehicle({ ...DEFAULT_PARAMS, family: "caret", aeroMethod: "newtonian" });
  const stl = meshToBinaryStl(built.mesh, "caret");
  const imp = parseCad(stl, "caret.stl", { scale: 1, axis: "cuspis" });
  assert.equal(imp.quality.watertight, true, `imported caret open ${imp.quality.openEdges}`);
  assert.equal(imp.quality.manifold, true);
  const volErr = Math.abs(imp.quality.volume - built.quality.volume) / built.quality.volume;
  assert.ok(volErr < 0.01, `volume ${imp.quality.volume} vs ${built.quality.volume}`);
  const sRef0 = built.aero.planformArea;
  const atm = atmosphere(built.params.altKm, built.params.mach, built.params.gamma);
  const a0 = panelAero(built.mesh, built.params, atm, 2, 0, sRef0, built.params.length, undefined, true);
  const a1 = panelAero(imp.mesh, { ...built.params, aeroMethod: "newtonian" }, atm, 2, 0, imp.sRef, imp.length, undefined, true);
  const f0 = a0.cl * sRef0;
  const f1 = a1.cl * imp.sRef;
  const rel = Math.abs(f0 - f1) / Math.max(Math.abs(f0), 1e-6);
  assert.ok(rel < 0.03, `lift ${f1} vs ${f0} (${(100 * rel).toFixed(2)}%)`);
  console.log(`stl caret V err ${(100 * volErr).toFixed(2)}% lift err ${(100 * rel).toFixed(2)}% Sref ${imp.sRef.toFixed(3)} vs ${sRef0.toFixed(3)}`);
}

{
  const box = boxMesh();
  const ascii = meshToAsciiStl(box, "box");
  const imp = parseCad(new TextEncoder().encode(ascii).buffer, "box.stl", { scale: 1, axis: "cuspis" });
  assert.equal(imp.quality.watertight, true, `ascii box open ${imp.quality.openEdges}`);
  assert.ok(Math.abs(imp.quality.volume - 1) < 1e-6, `box volume ${imp.quality.volume}`);
  assert.ok(Math.abs(imp.length - 1) < 1e-6 && Math.abs(imp.span - 1) < 1e-6);
  console.log(`ascii box V=${imp.quality.volume.toFixed(4)} tris=${imp.quality.triangles}`);
}

{
  const mm = meshToAsciiStl(boxMesh(), "boxmm").replace(/vertex\s+([eE0-9.+-]+)\s+([eE0-9.+-]+)\s+([eE0-9.+-]+)/g, (_m, x, y, z) => {
    return `vertex ${Number(x) * 1000} ${Number(y) * 1000} ${Number(z) * 1000}`;
  });
  const imp = parseCad(new TextEncoder().encode(mm).buffer, "box-mm.stl", { scale: 0.001, axis: "cuspis" });
  assert.ok(Math.abs(imp.length - 1) < 1e-4, `mm length ${imp.length}`);
  assert.ok(Math.abs(imp.quality.volume - 1) < 1e-3, `mm volume ${imp.quality.volume}`);
  console.log(`mm box L=${imp.length.toFixed(4)}`);
}

{
  const obj = `v 0 0 0
v 1 0 0
v 1 1 0
v 0 1 0
v 0 0 1
v 1 0 1
v 1 1 1
v 0 1 1
f 1 2 3 4
f 5 8 7 6
f 1 5 6 2
f 4 3 7 8
f 1 4 8 5
f 2 6 7 3
`;
  const imp = parseCad(new TextEncoder().encode(obj).buffer, "box.obj", { scale: 1, axis: "cuspis" });
  assert.equal(imp.quality.watertight, true, `obj open ${imp.quality.openEdges}`);
  assert.ok(Math.abs(imp.quality.volume - 1) < 1e-6, `obj volume ${imp.quality.volume}`);
  console.log(`obj box tris=${imp.quality.triangles}`);
}

{
  let threw = false;
  const step = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n";
  try {
    parseCad(fileBytes(step), "part.step", { scale: 1, axis: "cuspis" });
  } catch (err) {
    threw = true;
    assert.ok(String(err).includes("STL"), String(err));
  }
  assert.equal(threw, true);
}

console.log("import-cad.test ok");
