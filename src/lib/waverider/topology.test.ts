// Topology / geometry-quality regression tests.
// Run: bun src/lib/waverider/topology.test.ts   (the repo's TS tests are bun scripts, like fidelity.test.ts)
import assert from "node:assert/strict";
import { leBlendProfile } from "./blunt";
import { gridsToIges } from "./export-iges";
import { evalSurface, gridNet, interpolateNet } from "./export-nurbs";
import { gridsToPlot3d } from "./export-plot3d";
import { gridsToNurbsStep } from "./export-step";
import { meshToBinaryStl } from "./export-stl";
import { buildVehicle } from "./generate";
import {
  checkBinaryStl,
  checkIges,
  checkPlot3d,
  checkStep,
  ductFlowVolume,
  maxDistanceToMesh,
  parseIgesSurfaces,
  sampleSurfaces,
  stepBrepVolume,
  topoMetrics,
} from "./geom-quality";
import { refineGrids } from "./mesh";
import { orientConsistent, removeDoubleFaces } from "./topology";
import { DEFAULT_PARAMS, type DesignParams, type TriMesh, type WaveriderFamily } from "./types";

const ALL: WaveriderFamily[] = ["caret", "cone", "osculating", "viscopt", "elliptic", "wedgecone", "star", "liftbody", "ramjet", "scramjet", "busemann", "inward", "integrated"];
const DUCTS: WaveriderFamily[] = ["ramjet", "scramjet"];
/** Families whose LE is a swept fillet that must exist at the default grid. */
const FILLETED: WaveriderFamily[] = ["caret", "cone", "osculating", "viscopt", "elliptic", "wedgecone", "busemann", "inward", "integrated"];
/** Families exported as ONE closed NURBS shell. (star = 8 separate fin sheets, ducts = open flow-through.) */
const STEP_SOLIDS: WaveriderFamily[] = ["caret", "cone", "osculating", "viscopt", "elliptic", "wedgecone", "liftbody", "busemann", "inward", "integrated"];

function build(family: WaveriderFamily, patch: Partial<DesignParams> = {}) {
  return buildVehicle({ ...DEFAULT_PARAMS, family, flowThrough: DUCTS.includes(family), ...patch });
}

const CASES: { label: string; patch: Partial<DesignParams> }[] = [
  { label: "default", patch: {} },
  { label: "M5", patch: { mach: 5 } },
  { label: "M12 fine", patch: { mach: 12, nx: 80, ny: 56, leRadius: 0.01 } },
  { label: "no fillet", patch: { leBlunt: false } },
  { label: "big R wide", patch: { leRadius: 0.1, span: 3.2 } },
  { label: "coarse", patch: { nx: 20, ny: 12 } },
];

let nChecks = 0;
const ok = (cond: unknown, msg: string) => {
  nChecks++;
  assert.ok(cond, msg);
};

// ---------------------------------------------------------------- 1. watertight, manifold, wound, outward
for (const family of ALL) {
  for (const c of CASES) {
    const b = build(family, c.patch);
    const m = topoMetrics(b.mesh);
    const tag = `${family}/${c.label}`;
    ok(m.nonManifoldEdges === 0, `${tag}: ${m.nonManifoldEdges} non-manifold edges`);
    ok(m.inconsistentEdges === 0, `${tag}: ${m.inconsistentEdges} edges with inconsistent winding`);
    ok(m.duplicateVerts === 0, `${tag}: ${m.duplicateVerts} duplicate vertices`);
    ok(m.degenerateTris === 0, `${tag}: ${m.degenerateTris} degenerate triangles`);
    if (!DUCTS.includes(family)) {
      ok(m.openEdges === 0, `${tag}: ${m.openEdges} open edges`);
      ok(m.signedVolume > 0, `${tag}: signed volume ${m.signedVolume} (normals point inward)`);
    } else ok(m.openEdges > 0, `${tag}: flow-through duct should have open inlet/outlet`);
  }
}

// ---------------------------------------------------------------- 1b. parameter variants stay watertight
const VARIANTS: [string, Partial<DesignParams>][] = [
  ["half model", { halfModel: true }],
  ["lid bottom", { lid: "bottom" }],
  ["dihedral 5", { dihedralDeg: 5 }],
  ["camber", { camber: 0.1 }],
  ["elevon 10", { elevonDeg: 10 }],
  ["2 fins", { fins: 2 }],
  ["rect planform", { planform: "rect" }],
  ["double planform", { planform: "double" }],
  ["spatular planform", { planform: "spatular" }],
  ["power planform", { planform: "power", planformPower: 1.6 }],
  ["TE sweep 20", { teSweepDeg: 20 }],
  ["tiny R", { leRadius: 0.002 }],
  ["R 0.2", { leRadius: 0.2 }],
  ["tiny grid", { ny: 7, nx: 10 }],
  ["big grid", { nx: 200, ny: 90 }],
  ["M3", { mach: 3 }],
  ["M20", { mach: 20 }],
];
for (const family of ["caret", "osculating", "wedgecone", "busemann", "star", "liftbody"] as WaveriderFamily[]) {
  for (const [label, patch] of VARIANTS) {
    const m = topoMetrics(build(family, patch).mesh);
    const tag = `${family}/${label}`;
    ok(m.openEdges === 0 && m.nonManifoldEdges === 0, `${tag}: open ${m.openEdges}, non-manifold ${m.nonManifoldEdges}`);
    ok(m.inconsistentEdges === 0 && m.degenerateTris === 0, `${tag}: winding ${m.inconsistentEdges}, degenerate ${m.degenerateTris}`);
    ok(m.signedVolume > 0, `${tag}: inward normals`);
  }
}

// ---------------------------------------------------------------- 1c. regressions found by the fuzz harness
{
  const watertight = (tag: string, family: WaveriderFamily, patch: Partial<DesignParams>) => {
    const m = topoMetrics(build(family, patch).mesh);
    ok(m.openEdges === 0 && m.nonManifoldEdges === 0 && m.inconsistentEdges === 0, `${tag}: open ${m.openEdges} nonM ${m.nonManifoldEdges} incon ${m.inconsistentEdges}`);
  };
  // vertical-fin bumps moved LE-seam points after the fillet was built (detached fillet strip)
  for (const fh of [0.24, 0.97, 1.4]) watertight(`caret finHeight ${fh}`, "caret", { finHeight: fh, ny: 52 });
  // half model: j = 0 is the symmetry plane, the TE tidy must not zip it as a wing tip
  watertight("caret half nx200", "caret", { halfModel: true, nx: 200 });
  // knife-edge tip columns diverged after the monotone-chord pass; edge pinches next to the tip
  for (const ny of [8, 9, 10, 11, 12, 13, 16, 17, 24]) for (const nx of [40, 90]) watertight(`busemann ny${ny} nx${nx}`, "busemann", { ny, nx });
  watertight("viscopt spatular ny6 g2", "viscopt", { planform: "spatular", ny: 6, leBlend: "g2" });
  watertight("viscopt double half", "viscopt", { length: 0.2, span: 0.05, planform: "double", nx: 32, ny: 39, halfModel: true });
  // micro fillet arcs on a paper-thin rect planform
  watertight("inward rect thin g2", "inward", { span: 0.05, planform: "rect", leBlend: "g2" });
  // coarse grids: the fillet radius is boosted instead of stopping with a step
  for (const ny of [12, 14, 18]) {
    const b = build("cone", { ny, nx: Math.round(ny * 1.4) });
    const lead = b.grids.find((g) => g.name === "leading");
    ok(!!lead && topoMetrics(b.mesh).maxJumpLE < 15, `cone ny${ny}: continuous fillet`);
  }
  // small deterministic slice of the fuzz (seeded): mesh only
  const rnd = (() => { let a = 12345; return () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; })();
  const U = (lo: number, hi: number) => lo + (hi - lo) * rnd();
  const kinds = ["delta", "power", "spatular", "rect", "double"] as const;
  for (let k = 0; k < 52; k++) {
    const family = ALL[k % ALL.length];
    const patch: Partial<DesignParams> = {
      mach: U(3, 16), span: U(0.2, 8), height: U(0.05, 3), shockDeg: U(8, 40), nx: Math.round(U(12, 100)), ny: Math.round(U(8, 70)),
      leRadius: rnd() < 0.2 ? 0 : U(0.001, 0.2), halfModel: rnd() < 0.2, planform: kinds[Math.floor(rnd() * 5)], camber: U(-0.2, 0.4),
      elevonDeg: U(-12, 12), finHeight: rnd() < 0.5 ? 0 : U(0, 1.4), leBlend: rnd() < 0.3 ? "g2" : "circular", lid: rnd() < 0.2 ? "bottom" : "top",
    };
    const m = topoMetrics(build(family, patch).mesh);
    const duct = DUCTS.includes(family);
    ok((duct || m.openEdges === 0) && m.nonManifoldEdges === 0 && m.inconsistentEdges === 0 && m.degenerateTris === 0 && m.signedVolume > 0, `fuzz slice ${k} ${family} ${JSON.stringify(patch)}: open ${m.openEdges} nonM ${m.nonManifoldEdges} incon ${m.inconsistentEdges} degen ${m.degenerateTris} vol ${m.signedVolume}`);
  }
}

// ---------------------------------------------------------------- 2. slivers / aspect at the default grid
for (const family of ALL) {
  const b = build(family);
  const m = topoMetrics(b.mesh);
  const frac = m.bodySlivers / Math.max(1, m.triangles);
  ok(m.bodyMinAngleDeg > 0.5, `${family}: body min angle ${m.bodyMinAngleDeg.toFixed(2)} deg`);
  ok(frac < 0.06, `${family}: ${(100 * frac).toFixed(1)} % of triangles are slivers (<3 deg)`);
  ok(m.bodyEdgeRatio < 80, `${family}: body edge ratio ${m.bodyEdgeRatio.toFixed(0)}`);
}
{
  const m = topoMetrics(build("caret", CASES[2].patch).mesh);
  ok(m.bodySlivers / m.triangles < 0.06, `caret fine: ${m.bodySlivers} slivers of ${m.triangles}`);
  ok(m.bodyMinAngleDeg > 0.25, `caret fine: body min angle ${m.bodyMinAngleDeg.toFixed(2)}`);
}

// ---------------------------------------------------------------- 3. G1 on the LE fillet and nose
for (const family of FILLETED) {
  for (const c of [CASES[0], CASES[1], CASES[2], CASES[4]]) {
    const b = build(family, c.patch);
    const tag = `${family}/${c.label}`;
    ok(b.grids.some((g) => g.name === "leading"), `${tag}: LE fillet missing (README: always a circular fillet)`);
    const m = topoMetrics(b.mesh);
    ok(m.maxJumpLE < 20, `${tag}: LE seam normal jump ${m.maxJumpLE.toFixed(1)} deg`);
    ok(m.maxJumpNose < 50, `${tag}: nose normal jump ${m.maxJumpNose.toFixed(1)} deg (a swept delta nose used to have a ~145 deg ridge)`);
  }
}
ok(topoMetrics(build("caret", CASES[2].patch).mesh).maxJumpLE < 15, "caret fine LE seam jump");

// ---------------------------------------------------------------- 4. the G2 option
for (const h of [3, 6, 12]) {
  const circ = leBlendProfile((h * Math.PI) / 180, "circular", 401);
  const g2 = leBlendProfile((h * Math.PI) / 180, "g2", 4001);
  const curv = (pts: number[][]) => {
    const k: number[] = [];
    for (let i = 1; i < pts.length - 1; i++) {
      const a = [pts[i][0] - pts[i - 1][0], pts[i][2] - pts[i - 1][2]];
      const c = [pts[i + 1][0] - pts[i][0], pts[i + 1][2] - pts[i][2]];
      k.push(Math.atan2(a[0] * c[1] - a[1] * c[0], a[0] * c[0] + a[1] * c[1]) / (0.5 * (Math.hypot(...a) + Math.hypot(...c))));
    }
    return k;
  };
  const kc = curv(circ.pts);
  const kg = curv(g2.pts);
  const peakC = Math.max(...kc.map(Math.abs));
  const peakG = Math.max(...kg.map(Math.abs));
  ok(Math.abs(peakC * circ.radius - 1) < 0.02, `circular blend curvature is 1/R (h=${h})`);
  ok(Math.abs(kc[0]) > 0.9 * peakC, `circular blend has the expected curvature jump at the seam (h=${h})`);
  ok(Math.abs(kg[0]) < 0.01 * peakG && Math.abs(kg[kg.length - 1]) < 0.01 * peakG, `g2 blend curvature must vanish at both ends (h=${h})`);
  ok(Math.abs(peakG * g2.radius - 1) < 0.03, `g2 apex curvature equals 1/radius (h=${h})`);
  ok(new Set(kg.map((v) => Math.sign(v))).size === 1, `g2 blend has no inflection (h=${h})`);
  let maxStep = 0;
  for (let i = 1; i < kg.length; i++) maxStep = Math.max(maxStep, Math.abs(kg[i] - kg[i - 1]));
  ok(maxStep < 0.05 * peakG, `g2 curvature is continuous (h=${h}): max step ${(maxStep / peakG).toFixed(3)} of peak`);
}
{
  const m = topoMetrics(build("caret", { leBlend: "g2" }).mesh);
  ok(m.openEdges === 0 && m.nonManifoldEdges === 0, "g2 caret is watertight");
  ok(m.maxJumpLE < 3, `g2 fillet seam normal jump ${m.maxJumpLE.toFixed(2)} deg`);
}

// ---------------------------------------------------------------- 5. helpers
{
  // orientConsistent repairs a flipped patch and fixes inward-pointing closed meshes
  const b = build("caret");
  const mesh: TriMesh = { ...b.mesh, indices: b.mesh.indices.slice() };
  for (let t = 0; t < 200; t++) {
    const i = mesh.indices[t * 3 + 1];
    mesh.indices[t * 3 + 1] = mesh.indices[t * 3 + 2];
    mesh.indices[t * 3 + 2] = i;
  }
  ok(topoMetrics(mesh).inconsistentEdges > 0, "test fixture: flipped patch is inconsistent");
  orientConsistent(mesh);
  const mm = topoMetrics(mesh);
  ok(mm.inconsistentEdges === 0 && mm.signedVolume > 0, "orientConsistent repairs winding and orientation");
  // removeDoubleFaces drops back-to-back duplicate faces
  const tri: TriMesh = {
    positions: new Float64Array([0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1]),
    indices: new Uint32Array([0, 1, 2, 0, 2, 1, 0, 1, 3]),
    surfaces: new Uint8Array(3),
  };
  ok(removeDoubleFaces(tri).indices.length === 3, "removeDoubleFaces removes the coincident pair");
  // the NURBS interpolation passes through the grid it was given
  const g = b.grids[0];
  const net = gridNet(g, 200);
  const nurbs = interpolateNet(net.filter((_, i) => i % 3 === 0).slice(0, 12).map((r) => r.filter((_, j) => j % 3 === 0).slice(0, 9)), 3);
  ok(nurbs.ctrl.length === 12 && nurbs.ctrl[0].length === 9, "interpolating net has one control point per grid point");
  const corner = evalSurface(nurbs, 0, 0);
  const p0 = net[0][0];
  ok(Math.hypot(corner[0] - p0[0], corner[1] - p0[1], corner[2] - p0[2]) < 1e-9, "interpolating surface starts at the grid corner");
}

// ---------------------------------------------------------------- 6. exports
for (const family of ALL) {
  const b = build(family);
  const refined = refineGrids(b.grids, 3, 2); // what export-kit hands the NURBS writers
  const nt = b.mesh.indices.length / 3;
  const stl = checkBinaryStl(meshToBinaryStl(b.mesh, "t"), nt);
  ok(stl.ok, `${family} binary STL: ${stl.errors.join("; ")}`);
  const p3 = checkPlot3d(gridsToPlot3d(refined));
  ok(p3.ok, `${family} Plot3D: ${p3.errors.join("; ")}`);
  const iges = gridsToIges(refined, "t", "m");
  const ci = checkIges(iges);
  ok(ci.ok, `${family} IGES: ${ci.errors.join("; ")}`);
  const L = b.params.length;
  const dev = maxDistanceToMesh(sampleSurfaces(parseIgesSurfaces(iges), 5), b.mesh);
  // ducts also export their inlet/nozzle planes as BC patches (the flow-through mesh leaves them open)
  if (!DUCTS.includes(family)) ok(dev < 0.01 * L, `${family} IGES surfaces deviate ${dev.toExponential(2)} from the mesh`);
  if (family === "star" || DUCTS.includes(family)) {
    // star: base fan closes the TE star; ducts: flow-path solid (cowl, floor, side walls, planar inlet / nozzle caps)
    const step = gridsToNurbsStep(refined, "t", "m");
    const cs = checkStep(step);
    ok(cs.ok, `${family} STEP solid: ${cs.errors.join("; ")}`);
    const ref = family === "star" ? topoMetrics(b.mesh).signedVolume : ductFlowVolume(b.grids);
    const vol = stepBrepVolume(step, 48);
    ok(vol > 0 && Math.abs(vol - ref) < 0.03 * ref, `${family} STEP volume ${vol.toFixed(4)} vs ${ref.toFixed(4)}`);
  }
  if (STEP_SOLIDS.includes(family)) {
    const step = gridsToNurbsStep(refined, "t", "m");
    const cs = checkStep(step);
    ok(cs.ok, `${family} STEP: ${cs.errors.join("; ")}`);
    const vol = stepBrepVolume(step, 48);
    const mv = topoMetrics(b.mesh).signedVolume;
    ok(vol > 0 && Math.abs(vol - mv) < 0.03 * mv, `${family} STEP volume ${vol.toFixed(4)} vs mesh ${mv.toFixed(4)} (sign/orientation or geometry wrong)`);
  }
}

console.log(`topology assertions passed (${nChecks} checks)`);
