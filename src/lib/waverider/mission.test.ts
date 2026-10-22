/**
 * Mission-lab checks: new vehicle classes, the aero database, TPS, the
 * optimizer's dominance rule, and a short closed-loop flight.
 * Run: node --import ./scripts/ts-register.mjs src/lib/waverider/mission.test.ts
 */
import assert from "node:assert/strict";
import { axisBreaks, aeroDbToCsv, aeroDbToSimulink, generateAeroDatabase } from "./aerodb";
import { actuatorStep, flyClosedLoop } from "./guidance";
import { constraintViolation, dominatesPoint, evaluateDesign } from "./mdo";
import { buildVehicle } from "./generate";
import { analyzeMesh } from "./mesh";
import { DEFAULT_PARAMS, type DesignParams, type WaveriderFamily } from "./types";
import { runValidation } from "./validate";

const finite = (x: number) => Number.isFinite(x);

const coarse = (family: WaveriderFamily, patch: Partial<DesignParams> = {}): DesignParams => ({
  ...DEFAULT_PARAMS,
  family,
  aeroMethod: "newtonian",
  nx: 18,
  ny: 14,
  mach: 8,
  ...patch,
});

for (const family of ["capsule", "boostglide", "missile", "multistage"] as WaveriderFamily[]) {
  const built = buildVehicle(coarse(family, family === "missile" ? { span: 0.5, finHeight: 0.4, fins: 4 } : {}));
  const q = analyzeMesh(built.mesh);
  assert.equal(q.watertight, true, `${family} watertight, open ${q.openEdges}`);
  assert.equal(q.manifold, true, `${family} manifold, nonman ${q.nonManifoldEdges}`);
  assert.ok(q.volume > 1e-6, `${family} volume ${q.volume}`);
  assert.ok(built.aero.planformArea > 1e-4, `${family} sref`);
  const ev = evaluateDesign(built.params);
  assert.ok(finite(ev.metrics.ld) && finite(ev.metrics.cd) && finite(ev.metrics.qStag), `${family} eval`);
  console.log(`${family} tris=${q.triangles} V=${q.volume.toFixed(4)} S=${built.aero.planformArea.toFixed(3)} L/D=${ev.metrics.ld.toFixed(2)}`);
}

{
  const breaks = axisBreaks(2, 8, 2, [5]);
  assert.deepEqual(breaks, [2, 4, 5, 6, 8]);
  const built = buildVehicle(coarse("caret", { length: 3, span: 1.6, height: 0.4, shockDeg: 14 }));
  const db = await generateAeroDatabase(built, {
    machMin: 6,
    machMax: 8,
    machStep: 2,
    alphaMin: 0,
    alphaMax: 4,
    alphaStep: 4,
    betaMin: 0,
    betaMax: 0,
    betaStep: 4,
    deltaMin: 0,
    deltaMax: 0,
    deltaStep: 5,
    method: "newtonian",
    eulerSymmetry: false,
    dynamicDerivatives: true,
    altKm: 30,
  });
  assert.ok(db.points.length >= 4, `points ${db.points.length}`);
  assert.ok(db.points.some((p) => p.onDesign), "design Mach is in the table");
  assert.ok(db.points.some((p) => !p.onDesign), "off-design Mach is in the same table");
  assert.ok(db.dynamics.length >= 1 && finite(db.dynamics[0].cmq), "dynamic derivatives");
  const csv = aeroDbToCsv(db);
  assert.ok(csv.includes("sref_m2") && csv.startsWith("# cuspis"), "csv metadata");
  assert.ok(csv.includes("CL"));
  const m = aeroDbToSimulink(db);
  assert.ok(m.includes("aero.Sref") && m.includes("reshape"), "simulink lookup");
  const interior = db.points.find((p) => p.alpha_deg !== db.axes.alpha_deg[0] && p.alpha_deg !== db.axes.alpha_deg[db.axes.alpha_deg.length - 1]);
  if (interior && db.axes.alpha_deg.length >= 3) assert.ok(finite(interior.cla), "static CLα");
  console.log(`aerodb points=${db.points.length} maxL/D=${db.summary.maxLd.toFixed(2)} Cmq=${db.dynamics[0].cmq.toFixed(3)}`);
}

{
  const built = buildVehicle(coarse("caret", { nx: 14, ny: 10, length: 3, span: 1.4, height: 0.35 }));
  const fly = await flyClosedLoop(built, {
    mode: "boost-glide",
    law: "pid",
    durationS: 8,
    dt: 0.1,
    tauAct: 0.08,
    deltaMaxDeg: 8,
    rateMaxDeg: 40,
    sensorSigmaDeg: 0,
    seed: 2,
    qTargetKpa: 30,
    targetXKm: 50,
    kp: 1.2,
    ki: 0.05,
    kd: 0.05,
  });
  assert.ok(fly.samples.length > 4, "trajectory samples");
  assert.ok(fly.samples.every((s) => finite(s.hKm) && finite(s.alphaDeg) && finite(s.deltaDeg)), "finite history");
  console.log(`fly n=${fly.samples.length} range=${fly.rangeKm.toFixed(2)} km h=${fly.finalH.toFixed(2)}`);
}

assert.equal(actuatorStep(0, 10, 0.1, 0.01, 20, 30), 2);
assert.equal(dominatesPoint([1, 1], 0, [2, 2], 0), true, "smaller in both objectives dominates");
assert.equal(dominatesPoint([1, 2], 0, [0, 0], 0.2), true, "feasible dominates infeasible");

{
  const bad = constraintViolation(
    { ld: 1, rangeKm: 10, qStag: 80, mass: 100, staticMargin: -0.05, volume: 1, cd: 0.2, cm: 0, cl: 0.2, method: "newtonian" },
    [{ metric: "staticMargin", op: ">=", value: 0 }],
  );
  assert.ok(bad > 0, "static-margin constraint bites");
}

{
  const checks = runValidation();
  const mine = checks.filter((c) => c.id.startsWith("aerodb") || c.id.startsWith("tps") || c.id === "actuator-rate");
  assert.equal(mine.length, 6);
  assert.deepEqual(mine.filter((c) => !c.pass).map((c) => c.name), []);
  console.log(`mission checks ${mine.length} pass`);
}

console.log("mission.test ok");
