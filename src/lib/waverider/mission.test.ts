/**
 * Mission-lab checks: new vehicle classes, the aero database, TPS, the
 * optimizer's dominance rule, and a short closed-loop flight.
 * Run: node --import ./scripts/ts-register.mjs src/lib/waverider/mission.test.ts
 */
import assert from "node:assert/strict";
import { axisBreaks, aeroDbToCsv, aeroDbToSimulink, generateAeroDatabase } from "./aerodb";
import { atmosphere } from "./atmosphere";
import { actuatorStep, flyClosedLoop } from "./guidance";
import { DEG, newtonianCpMax } from "./math";
import { constraintViolation, dominatesPoint, evaluateDesign } from "./mdo";
import { buildVehicle } from "./generate";
import { analyzeMesh } from "./mesh";
import { topoMetrics } from "./geom-quality";
import { panelAero } from "./panel";
import { massBudget } from "./tps";
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
  const aLo = db.points.find((p) => p.alpha_deg === db.axes.alpha_deg[0] && p.beta_deg === 0 && p.delta_deg === 0 && p.mach === db.axes.mach[0]);
  const aHi = db.points.find((p) => p.alpha_deg === db.axes.alpha_deg[db.axes.alpha_deg.length - 1] && p.beta_deg === 0 && p.delta_deg === 0 && p.mach === db.axes.mach[0]);
  assert.ok(aLo && aHi);
  const dA = (aHi.alpha_deg - aLo.alpha_deg) * DEG;
  const secant = (aHi.cl - aLo.cl) / dA;
  assert.ok(Math.abs(secant) > 1e-3, `caret CLα secant ${secant}`);
  assert.ok(Math.abs(aLo.cla - secant) < 1e-9 && Math.abs(aHi.cla - secant) < 1e-9, "endpoint CLα is the one-sided secant, not 0");
  console.log(`aerodb points=${db.points.length} maxL/D=${db.summary.maxLd.toFixed(2)} Cmq=${db.dynamics[0].cmq.toFixed(3)} CLα=${secant.toFixed(3)}`);
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
  let kin = 0;
  for (const s of fly.samples) kin = Math.max(kin, Math.abs(s.alphaDeg - (s.thetaDeg - s.gammaDeg)));
  assert.ok(kin < 0.05, `α drifted from θ−γ by ${kin.toFixed(3)}°`);
  console.log(`fly n=${fly.samples.length} range=${fly.rangeKm.toFixed(2)} km h=${fly.finalH.toFixed(2)} |α−(θ−γ)|=${kin.toExponential(2)}°`);
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
  assert.equal(mine.length, 7);
  assert.deepEqual(mine.filter((c) => !c.pass).map((c) => c.name), []);
  console.log(`mission checks ${mine.length} pass`);
}

{
  const theta = 20 * DEG;
  const Rn = 0.25;
  const Rb = 0.55;
  const xt = Rn * (1 - Math.sin(theta));
  const rt = Rn * Math.cos(theta);
  const L = xt + (Rb - rt) / Math.tan(theta);
  const s = Math.sin(theta);
  const c = Math.cos(theta);
  const cpMax = newtonianCpMax(10, 1.4);
  const caExact = cpMax * ((Rn / Rb) ** 2 * (1 - s ** 4) / 2 + s * s * (1 - (Rn / Rb) ** 2 * c * c));
  const built = buildVehicle(
    coarse("capsule", {
      length: L,
      span: 2 * Rb,
      leRadius: Rn,
      coneDeg: 20,
      nx: 64,
      ny: 40,
      altKm: 0,
      mach: 10,
      lockFlight: true,
      alphaDeg: 0,
      gamma: 1.4,
    }),
  );
  const atm = atmosphere(0, 10, 1.4);
  const aero = panelAero(built.mesh, built.params, atm, 0, 0, built.aero.planformArea, L, undefined, true);
  const rel = Math.abs(aero.cdWave - caExact) / caExact;
  assert.ok(rel < 0.03, `sphere-cone CD ${aero.cdWave} vs Lees ${caExact} (${(100 * rel).toFixed(2)}%)`);
  const slope = (Rn - xt) / (Rn * Math.cos(theta));
  assert.ok(Math.abs(slope - Math.tan(theta)) < 1e-12, "sphere-cone tangency");
  console.log(`sphere-cone CDwave=${aero.cdWave.toFixed(4)} Lees=${caExact.toFixed(4)} err=${(100 * rel).toFixed(2)}%`);
}

{
  const Rb = 0.25;
  const L = 4;
  const Ln = 0.28 * L;
  const rho = (Rb * Rb + Ln * Ln) / (2 * Rb);
  const rAt = (x: number) => Math.sqrt(Math.max(0, rho * rho - (Ln - x) ** 2)) - (rho - Rb);
  assert.ok(Math.abs(rAt(0)) < 1e-12, "tangent ogive starts at a point");
  assert.ok(Math.abs(rAt(Ln) - Rb) < 1e-12, "ogive meets the cylinder");
  const slope = (rAt(Ln) - rAt(Ln - 1e-4)) / 1e-4;
  assert.ok(Math.abs(slope) < 1e-3, `ogive slope at the shoulder ${slope}`);
  const built = buildVehicle(coarse("missile", { length: L, span: 2 * Rb, wedgeFrac: 0.28, finHeight: 0, nx: 48, ny: 24 }));
  const pos = built.mesh.positions;
  let rMesh = 0;
  let n = 0;
  for (let i = 0; i < pos.length; i += 3) {
    if (Math.abs(pos[i] - Ln) < L / 47) {
      rMesh = Math.max(rMesh, Math.hypot(pos[i + 1], pos[i + 2]));
      n++;
    }
  }
  assert.ok(n > 0 && Math.abs(rMesh - Rb) / Rb < 0.04, `mesh radius at ogive end ${rMesh}`);
  console.log(`ogive r(0)=${rAt(0).toExponential(1)} r(Ln)=${rAt(Ln).toFixed(4)} mesh=${rMesh.toFixed(4)}`);
}

{
  const built = buildVehicle(coarse("boostglide", { planform: "delta", leBlunt: true, leRadius: 0.02, length: 4, span: 2.2, nx: 28, ny: 12 }));
  const pos = built.mesh.positions;
  const bins = new Map<number, number>();
  for (let i = 0; i < pos.length; i += 3) {
    const key = Math.round(pos[i] * 1e4) / 1e4;
    bins.set(key, Math.max(bins.get(key) ?? 0, Math.abs(pos[i + 1])));
  }
  const xs = [...bins.keys()].sort((a, b) => a - b);
  for (let i = 1; i < xs.length; i++) {
    assert.ok((bins.get(xs[i]) ?? 0) + 1e-6 >= (bins.get(xs[i - 1]) ?? 0), `planform narrowed ${xs[i - 1]}→${xs[i]}`);
  }
  const tm = topoMetrics(built.mesh);
  assert.equal(tm.openEdges, 0, "boost-glide open edges");
  assert.equal(tm.nonManifoldEdges, 0, "boost-glide non-manifold");
  assert.ok(tm.minAngleDeg > 0.5, `boost-glide nose slivers, min angle ${tm.minAngleDeg.toFixed(3)}°`);
  console.log(`boost-glide planform monotone, tip half-span ${(bins.get(xs[0]) ?? 0).toFixed(4)} min∠ ${tm.minAngleDeg.toFixed(2)}°`);
}

{
  const budget = massBudget({
    params: DEFAULT_PARAMS,
    volume: 0.4,
    length: 4,
    height: 0.5,
    structMass: 200,
    structCgX: 2.2,
    ixx: 10,
    iyy: 80,
    izz: 90,
    tpsMass: 40,
    payloadKg: 0,
    fuelFrac: 0,
    fuelRho: 800,
  });
  const dz = budget.cgZ;
  assert.ok(Math.abs(dz) > 1e-4, "TPS moves the CG off the axis");
  const scale = budget.total / 200;
  assert.ok(budget.ixx > 10 * scale + 1e-6, "roll inertia grew with the vertical CG shift");
  console.log(`mass Ixx ${budget.ixx.toFixed(3)} (scaled ${(10 * scale).toFixed(3)}) cgZ=${dz.toFixed(4)}`);
}

console.log("mission.test ok");
