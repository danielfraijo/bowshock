import assert from "node:assert/strict";
import { expansionAnchor, flatWedgeAnchor } from "./anchors";
import { eulerWedge } from "./euler2d";
import { buildVehicle } from "./generate";
import { analyzeMesh } from "./mesh";
import { DEFAULT_PARAMS, type DesignParams, type WaveriderFamily } from "./types";
import { runValidation, validationSummary } from "./validate";

const families: WaveriderFamily[] = [
  "caret",
  "cone",
  "osculating",
  "viscopt",
  "elliptic",
  "wedgecone",
  "star",
  "liftbody",
  "ramjet",
  "scramjet",
  "busemann",
  "inward",
  "integrated",
];

const tMesh = performance.now();
for (const family of families) {
  const params: DesignParams = {
    ...DEFAULT_PARAMS,
    family,
    flowThrough: family === "ramjet" || family === "scramjet",
    nx: 28,
    ny: 18,
  };
  const built = buildVehicle(params);
  const q = analyzeMesh(built.mesh);
  const openOk = params.flowThrough ? q.openEdges > 0 : q.openEdges === 0;
  console.log(
    `${family.padEnd(12)} watertight=${q.watertight} open=${q.openEdges} nonMan=${q.nonManifoldEdges} tris=${q.triangles} ${openOk ? "OK" : "BAD"}`,
  );
  if (!params.flowThrough) assert.equal(q.openEdges, 0, `${family} has open edges`);
  assert.equal(q.nonManifoldEdges, 0, `${family} has non-manifold edges`);
  assert.ok(q.triangles > 50, `${family} lost its surface`);
}
console.log(`mesh families ${(performance.now() - tMesh).toFixed(0)} ms`);

const tW = performance.now();
const wedge = flatWedgeAnchor();
console.log(
  `wedge strip  L/D_wave=${wedge.ldWave.toFixed(3)} cot=${wedge.cot.toFixed(3)} ratio=${(wedge.ldWave / wedge.cot).toFixed(3)} Cp=${wedge.cp.toFixed(4)} exactCp=${wedge.exactCp.toFixed(4)} in ${(performance.now() - tW).toFixed(1)} ms`,
);
const ramp = expansionAnchor();
const q8 = 0.5 * 1.4 * 64;
const pSe = 1 + ramp.aftCp * q8;
const pTw = 1 + ramp.tangentAft * q8;
console.log(
  `ramp 30°→0  nose Cp=${ramp.noseCp.toFixed(4)} exact=${ramp.exactNose.toFixed(4)}  aft SE Cp=${ramp.aftCp.toFixed(4)} exact=${ramp.exactAft.toFixed(4)} p/p∞=${pSe.toFixed(3)}  tangent-wedge Cp=${ramp.tangentAft.toFixed(4)} p/p∞=${pTw.toFixed(3)}`,
);

const tE = performance.now();
const euler = eulerWedge(2, 10, 1.4);
console.log(
  `euler M=2 θ=10  p2/p1=${euler.p2p1.toFixed(4)} exact=${euler.exactP.toFixed(4)} err=${(euler.relErr * 100).toFixed(2)}% residual=${euler.residual.toExponential(2)} in ${(performance.now() - tE).toFixed(0)} ms`,
);

const checks = runValidation();
const summary = validationSummary(checks);
console.log(`validation ${summary.pass}/${summary.n}`);

assert.ok(wedge.ldWave / wedge.cot > 0.92 && wedge.ldWave / wedge.cot < 1.08, `wave L/D ${wedge.ldWave} vs cot ${wedge.cot}`);
assert.ok(Math.abs(wedge.cp - wedge.exactCp) / wedge.exactCp < 0.08, `wedge Cp ${wedge.cp} vs ${wedge.exactCp}`);
assert.ok(Math.abs(ramp.noseCp - ramp.exactNose) / ramp.exactNose < 0.08, `ramp nose ${ramp.noseCp} vs ${ramp.exactNose}`);
assert.ok(Math.abs(ramp.aftCp - ramp.exactAft) < 0.012, `ramp aft SE ${ramp.aftCp} vs ${ramp.exactAft}`);
assert.ok(
  Math.abs(ramp.tangentAft - ramp.exactAft) > Math.abs(ramp.aftCp - ramp.exactAft) + 0.008,
  `tangent-wedge aft ${ramp.tangentAft} should miss the analytic residual ${ramp.exactAft} by more than the strip`,
);
assert.ok(euler.relErr < 0.08, `euler error ${(euler.relErr * 100).toFixed(1)}%`);
assert.ok(summary.ok, `validation ${summary.pass}/${summary.n}`);
console.log("fidelity assertions passed");
