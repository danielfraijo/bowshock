import assert from "node:assert/strict";
import { expansionAnchor, flatWedgeAnchor } from "./anchors";
import { eulerWedge } from "./euler2d";
import { buildVehicle } from "./generate";
import { betaFromThetaM, fastBeta } from "./math";
import { analyzeMesh } from "./mesh";
import { studyVehicle } from "./study";
import { DEFAULT_PARAMS, type DesignParams, type TriMesh, type WaveriderFamily } from "./types";
import { runValidation, validationSummary } from "./validate";

function maxAspect(mesh: TriMesh): number {
  const p = mesh.positions;
  const idx = mesh.indices;
  let worst = 1;
  const nt = idx.length / 3;
  for (let t = 0; t < nt; t++) {
    const ia = idx[t * 3] * 3;
    const ib = idx[t * 3 + 1] * 3;
    const ic = idx[t * 3 + 2] * 3;
    const ab = Math.hypot(p[ib] - p[ia], p[ib + 1] - p[ia + 1], p[ib + 2] - p[ia + 2]);
    const bc = Math.hypot(p[ic] - p[ib], p[ic + 1] - p[ib + 1], p[ic + 2] - p[ib + 2]);
    const ca = Math.hypot(p[ia] - p[ic], p[ia + 1] - p[ic + 1], p[ia + 2] - p[ic + 2]);
    const longest = Math.max(ab, bc, ca);
    const shortest = Math.min(ab, bc, ca);
    const aspect = shortest > 1e-12 ? longest / shortest : 1e9;
    if (aspect > worst) worst = aspect;
  }
  return worst;
}

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
  const aspect = maxAspect(built.mesh);
  const openOk = params.flowThrough ? q.openEdges > 0 : q.openEdges === 0;
  console.log(
    `${family.padEnd(12)} watertight=${q.watertight} open=${q.openEdges} nonMan=${q.nonManifoldEdges} tris=${q.triangles} aspect=${aspect.toFixed(1)} ${openOk ? "OK" : "BAD"}`,
  );
  if (!params.flowThrough) assert.ok(aspect < 80, `${family} aspect ${aspect.toFixed(1)}`);
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

const caret = buildVehicle(DEFAULT_PARAMS);
const caretAspect = maxAspect(caret.mesh);
const lead = caret.grids.some((g) => g.name === "leading");
console.log(
  `caret default aspect=${caretAspect.toFixed(1)} tris=${caret.quality.triangles} open=${caret.quality.openEdges} leading=${lead}`,
);
assert.equal(caret.quality.openEdges, 0, "default caret open edges");
assert.equal(caret.quality.nonManifoldEdges, 0, "default caret non-manifold");
assert.equal(lead, false, "default caret fillet must not tessellate a sub-span arc");
assert.ok(caretAspect < 80, `default caret aspect ${caretAspect.toFixed(1)}`);

let betaErr = 0;
for (let i = 1; i <= 24; i++) {
  const th = (i / 24) * 28 * (Math.PI / 180);
  const exact = betaFromThetaM(8, th, 1.4);
  const fast = fastBeta(8, th, 1.4);
  if (Number.isFinite(exact) && Number.isFinite(fast)) betaErr = Math.max(betaErr, Math.abs(exact - fast));
}
console.log(`fastBeta max |Δβ|=${((betaErr * 180) / Math.PI).toFixed(4)} deg`);
assert.ok(betaErr < (0.05 * Math.PI) / 180, `fastBeta err ${betaErr}`);

const tStudy = performance.now();
const quick = studyVehicle(caret, { quick: true });
const full = studyVehicle(caret, { quick: false });
console.log(
  `caret study quick=${(performance.now() - tStudy).toFixed(0)} ms incl full ${(full.elapsedMs).toFixed(0)} ms L/D=${full.aero.ld.toFixed(3)} quickL/D=${quick.aero.ld.toFixed(3)}`,
);

console.log("fidelity assertions passed");
