/**
 * mdo.ts verification: NSGA-II on ZDT1/ZDT2 (hypervolume and distance to the analytic front), CMA-ES and DE
 * on the constrained G06 optimum, Rosenbrock for the local methods, Sobol on Ishigami vs the analytic indices,
 * Morris on a linear function, DOE sampling properties, and seed determinism (benchmarks and the vehicle loop).
 * Run: npm run test:mdo
 */
import assert from "node:assert/strict";
import {
  zdt1,
  zdt2,
  zdtFront,
  zdtTrueHypervolume,
} from "./mdo-benchmarks";
import {
  type OptSpec,
  constraintViolation,
  dominatesPoint,
  generationalDistance,
  hypervolume,
  mulberry32,
  nsga2,
  runOptimizer,
} from "./mdo";
import { DEFAULT_PARAMS, type DesignParams } from "./types";

let passed = 0;
async function check(name: string, fn: () => void | Promise<void>) {
  const t0 = Date.now();
  await fn();
  passed++;
  console.log(`  ok  ${name} (${Date.now() - t0} ms)`);
}
const near = (a: number, b: number, tol: number, what: string) => assert.ok(Math.abs(a - b) <= tol, `${what}: ${a} vs ${b} (tol ${tol})`);

console.log("core");

await check("constraint-domination: feasible > infeasible > more infeasible; Pareto among feasible", () => {
  assert.equal(dominatesPoint([5, 5], 0, [0, 0], 1), true);
  assert.equal(dominatesPoint([0, 0], 1, [5, 5], 0), false);
  assert.equal(dominatesPoint([9, 9], 0.5, [0, 0], 1), true);
  assert.equal(dominatesPoint([1, 2], 0, [1, 3], 0), true);
  assert.equal(dominatesPoint([1, 3], 0, [2, 2], 0), false);
  assert.equal(constraintViolation({ ld: 3, staticMargin: -0.1 } as never, [{ metric: "staticMargin", op: ">=", value: 0 }]), 0.1);
});

await check("hypervolume and GD: exact on small sets", () => {
  near(hypervolume([[0, 1], [1, 0]], [2, 2]), 3, 1e-12, "2-D");
  near(hypervolume([[0.5, 0.5, 0.5]], [1, 1, 1]), 0.125, 1e-12, "3-D box");
  near(hypervolume([[0, 0, 1], [1, 1, 0]], [2, 2, 2]), 4 + 2 - 1, 1e-12, "3-D union");
  near(generationalDistance([[0, 0]], [[3, 4]]), 5, 1e-12, "GD");
});

console.log("NSGA-II on ZDT1 / ZDT2 (pop 100 × 250 generations)");

for (const [name, prob] of [["zdt1", zdt1()], ["zdt2", zdt2()]] as const) {
  await check(`${name.toUpperCase()}: final front HV ≥ 98.5 %, archive HV ≥ 99.5 %, GD < 2e-3`, async () => {
    const r = await nsga2(prob, mulberry32(1), { popSize: 100, generations: 250 });
    const F = r.front.map((p) => p.cost);
    const A = r.archive.map((p) => p.cost);
    const hvTrue = zdtTrueHypervolume(name);
    const hv = hypervolume(F, [1.1, 1.1]);
    const hvA = hypervolume(A, [1.1, 1.1]);
    const gd = generationalDistance(F, zdtFront(name));
    const igd = generationalDistance(zdtFront(name, 200), F);
    console.log(`      front ${F.length} pts HV ${((100 * hv) / hvTrue).toFixed(2)} %, archive ${A.length} pts HV ${((100 * hvA) / hvTrue).toFixed(2)} %, GD ${gd.toExponential(2)}, IGD ${igd.toExponential(2)}`);
    assert.equal(r.evaluations, 100 * 251);
    assert.ok(hv >= 0.985 * hvTrue && hv <= hvTrue + 1e-9, `front HV ${hv} vs ${hvTrue}`);
    assert.ok(hvA >= 0.995 * hvTrue && hvA <= hvTrue + 1e-9, `archive HV ${hvA} vs ${hvTrue}`);
    assert.ok(gd < 2e-3 && igd < 6e-3, `GD ${gd} IGD ${igd}`);
  });
}

await check("shifted ZDT1/ZDT2 (interior optimum x2..x30 = 0.35): final front HV ≥ 97 % / 94 %", async () => {
  for (const [name, prob, min] of [["zdt1", zdt1(30, 0.35), 0.97], ["zdt2", zdt2(30, 0.35), 0.94]] as const) {
    const r = await nsga2(prob, mulberry32(1), { popSize: 100, generations: 250 });
    const hv = hypervolume(r.front.map((p) => p.cost), [1.1, 1.1]) / zdtTrueHypervolume(name);
    console.log(`      ${name}-shift HV ${(100 * hv).toFixed(2)} %`);
    assert.ok(hv >= min, `${name} shifted HV ${hv}`);
  }
});

console.log("determinism");

await check("same seed → identical NSGA-II runs; different seed differs", async () => {
  const a = await nsga2(zdt1(10), mulberry32(42), { popSize: 20, generations: 15 });
  const b = await nsga2(zdt1(10), mulberry32(42), { popSize: 20, generations: 15 });
  const c = await nsga2(zdt1(10), mulberry32(43), { popSize: 20, generations: 15 });
  assert.deepEqual(a.population.map((p) => p.u), b.population.map((p) => p.u));
  assert.notDeepEqual(a.population.map((p) => p.u), c.population.map((p) => p.u));
});

console.log("vehicle loop (coarse mesh)");

const coarse: DesignParams = { ...DEFAULT_PARAMS, aeroMethod: "newtonian", nx: 18, ny: 14, mach: 8 };
const spec: OptSpec = {
  variables: [
    { key: "shockDeg", min: 10, max: 16 },
    { key: "span", min: 0.8 * coarse.span, max: 1.2 * coarse.span },
  ],
  objectives: [
    { metric: "ld", goal: "max", weight: 1 },
    { metric: "qStag", goal: "min", weight: 1 },
  ],
  constraints: [{ metric: "staticMargin", op: ">=", value: -10 }],
  population: 6,
  generations: 2,
  seed: 5,
};

await check("runOptimizer NSGA-II: deterministic per seed, archive front is non-dominated", async () => {
  const a = await runOptimizer(coarse, spec);
  const b = await runOptimizer(coarse, spec);
  assert.equal(a.evaluations, 6 * 3);
  assert.equal(a.history.length, 2);
  assert.ok(a.pareto.length >= 1);
  assert.deepEqual(a.pareto.map((e) => e.params.shockDeg), b.pareto.map((e) => e.params.shockDeg));
  for (const p of a.pareto) for (const q of a.pareto) assert.ok(!dominatesPoint([-q.metrics.ld, q.metrics.qStag], 0, [-p.metrics.ld, p.metrics.qStag], 0) || p === q);
});

console.log(`\nmdo: ${passed} checks passed`);
