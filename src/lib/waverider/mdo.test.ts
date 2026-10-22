/**
 * mdo.ts verification: NSGA-II on ZDT1/ZDT2 (hypervolume and distance to the analytic front), CMA-ES and DE
 * on the constrained G06 optimum, Rosenbrock for the local methods, Sobol on Ishigami vs the analytic indices,
 * Morris on a linear function, DOE sampling properties, and seed determinism (benchmarks and the vehicle loop).
 * Run: npm run test:mdo
 */
import assert from "node:assert/strict";
import {
  G06_OPT,
  g06,
  ishigami,
  ishigamiIndices,
  rosenbrock,
  sphere,
  zdt1,
  zdt2,
  zdtFront,
  zdtTrueHypervolume,
} from "./mdo-benchmarks";
import {
  type OptSpec,
  cmaes,
  constraintViolation,
  differentialEvolution,
  dominatesPoint,
  factorialSweep,
  factorialUnit,
  generationalDistance,
  hypervolume,
  lhsSweep,
  lhsUnit,
  morrisScreening,
  mulberry32,
  nelderMead,
  nsga2,
  patternSearch,
  runOptimizer,
  sobolIndices,
  symEig,
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

await check("symEig: reconstructs a symmetric matrix", () => {
  const A = [[4, 1, 0.5], [1, 3, 0.2], [0.5, 0.2, 2]];
  const { values, vectors } = symEig(A);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) {
      let s = 0;
      for (let k = 0; k < 3; k++) s += vectors[i][k] * values[k] * vectors[j][k];
      near(s, A[i][j], 1e-10, `A[${i}][${j}]`);
    }
});

await check("DOE: LHS is one-per-stratum; factorial order and count", () => {
  const pts = lhsUnit(mulberry32(7), 50, 4);
  for (let j = 0; j < 4; j++) {
    const bins = pts.map((p) => Math.floor(p[j] * 50)).sort((a, b) => a - b);
    assert.deepEqual(bins, Array.from({ length: 50 }, (_, i) => i), `dim ${j} stratified`);
  }
  const grid = factorialUnit([3, 2]);
  assert.deepEqual(grid, [[0, 0], [0, 1], [0.5, 0], [0.5, 1], [1, 0], [1, 1]]);
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

console.log("single-objective methods");

await check("G06 (constrained): CMA-ES with IPOP restarts reaches f* = −6961.81 feasibly", async () => {
  const r = await cmaes(g06(), mulberry32(3), { restarts: 6, maxEvals: 20000 });
  console.log(`      f ${r.best.cost[0].toFixed(3)} viol ${r.best.violation.toExponential(1)} evals ${r.evaluations}`);
  assert.ok(r.best.violation <= 1e-8, "feasible");
  near(r.best.cost[0], G06_OPT, 0.01 * Math.abs(G06_OPT) * 0.1, "f");
});

await check("G06 (constrained): DE/rand/1/bin reaches f* = −6961.81 feasibly", async () => {
  const r = await differentialEvolution(g06(), mulberry32(3), { popSize: 30, generations: 400 });
  console.log(`      f ${r.best.cost[0].toFixed(3)} viol ${r.best.violation.toExponential(1)} evals ${r.evaluations}`);
  assert.ok(r.best.violation <= 1e-8, "feasible");
  near(r.best.cost[0], G06_OPT, 0.01 * Math.abs(G06_OPT) * 0.1, "f");
});

await check("Rosenbrock: CMA-ES 5-D, Nelder–Mead 2-D, DE 2-D reach f < 1e-6; pattern search 2-D f < 1e-3", async () => {
  const c = await cmaes(rosenbrock(5), mulberry32(1), { maxEvals: 30000 });
  const nm = await nelderMead(rosenbrock(2), { restarts: 3, maxEvals: 5000 });
  const de = await differentialEvolution(rosenbrock(2), mulberry32(1), { popSize: 20, generations: 300 });
  const ps = await patternSearch(rosenbrock(2), { maxEvals: 20000 });
  console.log(`      cma ${c.best.cost[0].toExponential(1)} nm ${nm.best.cost[0].toExponential(1)} de ${de.best.cost[0].toExponential(1)} ps ${ps.best.cost[0].toExponential(1)}`);
  assert.ok(c.best.cost[0] < 1e-6, "cma");
  assert.ok(nm.best.cost[0] < 1e-6, "nm");
  assert.ok(de.best.cost[0] < 1e-6, "de");
  assert.ok(ps.best.cost[0] < 1e-3, "ps");
  const x = rosenbrock(5).toX(c.best.u);
  for (const xi of x) near(xi, 1, 1e-3, "x*");
});

await check("pattern search and Nelder–Mead: sphere optimum; budget is a hard cap", async () => {
  const ps = await patternSearch(sphere(4), {});
  const nm = await nelderMead(sphere(4), {});
  assert.ok(ps.best.cost[0] < 1e-8 && nm.best.cost[0] < 1e-8, `${ps.best.cost[0]} ${nm.best.cost[0]}`);
  const capped = await cmaes(sphere(4), mulberry32(1), { maxEvals: 37 });
  assert.equal(capped.evaluations, 37);
  assert.match(capped.message, /budget/);
});

console.log("sensitivity");

await check("Sobol (Saltelli/Jansen) on Ishigami: S1, ST within 0.03 of the analytic indices", async () => {
  const f = ishigami();
  const r = await sobolIndices(f, 3, mulberry32(11), { samples: 8192, bootstrap: 0 });
  const ex = ishigamiIndices();
  r.indices.forEach((ix, j) => {
    console.log(`      x${j + 1}: S1 ${ix.S1.toFixed(3)} (exact ${ex.S1[j].toFixed(3)})  ST ${ix.ST.toFixed(3)} (exact ${ex.ST[j].toFixed(3)})`);
    near(ix.S1, ex.S1[j], 0.03, `S1 x${j + 1}`);
    near(ix.ST, ex.ST[j], 0.03, `ST x${j + 1}`);
  });
  assert.equal(r.evaluations, 8192 * 5);
  near(r.variance, ex.V, 0.05 * ex.V, "variance");
});

await check("Morris on a linear function: μ* = |coefficient|, σ = 0", async () => {
  const r = await morrisScreening((u) => 2 * u[0] - 0.5 * u[1], 3, mulberry32(5), { trajectories: 8 });
  near(r.indices[0].muStar, 2, 1e-9, "μ*0");
  near(r.indices[1].muStar, 0.5, 1e-9, "μ*1");
  near(r.indices[1].mu, -0.5, 1e-9, "μ1");
  near(r.indices[2].muStar, 0, 1e-12, "μ*2");
  assert.ok(r.indices.every((ix) => ix.sigma < 1e-9));
  assert.equal(r.evaluations, 8 * 4);
});

console.log("determinism");

await check("same seed → identical NSGA-II, CMA-ES, DE runs; different seed differs", async () => {
  const a = await nsga2(zdt1(10), mulberry32(42), { popSize: 20, generations: 15 });
  const b = await nsga2(zdt1(10), mulberry32(42), { popSize: 20, generations: 15 });
  const c = await nsga2(zdt1(10), mulberry32(43), { popSize: 20, generations: 15 });
  assert.deepEqual(a.population.map((p) => p.u), b.population.map((p) => p.u));
  assert.notDeepEqual(a.population.map((p) => p.u), c.population.map((p) => p.u));
  const c1 = await cmaes(g06(), mulberry32(9), { maxEvals: 500 });
  const c2 = await cmaes(g06(), mulberry32(9), { maxEvals: 500 });
  assert.deepEqual(c1.best.u, c2.best.u);
  const d1 = await differentialEvolution(g06(), mulberry32(9), { generations: 20 });
  const d2 = await differentialEvolution(g06(), mulberry32(9), { generations: 20 });
  assert.deepEqual(d1.best.u, d2.best.u);
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

await check("runOptimizer CMA-ES / DE / NM / pattern: one feasible design within budget", async () => {
  for (const algorithm of ["cmaes", "de", "nelder-mead", "pattern-search"] as const) {
    const r = await runOptimizer(coarse, { ...spec, objectives: [spec.objectives[0]], algorithm, generations: 1, population: 5 });
    assert.ok(r.evaluations <= 10, `${algorithm} budget ${r.evaluations}`);
    assert.equal(r.pareto.length, 1, algorithm);
    assert.ok(r.pareto[0].feasible && Number.isFinite(r.pareto[0].metrics.ld), algorithm);
  }
});

await check("lhsSweep / factorialSweep over design variables", async () => {
  const lhs = await lhsSweep(coarse, spec.variables, 4, { seed: 2 });
  assert.equal(lhs.length, 4);
  const bins = lhs.map((r) => Math.floor(((r.values.shockDeg! - 10) / 6) * 4)).sort();
  assert.deepEqual(bins, [0, 1, 2, 3]);
  const fac = await factorialSweep(coarse, spec.variables, [2, 2]);
  assert.deepEqual(fac.map((r) => r.values.shockDeg), [10, 10, 16, 16]);
  assert.ok(fac.every((r) => Number.isFinite(r.metrics.ld)));
});

console.log(`\nmdo: ${passed} checks passed`);
