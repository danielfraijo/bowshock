# Optimizer, DOE and sensitivity (`src/lib/waverider/mdo.ts`)

`mdo.ts` is the single entry point. It imports four siblings:

| File | Contents |
|---|---|
| `mdo.ts` | Vehicle metrics (`evaluateDesign`), objectives/constraints, **the one NSGA-II** (`nsga2`, `runOptimizer`), `designProblem`, 1-D `parameterSweep`, central-difference `sensitivity`, and the new design-level `lhsSweep`, `factorialSweep`, `morrisSensitivity`, `sobolSensitivity`. Re-exports the siblings. |
| `mdo-core.ts` | Unit-cube problem types (`UnitProblem`, `UnitPoint`), `mulberry32` RNG, Deb constraint-domination, the budget/abort evaluator, hypervolume, GD/IGD. |
| `mdo-solvers.ts` | CMA-ES (IPOP), differential evolution, Nelder–Mead, pattern search. |
| `mdo-sampling.ts` | Full-factorial grid, Latin hypercube (and maximin LHS), Morris screening, Sobol/Saltelli indices. |
| `mdo-benchmarks.ts` | Analytic checks: sphere, Rosenbrock, G06, ZDT1/ZDT2 (plus shifted), Ishigami. |

Tests: `npm run test:mdo` (`src/lib/waverider/mdo.test.ts`).

## Conventions

- Every method works in the unit cube `u ∈ [0,1]^n`. `OptVariable {key, min, max}` maps `u` linearly onto
  `[min, max]`. Variables have to be numeric `DesignParams` keys (`numericDesignKeys()`), and `max > min`.
- Objectives are minimized. A `"max"` goal is negated.
- NSGA-II cost vector: `±metric / weight`. A positive scale changes neither Pareto dominance nor the
  normalized crowding distance, so **weights have no effect on NSGA-II**. They only matter to the
  single-objective methods.
- Single-objective methods minimize `Σ weight · (±metric)`.
- Constraints: `OptConstraint {metric, op: ">=" | "<=", value}`. Violation is `Σ max(0, shortfall)` in
  metric units, with no normalization. A design is feasible when the violation is ≤ 1e-8 and the build
  did not throw. A failed build has violation 1e6.
- Constraint handling is Deb's constraint-domination (feasible beats infeasible, infeasible points compare
  by violation, feasible points compare by Pareto dominance or the scalar). The single-objective methods
  accept `handling: "penalty"` (f + ρ·violation, ρ = 1e3) for unit-cube problems.
- Equality constraints are not in `OptConstraint`. Pose `h = c` as `h ≥ c − tol` together with `h ≤ c + tol`.
  On a `UnitProblem`, return `max(0, |h − c| − tol)` as the violation.
- Every stochastic method draws only from the `mulberry32(seed)` it is given, so a run is reproducible
  bit for bit for a given seed and Node/browser build.

## Methods

### NSGA-II (`nsga2`, `runOptimizer` default)
Deb et al. 2002 has fast non-dominated sorting with constraint-domination, crowding distance, a binary
crowded tournament and (μ+λ) elitist survival.
- Variation is bounded SBX (η_c = 20). It crosses each pair with probability 0.9 and each gene with
  probability ½, with the spread truncated to the box and the children swapped at random.
- Polynomial mutation (η_m = 18) acts on each gene with probability **1/n**.
- `runOptimizer` seeds the baseline design into the first population. It clamps the population to 4–24
  and the generations to 1–20 so the UI stays responsive.
- It returns the **archive**: the feasible non-dominated set over *every* evaluation, not only the last
  survivors. It falls back to the final rank-0 front when no design was feasible.

### CMA-ES (`cmaes`, `algorithm: "cmaes"`)
(μ/μ_w, λ)-CMA-ES following Hansen's 2016 tutorial, with optional IPOP restarts (λ doubles from a random
mean).
- The start is the baseline (or `x0`), with σ₀ = 0.3 in cube units.
- Samples are clipped to the box (projection repair), and ranking uses the constraint comparator. CMA-ES
  is rank-based, so only the ordering matters.
- It stops on tolX/tolFun, at `maxIter` or at `maxEvals`.

### Differential evolution (`differentialEvolution`, `algorithm: "de"`)
DE/rand/1/bin (or `best1bin`) after Storn & Price 1997.
- F is dithered uniformly in [F, 1] each generation (F = 0.5, CR = 0.9).
- An out-of-box mutant is repaired by a midpoint bounce-back toward the parent.
- Selection is greedy one-to-one under Deb's rules.

### Nelder–Mead (`nelderMead`, `algorithm: "nelder-mead"`)
Adaptive coefficients after Gao & Han 2012, with the simplex clipped to the box and optional restarts
from the best vertex. It is a **local** method.

### Pattern search (`patternSearch`, `algorithm: "pattern-search"`)
A compass poll (±step on each coordinate) with a pattern move on success, and the step halves when no
poll improves. It is **local**, and it suits smooth or box-bounded problems.

In `runOptimizer`, every single-objective method gets the same budget as NSGA-II
(`population · (generations + 1)` evaluations). It returns one design in `pareto`, and its history is
split into windows of `population` evaluations.

### DOE
- `factorialSweep(base, vars, levels)` builds a full-factorial grid with `levels` equally spaced values per
  variable. The first variable varies slowest, and the grid is capped at 1e6 points.
- `lhsSweep(base, vars, samples, {seed, candidates})` draws a Latin hypercube (one sample per stratum per
  variable). With `candidates > 1` it keeps the maximin-best of that many seeded designs.
- The existing one-variable `parameterSweep` is unchanged.

### Sensitivity
- The existing `sensitivity` is unchanged: local central differences (±5 %) and elasticities.
- `morrisSensitivity(base, vars, metric, {trajectories = 10, levels = 4})` computes Morris elementary
  effects: μ, μ* (Campolongo 2007) and σ, per full variable range. It costs r·(d+1) evaluations. μ* ranks
  importance, and a large σ/μ* means nonlinearity or interaction.
- `sobolSensitivity(base, vars, metric, {samples = 64, bootstrap = 100})` computes variance-based Sobol
  indices with Saltelli's A/B/AB_i scheme. First-order S1 uses Saltelli 2010, total ST uses Jansen 1999,
  and seeded bootstrap gives 95 % intervals. It costs N·(d+2) evaluations.

The generic versions (`morrisScreening`, `sobolIndices`, `lhsUnit`, `factorialUnit`) take any
`(u) => number` function.

## Verification (`npm run test:mdo`)

| Check | Result (seed 1 unless noted) | Gate |
|---|---|---|
| ZDT1, 30-D, NSGA-II 100 × 250 | final front HV 99.24 % of the exact value, archive 99.94 %, GD 8.1e-4 | ≥ 98.5 %, ≥ 99.5 %, GD < 2e-3 |
| ZDT2, 30-D, NSGA-II 100 × 250 | final 98.90 %, archive 99.89 %, GD 6.4e-4 | same |
| Shifted ZDT1/ZDT2 (interior optimum) | final HV ≥ 97 % / 94 % | guards against bound-piling operators |
| G06 (2 nonlinear constraints) | CMA-ES (6 IPOP restarts) and DE reach −6961.81 (f* = −6961.81388) | within 0.1 %, feasible |
| Rosenbrock | CMA-ES 5-D, NM 2-D and DE 2-D reach f < 1e-6, pattern search 2-D reaches f < 1e-3 | |
| Ishigami (a = 7, b = 0.1) | Sobol N = 8192: every S1 and ST within 0.02 of the analytic values | 0.03 |
| Morris on a linear function | μ* equals the coefficient magnitude exactly, σ = 0 | 1e-9 |
| Determinism | same seed gives an identical NSGA-II / CMA-ES / DE run and an identical `runOptimizer` front, and a different seed differs | exact |
| Vehicle loop | `runOptimizer` for all five algorithms, plus `lhsSweep` and `factorialSweep`, on a coarse Newtonian mesh | finite, feasible, within budget |

Hypervolume uses the reference point (1.1, 1.1). The exact value is `ref − ∫f₂ + (ref − 1)·ref`.

## Assumptions and limits

- **The UI budget is small.** At ≤ 24 × 20 = 504 evaluations, NSGA-II gives a rough front for 2–6
  variables. It does not converge a 30-variable problem (it reaches about 0–70 % HV on 30-D ZDT, depending
  on operators). Use more evaluations headless through `nsga2(designProblem(...))`.
- **Pattern search** fails on G06: it stops still infeasible (violation ≈ 0.87, f ≈ −7951). Its
  coordinate poll stalls where the feasible region is a thin sliver bounded by active nonlinear
  constraints, and it cannot track equality constraints. Nelder–Mead with 5 restarts did reach the G06
  optimum here, but it has no convergence guarantee under constraints. For constrained problems, use
  CMA-ES or DE.
- CMA-ES, Nelder–Mead and pattern search are clipped to the box, so an optimum on a bound is reached by
  projection. That works, but CMA-ES step-size adaptation is then only approximate.
- Constraint violations are summed in raw metric units. Mix constraints of very different magnitudes with
  care, or scale the limit.
- Morris and Sobol assume independent inputs, uniform over `[min, max]`. Failed builds are dropped from
  the Sobol estimate (`used < samples`), which biases the indices if failures cluster. At the default
  N = 64 on the vehicle, expect about ±0.1 on the indices, and read the bootstrap intervals.
- All evaluations are serial in one thread (one build plus one panel solve each). Euler is replaced by
  CBAERO inside the loop. Range is equilibrium glide, not closed-loop.
- There is no CLI here yet. `designProblem` together with any method is the hook a headless driver
  (`scripts/cuspis-eval.ts`) or OpenMDAO wrapper should call.
