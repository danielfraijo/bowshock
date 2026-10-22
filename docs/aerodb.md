# Aero database and headless runs

This page covers `npm run aerodb`, the headless/batch CLI. It runs main's `generateAeroDatabase` (`src/lib/waverider/aerodb.ts`), the MDO tools (`mdo.ts`) and closed-loop flight (`guidance.ts`) without the browser. It also documents the output formats and conventions as the code defines them, the Python package in `python/`, and the literature validation tests.

Everything here was checked against the code at base `55d643f`, origin/main as of Oct 22, 4:03 PM MST. Line references are to that commit.

## Quick start

```bash
npm ci
npm run aerodb -- --config examples/aerodb-caret.json --out aerodb-out          # aero table
npm run aerodb -- --config examples/mdo-caret.json --out mdo-out                # NSGA-II Pareto front
npm run aerodb -- --config examples/fly-caret.json --out fly-out                # closed-loop trajectory
npm run aerodb -- --config examples/fly-caret.json --mode sweep --out sweep-out # same file, other mode
npm run aerodb -- --config cfg.json --validate-only                             # check only, write nothing
node --import ./scripts/ts-register.mjs scripts/cuspis-eval.ts design.json      # legacy: metrics JSON on stdout
```

The entry point is still `scripts/cuspis-eval.ts`. The legacy form (exactly one positional `design.json`, either `DesignParams` or `{ "params": … }`) behaves as before. All other logic is in `src/lib/waverider/headless-cli.ts` and `headless-config.ts`, so it is typechecked: `tsconfig` covers only `src` and `server`.

| Option | Meaning |
| --- | --- |
| `--config <file>` | JSON config (below). Relative paths resolve against the directory you ran `npm` from (`INIT_CWD`). |
| `--out <dir>` | Output directory. It is created if needed. Each file is written to `name.tmp-<pid>` and then renamed. |
| `--mode <m>` | Overrides the config's `mode`. |
| `--reproducible` | `createdISO` and `aerodb.json` `meta.generatedAt` come from `$SOURCE_DATE_EPOCH`, else 1970-01-01. Timings are written as 0 and runtime info is dropped. Two runs then produce byte-identical files. |
| `--quiet` | No progress on stderr. Warnings and errors are still printed. |
| `--validate-only` | Validate the config and print the point count and config hash. |

Progress goes to stderr as `[aerodb] done/total (pct%) label`, throttled to 2 Hz. Stdout stays clean.

**Ctrl-C / SIGTERM.** These abort the run through an `AbortSignal`. `aerodb.ts`, `mdo.ts` and `guidance.ts` all check the signal and throw an `AbortError`. Nothing is written and the exit code is 130. A second Ctrl-C forces the exit.

**Exit codes.** 0 ok; 1 run or write failure; 2 usage or config error; 130 interrupted.

## Config

```jsonc
{
  "mode": "aerodb",                        // aerodb | eval | mdo | sweep | sensitivity | fly (default aerodb)
  "name": "caret-m8-aerodb",               // optional; recorded in the manifest
  "geometry": { "preset": "caret-m8",      // any id from presets.ts; omitted = defaults
                "params": { "nx": 28 } },  // DesignParams overrides, run through sanitizeParams
  "aerodb": { ... },                       // Partial<AeroDbSpec>; missing keys come from defaultAeroDbSpec(params)
  "mdo": { "variables": [{ "key": "span", "min": 1.8, "max": 2.6 }],
           "objectives": [{ "metric": "ld", "goal": "max", "weight": 1 }],
           "constraints": [{ "metric": "staticMargin", "op": ">=", "value": 0 }],
           "population": 8, "generations": 5, "seed": 1 },
  "sweep": { "key": "alphaDeg", "values": [0, 2, 4] },
  "sensitivity": { "keys": ["span", "shockDeg"], "metric": "ld", "frac": 0.05 },
  "fly": { "mode": "boost-glide", "law": "pid", "durationS": 80, "massKg": 900 }   // Partial<FlySpec> + mass/inertia
}
```

**Validation.** The validator reports every problem with its path and exits 2. For example:

```
aerodb: invalid config:
  - bogus: unknown key
  - geometry.params.span: expected number, got "x"
  - aerodb.method: expected one of newtonian, tangent, mixed, cbaero, euler
  - aerodb.machMin: 0.5 < 1.05 (panel methods are supersonic)
```

It checks the following:

- Unknown keys at every level.
- Types against `defaultAeroDbSpec`, `defaultFlySpec` and `DesignParams`.
- Enums.
- `max ≥ min` and `step > 0`.
- `machMin ≥ 1.05` (`panel.ts` floors M at 1.05).
- `altKm` in the US76 range 0–84.9.
- The grid point count against `AERODB_POINT_CAP` (480), using main's `gridCount`.
- MDO keys and sweep keys against `numericDesignKeys()`.
- Metrics against `MDO_METRICS`.

The validator also warns about clamping that main applies silently:

- An axis with more than 21 breakpoints is resampled by `axisBreaks`.
- `sanitizeParams` clamps parameters such as `ny ≥ 10`.
- guidance clamps `fly.dt` to 0.01–0.2 s.

`max < min` is an error here, although `axisBreaks` would swap the bounds itself.

### `aerodb` keys (AeroDbSpec, aerodb.ts:31)

| key | default (`defaultAeroDbSpec`) | notes |
| --- | --- | --- |
| `machMin`/`machMax`/`machStep` | M−2 … M+2, step 2. M is the flight Mach (design Mach when `lockFlight`), clamped to 1.5–24. | The design Mach is always inserted into the axis if it lies inside the range. |
| `alphaMin`/`alphaMax`/`alphaStep` | −4 … 12°, step 4 | The design α (`alphaDeg`) is inserted. |
| `betaMin`/`betaMax`/`betaStep` | 0 … 0, step 4 | 0 is inserted. |
| `deltaMin`/`deltaMax`/`deltaStep` | 0 … 0, step 5 | δ is the **absolute** `elevonDeg` (one symmetric elevon or fin-ridge shear). Each δ rebuilds the OML. 0 is inserted. |
| `method` | the design's `aeroMethod`, with `euler` replaced by `cbaero` | `newtonian`, `tangent`, `mixed` (shock-expansion), `cbaero`, `euler`. `euler` runs a symmetry-plane Euler solve at **every** point, which is slow. |
| `eulerSymmetry` | false | Adds `euler_CL/CD/Cm` at β nearest 0 and δ nearest 0. Otherwise those columns are empty or null. |
| `dynamicDerivatives` | true | See rate derivatives below. |
| `altKm` | the design's `altKm` | One US76 altitude for the whole table. q, V and Re follow from M at that altitude. |

Each axis is capped at 21 breakpoints, and the whole grid at 480 points (`AERODB_POINT_CAP`).

## Outputs per mode

Every run writes `config.resolved.json` (the fully defaulted config) and, last of all, `manifest.json`.

| mode | API (public only) | files |
| --- | --- | --- |
| `aerodb` | `buildVehicle` → `generateAeroDatabase` | `aerodb.csv` (`aeroDbToCsv`), `aerodb.json` (`aeroDbToJson`), `cuspis_aero_db.m` (`aeroDbToSimulink`) |
| `eval` | `evaluateDesign` | `metrics.json` |
| `mdo` | `runOptimizer` (NSGA-II) | `mdo.json` (the full `OptResult`), `pareto.csv`, `history.csv` |
| `sweep` | `parameterSweep` | `sweep.csv`, `sweep.json` |
| `sensitivity` | `sensitivity` | `sensitivity.csv`, `sensitivity.json` |
| `fly` | `buildVehicle` → `flyClosedLoop` | `trajectory.csv` (`samplesToCsv`), `fly.json` |

`mdo.ts` currently clamps `population` to 4–24 and `generations` to 1–20. The CLI passes your values through unchanged and leaves the clamping to `mdo.ts`. `mdo.ts` internals are not touched here. The metric columns in `pareto.csv` and `sweep.csv` are `ld, rangeKm, qStag, mass, staticMargin, volume, cd, cm, cl, method`.

**manifest.json** has these fields:

- `layoutVersion` (`cuspis-headless-1`), `mode`, `name`, `appVersion`.
- `gitSha` and `gitDirty`.
- `configHash` (cyrb53 over the sorted-key resolved config; it does not depend on `name`, `--reproducible` or key order) and `geometryHash`.
- `createdISO`, `counts`, `configWarnings`.
- Mode extras. For `aerodb` these are the axes, references, units, assumptions and summary.
- `runtime` and `timings_ms`.
- `files[]`, with `{name, bytes, hash}` for each file.

## aerodb formats (main's, unchanged)

**`aerodb.json`** is `JSON.stringify(db, null, 2)` of `AeroDatabase`: `{meta, axes, points[], dynamics[], summary, slices}`. It keeps full precision, so prefer it.

- `meta`:
  - `format: "cuspis-aerodb-1"`, `name`, `family`, `method`, `designMach`, `altitude_km`.
  - `sRef_m2`, `lRef_m`, `bRef_m`, `mass_kg` (smeared structural mass or the `massKg` override), `gamma`, `cg_m`.
  - `units {angles: "deg", derivatives: "1/rad", force: "wind axes, CL CD CY", moment: "body axes, Cm Cn Cll, nondim by q·Sref·Lref"}`.
  - `frame`, `index`, `generatedAt`, `assumptions[]`.
- `axes {mach, alpha_deg, beta_deg, delta_deg}`.
- `points[i]` with flat index `i = ((im·nAlpha + ia)·nBeta + ib)·nDelta + id`, so δ varies fastest. Fields: `mach, alpha_deg, beta_deg, delta_deg, onDesign, cl, cd, cy, cm, cn, cll, ca, ld, cla, cda, cma, cyb, cnb, clb, cld, cmd, clq, cmq, clp, cnp, cyp, cnr, clr, cyr, euler_cl, euler_cd, euler_cm`. Values not computed are `null`.
- `dynamics[]` has one row per Mach: `mach, alpha_deg, delta_deg, clq, cmq, clp, cnp, cyp, cnr, clr, cyr`.

**`aerodb.csv`** has `#` header lines first, then one row per point in the same order:

```
# cuspis aerodynamic database cuspis-aerodb-1
# name: … / family / method / design_mach / altitude_km / sref_m2 / lref_m / bref_m / mass_kg / gamma / cg_x_m / cg_y_m / cg_z_m
# angles: deg    derivatives: per radian
# frame: X streamwise (tip at origin, flow +X), Y span, Z up
# on_design marks Mach within half a Mach step of the inverse-design Mach
# <assumptions joined by " | ">
mach,alpha_deg,beta_deg,delta_deg,on_design,CL,CD,CY,Cm,Cn,Cll,CA,LD,CLa_per_rad,CDa_per_rad,Cma_per_rad,CYb_per_rad,Cnb_per_rad,Clb_per_rad,CLd_per_rad,Cmd_per_rad,CLq,Cmq,Clp,Cnp,Cyp,Cnr,Clr,Cyr,euler_CL,euler_CD,euler_Cm
```

Numbers are written with `toFixed(6)`, switching to `toExponential(6)` when |v| ≥ 1e4 or 0 < |v| < 1e-4. An empty cell means not computed. The CSV therefore loses precision below about 5e-7.

**`cuspis_aero_db.m`** is a MATLAB function, `cuspis_aero_db`. It returns `aero` with the breakpoints, `reshape(…, nD, nB, nA, nM)` tables, `aero.dyn.*`, `Sref`, `Lref`, `Bref`, `mass_kg` and `gamma`, ready for an n-D Lookup Table.

## Units, references and conventions (verified in code)

| quantity | definition | where |
| --- | --- | --- |
| Angles | degrees everywhere in files and configs | `meta.units.angles` |
| Sref | `built.aero.planformArea` (m²) of the undeflected vehicle → `meta.sRef_m2` | aerodb.ts:552 |
| Lref (pitch **and** yaw/roll moments) | `params.length` (m) → `meta.lRef_m` | aerodb.ts:563; panel.ts `cm/cn/cll = M/(S·L)` |
| bRef | `params.span` (m) → `meta.bRef_m`. Used **only** for the p̂ and r̂ rate scaling, not for Cn or Cll. | aerodb.ts:446 |
| CL, CD, CY | wind axes, on q·Sref. CD includes skin friction (Cf on the whole wetted area). | panel.ts `rotateForce`, `Dfric` |
| CA | body-axis pressure axial force on q·Sref. **No friction** (it is added to drag only). | panel.ts `ca = Fx·q/(q·S)` |
| Cm, Cn, Cll | moments about `cg = [clamp(cgFrac, 0.2, 0.85)·L, 0, 0]` (panel axes), all on **q·Sref·Lref**, with Lref = length | panel.ts:326 |
| Static derivatives `*_per_rad` | table differences **per radian**. Interior stations are central, `(f[i+1] − f[i−1]) / ((x[i+1] − x[i−1])·π/180)`. Axis ends are **one-sided** (`stencil()`). This changed in 55d643f; at e586e26 the edges were 0. An axis with a single breakpoint gives 0. CLα, CDα, Cmα run along α; CYβ, Cnβ, Clβ along β; CLδ, Cmδ along δ. There is no CDδ. | aerodb.ts `stencil`, 393–426 |
| Rate derivatives | **central** pair, hat step ±0.02: `Cmq = (Cm(q̂=+0.02) − Cm(q̂=−0.02))/0.04`, with q̂ = qL/2V, p̂ = pb/2V, r̂ = rb/2V. This changed in 55d643f; it was one-sided before. Rates are applied as panel-axis body rates (local V = V∞·v̂ − ω×r). | aerodb.ts:447–470; panel.ts (vx = V·v̂x − (q·rz − r·ry)) |
| Where rates are evaluated | α nearest the design α, β nearest 0, δ nearest 0, **once per Mach**. Stored in `dynamics[]` and copied onto that one anchor point. Every other point's `CLq…Cyr` is **0**, so do not interpolate them over the grid (the Python loader refuses to). | aerodb.ts:430–480 |
| Frame | X streamwise (tip at origin, flow +X), Y span, Z up | `meta.frame` |

Findings from checking `panel.ts` and the tables. These are documented here; the code is unchanged.

1. **The β sign is the opposite of the usual body convention.** The flow direction is `[cosα cosβ, sinβ, sinα cosβ]` (panel.ts `flowDir`), so +β is flow toward +Y. On caret-m8 at M 8 (nx 20, ny 14, cbaero) the static derivatives come out as:
   - CYβ = **+0.189 /rad**. The standard stability-axis convention would give a negative value.
   - Cnβ = +0.0066 /rad.
   - Clβ = −0.0228 /rad.

   To use the table in a standard right-handed body frame, flip the sign of β, or equivalently of CY, Cn and Cll, after checking your own frame.
2. **Other signs on the same vehicle:**
   - Cmα = −0.012, CLα = +0.57, CLδ = +0.195, Cmδ = −0.053 (all /rad).
   - Damping derivatives (central pair, 55d643f): Cmq = −0.052, Clp = −0.019, Cnr = −0.033.
3. **The CG differs between meta and solver.** `meta.cg_m` uses `clamp(cgFrac, 0.05, 0.95)·L` (aerodb.ts:551). The moments are taken about `clamp(cgFrac, 0.2, 0.85)·L` (panel.ts:326). When `cgFrac` is outside 0.2–0.85, the CG in the file is not the moment reference.
4. **Sref varies per δ.** Each deflected vehicle uses its own `planformArea` (`meshFor` in aerodb.ts). On caret-m8 that is 4.41372 m² at δ = ±5° vs 4.41393 m² at δ = 0, a relative difference of about 5e-5. `meta.sRef_m2` is the undeflected value.
5. **Yaw and roll moments use length, not span, as the reference length.** Dividing Cn and Cll by bRef/Lref converts them to the usual span-referenced coefficients.
6. **Friction depends on the reference length.** It is evaluated at Re = Re/m · lRef (panel.ts `ReBody = atm.ReL * L`), so rescaling Lref is not a pure post-processing step.

## Python (`python/`)

`pip install -e "python[openmdao,test]"`; numpy and pandas are required and OpenMDAO is optional. See `python/README.md`. In short:

- `run_aerodb(cfg_or_path, out, mode=…, reproducible=…)` drives the CLI. It forwards SIGINT and raises `AeroDbError(returncode, stderr)` on failure.
- `load_database(out)` reads `aerodb.json`, or falls back to `aerodb.csv`. Point columns always use the CSV names.
- `db.grid("CL")` returns an array of shape `(nM, nα, nβ, nδ)`.
- `db.interpolator((...))` is a 4-D multilinear interpolant with analytic gradients and `clip`, `linear` or `raise` extrapolation. `db.dynamic_interpolator(("Cmq",…))` interpolates the rate derivatives over Mach.
- `openmdao_component(db, coefs, vec_size)` takes inputs `mach, alpha, beta, delta` (angles in deg) and declares sparse partials.
- `load_trajectory`, `load_table` (pareto, history, sweep, sensitivity) and `load_manifest` read the other outputs.

## Tests

| command | what |
| --- | --- |
| `npm run test:aerodb-headless` | CLI: args, validation paths, layout, manifest hashes, reproducibility, every mode, abort, process-level SIGINT → 130, bad config → 2, legacy eval |
| `npm run test:aerodb-validation` | literature validation (below) |
| `PYTHON=python3 npm run test:python` | pytest: loaders (JSON ≡ CSV, flat index, per-radian derivatives), interpolator, OpenMDAO `check_partials`, CLI error path, fly/sweep loaders |

### Literature validation (`src/lib/waverider/aerodb-validation.test.ts`)

Every reference value is computed in the test from the closed form or by an independent integration. The only tabulated numbers used are the NASA SP-3004 cone shock angles already in `validate-published.ts` (`SP3004_BETA`), and they only check the test's own RK4 integrator.

| case | source | checks main's |
| --- | --- | --- |
| Oblique shock θ–β–M, weak branch, detachment, Rankine–Hugoniot jumps, strong-shock limits | NACA Report 1135 (Ames Research Staff, 1953) | `math.ts` `betaFromThetaM`, `maxTheta`, `obliqueShock`; `datcom/gasdyn.ts` |
| Shock stand-off for a sphere and a cylinder | Billig, *J. Spacecraft Rockets* 4(6):822–823 (1967) | `math.ts` `billigStandoff`, `billigStandoffCylinder`; `refinement.ts` `billigStandoffOverR` |
| Cp,max from the Rayleigh pitot formula and its M→∞ limit; modified Newtonian on a hemisphere (CD = Cp,max/2) and a sharp cone (CA, CN closed forms) | Lees (1955); Anderson, *Hypersonic and High-Temperature Gas Dynamics*, ch. 3 | `panelAero` and `generateAeroDatabase` (cone CA including the 1/M² base term, CL, and CLα per radian vs the closed-form slope) |
| Conical flow: shock angle and surface Cp | Taylor & Maccoll, *Proc. R. Soc. A* 139:278 (1933); Anderson, *Modern Compressible Flow*, ch. 10; NASA SP-3004 | `math.ts` `solveConeShock`, `coneSurfaceStateExact`, `coneSurfaceCp` (β within 0.01°, Cp within 2e-3, cached Cp within 5e-3, M 2–12, θc 5–30°), Mach-wave and wedge bounds |

## Limits

- One altitude per table, and one Mach-independent geometry (design Mach shapes the vehicle).
- Rate derivatives come from one central ±0.02 pair at one point per Mach.
- Table points run `panelAero` with `forcesOnly` (55d643f), so no surface heating is computed. Heating comes from the interactive solve, not from the table.
- Static derivatives at axis ends are one-sided, so they are first order.
- Grids are capped at 480 points and 21 breakpoints per axis.
- `fly` mode freezes β and rate derivatives at the design point (see `fly.json` notes).
- The CLI runs everything in one Node process. There is no parallel batch over configs; drive that from a shell loop or Python.
