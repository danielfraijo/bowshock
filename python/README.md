# bowshock (Python)

Thin Python layer over the repo's headless CLI (`npm run aerodb`, see `docs/aerodb.md`).

```bash
python -m venv .venv && . .venv/bin/activate
pip install -e "python[openmdao,test]"     # openmdao is optional
```

```python
from bowshock import run_aerodb, load_database, openmdao_component

out = run_aerodb("examples/aerodb-caret.json", "aerodb-out", reproducible=True)   # or a dict config
db = load_database(out)          # reads aerodb.json (full precision); falls back to aerodb.csv
db.points                        # pandas, CSV column names: CL, CD, Cm, CLa_per_rad, ...
db.grid("CL").shape              # (n_mach, n_alpha, n_beta, n_delta)
f = db.interpolator(("CL", "CD", "Cm"))
f(mach=7.2, alpha_deg=3.1, beta_deg=0.0, delta_deg=0.0)
db.dynamic_interpolator(("Cmq", "Clp", "Cnr"))(mach=7.2)    # rate derivatives live in db.dynamics (one row per Mach)
comp = openmdao_component(db, ("CL", "CD"), vec_size=4)    # inputs mach, alpha, beta, delta [deg]
```

Other modes: `run_aerodb(cfg, out, mode="fly")` then `load_trajectory(out)`; `load_table(out / "pareto.csv")`,
`sweep.csv`, `sensitivity.csv`, `history.csv`; `load_manifest(out)`.

`bowshock-aerodb --config cfg.json --out dir` is a console-script wrapper that forwards to `npm run aerodb` (the repo is
found from `$BOWSHOCK_REPO` or by walking up to the `package.json` named `bowshock`).

Tests: `PYTHON=/path/to/python npm run test:python` (needs numpy, pandas, pytest; node/npm on PATH for the CLI tests;
OpenMDAO tests are skipped when it is not installed).
