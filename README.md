# Bowshock

Inverse-design **waverider CAD lab** — geometry, panel aero, heating, stability, 3DOF trajectory, and ramjet/scramjet cycle analysis. Export watertight STL / STEP / IGES / Plot3D for SolidWorks, FreeCAD, and Pointwise.

The browser lab and the physics kernel run **on your machine**. Nothing is uploaded.

**Origin frame:** vehicle tip (or cowl lip) at `(0, 0, 0)`. **X** streamwise (flow +X), **Y** spanwise, **Z** up.

---

## Run the lab on your computer

You need **Node.js 20+** ([nodejs.org](https://nodejs.org/) — 22 LTS is what this was built on) and **git**.

```bash
git clone https://github.com/danielfraijo/bowshock.git
cd bowshock
npm install
npm run dev
```

Then open **[http://localhost:8080](http://localhost:8080)** in your browser.

That is the same lab as the live preview: 3D viewer, External / Internal toolkit, live analysis, and CAD export.

| Command | What it does |
|---|---|
| `npm run dev` | Lab + analysis UI at http://localhost:8080 |
| `npm run typecheck` | TypeScript check |
| `npm run build` | Production build |

Stop the server with `Ctrl+C`.

---

## How to use the lab

1. **Toolkit** — **External** (waveriders / lifting bodies) or **Internal** (inlets, ramjets, scramjets, Busemann, integrated).
2. Pick a **family** on the left. The mesh rebuilds with the nose at the origin (RGB triad).
3. **Geom** — length, span, height, Mach, shock/cone, lid (top or bottom), camber, elevons, fins, grid density.
4. **Flight / Aero / Heat / Stab / Traj** (External) or **Cycle / Shocks / Heat** (Internal) — engineering analysis, not Navier–Stokes.
5. **Checks** — closed-form kernel tests (Rankine–Hugoniot, isentropic A/A*, Prandtl–Meyer, θ-β-M, Taylor–Maccoll, US76, Kantrowitz). All should read PASS.
6. **CAD** — download STL, STEP (faceted or NURBS), IGES, Plot3D, OBJ, or the **full CFD kit (.zip)**.

The kit zip contains the mesh, `design.json`, `analysis.json`, `waverider_cad.py`, and `bowshock_aero.c`.

---

## Families

**External flow**

| Family | Generating flow |
|---|---|
| Caret | Nonweiler 2-D wedge (θ-β-M) |
| Cone | Jones–Moore–Pike osculating / conical |
| Osculating | Sobieczky osculating cones |
| Viscopt | Bowcutt / Corda viscous-optimized |
| Elliptic | Rasmussen elliptic cone |
| Wedge-cone | Mixed wedge + cone |
| Star | n-fold caret star |
| Lift body | HL-20 / HTV-class lifting body |

**Internal flow**

| Family | Notes |
|---|---|
| Ramjet | 2-D OML, multi-ramp + cowl, Heiser–Pratt 1-D |
| Scramjet | Dual-mode, no terminal normal shock |
| Busemann | Inward-turning scoop |
| Inward | REST / inward-turning |
| Integrated | Osculating waverider + underslung or dorsal ramjet |

---

## CAD without the browser (Python)

Zero third-party dependencies. Python 3.9+.

```bash
python3 public/waverider_cad.py --type caret --mach 8 --stl wr.stl --step wr.step
python3 public/waverider_cad.py --config design.json --out kit/
python3 public/waverider_cad.py --sweep-mach 5,6,8,10 --type osculating --out family/
python3 public/waverider_cad.py --type ramjet --mach 6 --all --out ramjet/
```

`--type` accepts: `caret`, `cone`, `osculating`, `viscopt`, `elliptic`, `wedgecone`, `star`, `liftbody`, `ramjet`, `scramjet`, `busemann`, `inward`, `integrated`.

Import:

- **SolidWorks** — File → Open → `.step` (prefer NURBS STEP for filleting).
- **FreeCAD** — Part → Import `.step`.
- **Pointwise** — unstructured: Import STL (binary); structured: Import Plot3D `.xyz`.

---

## C kernel (optional, no browser)

Laptop-cheap verification of the same gas-dynamic relations used in the UI.

```bash
gcc -O2 -std=c11 public/bowshock_aero.c -lm -o bowshock_aero
./bowshock_aero --check
./bowshock_aero --wedge --mach 8 --theta 8
./bowshock_aero --atm --mach 8 --alt 30
./bowshock_aero --ramjet --mach 6 --phi 0.9
./bowshock_aero --heat --mach 8 --alt 30 --rn 0.01
./bowshock_aero --traj --mach 8 --alt 30 --gamma -1
```

`--check` must print **ALL PASS** (normal-shock \(p_2/p_1 = 4.5\) at M=2, isentropic \(T_0/T = 6\) at M=5, inviscid wedge \(L/D = \cot\theta\)).

---

## Physics (what this is / is not)

CBAERO-class **engineering** methods. Fast enough to iterate on a laptop. Not a Navier–Stokes substitute.

- **Shocks:** θ-β-M, Rankine–Hugoniot oblique + normal, Taylor–Maccoll RK4
- **Expansion:** Prandtl–Meyer
- **Panel aero:** tangent-wedge windward + PM leeward + Newtonian blend + Love base + van Driest / Schlichting \(C_f\)
- **Heating:** Sutton–Graves stagnation, Tauber running-length, Tauber–Sutton radiative
- **Stability:** finite-difference \(C_{L\alpha}\), \(C_{m\alpha}\), \(C_{n\beta}\), \(C_{l\beta}\), static margin
- **Trajectory:** RK4 3DOF point-mass on the panel polar
- **Inlets / engines:** multi-ramp θ-β-M, Fanno isolator, Rayleigh heat addition, isentropic nozzle, Kantrowitz start, Heiser–Pratt 1-D cycle
- **Atmosphere:** 1976 US Standard Atmosphere

---

## License

MIT. See [LICENSE](LICENSE).
