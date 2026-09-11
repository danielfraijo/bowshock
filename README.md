# Bowshock

Inverse-design **waverider CAD lab** — geometry, panel aero, heating, stability, 6DOF, 3DOF trajectory, and ramjet/scramjet cycle analysis. Export watertight STL / STEP / IGES / Plot3D for SolidWorks, FreeCAD, and Pointwise.

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

**Already cloned?** Pull the latest:

```bash
cd bowshock
git pull
npm install
npm run dev
```

| Command | What it does |
|---|---|
| `npm run dev` | Lab + analysis UI at http://localhost:8080 |
| `npm run typecheck` | TypeScript check |
| `npm run build` | Production build |

Stop the server with `Ctrl+C`.

---

## How to use the lab

1. **Toolkit** — **External** (waveriders / lifting bodies) or **Internal** (inlets, ramjets, scramjets, Busemann, integrated).
2. Pick a **family** on the left. The mesh rebuilds with the nose at the origin (white / grey axis triad). The lab view is black, white, and grey so the surface, seams, and shock sheet stay readable.
3. **Geom** — length, span, height, Mach, shock/cone, lid (top or bottom), camber, elevons, fins, grid density.
4. **Flight / Aero / Heat / Stab / 6DOF / Traj** (External) or **Cycle / Shocks / Heat** (Internal) — engineering analysis, not Navier–Stokes.
5. **Checks** — closed-form kernel tests (Rankine–Hugoniot, isentropic A/A*, Prandtl–Meyer, θ-β-M, Taylor–Maccoll, US76, Kantrowitz). All should read PASS.
6. **CAD** — download STL, STEP (faceted or NURBS), IGES, Plot3D, OBJ, or the **full CFD kit (.zip)**.

The kit zip contains the mesh, `design.json`, `analysis.json`, `waverider_cad.py`, `bowshock_aero.c`, and `bowshock.cpp`.

Meshes are zipper-closed: the leading edge is a sharp seam (upper = lower), delta tips collapse to a point, and degenerate caps are skipped. Closed families should read **watertight / manifold** on the CAD tab.

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

## C / C++ kernels (optional, no browser)

Laptop-cheap verification of the same gas-dynamic relations used in the UI.

```bash
gcc -O2 -std=c11 public/bowshock_aero.c -lm -o bowshock_aero
./bowshock_aero --check
./bowshock_aero --wedge --mach 8 --theta 8
./bowshock_aero --atm --mach 8 --alt 30
./bowshock_aero --ramjet --mach 6 --phi 0.9
./bowshock_aero --heat --mach 8 --alt 30 --rn 0.01
./bowshock_aero --traj --mach 8 --alt 30 --gamma -1

g++ -O2 -std=c++17 public/bowshock.cpp -o bowshock
./bowshock --check
./bowshock --6dof --mach 8 --alt 30 --alpha 2 --CLa 1.8 --Cma -0.35 --Cmq -1.2
./bowshock --json --mach 8 --alt 30
```

`--check` must print **PASS** (normal-shock \(p_2/p_1 = 4.5\) at M=2, isentropic \(T_0/T = 6\) at M=5, Sutton–Graves units, US76). The C++ binary integrates the Etkin linear 6DOF (short period, phugoid, dutch roll, roll, spiral) with RK4.

---

## Physics (what this is / is not)

CBAERO-class **engineering** methods. Fast enough to iterate on a laptop. Not a Navier–Stokes substitute.

- **Shocks:** θ-β-M, Rankine–Hugoniot oblique + normal, Taylor–Maccoll RK4 (Sims NASA SP-3004)
- **Expansion:** Prandtl–Meyer
- **Panel aero:** attached tangent-wedge (2-D families) or tangent-cone (axisymmetric families); Modified Newtonian (Lees) if the shock detaches; Prandtl–Meyer leeward; Love base \(C_p = -1/M^2\); van Driest II \(C_f\) (Hopkins & Inouye 1971)
- **Heating:** Sutton–Graves stagnation (NASA TR R-802, \(k = 1.83\times 10^{-8}\) W/cm²), Tauber running-length (NASA TP-2914), Tauber–Sutton radiative (JSR 1991)
- **Stability:** finite-difference \(C_{L\alpha}\), \(C_{m\alpha}\), \(C_{n\beta}\), \(C_{l\beta}\), static margin
- **Rotary derivatives:** local velocity \(V_\infty + \omega \times r_{cg}\) on every panel → \(C_{mq}\), \(C_{lp}\), \(C_{nr}\), \(C_{lr}\), \(C_{np}\) (Etkin)
- **6DOF:** Etkin / Nelson linear modes (short period, phugoid, dutch roll, roll subsidence, spiral) + RK4 time history (400 steps × 0.02 s) from a +2° α pulse. Inertias from Mirtich tetrahedra.
- **Trajectory:** RK4 3DOF point-mass on the panel polar
- **Inlets / engines:** multi-ramp θ-β-M, Fanno isolator, Rayleigh heat addition, isentropic nozzle, Kantrowitz start, Heiser–Pratt 1-D cycle
- **Atmosphere:** 1976 US Standard Atmosphere
- **Geometry:** inverse-design lofts with a sharp-edge zipper (no LE/TE sliver walls, zero-chord tips collapse to a point)
- **Checks tab:** Anderson / NACA 1135 / Sims / US76 / Kantrowitz identities — same solvers the vehicle uses

---

## License

MIT. See [LICENSE](LICENSE).
