# Cuspis

Inverse-design **waverider CAD** — Roma, hypersonic geometry. Panel aero, heating, stability, 6DOF, 3DOF trajectory, and ramjet/scramjet cycle analysis. Export watertight STL / sewn NURBS STEP / IGES / Plot3D for SolidWorks, FreeCAD, and Pointwise.

Live lab: **[bowshock.vercel.app](https://bowshock.vercel.app/)**

The browser lab and the physics kernel run **on your machine**. Nothing is uploaded.

**Origin frame:** vehicle tip (or cowl lip) at `(0, 0, 0)`. **X** streamwise (flow +X), **Y** spanwise, **Z** up.

**Leading edge:** always a circular fillet of finite radius (toggle + size on the Geom tab). Default \(R = 0.5\%\) of length. Pointwise automatic mesh needs this — a knife-edge is inviscid-theory only.

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

That is the same lab as the live site: 3D viewer, External / Internal toolkit, live analysis, and CAD export.

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
3. **Geom** — length, span, height, Mach, shock/cone, lid (top or bottom), camber, elevons, fins, grid density, **rounded leading edge** (on by default) and **nose / LE radius**.
4. **Flight / Aero / Heat / Frontier / Stab / 6DOF / Traj** (External) or **Cycle / Shocks / Heat / Frontier** (Internal) — engineering analysis, not Navier–Stokes. Switch the 3D view to **Heat** for rainbow q (W/cm²) or **Cp**. Surface mode stays black / white / grey.
5. **Checks** — closed-form kernel tests (Rankine–Hugoniot, isentropic A/A*, Prandtl–Meyer, θ-β-M, Taylor–Maccoll, US76, Kantrowitz, γ_vib, mean free path, Fay–Riddell, Billig, Lees, Millikan–White, Waltrup–Billig). All should read PASS.
6. **CAD** — download binary STL (Pointwise auto-mesh), Plot3D `.x` (3-D formatted), IGES, sewn NURBS STEP (SolidWorks / FreeCAD / Pointwise Database), or the **full CFD kit (.zip)**.

The kit zip contains the mesh, `design.json`, `analysis.json`, `waverider_cad.py`, `bowshock_aero.c`, and `bowshock.cpp`.

The leading edge is a circular G1 fillet in the local osculating plane. Tips keep a minimum chord so they do not collapse to a pole. Closed families should read **watertight / manifold** on the CAD tab.

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

- **Pointwise (automatic mesh)** — File → Import → **STL** (binary). The nose is a circular fillet. Assemble a domain, T-Rex off the walls. Or File → Import → **IGES** (type-128) / **STEP** (sewn NURBS CLOSED_SHELL) and run the unstructured solver on the database. Plot3D **`.x`**: 3-D formatted, IBLANK off. Do **not** import STEP as XYZ points — that is the cyan cloud.
- **SolidWorks** — File → Open → `_nurbs.step` (degree-3 B-splines, sewn solid). Faceted STEP is a closed tessellation if NURBS knit fails.
- **FreeCAD** — Part → Import `_nurbs.step` or `.igs`.

Ramjet / scramjet: both surfaces run from x=0 to x=L. The capture plane (rectangular inlet) is at the origin and the nozzle is at x=L. Enable **flow-through** to leave those faces open for internal CFD.

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

- **Shocks:** θ-β-M, Rankine–Hugoniot oblique + normal, Taylor–Maccoll RK4 (Sims NASA SP-3004), Billig 1967 sphere standoff
- **Expansion:** Prandtl–Meyer
- **Panel aero:** attached tangent-wedge (2-D families) or tangent-cone (axisymmetric families); Modified Newtonian (Lees) if the shock detaches; Prandtl–Meyer leeward; Love base \(C_p = -1/M^2\); van Driest II \(C_f\) (Hopkins & Inouye 1971) or Blasius if laminar; Hayes–Probstein viscous interaction on windward \(p\); Schaaf–Chambre rarefaction bridging
- **Heating:** Fay–Riddell 1958 (Billig-corrected \(du_e/ds\)), Sutton–Graves (NASA TR R-802), Detra–Kemp–Riddell, Tauber running-length (NASA TP-2914) mixed with Lees 1956 \(q/q_s = (p/p_s)^{1/2}(R_n/(R_n+s))^{1/2}\), Tauber–Sutton radiative (JSR 1991), Beckwith swept-cylinder LE, Edney Type-IV bound at cowls; Reshotko / Mack \(Re_\theta/M_e\) transition
- **Real gas:** vibrational-equilibrium \(\gamma(T)\) (Hansen / SHO O₂–N₂); Millikan–White / Park 1990 \(\tau_v\) and Damköhler freeze/eq blend; Lighthill ideal-dissociating gas \(\alpha_{O_2},\alpha_{N_2}\); Gupta–Yos-class \(\mu(T)\) above 1500 K. Inverse-design shocks stay at the input \(\gamma\).
- **Stability:** 4th-order Richardson \(C_{L\alpha}\), \(C_{m\alpha}\), \(C_{n\beta}\), \(C_{l\beta}\); static margin
- **Rotary derivatives:** local velocity \(V_\infty - \omega \times r_{cg}\) on every panel → \(C_{mq}\), \(C_{lp}\), \(C_{nr}\), \(C_{lr}\), \(C_{np}\) (Etkin)
- **6DOF:** Etkin / Nelson linear modes (short period, phugoid, dutch roll, roll subsidence, spiral) + RK4 time history (400 steps × 0.02 s) from a +2° α pulse. Inertias from Mirtich tetrahedra.
- **Trajectory:** adaptive RK4 3DOF (step doubling / Richardson) on the panel polar, Knudsen CD growth, Fay–Riddell heating, 3 mm C/C lumped \(T_w\), peak-\(n\) / max-\(q\) / heat load
- **Inlets / engines:** multi-ramp θ-β-M, Fanno isolator, Waltrup–Billig shock-train \(L/H\), Rayleigh heat addition, isentropic nozzle, Kantrowitz start, Heiser–Pratt 1-D cycle
- **Atmosphere:** 1976 US Standard Atmosphere + kinetic mean free path / Knudsen
- **Geometry:** inverse-design lofts with a circular LE fillet (G1, osculating-plane rolling ball). Tips cropped to a minimum chord. Closed manifold solid for STL; sewn NURBS CLOSED_SHELL for STEP
- **Checks tab:** Anderson / NACA 1135 / Sims / US76 / Kantrowitz / γ_vib / λ / Fay–Riddell / Billig / Lees / Millikan–White / Waltrup–Billig identities — same solvers the vehicle uses

**Frontier tab** reports regime, Kn, \(\gamma_\mathrm{vib}(T_2)\), \(\gamma_\mathrm{eff}(Da)\), Lighthill \(\alpha\), \(\rho L\), Billig \(\Delta/R_n\), Van Dyke \(K=M\tau\), \(\bar\chi\), Fay–Riddell vs Sutton–Graves vs DKR, sweep factor, Edney bound, isolator \(L/H\), and equilibrium-glide CL.

---

## Public website

- Lab: [bowshock.vercel.app](https://bowshock.vercel.app/)
- Source: [github.com/danielfraijo/bowshock](https://github.com/danielfraijo/bowshock)

Each `git push` to `main` rebuilds the site.
