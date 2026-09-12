import type { BuiltVehicle, DesignParams } from "./types";
import { FAMILY_META } from "./types";
import { fmt } from "./math";

export function designJson(p: DesignParams): string {
  return JSON.stringify(p, null, 2) + "\n";
}

export function readmeFor(built: BuiltVehicle): string {
  const p = built.params;
  const q = built.quality;
  const a = built.aero;
  const fam = FAMILY_META[p.family];
  return `CUSPIS — waverider CFD kit
================================
Family:     ${fam.label} (${p.family})
Generating: ${fam.generating}
Name:       ${p.name}

Design
------
Mach            ${p.mach}
gamma           ${p.gamma}
Length          ${p.length} m  (export unit: ${p.unit})
Span            ${p.span} m
Height          ${p.height} m
Shock angle     ${p.shockDeg} deg
Cone angle      ${p.coneDeg} deg
Half-model      ${p.halfModel}
LE radius       ${p.leRadius} m  (blunt ${p.leBlunt !== false ? "ON" : "OFF"})
Lid             ${p.lid}
Dihedral        ${p.dihedralDeg} deg
Camber          ${p.camber}
TE sweep        ${p.teSweepDeg} deg
Elevon          ${p.elevonDeg} deg
Vert. fins h/H  ${p.finHeight}

On-design flow
--------------
theta           ${fmt(a.thetaDeg, 3)} deg
beta            ${fmt(a.betaDeg, 3)} deg
Mach angle      ${fmt(a.muDeg, 3)} deg
p2/p1           ${fmt(a.pressureRatio, 4)}
T2/T1           ${fmt(a.temperatureRatio, 4)}
M2              ${fmt(a.m2, 3)}
CL (inviscid)   ${fmt(a.cl, 4)}
CD (wave+base)  ${fmt(a.cd, 4)}
L/D             ${fmt(a.ld, 3)}
Volume          ${fmt(q.volume, 5)} m^3
Planform area   ${fmt(a.planformArea, 5)} m^2
Watertight      ${q.watertight ? "YES" : "NO"}
Manifold        ${q.manifold ? "YES" : "NO"}
Triangles       ${q.triangles}
Vertices        ${q.vertices}

Rounded leading edge
--------------------
When blunt is ON the knife-edge is replaced by a circular fillet of radius R
in the local osculating plane (G1 to upper and lower). Tips keep a minimum
chord so they do not collapse to a pole. Pointwise auto-mesh wants this.

Coordinates
-----------
X  streamwise, tip of the vehicle at the origin, flow +X
Y  spanwise
Z  up
The generator always translates the mesh so the nose / cowl lip is (0,0,0).
This is the same frame Pointwise, SU2, and most structured CFD codes expect.

SolidWorks
----------
File > Open > ${p.name}.step
  AP214 manifold solid, units = ${p.unit}.
  Prefer the NURBS STEP for filleting / thickening.
  Faceted STEP is a closed tessellated solid if NURBS knit fails.
  STL also opens via the mesh converter; do not use STL if you need a feature tree.

FreeCAD
-------
Part workbench > Import ${p.name}.step  (or File > Import)
  If you only have the STL: Mesh Design > Import, then Part > Create shape
  from mesh > Convert to solid. The STEP path is preferred.

Pointwise
---------
Do NOT import STEP as an XYZ point list — that is the cyan cloud.

Unstructured (recommended):
  File > Import > STL (binary)  ${p.name}.stl
  Watertight triangles. Assemble a domain, T-Rex from the walls.

Structured:
  File > Import > Plot3D
    3-D, Formatted ASCII, IBLANK off
    File: ${p.name}.x     (extension .x — not .xyz)
  Blocks are the surface patches (upper/cowl, lower, inlet, nozzle, sides)
  with coincident points on shared edges.

IGES NURBS (most reliable CAD surfaces in Pointwise):
  ${p.name}.igs  — type-128 B-splines.

NURBS STEP (CAD):
  ${p.name}_nurbs.step  — sewn degree-3 B-splines, CLOSED_SHELL solid,
  circular leading-edge face. SolidWorks / FreeCAD / Pointwise Database.
  If Pointwise lists poles, use the STL or IGES instead — that is a reader
  quirk, not a point cloud.

Glyph:
  File > Glyph > Execute  ${p.name}.glf
  Imports STL then IGES then Plot3D automatically.

Python (batch / parametric)
---------------------------
  python waverider_cad.py --config design.json --stl out.stl --step out.step
  python waverider_cad.py --type ${p.family} --mach ${p.mach} --length ${p.length} \\
      --span ${p.span} --height ${p.height} --stl ${p.name}.stl --step ${p.name}.step

C kernel (wedge L/D + atmosphere + ramjet + heat + 3DOF)
--------------------------------------------------------
  gcc -O2 -std=c11 bowshock_aero.c -lm -o bowshock_aero
  ./bowshock_aero --check
  ./bowshock_aero --wedge --mach ${p.mach} --theta ${fmt(a.thetaDeg, 2)}
  ./bowshock_aero --atm --mach ${p.mach} --alt 30
  ./bowshock_aero --ramjet --mach 6 --phi 0.9
  ./bowshock_aero --heat --mach ${p.mach} --alt 30 --rn ${Math.max(p.leRadius, 0.01)}
  ./bowshock_aero --traj --mach ${p.mach} --alt ${p.altKm} --gamma ${p.gammaDeg}

Zero third-party dependencies (Python 3.9+). C is optional verification.

Panel aero (CBAERO-class, not a Navier–Stokes substitute)
--------------------------------------------------------
Mixed method: attached tangent-wedge (θ-β-M) or tangent-cone (Taylor–Maccoll /
Sims) on the windward face; Modified Newtonian (Lees) if the shock detaches;
Prandtl–Meyer leeward; Love base Cp = −1/M²; van Driest II Cf (Hopkins–Inouye).
Heating: Sutton–Graves stagnation (TR R-802, q in W/cm² with k=1.83e-8) +
Tauber running-length (TP-2914) + Tauber–Sutton radiative (JSR 1991).
Stability: finite-difference CLα, Cmα, Cnβ, Clβ, static margin about CG x/L.
Mass: closed-mesh tetrahedron integrals. Trajectory: RK4 3DOF point-mass using
the panel polar. Ramjet/scram: Heiser–Pratt 1-D.
Use analysis.json as the quantitative dump.

These are engineering methods for configuration screening. Always follow
with a mesh in Pointwise / a solver (SU2, FUN3D, CFD++) before hardware.

CFD notes
---------
- Leading edges are circular fillets (toggle + radius on the Geom tab).
  Default R = 0.5 % of length. Heat flux and Pointwise auto-mesh both need
  a finite Rn; a knife-edge is inviscid-theory only.
- The base is a blunt aft face. Hypersonic base pressure is low; the
  kit's CD includes a simple p_base ~ 0 contribution.
- Freestream: +X. Typical farfield 10–20 body lengths upstream / sides,
  15–25 downstream if the wake matters.
- gamma = ${p.gamma}. For high-T air you will want a thermo model in the
  solver, not in the CAD.

${a.notes.length ? "Warnings\n--------\n" + a.notes.map((n) => "- " + n).join("\n") + "\n" : ""}Generated by Cuspis.
`;
}

export function pythonRunner(p: DesignParams): string {
  return `#!/usr/bin/env python3
"""Run the current Cuspis case. Requires waverider_cad.py in the same folder."""
from waverider_cad import Design, export_all

design = Design(
    family=${JSON.stringify(p.family)},
    mach=${p.mach},
    gamma=${p.gamma},
    length=${p.length},
    span=${p.span},
    height=${p.height},
    shock_deg=${p.shockDeg},
    cone_deg=${p.coneDeg},
    super_n=${p.superN},
    planform=${JSON.stringify(p.planform)},
    planform_power=${p.planformPower},
    fins=${p.fins},
    wedge_frac=${p.wedgeFrac},
    capture_frac=${p.captureFrac},
    nx=${p.nx},
    ny=${p.ny},
    le_radius=${p.leRadius},
    half_model=${p.halfModel ? "True" : "False"},
    lid=${JSON.stringify(p.lid)},
    dihedral_deg=${p.dihedralDeg},
    camber=${p.camber},
    te_sweep_deg=${p.teSweepDeg},
    elevon_deg=${p.elevonDeg},
    fin_height=${p.finHeight},
    n_ramps=${p.nRamps},
    cowl_side=${JSON.stringify(p.cowlSide)},
    inlet_height=${p.inletHeight},
    cowl_frac=${p.cowlFrac},
    combustor_frac=${p.combustorFrac},
    nozzle_er=${p.nozzleER},
    ramp_deg=${p.rampDeg},
    unit=${JSON.stringify(p.unit)},
    name=${JSON.stringify(p.name)},
)

if __name__ == "__main__":
    export_all(design, prefix=design.name)
    print("wrote", design.name + ".{stl,step,igs,xyz,obj}")
`;
}
