/**
 * Decks handed to external CFD. They do not mesh the volume — Pointwise / snappy
 * still builds that from the STL in the kit. These files carry freestream,
 * reference quantities, and a runnable case skeleton.
 */
import { atmosphere } from "./atmosphere";
import type { BuiltVehicle } from "./types";
import { CUSPIS_6DOF_PY } from "./cuspis-6dof-src";

export function su2Config(built: BuiltVehicle): string {
  const p = built.params;
  const M = p.lockFlight ? p.mach : p.flightMach;
  const atm = atmosphere(p.altKm, M, p.gamma);
  const name = p.name || "vehicle";
  return `% Cuspis SU2 skeleton — compressible RANS / Euler
% Mesh this vehicle in Pointwise from ${name}.stl, export ${name}.su2, then run.
% Freestream from US76 at ${p.altKm} km, Mach ${M}.
SOLVER= RANS
KIND_TURB_MODEL= SST
MATH_PROBLEM= DIRECT
RESTART_SOL= NO
MACH_NUMBER= ${M.toFixed(4)}
AOA= ${p.alphaDeg.toFixed(3)}
SIDESLIP_ANGLE= ${p.betaDeg.toFixed(3)}
FREESTREAM_PRESSURE= ${atm.p.toFixed(3)}
FREESTREAM_TEMPERATURE= ${atm.T.toFixed(3)}
FREESTREAM_DENSITY= ${atm.rho.toExponential(6)}
GAMMA_VALUE= ${p.gamma}
GAS_CONSTANT= 287.058
REYNOLDS_NUMBER= ${(atm.ReL * p.length).toExponential(6)}
REYNOLDS_LENGTH= ${p.length}
REF_ORIGIN_MOMENT_X= ${(p.cgFrac * p.length).toFixed(5)}
REF_ORIGIN_MOMENT_Y= 0.0
REF_ORIGIN_MOMENT_Z= 0.0
REF_LENGTH= ${p.length}
REF_AREA= ${Math.max(built.aero.planformArea, 1e-6).toFixed(6)}
REF_DIMENSIONALIZATION= DIMENSIONAL
MESH_FILENAME= ${name}.su2
MESH_FORMAT= SU2
MARKER_EULER= ( wall )
MARKER_FAR= ( farfield )
MARKER_SYM= ( symmetry )
MARKER_PLOTTING= ( wall )
MARKER_MONITORING= ( wall )
NUM_METHOD_GRAD= GREEN_GAUSS
CFL_NUMBER= 5.0
CFL_ADAPT= YES
ITER= 2000
CONV_RESIDUAL_MINVAL= -8
OUTPUT_FILES= RESTART, PARAVIEW
SCREEN_OUTPUT= ( INNER_ITER, RMS_DENSITY, LIFT, DRAG, MOMENT_Z )
`;
}

export interface TextFile {
  name: string;
  content: string;
}

export function openFoamFiles(built: BuiltVehicle): TextFile[] {
  const p = built.params;
  const M = p.lockFlight ? p.mach : p.flightMach;
  const atm = atmosphere(p.altKm, M, p.gamma);
  const a = Math.sin(p.alphaDeg * Math.PI / 180);
  const ca = Math.cos(p.alphaDeg * Math.PI / 180);
  const U = atm.V;
  const header = (cls: string, obj: string) => `FoamFile
{
    version     2.0;
    format      ascii;
    class       ${cls};
    object      ${obj};
}
`;
  return [
    {
      name: "openfoam/system/controlDict",
      content: `${header("dictionary", "controlDict")}
application     rhoCentralFoam;
startFrom       startTime;
startTime       0;
stopAt          endTime;
endTime         0.002;
deltaT          1e-7;
writeControl    adjustableRunTime;
writeInterval   2e-4;
adjustTimeStep  yes;
maxCo           0.3;
maxDeltaT       1e-6;
`,
    },
    {
      name: "openfoam/0/U",
      content: `${header("volVectorField", "U")}
dimensions      [0 1 -1 0 0 0 0];
internalField   uniform (${(U * ca).toFixed(3)} ${(U * Math.sin(p.betaDeg * Math.PI / 180)).toFixed(3)} ${(U * a).toFixed(3)});
boundaryField
{
    wall      { type noSlip; }
    farfield  { type freestreamVelocity; freestreamValue $internalField; }
    symmetry  { type symmetry; }
}
`,
    },
    {
      name: "openfoam/0/p",
      content: `${header("volScalarField", "p")}
dimensions      [1 -1 -2 0 0 0 0];
internalField   uniform ${atm.p.toFixed(3)};
boundaryField
{
    wall      { type zeroGradient; }
    farfield  { type freestreamPressure; freestreamValue $internalField; }
    symmetry  { type symmetry; }
}
`,
    },
    {
      name: "openfoam/0/T",
      content: `${header("volScalarField", "T")}
dimensions      [0 0 0 1 0 0 0];
internalField   uniform ${atm.T.toFixed(3)};
boundaryField
{
    wall      { type fixedValue; value uniform ${Math.max(300, p.twK).toFixed(1)}; }
    farfield  { type freestream; freestreamValue $internalField; }
    symmetry  { type symmetry; }
}
`,
    },
    {
      name: "openfoam/README.txt",
      content: `Cuspis OpenFOAM skeleton for rhoCentralFoam.
Freestream Mach ${M}, ${p.altKm} km, US76.
Velocity is in the Cuspis frame: +X downstream of the nose, +Z up.
Mesh the STL externally (snappyHexMesh or a Pointwise unstructured export)
and drop it in constant/polyMesh. Patch names expected: wall, farfield, symmetry.
This is a starting deck, not a validated case.
`,
    },
  ];
}

export function genericDeck(built: BuiltVehicle): string {
  const p = built.params;
  const M = p.lockFlight ? p.mach : p.flightMach;
  const atm = atmosphere(p.altKm, M, p.gamma);
  return JSON.stringify(
    {
      format: "cuspis-cfd-deck-1",
      name: p.name,
      family: p.family,
      frame: "X streamwise (tip at 0, flow +X), Y span, Z up",
      designMach: p.mach,
      flightMach: M,
      alpha_deg: p.alphaDeg,
      beta_deg: p.betaDeg,
      gamma: p.gamma,
      altitude_km: p.altKm,
      freestream: { T_K: atm.T, p_Pa: atm.p, rho: atm.rho, a_m_s: atm.a, V_m_s: atm.V, Re_per_m: atm.ReL },
      reference: {
        S_m2: built.aero.planformArea,
        L_m: p.length,
        b_m: p.span,
        cg_m: [p.cgFrac * p.length, 0, 0],
      },
      wallTemperature_K: p.twK,
      notes: [
        "Generic freestream deck for a solver that is not SU2 or OpenFOAM.",
        "Mesh is the watertight STL / STEP / Plot3D shipped next to this file.",
      ],
    },
    null,
    2,
  );
}

export function python6dofSource(): string {
  return CUSPIS_6DOF_PY;
}

export { CUSPIS_6DOF_PY };
