/**
 * The Bowshock benchmark suite. Every entry times a REAL hot path of the app:
 * geometry per family, panel aero / shock-expansion / Euler, stability sweeps, 6DOF, trajectory,
 * exports and the full "slider moved" live recompute. Inputs vary per iteration so memo caches
 * cannot turn a compute benchmark into a lookup benchmark.
 */
import type { BenchSpec } from "./harness";
import { atmosphere } from "../../src/lib/waverider/atmosphere";
import { buildVehicle } from "../../src/lib/waverider/generate";
import { panelAero, finiteStab } from "../../src/lib/waverider/panel";
import { integrateShockExpansion } from "../../src/lib/waverider/shockexp";
import { solveSixDof } from "../../src/lib/waverider/sixdof";
import { integrateGlide } from "../../src/lib/waverider/trajectory";
import { massProperties } from "../../src/lib/waverider/mass";
import { studyVehicle, flightMachOf } from "../../src/lib/waverider/study";
import { runValidation } from "../../src/lib/waverider/validate";
import { eulerWedge } from "../../src/lib/waverider/euler2d";
import { fileBlobs, cfdZip } from "../../src/lib/waverider/export-kit";
import { DEFAULT_PARAMS, type DesignParams, type WaveriderFamily, type BuiltVehicle } from "../../src/lib/waverider/types";

export const FAMILIES: WaveriderFamily[] = [
  "caret", "cone", "osculating", "viscopt", "elliptic", "wedgecone", "star",
  "liftbody", "ramjet", "scramjet", "busemann", "inward", "integrated",
];

export function paramsFor(family: WaveriderFamily, extra: Partial<DesignParams> = {}): DesignParams {
  return {
    ...DEFAULT_PARAMS,
    family,
    flowThrough: family === "ramjet" || family === "scramjet",
    ...extra,
  };
}

/** A pseudo-random but deterministic slider position in [lo, hi] for iteration i. */
export function slide(i: number, lo: number, hi: number): number {
  const t = ((i * 0.6180339887) % 1 + 1) % 1;
  return lo + (hi - lo) * t;
}

const SIX_FAMILIES: WaveriderFamily[] = ["caret", "osculating", "liftbody"];

function sref(b: BuiltVehicle) {
  return Math.max(b.aero.planformArea, 1e-6);
}

export function buildSuite(): BenchSpec[] {
  const specs: BenchSpec[] = [];

  // ---- geometry per family (default grid, and the export-density grid for two families)
  for (const fam of FAMILIES) {
    specs.push({
      name: `geom/${fam}`,
      group: "geom",
      maxRuns: 80,
      setup: () => (i) => buildVehicle(paramsFor(fam, { span: 2.0 + (i % 17) * 0.01 })),
    });
  }
  for (const fam of ["caret", "osculating"] as WaveriderFamily[]) {
    specs.push({
      name: `geom/${fam}@96x72`,
      group: "geom",
      maxRuns: 40,
      setup: () => (i) => buildVehicle(paramsFor(fam, { nx: 96, ny: 72, span: 2.0 + (i % 17) * 0.01 })),
    });
  }

  // ---- panel aero by method
  for (const fam of SIX_FAMILIES) {
    for (const method of ["cbaero", "mixed", "euler"] as const) {
      specs.push({
        name: `aero/${method}/${fam}`,
        group: "aero",
        maxRuns: 80,
        setup: () => {
          const p = paramsFor(fam, { aeroMethod: method });
          const b = buildVehicle(p);
          const atm = atmosphere(p.altKm, flightMachOf(p), p.gamma);
          const s = sref(b);
          return (i) => panelAero(b.mesh, p, atm, slide(i, 0.5, 9), 0, s, p.length);
        },
      });
    }
  }
  specs.push({
    name: "aero/shockexp-march/caret",
    group: "aero",
    setup: () => {
      const p = paramsFor("caret", { aeroMethod: "mixed" });
      const b = buildVehicle(p);
      const atm = atmosphere(p.altKm, flightMachOf(p), p.gamma);
      return (i) => integrateShockExpansion(b.mesh, p, atm, slide(i, 0.5, 9), 0, sref(b), p.length);
    },
  });

  // ---- stability sweep (full: alpha +-, beta, trim, 6-pt polar)
  for (const method of ["cbaero", "euler"] as const) {
    specs.push({
      name: `stab/finiteStab/${method}/caret`,
      group: "stab",
      maxRuns: 30,
      setup: () => {
        const base = paramsFor("caret", { aeroMethod: method });
        const b = buildVehicle(base);
        const atm = atmosphere(base.altKm, flightMachOf(base), base.gamma);
        return (i) => finiteStab(b.mesh, { ...base, alphaDeg: slide(i, 0, 4) }, atm, sref(b), base.length, true);
      },
    });
  }

  // ---- 6DOF + mass + trajectory
  for (const fam of ["caret", "liftbody"] as WaveriderFamily[]) {
    specs.push({
      name: `sixdof/full/${fam}`,
      group: "sixdof",
      maxRuns: 30,
      setup: () => {
        const base = paramsFor(fam);
        const b = buildVehicle(base);
        const mass = massProperties(b.mesh, base.rhoKgM3, 0);
        return (i) => {
          const p = { ...base, alphaDeg: slide(i, 0, 4) };
          const atm = atmosphere(p.altKm, flightMachOf(p), p.gamma);
          return solveSixDof(b.mesh, p, atm, mass, sref(b), p.length, true);
        };
      },
    });
  }
  specs.push({
    name: "mass/massProperties/caret@96x72",
    group: "sixdof",
    setup: () => {
      const b = buildVehicle(paramsFor("caret", { nx: 96, ny: 72 }));
      return () => massProperties(b.mesh, 160, 0);
    },
  });
  specs.push({
    name: "traj/integrateGlide/caret",
    group: "traj",
    setup: () => {
      const p0 = paramsFor("caret", { aeroMethod: "cbaero" });
      const b = buildVehicle(p0);
      const atm = atmosphere(p0.altKm, flightMachOf(p0), p0.gamma);
      const stab = finiteStab(b.mesh, p0, atm, sref(b), p0.length, true);
      const mass = massProperties(b.mesh, 160, 0);
      return (i) => integrateGlide({ ...p0, gammaDeg: -slide(i, 0, 5) }, stab.polar, mass.mass, sref(b), 0.02);
    },
  });

  // ---- Euler kernels
  specs.push({
    name: "euler/hllc-wedge-ts(M=2,10deg)",
    group: "euler",
    note: "TS 2-D HLLC wedge (Checks tab / fallback); inputs vary per call to defeat its memo",
    maxRuns: 20,
    setup: () => (i) => eulerWedge(2 + (i % 50) * 0.01 + i * 1e-6, 10 + (i % 7) * 0.1, 1.4),
  });
  specs.push({
    name: "euler/symmetry-plane(upper+lower via panelAero)",
    group: "euler",
    note: "panelAero(euler) minus nothing: wasm solve x2 + blend (cbaero base included)",
    setup: () => {
      const p = paramsFor("caret", { aeroMethod: "euler" });
      const b = buildVehicle(p);
      const atm = atmosphere(p.altKm, flightMachOf(p), p.gamma);
      return (i) => panelAero(b.mesh, p, atm, 0.2 + i * 0.07, 0, sref(b), p.length);
    },
  });

  // ---- validation (Checks tab)
  specs.push({
    name: "validate/runValidation",
    group: "validate",
    maxRuns: 30,
    setup: () => () => runValidation(),
  });

  // ---- exports (export-density mesh as the app does: nx>=96, ny>=72)
  const exp = () => {
    const b = buildVehicle(paramsFor("caret", { nx: 96, ny: 72 }));
    return fileBlobs(b);
  };
  const exportItems: [string, (f: ReturnType<typeof fileBlobs>) => Blob][] = [
    ["stl-binary", (f) => f.stlBin],
    ["stl-ascii", (f) => f.stlAscii],
    ["stl-regions", (f) => f.stlRegions],
    ["step-nurbs", (f) => f.stepNurbs],
    ["step-tess", (f) => f.stepTess],
    ["step-facet", (f) => f.stepFacet],
    ["iges", (f) => f.iges],
    ["plot3d", (f) => f.plot3d],
    ["obj", (f) => f.obj],
    ["vtk", (f) => f.vtk],
  ];
  for (const [name, get] of exportItems) {
    specs.push({
      name: `export/${name}`,
      group: "export",
      maxRuns: 20,
      setup: () => {
        const b = buildVehicle(paramsFor("caret", { nx: 96, ny: 72 }));
        return () => get(fileBlobs(b));
      },
    });
  }
  void exp;
  specs.push({
    name: "export/kit-zip",
    group: "export",
    maxRuns: 6,
    minRuns: 3,
    budgetMs: 4000,
    setup: () => {
      const b = buildVehicle(paramsFor("caret", { nx: 96, ny: 72 }));
      return async () => (await cfdZip(b, "# py", "/* c */", "{}", "// cpp", undefined, "/* k */")).size;
    },
  });

  // ---- live recompute: exactly what Designer does when a slider moves
  for (const fam of ["caret", "osculating", "liftbody", "scramjet"] as WaveriderFamily[]) {
    specs.push({
      name: `live/flight-slider-drag/${fam}`,
      group: "live",
      note: "alpha/Mach slider while dragging: studyVehicle(quick) on the existing geometry",
      maxRuns: 40,
      setup: () => {
        const base = paramsFor(fam);
        const b = buildVehicle(base);
        return (i) => studyVehicle({ ...b, params: { ...base, alphaDeg: slide(i, 0, 6), flightMach: 8, lockFlight: false } }, { quick: true });
      },
    });
    specs.push({
      name: `live/flight-slider-release/${fam}`,
      group: "live",
      note: "alpha slider released: studyVehicle(full) on existing geometry",
      maxRuns: 20,
      setup: () => {
        const base = paramsFor(fam);
        const b = buildVehicle(base);
        return (i) => studyVehicle({ ...b, params: { ...base, alphaDeg: slide(i, 0, 6) } }, {});
      },
    });
    specs.push({
      name: `live/geom-slider-drag/${fam}`,
      group: "live",
      note: "span slider while dragging: buildVehicle + studyVehicle(quick)",
      maxRuns: 30,
      setup: () => (i) => {
        const p = paramsFor(fam, { span: slide(i, 1.6, 2.6) });
        return studyVehicle(buildVehicle(p), { quick: true });
      },
    });
    specs.push({
      name: `live/geom-slider-release/${fam}`,
      group: "live",
      note: "span slider released: buildVehicle + studyVehicle(full)",
      maxRuns: 15,
      setup: () => (i) => {
        const p = paramsFor(fam, { span: slide(i, 1.6, 2.6) });
        return studyVehicle(buildVehicle(p), {});
      },
    });
  }
  return specs;
}
