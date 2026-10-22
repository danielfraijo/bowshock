/**
 * Preset vehicle library + aircraft (wing-body-tail) configuration builder.
 *
 * Basic Finner geometry follows the public description of the BRL/Army-Navy Finner (Murphy 1956):
 * 10-calibre cone-cylinder, 10 deg half-angle cone nose, four rectangular tail fins of 3-calibre
 * tip-to-tip span in 'x' orientation, trailing edge at the base. Fin chord (1 cal) and thickness
 * (0.1 cal assumed) are NOT verified against the original drawing — see NOTES.md.
 */
import type { FinSet, FlightSpec, VehicleConfig } from "./types.ts";
import { DEG } from "./util.ts";
import { buildBody, radiusAt } from "./geometry.ts";
import { liftOml, liftXs } from "../liftbody-oml.ts";

export interface Preset {
  id: string;
  name: string;
  blurb: string;
  config: VehicleConfig;
  flight: FlightSpec;
}

export function basicFinner(d = 0.03): VehicleConfig {
  const R = d / 2;
  const noseLen = R / Math.tan(10 * DEG);
  return {
    name: "Basic Finner (BRL/Army-Navy)",
    kind: "missile",
    body: {
      parametric: { noseKind: "cone", noseLength: noseLen, noseDiameter: d, centerLength: 10 * d - noseLen, aftLength: 0 },
      exitDiameter: 0,
    },
    sets: [
      {
        name: "fins",
        role: "fin",
        nPanels: 4,
        phiF: [45, 135, 225, 315],
        rootChord: d,
        tipChord: d,
        semispan: 1.5 * d,
        xLE: 9 * d,
        sweepLE: 0,
        thickness: 0.1,
        section: "wedge",
        leRadius: 0.0,
        cfoc: 1,
      },
    ],
    ref: { xcg: 5.5 * d, sRef: (Math.PI * d * d) / 4, lRef: d, roughness: 0 },
    notes: [
      "Geometry: 10-cal cone-cylinder, 10 deg cone half-angle, 3-cal tip-to-tip fins (Murphy/BRL description).",
      "Fin chord = 1 cal, t/c = 0.10 (wedge) assumed — not verified against the original BRL drawing.",
      "CG at 5.5 calibers from the nose (common published value; some reports use 6.1).",
    ],
  };
}

export function cruciformMissile(): VehicleConfig {
  return {
    name: "Generic cruciform missile",
    kind: "missile",
    body: {
      parametric: {
        noseKind: "ogive",
        noseLength: 0.4,
        noseDiameter: 0.15,
        centerLength: 1.6,
        aftLength: 0.2,
        aftKind: "cone",
        aftDiameter: 0.12,
      },
      exitDiameter: 0.08,
    },
    sets: [
      {
        name: "canards",
        role: "canard",
        nPanels: 4,
        phiF: [45, 135, 225, 315],
        rootChord: 0.12,
        tipChord: 0.06,
        semispan: 0.18,
        xLE: 0.45,
        sweepLE: 40,
        thickness: 0.04,
        section: "hex",
        flatFraction: 0.3,
        cfoc: 1,
      },
      {
        name: "tail",
        role: "tail",
        nPanels: 4,
        phiF: [45, 135, 225, 315],
        rootChord: 0.25,
        tipChord: 0.1,
        semispan: 0.28,
        xLE: 1.9,
        sweepLE: 35,
        thickness: 0.04,
        section: "hex",
        flatFraction: 0.3,
        cfoc: 0,
      },
    ],
    ref: { xcg: 1.2 },
    notes: ["Generic canard-tail cruciform missile — illustrative, not a published configuration."],
  };
}

export function canardMissile(): VehicleConfig {
  const c = cruciformMissile();
  c.name = "Generic canard missile";
  c.sets = [c.sets[0]];
  c.notes = ["Canard-only variant of the generic cruciform missile."];
  return c;
}

export function hypersonicConeCylinderFlare(): VehicleConfig {
  return {
    name: "Hypersonic cone-cylinder-flare",
    kind: "missile",
    body: {
      parametric: {
        noseKind: "cone",
        noseLength: 0.6,
        noseDiameter: 0.2,
        centerLength: 1.4,
        aftLength: 0.4,
        aftKind: "cone",
        aftDiameter: 0.3,
        noseBluntRadius: 0.01,
      },
      exitDiameter: 0,
    },
    sets: [],
    ref: { xcg: 1.4 },
    bodyMethod: "auto",
    notes: ["Cone-cylinder-flare body alone for hypersonic Newtonian/tangent-cone checks."],
  };
}

export function aircraftWBT(): VehicleConfig {
  const d = 1.2;
  return {
    name: "Generic aircraft wing-body-tail",
    kind: "aircraft",
    body: {
      parametric: {
        noseKind: "ogive",
        noseLength: 4,
        noseDiameter: d,
        centerLength: 8,
        aftLength: 3,
        aftKind: "cone",
        aftDiameter: 0.4,
      },
      exitDiameter: 0.3,
    },
    sets: [
      {
        name: "wing",
        role: "wing",
        nPanels: 2,
        phiF: [90, 270],
        rootChord: 3.5,
        tipChord: 1.2,
        semispan: 7,
        xLE: 5.5,
        sweepLE: 30,
        thickness: 0.12,
        section: "naca4",
        naca: "0012",
        cfoc: 0.25,
      },
      {
        name: "htail",
        role: "htail",
        nPanels: 2,
        phiF: [90, 270],
        rootChord: 1.8,
        tipChord: 0.8,
        semispan: 3.2,
        xLE: 13.5,
        sweepLE: 25,
        thickness: 0.1,
        section: "naca4",
        naca: "0010",
        cfoc: 0.3,
      },
      {
        name: "vtail",
        role: "vtail",
        nPanels: 1,
        phiF: [0],
        rootChord: 2.2,
        tipChord: 1.0,
        semispan: 2.5,
        xLE: 13.2,
        sweepLE: 35,
        thickness: 0.1,
        section: "naca4",
        naca: "0010",
        cfoc: 0.3,
      },
    ],
    ref: { xcg: 7.5 },
    notes: [
      "Generic transport-like wing-body-tail — illustrative configuration for the aircraft path.",
      "Interference and downwash are Missile-DATCOM-style slender-body / single-vortex methods.",
    ],
  };
}

export function searsHaackMissile(L = 4, rMax = 0.25): VehicleConfig {
  const n = 64;
  const stations = [];
  for (let i = 0; i <= n; i++) {
    const u = 0.5 * (1 - Math.cos((Math.PI * i) / n));
    const t = 4 * u * (1 - u);
    const r = i === 0 || i === n ? 0 : rMax * Math.pow(Math.max(t, 0), 0.75);
    stations.push({ x: u * L, r });
  }
  return {
    name: "Sears–Haack body",
    kind: "missile",
    body: { stations, exitDiameter: 0, baseDrag: false },
    sets: [],
    ref: { xcg: 0.5 * L, sRef: Math.PI * rMax * rMax, lRef: 2 * rMax },
    bodyMethod: "slender",
    notes: [
      "Closed Sears–Haack body: minimum wave drag for this length and volume in linearized supersonic flow (Sears 1947, Haack 1941).",
      "Area-rule D/q should sit on (9π/2)(Amax/L)². Tangent-cone is a different, nonlinear estimate and will not match.",
    ],
  };
}

export const DEFAULT_FLIGHT: FlightSpec = {
  machs: [0.3, 0.6, 0.9, 1.2, 1.5, 2, 3, 4, 5, 6, 8],
  alphas: [-4, -2, 0, 2, 4, 6, 8, 10, 12, 15, 20],
  beta: 0,
  phi: 0,
  altKm: 0,
};

export const PRESETS: Preset[] = [
  {
    id: "finner",
    name: "Basic Finner",
    blurb: "BRL/Army-Navy 10-calibre cone-cylinder with four rectangular fins.",
    config: basicFinner(),
    flight: { machs: [0.5, 0.8, 1.05, 1.2, 1.5, 2, 2.5, 3, 4], alphas: [0, 2, 4, 6, 8, 10, 12, 16, 20], altKm: 0 },
  },
  {
    id: "cruciform",
    name: "Cruciform missile",
    blurb: "Generic canard + tail cruciform missile.",
    config: cruciformMissile(),
    flight: DEFAULT_FLIGHT,
  },
  {
    id: "canard",
    name: "Canard missile",
    blurb: "Generic canard-only missile.",
    config: canardMissile(),
    flight: DEFAULT_FLIGHT,
  },
  {
    id: "flare",
    name: "Cone-cylinder-flare",
    blurb: "Hypersonic body alone with a conical flare.",
    config: hypersonicConeCylinderFlare(),
    flight: { machs: [2, 3, 4, 5, 6, 8, 10], alphas: [0, 2, 5, 10, 15, 20, 25], altKm: 30 },
  },
  {
    id: "sears-haack",
    name: "Sears–Haack",
    blurb: "Minimum-wave-drag body. Area rule vs the closed-form (9π/2)(Amax/L)².",
    config: searsHaackMissile(),
    flight: { machs: [1.2, 1.5, 2, 3, 4], alphas: [0, 2, 4, 6], altKm: 15 },
  },
  {
    id: "aircraft",
    name: "Wing-body-tail",
    blurb: "Generic aircraft WBT configuration (Missile DATCOM-style methods).",
    config: aircraftWBT(),
    flight: { machs: [0.3, 0.5, 0.7, 0.85, 0.95, 1.2, 1.5], alphas: [-4, -2, 0, 2, 4, 6, 8, 10, 12], altKm: 10 },
  },
];

export function cloneConfig(c: VehicleConfig): VehicleConfig {
  return JSON.parse(JSON.stringify(c)) as VehicleConfig;
}

/**
 * Build a DATCOM-style configuration from the current waverider DesignParams.
 * Approximate: body = elliptical-cross-section treated as a body of revolution of equal
 * planform / volume is NOT attempted; instead a cone-cylinder of diameter = height and
 * length = length is used, plus two elevon-like panels and optional fins.
 * See NOTES.md for the limitations of this adapter.
 */
export function fromWaveriderDesign(p: {
  length: number;
  span: number;
  height: number;
  coneDeg?: number;
  elevonDeg?: number;
  finHeight?: number;
  fins?: number;
  cgFrac?: number;
  name?: string;
  family?: string;
  leRadius?: number;
  planform?: string;
  planformPower?: number;
}): VehicleConfig {
  const L = p.length;
  const D = Math.max(p.height, 0.05);
  const cone = (p.coneDeg ?? 8) * DEG;
  const noseLen = Math.min(L * 0.35, D / 2 / Math.tan(Math.max(cone, 1 * DEG)));
  const sets: FinSet[] = [];
  {
    sets.push({
      name: "elevons",
      role: "htail",
      nPanels: 2,
      phiF: [90, 270],
      rootChord: 0.15 * L,
      tipChord: 0.08 * L,
      semispan: Math.max(p.span / 2, D / 2 + 0.05),
      xLE: 0.85 * L,
      sweepLE: 20,
      thickness: 0.04,
      section: "hex",
      cfoc: 1,
      deflection: [-(p.elevonDeg ?? 0), p.elevonDeg ?? 0],
    });
  }
  if ((p.finHeight ?? 0) > 0 && (p.fins ?? 0) > 0) {
    const n = Math.min(Math.max(p.fins ?? 2, 1), 8);
    sets.push({
      name: "fins",
      role: "fin",
      nPanels: n,
      rootChord: 0.12 * L,
      tipChord: 0.06 * L,
      semispan: D / 2 + (p.finHeight ?? 0),
      xLE: 0.78 * L,
      sweepLE: 30,
      thickness: 0.04,
      section: "hex",
      cfoc: 0,
    });
  }
  if (p.family === "liftbody") {
    const oml = liftOml({
      length: L,
      span: p.span,
      height: D,
      leRadius: p.leRadius ?? Math.max(0.008 * L, 0.02),
      planform: p.planform ?? "power",
      planformPower: p.planformPower ?? 0.75,
    });
    const xs = liftXs(oml, 72);
    const elliptic = [];
    for (const x of xs) {
      if (x <= 1e-8) continue;
      const b = 0.5 * (oml.upper(x) + oml.lower(x));
      elliptic.push({ x, a: Math.max(oml.a(x), 1e-4), b: Math.max(b, 1e-4) });
    }
    return {
      name: p.name ?? "lifting body",
      kind: "missile",
      body: { elliptic, exitDiameter: 0 },
      sets,
      ref: { xcg: (p.cgFrac ?? 0.62) * L, sRef: Math.max(p.span * L * 0.45, 1e-4), lRef: L },
      bodyMethod: "newtonian",
      notes: [
        `Lifting-body ELLBOD uses the CAD loft (${elliptic.length} stations). Nose curvature R = ${(oml.rn * 1000).toFixed(1)} mm, osculating parabola, slope monotone — not a spherical cap.`,
        "Sref is the planform, Lref is the length. Hypersonic body loads use modified Newtonian. Not an HL-20 wind-tunnel deck.",
      ],
    };
  }
  return {
    name: p.name ?? "from waverider design",
    kind: "missile",
    body: {
      parametric: {
        noseKind: "cone",
        noseLength: noseLen,
        noseDiameter: D,
        centerLength: Math.max(L - noseLen, 0),
        aftLength: 0,
      },
      exitDiameter: 0,
    },
    sets,
    ref: { xcg: (p.cgFrac ?? 0.58) * L, sRef: Math.PI * (D / 2) ** 2, lRef: D },
    notes: [
      "Adapter: waverider DesignParams -> cone-cylinder of diameter = height; elevons + optional fins.",
      "Does NOT capture the caret / osculating / elliptic generating-flow planform — for qualitative comparison only.",
    ],
  };
}

export function attachBodyRoot(cfg: VehicleConfig): VehicleConfig {
  // Ensure SSPAN(1)=0 style: leave semispan as absolute from the axis.
  if (!cfg.body) return cfg;
  const prof = buildBody(cfg.body);
  for (const s of cfg.sets) {
    const xMid = s.xLE + 0.5 * s.rootChord;
    const r = radiusAt(prof, xMid);
    if (s.semispan < r) s.semispan = r + 1e-3;
  }
  return cfg;
}
