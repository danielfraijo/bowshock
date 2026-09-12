export type WaveriderFamily =
  | "caret"
  | "cone"
  | "osculating"
  | "viscopt"
  | "inward"
  | "elliptic"
  | "busemann"
  | "star"
  | "wedgecone"
  | "liftbody"
  | "ramjet"
  | "scramjet"
  | "integrated";

export type FlowDomain = "external" | "internal";

export type LengthUnit = "m" | "mm" | "in";

export type PlanformKind = "delta" | "power" | "spatular" | "rect" | "double";

export type LidFace = "top" | "bottom";

export type AeroMethod = "newtonian" | "tangent" | "mixed";

export type FuelKind = "H2" | "JP";

export type ColorMode = "surface" | "cp" | "heat";

export type CowlSide = "belly" | "dorsal";

export interface DesignParams {
  family: WaveriderFamily;
  mach: number;
  gamma: number;
  length: number;
  span: number;
  height: number;
  shockDeg: number;
  coneDeg: number;
  superN: number;
  planform: PlanformKind;
  planformPower: number;
  fins: number;
  wedgeFrac: number;
  captureFrac: number;
  nx: number;
  ny: number;
  leRadius: number;
  leBlunt: boolean;
  halfModel: boolean;
  unit: LengthUnit;
  name: string;
  lid: LidFace;
  dihedralDeg: number;
  camber: number;
  teSweepDeg: number;
  elevonDeg: number;
  finHeight: number;
  nRamps: number;
  cowlSide: CowlSide;
  flightMach: number;
  lockFlight: boolean;
  alphaDeg: number;
  betaDeg: number;
  gammaDeg: number;
  altKm: number;
  cgFrac: number;
  aeroMethod: AeroMethod;
  twK: number;
  inletHeight: number;
  cowlFrac: number;
  combustorFrac: number;
  nozzleER: number;
  rampDeg: number;
  fuel: FuelKind;
  phi: number;
  flowThrough: boolean;
  rhoKgM3: number;
  massKg: number;
}

export const DEFAULT_PARAMS: DesignParams = {
  family: "caret",
  mach: 8,
  gamma: 1.4,
  length: 4,
  span: 2.2,
  height: 0.55,
  shockDeg: 16,
  coneDeg: 8,
  superN: 2.2,
  planform: "delta",
  planformPower: 1.15,
  fins: 4,
  wedgeFrac: 0.42,
  captureFrac: 0.55,
  nx: 40,
  ny: 28,
  leRadius: 0.02,
  leBlunt: true,
  halfModel: false,
  unit: "m",
  name: "waverider",
  lid: "top",
  dihedralDeg: 0,
  camber: 0,
  teSweepDeg: 0,
  elevonDeg: 0,
  finHeight: 0,
  nRamps: 2,
  cowlSide: "belly",
  flightMach: 8,
  lockFlight: true,
  alphaDeg: 0,
  betaDeg: 0,
  gammaDeg: 0,
  altKm: 30,
  cgFrac: 0.58,
  aeroMethod: "mixed",
  twK: 800,
  inletHeight: 0.14,
  cowlFrac: 0.38,
  combustorFrac: 0.22,
  nozzleER: 4,
  rampDeg: 10,
  fuel: "H2",
  phi: 0.9,
  flowThrough: false,
  rhoKgM3: 160,
  massKg: 0,
};

export type SurfaceKind = "upper" | "lower" | "base" | "leading" | "symmetry" | "inlet" | "nozzle" | "cowl";

export const SURFACE_ID: Record<SurfaceKind, number> = {
  upper: 0,
  lower: 1,
  base: 2,
  leading: 3,
  symmetry: 4,
  inlet: 5,
  nozzle: 6,
  cowl: 7,
};

export type Vec3 = [number, number, number];

export interface SurfaceGrid {
  name: string;
  ni: number;
  nj: number;
  /** row-major, j (span) fastest: index = (i * nj + j) * 3 */
  xyz: Float64Array;
}

export interface TriMesh {
  positions: Float64Array;
  indices: Uint32Array;
  surfaces: Uint8Array;
}

export interface MeshQuality {
  vertices: number;
  triangles: number;
  volume: number;
  area: number;
  watertight: boolean;
  manifold: boolean;
  openEdges: number;
  nonManifoldEdges: number;
  skippedDegenerate: number;
  minEdge: number;
  maxEdge: number;
  bbox: { min: Vec3; max: Vec3 };
}

export interface AeroEstimate {
  thetaDeg: number;
  betaDeg: number;
  muDeg: number;
  coneDeg: number;
  pressureRatio: number;
  temperatureRatio: number;
  densityRatio: number;
  m2: number;
  planformArea: number;
  wettedArea: number;
  volume: number;
  cl: number;
  cd: number;
  ld: number;
  volumetricEfficiency: number;
  attached: boolean;
  notes: string[];
}

export interface BuiltVehicle {
  params: DesignParams;
  mesh: TriMesh;
  grids: SurfaceGrid[];
  shockGrids: SurfaceGrid[];
  quality: MeshQuality;
  aero: AeroEstimate;
}

export const FAMILY_META: Record<
  WaveriderFamily,
  { label: string; blurb: string; generating: string; domain: FlowDomain }
> = {
  caret: {
    label: "Caret",
    blurb: "Nonweiler caret. Two planar shocks — inviscid L/D = cot θ.",
    generating: "2-D wedge / planar oblique shock",
    domain: "external",
  },
  cone: {
    label: "Cone-derived",
    blurb: "Jones–Moore–Pike. Conical shock from Taylor–Maccoll RK4.",
    generating: "Axisymmetric cone (Taylor–Maccoll)",
    domain: "external",
  },
  osculating: {
    label: "Osculating cone",
    blurb: "Sobieczky. Arbitrary planform from local cones.",
    generating: "Strip / osculating conical flow",
    domain: "external",
  },
  viscopt: {
    label: "Viscous-opt",
    blurb: "Bowcutt / Corda power-law. Volume vs viscous L/D.",
    generating: "Power-law ICC + streamwise exponent",
    domain: "external",
  },
  elliptic: {
    label: "Elliptic cone",
    blurb: "Rasmussen elliptic-cone waverider.",
    generating: "Elliptic conical shock",
    domain: "external",
  },
  wedgecone: {
    label: "Wedge–cone",
    blurb: "2-D wedge centerbody with conical outboard wings.",
    generating: "Wedge + cone hybrid",
    domain: "external",
  },
  star: {
    label: "Star body",
    blurb: "n-fold caret. Missile / entry, intersecting shocks.",
    generating: "n planar wedges around the axis",
    domain: "external",
  },
  liftbody: {
    label: "Lifting body",
    blurb: "Elliptic HL-20 / HTV-class. Entry heating study.",
    generating: "Elliptic loft / power-law planform",
    domain: "external",
  },
  ramjet: {
    label: "Ramjet",
    blurb: "2-D ramjet duct: ramps, isolator NS, Rayleigh heat, nozzle.",
    generating: "Shock-on-lip + Rankine–Hugoniot + Rayleigh",
    domain: "internal",
  },
  scramjet: {
    label: "Scramjet",
    blurb: "Supersonic combustor. Multi-ramp, Fanno isolator, no terminal NS.",
    generating: "Oblique train + Rayleigh (M>1)",
    domain: "internal",
  },
  busemann: {
    label: "Busemann inlet",
    blurb: "Inverse Busemann scoop. High contraction, Kantrowitz start.",
    generating: "Axisymmetric inward Busemann",
    domain: "internal",
  },
  inward: {
    label: "REST scoop",
    blurb: "Inward-turning inlet. Sidewalls + deep keel.",
    generating: "Concave ICC / inward turning",
    domain: "internal",
  },
  integrated: {
    label: "WR + ramjet",
    blurb: "Osculating waverider with belly or dorsal cowl — both domains.",
    generating: "OSC vehicle + fused cowl pod",
    domain: "internal",
  },
};

export const EXTERNAL_FAMILIES: WaveriderFamily[] = [
  "caret",
  "cone",
  "osculating",
  "viscopt",
  "elliptic",
  "wedgecone",
  "star",
  "liftbody",
];

export const INTERNAL_FAMILIES: WaveriderFamily[] = ["ramjet", "scramjet", "busemann", "inward", "integrated"];

export function domainOf(family: WaveriderFamily): FlowDomain {
  return FAMILY_META[family]?.domain ?? "external";
}

export const FAMILY_GROUPS: { id: FlowDomain; label: string; families: WaveriderFamily[] }[] = [
  { id: "external", label: "External", families: EXTERNAL_FAMILIES },
  { id: "internal", label: "Internal", families: INTERNAL_FAMILIES },
];
