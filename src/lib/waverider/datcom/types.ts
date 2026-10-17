/**
 * Configuration + result model for the Bowshock DATCOM-style estimator.
 *
 * Conventions (internal, SI):
 *  - lengths in metres, angles in degrees at the API surface (radians inside),
 *  - x measured from the nose tip, positive AFT; r radial; z up.
 *  - alpha > 0 nose up; beta > 0 = air arriving from the starboard side
 *    (looking forward from behind the vehicle the right-hand side);
 *    positive roll/yaw/pitch moments: right-wing-down (clockwise looking
 *    forward), nose-right, nose-up.
 *  - fin panel roll position phiF: degrees, measured clockwise from top
 *    vertical centre looking FORWARD (Missile DATCOM Fig. 8 convention);
 *    phiF = 90 is the starboard (right) panel.
 *  - panel deflection delta: Missile DATCOM sign: positive delta gives a
 *    NEGATIVE (counter-clockwise) body-axis roll moment at alpha = beta = 0;
 *    for a panel at phiF = 90 deg a positive delta gives an UP force
 *    (trailing edge down).
 */

export type NoseKind = "cone" | "ogive" | "karman" | "haack" | "power" | "spherical";
export type AftKind = "cone" | "ogive";
export type SectionKind = "hex" | "biconvex" | "wedge" | "flat" | "naca4";
export type SurfaceRole = "wing" | "tail" | "canard" | "fin" | "htail" | "vtail";
export type VehicleKind = "missile" | "aircraft";
export type BodyMethod = "auto" | "slender" | "newtonian";

export interface BodyStation {
  /** station from the nose tip [m], strictly non-decreasing */
  x: number;
  /** body radius [m] */
  r: number;
}

/** Parametric (Missile DATCOM AXIBOD option-1 style) body definition. */
export interface BodyParametric {
  noseKind: NoseKind;
  noseLength: number;
  /** nose base diameter */
  noseDiameter: number;
  /** exponent n for noseKind='power' (r/R = (x/L)^n) */
  power?: number;
  /** spherical bluntness radius at the nose tip (0 = sharp) */
  noseBluntRadius?: number;
  centerLength: number;
  /** centerbody diameter (default noseDiameter) */
  centerDiameter?: number;
  aftKind?: AftKind;
  aftLength: number;
  /** diameter at the base of the aft body (boattail <0 / flare >0 relative to centre) */
  aftDiameter?: number;
}

/** Elliptical cross-section station (Missile DATCOM ELLBOD option-2 style). a = half-width, b = half-height. */
export interface EllipticStation {
  x: number;
  a: number;
  b: number;
}

export interface BodySpec {
  /** Either a station/radius table (option 2) or the parametric form (option 1). */
  stations?: BodyStation[];
  parametric?: BodyParametric;
  /** Elliptical body stations (ELLBOD). When set, overrides circular stations for area/perimeter/crossflow. */
  elliptic?: EllipticStation[];
  /** Spherical bluntness radius applied to a station-table nose (m). */
  noseBluntRadius?: number;
  /**
   * Base drag handling (Missile DATCOM DEXIT semantics):
   *  undefined -> no base drag, 0 -> full base area, d>0 -> annulus around an exit of diameter d.
   */
  exitDiameter?: number;
  /** include base drag at all (default true when exitDiameter defined) */
  baseDrag?: boolean;
}

export type InletKind = "axi" | "2dside" | "2dtop";
/** Engineering-level inlet increment (capture / additive drag / normal force). */
export interface InletSpec {
  name?: string;
  kind: InletKind;
  /** axial station of the capture plane from the nose [m] */
  x: number;
  /** capture area [m^2] */
  captureArea: number;
  /** mass-flow ratio m_dot/(rho V A_c); default 0.9. Additive drag grows as (1-MFR). */
  mfr?: number;
  /** external compression ramp angle [deg] (0 = pitot) */
  rampDeg?: number;
  /** roll position of the inlet [deg], Missile DATCOM convention */
  phi?: number;
  /** covered (plugged) inlet — full additive + frontal drag */
  covered?: boolean;
}

export type ProtuberanceKind = "cylinder" | "wedge" | "hemisphere" | "block" | "fairing";
/** Simple protuberance drag / normal-force increment. */
export interface ProtuberanceSpec {
  name?: string;
  kind: ProtuberanceKind;
  x: number;
  length: number;
  width: number;
  height: number;
  /** identical copies around the body */
  count?: number;
  /** roll of the first copy [deg] */
  phi0?: number;
}

export interface FinSet {
  name: string;
  role: SurfaceRole;
  /** number of identical panels around the body (1..8) */
  nPanels: number;
  /** panel roll positions [deg], clockwise from top looking forward. default: even spacing starting at 45 deg for 4 panels ('x'), else see presets */
  phiF?: number[];
  /** exposed root chord (at the body surface) [m] */
  rootChord: number;
  tipChord: number;
  /** tip radial station: semispan measured from the BODY AXIS [m] */
  semispan: number;
  /** root leading-edge station from nose [m] */
  xLE: number;
  /** leading-edge sweep [deg] */
  sweepLE: number;
  /** section thickness-to-chord ratio */
  thickness: number;
  section: SectionKind;
  /** hex section: fraction of chord of constant thickness (default 0) */
  flatFraction?: number;
  /** NACA 4-digit designation for section='naca4', e.g. "0012" */
  naca?: string;
  /** leading-edge radius [m] (hypersonic LE drag), default 0 */
  leRadius?: number;
  /** flap chord / fin chord (1 = all-moving) */
  cfoc?: number;
  /** panel deflections [deg], one per panel (Missile DATCOM sign) */
  deflection?: number[];
  /** panel dihedral [deg] (not modelled in the loads, warned) */
  dihedral?: number;
}

export interface RefQuantities {
  /** reference area [m^2]; default: max body cross-section, else max fin panel area */
  sRef?: number;
  /** longitudinal reference length [m]; default: max body diameter, else fin MAC */
  lRef?: number;
  /** lateral reference length [m]; default lRef */
  latRef?: number;
  /** moment reference center from nose [m] */
  xcg: number;
  zcg?: number;
  /** equivalent sand roughness height [m] (0 = smooth) */
  roughness?: number;
  /** 'turb' = fully turbulent (default); 'natural' = transition at Re_x,tr */
  blayer?: "turb" | "natural";
  /** transition Reynolds number for blayer='natural' (default 5e5) */
  reTransition?: number;
  /** wall temperature ratio Tw/Taw for the friction model (default 0.8 hot-body proxy; 1 = adiabatic) */
  wallTempRatio?: number;
}

export interface VehicleConfig {
  name: string;
  kind: VehicleKind;
  body?: BodySpec;
  sets: FinSet[];
  ref: RefQuantities;
  /** body aero method selection; 'auto' = slender+crossflow blended to Newtonian above M~3-8 */
  bodyMethod?: BodyMethod;
  /** vortex decay length in fin semispans for the downwash model (default 30, 0 or Infinity = none) */
  vortexDecaySemispans?: number;
  /** optional inlets (engineering additive-drag / CN increments) */
  inlets?: InletSpec[];
  /** optional protuberances (drag / CN increments) */
  protuberances?: ProtuberanceSpec[];
  /** free-form notes carried into outputs */
  notes?: string[];
  /** length units of the ORIGINAL input (deck DIM card); internal data is always metres */
  units?: "m" | "ft" | "in" | "cm";
  /** derivative output units in text tables (deck DERIV card) */
  derivUnits?: "deg" | "rad";
  caseId?: string;
}

export interface FlightSpec {
  machs: number[];
  /** angles of attack [deg] (total angle of attack when phi != 0) */
  alphas: number[];
  /** sideslip [deg] (ignored when phi != 0, as in Missile DATCOM) */
  beta?: number;
  /** aerodynamic roll angle [deg]: alphas are total angle of attack */
  phi?: number;
  /** altitude per Mach [km]; scalar repeated. default 0 */
  altKm?: number | number[];
  /** Reynolds number per metre overrides altitude (per Mach, or scalar) */
  renPerM?: number | number[];
  gamma?: number;
}

export type Confidence = "high" | "medium" | "low";

export interface Validity {
  confidence: Confidence;
  /** false when the point is outside the stated range of the dominant method */
  valid: boolean;
  /** short tag like "supersonic-linear / tangent-cone" */
  method: string;
  /** honest notes, e.g. 'Mach 0.9-1.2 transonic interpolation — low confidence' */
  notes: string[];
}

export interface Coeffs {
  CN: number;
  CA: number;
  Cm: number;
  CY: number;
  /** yawing moment (body axes) */
  Cn: number;
  /** rolling moment (body axes) */
  Cl: number;
  CL: number;
  CD: number;
  LD: number;
  /** centre of pressure from nose [m] (Cm/CN based) */
  xcp: number;
}

export interface Derivs {
  CNa: number;
  Cma: number;
  CYb: number;
  Cnb: number;
  Clb: number;
  CLa: number;
  /** dynamic (per rad of p*b/2V, q*L/2V, r*b/2V) */
  Clp: number;
  Cmq: number;
  Cnr: number;
  CNq: number;
  /** control derivatives, per RAD of the control mixer, for the first set that has cfoc>0 */
  CNd: number;
  Cmd: number;
  Cld: number;
  /** static margin in calibers (reference lengths), positive = stable */
  staticMargin: number;
}

export interface Component {
  name: string;
  CN: number;
  CA: number;
  Cm: number;
  CY: number;
  Cn: number;
  Cl: number;
}

export interface PointResult extends Coeffs {
  alpha: number;
  beta: number;
  phi: number;
  /** total angle of attack used by the aerodynamic model */
  alphaTotal: number;
  derivs: Derivs;
  components: Component[];
  validity: Validity;
}

export interface CaseResult {
  mach: number;
  altKm: number;
  reynoldsPerM: number;
  reynoldsRefLength: number;
  qPa: number;
  tK: number;
  vMs: number;
  points: PointResult[];
}

export interface GeometrySummary {
  length: number;
  maxDiameter: number;
  sRef: number;
  lRef: number;
  latRef: number;
  xcg: number;
  baseArea: number;
  planformArea: number;
  volume: number;
  wettedArea: number;
  noseFineness: number;
  bodyFineness: number;
  sets: {
    name: string;
    role: SurfaceRole;
    nPanels: number;
    exposedArea: number;
    exposedSpan: number;
    aspectRatio: number;
    taper: number;
    sweepLE: number;
    sweepQC: number;
    sweepHC: number;
    mac: number;
    xMacLE: number;
    xAC: number;
    bodyRadiusAtRoot: number;
    tauRoverS: number;
  }[];
  warnings: string[];
}

export interface RunResult {
  config: VehicleConfig;
  flight: FlightSpec;
  geometry: GeometrySummary;
  cases: CaseResult[];
  warnings: string[];
}
