/**
 * Bowshock data package.
 *
 * Pure, UI-independent: takes the design parameters (the Geom-tab config) and returns a set of
 * tables — flight conditions, surface panels, streamwise / spanwise stations, α / Mach sweeps,
 * a Mach × α carpet, heating peaks, shock data, stability derivatives, validation and provenance —
 * that can be emitted as CSV, JSON, and a README data dictionary.
 *
 * Live recompute: `datapackKey(params, opts)` hashes every input that can change a number
 * (all of DesignParams except the cosmetic `name` / `unit`, plus the sweep options). `computeDataPack`
 * is memoized on that key, so the Data tab recomputes exactly when something real changed and
 * never when it did not. `computeDataPackAsync` runs the same code in slices, yielding to the
 * event loop between sweep points, so the UI thread is never blocked for more than one
 * panelAero call (~10–30 ms) at a time.
 *
 * Conventions (also printed in the README): x streamwise (flow +x, nose at origin), +y starboard,
 * z up. Units SI unless the column name says otherwise. Moment coefficients about the CG at
 * cgFrac·L, nose-up positive, referenced to planform area and body length.
 */

import { atmosphere, type Atmosphere } from "./atmosphere";
import { effectiveLeRadius } from "./blunt";
import { buildVehicle } from "./generate";
import {
  DEG,
  RAD,
  betaFromThetaM,
  billigStandoff,
  billigStandoffCylinder,
  normalShock,
  obliqueShock,
  newtonianCpMax,
  isentropic,
} from "./math";
import { panelAero, type PanelAero } from "./panel";
import { runValidation, validationSummary, type Check } from "./validate";
import { parametricSweep } from "./study";
import { budgetSummary, errorBudgetSteps } from "./errorbudget";
import { DEFAULT_PARAMS, SURFACE_ID, type BuiltVehicle, type DesignParams } from "./types";

export const DATAPACK_FORMAT = "bowshock-datapack/1";
export const BOWSHOCK_VERSION = "0.1.0";

/* ------------------------------------------------------------------ types */

export type Cell = number | string | boolean | null;

export interface Column {
  /** CSV header. Units are part of the name (…_m, …_W_cm2, …_deg). */
  key: string;
  unit: string;
  desc: string;
}

export interface DataTable {
  id: string;
  title: string;
  description: string;
  columns: Column[];
  rows: Cell[][];
}

export interface SweepSpec {
  min: number;
  max: number;
  n: number;
}

export interface DataPackOptions {
  /** angle-of-attack sweep, deg */
  alphaSweep?: SweepSpec;
  /** flight-Mach sweep (altitude held) */
  machSweep?: SweepSpec;
  /** Mach × α carpet grid: n values on each axis (0 disables) */
  carpet?: { alphaMin: number; alphaMax: number; na: number; machMin: number; machMax: number; nm: number };
  /** number of streamwise / spanwise stations */
  stations?: number;
  /** optional one-variable parametric sweep (rebuilds the vehicle per value) */
  parametric?: { variable: keyof DesignParams; min: number; max: number; n: number } | null;
  /** include the per-triangle surface table (can be thousands of rows) */
  panels?: boolean;
  /** provenance only — never part of the key */
  now?: string;
  gitHash?: string;
  version?: string;
}

export interface Provenance {
  format: string;
  bowshockVersion: string;
  gitHash: string;
  inputHash: string;
  timestamp: string;
  family: string;
  aeroMethod: string;
  models: string[];
  options: Required<Pick<DataPackOptions, "stations" | "panels">> & {
    parametric: DataPackOptions["parametric"] | null;
    alphaSweep: SweepSpec;
    machSweep: SweepSpec;
    carpet: NonNullable<DataPackOptions["carpet"]>;
  };
}

export interface DataPack {
  key: string;
  provenance: Provenance;
  tables: DataTable[];
  validation: { n: number; pass: number; fail: number; ok: boolean };
  /** wall-clock ms spent computing (not part of the deterministic content) */
  elapsedMs: number;
}

/* ------------------------------------------------------------------ key / hash */

export const DEFAULT_OPTIONS = {
  alphaSweep: { min: -4, max: 12, n: 9 } as SweepSpec,
  machSweep: { min: 4, max: 12, n: 9 } as SweepSpec,
  carpet: { alphaMin: -2, alphaMax: 10, na: 5, machMin: 4, machMax: 12, nm: 5 },
  stations: 20,
  panels: true,
  parametric: null as DataPackOptions["parametric"] | null,
};

type ResolvedOptions = typeof DEFAULT_OPTIONS;

function resolveOptions(o?: DataPackOptions): ResolvedOptions {
  return {
    alphaSweep: { ...DEFAULT_OPTIONS.alphaSweep, ...(o?.alphaSweep ?? {}) },
    machSweep: { ...DEFAULT_OPTIONS.machSweep, ...(o?.machSweep ?? {}) },
    carpet: { ...DEFAULT_OPTIONS.carpet, ...(o?.carpet ?? {}) },
    stations: o?.stations ?? DEFAULT_OPTIONS.stations,
    panels: o?.panels ?? DEFAULT_OPTIONS.panels,
    parametric: o?.parametric ?? null,
  };
}

/** cyrb53: 53-bit string hash. Not cryptographic; collision odds are irrelevant for a cache key. */
function cyrb53(str: string, seed = 0): number {
  let h1 = 0xdeadbeef ^ seed;
  let h2 = 0x41c6ce57 ^ seed;
  for (let i = 0; i < str.length; i++) {
    const ch = str.charCodeAt(i);
    h1 = Math.imul(h1 ^ ch, 2654435761);
    h2 = Math.imul(h2 ^ ch, 1597334677);
  }
  h1 = Math.imul(h1 ^ (h1 >>> 16), 2246822507) ^ Math.imul(h2 ^ (h2 >>> 13), 3266489909);
  h2 = Math.imul(h2 ^ (h2 >>> 16), 2246822507) ^ Math.imul(h1 ^ (h1 >>> 13), 3266489909);
  return 4294967296 * (2097151 & h2) + (h1 >>> 0);
}

/** Params that never change a number in any table. */
const COSMETIC = new Set(["name", "unit"]);

export function normalizeParams(params: Partial<DesignParams>): DesignParams {
  return { ...DEFAULT_PARAMS, ...params };
}

function canonical(value: unknown): string {
  if (value === null || typeof value !== "object") return JSON.stringify(value);
  if (Array.isArray(value)) return `[${value.map(canonical).join(",")}]`;
  const o = value as Record<string, unknown>;
  return `{${Object.keys(o)
    .sort()
    .map((k) => `${JSON.stringify(k)}:${canonical(o[k])}`)
    .join(",")}}`;
}

/** Hash of every input that can change a number in the pack. */
export function datapackKey(params: Partial<DesignParams>, opts?: DataPackOptions): string {
  const p = normalizeParams(params);
  const kept: Record<string, unknown> = {};
  for (const k of Object.keys(p).sort()) if (!COSMETIC.has(k)) kept[k] = (p as unknown as Record<string, unknown>)[k];
  const s = canonical({ v: DATAPACK_FORMAT, p: kept, o: resolveOptions(opts) });
  return `dp-${cyrb53(s).toString(16).padStart(14, "0")}${(cyrb53(s, 7919) & 0xffff).toString(16).padStart(4, "0")}`;
}

/* ------------------------------------------------------------------ helpers */

const col = (key: string, unit: string, desc: string): Column => ({ key, unit, desc });

function linspace(min: number, max: number, n: number): number[] {
  const m = Math.max(1, Math.round(n));
  if (m === 1) return [min];
  return Array.from({ length: m }, (_, i) => min + ((max - min) * i) / (m - 1));
}

const clamp = (x: number, a: number, b: number) => Math.min(b, Math.max(a, x));

function machOf(p: DesignParams) {
  return p.lockFlight ? p.mach : p.flightMach;
}

interface Tri {
  cx: number;
  cy: number;
  cz: number;
  area: number;
  nx: number;
  ny: number;
  nz: number;
  surf: number;
  rawSurf: number;
}

function triangles(built: BuiltVehicle): Tri[] {
  const { positions: pos, indices: ind, surfaces } = built.mesh;
  const nt = ind.length / 3;
  const out: Tri[] = [];
  for (let t = 0; t < nt; t++) {
    const a = ind[t * 3] * 3;
    const b = ind[t * 3 + 1] * 3;
    const c = ind[t * 3 + 2] * 3;
    const abx = pos[b] - pos[a];
    const aby = pos[b + 1] - pos[a + 1];
    const abz = pos[b + 2] - pos[a + 2];
    const acx = pos[c] - pos[a];
    const acy = pos[c + 1] - pos[a + 1];
    const acz = pos[c + 2] - pos[a + 2];
    const nx = aby * acz - abz * acy;
    const ny = abz * acx - abx * acz;
    const nz = abx * acy - aby * acx;
    const mag = Math.hypot(nx, ny, nz);
    if (mag < 1e-16) {
      out.push({ cx: 0, cy: 0, cz: 0, area: 0, nx: 0, ny: 0, nz: 0, surf: 0, rawSurf: 0 });
      continue;
    }
    let surf = surfaces[t] ?? 0;
    const raw = surf;
    // same re-tagging the panel loop applies: aft-facing facets are base.
    if (nx / mag > 0.72 && surf !== SURFACE_ID.leading && surf !== SURFACE_ID.inlet) surf = SURFACE_ID.base;
    out.push({
      cx: (pos[a] + pos[b] + pos[c]) / 3,
      cy: (pos[a + 1] + pos[b + 1] + pos[c + 1]) / 3,
      cz: (pos[a + 2] + pos[b + 2] + pos[c + 2]) / 3,
      area: 0.5 * mag,
      nx: nx / mag,
      ny: ny / mag,
      nz: nz / mag,
      surf,
      rawSurf: raw,
    });
  }
  return out;
}

const SURF_NAMES = Object.keys(SURFACE_ID) as (keyof typeof SURFACE_ID)[];
const surfName = (id: number) => SURF_NAMES.find((k) => SURFACE_ID[k] === id) ?? String(id);

function aeroPoint(built: BuiltVehicle, params: DesignParams, alphaDeg: number, M: number, betaDeg = params.betaDeg): { a: PanelAero; atm: Atmosphere } {
  const p: DesignParams = { ...params, alphaDeg, betaDeg, lockFlight: false, flightMach: M };
  const atm = atmosphere(p.altKm, M, p.gamma);
  const sRef = Math.max(built.aero.planformArea, 1e-6);
  return { a: panelAero(built.mesh, p, atm, alphaDeg, betaDeg, sRef, p.length), atm };
}

const COEF_COLS = (lead: Column[]): Column[] => [
  ...lead,
  col("CL", "-", "Lift coefficient (wind axes), referenced to planform area"),
  col("CD", "-", "Drag coefficient (wind axes) = wave + friction + base"),
  col("CD_wave", "-", "Pressure (wave) drag coefficient"),
  col("CD_friction", "-", "Skin-friction drag coefficient (van Driest II on the wetted area)"),
  col("CD_base", "-", "Base-pressure drag coefficient (Love, Cp = −1/M²)"),
  col("L_over_D", "-", "CL / CD"),
  col("CN_body", "-", "Body-axis normal-force coefficient (pressure only, +z up)"),
  col("CA_body", "-", "Body-axis axial-force coefficient (pressure only, +x aft)"),
  col("CY", "-", "Side-force coefficient (wind axes)"),
  col("Cm", "-", "Pitching-moment coefficient about the CG, nose-up positive"),
  col("x_cp_m", "m", "Centre of pressure from the nose, −Cm·c/CN"),
  col("q_stag_W_cm2", "W/cm^2", "Stagnation-point heat flux on the effective leading-edge radius"),
  col("q_max_W_cm2", "W/cm^2", "Peak heat flux over the surface"),
];

function coefRow(lead: Cell[], a: PanelAero): Cell[] {
  return [lead[0], ...lead.slice(1), a.cl, a.cd, a.cdWave, a.cdFric, a.cdBase, a.ld, a.cnA, a.ca, a.cy, a.cm, a.xCp, a.qStag, a.qMax];
}

/* ------------------------------------------------------------------ main generator */

type Progress = { frac: number; label: string };

function* packSteps(params: DesignParams, optsIn: DataPackOptions | undefined, key: string): Generator<Progress, DataPack> {
  const t0 = typeof performance !== "undefined" ? performance.now() : Date.now();
  const o = resolveOptions(optsIn);
  const p = params;
  const M = Math.max(1.05, machOf(p));
  const g = p.gamma;
  yield { frac: 0.02, label: "building geometry" };
  const built = buildVehicle(p);
  const atm = atmosphere(p.altKm, M, g);
  const sRef = Math.max(built.aero.planformArea, 1e-6);
  const lRef = p.length;
  yield { frac: 0.06, label: "design-point aerodynamics" };
  const aero = panelAero(built.mesh, p, atm, p.alphaDeg, p.betaDeg, sRef, lRef);
  const tris = triangles(built);
  const S = sRef;
  const qInf = atm.q || 0.5 * atm.rho * atm.V * atm.V;
  const Rn = effectiveLeRadius(p);
  const tables: DataTable[] = [];

  /* ---- design inputs */
  const paramRows: Cell[][] = (Object.keys(p) as (keyof DesignParams)[]).sort().map((k) => [k, String(p[k])]);
  tables.push({
    id: "design_inputs",
    title: "Design inputs",
    description: "Every DesignParams field that produced this pack (the Geom-tab configuration). Lengths in metres, angles in degrees.",
    columns: [col("parameter", "-", "DesignParams field name"), col("value", "-", "Value as entered")],
    rows: paramRows,
  });

  /* ---- flight conditions */
  const a = atm;
  const cpMaxN = newtonianCpMax(M, g);
  const ns = normalShock(M, g);
  const flightRows: Cell[][] = [
    ["flight_mach", M, "-", "Freestream Mach (design Mach when lockFlight)"],
    ["altitude_km", p.altKm, "km", "Geometric altitude, US Standard Atmosphere 1976"],
    ["alpha_deg", p.alphaDeg, "deg", "Angle of attack at the design point"],
    ["beta_deg", p.betaDeg, "deg", "Sideslip at the design point (β>0: flow toward +y)"],
    ["T_inf_K", a.T, "K", "Static temperature"],
    ["p_inf_Pa", a.p, "Pa", "Static pressure"],
    ["rho_inf_kg_m3", a.rho, "kg/m^3", "Density"],
    ["a_inf_m_s", a.a, "m/s", "Speed of sound"],
    ["V_inf_m_s", a.V, "m/s", "Freestream speed"],
    ["mu_inf_Pa_s", a.mu, "Pa s", "Dynamic viscosity (Sutherland)"],
    ["q_inf_Pa", qInf, "Pa", "Dynamic pressure ½ρV²"],
    ["Re_per_m", a.ReL, "1/m", "Unit Reynolds number ρV/μ"],
    ["Re_L", a.ReL * lRef, "-", "Reynolds number on body length"],
    ["T0_K", a.T * isentropic(M, g).TtT, "K", "Total temperature"],
    ["p0_pitot_Pa", a.p * ns.p2p1 * isentropic(ns.M2, g).ptP, "Pa", "Pitot (post-normal-shock stagnation) pressure"],
    ["Cp_max_newtonian", cpMaxN, "-", "Modified-Newtonian stagnation Cp"],
    ["gamma", g, "-", "Ratio of specific heats used by the shock/PM relations"],
    ["T_wall_K", p.twK, "K", "Specified wall temperature (cold-wall heating)"],
    ["S_ref_m2", S, "m^2", "Reference (planform) area"],
    ["L_ref_m", lRef, "m", "Reference length (body length)"],
    ["x_cg_m", aero.cg[0], "m", "Moment reference point (cgFrac·L from the nose)"],
    ["Rn_eff_m", Rn, "m", "Effective leading-edge radius after the Bowshock bluntness model"],
  ];
  tables.push({
    id: "flight_conditions",
    title: "Flight conditions",
    description: "Freestream state and reference quantities at the design point.",
    columns: [col("quantity", "-", "Name (units in the name)"), col("value", "see unit", "Value"), col("unit", "-", "Unit"), col("description", "-", "Meaning")],
    rows: flightRows,
  });

  /* ---- geometry summary */
  const ae = built.aero;
  const q = built.quality;
  tables.push({
    id: "geometry",
    title: "Geometry and generating flow",
    description: "Mesh quality and the generating-shock estimate from the waverider family (not the panel solution).",
    columns: [col("quantity", "-", "Name"), col("value", "see unit", "Value"), col("unit", "-", "Unit")],
    rows: [
      ["planform_area_m2", ae.planformArea, "m^2"],
      ["wetted_area_m2", ae.wettedArea, "m^2"],
      ["volume_m3", ae.volume, "m^3"],
      ["volumetric_efficiency", ae.volumetricEfficiency, "-"],
      ["generating_wedge_theta_deg", ae.thetaDeg, "deg"],
      ["generating_shock_beta_deg", ae.betaDeg, "deg"],
      ["mach_angle_deg", ae.muDeg, "deg"],
      ["generating_cone_deg", ae.coneDeg, "deg"],
      ["generating_p2_over_p1", ae.pressureRatio, "-"],
      ["generating_T2_over_T1", ae.temperatureRatio, "-"],
      ["generating_rho2_over_rho1", ae.densityRatio, "-"],
      ["generating_M2", ae.m2, "-"],
      ["inviscid_CL_estimate", ae.cl, "-"],
      ["inviscid_CD_estimate", ae.cd, "-"],
      ["inviscid_L_over_D_estimate", ae.ld, "-"],
      ["attached_shock", ae.attached, "-"],
      ["mesh_triangles", q.triangles, "-"],
      ["mesh_vertices", q.vertices, "-"],
      ["mesh_area_m2", q.area, "m^2"],
      ["mesh_volume_m3", q.volume, "m^3"],
      ["watertight", q.watertight, "-"],
      ["bbox_min_x_m", q.bbox.min[0], "m"],
      ["bbox_max_x_m", q.bbox.max[0], "m"],
      ["bbox_min_y_m", q.bbox.min[1], "m"],
      ["bbox_max_y_m", q.bbox.max[1], "m"],
      ["bbox_min_z_m", q.bbox.min[2], "m"],
      ["bbox_max_z_m", q.bbox.max[2], "m"],
    ],
  });

  /* ---- results summary at the design point */
  tables.push({
    id: "results_design_point",
    title: "Integrated results at the design point",
    description: "One-line aerodynamic and heating summary for the exact configuration (method = aeroMethod).",
    columns: [col("quantity", "-", "Name"), col("value", "see unit", "Value"), col("unit", "-", "Unit"), col("note", "-", "Meaning")],
    rows: [
      ["method", aero.method, "-", "Aerodynamic method used"],
      ["CL", aero.cl, "-", "Lift coefficient (wind axes)"],
      ["CD", aero.cd, "-", "Drag coefficient"],
      ["CD_wave", aero.cdWave, "-", "Wave drag"],
      ["CD_friction", aero.cdFric, "-", "Friction drag"],
      ["CD_base", aero.cdBase, "-", "Base drag"],
      ["L_over_D", aero.ld, "-", "CL/CD"],
      ["CN_body", aero.cnA, "-", "Body-axis normal force"],
      ["CA_body", aero.ca, "-", "Body-axis axial force"],
      ["CY", aero.cy, "-", "Side force"],
      ["Cm", aero.cm, "-", "Pitching moment about CG (nose-up +)"],
      ["Cn_yaw", aero.cn, "-", "Yawing moment (nose-right +)"],
      ["Cl_roll", aero.cll, "-", "Rolling moment"],
      ["x_cp_m", aero.xCp, "m", "Centre of pressure from the nose"],
      ["Cp_max", aero.cpMax, "-", "Stagnation Cp used"],
      ["q_stag_W_cm2", aero.qStag, "W/cm^2", "Stagnation heat flux"],
      ["q_fay_riddell_W_cm2", aero.qFay, "W/cm^2", "Fay–Riddell stagnation heating"],
      ["q_sutton_graves_W_cm2", aero.qSG, "W/cm^2", "Sutton–Graves stagnation heating"],
      ["q_radiative_W_cm2", aero.qRad, "W/cm^2", "Tauber–Sutton radiative heating"],
      ["q_max_W_cm2", aero.qMax, "W/cm^2", "Peak surface heat flux"],
      ["q_mean_windward_W_cm2", aero.qMeanWind, "W/cm^2", "Area-mean windward heat flux"],
      ["q_mean_lower_W_cm2", aero.qMeanLower, "W/cm^2", "Area-mean lower-surface heat flux"],
      ["q_mean_upper_W_cm2", aero.qMeanUpper, "W/cm^2", "Area-mean upper-surface heat flux"],
      ["T_wall_eq_max_K", aero.twMax, "K", "Radiation-equilibrium wall temperature at the peak flux (ε=0.8)"],
      ["Knudsen", aero.kn, "-", "Mean free path / L"],
      ["flow_regime", aero.regime, "-", "Continuum / slip / transitional / free-molecular"],
      ["viscous_chi_bar", aero.chiBar, "-", "Hayes–Probstein interaction parameter"],
      ["viscous_p_ratio", aero.pVisc, "-", "p/p_inviscid applied to windward panels"],
      ["gamma_equilibrium", aero.gammaEq, "-", "Post-shock effective γ (diagnostic)"],
      ["notes", aero.notes.join(" | "), "-", "Method notes"],
    ],
  });

  /* ---- surface panels */
  if (o.panels) {
    yield { frac: 0.12, label: "surface panel table" };
    const nt = tris.length;
    const rows: Cell[][] = [];
    for (let t = 0; t < nt; t++) {
      const tr = tris[t];
      rows.push([
        t,
        surfName(tr.surf),
        tr.cx,
        tr.cy,
        tr.cz,
        tr.area,
        tr.nx,
        tr.ny,
        tr.nz,
        aero.cp[t],
        aero.heat[t],
        aero.cf[t],
        aero.stanton[t],
        aero.machE[t],
        aero.twEq[t],
        aero.impact[t],
      ]);
    }
    tables.push({
      id: "surface_panels",
      title: "Surface panel table",
      description:
        "One row per mesh triangle at the design point: centroid, area, outward normal and local solution. Pressure force on a panel is −Cp·q∞·A·n.",
      columns: [
        col("tri", "-", "Triangle index in the mesh"),
        col("surface", "-", "Surface class (upper, lower, leading, base, ...)"),
        col("x_m", "m", "Centroid x (streamwise, nose at 0)"),
        col("y_m", "m", "Centroid y (+starboard)"),
        col("z_m", "m", "Centroid z (up)"),
        col("area_m2", "m^2", "Triangle area"),
        col("nx", "-", "Outward unit normal x"),
        col("ny", "-", "Outward unit normal y"),
        col("nz", "-", "Outward unit normal z"),
        col("Cp", "-", "Pressure coefficient (p − p∞)/q∞"),
        col("q_heat_W_cm2", "W/cm^2", "Cold-wall convective heat flux"),
        col("Cf", "-", "Local skin-friction coefficient (edge-based)"),
        col("St", "-", "Local Stanton number"),
        col("M_edge", "-", "Boundary-layer edge Mach number (0 where not computed)"),
        col("T_wall_eq_K", "K", "Radiation-equilibrium wall temperature at the local flux"),
        col("impact_cos", "-", "−n·v̂ (cosine of the impact angle; <0 is shadowed)"),
      ],
      rows,
    });
  }

  /* ---- stations */
  yield { frac: 0.2, label: "station tables" };
  const ns_ = Math.max(4, Math.round(o.stations));
  const xmin = q.bbox.min[0];
  const xmax = q.bbox.max[0];
  const ymax = Math.max(Math.abs(q.bbox.min[1]), Math.abs(q.bbox.max[1]), 1e-9);
  interface Bin {
    area: number;
    aLow: number;
    cpLow: number;
    aUp: number;
    cpUp: number;
    dFz: number;
    dFx: number;
    dFy: number;
    dM: number;
    qMax: number;
    cfSum: number;
    xmin: number;
    xmax: number;
    ymin: number;
    ymax: number;
    zmin: number;
    zmax: number;
  }
  const newBin = (): Bin => ({ area: 0, aLow: 0, cpLow: 0, aUp: 0, cpUp: 0, dFz: 0, dFx: 0, dFy: 0, dM: 0, qMax: 0, cfSum: 0, xmin: Infinity, xmax: -Infinity, ymin: Infinity, ymax: -Infinity, zmin: Infinity, zmax: -Infinity });
  const xb = Array.from({ length: ns_ }, newBin);
  const yb = Array.from({ length: ns_ }, newBin);
  const dx = Math.max(xmax - xmin, 1e-9) / ns_;
  const dy = ymax / ns_;
  const addTo = (b: Bin, tr: Tri, cp: number, heat: number, cf: number) => {
    b.area += tr.area;
    if (tr.rawSurf === SURFACE_ID.lower) {
      b.aLow += tr.area;
      b.cpLow += cp * tr.area;
    } else if (tr.rawSurf === SURFACE_ID.upper) {
      b.aUp += tr.area;
      b.cpUp += cp * tr.area;
    }
    // F = −Cp A n  (coefficient form, divided by S below)
    const f = -cp * tr.area;
    b.dFx += f * tr.nx;
    b.dFy += f * tr.ny;
    b.dFz += f * tr.nz;
    // moment about CG, nose-up positive: M_y = z·Fx − x·Fz with r = c − cg
    b.dM += (tr.cz - 0) * f * tr.nx - (tr.cx - aero.cg[0]) * f * tr.nz;
    b.qMax = Math.max(b.qMax, heat);
    b.cfSum += cf * tr.area;
    b.xmin = Math.min(b.xmin, tr.cx);
    b.xmax = Math.max(b.xmax, tr.cx);
    b.ymin = Math.min(b.ymin, Math.abs(tr.cy));
    b.ymax = Math.max(b.ymax, Math.abs(tr.cy));
    b.zmin = Math.min(b.zmin, tr.cz);
    b.zmax = Math.max(b.zmax, tr.cz);
  };
  for (let t = 0; t < tris.length; t++) {
    const tr = tris[t];
    if (tr.area <= 0) continue;
    const ix = clamp(Math.floor((tr.cx - xmin) / dx), 0, ns_ - 1);
    const iy = clamp(Math.floor(Math.abs(tr.cy) / dy), 0, ns_ - 1);
    addTo(xb[ix], tr, aero.cp[t], aero.heat[t], aero.cf[t]);
    addTo(yb[iy], tr, aero.cp[t], aero.heat[t], aero.cf[t]);
  }
  let cumN = 0;
  let cumA = 0;
  let cumM = 0;
  const xRows: Cell[][] = xb.map((b, i) => {
    cumN += b.dFz / S;
    cumA += b.dFx / S;
    cumM += b.dM / (S * lRef);
    return [
      i,
      xmin + (i + 0.5) * dx,
      (xmin + (i + 0.5) * dx - xmin) / Math.max(xmax - xmin, 1e-9),
      b.area,
      b.aLow > 0 ? b.cpLow / b.aLow : null,
      b.aUp > 0 ? b.cpUp / b.aUp : null,
      b.dFz / S,
      b.dFx / S,
      b.dM / (S * lRef),
      cumN,
      cumA,
      cumM,
      b.qMax,
      b.area > 0 ? b.cfSum / b.area : null,
      b.ymax === -Infinity ? null : b.ymax,
      b.zmax === -Infinity ? null : b.zmax - b.zmin,
    ];
  });
  tables.push({
    id: "stations_streamwise",
    title: "Streamwise stations",
    description: "Surface loads binned by x. dCN/dCA/dCm are the pressure-force contributions of each slab; cumulative columns integrate from the nose.",
    columns: [
      col("station", "-", "Bin index from the nose"),
      col("x_m", "m", "Bin centre"),
      col("x_over_L", "-", "x/L"),
      col("area_m2", "m^2", "Wetted triangle area in the slab"),
      col("Cp_lower_mean", "-", "Area-weighted Cp on lower-tagged triangles"),
      col("Cp_upper_mean", "-", "Area-weighted Cp on upper-tagged triangles"),
      col("dCN", "-", "Normal-force contribution (÷S_ref)"),
      col("dCA", "-", "Axial-force contribution (÷S_ref)"),
      col("dCm", "-", "Pitching-moment contribution about the CG (÷S_ref·L_ref)"),
      col("CN_cum", "-", "Cumulative CN from the nose"),
      col("CA_cum", "-", "Cumulative CA from the nose"),
      col("Cm_cum", "-", "Cumulative Cm from the nose"),
      col("q_max_W_cm2", "W/cm^2", "Peak heat flux in the slab"),
      col("Cf_mean", "-", "Area-weighted local Cf"),
      col("half_span_m", "m", "Largest |y| of any triangle centroid in the slab"),
      col("thickness_m", "m", "z-extent of triangle centroids in the slab"),
    ],
    rows: xRows,
  });
  const yRows: Cell[][] = yb.map((b, i) => [
    i,
    (i + 0.5) * dy,
    b.area,
    b.aLow > 0 ? b.cpLow / b.aLow : null,
    b.aUp > 0 ? b.cpUp / b.aUp : null,
    b.dFz / S,
    b.dFy / S,
    b.dFx / S,
    b.qMax,
    b.xmax === -Infinity ? null : b.xmax - b.xmin,
  ]);
  tables.push({
    id: "stations_spanwise",
    title: "Spanwise stations",
    description: "Surface loads binned by |y| (both halves folded together). Sums over all bins give the whole-vehicle force coefficients (pressure only).",
    columns: [
      col("station", "-", "Bin index from the centreline"),
      col("abs_y_m", "m", "Bin centre |y|"),
      col("area_m2", "m^2", "Triangle area in the bin"),
      col("Cp_lower_mean", "-", "Area-weighted Cp on lower-tagged triangles"),
      col("Cp_upper_mean", "-", "Area-weighted Cp on upper-tagged triangles"),
      col("dCN", "-", "Normal-force contribution (÷S_ref)"),
      col("dCY", "-", "Side-force contribution (÷S_ref, body axes)"),
      col("dCA", "-", "Axial-force contribution (÷S_ref)"),
      col("q_max_W_cm2", "W/cm^2", "Peak heat flux in the bin"),
      col("chord_extent_m", "m", "x-extent of triangle centroids in the bin"),
    ],
    rows: yRows,
  });

  /* ---- alpha sweep */
  const aSweep = linspace(o.alphaSweep.min, o.alphaSweep.max, o.alphaSweep.n);
  const aRes: PanelAero[] = [];
  for (let i = 0; i < aSweep.length; i++) {
    yield { frac: 0.25 + (0.2 * i) / aSweep.length, label: `α sweep ${aSweep[i].toFixed(1)}°` };
    aRes.push(aeroPoint(built, p, aSweep[i], M).a);
  }
  const clVec = aRes.map((r) => r.cl);
  const cmVec = aRes.map((r) => r.cm);
  const slope = (arr: number[], i: number) => {
    const i0 = Math.max(0, i - 1);
    const i1 = Math.min(aSweep.length - 1, i + 1);
    return i1 === i0 ? 0 : (arr[i1] - arr[i0]) / ((aSweep[i1] - aSweep[i0]) * DEG);
  };
  tables.push({
    id: "sweep_alpha",
    title: "Angle-of-attack sweep",
    description: `Integrated coefficients vs α at M=${M}, altitude ${p.altKm} km, method ${p.aeroMethod}. Derivatives are central differences along the sweep.`,
    columns: COEF_COLS([col("alpha_deg", "deg", "Angle of attack")]).concat([
      col("CL_alpha_per_rad", "1/rad", "dCL/dα (central difference along the sweep)"),
      col("Cm_alpha_per_rad", "1/rad", "dCm/dα (central difference along the sweep)"),
      col("static_margin_pct", "% L", "−Cm_α/CL_α ×100 (positive = CG ahead of the aerodynamic centre)"),
    ]),
    rows: aRes.map((r, i) => {
      const cla = slope(clVec, i);
      const cma = slope(cmVec, i);
      return [...coefRow([aSweep[i]], r), cla, cma, Math.abs(cla) > 1e-9 ? (-cma / cla) * 100 : null];
    }),
  });

  /* ---- Mach sweep */
  const mSweep = linspace(o.machSweep.min, o.machSweep.max, o.machSweep.n);
  const mRows: Cell[][] = [];
  for (let i = 0; i < mSweep.length; i++) {
    yield { frac: 0.45 + (0.2 * i) / mSweep.length, label: `Mach sweep ${mSweep[i].toFixed(1)}` };
    const m = mSweep[i];
    const d = 1;
    const mid = aeroPoint(built, p, p.alphaDeg, m);
    const up = aeroPoint(built, p, p.alphaDeg + d, m).a;
    const dn = aeroPoint(built, p, p.alphaDeg - d, m).a;
    const cla = (up.cl - dn.cl) / (2 * d * DEG);
    const cma = (up.cm - dn.cm) / (2 * d * DEG);
    mRows.push([
      ...coefRow([m], mid.a),
      mid.atm.V,
      mid.atm.q,
      mid.atm.ReL * lRef,
      cla,
      cma,
      Math.abs(cla) > 1e-9 ? (-cma / cla) * 100 : null,
    ]);
  }
  tables.push({
    id: "sweep_mach",
    title: "Mach sweep",
    description: `Integrated coefficients vs flight Mach at α=${p.alphaDeg}°, altitude ${p.altKm} km (the geometry is fixed; the atmosphere is evaluated at this altitude for every Mach).`,
    columns: COEF_COLS([col("mach", "-", "Flight Mach number")]).concat([
      col("V_m_s", "m/s", "Freestream speed at this Mach"),
      col("q_inf_Pa", "Pa", "Dynamic pressure"),
      col("Re_L", "-", "Reynolds number on body length"),
      col("CL_alpha_per_rad", "1/rad", "dCL/dα at the design α (±1°)"),
      col("Cm_alpha_per_rad", "1/rad", "dCm/dα at the design α (±1°)"),
      col("static_margin_pct", "% L", "−Cm_α/CL_α ×100"),
    ]),
    rows: mRows,
  });

  /* ---- parametric */
  if (o.parametric) {
    yield { frac: 0.62, label: `parametric ${String(o.parametric.variable)}` };
    const vals = linspace(o.parametric.min, o.parametric.max, o.parametric.n);
    tables.push(parametricSweep(p, o.parametric.variable, vals));
  }

  /* ---- carpet */
  const cs = o.carpet;
  if (cs.na > 0 && cs.nm > 0) {
    const cA = linspace(cs.alphaMin, cs.alphaMax, cs.na);
    const cM = linspace(cs.machMin, cs.machMax, cs.nm);
    const rows: Cell[][] = [];
    let k = 0;
    for (const m of cM) {
      for (const al of cA) {
        yield { frac: 0.65 + (0.2 * k) / (cA.length * cM.length), label: `carpet M${m.toFixed(1)} α${al.toFixed(1)}` };
        k++;
        const r = aeroPoint(built, p, al, m).a;
        rows.push([m, al, r.cl, r.cd, r.ld, r.cm, r.xCp, r.qStag]);
      }
    }
    tables.push({
      id: "sweep_mach_alpha",
      title: "Mach × α carpet",
      description: "Long-format grid for contour/carpet plots.",
      columns: [
        col("mach", "-", "Flight Mach number"),
        col("alpha_deg", "deg", "Angle of attack"),
        col("CL", "-", "Lift coefficient"),
        col("CD", "-", "Drag coefficient"),
        col("L_over_D", "-", "CL/CD"),
        col("Cm", "-", "Pitching moment about the CG"),
        col("x_cp_m", "m", "Centre of pressure"),
        col("q_stag_W_cm2", "W/cm^2", "Stagnation heat flux"),
      ],
      rows,
    });
  }

  /* ---- heating peaks */
  yield { frac: 0.86, label: "heating and shock tables" };
  const bySurf = new Map<number, { A: number; qSum: number; qMax: number; at: Tri | null; cpSum: number; tw: number }>();
  for (let t = 0; t < tris.length; t++) {
    const tr = tris[t];
    if (tr.area <= 0) continue;
    let s = bySurf.get(tr.surf);
    if (!s) {
      s = { A: 0, qSum: 0, qMax: -1, at: null, cpSum: 0, tw: 0 };
      bySurf.set(tr.surf, s);
    }
    s.A += tr.area;
    s.qSum += aero.heat[t] * tr.area;
    s.cpSum += aero.cp[t] * tr.area;
    if (aero.heat[t] > s.qMax) {
      s.qMax = aero.heat[t];
      s.at = tr;
    }
    s.tw = Math.max(s.tw, aero.twEq[t]);
  }
  const heatRows: Cell[][] = [...bySurf.entries()]
    .sort((x, y) => x[0] - y[0])
    .map(([id, s]) => [surfName(id), s.A, s.qSum / s.A, s.qMax, s.at?.cx ?? null, s.at?.cy ?? null, s.at?.cz ?? null, s.tw, s.cpSum / s.A]);
  tables.push({
    id: "heating_peaks",
    title: "Heating peaks by surface",
    description: "Cold-wall convective heating (Eckert/Zoby + Tauber + Lees blend, Fay–Riddell stagnation) summarised by surface class, with the location of each peak.",
    columns: [
      col("surface", "-", "Surface class"),
      col("area_m2", "m^2", "Area"),
      col("q_mean_W_cm2", "W/cm^2", "Area-weighted mean heat flux"),
      col("q_peak_W_cm2", "W/cm^2", "Peak heat flux"),
      col("x_peak_m", "m", "x of the peak triangle"),
      col("y_peak_m", "m", "y of the peak triangle"),
      col("z_peak_m", "m", "z of the peak triangle"),
      col("T_wall_eq_max_K", "K", "Radiation-equilibrium wall temperature at the peak (ε=0.8)"),
      col("Cp_mean", "-", "Area-weighted mean Cp"),
    ],
    rows: heatRows,
  });

  /* ---- shock / standoff */
  const thetaGen = ae.thetaDeg * DEG;
  const betaFlight = betaFromThetaM(M, thetaGen, g);
  const obFlight = Number.isFinite(betaFlight) ? obliqueShock(M, betaFlight, g) : null;
  const dBill = billigStandoff(M);
  const RnS = Math.max(Rn, 1e-6);
  const shockRows: Cell[][] = [
    ["normal_shock_p2_over_p1", ns.p2p1, "-", "Normal shock at flight Mach (stagnation streamline)"],
    ["normal_shock_T2_over_T1", ns.t2t1, "-", "Normal shock static temperature ratio"],
    ["normal_shock_rho2_over_rho1", ns.r2r1, "-", "Normal shock density ratio"],
    ["normal_shock_M2", ns.M2, "-", "Post-shock Mach"],
    ["normal_shock_pt2_over_pt1", ns.pt2pt1, "-", "Total-pressure recovery"],
    ["billig_standoff_sphere_over_Rn", dBill, "-", "Billig (1967) sphere Δ/Rn at flight Mach"],
    ["billig_standoff_cylinder_over_Rn", billigStandoffCylinder(M), "-", "Billig cylinder Δ/Rn (swept leading edge)"],
    ["Rn_eff_m", Rn, "m", "Effective nose / leading-edge radius"],
    ["standoff_sphere_m", dBill * RnS, "m", "Shock stand-off distance on a sphere of radius Rn_eff"],
    ["standoff_cylinder_m", billigStandoffCylinder(M) * RnS, "m", "Shock stand-off distance on a cylinder of radius Rn_eff"],
    ["generating_wedge_theta_deg", ae.thetaDeg, "deg", "Generating wedge half-angle of the family"],
    ["mach_angle_deg", Math.asin(1 / M) * RAD, "deg", "Mach angle at the flight Mach"],
    ["oblique_beta_at_flight_deg", Number.isFinite(betaFlight) ? betaFlight * RAD : null, "deg", "Weak-shock β for the generating wedge at the flight Mach (null = detached)"],
    ["oblique_p2_over_p1_at_flight", obFlight?.p2p1 ?? null, "-", "Static pressure ratio across that shock"],
    ["oblique_T2_over_T1_at_flight", obFlight?.t2t1 ?? null, "-", "Temperature ratio across that shock"],
    ["oblique_M2_at_flight", obFlight?.M2 ?? null, "-", "Mach number behind that shock"],
    ["design_shock_beta_deg", ae.betaDeg, "deg", "Generating shock angle at the design Mach"],
    ["design_mach", p.mach, "-", "Design Mach of the generating flow"],
    ["flight_mach", M, "-", "Flight Mach"],
    ["off_design_ratio", M / Math.max(p.mach, 1e-9), "-", "Flight / design Mach"],
    ["freestream_Kn", aero.kn, "-", "Knudsen number on body length"],
    ["viscous_chi_bar", aero.chiBar, "-", "Hayes–Probstein viscous-interaction parameter"],
    ["flow_regime", aero.regime, "-", "Regime from Kn"],
    ["gamma_equilibrium_post_shock", aero.gammaEq, "-", "Post-shock effective γ (vibrational excitation)"],
  ];
  tables.push({
    id: "shock_standoff",
    title: "Shock and stand-off data",
    description: "Closed-form shock relations at the flight condition for the generating geometry.",
    columns: [col("quantity", "-", "Name"), col("value", "see unit", "Value"), col("unit", "-", "Unit"), col("note", "-", "Meaning")],
    rows: shockRows,
  });

  /* ---- stability derivatives at the design point */
  yield { frac: 0.9, label: "stability derivatives" };
  const d = 1.5;
  const up = aeroPoint(built, p, p.alphaDeg + d, M).a;
  const dn = aeroPoint(built, p, p.alphaDeg - d, M).a;
  const bp = aeroPoint(built, p, p.alphaDeg, M, p.betaDeg + d).a;
  const cla = (up.cl - dn.cl) / (2 * d * DEG);
  const cma = (up.cm - dn.cm) / (2 * d * DEG);
  const cna = (up.cnA - dn.cnA) / (2 * d * DEG);
  const cnb = (bp.cn - aero.cn) / (d * DEG);
  const cyb = (bp.cy - aero.cy) / (d * DEG);
  const clb = (bp.cll - aero.cll) / (d * DEG);
  const sm = Math.abs(cla) > 1e-9 ? -cma / cla : 0;
  tables.push({
    id: "stability",
    title: "Stability derivatives",
    description:
      "Finite differences of the panel solution about the design point (α ±1.5°, β +1.5°). Sign convention: Cm nose-up positive; β > 0 is flow toward +y, so Cy_β < 0 and Cn_β > 0 for a directionally stable vehicle as defined here (Cn nose-right positive).",
    columns: [col("derivative", "-", "Name"), col("value", "see unit", "Value"), col("unit", "-", "Unit"), col("note", "-", "Meaning")],
    rows: [
      ["CL_alpha", cla, "1/rad", "Lift-curve slope"],
      ["CN_alpha", cna, "1/rad", "Body-axis normal-force slope"],
      ["Cm_alpha", cma, "1/rad", "Pitch stiffness (negative = stable)"],
      ["static_margin", sm, "L_ref", "−Cm_α/CL_α (positive = stable)"],
      ["static_margin_pct", sm * 100, "% L", "Static margin in percent of body length"],
      ["x_ac_m", aero.cg[0] + sm * lRef, "m", "Aerodynamic centre from the nose"],
      ["Cn_beta", cnb, "1/rad", "Directional stiffness (positive = stable)"],
      ["Cy_beta", cyb, "1/rad", "Side-force slope"],
      ["Cl_beta", clb, "1/rad", "Dihedral effect"],
      ["longitudinally_stable", cma < 0 && cla > 0, "-", "Cm_α < 0 and CL_α > 0"],
      ["directionally_stable", cnb > 0, "-", "Cn_β > 0"],
    ],
  });

  /* ---- per-method error budget (design-independent analytic bodies; computed once, then cached) */
  yield { frac: 0.9, label: "error budget" };
  const budgetIt = errorBudgetSteps();
  let budgetStep = budgetIt.next();
  while (!budgetStep.done) {
    yield { frac: 0.9 + 0.04 * budgetStep.value, label: "error budget" };
    budgetStep = budgetIt.next();
  }
  const budget = budgetStep.value;
  tables.push({
    id: "error_budget",
    title: "Aero-method error budget",
    description:
      "Error of each aero method against an exact/closed-form reference on analytic bodies (wedge, flat plate at α, double wedge/ramp, sharp cone, sphere) and on the default caret loft. Independent of the current design: it shows which method to trust where. err_pct = 100·(value − reference)/|reference|; tol_pct is the tolerance we hold the method to and rationale says why. The sphere reference is modified Newtonian (not truth).",
    columns: [
      col("shape", "-", "Body"),
      col("case", "-", "Case"),
      col("mach", "-", "Freestream Mach"),
      col("quantity", "-", "Quantity compared"),
      col("method", "-", "Aero method"),
      col("reference", "see quantity", "Exact / closed-form reference"),
      col("value", "see quantity", "Value from the method"),
      col("err_pct", "%", "100·(value − reference)/|reference|"),
      col("tol_pct", "%", "Tolerance for this method on this shape"),
      col("within_tol", "-", "TRUE if |err_pct| ≤ tol_pct"),
      col("rationale", "-", "Why this tolerance"),
    ],
    rows: budget.map((r) => [r.shape, r.case, r.M, r.quantity, r.method, r.reference, r.value, r.errPct, r.tolPct, r.within, r.rationale]),
  });
  tables.push({
    id: "error_budget_summary",
    title: "Aero-method error budget — summary",
    description: "Worst and mean |error| per shape and method from the error_budget table.",
    columns: [
      col("shape", "-", "Body"),
      col("method", "-", "Aero method"),
      col("n_cases", "-", "Number of comparisons"),
      col("max_abs_err_pct", "%", "Worst |error|"),
      col("mean_abs_err_pct", "%", "Mean |error|"),
      col("tol_pct", "%", "Largest tolerance applied"),
      col("all_within_tol", "-", "TRUE if every comparison is within tolerance"),
    ],
    rows: budgetSummary(budget).map((s) => [s.shape, s.method, s.n, s.maxAbsPct, s.meanAbsPct, s.tolPct, s.allWithin]),
  });

  /* ---- validation */
  yield { frac: 0.95, label: "validation" };
  const checks: Check[] = [...runValidation(), ...designChecks(built, p, aero, tris, M, qInf, S)];
  const vs = validationSummary(checks);
  tables.push({
    id: "validation",
    title: "Validation",
    description:
      "Code-level checks (closed forms and independent implementations) plus design-specific consistency checks. `reference` is what the check compares against; `basis` says how that reference was produced. tol is relative unless reference = 0, in which case it is the absolute bound on the error metric.",
    columns: [
      col("id", "-", "Check id"),
      col("scope", "-", "code = design-independent; design = this configuration"),
      col("name", "-", "What is checked"),
      col("formula", "-", "Relation used"),
      col("reference", "see name", "Reference value"),
      col("value", "see name", "Value computed by Bowshock"),
      col("abs_err", "see name", "|value − reference|"),
      col("rel_err", "-", "abs_err/|reference| (= abs_err when reference = 0)"),
      col("tol", "-", "Tolerance"),
      col("pass", "-", "TRUE if within tolerance"),
      col("basis", "-", "closed-form / independent-code / invariant / consistency / published-table / published-example / published-formula"),
      col("source", "-", "Citation"),
    ],
    rows: checks.map((c) => [c.id, c.id.startsWith("design-") ? "design" : "code", c.name, c.formula, c.expected, c.got, c.absErr, c.relErr, c.tol, c.pass, c.basis, c.source]),
  });

  /* ---- provenance */
  const inputHash = key;
  const prov: Provenance = {
    format: DATAPACK_FORMAT,
    bowshockVersion: optsIn?.version ?? BOWSHOCK_VERSION,
    gitHash: optsIn?.gitHash ?? "",
    inputHash,
    timestamp: optsIn?.now ?? new Date().toISOString(),
    family: p.family,
    aeroMethod: p.aeroMethod,
    models: modelList(p, aero),
    options: { stations: o.stations, panels: o.panels, parametric: o.parametric, alphaSweep: o.alphaSweep, machSweep: o.machSweep, carpet: o.carpet },
  };
  tables.push({
    id: "provenance",
    title: "Provenance",
    description: "How this pack was made. The input hash is the cache key: identical inputs give an identical pack.",
    columns: [col("field", "-", "Field"), col("value", "-", "Value")],
    rows: [
      ["format", prov.format],
      ["bowshock_version", prov.bowshockVersion],
      ["git_hash", prov.gitHash || "n/a"],
      ["input_hash", prov.inputHash],
      ["timestamp_utc", prov.timestamp],
      ["family", prov.family],
      ["aero_method", prov.aeroMethod],
      ...prov.models.map((m, i) => [`model_${i + 1}`, m] as Cell[]),
      ["validation_checks", `${vs.pass}/${vs.n} pass`],
    ],
  });

  const elapsedMs = (typeof performance !== "undefined" ? performance.now() : Date.now()) - t0;
  yield { frac: 1, label: "done" };
  return { key, provenance: prov, tables, validation: vs, elapsedMs };
}

function modelList(p: DesignParams, aero: PanelAero): string[] {
  const m = [
    "Atmosphere: U.S. Standard Atmosphere 1976 (geometric altitude)",
    "Gas: calorically perfect air, γ from design; vibrational γ_eq diagnostic (Hansen SP-3013 / SHO)",
    "Shocks: oblique θ-β-M (Anderson 4.17), Rankine–Hugoniot, Prandtl–Meyer (4.44), Taylor–Maccoll cone (RK4)",
    "Skin friction: van Driest II on the wetted area, laminar Blasius·√C* below Re_L=5e5 with Schlichting transition correction",
    "Heating: Fay–Riddell / Sutton–Graves stagnation; Eckert reference-temperature (Zoby) + Tauber + Lees on the surface; radiation-equilibrium wall",
    "Viscous interaction: Hayes–Probstein weak/strong p/p_inv applied to windward absolute pressure",
    "Base: Love Cp = −1/M²",
  ];
  const meth: Record<string, string> = {
    cbaero: "Aero method: CBAERO-class local inclination (tangent-wedge/cone windward, Prandtl–Meyer leeward)",
    tangent: "Aero method: tangent-wedge/cone",
    newtonian: "Aero method: modified Newtonian (Lees)",
    mixed: "Aero method: shock-expansion strip marching (oblique shock/Taylor–Maccoll at the nose, Prandtl–Meyer after)",
    euler: "Aero method: CBAERO base (exact oblique-shock/Taylor–Maccoll on attached surfaces) blended with a symmetry-plane HLLC Euler solve (Rust → wasm) only where the panel normal matches the centreline wall",
  };
  m.push(meth[p.aeroMethod] ?? `Aero method: ${p.aeroMethod}`);
  if (aero.notes.length) m.push(`Method notes: ${aero.notes.join(" ")}`);
  return m;
}

/* ------------------------------------------------------------------ design-specific checks */

function designChecks(built: BuiltVehicle, p: DesignParams, aero: PanelAero, tris: Tri[], M: number, qInf: number, S: number): Check[] {
  const out: Check[] = [];
  const mk = (id: string, name: string, formula: string, expected: number, got: number, tol: number, basis: Check["basis"], source: string, absTol = 1e-9): void => {
    const absErr = Math.abs(got - expected);
    const bound = expected === 0;
    const relErr = bound ? absErr : absErr / Math.max(Math.abs(expected), 1e-12);
    out.push({
      id,
      domain: "external",
      name,
      formula,
      expected,
      got,
      relErr,
      absErr,
      tol: bound ? absTol : tol,
      basis,
      source,
      pass: Number.isFinite(got) && (bound ? absErr <= absTol : relErr <= tol || absErr <= absTol),
    });
  };

  // 1. closed surface: Σ A n = 0
  if (built.quality.watertight) {
    let sx = 0;
    let sy = 0;
    let sz = 0;
    let A = 0;
    for (const t of tris) {
      sx += t.nx * t.area;
      sy += t.ny * t.area;
      sz += t.nz * t.area;
      A += t.area;
    }
    mk("design-closure", "Closed surface: |Σ A·n| / ΣA", "Gauss: ∮ n dA = 0 for a watertight body", 0, Math.hypot(sx, sy, sz) / Math.max(A, 1e-12), 0, "invariant", "divergence theorem", 1e-6);
  }

  // 2. table sums reproduce the integrated pressure-force coefficients
  let fx = 0;
  let fz = 0;
  for (let t = 0; t < tris.length; t++) {
    const tr = tris[t];
    const f = -aero.cp[t] * tr.area;
    fx += f * tr.nx;
    fz += f * tr.nz;
  }
  let my = 0;
  for (let t = 0; t < tris.length; t++) {
    const tr = tris[t];
    const f = -aero.cp[t] * tr.area;
    my += tr.cz * f * tr.nx - (tr.cx - aero.cg[0]) * f * tr.nz;
  }
  const cnTab = fz / S;
  const caTab = fx / S;
  const tolF = p.aeroMethod === "euler" ? 0.05 : 1e-3;
  mk("design-sum-cn", "Panel table: Σ(−Cp A n_z)/S vs integrated CN", "table consistency (Cp stored as float32)", aero.cnA, cnTab, tolF, "consistency", "internal", 2e-4);
  mk("design-sum-ca", "Panel table: Σ(−Cp A n_x)/S vs integrated CA", "table consistency", aero.ca, caTab, tolF, "consistency", "internal", 2e-4);

  mk("design-sum-cm", "Panel table: Σ(r×F)_y/(S·L) vs integrated Cm", "table consistency; r measured from the CG, nose-up positive", aero.cm, my / (S * p.length), tolF, "consistency", "internal", 5e-4);

  // 3. L/D ceiling for waverider families: the generating-wedge cot θ.
  if (built.aero.thetaDeg > 0.1 && aero.ld > 0 && ["caret", "cone", "osculating", "wedgecone", "viscopt", "elliptic"].includes(p.family)) {
    const cot = 1 / Math.tan(built.aero.thetaDeg * DEG);
    // pass if L/D ≤ ceiling·(1+tol); report ld as value, ceiling as reference
    const ok = aero.ld <= cot * 1.05;
    out.push({
      id: "design-ld-ceiling",
      domain: "external",
      name: "Panel L/D does not exceed the generating-wedge ceiling cot θ",
      formula: "L/D ≤ cot θ (inviscid 2-D wedge); friction and base drag only lower it",
      expected: cot,
      got: aero.ld,
      relErr: Math.abs(aero.ld - cot) / cot,
      absErr: Math.abs(aero.ld - cot),
      tol: 0.05,
      basis: "invariant",
      source: "Nonweiler; exact 2-D force ratio",
      pass: ok,
    });
  }

  // 4. caret lower-surface Cp vs the exact oblique shock (α = 0, flight = design Mach)
  if (p.family === "caret" && Math.abs(p.alphaDeg) < 1e-6 && Math.abs(p.betaDeg) < 1e-6 && Math.abs(M - p.mach) < 1e-6 && built.aero.attached) {
    let A = 0;
    let cpA = 0;
    for (let t = 0; t < tris.length; t++) {
      const tr = tris[t];
      if (tr.rawSurf !== SURFACE_ID.lower || tr.nz > -0.5) continue;
      A += tr.area;
      cpA += aero.cp[t] * tr.area;
    }
    if (A > 0) {
      // local face inclination to the freestream: asin(−n·x̂)
      let thA = 0;
      for (const tr of tris) if (tr.rawSurf === SURFACE_ID.lower && tr.nz <= -0.5) thA += Math.asin(clamp(-tr.nx, -1, 1)) * tr.area;
      const th = thA / A;
      const beta = betaFromThetaM(M, th, p.gamma);
      if (Number.isFinite(beta)) {
        const sh = obliqueShock(M, beta, p.gamma);
        // panel methods multiply windward static pressure by the Hayes–Probstein p/p_inv
        const qd = 0.5 * p.gamma * M * M;
        const cpExact = (sh.p2p1 * aero.pVisc - 1) / qd;
        mk(
          "design-caret-cp",
          "Caret lower-face mean Cp vs exact oblique shock at the face's own inclination",
          `Cp = (p₂/p₁·p/p_inv − 1)/(½γM²) at θ = ${(th * RAD).toFixed(3)}° (p/p_inv = ${aero.pVisc.toFixed(3)})`,
          cpExact,
          cpA / A,
          0.1,
          "closed-form",
          "Anderson 4.17 + Rankine–Hugoniot (method limitation: tangent-wedge/CBAERO under-predict; shock-expansion recovers it)",
        );
      }
    }
  }

  void qInf;
  return out;
}

/* ------------------------------------------------------------------ public compute API */

const CACHE_MAX = 6;
const cache = new Map<string, DataPack>();

function remember(pack: DataPack) {
  cache.delete(pack.key);
  cache.set(pack.key, pack);
  while (cache.size > CACHE_MAX) {
    const first = cache.keys().next().value;
    if (first === undefined) break;
    cache.delete(first);
  }
}

export function clearDataPackCache() {
  cache.clear();
}

export function datapackCached(params: Partial<DesignParams>, opts?: DataPackOptions): DataPack | null {
  return cache.get(datapackKey(params, opts)) ?? null;
}

/** Memoized on `datapackKey`. Synchronous; ~0.3–2 s depending on method and sweep sizes. */
export function computeDataPack(paramsIn: Partial<DesignParams>, opts?: DataPackOptions): DataPack {
  const params = normalizeParams(paramsIn);
  const key = datapackKey(params, opts);
  const hit = cache.get(key);
  if (hit) {
    remember(hit);
    return hit;
  }
  const it = packSteps(params, opts, key);
  let r = it.next();
  while (!r.done) r = it.next();
  remember(r.value);
  return r.value;
}

const tick = () => new Promise<void>((res) => setTimeout(res, 0));

/**
 * Same result as computeDataPack, but yields to the event loop between sweep points so the page
 * stays responsive. `signal` aborts (the promise rejects with Error("aborted")).
 */
export async function computeDataPackAsync(
  paramsIn: Partial<DesignParams>,
  opts?: DataPackOptions,
  hooks?: { signal?: AbortSignal; onProgress?: (p: Progress) => void },
): Promise<DataPack> {
  const params = normalizeParams(paramsIn);
  const key = datapackKey(params, opts);
  const hit = cache.get(key);
  if (hit) {
    remember(hit);
    return hit;
  }
  const it = packSteps(params, opts, key);
  let r = it.next();
  while (!r.done) {
    if (hooks?.signal?.aborted) throw new Error("aborted");
    hooks?.onProgress?.(r.value);
    await tick();
    r = it.next();
  }
  remember(r.value);
  return r.value;
}

/* ------------------------------------------------------------------ CSV / JSON / README */

export function formatCell(c: Cell): string {
  if (c === null || c === undefined) return "";
  if (typeof c === "number") return Number.isFinite(c) ? String(Number(c.toPrecision(9))) : "";
  if (typeof c === "boolean") return c ? "TRUE" : "FALSE";
  return c;
}

function csvEscape(s: string): string {
  return /[",\n\r]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
}

export function tableToCsv(t: DataTable): string {
  const lines = [t.columns.map((c) => csvEscape(c.key)).join(",")];
  for (const r of t.rows) lines.push(r.map((c) => csvEscape(formatCell(c))).join(","));
  return `${lines.join("\n")}\n`;
}

/** RFC-4180 parser, for round-trip tests and for scripts that read the CSV back. */
export function parseCsv(text: string): string[][] {
  const rows: string[][] = [];
  let row: string[] = [];
  let cur = "";
  let inQ = false;
  for (let i = 0; i < text.length; i++) {
    const ch = text[i];
    if (inQ) {
      if (ch === '"') {
        if (text[i + 1] === '"') {
          cur += '"';
          i++;
        } else inQ = false;
      } else cur += ch;
    } else if (ch === '"') inQ = true;
    else if (ch === ",") {
      row.push(cur);
      cur = "";
    } else if (ch === "\n" || ch === "\r") {
      if (ch === "\r" && text[i + 1] === "\n") i++;
      row.push(cur);
      cur = "";
      rows.push(row);
      row = [];
    } else cur += ch;
  }
  if (cur.length || row.length) {
    row.push(cur);
    rows.push(row);
  }
  return rows;
}

/** JSON-safe copy (NaN/Infinity → null). */
function jsonSafe(c: Cell): Cell {
  return typeof c === "number" && !Number.isFinite(c) ? null : c;
}

export function packToJson(pack: DataPack, pretty = false): string {
  const body = {
    format: DATAPACK_FORMAT,
    key: pack.key,
    provenance: pack.provenance,
    validation: pack.validation,
    tables: pack.tables.map((t) => ({
      id: t.id,
      title: t.title,
      description: t.description,
      columns: t.columns,
      rows: t.rows.map((r) => r.map(jsonSafe)),
    })),
  };
  return JSON.stringify(body, null, pretty ? 2 : undefined);
}

/** Table rows as an array of objects keyed by column name. */
export function tableRecords(t: DataTable): Record<string, Cell>[] {
  return t.rows.map((r) => Object.fromEntries(t.columns.map((c, i) => [c.key, r[i]])));
}

export function dataDictionary(pack: DataPack): string {
  const L: string[] = [];
  L.push("BOWSHOCK DATA PACKAGE");
  L.push("=====================");
  L.push(`format      ${pack.provenance.format}`);
  L.push(`input hash  ${pack.key}   (identical inputs → identical numbers)`);
  L.push(`family      ${pack.provenance.family}   method ${pack.provenance.aeroMethod}`);
  L.push(`generated   ${pack.provenance.timestamp}   version ${pack.provenance.bowshockVersion}   git ${pack.provenance.gitHash || "n/a"}`);
  L.push(`validation  ${pack.validation.pass}/${pack.validation.n} checks pass`);
  L.push("");
  L.push("CONVENTIONS");
  L.push("  x streamwise (flow toward +x, nose at the origin), +y starboard, z up. SI units unless the column name says otherwise.");
  L.push("  Units are part of every column name (_m, _m2, _deg, _W_cm2, _K, _Pa). '-' means dimensionless.");
  L.push("  Force/moment coefficients use S_ref = planform area, L_ref = body length. Cm is about the CG, nose-up positive.");
  L.push("  CN_body / CA_body are body-axis (pressure only); CL / CD are wind axes and CD includes friction and base.");
  L.push("  beta > 0 means the flow comes toward +y (wind from the left).");
  L.push("  Empty cell = not defined for that row (e.g. no lower-surface triangles in a bin).");
  L.push("");
  L.push("FILES  (each table is <id>.csv; all of them are also in datapack.json)");
  for (const t of pack.tables) {
    L.push("");
    L.push(`${t.id}.csv — ${t.title}   [${t.rows.length} rows]`);
    L.push(`  ${t.description}`);
    for (const c of t.columns) L.push(`    ${c.key.padEnd(28)} ${c.unit.padEnd(9)} ${c.desc}`);
  }
  L.push("");
  L.push("LIMITS (what is NOT validated)");
  L.push("  Panel/shock-expansion/Euler-centerline methods are engineering models. Validation covers closed-form shock/expansion/cone relations,");
  L.push("  the atmosphere, skin-friction and heating correlations against independent implementations, and internal consistency of this pack.");
  L.push("  There is no comparison against wind-tunnel or flight data in this repository; heating blend weights and the viscous-interaction");
  L.push("  coupling are engineering choices; real-gas effects are diagnostic only. Do not use for flight-critical decisions without Cart3D/CFD checks.");
  return `${L.join("\n")}\n`;
}

export interface PackFile {
  name: string;
  content: string;
}

/** README + datapack.json + one CSV per table, ready to drop into a zip. */
export function packFiles(pack: DataPack, dir = "data"): PackFile[] {
  const f: PackFile[] = [
    { name: `${dir}/README.txt`, content: dataDictionary(pack) },
    { name: `${dir}/datapack.json`, content: packToJson(pack, false) },
  ];
  for (const t of pack.tables) f.push({ name: `${dir}/${t.id}.csv`, content: tableToCsv(t) });
  return f;
}
