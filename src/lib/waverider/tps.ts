/**
 * Screening TPS, structure, and mass properties.
 *
 * This is not a thermal-protection qualification. Two limits are applied:
 *
 * 1. Reradiation. A reusable surface at Tmax rejects q_rad = ε σ Tmax⁴.
 *    If the convective peak is above that, the material is over temperature
 *    and an ablator thickness is added from a heat of ablation Q*:
 *        δ_abl = q̇ t / (ρ Q*)
 * 2. Insulation. Thickness is chosen so conduction into a 300 K backface
 *    stays under an allowable flux (default 0.5 W/cm²):
 *        δ_ins = k (Tmax − Tback) / q_allow
 *    Thicker is more protective. The reported thickness is the larger of the
 *    insulation need, the ablator need, and a minimum gauge.
 *
 * Structural mass is the existing smeared density × volume (the airframe
 * without TPS, propellant, or payload). Those three are added. CG is the
 * mass-weighted station. Inertias scale with the new mass about that CG.
 */
import type { DesignParams } from "./types";

export type TpsMaterialId = "pica" | "tufroc" | "rcc" | "sla" | "tile" | "li2200" | "aetb8" | "aetb12" | "inconel" | "titanium" | "aluminum";

export interface TpsMaterial {
  id: TpsMaterialId;
  label: string;
  /** kg/m³ */
  rho: number;
  /** W/m·K, through-thickness, a single screening value. */
  k: number;
  /** Surface limit, K. */
  tmax: number;
  emiss: number;
  /** J/kg·K, only used in the notes. */
  cp: number;
  /** Heat of ablation, J/kg. Screening value. */
  qStar: number;
  note: string;
}

/**
 * Material screening values.
 *
 * [accuracy-audit] ρ, k (through-the-thickness), cp, ε and the multiple-use temperature limit are the NASA Ames
 * TPSX Material Properties Database values "at standard conditions" (tpsx.arc.nasa.gov, retrieved 2026-10-22;
 * material ids in each note) wherever TPSX lists them. Cross-reference: Williams & Curry, "Thermal Protection
 * Materials: Thermophysical Property Data", NASA RP-1289 (1992) — not re-checked page by page here.
 * Room-temperature k is a lower bound: fibrous-insulation k rises with temperature and gas pressure (TPSX tabulates k(T, p)).
 * Q* (heat of ablation) and the TUFROC / SLA-561 / titanium rows are NOT from TPSX and stay screening values.
 */
export const TPS_LIB: Record<TpsMaterialId, TpsMaterial> = {
  pica: { id: "pica", label: "PICA", rho: 236, k: 0.305, tmax: 1800, emiss: 0.8, cp: 1200, qStar: 25e6, note: "Phenolic impregnated carbon ablator. ρ, k, cp, ε: TPSX #43 (virgin). Tmax and Q* are screening values (not in TPSX)." },
  tufroc: { id: "tufroc", label: "TUFROC-class", rho: 400, k: 0.15, tmax: 1920, emiss: 0.9, cp: 1100, qStar: 8e6, note: "Toughened fibrous ceramic. Reusable while q stays under reradiation. Screening values — not checked against TPSX." },
  rcc: { id: "rcc", label: "RCC", rho: 1580, k: 3.89, tmax: 1920, emiss: 0.78, cp: 712, qStar: 5e6, note: "Reinforced carbon-carbon (19-ply). TPSX #24. High conductivity, so the insulation thickness grows." },
  sla: { id: "sla", label: "SLA-561", rho: 250, k: 0.06, tmax: 1650, emiss: 0.8, cp: 1400, qStar: 15e6, note: "Silicone ablator used as an Earth/Mars entry screen. Screening values — not checked against TPSX." },
  tile: { id: "tile", label: "LI-900 tile", rho: 144, k: 0.0476, tmax: 1590, emiss: 0.88, cp: 628, qStar: 2e6, note: "Shuttle-class silica tile. TPSX #1 (uncoated). Low heat flux only." },
  li2200: { id: "li2200", label: "LI-2200 tile", rho: 352, k: 0.0744, tmax: 1640, emiss: 0.899, cp: 628, qStar: 2e6, note: "High-density silica tile. TPSX #2." },
  aetb8: { id: "aetb8", label: "AETB-8 tile", rho: 128, k: 0.0684, tmax: 1640, emiss: 0.899, cp: 628, qStar: 2e6, note: "Alumina-enhanced thermal barrier. TPSX #5 (k and cp are TPSX 'predicted')." },
  aetb12: { id: "aetb12", label: "AETB-12 tile", rho: 192, k: 0.064, tmax: 1700, emiss: 0.88, cp: 628, qStar: 2e6, note: "Alumina-enhanced thermal barrier. TPSX #6 (k and cp are TPSX 'predicted')." },
  inconel: { id: "inconel", label: "Inconel 617", rho: 8410, k: 15.1, tmax: 1370, emiss: 0.75, cp: 456, qStar: 1e6, note: "Hot metallic skin. ρ, k, cp: TPSX #54. Tmax, ε are screening values. A poor insulator." },
  titanium: { id: "titanium", label: "Titanium alloy", rho: 4430, k: 8, tmax: 820, emiss: 0.55, cp: 530, qStar: 0.8e6, note: "Warm structure. Ti-alloy screening values; TPSX #58 is unalloyed Ti (4500 kg/m³, 21.9 W/m·K) and was not used. Not a stagnation material." },
  aluminum: { id: "aluminum", label: "Aluminum 2024-T6", rho: 2770, k: 177, tmax: 450, emiss: 0.35, cp: 875, qStar: 0.4e6, note: "Substructure. ρ, k, cp: TPSX #60. It fails if it sees the aerothermal load directly." },
};

/** TPSX #11 strain isolator pad (bonds a tile to the skin). Not a surface material, so not in TPS_LIB. */
export const TPSX_SIP = { rho: 86.5, k: 0.0406, cp: 1310, tmaxMultiUse: 506 };

const SIGMA = 5.670374419e-8;

export interface TpsSizing {
  material: TpsMaterial;
  /** m */
  thickness: number;
  thicknessMm: number;
  mass: number;
  areaM2: number;
  /** W/m² the surface can reradiate at Tmax. */
  qRadWm2: number;
  /** Convective peak, W/m². */
  qWm2: number;
  overTemperature: boolean;
  /** (qRad − q) / qRad. Negative means the surface cannot reject the load by radiation. */
  radMargin: number;
  deltaInsul: number;
  deltaAbl: number;
  /**
   * [accuracy-audit] Peak backface temperature, K, from a 1-D transient conduction solve (`conduct1D`) of the sized
   * thickness: cold-wall q in, ε σ T⁴ out at the surface, adiabatic backface (no structural heat sink), held for
   * `timeS`. NaN when the material is ablating (recession is not modelled).
   */
  backfaceK: number;
  notes: string[];
}

export interface TpsQuery {
  /** Peak convective heat flux, W/cm² (same unit the Heat tab prints). */
  qWcm2: number;
  /** Wetted area that carries TPS, m². */
  areaM2: number;
  /** Heat-pulse duration for the ablator, s. */
  timeS: number;
  material: TpsMaterialId;
  backK?: number;
  /** Allowable flux into the backface, W/cm². Default 0.5. */
  qBackWcm2?: number;
  minMm?: number;
}

export function sizeTps(q: TpsQuery): TpsSizing {
  const mat = TPS_LIB[q.material] ?? TPS_LIB.pica;
  const qWm2 = Math.max(0, q.qWcm2) * 1e4;
  const qBack = Math.max(100, (q.qBackWcm2 ?? 0.5) * 1e4);
  const Tb = q.backK ?? 300;
  const dT = Math.max(40, mat.tmax - Tb);
  const qRad = mat.emiss * SIGMA * mat.tmax ** 4;
  const over = qWm2 > qRad * 1.02;
  const deltaInsul = (mat.k * dT) / qBack;
  const deltaAbl = over ? (qWm2 * Math.max(q.timeS, 1)) / (mat.rho * mat.qStar) : 0;
  const deltaMin = Math.max(0.001, (q.minMm ?? 3) / 1000);
  const thickness = Math.min(0.25, Math.max(deltaMin, deltaInsul, deltaAbl));
  const area = Math.max(0, q.areaM2);
  const mass = mat.rho * thickness * area;
  const backfaceK = over ? NaN : transientBackface(mat, thickness, qWm2, Math.max(q.timeS, 1), Tb).backMaxK;
  const notes = [
    `${mat.label}: δ = max(gauge, kΔT/q_back, ablator). q_back default 0.5 W/cm².`,
    over
      ? `Peak ${q.qWcm2.toFixed(2)} W/cm² exceeds reradiation at ${mat.tmax} K (${(qRad / 1e4).toFixed(2)} W/cm²). Ablator thickness uses Q* = ${(mat.qStar / 1e6).toFixed(0)} MJ/kg over ${q.timeS.toFixed(0)} s.`
      : `Peak is under the reradiation limit (${(qRad / 1e4).toFixed(2)} W/cm²). Thickness is insulation, not recession.`,
    mat.note,
    Number.isFinite(backfaceK)
      ? `Transient 1-D check: backface peaks at ${backfaceK.toFixed(0)} K after ${Math.max(q.timeS, 1).toFixed(0)} s (reradiating surface, adiabatic back, room-temperature k).`
      : "Transient backface check skipped: the surface is over temperature (ablating).",
  ];
  return {
    material: mat,
    thickness,
    thicknessMm: thickness * 1000,
    mass,
    areaM2: area,
    qRadWm2: qRad,
    qWm2,
    overTemperature: over,
    radMargin: qRad > 1 ? (qRad - qWm2) / qRad : 0,
    deltaInsul,
    deltaAbl,
    backfaceK,
    notes,
  };
}

export interface MassBudget {
  struct: number;
  tps: number;
  fuel: number;
  payload: number;
  total: number;
  cgX: number;
  cgFrac: number;
  cgZ: number;
  iyy: number;
  ixx: number;
  izz: number;
  volumeFuel: number;
  notes: string[];
}

export interface MassQuery {
  params: DesignParams;
  volume: number;
  length: number;
  height: number;
  /** Smeared-density airframe mass, kg. TPS / fuel / payload are added on top. */
  structMass: number;
  structCgX: number;
  ixx: number;
  iyy: number;
  izz: number;
  tpsMass: number;
  /** TPS centroid as a fraction of length, from the nose. */
  tpsXFrac?: number;
  payloadKg: number;
  payloadXFrac?: number;
  /** Fraction of the enclosed volume filled with propellant. */
  fuelFrac: number;
  /** kg/m³. Liquid H2 ~ 71, JP ~ 800. */
  fuelRho: number;
  fuelXFrac?: number;
  /** Structure CG height, m (massProperties cg[2]). Default 0. */
  structCgZ?: number;
  /** TPS volume, m³ (thickness × area). It sits inside the OML, so it is taken out of the tankable volume. Default 0. */
  tpsVolume?: number;
}

export function massBudget(q: MassQuery): MassBudget {
  const L = Math.max(q.length, 1e-6);
  const struct = Math.max(0, q.structMass);
  const tps = Math.max(0, q.tpsMass);
  // [accuracy-audit] TPS occupies part of the OML volume; only the rest can hold propellant.
  const volumeFree = Math.max(0, Math.max(0, q.volume) - Math.max(0, q.tpsVolume ?? 0));
  const volumeFuel = volumeFree * clamp01(q.fuelFrac);
  const fuel = volumeFuel * Math.max(0, q.fuelRho);
  const payload = Math.max(0, q.payloadKg);
  const total = Math.max(1e-6, struct + tps + fuel + payload);
  const structZ = q.structCgZ ?? 0;
  const parts = [
    { m: struct, x: q.structCgX, z: structZ },
    { m: tps, x: (q.tpsXFrac ?? 0.35) * L, z: structZ - 0.12 * Math.max(q.height, 0) },
    { m: fuel, x: (q.fuelXFrac ?? 0.55) * L, z: structZ },
    { m: payload, x: (q.payloadXFrac ?? 0.45) * L, z: structZ },
  ];
  let mx = 0;
  let mz = 0;
  for (const p of parts) {
    mx += p.m * p.x;
    mz += p.m * p.z;
  }
  const cgX = mx / total;
  const cgZ = mz / total;
  // [accuracy-audit] Each part keeps the airframe's radius of gyration about its own CG (the old screening
  // assumption) and then moves to the combined CG with its OWN offset: I = Σ m_i k² + Σ m_i d_i². The old
  // total·(cgX − structCgX)² term is not the parallel-axis theorem: a 10 % payload at 2 m gave 0.036 m·kg·m²
  // against the correct 0.364 m (10× low). 1812dcc added the vertical shift as total·cgZ² (Ixx) and
  // total·(Δx² + Δz²) (Iyy), which still places ALL the mass at the structure-to-combined CG offset; the
  // per-part sum below replaces it (it also covers the TPS z offset that 1812dcc introduced it for).
  const sInv = struct > 1e-9 ? 1 / struct : 0;
  let ixx = 0;
  let iyy = 0;
  let izz = 0;
  for (const p of parts) {
    const own = p.m * sInv;
    const dx = p.x - cgX;
    const dz = p.z - cgZ;
    ixx += q.ixx * own + p.m * dz * dz;
    iyy += q.iyy * own + p.m * (dx * dx + dz * dz);
    izz += q.izz * own + p.m * dx * dx;
  }
  const notes = [
    "Airframe mass is the smeared density × volume already used by the Mass tab. TPS, fuel, and payload are added; they are not inside that density.",
    `Fuel volume ${volumeFuel.toFixed(3)} m³ at ${q.fuelRho.toFixed(0)} kg/m³ (OML volume less TPS volume, times the fill fraction). Each part keeps the airframe radius of gyration and adds its own parallel-axis term about the combined CG.`,
    "No FEM. A thermo-structural constraint is the TPS over-temperature flag plus this CG, which the 6DOF and the optimizer can consume.",
  ];
  return {
    struct,
    tps,
    fuel,
    payload,
    total,
    cgX,
    cgFrac: cgX / L,
    cgZ,
    iyy,
    ixx,
    izz,
    volumeFuel,
    notes,
  };
}

function clamp01(v: number) {
  return Math.max(0, Math.min(1, v));
}

/** Write the budget back onto the design so the 6DOF and the trajectory use it. */
export function applyMassToParams(params: DesignParams, budget: MassBudget): DesignParams {
  return {
    ...params,
    massKg: budget.total,
    cgFrac: Math.max(0.08, Math.min(0.92, budget.cgFrac)),
  };
}

/* ---------------------------------------------------------------------------------------------- */
/* [accuracy-audit] 1-D transient conduction through a TPS layer.                                   */
/* ---------------------------------------------------------------------------------------------- */

/** Surface or backface condition. Flux is positive INTO the slab and may depend on the face temperature as q = a − b·T. */
export type ConductionBC =
  | { kind: "T"; T: number }
  | { kind: "adiabatic" }
  | { kind: "flux"; q: (T: number, t: number) => { a: number; b: number } };

export interface Conduct1DSpec {
  k: number;
  rho: number;
  cp: number;
  /** m */
  thickness: number;
  /** Cells (nodes = n + 1). */
  n: number;
  dt: number;
  tEnd: number;
  T0: number;
  surface: ConductionBC;
  back: ConductionBC;
  /** 0.5 Crank–Nicolson (default), 1 fully implicit. */
  theta?: number;
}

export interface Conduct1DResult {
  x: Float64Array;
  T: Float64Array;
  t: number;
  /** J/m² that crossed the two faces into the slab (Dirichlet faces are reconstructed from the stored-energy change). */
  energyIn: number;
  /** J/m² stored relative to T0. */
  stored: number;
  backMaxK: number;
  surfMaxK: number;
}

/**
 * Node-centred finite volume (half cells at both faces), θ-scheme, Thomas solve. Constant k, ρ, cp.
 * Energy is conserved to round-off for flux / adiabatic faces.
 */
export function conduct1D(spec: Conduct1DSpec): Conduct1DResult {
  const n = Math.max(2, Math.round(spec.n));
  const N = n + 1;
  const dx = spec.thickness / n;
  const th = spec.theta ?? 0.5;
  const G = spec.k / dx;
  const C = new Float64Array(N);
  for (let i = 0; i < N; i++) C[i] = spec.rho * spec.cp * dx * (i === 0 || i === n ? 0.5 : 1);
  let T = new Float64Array(N).fill(spec.T0);
  if (spec.surface.kind === "T") T[0] = spec.surface.T;
  if (spec.back.kind === "T") T[n] = spec.back.T;
  const x = new Float64Array(N);
  for (let i = 0; i < N; i++) x[i] = i * dx;
  const a = new Float64Array(N);
  const b = new Float64Array(N);
  const c = new Float64Array(N);
  const d = new Float64Array(N);
  let t = 0;
  let energyIn = 0;
  let backMax = T[n];
  let surfMax = T[0];
  const steps = Math.max(1, Math.ceil(spec.tEnd / spec.dt - 1e-9));
  const dt = spec.tEnd / steps;
  for (let s = 0; s < steps; s++) {
    for (let i = 0; i < N; i++) {
      const cdt = C[i] / dt;
      const left = i > 0 ? G : 0;
      const right = i < n ? G : 0;
      a[i] = -th * left;
      c[i] = -th * right;
      b[i] = cdt + th * (left + right);
      const expl = (i > 0 ? G * (T[i - 1] - T[i]) : 0) + (i < n ? G * (T[i + 1] - T[i]) : 0);
      d[i] = cdt * T[i] + (1 - th) * expl;
    }
    const face = (bc: ConductionBC, i: number) => {
      if (bc.kind === "T") {
        a[i] = 0;
        c[i] = 0;
        b[i] = 1;
        d[i] = bc.T;
      } else if (bc.kind === "flux") {
        const f = bc.q(T[i], t);
        b[i] += th * f.b;
        d[i] += f.a - (1 - th) * f.b * T[i];
      }
    };
    face(spec.surface, 0);
    face(spec.back, n);
    // Thomas
    for (let i = 1; i < N; i++) {
      const m = a[i] / b[i - 1];
      b[i] -= m * c[i - 1];
      d[i] -= m * d[i - 1];
    }
    const Tn = new Float64Array(N);
    Tn[n] = d[n] / b[n];
    for (let i = n - 1; i >= 0; i--) Tn[i] = (d[i] - c[i] * Tn[i + 1]) / b[i];
    for (const [bc, i] of [
      [spec.surface, 0],
      [spec.back, n],
    ] as const) {
      if (bc.kind === "flux") {
        const f = bc.q(T[i], t);
        energyIn += dt * (f.a - f.b * (th * Tn[i] + (1 - th) * T[i]));
      }
    }
    T = Tn;
    t += dt;
    if (T[n] > backMax) backMax = T[n];
    if (T[0] > surfMax) surfMax = T[0];
  }
  let stored = 0;
  for (let i = 0; i < N; i++) stored += C[i] * (T[i] - spec.T0);
  if (spec.surface.kind === "T" || spec.back.kind === "T") energyIn = stored;
  return { x, T, t, energyIn, stored, backMaxK: backMax, surfMaxK: surfMax };
}

/** Reradiating surface: q = q_conv − ε σ T⁴, linearised about the previous surface temperature. */
export function reradiatingSurface(qConvWm2: number, emiss: number): ConductionBC {
  return {
    kind: "flux",
    q: (T: number) => {
      const r = emiss * SIGMA * T * T * T;
      return { a: qConvWm2 + 3 * r * T, b: 4 * r };
    },
  };
}

/** Backface history of one TPS layer under a constant cold-wall flux with surface reradiation and an adiabatic back. */
export function transientBackface(mat: TpsMaterial, thicknessM: number, qWm2: number, timeS: number, T0 = 300, n = 40): Conduct1DResult {
  const alpha = mat.k / (mat.rho * mat.cp);
  const dx = thicknessM / n;
  const dt = Math.min(timeS / 200, Math.max((0.5 * dx * dx) / Math.max(alpha, 1e-12), timeS / 4000));
  return conduct1D({
    k: mat.k,
    rho: mat.rho,
    cp: mat.cp,
    thickness: thicknessM,
    n,
    dt,
    tEnd: timeS,
    T0,
    surface: reradiatingSurface(qWm2, mat.emiss),
    back: { kind: "adiabatic" },
    theta: 1,
  });
}
