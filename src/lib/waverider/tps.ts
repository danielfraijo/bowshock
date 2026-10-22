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

export type TpsMaterialId = "pica" | "tufroc" | "rcc" | "sla" | "tile" | "inconel" | "titanium" | "aluminum";

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

export const TPS_LIB: Record<TpsMaterialId, TpsMaterial> = {
  pica: { id: "pica", label: "PICA", rho: 240, k: 0.08, tmax: 1800, emiss: 0.85, cp: 1500, qStar: 25e6, note: "Phenolic impregnated carbon ablator. Q* is a screening heat of ablation." },
  tufroc: { id: "tufroc", label: "TUFROC-class", rho: 400, k: 0.15, tmax: 1920, emiss: 0.9, cp: 1100, qStar: 8e6, note: "Toughened fibrous ceramic. Reusable while q stays under reradiation." },
  rcc: { id: "rcc", label: "RCC", rho: 1580, k: 4.5, tmax: 1920, emiss: 0.89, cp: 800, qStar: 5e6, note: "Reinforced carbon-carbon. High conductivity, so the insulation thickness grows." },
  sla: { id: "sla", label: "SLA-561", rho: 250, k: 0.06, tmax: 1650, emiss: 0.8, cp: 1400, qStar: 15e6, note: "Silicone ablator used as an Earth/Mars entry screen." },
  tile: { id: "tile", label: "LI-900 tile", rho: 144, k: 0.05, tmax: 1530, emiss: 0.85, cp: 900, qStar: 2e6, note: "Shuttle-class tile. Low heat flux only." },
  inconel: { id: "inconel", label: "Inconel-class", rho: 8220, k: 15, tmax: 1370, emiss: 0.75, cp: 500, qStar: 1e6, note: "Hot metallic skin. A poor insulator." },
  titanium: { id: "titanium", label: "Titanium", rho: 4430, k: 8, tmax: 820, emiss: 0.55, cp: 530, qStar: 0.8e6, note: "Warm structure. Not a stagnation material." },
  aluminum: { id: "aluminum", label: "Aluminum", rho: 2700, k: 160, tmax: 450, emiss: 0.35, cp: 900, qStar: 0.4e6, note: "Substructure. It fails if it sees the aerothermal load directly." },
};

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
  const notes = [
    `${mat.label}: δ = max(gauge, kΔT/q_back, ablator). q_back default 0.5 W/cm².`,
    over
      ? `Peak ${q.qWcm2.toFixed(2)} W/cm² exceeds reradiation at ${mat.tmax} K (${(qRad / 1e4).toFixed(2)} W/cm²). Ablator thickness uses Q* = ${(mat.qStar / 1e6).toFixed(0)} MJ/kg over ${q.timeS.toFixed(0)} s.`
      : `Peak is under the reradiation limit (${(qRad / 1e4).toFixed(2)} W/cm²). Thickness is insulation, not recession.`,
    mat.note,
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
}

export function massBudget(q: MassQuery): MassBudget {
  const L = Math.max(q.length, 1e-6);
  const struct = Math.max(0, q.structMass);
  const tps = Math.max(0, q.tpsMass);
  const volumeFuel = Math.max(0, q.volume) * clamp01(q.fuelFrac);
  const fuel = volumeFuel * Math.max(0, q.fuelRho);
  const payload = Math.max(0, q.payloadKg);
  const total = Math.max(1e-6, struct + tps + fuel + payload);
  const parts = [
    { m: struct, x: q.structCgX, z: 0 },
    { m: tps, x: (q.tpsXFrac ?? 0.35) * L, z: -0.12 * Math.max(q.height, 0) },
    { m: fuel, x: (q.fuelXFrac ?? 0.55) * L, z: 0 },
    { m: payload, x: (q.payloadXFrac ?? 0.45) * L, z: 0 },
  ];
  let mx = 0;
  let mz = 0;
  for (const p of parts) {
    mx += p.m * p.x;
    mz += p.m * p.z;
  }
  const cgX = mx / total;
  const cgZ = mz / total;
  const scale = total / Math.max(struct, 1e-6);
  const shift = struct > 1 ? (cgX - q.structCgX) ** 2 : 0;
  const notes = [
    "Airframe mass is the smeared density × volume already used by the Mass tab. TPS, fuel, and payload are added; they are not inside that density.",
    `Fuel volume ${volumeFuel.toFixed(3)} m³ at ${q.fuelRho.toFixed(0)} kg/m³. CG shifts with the load; inertias scale with mass and pick up a parallel-axis term from the CG move.`,
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
    ixx: q.ixx * scale,
    iyy: q.iyy * scale + total * shift,
    izz: q.izz * scale + total * shift,
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
