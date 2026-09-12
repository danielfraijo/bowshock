/** 1-D internal flow: multi-ramp θ-β-M, Rankine–Hugoniot, Fanno, Rayleigh, isentropic nozzle. */

import type { DesignParams, FuelKind } from "./types";
import { domainOf } from "./types";
import {
  DEG,
  RAD,
  areaRatio,
  betaFromThetaM,
  clamp,
  isentropic,
  kantrowitz,
  machFromArea,
  normalShock,
  obliqueShock,
  rayleighM2,
  rayleighPStar,
  waltrupBillig,
} from "./math";
import type { Atmosphere } from "./atmosphere";

export interface Station {
  name: string;
  M: number;
  p: number;
  T: number;
  pt: number;
  Tt: number;
  p_p0: number;
}

export interface ShockJump {
  name: string;
  thetaDeg: number;
  betaDeg: number;
  M1: number;
  M2: number;
  p2p1: number;
  T2T1: number;
}

export interface InternalResult {
  kind: "ramjet" | "scramjet" | "inlet";
  stations: Station[];
  shocks: ShockJump[];
  capture: number;
  mdot: number;
  mdotFuel: number;
  thrust: number;
  isp: number;
  tsfc: number;
  Tt0: number;
  pt0: number;
  Tt4: number;
  Te: number;
  Ue: number;
  Ue0: number;
  M2: number;
  p2p0: number;
  piD: number;
  started: boolean;
  kantrowitz: number;
  contraction: number;
  isolatorLH: number;
  etaThermal: number;
  phi: number;
  fuel: FuelKind;
  erGeom: number;
  equations: { name: string; expr: string }[];
  notes: string[];
}

const CP_AIR = 1004.7;
const R_AIR = 287.05;
const G0 = 9.80665;

function farStoich(fuel: FuelKind) {
  return fuel === "H2" ? 0.0291 : 0.068;
}

function hf(fuel: FuelKind) {
  return fuel === "H2" ? 1.2e8 : 4.3e7;
}

function Tlimit(fuel: FuelKind) {
  return fuel === "H2" ? 2400 : 1900;
}

function station(name: string, M: number, p: number, T: number, p0inf: number, g: number): Station {
  const iso = isentropic(M, g);
  return {
    name,
    M,
    p,
    T,
    pt: p * iso.ptP,
    Tt: T * iso.TtT,
    p_p0: p / p0inf,
  };
}

export function internalCycle(params: DesignParams, atm: Atmosphere): InternalResult | null {
  const fam = params.family;
  if (domainOf(fam) !== "internal") return null;

  const notes: string[] = [];
  const g = params.gamma;
  const M0 = Math.max(1.5, params.lockFlight ? params.mach : params.flightMach);
  const T0 = atm.T;
  const p0 = atm.p;
  const iso0 = isentropic(M0, g);
  const Tt0 = T0 * iso0.TtT;
  const pt0 = p0 * iso0.ptP;
  const scram = fam === "scramjet";
  const inletOnly = fam === "busemann" || fam === "inward";
  const nR = clamp(Math.round(params.nRamps || 2), 1, 3);
  const delta = (params.rampDeg * DEG) / nR;

  const stations: Station[] = [station("0  freestream", M0, p0, T0, p0, g)];
  const shocks: ShockJump[] = [];

  let M = M0;
  let p = p0;
  let T = T0;
  let attached = true;

  for (let k = 0; k < nR; k++) {
    const beta = betaFromThetaM(M, delta, g);
    if (!Number.isFinite(beta)) {
      attached = false;
      notes.push(`Ramp ${k + 1}: shock detached at M=${M.toFixed(2)}, δ=${(delta * RAD).toFixed(1)}°.`);
      break;
    }
    const sh = obliqueShock(M, beta, g);
    shocks.push({
      name: `ramp ${k + 1}`,
      thetaDeg: delta * RAD,
      betaDeg: beta * RAD,
      M1: M,
      M2: sh.M2,
      p2p1: sh.p2p1,
      T2T1: sh.t2t1,
    });
    M = sh.M2;
    p *= sh.p2p1;
    T *= sh.t2t1;
    stations.push(station(`${k + 1}  after ramp ${k + 1}`, M, p, T, p0, g));
  }
  const MisoIn = M;
  const pIsoIn = p;

  // Isolator: short Fanno (4fL/D ≈ 0.04) — M drifts toward 1.
  const fL = inletOnly ? 0.02 : 0.04;
  if (M > 1.05) {
    const Miso = M * (1 - 0.08 * fL * (M * M - 1));
    M = Math.max(1.05, Miso);
    notes.push(`Isolator Fanno 4fL/D ≈ ${fL.toFixed(2)} (supersonic, M ↓ toward 1).`);
  } else if (M < 0.95) {
    M = Math.min(0.95, M * (1 + 0.05 * fL));
    notes.push(`Isolator Fanno 4fL/D ≈ ${fL.toFixed(2)} (subsonic, M ↑ toward 1).`);
  }
  stations.push(station("i  isolator exit", M, p, T, p0, g));

  const K = kantrowitz(M0, g);
  const contraction = fam === "busemann" ? 0.22 : fam === "inward" ? 0.35 : 0.45;
  const started = contraction > K * 0.92;
  if (!started) notes.push(`Kantrowitz: A_t/A_c min = ${K.toFixed(3)}; geometric ${contraction.toFixed(3)} — may not self-start.`);
  else notes.push(`Kantrowitz A_t/A_c min = ${K.toFixed(3)}; geometric ${contraction.toFixed(3)} — startable.`);

  let pIsoOut = p;
  if (!scram && !inletOnly && M > 1.12) {
    const ns = normalShock(M, g);
    shocks.push({
      name: "isolator NS",
      thetaDeg: 0,
      betaDeg: 90,
      M1: M,
      M2: ns.M2,
      p2p1: ns.p2p1,
      T2T1: ns.t2t1,
    });
    p *= ns.p2p1;
    T *= ns.t2t1;
    M = ns.M2;
    pIsoOut = p;
    stations.push(station("n  after normal shock", M, p, T, p0, g));
    notes.push("Ramjet: terminal normal shock (Rankine–Hugoniot) in the isolator.");
  } else if (scram) {
    notes.push("Scramjet: combustor remains supersonic — no terminal normal shock.");
  }
  const pTrain = scram
    ? 1.8
    : inletOnly
      ? clamp(pIsoOut / Math.max(pIsoIn, 1e-6), 1.05, 2.2)
      : clamp(pIsoOut / Math.max(pIsoIn, 1e-6), 1.2, 2.8);
  const Mwb = clamp(MisoIn, 1.5, 3.3);
  const isolatorLH = waltrupBillig(Mwb, pTrain, 0.02);
  notes.push(
    `Waltrup–Billig isolator L/H ≈ ${isolatorLH.toFixed(1)} (M_in=${MisoIn.toFixed(2)}, correlation M=${Mwb.toFixed(2)}, shock-train p_r=${pTrain.toFixed(2)}; WB is calibrated near M≲3).`,
  );

  const fuel = params.fuel;
  const phi = clamp(params.phi, 0.2, 1.4);
  const far = inletOnly ? 0 : phi * farStoich(fuel);
  const Tt1 = T * (1 + 0.5 * (g - 1) * M * M);
  const qAdd = far * hf(fuel) * 0.92;
  let Tt4 = Tt1 + qAdd / CP_AIR;
  const Tmax = Tlimit(fuel);
  if (Tt4 > Tmax) {
    Tt4 = Tmax;
    notes.push(`Tt4 limited to ${Tmax} K (${fuel} material / dissociation cap).`);
  }
  const TtRatio = Tt4 / Math.max(Tt1, 1);
  if (!inletOnly && TtRatio > 1.01) {
    const M2b = rayleighM2(M, TtRatio, g);
    const p2p1R = rayleighPStar(M2b, g) / rayleighPStar(M, g);
    p *= p2p1R;
    T = Tt4 / (1 + 0.5 * (g - 1) * M2b * M2b);
    M = M2b;
    stations.push(station("4  combustor exit (Rayleigh)", M, p, T, p0, g));
    notes.push("Rayleigh line heat addition. Subsonic: M↑ toward 1. Supersonic: M↓ toward 1.");
  }

  const ER = Math.max(1.2, params.nozzleER);
  let Me = M;
  let Te = T;
  let pe = p;
  if (!inletOnly) {
    const choked = M < 1;
    const AstarRatio = areaRatio(Math.max(M, 0.2), g);
    const AeAstar = choked ? ER : ER * AstarRatio;
    Me = machFromArea(Math.max(AeAstar, 1.05), g, true);
    Te = Tt4 / (1 + 0.5 * (g - 1) * Me * Me);
    const pe_pt = 1 / isentropic(Me, g).ptP;
    pe = (p * isentropic(M, g).ptP) * pe_pt;
    stations.push(station("e  nozzle exit", Me, pe, Te, p0, g));
    notes.push("Nozzle: isentropic A/A* inversion (Newton). pe matched toward p∞.");
  }

  const Ue = Me * Math.sqrt(g * R_AIR * Te);
  const U0 = atm.V;
  const hCap = Math.max(0.04, params.inletHeight);
  const wCap =
    fam === "ramjet" || fam === "scramjet"
      ? params.span
      : params.span * clamp(params.wedgeFrac, 0.12, 0.7);
  const capture = Math.max(1e-4, hCap * wCap);
  const mdot = atm.rho * U0 * capture * 0.92;
  const mdotFuel = mdot * far;
  const Ae = capture * (inletOnly ? 1 : ER);
  const thrust = inletOnly ? 0 : mdot * (Ue - U0) + (pe - p0) * Ae;
  const isp = mdotFuel > 1e-8 ? thrust / (mdotFuel * G0) : 0;
  const tsfc = thrust > 1e-6 ? (mdotFuel * 3600) / thrust : 0;
  const eta = (Ue * Ue - U0 * U0) / (2 * Math.max(qAdd, 1));
  const piD = stations.length > 1 ? stations[stations.length - (inletOnly ? 1 : 2)].pt / pt0 : 1;

  const equations = [
    {
      name: "θ-β-M (each ramp)",
      expr: "tan θ = 2 cot β (M² sin²β − 1) / (M² (γ + cos 2β) + 2)",
    },
    {
      name: "Oblique shock",
      expr: "p₂/p₁ = 1 + 2γ/(γ+1) (M² sin²β − 1)    M₂ sin(β−θ) = Mn₂",
    },
    {
      name: "Normal shock",
      expr: "p₂/p₁ = 1 + 2γ/(γ+1)(M²−1)    M₂² = (M² + 2/(γ−1)) / (2γ M²/(γ−1) − 1)",
    },
    {
      name: "Rayleigh (heat)",
      expr: "Tt/Tt* = 2(1+γ) M² (1 + ½(γ−1)M²) / (1 + γ M²)²",
    },
    {
      name: "Isentropic nozzle",
      expr: "A/A* = (1/M) [(1 + ½(γ−1)M²) / (½(γ+1))]^((γ+1)/2(γ−1))",
    },
    {
      name: "Thrust",
      expr: "F = ṁ (Ue − U₀) + (pe − p₀) Ae     Isp = F / (ṁ_f g₀)",
    },
    {
      name: "Kantrowitz",
      expr: "(A_t/A_c)_min = (A/A*)_{M₂,NS} / (A/A*)_{M₀}",
    },
    {
      name: "Waltrup–Billig isolator",
      expr: "L/H = √(θ/H) [50(p_r−1) + 170(p_r−1)²] / (M²−1)",
    },
  ];

  if (!attached) notes.push("Use a weaker ramp or higher Mach — the inlet shock is detached.");

  return {
    kind: inletOnly ? "inlet" : scram ? "scramjet" : "ramjet",
    stations,
    shocks,
    capture,
    mdot,
    mdotFuel,
    thrust,
    isp,
    tsfc,
    Tt0,
    pt0,
    Tt4,
    Te,
    Ue,
    Ue0: U0,
    M2: M,
    p2p0: p / p0,
    piD,
    started,
    kantrowitz: K,
    contraction,
    isolatorLH,
    etaThermal: clamp(eta, 0, 0.75),
    phi,
    fuel,
    erGeom: ER,
    equations,
    notes,
  };
}
