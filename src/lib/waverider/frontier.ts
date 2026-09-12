/**
 * Frontier under-CFD physics assembled from the flight state.
 * Papers: Fay–Riddell 1958, Billig 1967, Hayes–Probstein, Hansen SP-3013,
 * Millikan–White 1963 / Park 1990, Lighthill IDG, Lees 1956, Waltrup–Billig 1973,
 * Beckwith–Gallagher, Cheng rarefaction, Edney Type IV, Sänger glide, Van Dyke.
 */
import type { DesignParams } from "./types";
import type { Atmosphere } from "./atmosphere";
import type { PanelAero } from "./panel";
import type { MassProps } from "./mass";
import type { TrajResult } from "./trajectory";
import type { PropResult } from "./propulsion";
import {
  DEG,
  airDissoc,
  billigStandoff,
  binaryScale,
  damkohlerVib,
  detraKempRiddell,
  fayRiddell,
  flowRegime,
  gammaEffective,
  knudsen,
  meanFreePath,
  millikanWhiteTau,
  postShockT,
  suttonGraves,
  sweepHeatFactor,
  tauberSutton,
  twEquilibrium,
  vanDykeK,
  viscousChi,
  viscousInteractionPressure,
  waltrupBillig,
} from "./math";

export interface FrontierResult {
  kn: number;
  lambda: number;
  regime: ReturnType<typeof flowRegime>;
  gammaEq: number;
  gammaEff: number;
  T2: number;
  chiBar: number;
  pVisc: number;
  qFay: number;
  qSG: number;
  qDkr: number;
  qRad: number;
  twFay: number;
  sweepDeg: number;
  sweepFactor: number;
  qEdney: number;
  edney: boolean;
  clEqGlide: number;
  clAvail: number;
  billigDelta: number;
  daVib: number;
  tauVib: number;
  alphaO2: number;
  alphaN2: number;
  gammaReal: number;
  rhoL: number;
  vanDykeK: number;
  isolatorLH: number;
  maxN: number;
  twPeak: number;
  heatLoad: number;
  notes: string[];
}

export function frontierPhysics(
  params: DesignParams,
  atm: Atmosphere,
  aero: PanelAero,
  mass: MassProps,
  traj: TrajResult,
  prop: PropResult | null = null,
): FrontierResult {
  const notes: string[] = [];
  const M = Math.max(1.05, params.lockFlight ? params.mach : params.flightMach);
  const L = Math.max(params.length, 1e-6);
  const Rn = Math.max(params.leRadius, 0.0015 * L);
  const Tw = Math.max(200, params.twK);
  const lambda = meanFreePath(atm.T, atm.p);
  const kn = knudsen(atm.T, atm.p, L);
  const regime = flowRegime(kn);
  const ps = postShockT(M, atm.T, params.gamma);
  const chiBar = viscousChi(M, Math.max(atm.ReL * L, 1), Tw, atm.T);
  const pVisc = viscousInteractionPressure(chiBar);
  const recov = 0.5;
  const qSG = suttonGraves(atm.rho, atm.V, Rn, recov);
  const qFay = fayRiddell(atm.rho, atm.V, Rn, atm.T, atm.p, Tw, params.gamma);
  const qDkr = detraKempRiddell(atm.rho, atm.V, Rn, recov);
  const qRad = tauberSutton(atm.rho, atm.V, Rn);
  const s = Math.max(params.span / 2, 1e-6);
  const sweep = Math.atan2(L, s);
  const sweepFactor = sweepHeatFactor(sweep);
  const edney = params.family === "ramjet" || params.family === "scramjet" || params.family === "integrated";
  const qEdney = edney ? 8 * Math.max(qFay, qSG) : 0;
  const g0 = 9.80665;
  const Re = 6371000;
  const q = Math.max(atm.q, 1);
  const W = Math.max(mass.mass, 5) * g0 * (Re / (Re + atm.h)) ** 2;
  const clEq = (W * (1 - (atm.V * atm.V) / (g0 * (Re + atm.h)))) / (q * Math.max(aero.sRef, 1e-8));

  const rho2 = atm.rho * ps.p2p1 * (atm.T / Math.max(ps.T2, 1));
  const p2 = atm.p * ps.p2p1;
  const diss = airDissoc(ps.T2, rho2);
  const daVib = damkohlerVib(ps.T2, p2, L, atm.V);
  const tauVib = millikanWhiteTau(ps.T2, p2);
  const gammaEff = gammaEffective(ps.T2, p2, L, atm.V);
  const billigDelta = billigStandoff(M);
  const rhoL = binaryScale(atm.rho, L);
  const kSim = vanDykeK(M, params.height / L);
  const isolatorLH = prop?.internal?.isolatorLH ?? (edney ? waltrupBillig(Math.max(M * 0.45, 1.2), 2.2, 0.012) : 0);

  notes.push(
    `Air γ_vib(T₂=${ps.T2.toFixed(0)} K) = ${ps.gammaEq.toFixed(3)}; Da_vib = ${daVib.toExponential(2)} (Millikan–White / Park). γ_eff = ${gammaEff.toFixed(3)} (frozen→eq). Inverse-design shocks stay at γ=${params.gamma}.`,
  );
  notes.push(
    `Kn = ${kn.toExponential(2)} (${regime}). Continuum < 0.01, slip < 0.1, free-molecular > 10 (Bird / Schaaf–Chambre).`,
  );
  notes.push(
    `Hayes–Probstein χ̄ = ${chiBar.toFixed(2)} → p/p_inv = ${pVisc.toFixed(3)} on the windward BL (weak interaction if χ̄≲3).`,
  );
  notes.push(
    `Fay–Riddell (Billig Δ/Rn = ${billigDelta.toFixed(3)}) q_s = ${qFay.toFixed(2)} W/cm² vs Sutton–Graves ${qSG.toFixed(2)}.`,
  );
  notes.push(
    `Lighthill IDG: α_O₂ = ${diss.alphaO2.toFixed(3)}, α_N₂ = ${diss.alphaN2.toFixed(3)} → γ_real = ${diss.gamma.toFixed(3)}. ρL = ${rhoL.toExponential(2)} kg/m².`,
  );
  notes.push(`Van Dyke K = M τ = ${kSim.toFixed(2)} (τ = h/L).`);
  if (edney) {
    notes.push(
      `Edney Type-IV bound at the cowl lip ≈ ${qEdney.toFixed(0)} W/cm² (8× stag.). Waltrup–Billig isolator L/H ≈ ${isolatorLH.toFixed(1)}.`,
    );
  }
  if (regime !== "continuum") {
    notes.push("Rarefaction bridging is on: panel Cp is mixed toward free-molecular Newtonian.");
  }
  if (ps.T2 > 2500) {
    notes.push("T₂ > 2500 K: O₂ dissociation is on in γ_real. Treat γ_vib as a lower bound on the drop.");
  }
  if (daVib < 0.3) notes.push("Da_vib < 0.3: vibrationally frozen. Use γ=1.4 on the shock, not γ_eq.");
  else if (daVib > 5) notes.push("Da_vib > 5: vibrational equilibrium is a fair assumption behind the shock.");
  if (!traj.skipped && traj.notes[0]) {
    notes.push(
      `3DOF adaptive RK4. Peak n = ${traj.maxN.toFixed(2)} g, Tw lump ${traj.twPeak.toFixed(0)} K, heat load ${traj.heatLoad.toFixed(0)} J/cm².`,
    );
  }

  return {
    kn,
    lambda,
    regime,
    gammaEq: ps.gammaEq,
    gammaEff,
    T2: ps.T2,
    chiBar,
    pVisc,
    qFay,
    qSG,
    qDkr,
    qRad,
    twFay: twEquilibrium(qFay),
    sweepDeg: sweep / DEG,
    sweepFactor,
    qEdney,
    edney,
    clEqGlide: clEq,
    clAvail: aero.cl,
    billigDelta,
    daVib,
    tauVib,
    alphaO2: diss.alphaO2,
    alphaN2: diss.alphaN2,
    gammaReal: diss.gamma,
    rhoL,
    vanDykeK: kSim,
    isolatorLH,
    maxN: traj.maxN,
    twPeak: traj.twPeak,
    heatLoad: traj.heatLoad,
    notes,
  };
}