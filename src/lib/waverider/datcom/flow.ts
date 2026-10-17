/**
 * Flow conditions (US76 atmosphere reuse) and flat-plate skin friction.
 *
 * Skin friction: Eckert reference-temperature method (Eckert 1955; Sommer & Short 1955
 * for the turbulent reference temperature) applied to Prandtl–Schlichting turbulent
 * (Cf = 0.455/(log10 Re)^2.58, Schlichting, Boundary-Layer Theory) and Blasius laminar
 * (Cf = 1.328/sqrt(Re)) flat-plate laws; transition blend per Schlichting
 * (Cf = Cf_turb(Re) - (Re_tr/Re)(Cf_turb(Re_tr) - Cf_lam(Re_tr))); roughness cut-off
 * Reynolds number per Raymer (Aircraft Design: A Conceptual Approach), Eq. 12.28 form.
 */
import { atmosphere } from "../atmosphere.ts";
import type { FlightSpec, RefQuantities } from "./types.ts";
import { GAMMA_AIR } from "./gasdyn.ts";

export interface Flow {
  mach: number;
  gamma: number;
  altKm: number;
  T: number;
  p: number;
  rho: number;
  a: number;
  mu: number;
  V: number;
  q: number;
  renPerM: number;
  /** sqrt(|1 - M^2|) */
  beta: number;
}

export function sutherland(T: number): number {
  return (1.716e-5 * Math.pow(T / 273.15, 1.5) * (273.15 + 110.4)) / (T + 110.4);
}

function pick(v: number | number[] | undefined, i: number, def: number): number {
  if (v === undefined) return def;
  if (Array.isArray(v)) return v.length ? v[Math.min(i, v.length - 1)] : def;
  return v;
}

export function flowConditions(f: FlightSpec, i: number): Flow {
  const mach = f.machs[i];
  const gamma = f.gamma ?? GAMMA_AIR;
  const altKm = pick(f.altKm, i, 0);
  const atm = atmosphere(altKm, mach, gamma);
  const ren = f.renPerM !== undefined ? pick(f.renPerM, i, atm.ReL) : atm.ReL;
  return {
    mach,
    gamma,
    altKm,
    T: atm.T,
    p: atm.p,
    rho: atm.rho,
    a: atm.a,
    mu: atm.mu,
    V: atm.V,
    q: atm.q,
    renPerM: ren,
    beta: Math.sqrt(Math.abs(1 - mach * mach)),
  };
}

function cfTurbInc(Re: number): number {
  return 0.455 / Math.pow(Math.log10(Math.max(Re, 1e3)), 2.58);
}
function cfLamInc(Re: number): number {
  return 1.328 / Math.sqrt(Math.max(Re, 1));
}

export interface CfOptions {
  /** wall-to-adiabatic-wall temperature ratio, default 0.8 */
  wallTempRatio?: number;
  blayer?: "turb" | "natural";
  reTransition?: number;
  /** equivalent sand roughness [m] */
  roughness?: number;
}

/**
 * Mean flat-plate friction coefficient (based on q_inf, wetted area) for a plate of length L.
 */
export function skinFrictionCf(flow: Flow, L: number, o: CfOptions = {}): number {
  const M = flow.mach;
  const g = flow.gamma;
  const Te = flow.T;
  const ReL = Math.max(flow.renPerM * L, 1e3);
  const rT = Math.cbrt(0.72); // turbulent recovery factor ~0.896
  const rL = Math.sqrt(0.72); // laminar ~0.849
  const wallRatio = o.wallTempRatio ?? 0.8;
  const mk = (turb: boolean, Re: number) => {
    const r = turb ? rT : rL;
    const Taw = Te * (1 + r * ((g - 1) / 2) * M * M);
    const Tw = Math.max(0.3 * Te, wallRatio * Taw);
    // Eckert (laminar) / Sommer-Short-type reference temperature
    const Tstar = turb
      ? Te * (1 + 0.035 * M * M + 0.45 * (Tw / Te - 1))
      : Te * (0.5 + 0.039 * M * M + 0.5 * (Tw / Te));
    const rhoRatio = Te / Tstar;
    const muRatio = sutherland(Tstar) / sutherland(Te);
    // Re* = rho* V L / mu*  = Re * (rho*/rho) / (mu*/mu)
    const ReS = Re * (rhoRatio / muRatio);
    return { rhoRatio, ReS };
  };
  const turbCf = (Re: number) => {
    const { rhoRatio, ReS } = mk(true, Re);
    let Reeff = ReS;
    if (o.roughness && o.roughness > 0) {
      const cut = M < 1 ? 38.21 * Math.pow(L / o.roughness, 1.053) : 44.62 * Math.pow(L / o.roughness, 1.053) * Math.pow(M, 1.16);
      Reeff = Math.min(ReS, cut);
    }
    return cfTurbInc(Reeff) * rhoRatio;
  };
  const lamCf = (Re: number) => {
    const { rhoRatio, ReS } = mk(false, Re);
    return cfLamInc(ReS) * rhoRatio;
  };
  if (o.blayer === "natural") {
    const Retr = Math.min(o.reTransition ?? 5e5, ReL);
    if (Retr >= ReL) return lamCf(ReL);
    return Math.max(turbCf(ReL) - (Retr / ReL) * (turbCf(Retr) - lamCf(Retr)), lamCf(ReL) * 0.5);
  }
  return turbCf(ReL);
}

export function cfOptions(ref: RefQuantities): CfOptions {
  return { wallTempRatio: ref.wallTempRatio, blayer: ref.blayer, reTransition: ref.reTransition, roughness: ref.roughness };
}
