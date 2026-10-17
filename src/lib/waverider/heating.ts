/**
 * Per-panel convective heating shared by the local-inclination panel loop
 * (panel.ts) and the shock-expansion march (shockexp.ts).
 *
 * Windward: 0.5 Eckert/Zoby + 0.3 Tauber running-length + 0.2 Lees-scaled stagnation,
 * all on the post-shock edge state. Leading edge: swept Lees-scaled stagnation.
 * Leeward / base: small residual fractions. Units: W/cm². All of the blend weights
 * are engineering choices, not published constants.
 */
import type { DesignParams } from "./types";
import {
  eckertTstar,
  skinCf,
  vanDriestII,
  eckertStanton,
  airMu,
  clamp,
  cpGas,
  hypersonicTripped,
  leesHeatFactor,
  stantonToCf,
  tauberLaminar,
  tauberTurbulent,
  zobyHeatWcm2,
} from "./math";

export type HeatKind = "leading" | "base" | "windward" | "leeward";

export interface HeatCtx {
  /** freestream static pressure, Pa */
  pInf: number;
  gamma: number;
  /** wall temperature, K */
  Tw: number;
  /** effective nose / LE radius, m */
  Rn: number;
  cpMax: number;
  /** ½ γ M² */
  qDyn: number;
  /** stagnation heating used for LE scaling, W/cm² */
  qStag0: number;
  /** swept-LE factor (Beckwith–Gallagher) */
  sweepF: number;
}

export interface HeatIn {
  kind: HeatKind;
  cp: number;
  /** edge Mach, static T (K), density (kg/m³), velocity (m/s), pressure (Pa) */
  Me: number;
  Te: number;
  rhoe: number;
  Ue: number;
  pe: number;
  /** running length from the local leading edge, m */
  xRun: number;
  /** sin of the windward inclination (≥ 0) */
  sinth: number;
}

export interface HeatOut {
  q: number;
  St: number;
  cf: number;
  tripped: boolean;
  recov: number;
}

export function surfaceHeat(c: HeatCtx, i: HeatIn): HeatOut {
  const g = c.gamma;
  const cpg = cpGas(g);
  const hw = c.Tw * cpg;
  const mu = airMu(i.Te);
  const ReX = (i.rhoe * i.Ue * i.xRun) / Math.max(mu, 1e-10);
  const tripped = hypersonicTripped(ReX, i.Me);
  const rRec = tripped ? 0.89 : Math.sqrt(0.71);
  const hrec = i.Te * cpg * (1 + rRec * 0.5 * (g - 1) * i.Me * i.Me);
  const recov = clamp(1 - hw / Math.max(hrec, 1), 0.05, 0.95);
  const pRatio = clamp(i.cp / Math.max(c.cpMax, 1e-6), 0.01, 1);
  const ReS = (Math.max(i.rhoe, 1e-12) * Math.max(i.Ue, 1) * Math.max(i.xRun, 1e-6)) / Math.max(mu, 1e-10);
  const St = eckertStanton(ReS, i.Te, c.Tw, i.Me, g, tripped);
  // Cap: near-vacuum cells (Re_x clamp) otherwise report Cf ~ 0.1 which is meaningless.
  const cf = Math.min(stantonToCf(St, tripped), 0.02);
  let q: number;
  if (i.kind === "leading") {
    q = c.qStag0 * c.sweepF * leesHeatFactor(Math.max(pRatio, 0.25), 1, tripped);
  } else if (i.kind === "base") {
    q = 0.08 * c.qStag0 * Math.pow(Math.max(i.pe / Math.max(c.pInf * (1 + c.cpMax * c.qDyn), 1), 0.01), 0.8);
  } else {
    const qZ = zobyHeatWcm2(i.rhoe, i.Ue, i.Te, c.Tw, i.xRun, i.Me, g, tripped);
    if (i.kind === "windward") {
      const impactS = Math.max(i.sinth, 0.02);
      const qLam = tauberLaminar(i.rhoe, i.Ue, i.xRun, recov, impactS);
      const qTurb = tauberTurbulent(i.rhoe, i.Ue, i.xRun, recov, impactS);
      const qW0 = tripped ? Math.max(qLam, qTurb) : qLam;
      const qLees = c.qStag0 * leesHeatFactor(pRatio, i.xRun / Math.max(c.Rn, 1e-8), tripped);
      q = 0.5 * qZ + 0.3 * qW0 + 0.2 * qLees;
    } else {
      // Prandtl–Meyer edge is cold; keep a small residual so the lid isn't a hole.
      q = Math.max(0.18 * qZ, 0.015 * c.qStag0 * Math.sqrt(Math.max(pRatio, 0.02)));
    }
  }
  return { q, St, cf, tripped, recov };
}

/** Planform LE x so running length is s = x − x_LE(y), not just x (outboard LE was too cold). */
export function xLeadingOf(params: DesignParams, y: number): number {
  const L = params.length;
  const s = Math.max(params.span / 2, 1e-9);
  const yn = clamp(Math.abs(y) / s, 0, 1);
  if (params.family === "liftbody" || params.family === "star") return 0;
  if (params.family === "caret") return L * yn;
  if (params.family === "cone" || params.family === "busemann") {
    const nose = clamp(params.captureFrac, 0.04, 0.45);
    const blunt = 1 - Math.sqrt(Math.max(0, 1 - yn * yn));
    return L * (nose * blunt * 0.65 + yn * 0.35);
  }
  const planform = params.planform;
  if (planform === "rect") return 0;
  if (planform === "spatular") {
    const nose = clamp(params.captureFrac, 0.04, 0.45);
    const blunt = 1 - Math.sqrt(Math.max(0, 1 - yn * yn));
    return L * (nose * blunt * 0.65 + yn * 0.35);
  }
  if (planform === "double") {
    const kink = 0.42;
    if (yn < kink) return L * 0.1 * (yn / kink);
    return L * (0.1 + 0.9 * ((yn - kink) / (1 - kink)) ** 1.05);
  }
  const pow = planform === "delta" ? 1 : clamp(params.planformPower, 0.6, 2.4);
  return L * yn ** pow;
}


/**
 * Whole-body mean flat-plate skin-friction coefficient (based on freestream q and wetted area).
 * Laminar: Blasius × Eckert √C* (Chapman–Rubesin). Turbulent: van Driest II.
 * Transition at Re_L = 5×10⁵ uses Schlichting's −1700/Re correction so Cf is
 * continuous (the old switch jumped 0.0019 → 0.0053 at 5×10⁵).
 */
export function bodyCf(ReL: number, M: number, T: number, Tw: number, gamma: number): number {
  const Re = Math.max(ReL, 100);
  const Ts = eckertTstar(T, Tw, M, gamma, Math.sqrt(0.71));
  const Cstar = (T / Math.max(Ts, 1)) * (airMu(Ts) / airMu(T));
  const lam = (1.328 * Math.sqrt(Math.min(Cstar, 1.2))) / Math.sqrt(Re);
  if (Re < 5e5) return lam;
  const turb = vanDriestII(Re, M, T, Tw, gamma);
  const ps = skinCf(Re);
  const corr = Math.max(0, 1 - 1700 / Re / Math.max(ps, 1e-9));
  const mixed = turb * corr;
  return Math.max(mixed, Math.min(lam, turb));
}
