import type { BuiltVehicle, DesignParams } from "./types";
import { domainOf } from "./types";
import { atmosphere, type Atmosphere } from "./atmosphere";
import { finiteStab, panelAero, type PanelAero } from "./panel";
import { ramjetCycle, type PropResult } from "./propulsion";
import { massProperties, type MassProps } from "./mass";
import { integrateGlide, type TrajResult } from "./trajectory";
import { runValidation, validationSummary, type Check } from "./validate";
import { solveSixDof, type SixDofResult } from "./sixdof";
import { frontierPhysics, type FrontierResult } from "./frontier";
import { effectiveLeRadius } from "./blunt";

export interface LitBand {
  ld: [number, number];
  note: string;
}

const LIT: Record<string, LitBand> = {
  caret: {
    ld: [3.2, 7.2],
    note: "Nonweiler inviscid L/D = cot θ. Viscous cruise 3.5–6 at M8 (Bowcutt, Corda).",
  },
  cone: {
    ld: [3.5, 7.5],
    note: "Jones–Moore–Pike cone-derived; spatular lids trade L/D for volume.",
  },
  osculating: {
    ld: [4.0, 8.0],
    note: "Sobieczky OSC / HyCAUSE-class viscous L/D ~ 4–7 at M6–8.",
  },
  viscopt: {
    ld: [4.5, 8.5],
    note: "Bowcutt viscous-optimized power-law; peak L/D slightly above caret at same M.",
  },
  inward: {
    ld: [3.0, 6.5],
    note: "REST scoop: L/D is secondary to contraction and startability.",
  },
  elliptic: {
    ld: [3.8, 7.5],
    note: "Rasmussen elliptic-cone: L/D between caret and circular cone at same volume.",
  },
  busemann: {
    ld: [2.5, 5.5],
    note: "Busemann inlet: Kantrowitz start and contraction, not cruise L/D.",
  },
  star: {
    ld: [1.8, 4.0],
    note: "Star bodies are volume/entry shapes (Kuchemann, JPL).",
  },
  wedgecone: {
    ld: [3.8, 7.2],
    note: "Wedge–cone hybrids sit between caret and OSC for L/D.",
  },
  liftbody: {
    ld: [1.2, 3.2],
    note: "HL-20 / HTV-class: L/D ~ 1.3–2.5 hypersonic (NASA).",
  },
  ramjet: {
    ld: [1.5, 4.0],
    note: "Ramjet OML is a propulsion body; use the Cycle tab for Isp / F.",
  },
  scramjet: {
    ld: [1.5, 4.2],
    note: "X-43 / X-51 class 2-D scram OML: cruise L/D ~ 2–4.",
  },
  integrated: {
    ld: [3.0, 6.5],
    note: "Waverider + cowl: L/D penalty vs clean OSC, gain in Isp.",
  },
};

export interface Bench {
  wedgeCot: number;
  panelLd: number;
  ratio: number;
  inLitBand: boolean;
  note: string;
}

export interface StudyResult {
  atm: Atmosphere;
  aero: PanelAero;
  stab: ReturnType<typeof finiteStab>;
  six: SixDofResult;
  prop: PropResult | null;
  mass: MassProps;
  traj: TrajResult;
  frontier: FrontierResult;
  bench: Bench;
  literature: LitBand & { inBand: boolean; source: string };
  checks: Check[];
  checksOk: boolean;
  elapsedMs: number;
  flightMach: number;
  domain: "external" | "internal";
}

export function flightMachOf(p: DesignParams) {
  return p.lockFlight ? p.mach : p.flightMach;
}

export function studyVehicle(built: BuiltVehicle): StudyResult {
  const t0 = performance.now?.() ?? Date.now();
  const p = built.params;
  const domain = domainOf(p.family);
  const Mf = flightMachOf(p);
  const atm = atmosphere(p.altKm, Mf, p.gamma);
  const sRef = Math.max(built.aero.planformArea, 1e-6);
  const lRef = p.length;
  const full = domain === "external" || p.family === "integrated";
  const aero = panelAero(built.mesh, p, atm, p.alphaDeg, p.betaDeg, sRef, lRef);
  const mass = massProperties(built.mesh, p.rhoKgM3 || 160, p.massKg || 0);
  const six = solveSixDof(built.mesh, p, atm, mass, sRef, lRef, full);
  const stab = {
    cla: six.derivs.cla,
    cma: six.derivs.cma,
    cnb: six.derivs.cnb,
    cyb: six.derivs.cyb,
    clb: six.derivs.clb,
    staticMargin: six.derivs.staticMargin,
    staticMarginPct: six.derivs.staticMarginPct,
    trimAlpha: six.derivs.trimAlpha,
    trimCm: six.derivs.trimCm,
    longitudinallyStable: six.derivs.longitudinallyStable,
    directionallyStable: six.derivs.directionallyStable,
    polar: six.polar,
    atAlpha: six.atAlpha,
  };
  const prop = ramjetCycle(p, atm);
  mass.ballistic = aero.cd > 1e-8 ? mass.mass / (aero.cd * sRef) : 0;
  const Rn = Math.max(effectiveLeRadius(p), 0.0015 * p.length);
  const traj = full
    ? integrateGlide(p, stab.polar, mass.mass, sRef, Rn)
    : {
        samples: [] as TrajResult["samples"],
        rangeKm: 0,
        timeS: 0,
        maxQ: atm.q,
        maxQAltKm: p.altKm,
        maxHeat: aero.qStag,
        maxHeatAltKm: p.altKm,
        maxN: 0,
        heatLoad: 0,
        twPeak: 0,
        finalV: atm.V,
        finalH: p.altKm,
        skipped: true,
        notes: ["Trajectory is an external-flow tool — switch to External to glide."],
      };
  const lit = LIT[p.family] ?? LIT.caret;
  const ld = aero.ld;
  const inBand = ld >= lit.ld[0] * 0.7 && ld <= lit.ld[1] * 1.35;
  const th = Math.max(1e-3, built.aero.thetaDeg * (Math.PI / 180));
  const cot = 1 / Math.tan(th);
  const bench: Bench = {
    wedgeCot: cot,
    panelLd: ld,
    ratio: cot > 1e-6 ? ld / cot : 0,
    inLitBand: inBand,
    note:
      p.family === "caret"
        ? `Exact inviscid 2-D wedge L/D = cot θ = ${cot.toFixed(2)}. Mixed panel with friction should sit at ~0.5–0.8 of that (Bowcutt).`
        : `Generating-wedge cot θ = ${cot.toFixed(2)} is a ceiling, not a target, for this family.`,
  };
  const checks = runValidation();
  const { ok } = validationSummary(checks);
  const elapsedMs = (performance.now?.() ?? Date.now()) - t0;
  const frontier = frontierPhysics(p, atm, aero, mass, traj, prop);
  return {
    atm,
    aero,
    stab,
    six,
    prop,
    mass,
    traj,
    frontier,
    bench,
    literature: { ...lit, inBand, source: p.family },
    checks,
    checksOk: ok,
    elapsedMs,
    flightMach: Mf,
    domain,
  };
}

export function studyJson(built: BuiltVehicle, study: StudyResult): string {
  const { atm, aero, stab, six, prop, mass, traj, frontier, bench, literature, checks, elapsedMs, flightMach, domain } = study;
  return JSON.stringify(
    {
      name: built.params.name,
      family: built.params.family,
      domain,
      designMach: built.params.mach,
      flightMach,
      alphaDeg: built.params.alphaDeg,
      altKm: built.params.altKm,
      method: built.params.aeroMethod,
      origin: [0, 0, 0],
      frame: "X streamwise (tip at origin), Y span, Z up",
      atmosphere: { T: atm.T, p: atm.p, rho: atm.rho, q: atm.q, V: atm.V, ReL: atm.ReL },
      aero: {
        cl: aero.cl,
        cd: aero.cd,
        ld: aero.ld,
        cm: aero.cm,
        cdWave: aero.cdWave,
        cdFric: aero.cdFric,
        cdBase: aero.cdBase,
        qStag: aero.qStag,
        qFay: aero.qFay,
        qSG: aero.qSG,
        qMax: aero.qMax,
        qRad: aero.qRad,
        kn: aero.kn,
        chiBar: aero.chiBar,
        gammaEq: aero.gammaEq,
        regime: aero.regime,
      },
      stability: {
        cla: stab.cla,
        cma: stab.cma,
        cnb: stab.cnb,
        clb: stab.clb,
        cmq: six.derivs.cmq,
        clp: six.derivs.clp,
        cnr: six.derivs.cnr,
        staticMarginPct: stab.staticMarginPct,
        trimAlpha: stab.trimAlpha,
        longitudinallyStable: stab.longitudinallyStable,
        modes: six.modes.map((m) => ({
          name: m.name,
          wn: m.wn,
          zeta: m.zeta,
          period: m.period,
          stable: m.stable,
        })),
        sixDof: {
          method: six.method,
          iterations: six.iterations,
          dt: six.dt,
          skipped: six.skipped,
        },
      },
      mass: { kg: mass.mass, cg: mass.cg, Iyy: mass.Iyy, ballistic: mass.ballistic },
      trajectory: {
        rangeKm: traj.rangeKm,
        timeS: traj.timeS,
        maxQ: traj.maxQ,
        maxHeat: traj.maxHeat,
        maxHeatAltKm: traj.maxHeatAltKm,
        maxN: traj.maxN,
        heatLoad: traj.heatLoad,
        twPeak: traj.twPeak,
      },
      frontier: {
        kn: frontier.kn,
        regime: frontier.regime,
        gammaEq: frontier.gammaEq,
        gammaEff: frontier.gammaEff,
        T2: frontier.T2,
        chiBar: frontier.chiBar,
        pVisc: frontier.pVisc,
        qFay: frontier.qFay,
        qSG: frontier.qSG,
        qDkr: frontier.qDkr,
        billigDelta: frontier.billigDelta,
        daVib: frontier.daVib,
        alphaO2: frontier.alphaO2,
        alphaN2: frontier.alphaN2,
        gammaReal: frontier.gammaReal,
        rhoL: frontier.rhoL,
        vanDykeK: frontier.vanDykeK,
        isolatorLH: frontier.isolatorLH,
        sweepDeg: frontier.sweepDeg,
        clEqGlide: frontier.clEqGlide,
        notes: frontier.notes,
      },
      bench,
      propulsion: prop
        ? {
            thrust: prop.thrust,
            isp: prop.isp,
            mdot: prop.mdot,
            Tt4: prop.Tt4,
            kind: prop.kind,
            piD: prop.internal?.piD,
            started: prop.internal?.started,
            stations: prop.internal?.stations,
            shocks: prop.internal?.shocks,
          }
        : null,
      literature,
      checks: checks.map((c) => ({ id: c.id, pass: c.pass, relErr: c.relErr, got: c.got, expected: c.expected })),
      elapsedMs,
      mesh: { triangles: built.quality.triangles, watertight: built.quality.watertight, volume: built.quality.volume },
    },
    null,
    2,
  );
}
