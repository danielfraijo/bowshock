/**
 * Configuration synthesis: body + lifting-surface sets + interference -> forces, moments, derivatives.
 * See the module headers of body.ts / fin.ts / interference.ts for the method references.
 *
 * Panel geometry convention and sign conventions: see types.ts.
 */
import type {
  CaseResult,
  Component,
  Confidence,
  Derivs,
  FlightSpec,
  GeometrySummary,
  PointResult,
  RunResult,
  Validity,
  VehicleConfig,
} from "./types.ts";
import { buildBody, bodyIntegrals, finGeom, resolveRefs, summarizeGeometry, type BodyIntegrals, type BodyProfile, type FinGeom } from "./geometry.ts";
import { cfOptions, flowConditions, type Flow } from "./flow.ts";
import { bodyAxial, bodyDamping, bodyNormal, type BodyAxial, type BodyCtx } from "./body.ts";
import { finAxial, finCLa, finXcp, flapEffectiveness, panelNormal, panelRollDamping, type FinAxial } from "./fin.ts";
import { inducedVelocity, interferenceFactors, makeVortex, type KFactors, type Vortex } from "./interference.ts";
import { ellipticIntegrals, ellipticToProfile, inletIncrements, isElliptic, protuberanceIncrements } from "./extras.ts";
import { sanitizeConfig, sanitizeFlight } from "./validate-input.ts";
import { DEG, RAD, clamp, smoothstep } from "./util.ts";

export interface Model {
  cfg: VehicleConfig;
  prof: BodyProfile | null;
  ints: BodyIntegrals | null;
  fins: FinGeom[];
  sRef: number;
  lRef: number;
  latRef: number;
  xcg: number;
  geometry: GeometrySummary;
}

export function buildModel(cfgIn: VehicleConfig): Model {
  const cfg = sanitizeConfig(cfgIn);
  let prof = null as ReturnType<typeof buildBody> | null;
  let ints = null as ReturnType<typeof bodyIntegrals> | null;
  if (cfg.body) {
    if (isElliptic(cfg.body) && cfg.body.elliptic) {
      prof = ellipticToProfile(cfg.body.elliptic);
      const ei = ellipticIntegrals(cfg.body.elliptic);
      ints = {
        volume: ei.volume,
        planform: ei.planform,
        wetted: ei.wetted,
        baseArea: ei.baseArea,
        xPlanform: ei.xPlanform,
        xVolume: ei.xVolume,
      };
      prof.warnings.push(`elliptic eccentricity path active (max e≈ engineering)`);
    } else {
      prof = buildBody(cfg.body);
      ints = bodyIntegrals(prof);
    }
  }
  const fins = cfg.sets.map((s) => finGeom(s, prof));
  // drop buried panels softly
  for (const f of fins) {
    if (f.exposedSpan < 1e-8) {
      f.exposedSpan = 1e-8;
      f.area = Math.max(f.area, 1e-12);
      f.aspect = Math.max(f.aspect, 1e-4);
    }
  }
  const refs = resolveRefs(cfg, prof, fins);
  const sRef = Math.max(refs.sRef, 1e-12);
  const lRef = Math.max(refs.lRef, 1e-12);
  const latRef = Math.max(refs.latRef, 1e-12);
  const xcg = Number.isFinite(refs.xcg) ? refs.xcg : 0;
  const geometry = summarizeGeometry(cfg, prof, fins);
  geometry.sRef = sRef;
  geometry.lRef = lRef;
  geometry.latRef = latRef;
  geometry.xcg = xcg;
  return { cfg, prof, ints, fins, sRef, lRef, latRef, xcg, geometry };
}

interface SetState {
  g: FinGeom;
  K: KFactors;
  xcp: number;
  ax: FinAxial;
  cla: number;
  tauFlap: number;
  sucFrac: number;
  slope: number; // CLa * Se/Sref * (KWB+KBW) per panel, Sref basis
}

export interface MachState {
  m: Model;
  flow: Flow;
  bodyCtx: BodyCtx | null;
  bodyAx: BodyAxial | null;
  sets: SetState[];
  decay: number;
}

export function machState(m: Model, flow: Flow): MachState {
  const cfg = m.cfg;
  let bodyCtx: BodyCtx | null = null;
  let bodyAx: BodyAxial | null = null;
  if (m.prof && m.ints) {
    bodyCtx = {
      prof: m.prof,
      ints: m.ints,
      sRef: m.sRef,
      lRef: m.lRef,
      latRef: m.latRef,
      xcg: m.xcg,
      flow,
      method: cfg.bodyMethod ?? "auto",
      cf: cfOptions(cfg.ref),
      exitDiameter: cfg.body?.exitDiameter,
      baseDrag: cfg.body?.baseDrag,
    };
    bodyAx = bodyAxial(bodyCtx);
  }
  const sets: SetState[] = m.fins.map((g) => {
    const K = interferenceFactors(g, flow.mach);
    const cla = finCLa(g, flow.mach).CLa;
    const ax = finAxial(g, flow, cfOptions(cfg.ref));
    const e = 0.9;
    const suc = clamp(1 - cla / (Math.PI * Math.max(g.aspect, 1e-3) * e), 0, 1) * (1 - smoothstep(0.9, 1.4, flow.mach));
    return {
      g,
      K,
      xcp: finXcp(g, flow.mach),
      ax,
      cla,
      tauFlap: g.set.cfoc !== undefined ? flapEffectiveness(g.set.cfoc) : 1,
      sucFrac: suc,
      slope: cla * (g.area / m.sRef) * (K.KWB + K.KBW),
    };
  });
  return { m, flow, bodyCtx, bodyAx, sets, decay: cfg.vortexDecaySemispans ?? 30 };
}

export interface RawForces {
  CN: number;
  CY: number;
  CA: number;
  Cm: number;
  Cn: number;
  Cl: number;
  comps: Component[];
  bodyTag: string;
  bodyNotes: string[];
}

/** Evaluate forces at total angle of attack aT [rad], cross-flow azimuth psi [rad]; optional deflection override per set. */
export function forces(ms: MachState, aT: number, psi: number, dOverride?: (setIdx: number, panel: number, phiF: number) => number | undefined): RawForces {
  const { m, flow } = ms;
  const M = flow.mach;
  const sRef = m.sRef;
  const lRef = m.lRef;
  const bRef = m.latRef;
  const xcg = m.xcg;
  const comps: Component[] = [];
  let CN = 0;
  let CY = 0;
  let CA = 0;
  let Cm = 0;
  let Cn = 0;
  let Cl = 0;
  let bodyTag = "none";
  let bodyNotes: string[] = [];
  const cosPsi = Math.cos(psi);
  const sinPsi = Math.sin(psi);
  const cosA = Math.cos(aT);
  if (ms.bodyCtx && ms.bodyAx) {
    const bn = bodyNormal(ms.bodyCtx, aT);
    const cnB = bn.N * cosPsi;
    const cyB = bn.N * sinPsi;
    const arm = xcg * bn.N - bn.Nx;
    const caB = (ms.bodyAx.CAwave + ms.bodyAx.CAfric) * cosA + ms.bodyAx.CAbase + bn.dCA;
    const c: Component = { name: "body", CN: cnB, CY: cyB, CA: caB, Cm: (arm * cosPsi) / lRef, Cn: (arm * sinPsi) / bRef, Cl: 0 };
    comps.push(c);
    CN += c.CN;
    CY += c.CY;
    CA += c.CA;
    Cm += c.Cm;
    Cn += c.Cn;
    bodyTag = bn.tag;
    bodyNotes = bn.notes;
  }
  // Inlet + protuberance engineering increments
  if (m.cfg.inlets?.length) {
    const inc = inletIncrements(m.cfg.inlets, flow, sRef, lRef, xcg);
    const c: Component = { name: "inlets", CN: inc.CN * cosPsi, CY: inc.CY, CA: inc.CA, Cm: inc.Cm * cosPsi, Cn: inc.Cn, Cl: 0 };
    comps.push(c);
    CN += c.CN; CY += c.CY; CA += c.CA; Cm += c.Cm; Cn += c.Cn;
    bodyNotes = [...bodyNotes, ...inc.notes];
  }
  if (m.cfg.protuberances?.length) {
    const inc = protuberanceIncrements(m.cfg.protuberances, flow, sRef, lRef, xcg, m.geometry.length);
    const c: Component = { name: "protuberances", CN: 0, CY: 0, CA: inc.CA, Cm: 0, Cn: 0, Cl: 0 };
    comps.push(c);
    CA += c.CA;
    bodyNotes = [...bodyNotes, ...inc.notes];
  }
  const vortices: Vortex[] = [];
  const g = flow.gamma;
  ms.sets.forEach((ss, si) => {
    const geo = ss.g;
    const set = geo.set;
    const comp: Component = { name: set.name, CN: 0, CY: 0, CA: 0, Cm: 0, Cn: 0, Cl: 0 };
    const newVort: Vortex[] = [];
    for (let i = 0; i < set.nPanels; i++) {
      const phiDeg = geo.panelPhi[i];
      const phi = phiDeg * DEG;
      const ov = dOverride ? dOverride(si, i, phiDeg) : undefined;
      const defl = (ov !== undefined ? ov : set.deflection?.[i] ?? 0) * DEG;
      // panel-local incidence from the crossflow (tan form), leeward shielding of the geometric part
      const sinRel = Math.sin(psi - phi);
      let aGeom = Math.atan(Math.tan(aT) * sinRel);
      const cl = Math.cos(psi - phi);
      if (cl > 0 && aT > 0) {
        const lee = 0.3 * cl * (aT / (aT + 8 * DEG));
        aGeom *= 1 - lee;
      }
      // downwash from upstream tip vortices at the panel control point
      let dAlpha = 0;
      if (vortices.length) {
        const rc = geo.rRoot + 0.5 * geo.exposedSpan;
        const up = rc * Math.cos(phi);
        const stb = rc * Math.sin(phi);
        const x = set.xLE + 0.5 * set.rootChord;
        const tUp = -Math.sin(phi);
        const tStb = Math.cos(phi);
        for (const v of vortices) {
          const vel = inducedVelocity(v, up, stb, x);
          dAlpha += vel.up * tUp + vel.stb * tStb;
        }
      }
      const aEff = aGeom - ss.tauFlap * defl + dAlpha;
      const Nalone = panelNormal(geo, M, aEff, g);
      const Fp = Nalone * (geo.area / sRef);
      const FW = Fp * ss.K.KWB;
      const FB = Fp * ss.K.KBW;
      const Ft = FW + FB;
      const up = -Ft * Math.sin(phi);
      const stb = Ft * Math.cos(phi);
      comp.CN += up;
      comp.CY += stb;
      comp.Cm += up * ((xcg - ss.xcp) / lRef);
      comp.Cn += stb * ((xcg - ss.xcp) / bRef);
      comp.Cl += (FW * geo.yBar) / bRef;
      const dCAind = -ss.sucFrac * FW * Math.sin(aEff);
      comp.CA += (ss.ax.cd0 * geo.area) / sRef * Math.cos(aT) + dCAind;
      const vx = makeVortex(geo, phiDeg, FW, sRef, ms.decay, Math.abs(aT));
      if (vx) newVort.push(vx);
    }
    vortices.push(...newVort);
    comps.push(comp);
    CN += comp.CN;
    CY += comp.CY;
    CA += comp.CA;
    Cm += comp.Cm;
    Cn += comp.Cn;
    Cl += comp.Cl;
  });
  const fin = (x: number) => (Number.isFinite(x) ? x : 0);
  return { CN: fin(CN), CY: fin(CY), CA: fin(CA), Cm: fin(Cm), Cn: fin(Cn), Cl: fin(Cl), comps, bodyTag, bodyNotes };
}

/** body-axis (alpha, beta) -> (total alpha, crossflow azimuth). */
export function toTotal(alpha: number, beta: number): { aT: number; psi: number } {
  const aT = Math.acos(clamp(Math.cos(alpha) * Math.cos(beta), -1, 1));
  const psi = Math.atan2(-Math.sin(beta), Math.sin(alpha) * Math.cos(beta));
  return { aT, psi };
}
/** (total alpha, aerodynamic roll phi) -> body (alpha, beta). */
export function fromTotalRoll(aT: number, phi: number): { alpha: number; beta: number } {
  const sinb = -Math.sin(aT) * Math.sin(phi);
  const beta = Math.asin(clamp(sinb, -1, 1));
  const alpha = Math.atan2(Math.sin(aT) * Math.cos(phi), Math.cos(aT));
  return { alpha, beta };
}

function evalBody(ms: MachState, alpha: number, beta: number, dOverride?: Parameters<typeof forces>[3]) {
  const { aT, psi } = toTotal(alpha, beta);
  return forces(ms, aT, psi, dOverride);
}

export function dynamicDerivs(ms: MachState): { Clp: number; Cmq: number; Cnr: number; CNq: number } {
  const m = ms.m;
  let Cmq = 0;
  let Cnr = 0;
  let Clp = 0;
  let CNq = 0;
  if (ms.bodyCtx && m.ints && m.prof) {
    Cmq += bodyDamping(ms.bodyCtx, m.lRef);
    Cnr += bodyDamping(ms.bodyCtx, m.latRef);
    const xb = m.prof.x[m.prof.x.length - 1];
    CNq += (4 * m.ints.baseArea * (xb - m.xcg)) / (m.sRef * m.lRef);
  }
  for (const ss of ms.sets) {
    const geo = ss.g;
    const l = ss.xcp - m.xcg;
    for (let i = 0; i < geo.set.nPanels; i++) {
      const phi = geo.panelPhi[i] * DEG;
      Cmq += -2 * ss.slope * Math.sin(phi) ** 2 * (l / m.lRef) ** 2;
      Cnr += -2 * ss.slope * Math.cos(phi) ** 2 * (l / m.latRef) ** 2;
      CNq += 2 * ss.slope * Math.sin(phi) ** 2 * (l / m.lRef);
      Clp += panelRollDamping(geo, ms.flow.mach, m.sRef, m.latRef);
    }
  }
  return { Clp, Cmq, Cnr, CNq };
}

export function assessValidity(ms: MachState, aT: number, body: boolean): Validity {
  const M = ms.flow.mach;
  const notes: string[] = [];
  let conf: Confidence = "high";
  let valid = true;
  const down = (c: Confidence) => {
    const order: Confidence[] = ["high", "medium", "low"];
    if (order.indexOf(c) > order.indexOf(conf)) conf = c;
  };
  const tags: string[] = [];
  if (M < 0.05) {
    valid = false;
    notes.push("Mach < 0.05: outside the compressible-flow-based methods");
  }
  if (M < 0.3) tags.push("incompressible");
  else if (M < 0.8) tags.push("subsonic");
  else if (M < 1.2) {
    tags.push("transonic");
    down("low");
    notes.push("Mach 0.8-1.2 transonic interpolation — low confidence");
  } else if (M < 5) tags.push("supersonic");
  else {
    tags.push("hypersonic");
    down("medium");
    notes.push("Mach > 5: perfect-gas Newtonian/shock-expansion limits; no real-gas or viscous interaction");
  }
  if (M > 10) {
    down("low");
    notes.push("Mach > 10: real-gas / viscous-interaction effects not modelled — low confidence");
  }
  if (M > 15) {
    valid = false;
    notes.push("Mach > 15: outside the supported range");
  }
  const aDeg = aT * RAD;
  if (aDeg > 15) {
    down("medium");
    notes.push("total angle of attack > 15 deg: non-linear crossflow / vortex terms are engineering approximations");
  }
  if (aDeg > 30) {
    down("low");
    notes.push("total angle of attack > 30 deg: low confidence");
  }
  if (aDeg > 60) {
    down("low");
    notes.push("total angle of attack > 60 deg: engineering extrapolation — low confidence");
  }
  if (aDeg > 120) {
    valid = false;
    notes.push("total angle of attack > 120 deg: outside the supported range");
  }
  if (ms.sets.length) {
    if (ms.sets.some((s) => s.g.aspect > 8)) {
      down("medium");
      notes.push("high-aspect-ratio panel: slender-body interference factors blended to strip theory");
    }
    if (ms.sets.length > 1) {
      down("medium");
      notes.push("multiple surface sets: single rolled-up vortex downwash model (no vortex decay beyond the configured length)");
    }
    if (M > 1.2 && ms.sets.some((s) => s.g.tanLE > 0 && s.g.tau > 0.5)) {
      down("medium");
      notes.push("large body-radius/semispan ratio: interference factors from slender-body theory only");
    }
  }
  if (body && ms.m.prof && ms.m.prof.length / (2 * ms.m.prof.rMax) < 3) {
    down("medium");
    notes.push("body fineness < 3: slender-body theory degraded");
  }
  const method = [
    tags.join("/"),
    body ? ms.bodyAx?.tag : null,
    ms.sets.length ? (M < 0.9 ? "DATCOM straight-taper CLa" : M >= 1.2 ? "linear supersonic + shock-expansion panels" : "transonic CLa interpolation") : null,
  ]
    .filter(Boolean)
    .join(" | ");
  return { confidence: conf, valid, method, notes };
}

const H = 0.5 * DEG;

export function evalPoint(ms: MachState, alphaDeg: number, betaDeg: number, phiDeg: number): PointResult {
  const m = ms.m;
  let alpha: number;
  let beta: number;
  let aTin: number;
  if (Math.abs(phiDeg) > 1e-9) {
    aTin = alphaDeg * DEG;
    const r = fromTotalRoll(aTin, phiDeg * DEG);
    alpha = r.alpha;
    beta = r.beta;
  } else {
    alpha = alphaDeg * DEG;
    beta = betaDeg * DEG;
    aTin = toTotal(alpha, beta).aT;
  }
  const f0 = evalBody(ms, alpha, beta);
  const fa1 = evalBody(ms, alpha + H, beta);
  const fa0 = evalBody(ms, alpha - H, beta);
  const fb1 = evalBody(ms, alpha, beta + H);
  const fb0 = evalBody(ms, alpha, beta - H);
  const d = (a: number, b: number) => (a - b) / (2 * H);
  const CNa = d(fa1.CN, fa0.CN);
  const Cma = d(fa1.Cm, fa0.Cm);
  const CYb = d(fb1.CY, fb0.CY);
  const Cnb = d(fb1.Cn, fb0.Cn);
  const Clb = d(fb1.Cl, fb0.Cl);
  const dyn = dynamicDerivs(ms);
  // control derivatives on the first set with a control (cfoc defined or all-moving) — set 0 by default
  let CNd = 0;
  let Cmd = 0;
  let Cld = 0;
  const ci = Math.max(
    0,
    ms.sets.findIndex((s) => s.g.set.role === "tail" || s.g.set.role === "canard" || s.g.set.role === "fin" || s.g.set.role === "htail"),
  );
  if (ms.sets.length) {
    const pitch = (sgn: number) => (si: number, _i: number, phiF: number) => {
      if (si !== ci) return undefined;
      const base = ms.sets[ci].g.set.deflection;
      void base;
      return sgn * 0.5 * Math.sin(phiF * DEG);
    };
    const roll = (sgn: number) => (si: number) => (si === ci ? sgn * 0.5 : undefined);
    const p1 = evalBody(ms, alpha, beta, pitch(1));
    const p0 = evalBody(ms, alpha, beta, pitch(-1));
    const r1 = evalBody(ms, alpha, beta, roll(1));
    const r0 = evalBody(ms, alpha, beta, roll(-1));
    CNd = (p1.CN - p0.CN) / (1 * DEG);
    Cmd = (p1.Cm - p0.Cm) / (1 * DEG);
    Cld = (r1.Cl - r0.Cl) / (1 * DEG);
  }
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);
  const cb = Math.cos(beta);
  const sb = Math.sin(beta);
  const CL = f0.CN * ca - f0.CA * sa;
  const CD = f0.CA * ca * cb - f0.CY * sb + f0.CN * sa * cb;
  const xcp = Math.abs(f0.CN) > 1e-6 ? m.xcg - (f0.Cm * m.lRef) / f0.CN : Math.abs(CNa) > 1e-9 ? m.xcg - (Cma * m.lRef) / CNa : m.xcg;
  const CLa = CNa * ca - f0.CA * 0 - 0;
  const derivs: Derivs = {
    CNa,
    Cma,
    CYb,
    Cnb,
    Clb,
    CLa,
    Clp: dyn.Clp,
    Cmq: dyn.Cmq,
    Cnr: dyn.Cnr,
    CNq: dyn.CNq,
    CNd,
    Cmd,
    Cld,
    staticMargin: Math.abs(CNa) > 1e-9 ? -Cma / CNa : 0,
  };
  const validity = assessValidity(ms, toTotal(alpha, beta).aT, !!ms.bodyCtx);
  for (const n of f0.bodyNotes) if (!validity.notes.includes(n)) validity.notes.push(n);
  if (aTin * RAD > 0 && Math.abs(phiDeg) > 1e-9) validity.notes.push("aerodynamic roll angle phi: alpha is the total angle of attack; coefficients in rolled body axes");
  return {
    CN: f0.CN,
    CA: f0.CA,
    Cm: f0.Cm,
    CY: f0.CY,
    Cn: f0.Cn,
    Cl: f0.Cl,
    CL,
    CD,
    LD: Math.abs(CD) > 1e-9 ? CL / CD : 0,
    xcp,
    alpha: alphaDeg,
    beta: beta * RAD,
    phi: phiDeg,
    alphaTotal: aTin * RAD,
    derivs,
    components: f0.comps,
    validity,
  };
}

export function runCases(cfgIn: VehicleConfig, flightIn: FlightSpec): RunResult {
  const flight = sanitizeFlight(flightIn);
  const model = buildModel(cfgIn);
  const warnings = [...model.geometry.warnings, ...(model.cfg.notes ?? []).filter((n) => n.includes("→"))];
  const cases: CaseResult[] = [];
  const phi = flight.phi ?? 0;
  const beta = flight.beta ?? 0;
  if (model.cfg.sets.some((s) => s.dihedral && Math.abs(s.dihedral) > 1e-9)) warnings.push("panel dihedral is accepted but not modelled in the loads");
  if (isElliptic(model.cfg.body)) warnings.push("ELLBOD: elliptical body via equivalent-radius + Ramanujan perimeter (engineering)");
  if (model.cfg.inlets?.length) warnings.push("INLET: additive-drag / ramp-CN engineering model active");
  if (model.cfg.protuberances?.length) warnings.push("PROTUB: Hoerner-style protuberance drag active");
  const machs = [...flight.machs];
  machs.forEach((_mach, i) => {
    const flow = flowConditions({ ...flight, machs }, i);
    const ms = machState(model, flow);
    const points = flight.alphas.map((a) => evalPoint(ms, a, beta, phi));
    cases.push({
      mach: flow.mach,
      altKm: flow.altKm,
      reynoldsPerM: flow.renPerM,
      reynoldsRefLength: flow.renPerM * model.lRef,
      qPa: flow.q,
      tK: flow.T,
      vMs: flow.V,
      points,
    });
  });
  return { config: model.cfg, flight, geometry: model.geometry, cases, warnings };
}
