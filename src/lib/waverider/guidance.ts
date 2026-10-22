/**
 * Closed-loop trajectory on a nonlinear aero table.
 *
 * The linear 6DOF tab integrates derivatives about one condition. This one
 * builds a small CL/CD/Cm table in Mach, α, and elevon, then integrates the
 * same nonlinear state equations (α, q, θ, β, p, r, φ, V, γ, h, x) with:
 *
 * - spherical-Earth γ̇ and ground range, matching the 3DOF integrator
 * - α̇ = θ̇ − γ̇, so incidence stays θ − γ (the previous sin() term zeroed gravity)
 * - RK4 on the vehicle state; the elevon is a zero-order hold across the step
 *
 * - a guidance law that commands angle of attack,
 * - PID or nonlinear dynamic inversion that commands elevon deflection,
 * - a first-order actuator with a rate limit,
 * - optional Gaussian noise on the measured angle of attack.
 *
 * The table is the panel method (CBAERO stands in when Euler is selected,
 * because a population of Euler solves will not finish in a browser). Rate
 * derivatives are the same body-rate perturbation as `sixdof.ts`, frozen at
 * the design point and added on top of the table.
 *
 * Headless: `samplesToCsv` is the exchange format. `public` consumers and
 * `cuspis_6dof.py` (shipped in the CFD kit) read the aero JSON and integrate
 * the same PID / boost-glide loop outside the browser.
 */
import { atmosphere, type Atmosphere } from "./atmosphere";
import { buildVehicle } from "./generate";
import { DEG, RAD, clamp } from "./math";
import { massProperties } from "./mass";
import { panelAero } from "./panel";
import type { BuiltVehicle, DesignParams } from "./types";

export type GuidanceMode = "boost-glide" | "constant-q" | "skip" | "terminal";
export type ControlLaw = "pid" | "ndi";

export interface FlySpec {
  mode: GuidanceMode;
  law: ControlLaw;
  durationS: number;
  dt: number;
  tauAct: number;
  deltaMaxDeg: number;
  rateMaxDeg: number;
  sensorSigmaDeg: number;
  seed: number;
  qTargetKpa: number;
  targetXKm: number;
  kp: number;
  ki: number;
  kd: number;
}

export interface FlySample {
  t: number;
  alphaDeg: number;
  betaDeg: number;
  deltaDeg: number;
  alphaCmd: number;
  qDeg: number;
  pDeg: number;
  rDeg: number;
  thetaDeg: number;
  phiDeg: number;
  gammaDeg: number;
  V: number;
  mach: number;
  hKm: number;
  xKm: number;
  qKpa: number;
  nG: number;
}

export interface FlyResult {
  samples: FlySample[];
  notes: string[];
  maxQ: number;
  maxN: number;
  rangeKm: number;
  finalH: number;
  timeS: number;
  mode: GuidanceMode;
  law: ControlLaw;
}

export function defaultFlySpec(p: DesignParams): FlySpec {
  return {
    mode: "boost-glide",
    law: "pid",
    durationS: 80,
    dt: 0.05,
    tauAct: 0.08,
    deltaMaxDeg: 12,
    rateMaxDeg: 40,
    sensorSigmaDeg: 0,
    seed: 1,
    qTargetKpa: 40,
    targetXKm: 200,
    kp: 1.4,
    ki: 0.15,
    kd: 0.08,
  };
}

interface C3 {
  cl: number;
  cd: number;
  cm: number;
}

interface Table {
  mach: number[];
  alpha: number[];
  delta: number[];
  /** [im][ia][id] */
  c: C3[][][];
  cyb: number;
  cnb: number;
  clb: number;
  clq: number;
  cmq: number;
  clp: number;
  cnr: number;
}

function mulberry32(seed: number) {
  let a = seed >>> 0 || 1;
  return () => {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function gauss(rng: () => number) {
  const u = Math.max(1e-9, rng());
  const v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

function bracket(axis: number[], x: number) {
  if (x <= axis[0]) return { i0: 0, i1: 0, u: 0 };
  const last = axis.length - 1;
  if (x >= axis[last]) return { i0: last, i1: last, u: 0 };
  let i = 0;
  while (i < last - 1 && axis[i + 1] < x) i++;
  const span = axis[i + 1] - axis[i] || 1;
  return { i0: i, i1: i + 1, u: (x - axis[i]) / span };
}

function lerpC(a: C3, b: C3, u: number): C3 {
  return { cl: a.cl + (b.cl - a.cl) * u, cd: a.cd + (b.cd - a.cd) * u, cm: a.cm + (b.cm - a.cm) * u };
}

function lookup(table: Table, mach: number, alpha: number, delta: number): C3 {
  const m = bracket(table.mach, mach);
  const a = bracket(table.alpha, alpha);
  const d = bracket(table.delta, delta);
  const at = (im: number, ia: number) => lerpC(table.c[im][ia][d.i0], table.c[im][ia][d.i1], d.u);
  const row = (im: number) => lerpC(at(im, a.i0), at(im, a.i1), a.u);
  return lerpC(row(m.i0), row(m.i1), m.u);
}

function bestLdAlpha(table: Table, mach: number): number {
  let bestA = table.alpha[0];
  let best = -Infinity;
  for (const alpha of table.alpha) {
    const c = lookup(table, mach, alpha, 0);
    const ld = c.cd > 1e-6 ? c.cl / c.cd : 0;
    if (ld > best) {
      best = ld;
      bestA = alpha;
    }
  }
  return bestA;
}

export interface FlyProgress {
  done: number;
  total: number;
  label: string;
}

export async function flyClosedLoop(
  built: BuiltVehicle,
  spec: FlySpec,
  opts?: {
    signal?: AbortSignal;
    onProgress?: (p: FlyProgress) => void;
    massKg?: number;
    ixx?: number;
    iyy?: number;
    izz?: number;
  },
): Promise<FlyResult> {
  const p = built.params;
  const method = p.aeroMethod === "euler" ? "cbaero" : p.aeroMethod;
  const Mdesign = p.lockFlight ? p.mach : p.flightMach;
  const machs = [clamp(Mdesign * 0.7, 1.4, 22), clamp(Mdesign, 1.5, 24), clamp(Mdesign * 1.25, 1.8, 26)];
  const uniqM = [...new Set(machs.map((v) => Math.round(v * 100) / 100))].sort((a, b) => a - b);
  const alphas = [-4, 0, 4, 8, 12];
  const deltas = [-spec.deltaMaxDeg, 0, spec.deltaMaxDeg];
  const sRef = Math.max(built.aero.planformArea, 1e-6);
  const Lref = Math.max(p.length, 1e-6);
  const bRef = Math.max(p.span, 1e-6);
  const total = uniqM.length * alphas.length * deltas.length + 8;
  let done = 0;
  const tick = async (label: string) => {
    if (opts?.signal?.aborted) throw new DOMException("Flight cancelled", "AbortError");
    opts?.onProgress?.({ done, total, label });
    await new Promise((r) => setTimeout(r, 0));
  };

  const meshes = new Map<number, BuiltVehicle>();
  const vehicleAt = (delta: number) => {
    const key = Math.round(delta * 100);
    const hit = meshes.get(key);
    if (hit) return hit;
    if (Math.abs(delta - (p.elevonDeg || 0)) < 1e-6) {
      meshes.set(key, built);
      return built;
    }
    const v = buildVehicle({ ...p, elevonDeg: delta });
    meshes.set(key, v);
    return v;
  };

  const c: C3[][][] = [];
  for (let im = 0; im < uniqM.length; im++) {
    c[im] = [];
    for (let ia = 0; ia < alphas.length; ia++) {
      c[im][ia] = [];
      for (let id = 0; id < deltas.length; id++) {
        await tick(`table M ${uniqM[im].toFixed(1)} α ${alphas[ia]} δ ${deltas[id]}`);
        const v = vehicleAt(deltas[id]);
        const atm = atmosphere(p.altKm, uniqM[im], p.gamma);
        const aero = panelAero(
          v.mesh,
          { ...p, aeroMethod: method, elevonDeg: deltas[id], lockFlight: false, flightMach: uniqM[im] },
          atm,
          alphas[ia],
          0,
          sRef,
          Lref,
          undefined,
          true,
        );
        c[im][ia][id] = { cl: aero.cl, cd: aero.cd, cm: aero.cm };
        done++;
      }
    }
  }

  const atm0 = atmosphere(p.altKm, Mdesign, p.gamma);
  const baseMesh = vehicleAt(0);
  const flightP: DesignParams = { ...p, aeroMethod: method, elevonDeg: 0, lockFlight: false, flightMach: Mdesign };
  const bp = panelAero(baseMesh.mesh, flightP, atm0, p.alphaDeg, 2, sRef, Lref, undefined, true);
  const bm = panelAero(baseMesh.mesh, flightP, atm0, p.alphaDeg, -2, sRef, Lref, undefined, true);
  const V0 = Math.max(80, atm0.V);
  const qHat = 0.02;
  const qRate = (qHat * 2 * V0) / Lref;
  const pRate = (qHat * 2 * V0) / bRef;
  const rRate = pRate;
  const qP = panelAero(baseMesh.mesh, flightP, atm0, p.alphaDeg, 0, sRef, Lref, { p: 0, q: qRate, r: 0 }, true);
  const qM = panelAero(baseMesh.mesh, flightP, atm0, p.alphaDeg, 0, sRef, Lref, { p: 0, q: -qRate, r: 0 }, true);
  const pP = panelAero(baseMesh.mesh, flightP, atm0, p.alphaDeg, 0, sRef, Lref, { p: pRate, q: 0, r: 0 }, true);
  const pM = panelAero(baseMesh.mesh, flightP, atm0, p.alphaDeg, 0, sRef, Lref, { p: -pRate, q: 0, r: 0 }, true);
  const rP = panelAero(baseMesh.mesh, flightP, atm0, p.alphaDeg, 0, sRef, Lref, { p: 0, q: 0, r: rRate }, true);
  const rM = panelAero(baseMesh.mesh, flightP, atm0, p.alphaDeg, 0, sRef, Lref, { p: 0, q: 0, r: -rRate }, true);
  done += 8;
  const hBeta = 4 * DEG;
  const table: Table = {
    mach: uniqM,
    alpha: alphas,
    delta: deltas,
    c,
    cyb: (bp.cy - bm.cy) / hBeta,
    cnb: (bp.cn - bm.cn) / hBeta,
    clb: (bp.cll - bm.cll) / hBeta,
    clq: (qP.cl - qM.cl) / (2 * qHat),
    cmq: (qP.cm - qM.cm) / (2 * qHat),
    clp: (pP.cll - pM.cll) / (2 * qHat),
    cnr: (rP.cn - rM.cn) / (2 * qHat),
  };

  const mass0 = massProperties(built.mesh, p.rhoKgM3 || 160, p.massKg || 0);
  const mass = Math.max(5, opts?.massKg ?? mass0.mass);
  const Ixx = Math.max(1e-3, opts?.ixx ?? mass0.Ixx);
  const Iyy = Math.max(1e-3, opts?.iyy ?? mass0.Iyy);
  const Izz = Math.max(1e-3, opts?.izz ?? mass0.Izz);
  const g0 = 9.80665;
  const Re = 6_371_000;
  const rng = mulberry32(spec.seed || 1);

  let alpha = p.alphaDeg * DEG;
  let q = 0;
  let theta = p.alphaDeg * DEG + p.gammaDeg * DEG;
  let beta = p.betaDeg * DEG;
  let roll = 0;
  let yaw = 0;
  let phi = 0;
  let V = V0;
  let gamma = p.gammaDeg * DEG;
  let h = Math.max(200, p.altKm * 1000);
  let x = 0;
  let delta = 0;
  let integ = 0;
  let ePrev = 0;
  const dt = clamp(spec.dt, 0.01, 0.2);
  const nStep = Math.min(4000, Math.max(10, Math.round(spec.durationS / dt)));
  const samples: FlySample[] = [];
  let maxQ = 0;
  let maxN = 0;
  const notes = [
    `Nonlinear 6DOF on a ${uniqM.length}×${alphas.length}×${deltas.length} panel table (${method}). β derivatives and rate derivatives are frozen at the design point.`,
    `Guidance: ${spec.mode}. Control: ${spec.law}. Actuator τ = ${spec.tauAct} s, |δ| ≤ ${spec.deltaMaxDeg}°, rate ≤ ${spec.rateMaxDeg}°/s.`,
    spec.sensorSigmaDeg > 0 ? `Angle-of-attack measurement noise σ = ${spec.sensorSigmaDeg}° (seed ${spec.seed}).` : "Sensors are perfect.",
    "Spherical-Earth kinematics (same γ̇ and arc range as the 3DOF), inverse-square gravity, US76. α̇ = θ̇ − γ̇. Not a Monte-Carlo dispersion set.",
  ];

  const sampleEvery = Math.max(1, Math.round(0.25 / dt));

  interface Dyn {
    alpha: number;
    q: number;
    theta: number;
    beta: number;
    roll: number;
    yaw: number;
    phi: number;
    V: number;
    gamma: number;
    h: number;
    x: number;
  }
  const deriv = (st: Dyn, deltaRad: number) => {
    const atm: Atmosphere = atmosphere(Math.max(0, st.h) / 1000, 1, p.gamma);
    const Mach = st.V / Math.max(atm.a, 1);
    const qDyn = 0.5 * atm.rho * st.V * st.V;
    const g = g0 * (Re / (Re + Math.max(st.h, 0))) ** 2;
    const alphaDeg = st.alpha * RAD;
    const deltaDeg = deltaRad * RAD;
    const base = lookup(table, Mach, alphaDeg, 0);
    const here = lookup(table, Mach, alphaDeg, deltaDeg);
    const qh = (st.q * Lref) / (2 * Math.max(st.V, 1));
    const ph = (st.roll * bRef) / (2 * Math.max(st.V, 1));
    const rh = (st.yaw * bRef) / (2 * Math.max(st.V, 1));
    const CL = here.cl + table.clq * qh;
    const CD = Math.max(0.002, here.cd);
    const Cm = here.cm + table.cmq * qh;
    const CY = table.cyb * st.beta;
    const Cl = table.clb * st.beta + table.clp * ph;
    const Cn = table.cnb * st.beta + table.cnr * rh;
    const Lift = qDyn * sRef * CL;
    const Drag = qDyn * sRef * CD;
    const nG = Math.hypot(Lift, Drag) / (mass * g0);
    // Vinh: bank tilts lift out of the vertical plane. φ = 0 recovers the 3DOF γ̇.
    const gDot = st.V > 30 ? (Lift * Math.cos(st.phi)) / (mass * st.V) + (st.V / (Re + Math.max(st.h, 0)) - g / st.V) * Math.cos(st.gamma) : 0;
    const thDot = st.q * Math.cos(st.phi) - st.yaw * Math.sin(st.phi);
    const aDot = thDot - gDot;
    const qDot = (qDyn * sRef * Lref * Cm) / Iyy;
    const bDot = st.roll * Math.sin(st.alpha) - st.yaw * Math.cos(st.alpha) + (qDyn * sRef * CY) / (mass * Math.max(st.V, 1));
    const pDot = (qDyn * sRef * bRef * Cl) / Ixx;
    const rDot = (qDyn * sRef * bRef * Cn) / Izz;
    const phiDot = st.roll + Math.tan(clamp(st.theta, -1.2, 1.2)) * (st.q * Math.sin(st.phi) + st.yaw * Math.cos(st.phi));
    const vDot = -Drag / mass - g * Math.sin(st.gamma);
    const hDot = st.V * Math.sin(st.gamma);
    const xDot = (st.V * Math.cos(st.gamma) * Re) / (Re + Math.max(st.h, 0));
    return { aDot, qDot, thDot, bDot, pDot, rDot, phiDot, vDot, gDot, hDot, xDot, qDyn, Mach, nG, alphaDeg, deltaDeg, base };
  };
  const add = (st: Dyn, k: ReturnType<typeof deriv>, s: number): Dyn => ({
    alpha: st.alpha + s * k.aDot,
    q: st.q + s * k.qDot,
    theta: st.theta + s * k.thDot,
    beta: st.beta + s * k.bDot,
    roll: st.roll + s * k.pDot,
    yaw: st.yaw + s * k.rDot,
    phi: st.phi + s * k.phiDot,
    V: st.V + s * k.vDot,
    gamma: st.gamma + s * k.gDot,
    h: st.h + s * k.hDot,
    x: st.x + s * k.xDot,
  });
  const rk = (a: number, b: number, c: number, d: number) => (dt / 6) * (a + 2 * b + 2 * c + d);

  let st: Dyn = { alpha, q, theta, beta, roll, yaw, phi, V, gamma, h, x };

  for (let k = 0; k <= nStep; k++) {
    const k1 = deriv(st, delta);
    if (k1.qDyn > maxQ) maxQ = k1.qDyn;
    if (k1.nG > maxN) maxN = k1.nG;

    const qTarget = spec.qTargetKpa * 1000;
    let alphaCmdDeg = bestLdAlpha(table, k1.Mach);
    if (spec.mode === "constant-q") {
      alphaCmdDeg = clamp(alphaCmdDeg + 8 * ((k1.qDyn - qTarget) / Math.max(qTarget, 1)), -2, 14);
    } else if (spec.mode === "skip") {
      if (k1.qDyn > 1.25 * qTarget) alphaCmdDeg = 12;
      else if (k1.qDyn < 0.55 * qTarget) alphaCmdDeg = 1;
      else alphaCmdDeg = 6;
    } else if (spec.mode === "terminal") {
      const dx = spec.targetXKm * 1000 - st.x;
      const los = Math.atan2(-st.h, Math.max(dx, 50));
      alphaCmdDeg = clamp(4 + ((los - st.gamma) * RAD) * 0.8, -2, 14);
    }
    const alphaMeas = k1.alphaDeg + (spec.sensorSigmaDeg > 0 ? gauss(rng) * spec.sensorSigmaDeg : 0);
    const err = alphaCmdDeg - alphaMeas;
    let deltaCmd = 0;
    if (spec.law === "ndi") {
      const aErr = err * DEG;
      const qCmd = 2.2 * aErr;
      const qdotCmd = 6 * (qCmd - st.q);
      const CmCmd = (Iyy * qdotCmd) / Math.max(k1.qDyn * sRef * Lref, 1);
      const span = spec.deltaMaxDeg || 8;
      const cmPlus = lookup(table, k1.Mach, k1.alphaDeg, span).cm;
      const cm0 = k1.base.cm;
      const cmd = (cmPlus - cm0) / (span * DEG || 1e-3);
      const dRad = Math.abs(cmd) > 1e-4 ? (CmCmd - cm0) / cmd : 0;
      deltaCmd = clamp(dRad * RAD, -spec.deltaMaxDeg, spec.deltaMaxDeg);
    } else {
      integ += err * dt;
      integ = clamp(integ, -40, 40);
      const derivE = (err - ePrev) / dt;
      ePrev = err;
      deltaCmd = clamp(spec.kp * err + spec.ki * integ + spec.kd * derivE, -spec.deltaMaxDeg, spec.deltaMaxDeg);
    }
    const tau = Math.max(0.02, spec.tauAct);
    const rate = spec.rateMaxDeg;
    const deltaDegNow = delta * RAD;
    let dDelta = ((deltaCmd - deltaDegNow) / tau) * dt;
    dDelta = clamp(dDelta, -rate * dt, rate * dt);
    const deltaNext = clamp((deltaDegNow + dDelta) * DEG, -spec.deltaMaxDeg * DEG, spec.deltaMaxDeg * DEG);

    if (k % sampleEvery === 0) {
      samples.push({
        t: k * dt,
        alphaDeg: k1.alphaDeg,
        betaDeg: st.beta * RAD,
        deltaDeg: k1.deltaDeg,
        alphaCmd: alphaCmdDeg,
        qDeg: st.q * RAD,
        pDeg: st.roll * RAD,
        rDeg: st.yaw * RAD,
        thetaDeg: st.theta * RAD,
        phiDeg: st.phi * RAD,
        gammaDeg: st.gamma * RAD,
        V: st.V,
        mach: k1.Mach,
        hKm: st.h / 1000,
        xKm: st.x / 1000,
        qKpa: k1.qDyn / 1000,
        nG: k1.nG,
      });
    }
    if (st.h < 0 || st.V < 180 || !Number.isFinite(st.V) || !Number.isFinite(st.alpha)) {
      notes.push(st.h < 0 ? "Integration stopped at the surface." : "Integration stopped on a speed or numerical limit.");
      break;
    }

    const k2 = deriv(add(st, k1, 0.5 * dt), delta);
    const k3 = deriv(add(st, k2, 0.5 * dt), delta);
    const k4 = deriv(add(st, k3, dt), delta);
    const aRaw = st.alpha + rk(k1.aDot, k2.aDot, k3.aDot, k4.aDot);
    const gRaw = st.gamma + rk(k1.gDot, k2.gDot, k3.gDot, k4.gDot);
    const aLim = clamp(aRaw, -20 * DEG, 25 * DEG);
    const gLim = clamp(gRaw, -40 * DEG, 30 * DEG);
    st = {
      alpha: aLim,
      q: st.q + rk(k1.qDot, k2.qDot, k3.qDot, k4.qDot),
      theta: st.theta + rk(k1.thDot, k2.thDot, k3.thDot, k4.thDot),
      beta: clamp(st.beta + rk(k1.bDot, k2.bDot, k3.bDot, k4.bDot), -15 * DEG, 15 * DEG),
      roll: st.roll + rk(k1.pDot, k2.pDot, k3.pDot, k4.pDot),
      yaw: st.yaw + rk(k1.rDot, k2.rDot, k3.rDot, k4.rDot),
      phi: st.phi + rk(k1.phiDot, k2.phiDot, k3.phiDot, k4.phiDot),
      V: Math.max(0, st.V + rk(k1.vDot, k2.vDot, k3.vDot, k4.vDot)),
      gamma: gLim,
      h: st.h + rk(k1.hDot, k2.hDot, k3.hDot, k4.hDot),
      x: st.x + rk(k1.xDot, k2.xDot, k3.xDot, k4.xDot),
    };
    if (aLim !== aRaw || gLim !== gRaw) st.theta = st.alpha + st.gamma;
    delta = deltaNext;
  }

  opts?.onProgress?.({ done: total, total, label: "done" });
  const last = samples[samples.length - 1];
  return {
    samples,
    notes,
    maxQ,
    maxN,
    rangeKm: last?.xKm ?? 0,
    finalH: last?.hKm ?? 0,
    timeS: last?.t ?? 0,
    mode: spec.mode,
    law: spec.law,
  };
}

export function samplesToCsv(result: FlyResult): string {
  const head = "t_s,alpha_deg,alpha_cmd_deg,beta_deg,delta_deg,q_deg_s,p_deg_s,r_deg_s,theta_deg,phi_deg,gamma_deg,V_m_s,mach,h_km,x_km,q_kPa,n_g";
  const lines = result.samples.map((s) =>
    [s.t, s.alphaDeg, s.alphaCmd, s.betaDeg, s.deltaDeg, s.qDeg, s.pDeg, s.rDeg, s.thetaDeg, s.phiDeg, s.gammaDeg, s.V, s.mach, s.hKm, s.xKm, s.qKpa, s.nG]
      .map((v) => (Number.isFinite(v) ? v.toFixed(5) : ""))
      .join(","),
  );
  return ["# cuspis closed-loop trajectory", `# mode ${result.mode}`, `# law ${result.law}`, ...result.notes.map((n) => `# ${n}`), head, ...lines].join("\n") + "\n";
}

/** One actuator step. Exported so the lag and the rate limit can be checked without a mesh. */
export function actuatorStep(deltaDeg: number, commandDeg: number, dt: number, tau: number, rateDeg: number, limitDeg: number): number {
  const d = ((commandDeg - deltaDeg) / Math.max(tau, 1e-3)) * dt;
  const limited = clamp(d, -Math.abs(rateDeg) * dt, Math.abs(rateDeg) * dt);
  return clamp(deltaDeg + limited, -Math.abs(limitDeg), Math.abs(limitDeg));
}
