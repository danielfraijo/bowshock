import { DEG, clamp, newtonianCpMax, suttonGraves, tauberSutton } from "./math";
import { atmosphere } from "./atmosphere";
import type { DesignParams } from "./types";

export interface PolarPt {
  a: number;
  cl: number;
  cd: number;
  cm: number;
  ld: number;
}

export interface TrajSample {
  t: number;
  hKm: number;
  v: number;
  M: number;
  q: number;
  gammaDeg: number;
  xKm: number;
  qConv: number;
  qRad: number;
}

export interface TrajResult {
  samples: TrajSample[];
  rangeKm: number;
  timeS: number;
  maxQ: number;
  maxQAltKm: number;
  maxHeat: number;
  heatLoad: number;
  finalV: number;
  finalH: number;
  skipped: boolean;
  notes: string[];
}

function lerpPolar(polar: PolarPt[], alpha: number) {
  if (!polar.length) return { cl: 0.2, cd: 0.08 };
  if (alpha <= polar[0].a) return polar[0];
  const last = polar[polar.length - 1];
  if (alpha >= last.a) return last;
  for (let i = 0; i < polar.length - 1; i++) {
    const a = polar[i];
    const b = polar[i + 1];
    if (alpha >= a.a && alpha <= b.a) {
      const u = (alpha - a.a) / Math.max(b.a - a.a, 1e-9);
      return { cl: a.cl + (b.cl - a.cl) * u, cd: a.cd + (b.cd - a.cd) * u };
    }
  }
  return last;
}

/** Point-mass 3DOF (V, γ, h, x) RK4. Lift/drag from the panel polar at a fixed α. */
export function integrateGlide(
  params: DesignParams,
  polar: PolarPt[],
  mass: number,
  sRef: number,
  Rn: number,
): TrajResult {
  const notes: string[] = [];
  const g0 = 9.80665;
  const Re = 6371000;
  const gamma = params.gamma;
  const alpha = params.alphaDeg;
  let v = 0;
  let gam = params.gammaDeg * DEG;
  let h = Math.max(500, params.altKm * 1000);
  let x = 0;
  const atm0 = atmosphere(h / 1000, 8, gamma);
  v = Math.max(400, (params.lockFlight ? params.mach : params.flightMach) * atm0.a);
  const Mref = Math.max(1.2, params.lockFlight ? params.mach : params.flightMach);
  const cpRef = newtonianCpMax(Mref, gamma);
  const dt = 0.25;
  const samples: TrajSample[] = [];
  let maxQ = 0;
  let maxQAltKm = h / 1000;
  let maxHeat = 0;
  let heatLoad = 0;
  let skipped = false;
  const m = Math.max(5, mass);

  const deriv = (vv: number, gg: number, hh: number) => {
    const atm = atmosphere(hh / 1000, Math.max(0.2, vv / Math.max(atm0.a, 1)), gamma);
    const M = vv / Math.max(atm.a, 1);
    const polarA = lerpPolar(polar, alpha);
    const scale = newtonianCpMax(Math.max(1.05, M), gamma) / Math.max(cpRef, 1e-6);
    const cl = polarA.cl * scale;
    const cd = Math.max(0.008, polarA.cd * scale);
    const q = 0.5 * atm.rho * vv * vv;
    const L = q * sRef * cl;
    const D = q * sRef * cd;
    const g = g0 * (Re / (Re + hh)) ** 2;
    const dv = -D / m - g * Math.sin(gg);
    const dgam = vv > 20 ? L / (m * vv) + (vv / (Re + hh) - g / vv) * Math.cos(gg) : 0;
    const dh = vv * Math.sin(gg);
    const dx = (vv * Math.cos(gg) * Re) / (Re + hh);
    return { dv, dgam, dh, dx, q, M, atm };
  };

  for (let k = 0; k < 720; k++) {
    const d1 = deriv(v, gam, h);
    if (k % 4 === 0 || k === 0) {
      const recov = 0.5;
      const qConv = suttonGraves(d1.atm.rho, v, Math.max(Rn, 0.004), recov);
      const qRad = tauberSutton(d1.atm.rho, v, Rn);
      samples.push({
        t: k * dt,
        hKm: h / 1000,
        v,
        M: d1.M,
        q: d1.q,
        gammaDeg: gam * (180 / Math.PI),
        xKm: x / 1000,
        qConv,
        qRad,
      });
      if (d1.q > maxQ) {
        maxQ = d1.q;
        maxQAltKm = h / 1000;
      }
      const qTot = qConv + qRad;
      if (qTot > maxHeat) maxHeat = qTot;
      heatLoad += qTot * dt * 4;
    }
    if (h < 400 && gam < 0) break;
    if (h > 92000) {
      skipped = true;
      notes.push("Trajectory skipped out of the atmosphere (γ>0 after pull-up).");
      break;
    }
    if (v < 180) break;
    const d2 = deriv(v + 0.5 * dt * d1.dv, gam + 0.5 * dt * d1.dgam, h + 0.5 * dt * d1.dh);
    const d3 = deriv(v + 0.5 * dt * d2.dv, gam + 0.5 * dt * d2.dgam, h + 0.5 * dt * d2.dh);
    const d4 = deriv(v + dt * d3.dv, gam + dt * d3.dgam, h + dt * d3.dh);
    v += (dt / 6) * (d1.dv + 2 * d2.dv + 2 * d3.dv + d4.dv);
    gam += (dt / 6) * (d1.dgam + 2 * d2.dgam + 2 * d3.dgam + d4.dgam);
    h += (dt / 6) * (d1.dh + 2 * d2.dh + 2 * d3.dh + d4.dh);
    x += (dt / 6) * (d1.dx + 2 * d2.dx + 2 * d3.dx + d4.dx);
    v = Math.max(50, v);
    gam = clamp(gam, -40 * DEG, 18 * DEG);
    h = clamp(h, 0, 95000);
  }
  notes.push(
    "3DOF point-mass RK4 (Δt=0.25 s). CL/CD from the panel polar, scaled by Cp_max(M)/Cp_max(M_ref) (Lees). Spherical Earth, US76. No bank.",
  );
  const last = samples[samples.length - 1];
  return {
    samples,
    rangeKm: last ? last.xKm : x / 1000,
    timeS: last ? last.t : 0,
    maxQ,
    maxQAltKm,
    maxHeat,
    heatLoad,
    finalV: last ? last.v : v,
    finalH: last ? last.hKm : h / 1000,
    skipped,
    notes,
  };
}
