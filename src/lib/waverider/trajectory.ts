import { DEG, clamp, fayRiddell, knudsen, newtonianCpMax, rarefactionWeight, suttonGraves, tauberSutton } from "./math";
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
  maxHeatAltKm: number;
  maxN: number;
  heatLoad: number;
  twPeak: number;
  finalV: number;
  finalH: number;
  skipped: boolean;
  rangeEqKm: number;
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
  const samples: TrajSample[] = [];
  let maxQ = 0;
  let maxQAltKm = h / 1000;
  let maxHeat = 0;
  let maxHeatAltKm = h / 1000;
  let maxN = 0;
  let heatLoad = 0;
  let twPeak = 300;
  let Tw = Math.max(300, params.twK || 400);
  let skipped = false;
  const m = Math.max(5, mass);
  const rhoCdelta = 1600 * 800 * 0.003;
  const g0loc = 9.80665;

  const deriv = (vv: number, gg: number, hh: number) => {
    const atm = atmosphere(hh / 1000, Math.max(0.2, vv / Math.max(atm0.a, 1)), gamma);
    const M = vv / Math.max(atm.a, 1);
    const polarA = lerpPolar(polar, alpha);
    const scale = newtonianCpMax(Math.max(1.05, M), gamma) / Math.max(cpRef, 1e-6);
    const kn = knudsen(atm.T, atm.p, Math.max(params.length, 1e-6));
    const cl = polarA.cl * scale;
    const cd = Math.max(0.008, polarA.cd * scale * (1 + 1.8 * rarefactionWeight(kn)));
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

  const rk4 = (vv: number, gg: number, hh: number, xx: number, hStep: number) => {
    const d1 = deriv(vv, gg, hh);
    const d2 = deriv(vv + 0.5 * hStep * d1.dv, gg + 0.5 * hStep * d1.dgam, hh + 0.5 * hStep * d1.dh);
    const d3 = deriv(vv + 0.5 * hStep * d2.dv, gg + 0.5 * hStep * d2.dgam, hh + 0.5 * hStep * d2.dh);
    const d4 = deriv(vv + hStep * d3.dv, gg + hStep * d3.dgam, hh + hStep * d3.dh);
    return {
      v: vv + (hStep / 6) * (d1.dv + 2 * d2.dv + 2 * d3.dv + d4.dv),
      gam: gg + (hStep / 6) * (d1.dgam + 2 * d2.dgam + 2 * d3.dgam + d4.dgam),
      h: hh + (hStep / 6) * (d1.dh + 2 * d2.dh + 2 * d3.dh + d4.dh),
      x: xx + (hStep / 6) * (d1.dx + 2 * d2.dx + 2 * d3.dx + d4.dx),
      d1,
    };
  };

  let dt = 0.25;
  let t = 0;
  for (let k = 0; k < 900; k++) {
    const coarse = rk4(v, gam, h, x, dt);
    const half = rk4(v, gam, h, x, dt / 2);
    const fine = rk4(half.v, half.gam, half.h, half.x, dt / 2);
    const err = Math.hypot(fine.v - coarse.v, (fine.h - coarse.h) / 400);
    if (err > 5 && dt > 0.06) {
      dt *= 0.5;
      continue;
    }
    v = Math.max(50, fine.v);
    gam = clamp(fine.gam, -40 * DEG, 18 * DEG);
    h = clamp(fine.h, 0, 95000);
    x = fine.x;
    t += dt;
    if (err < 0.3) dt = Math.min(0.8, dt * 1.12);
    const recov = 0.5;
    const RnUse = Math.max(Rn, 0.004);
    const qSG = suttonGraves(fine.d1.atm.rho, v, RnUse, recov);
    const qFR = fayRiddell(fine.d1.atm.rho, v, RnUse, fine.d1.atm.T, fine.d1.atm.p, Tw, gamma);
    const qConv = qFR > 0 ? qFR : qSG;
    const qRad = tauberSutton(fine.d1.atm.rho, v, Rn);
    const qTot = qConv + qRad;
    heatLoad += qTot * dt;
    const qNet = qTot * 1e4 - 0.8 * 5.670374419e-8 * Tw * Tw * Tw * Tw;
    Tw = clamp(Tw + (dt * qNet) / rhoCdelta, 200, 4500);
    if (Tw > twPeak) twPeak = Tw;
    const polarA = lerpPolar(polar, alpha);
    const nLoad = (fine.d1.q * sRef * Math.max(polarA.cl, 0)) / Math.max(m * g0loc, 1);
    if (nLoad > maxN) maxN = nLoad;
    if (fine.d1.q > maxQ) {
      maxQ = fine.d1.q;
      maxQAltKm = h / 1000;
    }
    if (qTot > maxHeat) {
      maxHeat = qTot;
      maxHeatAltKm = h / 1000;
    }
    if (k % 3 === 0 || k === 0) {
      samples.push({
        t,
        hKm: h / 1000,
        v,
        M: fine.d1.M,
        q: fine.d1.q,
        gammaDeg: gam * (180 / Math.PI),
        xKm: x / 1000,
        qConv,
        qRad,
      });
    }
    if (h < 400 && gam < 0) break;
    if (h > 88000 && gam > 0) {
      // Skip: loft, then let gravity pull γ back. Don't abort the glide.
      gam = Math.min(gam, 4 * DEG);
    }
    if (h > 98000) {
      h = 98000;
      if (gam > 0) gam *= 0.6;
    }
    if (v < 180) break;
  }
  const polar0 = lerpPolar(polar, alpha);
  const ld0 = polar0.cd > 1e-8 ? polar0.cl / polar0.cd : 0;
  const vCirc = Math.sqrt(g0 * Re);
  const v0 = Math.max(400, (params.lockFlight ? params.mach : params.flightMach) * atm0.a);
  const num = 1 - (v / vCirc) ** 2;
  const den = 1 - (v0 / vCirc) ** 2;
  const rangeEqKm =
    ld0 > 0.2 && den > 1e-6 && num > 0
      ? (0.5 * Re * ld0 * Math.log(Math.max(num / den, 1.001))) / 1000
      : 0;
  notes.push(
    "3DOF point-mass adaptive RK4 (step doubling). CL/CD from the panel polar, Mach-scaled by Cp_max (Lees). CD grows with Knudsen at altitude. Heating is Fay–Riddell + Tauber–Sutton; Tw from a 3 mm C/C lump. Spherical Earth, US76. Skips are integrated, not aborted. Eq-glide range is Sänger / Eggers: ½ Re (L/D) ln[(1−Vf²/Vc²)/(1−V0²/Vc²)].",
  );
  const last = samples[samples.length - 1];
  return {
    samples,
    rangeKm: last ? last.xKm : x / 1000,
    timeS: last ? last.t : 0,
    maxQ,
    maxQAltKm,
    maxHeat,
    maxHeatAltKm,
    maxN,
    heatLoad,
    twPeak,
    finalV: last ? last.v : v,
    finalH: last ? last.hKm : h / 1000,
    skipped,
    rangeEqKm,
    notes,
  };
}
