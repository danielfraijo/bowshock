import assert from "node:assert/strict";
import { describe, it } from "node:test";
import {
  G0,
  RE_M,
  ballisticCoeff,
  energyHeight,
  eqGlideLoad,
  allenEggers,
  equilibriumGlideRange,
  flatGlideRange,
  lanchesterPhugoid,
  shortPeriodUndamped,
  vacuumRangeM,
} from "./flight-science";

describe("equilibrium glide vs its flat-Earth limit", () => {
  it("matches (L/D)(V0²−Vf²)/(2g) when both speeds are far below circular", () => {
    // Vc ≈ 7.90 km/s. The next term is ~(V0²+Vf²)/(2 Vc²). At 1.5 km/s that is already ~2.4%.
    // 0.8 → 0.4 km/s keeps it under 1%, which is what "far below circular" means here.
    const ld = 4;
    const v0 = 800;
    const vf = 400;
    const eq = equilibriumGlideRange(ld, v0, vf);
    const flat = flatGlideRange(ld, v0, vf);
    assert.ok(Math.abs(eq - flat) / flat < 0.02, `eq ${eq} flat ${flat}`);
  });

  it("the 1.5 km/s residual is the circular-speed expansion, not a bad formula", () => {
    const ld = 4;
    const v0 = 1500;
    const vf = 800;
    const vc2 = G0 * RE_M;
    const ratio = (1 - (vf * vf) / vc2) / (1 - (v0 * v0) / vc2);
    const analytic = 0.5 * RE_M * ld * Math.log(ratio);
    const eq = equilibriumGlideRange(ld, v0, vf);
    const flat = flatGlideRange(ld, v0, vf);
    assert.ok(Math.abs(eq - analytic) / analytic < 1e-12, `eq ${eq} analytic ${analytic}`);
    const rel = Math.abs(eq - flat) / flat;
    assert.ok(rel > 0.02 && rel < 0.03, `rel ${rel}`);
  });

  it("is longer than the flat glide once centrifugal relief matters", () => {
    const eq = equilibriumGlideRange(2.5, 6500, 3000);
    const flat = flatGlideRange(2.5, 6500, 3000);
    assert.ok(eq > flat * 1.2, `eq ${eq} flat ${flat}`);
  });
});

describe("Kepler surface-to-surface range", () => {
  it("recovers V²/g for a 45° suborbital loft", () => {
    const v = 1000;
    const got = vacuumRangeM(v, Math.PI / 4);
    const flat = (v * v) / G0;
    assert.ok(Math.abs(got - flat) / flat < 0.03, `vacuum ${got} flat ${flat}`);
  });

  it("is essentially zero for a vertical toss and for a horizontal skim", () => {
    assert.equal(vacuumRangeM(1000, Math.PI / 2), 0);
    assert.ok(vacuumRangeM(1000, 0) === 0);
  });

  it("grows with speed and stays inside one hemisphere", () => {
    const a = vacuumRangeM(3000, (20 * Math.PI) / 180);
    const b = vacuumRangeM(5000, (20 * Math.PI) / 180);
    assert.ok(b > a && b < Math.PI * RE_M, `${a} ${b}`);
  });
});

describe("energy, load factor, ballistic coefficient", () => {
  it("energy height is h + V²/2g", () => {
    assert.ok(Math.abs(energyHeight(10_000, 1000) - (10_000 + (1000 * 1000) / (2 * G0))) < 1e-6);
  });

  it("equilibrium load factor is 1 at rest and 0 on a circular orbit", () => {
    assert.ok(Math.abs(eqGlideLoad(0) - 1) < 1e-12);
    const vc = Math.sqrt(G0 * RE_M);
    assert.ok(Math.abs(eqGlideLoad(vc)) < 1e-9);
  });

  it("ballistic coefficient is m / (CD S)", () => {
    assert.equal(ballisticCoeff(800, 0.04, 2), 10000);
    assert.equal(ballisticCoeff(0, 0.04, 2), 0);
  });
});

describe("Allen–Eggers ballistic entry", () => {
  it("peak g is V² sin|γ| / (2 e H g) and scales with V²", () => {
    const v = 7000;
    const gamma = (-20 * Math.PI) / 180;
    const beta = 5000;
    const got = allenEggers(v, gamma, beta);
    const expectG = (v * v * Math.sin(20 * Math.PI / 180)) / (2 * Math.E * 7200 * G0);
    assert.ok(Math.abs(got.peakG - expectG) / expectG < 1e-12, `${got.peakG} vs ${expectG}`);
    const twice = allenEggers(2 * v, gamma, beta);
    assert.ok(Math.abs(twice.peakG / got.peakG - 4) < 1e-9);
    assert.equal(allenEggers(v, 0.1, beta).peakG, 0);
  });

  it("matches an independent downward integration of dV/dh", () => {
    const vE = 6500;
    const gamma = (-15 * Math.PI) / 180;
    const beta = 8000;
    const H = 7200;
    const rho0 = 1.225;
    let h = 110000;
    let v = vE;
    let peak = 0;
    let vAtPeak = 0;
    const dh = -40;
    const sinG = Math.sin(gamma);
    for (let i = 0; i < 4000 && h > 0 && v > 80; i++) {
      const rho = rho0 * Math.exp(-h / H);
      const a = (rho * v * v) / (2 * beta);
      if (a > peak) {
        peak = a;
        vAtPeak = v;
      }
      const dVdh = (-rho * v) / (2 * beta * sinG);
      v += dVdh * dh;
      h += dh;
    }
    const closed = allenEggers(vE, gamma, beta);
    const n = peak / G0;
    assert.ok(Math.abs(n - closed.peakG) / closed.peakG < 0.03, `integrated ${n} closed ${closed.peakG}`);
    const rhoStar = (beta * -sinG) / H;
    const alt = (H * Math.log(rho0 / rhoStar)) / 1000;
    assert.ok(Math.abs(closed.altKm - alt) < 1e-9, `alt ${closed.altKm}`);
    assert.ok(closed.altKm > 5 && closed.altKm < 40, `alt ${closed.altKm}`);
    assert.equal(closed.underground, false);
    assert.ok(Math.abs(vAtPeak - closed.vPeak) / closed.vPeak < 0.02, `V* ${vAtPeak} vs ${closed.vPeak}`);
  });

  it("puts a light vehicle's pulse high and a heavy one's under the surface", () => {
    const light = allenEggers(6500, (-15 * Math.PI) / 180, 400);
    assert.ok(light.altKm > 20 && light.altKm < 80, `alt ${light.altKm}`);
    assert.equal(light.underground, false);
    const deep = allenEggers(3000, (-60 * Math.PI) / 180, 20000);
    assert.equal(deep.underground, true);
    assert.equal(deep.altKm, 0);
    assert.ok(deep.peakG > 0);
    assert.equal(deep.heatUnderground, false);
    const buried = allenEggers(3000, (-60 * Math.PI) / 180, 50000);
    assert.equal(buried.heatUnderground, true);
    assert.equal(buried.heatAltKm, 0);
    assert.equal(allenEggers(3000, (-60 * Math.PI) / 180, 50).underground, false);
  });

  it("Sutton–Graves peaks H ln 3 above the g pulse and the load matches the integral", () => {
    const vE = 7000;
    const gamma = (-20 * Math.PI) / 180;
    const beta = 1500;
    const rn = 0.05;
    const H = 7200;
    const rho0 = 1.225;
    const k = 1.7415e-8;
    let h = 120000;
    let v = vE;
    let peakQ = 0;
    let peakQH = 0;
    let load = 0;
    const dh = -25;
    const sinG = Math.sin(gamma);
    for (let i = 0; i < 6000 && h > 200 && v > 80; i++) {
      const rho = rho0 * Math.exp(-h / H);
      const q = k * Math.sqrt(rho / rn) * v ** 3;
      const dt = dh / (v * sinG);
      if (q > peakQ) {
        peakQ = q;
        peakQH = h;
      }
      load += q * dt;
      const dVdh = (-rho * v) / (2 * beta * sinG);
      v += dVdh * dh;
      h += dh;
    }
    const closed = allenEggers(vE, gamma, beta, H, rho0, G0, rn);
    assert.ok(Math.abs(closed.heatAltKm - closed.altKm - (H * Math.log(3)) / 1000) < 1e-6);
    assert.ok(Math.abs(peakQ - closed.qPeak) / closed.qPeak < 0.03, `q ${peakQ} vs ${closed.qPeak}`);
    assert.ok(Math.abs(peakQH / 1000 - closed.heatAltKm) < 1, `h ${peakQH / 1000} vs ${closed.heatAltKm}`);
    assert.ok(Math.abs(load - closed.heatLoad) / closed.heatLoad < 0.03, `Q ${load} vs ${closed.heatLoad}`);
    assert.ok(Math.abs(closed.vPeak - vE / Math.sqrt(Math.E)) / vE < 1e-12);
  });
});

describe("modes", () => {
  it("undamped short-period matches Etkin ω² = −q S c Cmα / Iyy", () => {
    const w = shortPeriodUndamped(10_000, 2, 4, 500, -0.5);
    assert.ok(Math.abs(w - Math.sqrt(80)) < 1e-9, String(w));
    assert.equal(shortPeriodUndamped(10_000, 2, 4, 500, 0.2), 0);
  });

  it("Lanchester phugoid is ω = g√2 / V and ζ = 1/(√2 L/D)", () => {
    const v = G0 * Math.SQRT2;
    const ph = lanchesterPhugoid(v, 1 / Math.SQRT2);
    assert.ok(Math.abs(ph.wn - 1) < 1e-12, String(ph.wn));
    assert.ok(Math.abs(ph.zeta - 1) < 1e-12, String(ph.zeta));
    assert.ok(Math.abs(ph.period - 2 * Math.PI) < 1e-9);
  });
});
