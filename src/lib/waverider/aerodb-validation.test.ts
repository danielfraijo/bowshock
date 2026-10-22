/**
 * [aerodb-headless] Literature validation of the kernels behind the aero database (aerodb.ts → panel.ts → math.ts).
 * Run: npm run test:aerodb-validation
 *
 * Every reference value below is DERIVED IN THIS FILE from a closed-form relation (or, for Taylor–Maccoll, by an
 * independent RK4 integration of the ODE written here from the cited equation). No reference number is typed in;
 * the only literal numbers are inputs (M, angles, γ), the correlation constants printed in the cited source, and
 * tolerances. Published tabulated values (NACA 1135 Table II, SP-3004 headers) are already checked in
 * validate-published.ts; SP3004_BETA is reused here only to validate the independent reference integrator itself.
 *
 * Sources
 *  [NACA1135] Ames Research Staff, "Equations, Tables, and Charts for Compressible Flow", NACA Report 1135 (1953),
 *             NTRS 19930091059 — oblique-shock θ–β–M relation, Rankine–Hugoniot jumps on the normal component,
 *             detachment (maximum-deflection) condition, Rayleigh pitot formula.
 *  [Billig67] F. S. Billig, "Shock-wave shapes around spherical- and cylindrical-nosed bodies",
 *             J. Spacecraft Rockets 4(6):822–823 (1967) — Δ/R = 0.143 exp(3.24/M²) (sphere), 0.386 exp(4.67/M²) (cylinder).
 *  [Lees55]   L. Lees, "Hypersonic flow", Proc. 5th Int. Aeronautical Conf., IAS (1955) — modified Newtonian
 *             Cp = Cp,max sin²θ, Cp,max = (p₀₂/p∞ − 1)/(γM²/2); see also J. D. Anderson, "Hypersonic and High-Temperature
 *             Gas Dynamics", ch. 3. Sphere and cone force integrals are evaluated analytically below.
 *  [TM33]     G. I. Taylor & J. W. Maccoll, "The air pressure on a cone moving at high speeds", Proc. R. Soc. A 139:278 (1933);
 *             ODE as in J. D. Anderson, "Modern Compressible Flow", ch. 10; tables in J. L. Sims, NASA SP-3004 (1964).
 */
import assert from "node:assert/strict";
import { test } from "node:test";
import { atmosphere } from "./atmosphere";
import { DEG, RAD, betaFromThetaM, billigStandoff, billigStandoffCylinder, coneSurfaceCp, coneSurfaceStateExact, maxTheta, newtonianCpMax, obliqueShock, solveConeShock } from "./math";
import { panelAero } from "./panel";
import { billigStandoffOverR } from "./refinement";
import { DEFAULT_PARAMS, SURFACE_ID, type BuiltVehicle, type DesignParams, type TriMesh } from "./types";
import { SP3004_BETA } from "./validate-published";
import { generateAeroDatabase } from "./aerodb";
import { cpMaxNewton, obliqueM2, thetaMax as datcomThetaMax, weakBeta } from "./datcom/gasdyn";

const G = 1.4;
const rel = (a: number, b: number) => Math.abs(a - b) / Math.max(Math.abs(b), 1e-300);
const near = (got: number, ref: number, tol: number, msg: string) => assert.ok(Number.isFinite(got) && rel(got, ref) <= tol, `${msg}: got ${got}, ref ${ref}, rel ${rel(got, ref).toExponential(2)} > ${tol}`);

/* ------------------------------------------------------------------ closed forms (independent of math.ts) */

/** [NACA1135] tan θ = 2 cot β (M² sin²β − 1) / (M² (γ + cos 2β) + 2). */
const thetaClosed = (M: number, b: number, g = G) => Math.atan((2 / Math.tan(b)) * (M * M * Math.sin(b) ** 2 - 1) / (M * M * (g + Math.cos(2 * b)) + 2));
/** [NACA1135] detachment: sin²β* = [ (γ+1)M²/4 − 1 + √((γ+1)(1 + (γ−1)M²/2 + (γ+1)M⁴/16)) ] / (γ M²). */
const betaDetach = (M: number, g = G) => {
  const M2 = M * M;
  const s2 = ((g + 1) * M2 / 4 - 1 + Math.sqrt((g + 1) * (1 + ((g - 1) * M2) / 2 + ((g + 1) * M2 * M2) / 16))) / (g * M2);
  return Math.asin(Math.sqrt(s2));
};
/** [NACA1135] normal-component Rankine–Hugoniot. */
const rhClosed = (Mn: number, g = G) => ({
  p: (2 * g * Mn * Mn - (g - 1)) / (g + 1),
  rho: ((g + 1) * Mn * Mn) / ((g - 1) * Mn * Mn + 2),
  Mn2: Math.sqrt((1 + ((g - 1) / 2) * Mn * Mn) / (g * Mn * Mn - (g - 1) / 2)),
});
/** [NACA1135] Rayleigh pitot p₀₂/p₁ = [((γ+1)M/2)^2 ]^{γ/(γ−1)} · [ (γ+1)/(2γM² − (γ−1)) ]^{1/(γ−1)}. */
const pitotClosed = (M: number, g = G) => (((g + 1) * M * M) / 2) ** (g / (g - 1)) * ((g + 1) / (2 * g * M * M - (g - 1))) ** (1 / (g - 1));
const cpMaxClosed = (M: number, g = G) => (pitotClosed(M, g) - 1) / (0.5 * g * M * M);

/* ------------------------------------------------------------------ oblique shock θ–β–M */

test("NACA 1135: θ–β–M weak branch — betaFromThetaM inverts the closed form", () => {
  let n = 0;
  for (const M of [1.5, 2, 3, 5, 8, 12, 20]) {
    const mu = Math.asin(1 / M);
    const bStar = betaDetach(M);
    for (let k = 1; k <= 12; k++) {
      const b = mu + ((bStar - mu) * k) / 13;
      const th = thetaClosed(M, b);
      if (th >= 0.98 * thetaClosed(M, bStar)) continue; // code's detachment guard sits at 0.999 of a sampled max
      const got = betaFromThetaM(M, th, G);
      assert.ok(Math.abs(got - b) < 1e-7, `M=${M} θ=${(th * RAD).toFixed(4)}°: β ${got * RAD}° vs ${b * RAD}°`);
      n++;
    }
  }
  assert.ok(n > 70);
});

test("NACA 1135: detachment angle — maxTheta vs the closed-form β* (and θ(β*) is the true maximum)", () => {
  for (const M of [1.5, 2, 3, 5, 10, 20]) {
    const bs = betaDetach(M);
    const thMax = thetaClosed(M, bs);
    // β* is a stationary point of the closed form (independent check of the formula transcription)
    assert.ok(thetaClosed(M, bs - 1e-4) < thMax && thetaClosed(M, bs + 1e-4) < thMax, `β* not a maximum at M=${M}`);
    const mt = maxTheta(M, G);
    near(mt.theta, thMax, 5e-4, `θmax M=${M}`);
    assert.ok(Math.abs(mt.beta - bs) < 1.5 * DEG, `β at θmax M=${M}: ${mt.beta * RAD} vs ${bs * RAD}`);
    assert.ok(Number.isNaN(betaFromThetaM(M, thMax * 1.001, G)), `detached beyond θmax at M=${M}`);
  }
});

test("NACA 1135: Rankine–Hugoniot jumps on the normal component — obliqueShock", () => {
  for (const M of [1.8, 2.5, 4, 8, 15]) {
    for (const thDeg of [2, 5, 10, 15, 20]) {
      const th = thDeg * DEG;
      const b = betaFromThetaM(M, th, G);
      if (!Number.isFinite(b)) continue;
      const s = obliqueShock(M, b, G);
      const r = rhClosed(M * Math.sin(b));
      near(s.p2p1, r.p, 1e-12, `p2/p1 M=${M} θ=${thDeg}`);
      near(s.r2r1, r.rho, 1e-12, `ρ2/ρ1 M=${M} θ=${thDeg}`);
      near(s.M2, r.Mn2 / Math.sin(b - th), 1e-6, `M2 M=${M} θ=${thDeg}`);
      near(s.theta, th, 1e-7, `θ round trip M=${M} θ=${thDeg}`);
    }
  }
});

test("NACA 1135 limits: ρ₂/ρ₁ → (γ+1)/(γ−1); hypersonic slender wedge β/θ → (γ+1)/2", () => {
  near(obliqueShock(1e4, Math.PI / 2, G).r2r1, (G + 1) / (G - 1), 1e-6, "strong-shock density ratio");
  // small-disturbance limit: Mθ ≫ 1 and θ ≪ 1 (θ = 1°, M = 3000) → O(θ) relative correction
  const th = 1 * DEG;
  near(betaFromThetaM(3000, th, G) / th, (G + 1) / 2, 0.02, "β/θ hypersonic limit");
});

test("NACA 1135 cross-implementation: app kernels (math.ts) and the DATCOM estimator (datcom/gasdyn.ts) agree with the closed forms", () => {
  // datcom/gasdyn.ts (added on main in 268f490) is a separate implementation of the same NACA 1135 relations; the
  // aero database and the DATCOM panel must not drift apart.
  for (const M of [1.5, 2, 3, 5, 8, 12, 20]) {
    const thMax = thetaClosed(M, betaDetach(M));
    near(datcomThetaMax(M, G), thMax, 1e-6, `DATCOM θmax M=${M}`);
    for (const thDeg of [1, 5, 10, 20]) {
      const th = thDeg * DEG;
      if (th >= 0.98 * thMax) continue;
      const bApp = betaFromThetaM(M, th, G);
      const bDat = weakBeta(M, th, G) as number;
      near(thetaClosed(M, bDat), th, 1e-8, `DATCOM weakBeta inverts θ–β–M, M=${M} θ=${thDeg}`);
      assert.ok(Math.abs(bApp - bDat) < 1e-7, `β app ${bApp * RAD} vs DATCOM ${bDat * RAD}, M=${M} θ=${thDeg}`);
      near(obliqueShock(M, bApp, G).M2, obliqueM2(M, bDat, th, G), 1e-6, `M2 app vs DATCOM, M=${M} θ=${thDeg}`);
    }
    near(cpMaxNewton(M, G), cpMaxClosed(M), 1e-10, `DATCOM Cp,max M=${M}`);
  }
});

/* ------------------------------------------------------------------ Billig standoff */

test("Billig 1967: sphere and cylinder stand-off correlations, both code paths", () => {
  for (const M of [1.2, 1.5, 2, 3, 5, 8, 12, 20, 30]) {
    const sph = 0.143 * Math.exp(3.24 / (M * M));
    const cyl = 0.386 * Math.exp(4.67 / (M * M));
    near(billigStandoff(M), sph, 1e-12, `sphere Δ/R M=${M}`);
    near(billigStandoffOverR(M), sph, 1e-12, `refinement.ts sphere Δ/R M=${M}`);
    near(billigStandoffCylinder(M), cyl, 1e-12, `cylinder Δ/R M=${M}`);
    assert.ok(billigStandoffCylinder(M) > billigStandoff(M), "2-D body stands the shock off further than a sphere");
  }
  // M → ∞ limits are the correlation constants; monotone decreasing in M
  near(billigStandoff(1e6), 0.143, 1e-9, "sphere M→∞");
  near(billigStandoffCylinder(1e6), 0.386, 1e-9, "cylinder M→∞");
  for (let M = 1.1; M < 20; M += 0.5) assert.ok(billigStandoff(M + 0.5) < billigStandoff(M));
});

/* ------------------------------------------------------------------ modified Newtonian */

test("Lees modified Newtonian: Cp,max = (p₀₂/p∞ − 1)/q from the Rayleigh pitot formula, and its M→∞ limit", () => {
  for (const M of [2, 3, 5, 8, 12, 20]) near(newtonianCpMax(M, G), cpMaxClosed(M), 1e-10, `Cp,max M=${M}`);
  // M→∞ limit of the Rayleigh pitot closed form divided by q = γM²/2: Cp,max → ((γ+1)²/(4γ))^{γ/(γ−1)} · 4/(γ+1)
  const lim = (((G + 1) ** 2 / (4 * G)) ** (G / (G - 1)) * 4) / (G + 1);
  near(newtonianCpMax(1e5, G), lim, 1e-6, "Cp,max M→∞");
});

/** Closed triangulated body of revolution about +X: r(x) given at stations, outward normals, base disk at the end. */
function bodyOfRevolution(xs: number[], rs: number[], nPhi: number): TriMesh {
  const pos: number[] = [];
  const idx: number[] = [];
  const ring: number[][] = [];
  for (let i = 0; i < xs.length; i++) {
    const row: number[] = [];
    const n = rs[i] < 1e-12 ? 1 : nPhi;
    for (let k = 0; k < n; k++) {
      const ph = (2 * Math.PI * k) / nPhi;
      row.push(pos.length / 3);
      pos.push(xs[i], rs[i] * Math.cos(ph), rs[i] * Math.sin(ph));
    }
    ring.push(row);
  }
  const at = (row: number[], k: number) => row[row.length === 1 ? 0 : k % nPhi];
  for (let i = 0; i + 1 < xs.length; i++) {
    for (let k = 0; k < nPhi; k++) {
      const a = at(ring[i], k), b = at(ring[i], k + 1), c = at(ring[i + 1], k), d = at(ring[i + 1], k + 1);
      if (a !== b) idx.push(a, b, d);
      idx.push(a, d, c);
    }
  }
  const last = ring[ring.length - 1];
  const centre = pos.length / 3;
  pos.push(xs[xs.length - 1], 0, 0);
  for (let k = 0; k < nPhi; k++) idx.push(centre, at(last, k), at(last, k + 1));
  // orient every triangle outward (centroid direction from the axis / +X for the base)
  const P = new Float64Array(pos);
  for (let t = 0; t < idx.length; t += 3) {
    const [i, j, k] = [idx[t] * 3, idx[t + 1] * 3, idx[t + 2] * 3];
    const ux = P[j] - P[i], uy = P[j + 1] - P[i + 1], uz = P[j + 2] - P[i + 2];
    const vx = P[k] - P[i], vy = P[k + 1] - P[i + 1], vz = P[k + 2] - P[i + 2];
    const n = [uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx];
    const cy = (P[i + 1] + P[j + 1] + P[k + 1]) / 3, cz = (P[i + 2] + P[j + 2] + P[k + 2]) / 3;
    const isBase = Math.abs(P[i] - xs[xs.length - 1]) < 1e-12 && Math.abs(P[j] - xs[xs.length - 1]) < 1e-12 && Math.abs(P[k] - xs[xs.length - 1]) < 1e-12;
    const out = isBase ? n[0] : n[1] * cy + n[2] * cz + (Math.hypot(cy, cz) < 1e-9 ? -n[0] : 0);
    if (out < 0) [idx[t + 1], idx[t + 2]] = [idx[t + 2], idx[t + 1]];
  }
  const nt = idx.length / 3;
  const surfaces = new Uint8Array(nt);
  for (let t = 0; t < nt; t++) {
    const xsOf = [idx[3 * t], idx[3 * t + 1], idx[3 * t + 2]].map((v) => P[v * 3]);
    if (xsOf.every((x) => Math.abs(x - xs[xs.length - 1]) < 1e-12)) surfaces[t] = SURFACE_ID.base;
  }
  return { positions: P, indices: new Uint32Array(idx), surfaces };
}

const newtonParams = (M: number, length: number, span: number): DesignParams => ({ ...DEFAULT_PARAMS, family: "caret", aeroMethod: "newtonian", lockFlight: false, mach: M, flightMach: M, length, span, cgFrac: 0.5, altKm: 0 });

test("Lees modified Newtonian on a hemisphere: panelAero C_D,pressure = Cp,max / 2 (ref πR²)", () => {
  // ∫ Cp,max cos²φ · cos φ · 2πR² sin φ dφ over 0..π/2, divided by πR²  =  Cp,max/2.
  const R = 1;
  const nTh = 60;
  const xs: number[] = [];
  const rs: number[] = [];
  for (let i = 0; i <= nTh; i++) {
    const phi = (Math.PI / 2) * (i / nTh);
    xs.push(R - R * Math.cos(phi));
    rs.push(R * Math.sin(phi));
  }
  const mesh = bodyOfRevolution(xs, rs, 120);
  for (const M of [4, 8, 16]) {
    const p = newtonParams(M, R, 2 * R);
    // α = 0 only: at α ≠ 0 a hemisphere-with-base is not the full windward hemisphere, so Cp,max/2 is no longer exact.
    const r = panelAero(mesh, p, atmosphere(0, M, G), 0, 0, Math.PI * R * R, R);
    near(r.cdWave, newtonianCpMax(M, G) / 2, 2e-3, `hemisphere CD M=${M}`); // faceting error ≈ 7e-4 at 60×120
    assert.ok(Math.abs(r.cl) < 1e-9 && Math.abs(r.cy) < 1e-9, "no lift / side force at α=0");
  }
});

test("Lees modified Newtonian on a sharp cone: CA = Cp,max(sin²θc cos²α + ½cos²θc sin²α), CN = Cp,max cos²θc sinα cosα", () => {
  // Analytic integral of Cp,max (n̂·v̂)² over a cone with no shadowed panels (α < θc), ref base area πR².
  for (const thDeg of [10, 20]) {
    const th = thDeg * DEG;
    const L = 1;
    const R = L * Math.tan(th);
    const ns = 8;
    const xs = Array.from({ length: ns + 1 }, (_, i) => (L * i) / ns);
    const rs = xs.map((x) => x * Math.tan(th));
    const mesh = bodyOfRevolution(xs, rs, 360);
    for (const M of [6, 12]) {
      const cpm = newtonianCpMax(M, G);
      for (const aDeg of [0, 4, thDeg / 2]) {
        const a = aDeg * DEG;
        const r = panelAero(mesh, newtonParams(M, L, 2 * R), atmosphere(0, M, G), aDeg, 0, Math.PI * R * R, L);
        const caRef = cpm * (Math.sin(th) ** 2 * Math.cos(a) ** 2 + 0.5 * Math.cos(th) ** 2 * Math.sin(a) ** 2);
        const cnRef = cpm * Math.cos(th) ** 2 * Math.sin(a) * Math.cos(a);
        near(r.ca - r.cdBase, caRef, 5e-4, `cone ${thDeg}° M=${M} α=${aDeg} CA`);
        if (aDeg > 0) near(r.cnA, cnRef, 5e-4, `cone ${thDeg}° M=${M} α=${aDeg} CN`);
      }
    }
  }
});

test("Lees modified Newtonian through main's generateAeroDatabase: cone CN, CA, CL and CLα [1/rad] vs closed form", async () => {
  const th = 15 * DEG;
  const L = 1;
  const R = L * Math.tan(th);
  const xs = Array.from({ length: 9 }, (_, i) => (L * i) / 8);
  const mesh = bodyOfRevolution(xs, xs.map((x) => x * Math.tan(th)), 360);
  const M = 10;
  const Sb = Math.PI * R * R;
  // Fake BuiltVehicle: aerodb.ts only needs params, mesh, aero.planformArea (= Sref) and quality.volume (meta.mass_kg) when δ stays at params.elevonDeg.
  const built = { params: { ...newtonParams(M, L, 2 * R), alphaDeg: 4 }, mesh, aero: { planformArea: Sb }, quality: { volume: (Math.PI * R * R * L) / 3 } } as unknown as BuiltVehicle;
  const db = await generateAeroDatabase(built, {
    machMin: M, machMax: M, machStep: 1, alphaMin: 0, alphaMax: 8, alphaStep: 2, betaMin: 0, betaMax: 0, betaStep: 1,
    deltaMin: 0, deltaMax: 0, deltaStep: 1, method: "newtonian", eulerSymmetry: false, dynamicDerivatives: false, altKm: 0,
  });
  assert.equal(db.meta.sRef_m2, Sb, "Sref = aero.planformArea");
  const cpm = newtonianCpMax(M, G);
  const caOf = (a: number) => cpm * (Math.sin(th) ** 2 * Math.cos(a) ** 2 + 0.5 * Math.cos(th) ** 2 * Math.sin(a) ** 2) + 1 / (M * M); // + Love base Cp = −1/M²
  const cnOf = (a: number) => cpm * Math.cos(th) ** 2 * Math.sin(a) * Math.cos(a);
  const clOf = (a: number) => cnOf(a) * Math.cos(a) - caOf(a) * Math.sin(a); // pressure part; friction only adds to CD
  for (const p of db.points) {
    const a = p.alpha_deg * DEG;
    near(p.ca, caOf(a), 3e-3, `aerodb CA α=${p.alpha_deg}`);
    if (p.alpha_deg > 0) near(p.cl, clOf(a), 5e-3, `aerodb CL α=${p.alpha_deg}`);
    assert.ok(Math.abs(p.cy) < 1e-9 && Math.abs(p.cll) < 1e-9 && Math.abs(p.cn) < 1e-9, "axisymmetric body at β=0: no lateral loads");
    const ia = db.axes.alpha_deg.indexOf(p.alpha_deg);
    if (ia > 0 && ia < db.axes.alpha_deg.length - 1) {
      // main's static derivatives are table central differences PER RADIAN (aerodb.ts header, meta.units.derivatives)
      const h = 2 * DEG;
      const ref = (clOf(a + h) - clOf(a - h)) / (2 * h);
      near(p.cla, ref, 1e-2, `aerodb CLα [1/rad] α=${p.alpha_deg}`);
      const exact = (clOf(a + 1e-6) - clOf(a - 1e-6)) / 2e-6; // analytic slope; table stencil error O(h²)
      near(p.cla, exact, 2e-2, `aerodb CLα vs analytic slope α=${p.alpha_deg}`);
    }
  }
  assert.equal(db.meta.units.derivatives, "1/rad");
});

/* ------------------------------------------------------------------ Taylor–Maccoll cone */

/**
 * Independent RK4 of the Taylor–Maccoll ODE [TM33] in V/V_max form, written from the equation (Anderson MCF ch. 10):
 *   (γ−1)/2 [1 − V_r² − V_θ²] (2V_r + V_θ cot θ + V_θ') − V_θ (V_r V_θ + V_θ V_θ') = 0,  V_θ = dV_r/dθ.
 * Integrates from the shock (oblique-shock state at β) inward until V_θ = 0, which defines the cone angle.
 */
function tmIndependent(M: number, beta: number, g = G, dth = 2e-5) {
  const Vp = 1 / Math.sqrt(2 / ((g - 1) * M * M) + 1); // V∞/Vmax
  const Mn = M * Math.sin(beta);
  const r = rhClosed(Mn, g);
  const M2 = r.Mn2 / Math.sin(beta - thetaClosed(M, beta, g));
  const V2 = 1 / Math.sqrt(2 / ((g - 1) * M2 * M2) + 1);
  const del = thetaClosed(M, beta, g);
  let vr = V2 * Math.cos(beta - del);
  let vt = -V2 * Math.sin(beta - del);
  let th = beta;
  const f = (t: number, a: number, b: number) => {
    const A = ((g - 1) / 2) * (1 - a * a - b * b);
    const bp = (b * b * a - A * (2 * a + b / Math.tan(t))) / (A - b * b);
    return [b, bp];
  };
  while (th > 1e-4) {
    const h = -dth;
    const k1 = f(th, vr, vt);
    const k2 = f(th + h / 2, vr + (h / 2) * k1[0], vt + (h / 2) * k1[1]);
    const k3 = f(th + h / 2, vr + (h / 2) * k2[0], vt + (h / 2) * k2[1]);
    const k4 = f(th + h, vr + h * k3[0], vt + h * k3[1]);
    const nvr = vr + (h / 6) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
    const nvt = vt + (h / 6) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
    if (nvt >= 0) {
      const s = vt / (vt - nvt);
      const vrc = vr + (nvr - vr) * s;
      // surface pressure: isentropic from the post-shock state, p₀₂/p∞ from RH and V/Vmax ↔ T/T₀ = 1 − V²
      const p2p1 = r.p;
      const T2 = 1 - V2 * V2;
      const Tc = 1 - vrc * vrc;
      const pc = p2p1 * (Tc / T2) ** (g / (g - 1));
      return { cone: th + h * s, cp: (pc - 1) / (0.5 * g * M * M), Vp };
    }
    th += h;
    vr = nvr;
    vt = nvt;
  }
  return null;
}

function coneFromIndependent(M: number, cone: number) {
  let lo = Math.asin(1 / M) * (1 + 1e-9);
  let hi = betaDetach(M) - 1e-6;
  for (let i = 0; i < 50; i++) {
    const mid = 0.5 * (lo + hi);
    const r = tmIndependent(M, mid);
    if (!r || r.cone < cone) lo = mid;
    else hi = mid;
  }
  const beta = 0.5 * (lo + hi);
  return { beta, ...(tmIndependent(M, beta) as { cone: number; cp: number }) };
}

test("Taylor–Maccoll: independent RK4 reference reproduces NASA SP-3004 shock angles (validates the reference)", () => {
  // a subset spanning M 1.75–20 and θc 2.5°–30°
  const cases = SP3004_BETA.filter((_, i) => i % 5 === 0);
  for (const [c, M, bTab] of cases) {
    const ref = coneFromIndependent(M, c * DEG);
    assert.ok(Math.abs(ref.beta * RAD - bTab) < 0.01, `SP-3004 θc=${c}° M=${M}: independent β ${ref.beta * RAD} vs table ${bTab}`);
  }
});

test("Taylor–Maccoll: code cone shock angle and surface Cp vs the independent integration", () => {
  for (const M of [2, 3, 5, 8, 12]) {
    for (const cDeg of [5, 10, 15, 20, 30]) {
      const cone = cDeg * DEG;
      if (cDeg === 30 && M < 3) continue; // near detachment
      const ref = coneFromIndependent(M, cone);
      const b = solveConeShock(M, cone, G);
      assert.ok(Math.abs(b - ref.beta) * RAD < 0.01, `β M=${M} θc=${cDeg}: ${b * RAD} vs ${ref.beta * RAD}`);
      near(coneSurfaceStateExact(M, cone, G).cp, ref.cp, 2e-3, `exact Cp M=${M} θc=${cDeg}`);
      near(coneSurfaceCp(M, cone, G), ref.cp, 5e-3, `cached/interpolated Cp (panel path) M=${M} θc=${cDeg}`);
    }
  }
});

test("Taylor–Maccoll limits: conical shock lies between the Mach wave and the 2-D wedge shock; θc → 0 gives β → μ", () => {
  for (const M of [2, 4, 8]) {
    for (const cDeg of [5, 10, 15]) {
      const b = solveConeShock(M, cDeg * DEG, G);
      assert.ok(b > Math.asin(1 / M), "β > μ");
      assert.ok(b < betaFromThetaM(M, cDeg * DEG, G), "cone shock weaker than the wedge shock of the same angle");
    }
    assert.ok(Math.abs(solveConeShock(M, 1e-4, G) - Math.asin(1 / M)) < 2e-3, `β→μ at M=${M}`);
  }
});
