/**
 * Accuracy audit, area 1: atmosphere, panel method, aero database.
 * Run: node --import ./scripts/ts-register.mjs src/lib/waverider/audit-aero.test.ts   (npm run test:audit)
 *
 * Every reference is a closed form (derived in the comment) or a cited table value.
 */
import assert from "node:assert/strict";
import { atmosphere } from "./atmosphere";
import { buildVehicle } from "./generate";
import { DEG, newtonianCpMax } from "./math";
import { panelAero, type RateState } from "./panel";
import { defaultAeroDbSpec, derivsToConventional, generateAeroDatabase, panelToConventional } from "./aerodb";
import { DEFAULT_PARAMS, type DesignParams } from "./types";
import { flatPlateMesh, rotateX90, translate } from "./audit.fixtures";

const rel = (a: number, b: number) => Math.abs(a - b) / Math.max(Math.abs(b), 1e-300);
const close = (a: number, b: number, tol: number, msg: string) => assert.ok(rel(a, b) <= tol, `${msg}: got ${a}, ref ${b}, rel ${rel(a, b).toExponential(2)} > ${tol}`);
const near = (a: number, b: number, tol: number, msg: string) => assert.ok(Math.abs(a - b) <= tol, `${msg}: got ${a}, ref ${b}, |Δ| ${Math.abs(a - b).toExponential(2)} > ${tol}`);
const log = (s: string) => console.log(`  ${s}`);

/* ---- A1. US Standard Atmosphere 1976 ----
 * Reference: U.S. Standard Atmosphere, 1976 (NOAA-S/T 76-1562, NASA-TM-X-74335), Part 4 Table I, geometric altitude,
 * as reproduced at https://www.pdas.com/bigtables.html (5 significant figures). Tolerance 0.05 % (T) / 0.1 % (p, ρ):
 * the table is rounded to 5 figures. */
{
  const ref: [number, number, number, number][] = [
    // Z km, T K, p Pa, rho kg/m^3
    [10, 223.252, 2.65e4, 4.1351e-1],
    [30, 226.509, 1.197e3, 1.841e-2],
    [50, 270.65, 7.9779e1, 1.0269e-3],
    [70, 219.585, 5.2209, 8.2829e-5],
    [80, 198.639, 1.0525, 1.8458e-5],
  ];
  for (const [z, T, p, rho] of ref) {
    const a = atmosphere(z, 1);
    close(a.T, T, 5e-4, `US76 T(${z} km)`);
    close(a.p, p, 1e-3, `US76 p(${z} km)`);
    close(a.rho, rho, 1e-3, `US76 rho(${z} km)`);
  }
  // Model limit (documented): clamped to 0–84.9 km. Above that it returns the 84.9 km state.
  assert.equal(atmosphere(120, 5).rho, atmosphere(84.9, 5).rho, "altitude cap at 84.9 km");
  assert.equal(atmosphere(-2, 5).rho, atmosphere(0, 5).rho, "altitude floor at 0 km");
  log("A1 US76 10/30/50/70/80 km within 0.1 %; cap 84.9 km confirmed");
}

/* ---- A2. Modified-Newtonian Cpmax = (p02/p∞ − 1)/(γM²/2), Rayleigh pitot. Hypersonic limit
 * Cpmax(∞) = 4/(γ+1)·[(γ+1)²/(4γ)]^{γ/(γ−1)} = 1.8394 for γ = 1.4 (Anderson, Hypersonic and High-Temperature Gas
 * Dynamics, ch. 3, modified Newtonian; closed form re-derived here). */
{
  const g = 1.4;
  const lim = (4 / (g + 1)) * ((g + 1) ** 2 / (4 * g)) ** (g / (g - 1));
  close(newtonianCpMax(1e4, g), lim, 1e-6, "Cpmax M→∞");
  for (const M of [2, 5, 10]) {
    const M2 = M * M;
    const p02 = (((g + 1) ** 2 * M2) / (4 * g * M2 - 2 * (g - 1))) ** (g / (g - 1)) * ((1 - g + 2 * g * M2) / (g + 1));
    close(newtonianCpMax(M, g), (p02 - 1) / (0.5 * g * M2), 1e-12, `Cpmax(M=${M})`);
  }
  log(`A2 Cpmax(∞) = ${newtonianCpMax(1e4, g).toFixed(5)} vs ${lim.toFixed(5)}`);
}

/* Newtonian flat plate, panel frame X aft, Y span, Z up. */
const L = 4;
const B = 2;
const T = 1e-5;
const M = 10;
const plateParams = (h: number): DesignParams => ({ ...DEFAULT_PARAMS, family: "caret", aeroMethod: "newtonian", lockFlight: false, flightMach: M, mach: M, length: L, span: B, cgFrac: h, altKm: 30 });
const atm = atmosphere(30, M, 1.4);
const Cpmax = newtonianCpMax(M, 1.4);
const S = L * B;

/* ---- A3. Mach clamp ≥ 1.05 (panel.ts) ---- */
{
  const plate = flatPlateMesh(L, B, T, 4, 2);
  const p = plateParams(0.5);
  const at = (m: number) => panelAero(plate, { ...p, flightMach: m }, atm, 10, 0, S, L, undefined, true).cpMax;
  assert.equal(at(0.5), at(1.05), "M < 1.05 clamps to 1.05");
  assert.equal(at(Number.NaN), at(1.05), "NaN Mach clamps to 1.05");
  log("A3 Mach clamp 1.05 confirmed (also NaN)");
}

/* ---- A4. Newtonian flat plate (Newtonian flat plate, Anderson, Hypersonic and High-Temperature Gas Dynamics, ch. 3; closed form: CN = Cpmax sin²α,
 * CL = Cpmax sin²α cosα, CD = Cpmax sin³α; L/D = cot α). Pressure moment about x_cg = h·L for uniform Cp on the lower
 * face, centroid L/2: Cm = −(0.5 − h)·Cpmax sin²α (nose-up positive). Friction is added to CD only and is
 * removed with cdFric; the thin trailing face carries the −1/M² base Cp and is removed with cdBase; the thin leading
 * face is added analytically. Tolerance 1e-6 (forces), 1e-5 (Cm: edge-face moment arm not modelled). */
{
  const h = 0.3;
  const plate = flatPlateMesh(L, B, T, 8, 4);
  const p = plateParams(h);
  for (const a of [5, 10, 20, 30, -10]) {
    const r = panelAero(plate, p, atm, a, 0, S, L, undefined, true);
    const sa = Math.sin(Math.abs(a) * DEG);
    const ca = Math.cos(a * DEG);
    const sgn = Math.sign(a);
    const cn = Cpmax * sa * sa;
    // the 1e-5 L leading-edge face (normal −X) is windward too: axial force Cpmax cos²α · (t b / S)
    const caEdge = (Cpmax * ca * ca * T * B) / S;
    close(r.cl, sgn * cn * ca - Math.sin(a * DEG) * caEdge, 1e-6, `plate CL α=${a}`);
    close(r.cd - r.cdFric - r.cdBase, cn * sa + ca * caEdge, 1e-6, `plate pressure CD α=${a}`);
    close(r.cm, -sgn * (0.5 - h) * cn, 1e-5, `plate Cm α=${a}`);
    assert.ok(r.cdFric > 0 && r.cdFric < 0.01, `friction added to CD (${r.cdFric})`);
    assert.equal(r.cy, 0);
  }
  const r10 = panelAero(plate, p, atm, 10, 0, S, L, undefined, true);
  log(`A4 plate α=10: CL ${r10.cl.toFixed(5)} (ref ${(Cpmax * Math.sin(10 * DEG) ** 2 * Math.cos(10 * DEG)).toFixed(5)}), CDp ${(r10.cd - r10.cdFric - r10.cdBase).toFixed(5)}, Cm ${r10.cm.toFixed(5)}, cdFric ${r10.cdFric.toFixed(5)}`);
}

/* ---- A5. β sign and lateral moment normalisation: vertical Newtonian fin (plate rotated 90° about X), raised z0.
 * panel flowDir is (cosα cosβ, sinβ, sinα cosβ): β > 0 sends the flow toward +Y, so the −Y face is windward and the
 * normal force is +Y: CY = Cpmax sin²β cosβ, CD = Cpmax sin³β (α = 0, CY along the rotated +Y).
 * Body moments on S·L: Cn = Mz/(qSL) = (0.5 − h)·Cpmax sin²β (force at x = L/2), Cll = Mx/(qSL) = −(z0/L)·Cpmax sin²β.
 * Conventional axes (x fwd, y right, z down; β_conv = −β_panel): CYβ < 0, Cnβ = (0.5−h)Cpmax·2 sinβ cosβ·L/b-scaled > 0
 * for h > 0.5 is unstable / h < 0.5 stable — weathervane. */
{
  const h = 0.3;
  const z0 = 0.5;
  const fin = translate(rotateX90(flatPlateMesh(L, B, T, 8, 4)), [0, 0, z0]);
  const p = plateParams(h);
  for (const b of [4, 10, -6]) {
    const r = panelAero(fin, p, atm, 0, b, S, L, undefined, true);
    const sb = Math.sin(Math.abs(b) * DEG);
    const sgn = Math.sign(b);
    const fy = sgn * Cpmax * sb * sb;
    close(r.cy, fy * Math.cos(b * DEG), 2e-3, `fin CY β=${b}`);
    close(r.cn, (0.5 - h) * fy, 2e-3, `fin Cn β=${b}`);
    close(r.cll, -(z0 / L) * fy, 2e-3, `fin Cll β=${b}`);
    assert.ok(Math.abs(r.cl) < 1e-6, "fin has no lift at α = 0");
  }
  // derivatives at small β by central difference, then convert
  const d = 0.5;
  const rp = panelAero(fin, p, atm, 0, 2 + d, S, L, undefined, true);
  const rm = panelAero(fin, p, atm, 0, 2 - d, S, L, undefined, true);
  const cyb = (rp.cy - rm.cy) / (2 * d * DEG);
  const cnb = (rp.cn - rm.cn) / (2 * d * DEG);
  const clb = (rp.cll - rm.cll) / (2 * d * DEG);
  const conv = derivsToConventional({ cyb, cnb, clb, cmq: 0, clq: 0, clp: 0, cnp: 0, cyp: 0, cnr: 0, clr: 0, cyr: 0 }, L, B);
  assert.ok(conv.CYb < 0, `conventional CYβ < 0 (${conv.CYb})`);
  assert.ok(conv.Cnb > 0, `conventional Cnβ > 0 for a fin aft of the CG (${conv.Cnb})`);
  assert.ok(conv.Clb < 0, `conventional Clβ < 0 for a fin above the CG (${conv.Clb})`);
  const m = panelToConventional({ cm: 0.01, cn: rp.cn, cll: rp.cll }, L, B);
  close(m.Cn, (-rp.cn * L) / B, 1e-15, "Cn = −cn L/b");
  close(m.Cl, (-rp.cll * L) / B, 1e-15, "Cl = −cll L/b");
  log(`A5 fin β=2°: panel cyb ${cyb.toFixed(4)} cnb ${cnb.toFixed(4)} clb ${clb.toFixed(4)} → conventional CYβ ${conv.CYb.toFixed(4)} Cnβ ${conv.Cnb.toFixed(4)} Clβ ${conv.Clb.toFixed(4)}`);
}

/* ---- A6. Rate derivatives, Newtonian flat plate, closed form.
 * Local velocity V v̂ − ω×r; Newtonian normal force ∝ ρ V_n². With ω = (0, q, 0): V_n = V sinα + q (x − x_cg), so
 *   Cmq (q̂ = qL/2V) = −4 Cpmax sinα [1/12 + (0.5 − h)²],   CLq = 4 Cpmax sinα cosα (0.5 − h).
 * With ω = (p, 0, 0): V_n = V sinα − p y  ⇒  Clp (p̂ = pb/2V, Cll on S·L) = −Cpmax sinα · b/(3L).
 * With ω = (0, 0, r): the perturbation is tangential to the plate, V_n unchanged ⇒ Clr = 0 exactly.
 * Discretisation: centroid quadrature of (x − x_cg)² on nx = 40 strips is exact to (Δx²/12)/(L²/12) = 1/1600 → tol 2e-3. */
{
  const h = 0.3;
  const plate = flatPlateMesh(L, B, T, 40, 20);
  const p = plateParams(h);
  const a = 10;
  const sa = Math.sin(a * DEG);
  const ca = Math.cos(a * DEG);
  const hat = 0.004;
  const V = atm.V;
  const pair = (w: (s: number) => RateState) => {
    const up = panelAero(plate, p, atm, a, 0, S, L, w(1), true);
    const dn = panelAero(plate, p, atm, a, 0, S, L, w(-1), true);
    return { up, dn };
  };
  const q = pair((s) => ({ p: 0, q: (s * hat * 2 * V) / L, r: 0 }));
  const pp = pair((s) => ({ p: (s * hat * 2 * V) / B, q: 0, r: 0 }));
  const rr = pair((s) => ({ p: 0, q: 0, r: (s * hat * 2 * V) / B }));
  const cmq = (q.up.cm - q.dn.cm) / (2 * hat);
  const clq = (q.up.cl - q.dn.cl) / (2 * hat);
  const clp = (pp.up.cll - pp.dn.cll) / (2 * hat);
  const clr = (rr.up.cll - rr.dn.cll) / (2 * hat);
  const cmqRef = -4 * Cpmax * sa * (1 / 12 + (0.5 - h) ** 2);
  const clqRef = 4 * Cpmax * sa * ca * (0.5 - h);
  const clpRef = (-Cpmax * sa * B) / (3 * L);
  close(cmq, cmqRef, 3e-3, "plate Cmq");
  close(clq, clqRef, 3e-3, "plate CLq");
  close(clp, clpRef, 3e-3, "plate Clp");
  near(clr, 0, 1e-4 * Math.abs(clpRef), "plate Clr (Newtonian: 0)");
  log(`A6 plate α=10°: Cmq ${cmq.toFixed(5)} (ref ${cmqRef.toFixed(5)}), CLq ${clq.toFixed(5)} (ref ${clqRef.toFixed(5)}), Clp ${clp.toFixed(5)} (ref ${clpRef.toFixed(5)}), Clr ${clr.toExponential(2)} (ref 0)`);
}

/* ---- A7. generateAeroDatabase vs independent central differences of panelAero ----
 * (i) Same stencil as the table → must agree to round-off (indexing, per-radian scaling, Sref, Lref).
 * (ii) Fine stencil (±0.25°) → truncation of the 4° / 2° table step; tolerance reported, not tuned:
 *      |table − fine| ≤ 0.02·|fine| + 2e-3·|CLα| style floor (see the log). */
for (const family of ["caret", "cone", "liftbody"] as const) {
  const p: DesignParams = { ...DEFAULT_PARAMS, family, nx: 28, ny: 18, alphaDeg: 4 };
  const built = buildVehicle(p);
  const spec = { ...defaultAeroDbSpec(p), machMin: 8, machMax: 8, alphaMin: 0, alphaMax: 8, alphaStep: 4, betaMin: -2, betaMax: 2, betaStep: 2, deltaMin: -4, deltaMax: 4, deltaStep: 4 };
  const db = await generateAeroDatabase(built, spec);
  const S0 = db.meta.sRef_m2;
  assert.equal(S0, built.aero.planformArea, "Sref is the undeflected planform");
  const pp = { ...p, aeroMethod: spec.method, lockFlight: false, flightMach: 8 };
  const a8 = atmosphere(p.altKm, 8, p.gamma);
  const c = (al: number, be: number, r?: RateState) => panelAero(built.mesh, pp, a8, al, be, S0, p.length, r, true);
  const pt = db.points.find((q) => q.alpha_deg === 4 && q.beta_deg === 0 && q.delta_deg === 0);
  assert.ok(pt, "anchor point");
  const hi = c(8, 0);
  const lo = c(0, 0);
  const bh = c(4, 2);
  const bl = c(4, -2);
  const sp = 8 * DEG;
  const sb = 4 * DEG;
  close(pt.cla, (hi.cl - lo.cl) / sp, 1e-9, `${family} CLα table stencil`);
  close(pt.cma, (hi.cm - lo.cm) / sp, 1e-9, `${family} Cmα table stencil`);
  close(pt.cyb, (bh.cy - bl.cy) / sb, 1e-9, `${family} CYβ table stencil`);
  close(pt.cnb, (bh.cn - bl.cn) / sb, 1e-9, `${family} Cnβ table stencil`);
  close(pt.clb, (bh.cll - bl.cll) / sb, 1e-9, `${family} Clβ table stencil`);
  // δ derivative uses the same reference area as δ = 0 (was each deflected mesh's own planform)
  const vp = buildVehicle({ ...p, elevonDeg: 4 });
  const vm = buildVehicle({ ...p, elevonDeg: -4 });
  const cdP = panelAero(vp.mesh, { ...pp, elevonDeg: 4 }, a8, 4, 0, S0, p.length, undefined, true);
  const cdM = panelAero(vm.mesh, { ...pp, elevonDeg: -4 }, a8, 4, 0, S0, p.length, undefined, true);
  close(pt.cmd, (cdP.cm - cdM.cm) / (8 * DEG), 1e-9, `${family} Cmδ table stencil, single Sref`);
  // fine stencil
  const f = 0.25;
  const fine = (k: "cl" | "cm" | "cy" | "cn" | "cll", beta: boolean) => {
    const u = beta ? c(4, f) : c(4 + f, 0);
    const d = beta ? c(4, -f) : c(4 - f, 0);
    return (u[k] - d[k]) / (2 * f * DEG);
  };
  const fCla = fine("cl", false);
  const fCnb = fine("cn", true);
  const fClb = fine("cll", true);
  close(pt.cla, fCla, 0.05, `${family} CLα truncation (4° step)`);
  close(pt.cnb, fCnb, 0.05, `${family} Cnβ truncation (2° step)`);
  close(pt.clb, fClb, 0.05, `${family} Clβ truncation (2° step)`);
  // dynamic rows: central ±0.02 differences
  const dyn = db.dynamics[0];
  const V = Math.max(50, a8.V);
  const w = (0.02 * 2 * V) / p.length;
  const wb = (0.02 * 2 * V) / p.span;
  close(dyn.cmq, (c(4, 0, { p: 0, q: w, r: 0 }).cm - c(4, 0, { p: 0, q: -w, r: 0 }).cm) / 0.04, 1e-9, `${family} Cmq central`);
  close(dyn.clp, (c(4, 0, { p: wb, q: 0, r: 0 }).cll - c(4, 0, { p: -wb, q: 0, r: 0 }).cll) / 0.04, 1e-9, `${family} Clp central`);
  close(dyn.cnr, (c(4, 0, { p: 0, q: 0, r: wb }).cn - c(4, 0, { p: 0, q: 0, r: -wb }).cn) / 0.04, 1e-9, `${family} Cnr central`);
  assert.ok(dyn.cmq < 0 && dyn.clp < 0 && dyn.cnr < 0, `${family}: pitch, roll and yaw damping (Cmq ${dyn.cmq}, Clp ${dyn.clp}, Cnr ${dyn.cnr})`);
  // exported CG is the moment reference panelAero used
  close(db.meta.cg_m[0], c(4, 0).cg[0], 1e-12, `${family} meta CG = panel moment reference`);
  log(
    `A7 ${family}: CLα table ${pt.cla.toFixed(4)} / fine ${fCla.toFixed(4)} (${(100 * rel(pt.cla, fCla)).toFixed(2)} %), Cnβ ${pt.cnb.toFixed(5)} / ${fCnb.toFixed(5)}, Clβ ${pt.clb.toFixed(5)} / ${fClb.toFixed(5)}, Cmq ${dyn.cmq.toFixed(4)} Clp ${dyn.clp.toFixed(4)} Cnr ${dyn.cnr.toFixed(4)}`,
  );
}

console.log("audit-aero passed");
