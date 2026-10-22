/**
 * Accuracy audit, area 2: TPS material data (tps.ts) vs NASA TPSX, 1-D conduction vs closed forms,
 * mass properties (mass.ts) and the mass budget vs analytic rigid-body results.
 * Run: node --import ./scripts/ts-register.mjs src/lib/waverider/audit-tps.test.ts   (npm run test:audit)
 */
import assert from "node:assert/strict";
import { boxMesh, sphereMesh } from "./audit.fixtures";
import { massProperties } from "./mass";
import { conduct1D, massBudget, sizeTps, TPS_LIB, TPSX_SIP } from "./tps";
import { DEFAULT_PARAMS, type TriMesh } from "./types";

const rel = (a: number, b: number) => Math.abs(a - b) / Math.max(Math.abs(b), 1e-300);
const close = (a: number, b: number, tol: number, msg: string) => assert.ok(rel(a, b) <= tol, `${msg}: got ${a}, ref ${b}, rel ${rel(a, b).toExponential(2)} > ${tol}`);
const log = (s: string) => console.log(`  ${s}`);

/* ---- T1. TPS_LIB vs NASA Ames TPSX Material Properties Database, "standard conditions" table of each material
 * (tpsx.arc.nasa.gov, retrieved 2026-10-22; TPSX material id in brackets). TPSX prints 3 significant figures,
 * so tolerance 0.5 %. TUFROC, SLA-561 and the titanium alloy are not checked (no matching TPSX entry used). */
{
  const tpsx: [keyof typeof TPS_LIB, number, { rho: number; k: number; cp: number; emiss?: number; tmax?: number }][] = [
    ["tile", 1, { rho: 144, k: 0.0476, cp: 628, emiss: 0.88, tmax: 1590 }], // LI-900; Tmax = multiple-use limit
    ["li2200", 2, { rho: 352, k: 0.0744, cp: 628, emiss: 0.899, tmax: 1640 }],
    ["aetb8", 5, { rho: 128, k: 0.0684, cp: 628, emiss: 0.899, tmax: 1640 }],
    ["aetb12", 6, { rho: 192, k: 0.064, cp: 628, emiss: 0.88, tmax: 1700 }],
    ["rcc", 24, { rho: 1580, k: 3.89, cp: 712, emiss: 0.78, tmax: 1920 }],
    ["pica", 43, { rho: 236, k: 0.305, cp: 1200, emiss: 0.8 }], // virgin; TPSX gives no temperature limit
    ["inconel", 54, { rho: 8410, k: 15.1, cp: 456 }],
    ["aluminum", 60, { rho: 2770, k: 177, cp: 875 }], // 2024-T6
  ];
  for (const [id, n, ref] of tpsx) {
    const m = TPS_LIB[id];
    for (const key of ["rho", "k", "cp", "emiss", "tmax"] as const) {
      const v = ref[key];
      if (v !== undefined) close(m[key], v, 5e-3, `${id} ${key} vs TPSX #${n}`);
    }
  }
  close(TPSX_SIP.rho, 86.5, 5e-3, "SIP ρ vs TPSX #11");
  close(TPSX_SIP.k, 0.0406, 5e-3, "SIP k vs TPSX #11");
  log(`T1 ${tpsx.length} TPS_LIB rows + SIP match TPSX within 0.5 %`);
}

/** erfc via Abramowitz & Stegun (1964) eq. 7.1.26, |error| ≤ 1.5e-7. */
function erfc(x: number): number {
  const t = 1 / (1 + 0.3275911 * x);
  return t * (0.254829592 + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429)))) * Math.exp(-x * x);
}

/* Conduction reference solutions: Incropera, DeWitt, Bergman & Lavine, "Fundamentals of Heat and Mass Transfer",
 * 6th ed., §5.7 semi-infinite solid, eqs. 5.57 (case 1, constant surface T) and 5.59 (case 2, constant surface flux);
 * §5.3 lumped capacitance, eq. 5.6; §3.1 plane wall, eq. 3.3 (steady linear profile). Equation numbers as recalled for the 6th ed.; the closed forms
 * themselves are standard and are what the test uses. */
const mat = { k: 0.0476, rho: 144, cp: 628 }; // LI-900 room-temperature values (TPSX #1)
const alpha = mat.k / (mat.rho * mat.cp);

/* ---- T2. Case 1, surface stepped to Ts: (T − Ts)/(Ti − Ts) = erf(x / 2√(αt)). 10 cm slab, 300 s: penetration
 * 4√(αt) ≈ 2.8 cm, so the adiabatic back is semi-infinite to < 1e-6. Crank–Nicolson, 200 cells, dt 0.25 s. */
{
  const Ti = 300;
  const Ts = 1200;
  const t = 300;
  const r = conduct1D({ ...mat, thickness: 0.1, n: 200, dt: 0.25, tEnd: t, T0: Ti, surface: { kind: "T", T: Ts }, back: { kind: "adiabatic" } });
  let worst = 0;
  for (let i = 0; i < r.x.length; i += 4) {
    const ref = Ts + (Ti - Ts) * (1 - erfc(r.x[i] / (2 * Math.sqrt(alpha * t))));
    worst = Math.max(worst, Math.abs(r.T[i] - ref) / (Ts - Ti));
  }
  assert.ok(worst < 5e-3, `erf profile max error ${worst} of ΔT`);
  log(`T2 step surface T (Incropera eq. 5.57): max |ΔT| ${(worst * 100).toFixed(3)} % of Ts − Ti (tol 0.5 %)`);
}

/* ---- T3. Case 2, constant flux q0: T − Ti = (2 q0/k)√(αt/π) exp(−x²/4αt) − (q0 x/k) erfc(x/2√(αt)). */
{
  const q0 = 2000;
  const t = 300;
  const r = conduct1D({ ...mat, thickness: 0.1, n: 200, dt: 0.25, tEnd: t, T0: 300, surface: { kind: "flux", q: () => ({ a: q0, b: 0 }) }, back: { kind: "adiabatic" } });
  const ref = (x: number) => 300 + ((2 * q0) / mat.k) * Math.sqrt((alpha * t) / Math.PI) * Math.exp((-x * x) / (4 * alpha * t)) - ((q0 * x) / mat.k) * erfc(x / (2 * Math.sqrt(alpha * t)));
  const rise = ref(0) - 300;
  let worst = 0;
  for (let i = 0; i < r.x.length; i += 4) worst = Math.max(worst, Math.abs(r.T[i] - ref(r.x[i])) / rise);
  assert.ok(worst < 5e-3, `const-flux profile error ${worst}`);
  /* T4. Energy: with flux / adiabatic faces the scheme is conservative; stored = q0 t. */
  close(r.stored, q0 * t, 1e-9, "stored energy = q0 t");
  close(r.energyIn, q0 * t, 1e-9, "energyIn = q0 t");
  log(`T3 constant flux (eq. 5.59): surface ${r.T[0].toFixed(1)} K vs ${ref(0).toFixed(1)} K, max err ${(worst * 100).toFixed(3)} % of rise; T4 stored/q0t − 1 = ${(r.stored / (q0 * t) - 1).toExponential(1)}`);
}

/* ---- T5. Steady plane wall, T1 = 1000 K, T2 = 300 K: linear profile (eq. 3.3), flux k ΔT / L. */
{
  const L = 0.02;
  const r = conduct1D({ ...mat, thickness: L, n: 40, dt: 20, tEnd: 4e4, T0: 300, surface: { kind: "T", T: 1000 }, back: { kind: "T", T: 300 }, theta: 1 });
  let worst = 0;
  for (let i = 0; i < r.x.length; i++) worst = Math.max(worst, Math.abs(r.T[i] - (1000 - (700 * r.x[i]) / L)));
  assert.ok(worst < 1e-3, `steady profile error ${worst} K`);
  log(`T5 steady wall (eq. 3.3): max |T − linear| ${worst.toExponential(1)} K`);
}

/* ---- T6. Lumped capacitance (eq. 5.6), convective face h(T∞ − T) = a − bT with a = h T∞, b = h on a high-k slab
 * (Bi = hL/k = 2.8e-3 < 0.1), adiabatic back: T = T∞ + (T0 − T∞) exp(−t/τ), τ = ρ c L / h. */
{
  const h = 50;
  const L = 0.01;
  const m2 = { k: 177, rho: 2770, cp: 875 }; // 2024-T6, TPSX #60
  const tau = (m2.rho * m2.cp * L) / h;
  const r = conduct1D({ ...m2, thickness: L, n: 10, dt: 1, tEnd: tau, T0: 300, surface: { kind: "flux", q: () => ({ a: h * 1000, b: h }) }, back: { kind: "adiabatic" } });
  const ref = 1000 + (300 - 1000) * Math.exp(-1);
  // Lumped capacitance neglects the internal gradient (q L / 2k ≈ 0.36 K here), so compare the mean temperature
  // and allow an O(Bi) error: |T̄ − T_ref| < 10 Bi (T∞ − T0).
  const Tmean = 300 + r.stored / (m2.rho * m2.cp * L);
  const Bi = (h * L) / m2.k;
  assert.ok(Math.abs(Tmean - ref) < 10 * Bi * 700, `lumped mean T ${Tmean} vs ${ref}`);
  log(`T6 lumped capacitance (Bi ${Bi.toExponential(1)}) at t = τ = ${tau.toFixed(0)} s: mean ${Tmean.toFixed(3)} K vs ${ref.toFixed(3)} K`);
}

/* ---- T7. sizeTps transient check is finite and physical under reradiation, NaN when ablating. */
{
  const ok = sizeTps({ qWcm2: 5, areaM2: 10, timeS: 600, material: "tile" });
  assert.ok(!ok.overTemperature && Number.isFinite(ok.backfaceK) && ok.backfaceK >= 300 && ok.backfaceK < TPS_LIB.tile.tmax, `tile backface ${ok.backfaceK}`);
  const hot = sizeTps({ qWcm2: 500, areaM2: 1, timeS: 60, material: "pica" });
  assert.ok(hot.overTemperature && Number.isNaN(hot.backfaceK));
  log(`T7 LI-900 at 5 W/cm² for 600 s: δ ${ok.thicknessMm.toFixed(1)} mm, backface peak ${ok.backfaceK.toFixed(0)} K`);
}

/* ---- T8. massProperties vs analytic solids. Box a×b×c: Ixx = m(b² + c²)/12 etc. (exact for a polyhedron).
 * Centred away from the origin to exercise the parallel-axis shift (the 1/120 bug gave Ixx 6448 vs 104.2). */
{
  const mp = massProperties(boxMesh(2, 1, 0.5, [1, 2, 3]), 1000);
  const m = 1000;
  close(mp.mass, m, 1e-12, "box mass");
  close(mp.Ixx, (m * (1 + 0.25)) / 12, 1e-10, "box Ixx");
  close(mp.Iyy, (m * (4 + 0.25)) / 12, 1e-10, "box Iyy");
  close(mp.Izz, (m * (4 + 1)) / 12, 1e-10, "box Izz");
  assert.ok(Math.abs(mp.Ixz) < 1e-9 && Math.abs(mp.Ixy ?? 1) < 1e-9 && Math.abs(mp.Iyz ?? 1) < 1e-9, "box products zero");
  // Two unit cubes at (±1, ±1, 0): about the CG, ∫xy dm = 2 · m_cube · 1 · 1 (point-mass offsets; each cube's own product is 0).
  const merge = (a: TriMesh, b: TriMesh): TriMesh => {
    const off = a.positions.length / 3;
    const ind = new Uint32Array(a.indices.length + b.indices.length);
    ind.set(a.indices);
    for (let i = 0; i < b.indices.length; i++) ind[a.indices.length + i] = b.indices[i] + off;
    const pos = new Float64Array(a.positions.length + b.positions.length);
    pos.set(a.positions);
    pos.set(b.positions, a.positions.length);
    const surf = new Uint8Array(a.surfaces.length + b.surfaces.length);
    surf.set(a.surfaces);
    surf.set(b.surfaces, a.surfaces.length);
    return { positions: pos, indices: ind, surfaces: surf };
  };
  const two = massProperties(merge(boxMesh(1, 1, 1, [1, 1, 0]), boxMesh(1, 1, 1, [-1, -1, 0])), 500);
  close(two.Ixy ?? NaN, 2 * 500 * 1 * 1, 1e-10, "two-cube Ixy (+∫xy dm)");
  close(two.Izz, 2 * ((500 * 2) / 12 + 500 * 2), 1e-10, "two-cube Izz");
  // UV sphere: I = 2/5 m R² for the true sphere; the polyhedron is compared through its own mass, tolerance 1 %.
  const R = 1.5;
  const sp = massProperties(sphereMesh(R, 4, 64, 96), 1000);
  close(sp.Ixx, 0.4 * sp.mass * R * R, 1e-2, "sphere Ixx");
  close(sp.Iyy, 0.4 * sp.mass * R * R, 1e-2, "sphere Iyy");
  close(sp.cg[0], 4, 1e-9, "sphere cg x");
  log(`T8 box Ixx ${mp.Ixx.toFixed(3)} (analytic 104.167); two-cube Ixy ${two.Ixy?.toFixed(3)}; sphere Ixx/(0.4 m R²) ${(sp.Ixx / (0.4 * sp.mass * R * R)).toFixed(5)}`);
}

/* ---- T9. massBudget parallel axis: I = Σ (I_own,i + m_i d_i²) about the combined CG (analytic, Steiner). */
{
  const base = {
    params: DEFAULT_PARAMS,
    volume: 10,
    length: 10,
    height: 1,
    structMass: 1000,
    structCgX: 5,
    ixx: 200,
    iyy: 8000,
    izz: 8100,
    tpsMass: 0,
    fuelFrac: 0,
    fuelRho: 71,
  };
  const b = massBudget({ ...base, payloadKg: 100, payloadXFrac: 0.7 });
  const cg = (1000 * 5 + 100 * 7) / 1100;
  close(b.cgX, cg, 1e-12, "budget cg");
  const iyy = 8000 * 1.1 + 1000 * (5 - cg) ** 2 + 100 * (7 - cg) ** 2;
  close(b.iyy, iyy, 1e-12, "budget Iyy");
  close(b.ixx, 200 * 1.1, 1e-12, "budget Ixx (no z offset)");
  // TPS volume comes out of the tankable volume
  const f = massBudget({ ...base, payloadKg: 0, fuelFrac: 1, tpsVolume: 2 });
  close(f.volumeFuel, 8, 1e-12, "fuel volume = OML − TPS");
  log(`T9 budget Iyy ${b.iyy.toFixed(2)} vs Steiner ${iyy.toFixed(2)}`);
}

console.log("audit-tps passed");
