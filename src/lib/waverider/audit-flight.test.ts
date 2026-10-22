/**
 * Accuracy audit, area 3: rigid-body equations (sixdof.ts), closed loop (guidance.ts), shipped cuspis_6dof.py.
 * Run: node --import ./scripts/ts-register.mjs src/lib/waverider/audit-flight.test.ts   (npm run test:audit)
 */
import assert from "node:assert/strict";
import { execFileSync } from "node:child_process";
import { mkdtempSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { atmosphere } from "./atmosphere";
import { CUSPIS_6DOF_PY } from "./cuspis-6dof-src";
import { buildVehicle } from "./generate";
import { defaultFlySpec, flyClosedLoop } from "./guidance";
import { massProperties } from "./mass";
import { DEG } from "./math";
import { panelAero } from "./panel";
import { rigidBodyRates, rk4Step, solveSixDof, specificEnergy, type RigidBody, type RigidCoeffs, type RigidState } from "./sixdof";
import { DEFAULT_PARAMS, type DesignParams } from "./types";

const rel = (a: number, b: number) => Math.abs(a - b) / Math.max(Math.abs(b), 1e-300);
const close = (a: number, b: number, tol: number, msg: string) => assert.ok(rel(a, b) <= tol, `${msg}: got ${a}, ref ${b}, rel ${rel(a, b).toExponential(2)} > ${tol}`);
const log = (s: string) => console.log(`  ${s}`);

const body: RigidBody = { S: 4, L: 4, m: 500, Ixx: 30, Iyy: 300, Izz: 320, rho: (h) => 1.225 * Math.exp(-h / 7200) };
const start = (): RigidState & { x: number } => ({ alpha: 4 * DEG, q: 0, theta: 4 * DEG - 5 * DEG, beta: 0, p: 0, r: 0, phi: 0, VV: 2400, gamma: -5 * DEG, h: 30000, x: 0 });

/* ---- F1. Unpowered, drag-free flight conserves E = ½V² − g0 Re²/(Re + h) (inverse-square field, lift ⟂ V does no work).
 * Analytic invariant of the state equations; RK4 dt = 0.05 s over 120 s → |ΔE| / (½V0²) < 1e-9. */
{
  const coeffs = (s: RigidState): RigidCoeffs => ({ CL: 0.3 + 0.5 * (s.alpha - 4 * DEG), CD: 0, CY: 0, Cl: 0, Cm: -0.02 * (s.alpha - 4 * DEG) - 0.1 * ((s.q * 4) / (2 * s.VV)), Cn: 0 });
  let s = start();
  const E0 = specificEnergy(s.VV, s.h);
  let worst = 0;
  let inv = 0;
  for (let k = 0; k < 2400; k++) {
    s = rk4Step(s, 0.05, coeffs, body);
    // normalised by the initial kinetic energy (E itself is dominated by the −g0 Re potential)
    worst = Math.max(worst, Math.abs(specificEnergy(s.VV, s.h) - E0) / (0.5 * 2400 * 2400));
    inv = Math.max(inv, Math.abs(s.alpha - (s.theta - s.gamma)));
  }
  assert.ok(worst < 1e-9, `energy drift ${worst}`);
  /* F2. θ = α + γ is preserved exactly (α̇ = q − γ̇, θ̇ = q). It was not before the fix (α̇ lacked gravity). */
  assert.ok(inv < 1e-12, `α − (θ − γ) drift ${inv}`);
  log(`F1 drag-free 120 s: max |ΔE|/(½V0²) ${worst.toExponential(2)} (tol 1e-9); F2 max |α − (θ − γ)| ${inv.toExponential(2)} rad`);
}

/* ---- F3. With drag, dE/dt = −D V / m exactly. Trapezoid of D V/m on the RK4 states, dt = 0.02 s, 40 s → 1e-5. */
{
  const coeffs = (): RigidCoeffs => ({ CL: 0.2, CD: 0.05, CY: 0, Cl: 0, Cm: 0, Cn: 0 });
  let s = start();
  const E0 = specificEnergy(s.VV, s.h);
  const pw = (st: RigidState) => (0.5 * body.rho(st.h) * st.VV ** 3 * body.S * 0.05) / body.m;
  let W = 0;
  for (let k = 0; k < 2000; k++) {
    const p0 = pw(s);
    s = rk4Step(s, 0.02, coeffs, body);
    W += 0.01 * (p0 + pw(s));
  }
  close(E0 - specificEnergy(s.VV, s.h), W, 1e-5, "energy lost = ∫ D V/m dt");
  log(`F3 drag work ${W.toFixed(1)} J/kg matches the energy loss`);
}

/* ---- F4. Rate / sign conventions in the panel frame (X aft, Y span, Z up; β_panel = −β_conventional).
 * Side force along +Y accelerates the body toward +Y, which reduces the +Y relative flow: β̇ = … − Y/(mV) < 0.
 * Moments are q·S·Lref·C on all three axes (panel Cll, Cm, Cn are on S·Lref). */
{
  const st: RigidState = { ...start(), beta: 2 * DEG };
  const c: RigidCoeffs = { CL: 0, CD: 0, CY: 0.1, Cl: 0.001, Cm: 0.002, Cn: 0.003 };
  const r = rigidBodyRates(st, c, body);
  const qS = 0.5 * body.rho(st.h) * st.VV * st.VV * body.S;
  assert.ok(r.bDot < 0, "side force opposes panel β");
  close(r.bDot, (-qS * 0.1) / (body.m * st.VV), 1e-12, "β̇ = −Y/(mV) at p = r = 0");
  close(r.pDot, (qS * body.L * 0.001) / body.Ixx, 1e-12, "ṗ = q S Lref Cll / Ixx");
  close(r.qDot, (qS * body.L * 0.002) / body.Iyy, 1e-12, "q̇ = q S Lref Cm / Iyy");
  close(r.rDot, (qS * body.L * 0.003) / body.Izz, 1e-12, "ṙ = q S Lref Cn / Izz");
  log("F4 β̇ sign and S·Lref moment scaling confirmed");
}

/* ---- F5. solveSixDof: trim consistency and lateral mode formulas on the default caret (cbaero). */
{
  const p: DesignParams = { ...DEFAULT_PARAMS, nx: 28, ny: 18, aeroMethod: "cbaero", alphaDeg: 4, cgFrac: 0.5 };
  const built = buildVehicle(p);
  const atm = atmosphere(p.altKm, p.mach, p.gamma);
  const mp = massProperties(built.mesh, p.rhoKgM3, 0);
  const S = built.aero.planformArea;
  const res = solveSixDof(built.mesh, p, atm, mp, S, p.length, true);
  const d = res.derivs;
  const cmTrim = panelAero(built.mesh, p, atm, d.trimAlpha, 0, S, p.length, undefined, true).cm;
  assert.equal(d.trimCm, cmTrim, "trimCm is Cm evaluated at trimAlpha");
  assert.ok(d.trimAlpha > -8 && d.trimAlpha < 16 && Math.abs(cmTrim) < 1e-4, `interior trim with Cm ≈ 0 (α ${d.trimAlpha}, Cm ${cmTrim})`);
  const V = Math.max(50, atm.V);
  const qS = Math.max(atm.q, 1) * S;
  const Nb = (qS * p.length * d.cnb) / mp.Izz;
  const Nr = (qS * p.length * d.cnr * (p.span / (2 * V))) / mp.Izz;
  const YbV = -(qS * d.cyb) / mp.mass / V;
  const dr = res.modes.find((m) => m.name === "Dutch roll");
  assert.ok(dr);
  close(dr.wn, Math.sqrt(Nb + YbV * Nr), 1e-12, "Dutch roll ω² = Nβ + Yβ Nr/V with S·Lref moments");
  close(dr.zeta, -(YbV + Nr) / (2 * dr.wn), 1e-12, "Dutch roll ζ with conventional Yβ = −cyb_panel");
  assert.ok(dr.zeta > 0, `Dutch roll damped (ζ ${dr.zeta})`);
  const roll = res.modes.find((m) => m.name === "Roll subsidence");
  assert.ok(roll);
  close(roll.wn, Math.abs((qS * p.length * d.clp * (p.span / (2 * V))) / mp.Ixx), 1e-12, "Lp = q S Lref Clp (b/2V) / Ixx");
  // θ starts at α0 + γ0 and the RK4 history keeps θ − γ − α at its initial value unless a clamp acts
  log(`F5 caret cg 0.5: trim α ${d.trimAlpha.toFixed(3)}°, Cm ${cmTrim.toExponential(2)}; Dutch roll ω ${dr.wn.toFixed(2)} ζ ${dr.zeta.toFixed(3)}; roll λ ${roll.wn.toFixed(3)} /s`);
}

/* ---- F6. Closed loop. (i) NDI holds θ = α + γ. (ii) PID acts through sign(Cmδ): with a fast actuator it settles on
 * the command. (Default τ = 0.08 s / 40 °/s with kp = 1.4 is NOT stable on these vehicles — a gain-tuning limit, see NOTES.) */
{
  const p: DesignParams = { ...DEFAULT_PARAMS, family: "liftbody", nx: 28, ny: 18, aeroMethod: "cbaero", alphaDeg: 4 };
  const built = buildVehicle(p);
  const ndi = await flyClosedLoop(built, { ...defaultFlySpec(p), law: "ndi", durationS: 40 });
  let drift = 0;
  for (const s of ndi.samples) drift = Math.max(drift, Math.abs(s.alphaDeg - (s.thetaDeg - s.gammaDeg)));
  assert.ok(drift < 1e-6, `NDI α − (θ − γ) drift ${drift}°`);
  const pid = await flyClosedLoop(built, { ...defaultFlySpec(p), law: "pid", durationS: 40, dt: 0.005, tauAct: 0.02, rateMaxDeg: 2000 });
  const last = pid.samples[pid.samples.length - 1];
  assert.ok(Math.abs(last.alphaDeg - last.alphaCmd) < 0.5, `PID settles (α ${last.alphaDeg} vs cmd ${last.alphaCmd})`);
  const qMax = Math.max(...pid.samples.map((s) => Math.abs(s.qDeg)));
  assert.ok(qMax < 60, `PID pitch rate bounded (${qMax} °/s)`);
  log(`F6 liftbody: NDI drift ${drift.toExponential(1)}°, PID (fast actuator) α ${last.alphaDeg.toFixed(2)}° vs cmd ${last.alphaCmd}°, max |q| ${qMax.toFixed(1)} °/s`);
}

/* ---- F7. Shipped cuspis_6dof.py compiles and integrates a synthetic table (no reference data: CL = 0.02 α, CD 0.05,
 * Cm = −0.0002 (α − 4) − 0.0003 δ, fast actuator τ = 0.01 s). Checks the PID sign: α must approach the best-L/D α of the table (12°), not run away. */
{
  let py = "";
  try {
    py = execFileSync("python3", ["--version"], { encoding: "utf8" });
  } catch {
    py = "";
  }
  if (!py) {
    log("F7 skipped: python3 not available");
  } else {
    const dir = mkdtempSync(join(tmpdir(), "cuspis6dof-"));
    writeFileSync(join(dir, "cuspis_6dof.py"), CUSPIS_6DOF_PY);
    const mach = [6, 8, 10];
    const alpha = [0, 4, 8, 12];
    const delta = [-10, 0, 10];
    const points: object[] = [];
    for (const m of mach) for (const a of alpha) for (const d of delta) points.push({ mach: m, alpha_deg: a, beta_deg: 0, delta_deg: d, cl: 0.02 * a, cd: 0.05, cm: -0.0002 * (a - 4) - 0.0003 * d, cmq: -0.3 });
    const doc = { meta: { format: "cuspis-aerodb-1", sRef_m2: 4, lRef_m: 4, bRef_m: 2, designMach: 8, altitude_km: 30, mass_kg: 500, gamma: 1.4 }, axes: { mach, alpha_deg: alpha, beta_deg: [0], delta_deg: delta }, points };
    writeFileSync(join(dir, "aero.json"), JSON.stringify(doc));
    const out = execFileSync("python3", [join(dir, "cuspis_6dof.py"), join(dir, "aero.json"), "--seconds", "20", "--dt", "0.002", "--tau", "0.01"], { encoding: "utf8", stdio: ["ignore", "pipe", "ignore"] });
    const rows = out.trim().split("\n");
    const head = rows[0].split(",");
    const last = rows[rows.length - 1].split(",").map(Number);
    const a = last[head.indexOf("alpha_deg")];
    const cmd = last[head.indexOf("alpha_cmd_deg")];
    assert.ok(Number.isFinite(a) && Math.abs(a - cmd) < 1.5, `python PID settles (α ${a} vs cmd ${cmd})`);
    log(`F7 cuspis_6dof.py: α ${a.toFixed(2)}° vs cmd ${cmd}° after 20 s`);
  }
}

console.log("audit-flight passed");
