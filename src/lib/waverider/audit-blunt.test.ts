/**
 * Accuracy audit, area 4: blunt bodies (vehicles.ts capsule builder + panel.ts modified Newtonian).
 * Run: node --import ./scripts/ts-register.mjs src/lib/waverider/audit-blunt.test.ts   (npm run test:audit)
 *
 * References are analytic modified-Newtonian closed forms (Cp = Cpmax cos²η on the windward face, 0 leeward;
 * Anderson, Hypersonic and High-Temperature Gas Dynamics, ch. 3), integrated here in closed form. The
 * base (−1/M² Love) and friction terms are removed with cdBase / cdFric before comparing.
 */
import assert from "node:assert/strict";
import { atmosphere } from "./atmosphere";
import { sphereMesh } from "./audit.fixtures";
import { buildVehicle } from "./generate";
import { DEG, newtonianCpMax } from "./math";
import { panelAero } from "./panel";
import { DEFAULT_PARAMS, type DesignParams } from "./types";

const rel = (a: number, b: number) => Math.abs(a - b) / Math.max(Math.abs(b), 1e-300);
const close = (a: number, b: number, tol: number, msg: string) => assert.ok(rel(a, b) <= tol, `${msg}: got ${a}, ref ${b}, rel ${rel(a, b).toExponential(2)} > ${tol}`);
const log = (s: string) => console.log(`  ${s}`);

const M = 20;
const atm = atmosphere(40, M, 1.4);
const Cpmax = newtonianCpMax(M, 1.4);
const base: DesignParams = { ...DEFAULT_PARAMS, family: "caret", aeroMethod: "newtonian", lockFlight: false, flightMach: M, mach: M, altKm: 40, length: 2, span: 2, cgFrac: 0.5 };

/* ---- B1. Sphere: CD = ∫ Cpmax cos²η cosη dA / πR² over the forward hemisphere = Cpmax / 2. UV sphere 96 × 192;
 * the faceted surface converges at O(1/n²): tolerance 0.1 %. */
{
  const R = 1;
  const r = panelAero(sphereMesh(R, R, 96, 192), base, atm, 0, 0, Math.PI * R * R, 2 * R, undefined, true);
  const cdp = r.cd - r.cdFric - r.cdBase;
  close(cdp, Cpmax / 2, 1e-3, "sphere pressure CD");
  log(`B1 sphere M${M}: CD_p ${cdp.toFixed(5)} vs Cpmax/2 = ${(Cpmax / 2).toFixed(5)}`);
}

/** Sphere-cone forebody, α = 0, on πRb²: CA/Cpmax = (Rn/Rb)²(1 − sin⁴θ)/2 + sin²θ [1 − (Rn/Rb)² cos²θ]
 * (cap: ∫ cos³η over polar angle 0…90°−θ; cone frustum: Cpmax sin²θ on the annulus Rn cosθ … Rb). */
const sphereConeCA = (Rn: number, Rb: number, th: number) => {
  const e = Rn / Rb;
  return Cpmax * ((e * e * (1 - Math.sin(th) ** 4)) / 2 + Math.sin(th) ** 2 * (1 - e * e * Math.cos(th) ** 2));
};
/** Capsule built so the cone just reaches Rb at x = L (no cylinder, no shoulder flare): L = xt + (Rb − rt)/tanθ. */
const capsule = (Rn: number, Rb: number, thDeg: number): DesignParams => {
  const th = thDeg * DEG;
  const L = Rn * (1 - Math.sin(th)) + (Rb - Rn * Math.cos(th)) / Math.tan(th);
  return { ...base, family: "capsule", span: 2 * Rb, coneDeg: thDeg, leRadius: Rn, length: L, nx: 72, ny: 56 };
};

/* ---- B2. Capsule builder + panel CD vs the closed form. The builder samples x uniformly (72 rings max), so the
 * cap/cone junction is a chord: tolerance 0.5 %. */
{
  const out: string[] = [];
  for (const [Rn, Rb, th] of [
    [0.4, 1, 20],
    [0.4, 1, 45],
    [0.4, 1, 55],
  ] as const) {
    const p = capsule(Rn, Rb, th);
    const r = panelAero(buildVehicle(p).mesh, p, atm, 0, 0, Math.PI * Rb * Rb, p.length, undefined, true);
    const cdp = r.cd - r.cdFric - r.cdBase;
    const ref = sphereConeCA(Rn, Rb, th * DEG);
    close(cdp, ref, 5e-3, `sphere-cone θ ${th}° CD`);
    out.push(`${th}°: ${cdp.toFixed(4)} vs ${ref.toFixed(4)}`);
  }
  log(`B2 sphere-cone Rn/Rb 0.4: ${out.join("; ")}`);
}

/* ---- B3. Mars Pathfinder aeroshell geometry: 70° half-angle, Db = 2.65 m, Rn = 0.6638 m (Gnoffo, Braun, Weilmuenster,
 * Mitcheltree, Engelund & Powell, "Prediction and Validation of Mars Pathfinder Hypersonic Aerodynamic Data Base",
 * J. Spacecraft & Rockets 36(3), 1999). The reference is the closed form on that geometry, NOT a published CA:
 * the modified-Newtonian / DSMC CA values quoted for Pathfinder (e.g. Moss et al., AIAA 97-2508) were not verified
 * and are not asserted. Needs the 75° cone clamp and the cap-depth Rn bound in buildCapsule (both [accuracy-audit]). */
{
  const Rn = 0.6638;
  const Rb = 2.65 / 2;
  const p = capsule(Rn, Rb, 70);
  const built = buildVehicle(p);
  const r = panelAero(built.mesh, p, atm, 0, 0, Math.PI * Rb * Rb, p.length, undefined, true);
  const cdp = r.cd - r.cdFric - r.cdBase;
  const ref = sphereConeCA(Rn, Rb, 70 * DEG);
  close(cdp, ref, 5e-3, "Pathfinder-geometry CD");
  assert.ok(built.aero.notes.some((n) => n.includes("cone 70.0°") && n.includes(`Rn = ${Rn.toFixed(3)}`)), "builder keeps 70° and Rn 0.664 m");
  log(`B3 Pathfinder geometry M${M}: CA ${cdp.toFixed(4)} vs closed form ${ref.toFixed(4)} (CA/Cpmax ${(ref / Cpmax).toFixed(4)}; classic Newtonian Cpmax = 2 gives ${((2 * ref) / Cpmax).toFixed(4)})`);
}

console.log("audit-blunt passed");
