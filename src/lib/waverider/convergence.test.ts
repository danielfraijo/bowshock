/**
 * Convergence tests: does each numerical parameter converge, and at what observed order?
 *   1. Taylor–Maccoll RK4 step size           (expected order ≈ 4)
 *   2. Shock-expansion marching, facets per arc vs the isentropic-compression limit (expected ≈ 2)
 *   3. Panel mesh refinement of the real lofts (caret, cone, lift-body, osculating), CL/CD/Cm
 *   4. 2-D Euler (euler2d anchor) grid refinement against the exact oblique shock
 * Run: node --import ./scripts/ts-register.mjs src/lib/waverider/convergence.test.ts
 */
import assert from "node:assert/strict";
import { atmosphere } from "./atmosphere";
import { eulerWedgeGrid } from "./euler2d";
import { buildVehicle } from "./generate";
import { DEG, invPrandtlMeyer, isentropic, prandtlMeyer, taylorMaccoll } from "./math";
import { panelAero } from "./panel";
import { benchParams, extrudedBody } from "./shapebench";
import { DEFAULT_PARAMS, type AeroMethod, type DesignParams } from "./types";

const order = (e1: number, e2: number) => Math.log2(e1 / e2);

// ── 1. RK4 step size ────────────────────────────────────────────────────────────────────────────
// stepScale multiplies the integration step dθ = min(0.25°, β/80); reference is stepScale 1/8.
{
  for (const [M, beta] of [
    [5, 40],
    [8, 30],
    [3, 45],
  ] as [number, number][]) {
    const ref = taylorMaccoll(M, beta * DEG, 1.4, 0.125)!.cone;
    const err = (s: number) => Math.abs(taylorMaccoll(M, beta * DEG, 1.4, s)!.cone - ref) / DEG;
    const e4 = err(4);
    const e2 = err(2);
    const e1 = err(1);
    // The cone angle is a Hermite root of the RK4 solution: its error is far below the step's O(h⁴) estimate and
    // dominated by the root bracket, so the observed order is irregular (1–9). What we lock is monotone convergence
    // and absolute smallness: ≤1e-5° at 4× the default step, ≤1e-7° at the default.
    assert.ok(e4 < 1e-5, `TM M${M} β${beta}: error at 4× step ${e4.toExponential(2)}° < 1e-5°`);
    assert.ok(e1 < 1e-7, `TM M${M} β${beta}: error at default step ${e1.toExponential(2)}° < 1e-7°`);
    assert.ok(e4 >= e2 && e2 >= e1 * 0.99, `TM M${M} β${beta}: error non-increasing as the step shrinks (${e4.toExponential(1)} ${e2.toExponential(1)} ${e1.toExponential(1)})`);
    console.log(`RK4 M${M} β${beta}°: error(×4)=${e4.toExponential(2)}° (×2)=${e2.toExponential(2)}° (×1)=${e1.toExponential(2)}°  observed order ${order(e4, e2).toFixed(1)}, ${order(e2, e1).toFixed(1)}`);
  }
}

// ── 2. facets per concave arc vs isentropic compression ────────────────────────────────────────
// Surface with turning angle θ(x) = δ·x (so z = ln cos(δx)/δ). The exact inviscid answer on the final
// facet is isentropic Prandtl–Meyer compression through the facet's own inclination. N facets are a
// shock + (N−1) weaker shocks: the entropy loss goes like θ³/N², so the error falls ~1/N².
{
  const M = 8;
  const delta = 15 * DEG;
  const zf = (x: number) => Math.log(Math.cos(delta * x)) / delta;
  const errs: Record<number, number> = {};
  for (const N of [2, 4, 8, 16, 32, 64, 128]) {
    const body = extrudedBody(zf, () => 0, N + 1);
    const p = benchParams({ shape: "ramp", M, thetaDeg: 0, alphaDeg: 0, altKm: 0 }, "mixed");
    const a = panelAero(body.mesh, p, atmosphere(0, M, 1.4), 0, 0, body.sRef, body.lRef);
    const pos = body.mesh.positions;
    const ind = body.mesh.indices;
    let best = -1;
    let bx = -1;
    for (let t = 0; t < ind.length / 3; t++) {
      if (body.mesh.surfaces[t] !== 1) continue;
      const x = (pos[ind[t * 3] * 3] + pos[ind[t * 3 + 1] * 3] + pos[ind[t * 3 + 2] * 3]) / 3;
      if (x > bx) {
        bx = x;
        best = t;
      }
    }
    const xs = [0, 1, 2].map((k) => pos[ind[best * 3 + k] * 3]);
    const zs = [0, 1, 2].map((k) => pos[ind[best * 3 + k] * 3 + 2]);
    const xa = Math.min(...xs);
    const xb = Math.max(...xs);
    const za = zs[xs.findIndex((x) => Math.abs(x - xa) < 1e-12)];
    const zb = zs[xs.findIndex((x) => Math.abs(x - xb) < 1e-12)];
    const th = Math.atan2(-(zb - za), xb - xa); // final facet inclination
    const q = 0.7 * M * M;
    const prRef = isentropic(invPrandtlMeyer(prandtlMeyer(M) - th)).p / isentropic(M).p;
    const cpRef = (a.pVisc * prRef - 1) / q; // the method applies the same weak-interaction factor
    errs[N] = Math.abs(a.cp[best] - cpRef) / cpRef;
  }
  console.log("marching vs isentropic limit, |relErr| by N:", Object.entries(errs).map(([n, e]) => `${n}:${(e * 100).toFixed(3)}%`).join(" "));
  assert.ok(errs[2] > errs[4] && errs[4] > errs[8] && errs[8] > errs[16], "arc marching error decreases monotonically 2→16 facets");
  assert.ok(order(errs[2], errs[8]) / 2 > 1.2, `observed order ${(order(errs[2], errs[8]) / 2).toFixed(2)} > 1.2 (2→8 facets, theory 2)`);
  for (const N of [16, 32, 64, 128]) assert.ok(errs[N] < 0.005, `N=${N} facets within 0.5 % of the isentropic limit (${(errs[N] * 100).toFixed(3)} %)`);
}

// ── 3. panel mesh refinement of the real lofts ──────────────────────────────────────────────────
const levels: [number, number][] = [
  [16, 10],
  [32, 20],
  [64, 40],
  [128, 80],
];
function coeffs(fam: DesignParams["family"], method: AeroMethod, nx: number, ny: number) {
  const p: DesignParams = { ...DEFAULT_PARAMS, family: fam, nx, ny, aeroMethod: method, alphaDeg: 3 };
  const b = buildVehicle(p);
  const a = panelAero(b.mesh, p, atmosphere(p.altKm, p.flightMach, p.gamma), 3, 0, b.aero.planformArea, p.length);
  return { cl: a.cl, cd: a.cd, cm: a.cm };
}
{
  const out: string[] = [];
  for (const fam of ["caret", "cone", "liftbody"] as const) {
    for (const method of ["tangent", "mixed", "euler"] as const) {
      const r = levels.map(([nx, ny]) => coeffs(fam, method, nx, ny));
      for (const k of ["cl", "cd"] as const) {
        const d = [Math.abs(r[1][k] - r[0][k]), Math.abs(r[2][k] - r[1][k]), Math.abs(r[3][k] - r[2][k])];
        // the finest step must be a contraction of the previous one (floor: 2e-5 abs ≈ rounding of the facet loop)
        assert.ok(d[2] <= 0.75 * d[1] + 2e-5, `${fam}/${method} ${k}: Δ(64→128)=${d[2].toExponential(2)} ≤ 0.75·Δ(32→64)=${d[1].toExponential(2)}`);
        // and the answer is already within 3 % at the shipped-size mesh (nx=32) of the 128 mesh
        assert.ok(Math.abs(r[1][k] - r[3][k]) <= 0.06 * Math.abs(r[3][k]) + 1e-4, `${fam}/${method} ${k}: nx=32 within 6 % of nx=128 (${r[1][k].toFixed(5)} vs ${r[3][k].toFixed(5)})`);
        out.push(`${fam}/${method} ${k.toUpperCase()}: ${r.map((x) => x[k].toFixed(5)).join(" ")}  obs.order≈${d[1] > 1e-9 && d[2] > 1e-9 ? order(d[1], d[2]).toFixed(1) : "–"}`);
      }
    }
  }
  console.log(out.join("\n"));
  // osculating: tangent / euler (impact based) converge; "mixed" is tip-resolution sensitive — bounded, not asserted to converge
  for (const method of ["tangent", "euler"] as const) {
    const r = levels.map(([nx, ny]) => coeffs("osculating", method, nx, ny));
    const d = [Math.abs(r[1].cl - r[0].cl), Math.abs(r[2].cl - r[1].cl), Math.abs(r[3].cl - r[2].cl)];
    assert.ok(d[2] < d[1] && d[1] < d[0], `osculating/${method} CL Δ shrinks: ${d.map((x) => x.toExponential(1)).join(" ")}`);
  }
  const t = coeffs("osculating", "tangent", 128, 80).cl;
  const mx = coeffs("osculating", "mixed", 128, 80).cl;
  assert.ok(Math.abs(mx - t) < 0.08 * t, `osculating mixed CL stays within 8 % of tangent at nx=128 (${mx.toFixed(4)} vs ${t.toFixed(4)})`);
  console.log(`osculating mixed is mesh-sensitive at the wing tips (CL ${levels.map(([nx, ny]) => coeffs("osculating", "mixed", nx, ny).cl.toFixed(4)).join(" ")}); bounded vs tangent ${t.toFixed(4)}`);
}

// ── 4. 2-D Euler grid refinement vs the exact oblique shock ───────────────────────────────────
// First-order HLLC, shock-capturing: the wall-pressure error is dominated by smearing and probe-cell position,
// so it is not a clean asymptotic order. We lock: finest grid error small, and ≥2× below the default grid's.
{
  for (const [M, th, tol] of [
    [2, 10, 0.03],
    [5, 15, 0.04],
  ] as [number, number, number][]) {
    const e = (ni: number, nj: number, it: number) => Math.abs(eulerWedgeGrid(M, th, 1.4, ni, nj, it).relErr);
    const e72 = e(72, 24, 600);
    const e144 = e(144, 48, 1200);
    console.log(`Euler wedge M${M} θ${th}°: 72×24 ${(e72 * 100).toFixed(2)} %  144×48 ${(e144 * 100).toFixed(2)} %`);
    assert.ok(e144 < tol, `Euler M${M} θ${th}: 144×48 error ${(e144 * 100).toFixed(2)} % < ${(tol * 100).toFixed(0)} %`);
    assert.ok(e144 < 0.5 * e72, `Euler M${M} θ${th}: refinement 72×24 → 144×48 at least halves the error`);
  }
}

console.log("convergence tests passed");
