/**
 * Closed-form + published-table checks for the gas-dynamic kernel.
 * These do not use the mesh. They run in well under a millisecond.
 *
 * Sources: Anderson, Modern Compressible Flow; NACA 1135; Sims conical-flow
 * tables; US Standard Atmosphere 1976; Heiser & Pratt, Hypersonic Airbreathing
 * Propulsion.
 */

import {
  DEG,
  RAD,
  areaRatio,
  betaFromThetaM,
  isentropic,
  kantrowitz,
  machFromArea,
  newtonianCpMax,
  normalShock,
  obliqueShock,
  prandtlMeyer,
  solveConeShock,
  taylorMaccoll,
  thetaFromBetaM,
} from "./math";
import { atmosphere } from "./atmosphere";

export interface Check {
  id: string;
  domain: "gas" | "external" | "internal";
  name: string;
  formula: string;
  expected: number;
  got: number;
  relErr: number;
  pass: boolean;
  source: string;
}

function chk(
  id: string,
  domain: Check["domain"],
  name: string,
  formula: string,
  expected: number,
  got: number,
  source: string,
  tol = 0.012,
): Check {
  const relErr = Math.abs(got - expected) / Math.max(Math.abs(expected), 1e-12);
  return { id, domain, name, formula, expected, got, relErr, pass: relErr <= tol || Math.abs(got - expected) < 1e-4, source };
}

export function runValidation(): Check[] {
  const out: Check[] = [];

  // — exact Rankine–Hugoniot —
  const ns2 = normalShock(2, 1.4);
  out.push(
    chk(
      "ns-p",
      "gas",
      "Normal shock M=2, p₂/p₁",
      "1 + 2γ/(γ+1) (M²−1) = 4.5",
      4.5,
      ns2.p2p1,
      "Anderson / NACA 1135",
      1e-6,
    ),
  );
  out.push(
    chk(
      "ns-m",
      "gas",
      "Normal shock M=2, M₂",
      "M₂ = √[(M²+2/(γ−1)) / (2γM²/(γ−1)−1)] = 1/√3",
      1 / Math.sqrt(3),
      ns2.M2,
      "Anderson Table 3.2",
      1e-5,
    ),
  );

  // — isentropic, M=5 gives A/A* = 25 and T0/T = 6 exactly for γ=1.4 —
  const iso5 = isentropic(5, 1.4);
  out.push(chk("isen-T", "gas", "Isentropic M=5, T₀/T", "1 + ½(γ−1)M² = 6", 6, iso5.TtT, "Anderson Table 3.1", 1e-9));
  out.push(
    chk("isen-A", "gas", "Isentropic M=5, A/A*", "(1/M)[(1+½(γ−1)M²)/(½(γ+1))]^(…) = 25", 25, areaRatio(5, 1.4), "Anderson Table 3.1", 1e-6),
  );

  // invert A/A*
  out.push(
    chk("inv-A", "internal", "Newton invert A/A*=25 → M", "M = 5 (supersonic branch)", 5, machFromArea(25, 1.4, true), "same identity", 2e-4),
  );

  // — Prandtl–Meyer M=2 —
  const nu2 = prandtlMeyer(2, 1.4) * RAD;
  out.push(
    chk(
      "pm-2",
      "external",
      "Prandtl–Meyer M=2",
      "ν = √((γ+1)/(γ−1)) tan⁻¹√[…] − tan⁻¹√(M²−1)",
      26.3798,
      nu2,
      "Anderson Table 4.1",
      0.002,
    ),
  );

  // — θ-β-M: Anderson M=2, θ=10°, weak β ≈ 39.32° —
  const betaW = betaFromThetaM(2, 10 * DEG, 1.4) * RAD;
  out.push(
    chk("tbm", "external", "θ-β-M  M=2, θ=10°  (weak β)", "tan θ = 2 cot β (M²sin²β−1) / (M²(γ+cos2β)+2)", 39.32, betaW, "Anderson Table 4.2", 0.004),
  );

  // recover θ from that β
  const thBack = thetaFromBetaM(2, betaW * DEG, 1.4) * RAD;
  out.push(chk("tbm-rt", "external", "θ-β-M round-trip θ", "θ(β(θ)) = θ", 10, thBack, "identity", 0.003));

  // oblique p2/p1 at M=2, β=40°
  const obl = obliqueShock(2, 40 * DEG, 1.4);
  // Mn = 2 sin 40° = 1.2856; p2p1 = 1 + 2.8/2.4 * (Mn²-1) = 1 + 1.1667*(1.6528-1) = 1.761
  const Mn = 2 * Math.sin(40 * DEG);
  const p2exp = 1 + (2.8 / 2.4) * (Mn * Mn - 1);
  out.push(chk("obl-p", "external", "Oblique M=2, β=40°, p₂/p₁", "1 + 2γ/(γ+1)(M²sin²β−1)", p2exp, obl.p2p1, "Rankine–Hugoniot on Mn", 1e-6));

  // — 2-D wedge L/D —
  const cot8 = 1 / Math.tan(8 * DEG);
  out.push(chk("cot", "external", "Inviscid 2-D wedge L/D, θ=8°", "L/D = cot θ", 7.11537, cot8, "Nonweiler / exact force ratio", 1e-5));

  // — Taylor–Maccoll round-trip —
  const betaC = solveConeShock(8, 10 * DEG, 1.4);
  const tm = taylorMaccoll(8, betaC, 1.4);
  const coneDeg = tm ? tm.cone * RAD : NaN;
  out.push(
    chk(
      "tm",
      "external",
      "Taylor–Maccoll RK4  M=8, θc=10°",
      "d²v_r/dθ² + v_r = …  (RK4, recover cone)",
      10,
      coneDeg,
      "Sims conical-flow tables (round-trip)",
      0.03,
    ),
  );

  // — Newtonian high-M Cp_max → ~1.84 for γ=1.4 —
  out.push(chk("cpinf", "external", "Modified Newtonian Cp_max, M=20", "(p₀₂/p∞−1)/(½γM²) → 1.839", 1.839, newtonianCpMax(20, 1.4), "Lees / Rayleigh pitot", 0.02));

  // — US76 sea level —
  const sl = atmosphere(0, 0, 1.4);
  out.push(chk("us76-T", "gas", "US76 sea-level T", "T = 288.15 K", 288.15, sl.T, "U.S. Standard Atmosphere 1976", 1e-4));
  out.push(chk("us76-p", "gas", "US76 sea-level p", "p = 101325 Pa", 101325, sl.p, "U.S. Standard Atmosphere 1976", 1e-4));

  // — Kantrowitz at M=3 is ~0.33 —
  const K3 = kantrowitz(3, 1.4);
  out.push(chk("kant", "internal", "Kantrowitz A_t/A_c, M=3", "(A/A*)_{M₂,NS} / (A/A*)_{M=3}", 0.328, K3, "Kantrowitz & Donaldson 1945", 0.04));

  return out;
}

export function validationSummary(checks = runValidation()) {
  const n = checks.length;
  const pass = checks.filter((c) => c.pass).length;
  return { n, pass, fail: n - pass, ok: pass === n };
}
