/**
 * Closed-form + published-table checks for the gas-dynamic kernel.
 * These do not use the mesh. They run in well under a millisecond.
 *
 * Sources: Anderson, Modern Compressible Flow; NACA 1135; Sims NASA SP-3004
 * conical-flow tables; US Standard Atmosphere 1976; Heiser & Pratt; Lees
 * modified Newtonian; Sutton & Graves NASA TR R-802; Tauber NASA TP-2914;
 * Kantrowitz & Donaldson NACA WR L-713; van Driest II / Hopkins–Inouye.
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
  skinCf,
  solveConeShock,
  suttonGraves,
  taylorMaccoll,
  thetaFromBetaM,
  vanDriestII,
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

  const iso5 = isentropic(5, 1.4);
  out.push(chk("isen-T", "gas", "Isentropic M=5, T₀/T", "1 + ½(γ−1)M² = 6", 6, iso5.TtT, "Anderson Table 3.1", 1e-9));
  out.push(
    chk("isen-A", "gas", "Isentropic M=5, A/A*", "(1/M)[(1+½(γ−1)M²)/(½(γ+1))]^(…) = 25", 25, areaRatio(5, 1.4), "Anderson Table 3.1", 1e-6),
  );

  out.push(
    chk("inv-A", "internal", "Newton invert A/A*=25 → M", "M = 5 (supersonic branch)", 5, machFromArea(25, 1.4, true), "same identity", 2e-4),
  );

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

  const betaW = betaFromThetaM(2, 10 * DEG, 1.4) * RAD;
  out.push(
    chk("tbm", "external", "θ-β-M  M=2, θ=10°  (weak β)", "tan θ = 2 cot β (M²sin²β−1) / (M²(γ+cos2β)+2)", 39.32, betaW, "Anderson Table 4.2", 0.004),
  );

  const thBack = thetaFromBetaM(2, betaW * DEG, 1.4) * RAD;
  out.push(chk("tbm-rt", "external", "θ-β-M round-trip θ", "θ(β(θ)) = θ", 10, thBack, "identity", 0.003));

  const obl = obliqueShock(2, 40 * DEG, 1.4);
  const Mn = 2 * Math.sin(40 * DEG);
  const p2exp = 1 + (2.8 / 2.4) * (Mn * Mn - 1);
  out.push(chk("obl-p", "external", "Oblique M=2, β=40°, p₂/p₁", "1 + 2γ/(γ+1)(M²sin²β−1)", p2exp, obl.p2p1, "Rankine–Hugoniot on Mn", 1e-6));

  const cot8 = 1 / Math.tan(8 * DEG);
  out.push(chk("cot", "external", "Inviscid 2-D wedge L/D, θ=8°", "L/D = cot θ", 7.11537, cot8, "Nonweiler / exact force ratio", 1e-5));

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
      "Sims NASA SP-3004 (round-trip)",
      0.03,
    ),
  );
  out.push(
    chk(
      "sims-beta",
      "external",
      "Cone shock β  M=8, θc=10°",
      "Taylor–Maccoll shock angle vs Sims table",
      12.95,
      betaC * RAD,
      "Sims NASA SP-3004",
      0.04,
    ),
  );

  out.push(chk("cpinf", "external", "Modified Newtonian Cp_max, M=20", "(p₀₂/p∞−1)/(½γM²) → 1.839", 1.839, newtonianCpMax(20, 1.4), "Lees / Rayleigh pitot", 0.02));

  const sl = atmosphere(0, 0, 1.4);
  out.push(chk("us76-T", "gas", "US76 sea-level T", "T = 288.15 K", 288.15, sl.T, "U.S. Standard Atmosphere 1976", 1e-4));
  out.push(chk("us76-p", "gas", "US76 sea-level p", "p = 101325 Pa", 101325, sl.p, "U.S. Standard Atmosphere 1976", 1e-4));

  const km30 = atmosphere(30, 0, 1.4);
  out.push(chk("us76-30", "gas", "US76 30 km T", "T ≈ 226.65 K (geopotential layer)", 226.65, km30.T, "U.S. Standard Atmosphere 1976", 0.01));

  const K3 = kantrowitz(3, 1.4);
  out.push(chk("kant", "internal", "Kantrowitz A_t/A_c, M=3", "(A/A*)_{M₂,NS} / (A/A*)_{M=3}", 0.328, K3, "Kantrowitz & Donaldson 1945", 0.04));

  out.push(
    chk(
      "sg-unit",
      "external",
      "Sutton–Graves units",
      "q = 1.83×10⁻⁸ √(ρ/Rn) V³   (ρ=1, Rn=1, V=1000) = 18.3 W/cm²",
      18.3,
      suttonGraves(1, 1000, 1, 1),
      "Sutton & Graves NASA TR R-802 / Tauber TP-2914",
      0.002,
    ),
  );

  const cfInc = skinCf(1e7);
  out.push(
    chk(
      "schlicht",
      "external",
      "Prandtl–Schlichting Cf, Re=10⁷",
      "Cf = 0.455 / (log₁₀ Re)²",
      0.455 / 49,
      cfInc,
      "Schlichting, Boundary Layer Theory",
      0.002,
    ),
  );

  const cfVD = vanDriestII(1e7, 0.1, 288, 288, 1.4);
  out.push(
    chk(
      "vd2-incomp",
      "external",
      "van Driest II → Schlichting as M→0",
      "Cf(M=0.1) ≈ 0.455/(log₁₀ Re)²",
      cfInc,
      cfVD,
      "Hopkins & Inouye 1971 / van Driest 1956",
      0.02,
    ),
  );

  return out;
}

export function validationSummary(checks = runValidation()) {
  const n = checks.length;
  const pass = checks.filter((c) => c.pass).length;
  return { n, pass, fail: n - pass, ok: pass === n };
}
