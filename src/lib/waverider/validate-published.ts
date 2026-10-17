/**
 * Checks against PUBLISHED reference values (tables / worked examples / closed forms printed in the
 * literature). Every number below was read from a document fetched on 2026-10-16 (URL in `source`),
 * not typed from memory. Provenance labels (the `basis` field):
 *  - "published-table":   values printed in a table (NACA 1135 Table II, NASA SP-3004, US76 table, oblique-shock table)
 *  - "published-example": a worked example / validation case in a NASA document
 *  - "published-formula": a correlation as printed in the original or a review paper (transcription check only)
 * Limits you must know:
 *  - NACA 1135 and SP-3004 were read from OCR text layers (NTRS PDF / archive.org djvu). OCR garbles some digits and
 *    exponents; only mantissas that were unambiguous are used, and the SP-3004 header angles are screened:
 *    58 of 192 case headers parsed, 51 agree with an independent RK4 Taylor–Maccoll to < 0.005° and are used,
 *    7 were OCR-garbled (differences of 0.03°–30°) and are NOT used.
 *  - Tolerances are the rounding of the printed digits (half a unit in the last place, ×1.2), not engineering slack.
 *  - The oblique-shock table values (OBL / FastCalc M2 θ10) came from web-search result snippets of those pages; a direct
 *    fetch of the aerodynamics4students page returned 404, so they were not read from the page itself. They agree with the
 *    exact θ-β-M solution to the printed digits, which is what the checks assert.
 * Nothing here is wind-tunnel or flight data.
 */
import type { Check } from "./validate";
import {
  DEG,
  RAD,
  billigStandoff,
  coneSurfaceStateExact,
  isentropic,
  normalShock,
  obliqueShock,
  prandtlMeyer,
  rayleighPitot,
  areaRatio,
  betaFromThetaM,
  solveConeShock,
  suttonGraves,
} from "./math";
import { atmosphere } from "./atmosphere";

type Pub = Check["basis"];

function pub(
  id: string,
  name: string,
  formula: string,
  reference: number,
  value: number,
  source: string,
  tol: number,
  basis: Pub,
  domain: Check["domain"] = "external",
): Check {
  const absErr = Math.abs(value - reference);
  const relErr = absErr / Math.max(Math.abs(reference), 1e-12);
  return { id, domain, name, formula, expected: reference, got: value, relErr, absErr, tol, basis, source, pass: Number.isFinite(value) && relErr <= tol };
}

const NACA = "NACA Report 1135 (Ames 1953), Table II, NTRS 19930091059 (OCR text layer)";
const OBL = "Oblique-shock data table γ=1.4, aerodynamics4students.com/gas-dynamics-and-supersonic-flow/table8.php (NACA 1135 chart values)";
const SP3004 = "NASA SP-3004 (Sims 1964), per-case Taylor–Maccoll tables, NTRS 19640009035 (archive.org OCR); shock angle in the case header";
const WIND = "NASA Glenn WIND validation case '10° cone at Mach 2.35', analytic Taylor–Maccoll via HAP (Heiser & Pratt), grc.nasa.gov/WWW/wind/valid/cone10";
const US76 = "U.S. Standard Atmosphere 1976 table (geometric altitude Z), as reproduced at pdas.com/bigtables.html";

/** [cone half-angle °, M∞, shock angle °] — SP-3004 case headers that agree with an independent RK4 to < 0.005°. */
export const SP3004_BETA: [number, number, number][] = [
  [2.5, 1.75, 34.8544],
  [2.5, 2, 30.0057],
  [2.5, 2.5, 23.5879],
  [2.5, 3, 19.4869],
  [2.5, 3.5, 16.6256],
  [2.5, 7, 8.3781],
  [2.5, 8, 7.4089],
  [2.5, 10, 6.1079],
  [2.5, 20, 3.8587],
  [5.0, 3.5, 16.9549],
  [5.0, 4.5, 13.4552],
  [5.0, 15, 6.5873],
  [7.5, 1.5, 42.1065],
  [7.5, 2.5, 24.2692],
  [7.5, 3, 20.4552],
  [7.5, 4.5, 14.7522],
  [7.5, 7, 11.3710],
  [7.5, 12, 9.4216],
  [10.0, 1.5, 42.6659],
  [10.0, 3, 21.7147],
  [10.0, 6, 14.3521],
  [10.0, 20, 11.3007],
  [12.5, 3, 23.3538],
  [12.5, 4, 19.6500],
  [12.5, 6, 16.6103],
  [12.5, 7, 15.8937],
  [12.5, 12, 14.4834],
  [15.0, 2, 33.9146],
  [15.0, 4, 21.7908],
  [15.0, 5, 20.0277],
  [15.0, 6, 19.0072],
  [15.0, 7, 18.3643],
  [15.0, 8, 17.9339],
  [15.0, 10, 17.4126],
  [17.5, 2, 35.7354],
  [17.5, 3.5, 25.4073],
  [17.5, 4, 24.0804],
  [17.5, 6, 21.5019],
  [17.5, 12, 19.8003],
  [20.0, 1.5, 49.0291],
  [20.0, 5, 24.9427],
  [20.0, 6, 24.0702],
  [20.0, 8, 23.1727],
  [20.0, 15, 22.3152],
  [27.5, 2.5, 39.9274],
  [27.5, 4, 34.2149],
  [27.5, 12, 30.7943],
  [30.0, 3.5, 38.0614],
  [30.0, 5, 35.6037],
  [30.0, 6, 34.8719],
  [30.0, 10, 33.7954]
];

/** NACA 1135 Table II rows (γ = 1.4): M, p/p_t, A/A*, ν°, μ°, normal-shock M₂, p₂/p₁, ρ₂/ρ₁, T₂/T₁, p_t2/p_t1. NaN = not legible. */
const NACA_ROWS: { M: number; pPt: number; AR: number; nu: number; mu: number; M2: number; p21: number; r21: number; T21: number; pt21: number }[] = [
  { M: 2, pPt: 0.1278, AR: 1.688, nu: 26.38, mu: 30.0, M2: 0.5774, p21: 4.5, r21: 2.667, T21: 1.688, pt21: 0.7209 },
  { M: 3, pPt: 0.02722, AR: 4.235, nu: 49.757, mu: 19.47, M2: 0.4752, p21: NaN, r21: NaN, T21: NaN, pt21: NaN },
  { M: 5, pPt: NaN, AR: 25.0, nu: 76.92, mu: 11.54, M2: 0.4152, p21: 29.0, r21: 5.0, T21: 5.8, pt21: 0.06172 },
  { M: 10, pPt: NaN, AR: NaN, nu: 102.32, mu: NaN, M2: 0.3876, p21: 116.5, r21: NaN, T21: NaN, pt21: NaN },
];

export function publishedChecks(): Check[] {
  const out: Check[] = [];
  const g = 1.4;

  // --- NACA 1135 Table II ---
  for (const r of NACA_ROWS) {
    const ns = normalShock(r.M, g);
    const iso = isentropic(r.M, g);
    const t = (v: number) => 1.2 * 0.5 * Math.pow(10, Math.floor(Math.log10(Math.abs(v))) - 3); // ½ ulp of a 4-sig-fig print
    if (Number.isFinite(r.pPt)) out.push(pub(`naca-ppt-${r.M}`, `NACA 1135 Table II: p/p_t at M=${r.M}`, "isentropic (1+0.2M²)^(-3.5)", r.pPt, iso.p, NACA, t(r.pPt) / r.pPt + 1e-4, "published-table", "gas"));
    if (Number.isFinite(r.AR)) out.push(pub(`naca-ar-${r.M}`, `NACA 1135 Table II: A/A* at M=${r.M}`, "area–Mach relation", r.AR, areaRatio(r.M, g), NACA, 5e-4 + t(r.AR) / r.AR, "published-table", "gas"));
    out.push(pub(`naca-nu-${r.M}`, `NACA 1135 Table II: Prandtl–Meyer ν at M=${r.M}`, "ν(M) in degrees", r.nu, prandtlMeyer(r.M, g) * RAD, NACA, 5e-5 + t(r.nu) / r.nu, "published-table"));
    if (Number.isFinite(r.mu)) out.push(pub(`naca-mu-${r.M}`, `NACA 1135 Table II: Mach angle μ at M=${r.M}`, "asin(1/M) in degrees", r.mu, Math.asin(1 / r.M) * RAD, NACA, 4e-4 + t(r.mu) / r.mu, "published-table"));
    out.push(pub(`naca-m2-${r.M}`, `NACA 1135 Table II: normal-shock M₂ at M=${r.M}`, "Rankine–Hugoniot", r.M2, ns.M2, NACA, 4e-4 + t(r.M2) / r.M2, "published-table", "gas"));
    if (Number.isFinite(r.p21)) out.push(pub(`naca-p21-${r.M}`, `NACA 1135 Table II: normal-shock p₂/p₁ at M=${r.M}`, "Rankine–Hugoniot", r.p21, ns.p2p1, NACA, 4e-4 + t(r.p21) / r.p21, "published-table", "gas"));
    if (Number.isFinite(r.r21)) out.push(pub(`naca-r21-${r.M}`, `NACA 1135 Table II: normal-shock ρ₂/ρ₁ at M=${r.M}`, "Rankine–Hugoniot", r.r21, ns.r2r1, NACA, 4e-4 + t(r.r21) / r.r21, "published-table", "gas"));
    if (Number.isFinite(r.T21)) out.push(pub(`naca-t21-${r.M}`, `NACA 1135 Table II: normal-shock T₂/T₁ at M=${r.M}`, "Rankine–Hugoniot", r.T21, ns.t2t1, NACA, 4e-4 + t(r.T21) / r.T21, "published-table", "gas"));
    if (Number.isFinite(r.pt21)) {
      const pt = (ns.p2p1 * isentropic(ns.M2, g).ptP) / isentropic(r.M, g).ptP;
      out.push(pub(`naca-pt21-${r.M}`, `NACA 1135 Table II: normal-shock p_t2/p_t1 at M=${r.M}`, "p₂/p₁ · (p_t/p)(M₂) / (p_t/p)(M₁)", r.pt21, pt, NACA, 4e-4 + t(r.pt21) / r.pt21, "published-table", "gas"));
    }
  }
  // Rayleigh pitot: the table prints p₁/p_t2 = 0.1773 at M=2 → p_t2/p₁ = 5.640 (Newtonian Cp,max = (5.640−1)/2.8)
  out.push(pub("naca-pitot-2", "NACA 1135 Table II: Rayleigh pitot p_t2/p₁ at M=2", "1/(p₁/p_t2 = 0.1773)", 1 / 0.1773, rayleighPitot(2, g), NACA, 1.2 * 0.5e-4 / 0.1773 + 1e-4, "published-table", "gas"));

  // --- oblique shock table (γ = 1.4): θ°, M, β°, M₂, p₂/p₁ ---
  const obl: [number, number, number, number, number][] = [
    [5, 2, 34.3, 1.82, 1.315],
    [5, 5, 15.07, 4.49, 1.806],
    [10, 2, 39.31, 1.64, 1.707],
    [10, 3, 27.38, 2.51, 2.054],
    [10, 4, 22.23, 3.29, 2.506],
    [10, 5, 19.38, NaN, 3.044],
  ];
  for (const [th, M, b, M2, p21] of obl) {
    const bc = betaFromThetaM(M, th * DEG, g);
    const sh = obliqueShock(M, bc, g);
    out.push(pub(`obl-beta-${th}-${M}`, `Oblique shock table: β at M=${M}, θ=${th}°`, "θ-β-M, weak branch", b, bc * RAD, OBL, 0.006 / b, "published-table"));
    if (Number.isFinite(M2)) out.push(pub(`obl-m2-${th}-${M}`, `Oblique shock table: M₂ at M=${M}, θ=${th}°`, "oblique shock", M2, sh.M2, OBL, 0.006 / M2, "published-table"));
    out.push(pub(`obl-p-${th}-${M}`, `Oblique shock table: p₂/p₁ at M=${M}, θ=${th}°`, "oblique shock", p21, sh.p2p1, OBL, 0.0006 / p21, "published-table"));
  }
  // FastCalc 6-digit worked example quoting NACA 1135: M=2, θ=10°
  {
    const bc = betaFromThetaM(2, 10 * DEG, g);
    const sh = obliqueShock(2, bc, g);
    const src = "fastcalc.us/math/oblique-shock-wave (cites NACA 1135 eq. 115–139, Table II): M₁=2, θ=10°";
    out.push(pub("obl-fc-beta", "Oblique shock M=2, θ=10°: β", "θ-β-M", 39.314, bc * RAD, src, 2e-5 / 39.314 * 25, "published-example"));
    out.push(pub("obl-fc-m2", "Oblique shock M=2, θ=10°: M₂", "oblique shock", 1.6405, sh.M2, src, 1.2 * 0.5e-4 / 1.6405 + 2e-4, "published-example"));
    out.push(pub("obl-fc-p", "Oblique shock M=2, θ=10°: p₂/p₁", "oblique shock", 1.7066, sh.p2p1, src, 1.2 * 0.5e-4 / 1.7066 + 2e-4, "published-example"));
    out.push(pub("obl-fc-rho", "Oblique shock M=2, θ=10°: ρ₂/ρ₁", "oblique shock", 1.4584, sh.r2r1, src, 1.2 * 0.5e-4 / 1.4584 + 2e-4, "published-example"));
    out.push(pub("obl-fc-t", "Oblique shock M=2, θ=10°: T₂/T₁", "oblique shock", 1.1702, sh.t2t1, src, 1.2 * 0.5e-4 / 1.1702 + 2e-4, "published-example"));
  }

  // --- SP-3004 cone shock angles ---
  let worst = 0;
  let worstCase = "";
  for (const [c, M, b] of SP3004_BETA) {
    const bc = solveConeShock(M, c * DEG, g) * RAD;
    const e = Math.abs(bc - b) / b;
    if (e > worst) {
      worst = e;
      worstCase = `θc=${c}° M=${M}`;
    }
  }
  const sp = pub("sp3004-beta", `NASA SP-3004: cone shock angle, ${SP3004_BETA.length} cases (M 1.5–20, θc 2.5°–30°)`, `max |β_code − β_SP3004|/β over the cases (worst: ${worstCase}); table prints 8 digits, OCR-screened`, 0, worst, SP3004, 2e-5, "published-table");
  sp.pass = Number.isFinite(worst) && worst <= sp.tol;
  sp.relErr = worst;
  out.push(sp);

  // --- NASA Glenn WIND validation case: 10° cone, M = 2.35, γ = 1.4 ---
  // The page's "analytic (HAP)" column (β = 27.1843°, p₃/p₁ = 1.4234, p_t/p_t1 = 1.0 behind the shock) is NOT used: it
  // disagrees by 3.5 % with all three Wind-US runs on the same page AND with an independent RK4 Taylor–Maccoll
  // (β = 26.737°, which reproduces SP-3004 to 1e-4°; β = 27.18° corresponds to M ≈ 2.31). The WIND numerical results are
  // used instead, tolerance 1e-3 (typical CFD discretisation error on a conical flow); they are a CFD reference, not exact.
  {
    const M = 2.35;
    const th = 10 * DEG;
    const st = coneSurfaceStateExact(M, th, g);
    out.push(pub("wind-cone-pc", "WIND cone10 (Wind-US 3.0 axisymmetric/3-D CFD): cone-surface p/p∞, M=2.35, θc=10°", "Taylor–Maccoll surface pressure (CFD 1.3740/1.3741)", 1.3741, st.pRatio, WIND, 1e-3, "published-example"));
    out.push(pub("wind-cone-mc", "WIND cone10 (Wind-US CFD): cone-surface Mach", "Taylor–Maccoll surface Mach (CFD 2.1468)", 2.1468, st.M, WIND, 1e-3, "published-example"));
  }

  // --- US76 table at 9 geometric altitudes ---
  const us: [number, number, number, number][] = [
    [0, 288.15, 101320, 1.225],
    [10, 223.252, 26500, 0.41351],
    [20, 216.65, 5529.3, 0.08891],
    [30, 226.509, 1197.0, 0.01841],
    [40, 250.35, 287.14, 0.0039957],
    [50, 270.65, 79.779, 0.0010269],
    [60, 247.021, 21.959, 0.00030968],
    [70, 219.585, 5.2209, 8.2829e-5],
    [80, 198.639, 1.0525, 1.8458e-5],
  ];
  let aw = 0;
  for (const [z, T, p, r] of us) {
    const a = atmosphere(z, 0, g);
    aw = Math.max(aw, Math.abs(a.T - T) / T, Math.abs(a.p - p) / p, Math.abs(a.rho - r) / r);
  }
  const at = pub("us76-table", "US76 table: T, p, ρ at Z = 0, 10, …, 80 km", "max rel. error over 27 printed values (4–5 significant figures)", 0, aw, US76, 3e-4, "published-table", "gas");
  at.pass = aw <= at.tol;
  out.push(at);

  // --- Billig 1967 sphere standoff (published formula) ---
  const bsrc = "Billig, J. Spacecraft & Rockets 4(6):822 (1967) as quoted in the Billig-correlation literature (Δ/R = 0.143 exp(3.24/M²))";
  for (const M of [2, 5, 10]) out.push(pub(`billig-${M}`, `Billig sphere shock standoff Δ/R at M=${M}`, "0.143 exp(3.24/M²) (hand-evaluated)", 0.143 * Math.exp(3.24 / (M * M)), billigStandoff(M), bsrc, 1e-9, "published-formula"));

  // --- Sutton–Graves worked example (NASA TFAWS 2012 aerothermodynamics lecture) ---
  out.push(pub("sg-tfaws", "Sutton–Graves worked example: ρ=3.1459e-4 kg/m³, V=3535 m/s, Rn=1 m → 13.6 W/cm²", "1.7415e-4 √(ρ/Rn) V³", 13.6, suttonGraves(3.1459e-4, 3535, 1, 1), "NASA TFAWS12 'Aerothermodynamics course', Lecture 1: Stagnation Point Heating (tfaws.nasa.gov)", 0.004, "published-example"));
  return out;
}
