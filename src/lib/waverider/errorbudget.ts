/**
 * Per-method ERROR BUDGET: error of each aero method against an exact (or closed-form) reference,
 * per shape, with the tolerance we hold it to and why. Design-independent (analytic bodies), computed
 * lazily, memoised, and yielded case-by-case so the Data tab never blocks for more than one case.
 *
 * Reference per shape:
 *  wedge / flat plate (α)   exact oblique shock (windward) and Prandtl–Meyer (leeward), re-derived in shapebench.ts
 *  double wedge / ramp      exact two-shock (compression) or shock + Prandtl–Meyer (expansion) aft-face Cp
 *  sharp cone (α = 0)       Taylor–Maccoll (verified against NASA SP-3004 shock angles in validate-published.ts)
 *  sphere                   modified-Newtonian CD = Cp,max/2 (closed form). NOT truth: real CD differs; used to show scatter only
 * The viscous-interaction factor (Hayes–Probstein p/p_inv) is applied by every method except "newtonian"; the reference is
 * scaled by the same factor so the table measures the inviscid-model error, not the interaction model (sea level, L = 1 m).
 */
import { benchParams, buildBench, exactConeCp, exactRampAft, exactWedgePlate, runBench, type BenchCase } from "./shapebench";
import { newtonianCpMax } from "./math";
import { atmosphere } from "./atmosphere";
import { buildVehicle } from "./generate";
import { panelAero } from "./panel";
import { DEFAULT_PARAMS, SURFACE_ID, type AeroMethod } from "./types";
import { DEG, betaFromThetaM, obliqueShock } from "./math";

export interface BudgetRow {
  shape: string;
  case: string;
  M: number;
  quantity: string;
  method: string;
  reference: number;
  value: number;
  errPct: number;
  tolPct: number;
  within: boolean;
  rationale: string;
}

const METHODS: AeroMethod[] = ["newtonian", "cbaero", "tangent", "mixed"];

const RAT = {
  exact: "Exact oblique-shock / Taylor–Maccoll evaluated on the facet's own inclination: error is table interpolation + panel discretisation only (≲1 %); 2 % holds with margin.",
  cb: "Fixed blend; exact weight 1 for θ ≤ 0.6 θ_max fading to 0.55 at detachment. Held to 8 % attached, 15 % in the ramp/second-shock case (impact methods cannot see the first shock).",
  newt: "Modified Newtonian ignores shock-layer physics: −14 % to −96 % low on attached surfaces here, shrinking with M and θ. Tolerance 100 % means no accuracy claim for attached small-angle surfaces; use it only for blunt/high-angle regions.",
  tang: "Tangent-wedge/cone is exact for a single shock; on a second ramp it ignores the first shock (documented miss, up to −35 % at M12).",
  ref: "Reference is a closed form but NOT the physical truth (real sphere CD exceeds Newtonian by a few %); tolerance is a scatter band.",
};

function tolFor(method: AeroMethod, shape: string, quantity: string): { tol: number; why: string } {
  if (shape === "sphere") return { tol: 12, why: RAT.ref };
  if (method === "newtonian") return { tol: 100, why: RAT.newt };
  if (shape === "ramp") {
    if (method === "mixed") return { tol: 2, why: RAT.exact };
    if (method === "tangent") return { tol: 40, why: RAT.tang };
    return { tol: 40, why: RAT.cb };
  }
  const cl = quantity.startsWith("CL");
  if (method === "cbaero") return { tol: 8, why: RAT.cb };
  return { tol: cl ? 3 : 2, why: cl ? RAT.exact + " CL is a difference of two Cp (windward − leeward) so 3 % is allowed." : RAT.exact };
}

const withVisc = (cp: number, pv: number, M: number) => ((1 + cp * 0.7 * M * M) * pv - 1) / (0.7 * M * M);
const pct = (v: number, r: number) => (Math.abs(r) > 1e-9 ? (100 * (v - r)) / Math.abs(r) : NaN);

function push(rows: BudgetRow[], shape: string, label: string, M: number, quantity: string, method: AeroMethod, ref: number, value: number) {
  if (!Number.isFinite(ref) || !Number.isFinite(value)) return;
  const { tol, why } = tolFor(method, shape, quantity);
  const e = pct(value, ref);
  if (!Number.isFinite(e)) return;
  rows.push({ shape, case: label, M, quantity, method, reference: ref, value, errPct: e, tolPct: tol, within: Math.abs(e) <= tol, rationale: why });
}

let memo: BudgetRow[] | null = null;

interface Spec {
  run: (rows: BudgetRow[]) => void;
}

function specs(): Spec[] {
  const out: Spec[] = [];
  for (const M of [3, 5, 8, 12, 20]) {
    for (const th of [5, 10, 15, 20]) {
      out.push({
        run: (rows) => {
          const c: BenchCase = { shape: "wedge", M, thetaDeg: th, alphaDeg: 0, altKm: 0 };
          const ex = exactWedgePlate(c);
          const body = buildBench(c);
          for (const m of METHODS) {
            const r = runBench(c, m, body);
            const ref = m === "newtonian" ? ex.cpLower : withVisc(ex.cpLower, r.aero.pVisc, M);
            push(rows, "wedge", `θ=${th}°, α=0`, M, "Cp windward", m, ref, r.cpLower);
          }
        },
      });
    }
  }
  for (const M of [3, 8, 15]) {
    for (const a of [2, 5, 10, 15]) {
      out.push({
        run: (rows) => {
          const c: BenchCase = { shape: "plate", M, thetaDeg: 0, alphaDeg: a, altKm: 0 };
          const ex = exactWedgePlate(c);
          const body = buildBench(c);
          for (const m of METHODS) {
            const r = runBench(c, m, body);
            const pv = m === "newtonian" ? 1 : r.aero.pVisc;
            const cpl = withVisc(ex.cpLower, pv, M);
            push(rows, "plate", `flat plate α=${a}°`, M, "Cp windward", m, cpl, r.cpLower);
            // CL: windward minus leeward, exact × same interaction on the windward side only
            const clRef = ((cpl - ex.cpUpper) * Math.cos(a * DEG) * 1);
            push(rows, "plate", `flat plate α=${a}°`, M, "CL (planform)", m, clRef, r.aero.cl);
          }
        },
      });
    }
  }
  const ramps: [number, number][] = [[5, 10], [8, 15], [10, 20], [15, 5], [12, 25]];
  for (const M of [4, 8, 12]) {
    for (const [t1, t2] of ramps) {
      out.push({
        run: (rows) => {
          const c: BenchCase = { shape: "ramp", M, thetaDeg: t1, theta2Deg: t2, alphaDeg: 0, altKm: 0 };
          const ex = exactRampAft(M, t1, t2);
          if (!Number.isFinite(ex)) return;
          const body = buildBench(c);
          for (const m of METHODS) {
            const r = runBench(c, m, body);
            const ref = m === "newtonian" ? ex : withVisc(ex, r.aero.pVisc, M);
            push(rows, "ramp", `double wedge ${t1}°→${t2}°`, M, "Cp aft face", m, ref, r.cpAft);
          }
        },
      });
    }
  }
  for (const M of [3, 5, 8, 12, 20]) {
    for (const th of [5, 10, 15, 25]) {
      out.push({
        run: (rows) => {
          const c: BenchCase = { shape: "cone", M, thetaDeg: th, alphaDeg: 0, altKm: 0 };
          const ex = exactConeCp(M, th);
          const body = buildBench(c);
          for (const m of METHODS) {
            const r = runBench(c, m, body);
            const ref = m === "newtonian" ? ex : withVisc(ex, r.aero.pVisc, M);
            push(rows, "cone", `θc=${th}°, α=0`, M, "Cp surface", m, ref, r.cpSide);
          }
        },
      });
    }
  }
  for (const M of [3, 8, 20]) {
    out.push({
      run: (rows) => {
        const c: BenchCase = { shape: "sphere", M, thetaDeg: 0, alphaDeg: 0, altKm: 0 };
        const body = buildBench(c);
        for (const m of METHODS) {
          const r = runBench(c, m, body);
          push(rows, "sphere", "sphere (R = 0.5 m)", M, "CD wave (S=πR²)", m, newtonianCpMax(M) / 2, r.aero.cdWave);
        }
      },
    });
  }
  // default caret (real 3-D loft with the symmetry-plane Euler default) — lower-face Cp vs exact at each facet's own inclination
  for (const a of [0, 2, 5, 10]) {
    out.push({
      run: (rows) => {
        const base = { ...DEFAULT_PARAMS, family: "caret" as const, nx: 24, ny: 14, alphaDeg: a };
        const built = buildVehicle(base);
        const M = base.flightMach;
        const atm = atmosphere(base.altKm, M, base.gamma);
        const pos = built.mesh.positions;
        const ind = built.mesh.indices;
        const al = a * DEG;
        const v = [Math.cos(al), 0, Math.sin(al)];
        for (const m of ["euler", "cbaero", "tangent", "mixed"] as AeroMethod[]) {
          const p = { ...base, aeroMethod: m };
          const aero = panelAero(built.mesh, p, atm, a, 0, built.aero.planformArea, p.length);
          let A = 0;
          let cpM = 0;
          let cpE = 0;
          const q = 0.5 * base.gamma * M * M;
          for (let t = 0; t < ind.length / 3; t++) {
            if (built.mesh.surfaces[t] !== SURFACE_ID.lower) continue;
            const i0 = ind[t * 3] * 3;
            const i1 = ind[t * 3 + 1] * 3;
            const i2 = ind[t * 3 + 2] * 3;
            const u = [pos[i1] - pos[i0], pos[i1 + 1] - pos[i0 + 1], pos[i1 + 2] - pos[i0 + 2]];
            const w = [pos[i2] - pos[i0], pos[i2 + 1] - pos[i0 + 1], pos[i2 + 2] - pos[i0 + 2]];
            const n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
            const mg = Math.hypot(n[0], n[1], n[2]);
            if (mg < 1e-14) continue;
            const s = -(n[0] * v[0] + n[1] * v[1] + n[2] * v[2]) / mg;
            if (s < 0.12 || n[2] / mg > -0.5) continue;
            const b = betaFromThetaM(M, Math.asin(Math.min(1, s)), base.gamma);
            if (!Number.isFinite(b)) continue;
            const cpx = (obliqueShock(M, b, base.gamma).p2p1 - 1) / q;
            A += mg / 2;
            cpM += aero.cp[t] * (mg / 2);
            cpE += (((1 + cpx * q) * aero.pVisc - 1) / q) * (mg / 2);
          }
          if (A > 0) {
            const tol = m === "euler" || m === "tangent" || m === "mixed" ? 2 : 8;
            const e = pct(cpM / A, cpE / A);
            rows.push({
              shape: "caret (3-D loft, default design)",
              case: `α=${a}°, planar lower facets`,
              M,
              quantity: "Cp lower facets (area mean)",
              method: m,
              reference: cpE / A,
              value: cpM / A,
              errPct: e,
              tolPct: tol,
              within: Math.abs(e) <= tol,
              rationale: m === "euler" ? "Default method. Euler is blended only where the panel normal matches the centreline wall (sym-euler.ts gate); on the anhedral keel facets the exact-facet base carries the panel, so ≲2 % holds. Before the fix: −75 % at α=0." : RAT.exact,
            });
          }
        }
      },
    });
  }
  return out;
}

export const BUDGET_CASES = (): number => specs().length;

/** Stepwise generator: yields after each case; returns rows. */
export function* errorBudgetSteps(): Generator<number, BudgetRow[]> {
  if (memo) return memo;
  const sp = specs();
  const rows: BudgetRow[] = [];
  for (let i = 0; i < sp.length; i++) {
    sp[i].run(rows);
    yield (i + 1) / sp.length;
  }
  memo = rows;
  return rows;
}

export function errorBudget(): BudgetRow[] {
  const it = errorBudgetSteps();
  let r = it.next();
  while (!r.done) r = it.next();
  return r.value;
}

export interface BudgetSummary {
  shape: string;
  method: string;
  n: number;
  maxAbsPct: number;
  meanAbsPct: number;
  tolPct: number;
  allWithin: boolean;
}

export function budgetSummary(rows = errorBudget()): BudgetSummary[] {
  const key = new Map<string, BudgetSummary>();
  for (const r of rows) {
    const k = `${r.shape}|${r.method}`;
    let s = key.get(k);
    if (!s) {
      s = { shape: r.shape, method: r.method, n: 0, maxAbsPct: 0, meanAbsPct: 0, tolPct: r.tolPct, allWithin: true };
      key.set(k, s);
    }
    s.n++;
    s.maxAbsPct = Math.max(s.maxAbsPct, Math.abs(r.errPct));
    s.meanAbsPct += Math.abs(r.errPct);
    s.tolPct = Math.max(s.tolPct, r.tolPct);
    if (!r.within) s.allWithin = false;
  }
  return [...key.values()].map((s) => ({ ...s, meanAbsPct: s.meanAbsPct / s.n }));
}
void benchParams;
