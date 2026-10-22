/**
 * Full aerodynamic database.
 *
 * Sweeps Mach, angle of attack, sideslip, and control deflection through the
 * existing panel method (`panelAero`). Geometry is rebuilt only when the
 * deflection changes (elevon / faired-fin shear). Flight Mach is applied with
 * `lockFlight` off, so the inverse-design shape stays frozen and every other
 * Mach is an off-design point in the same table.
 *
 * Static derivatives are central differences of that table, per radian.
 * Dynamic derivatives use the same body-rate perturbation as the linear 6DOF
 * (local velocity V − ω × r) and are stored at β = 0, δ nearest 0, α nearest
 * the design angle, once per Mach. Symmetry-plane Euler, when requested, is an
 * extra β = 0, δ = 0 slice — the kernel is a centerline solve, not a 3-D table.
 *
 * Angles in the tables are degrees. Derivatives are per radian. Forces are
 * wind-axis (CL, CD, CY). Moments are body-axis, nondimensional on Sref × Lref,
 * matching `panel.ts`.
 */
import { atmosphere } from "./atmosphere";
import { buildVehicle } from "./generate";
import { DEG, clamp } from "./math";
import { panelAero, type RateState } from "./panel";
import type { AeroMethod, BuiltVehicle, DesignParams, TriMesh } from "./types";

export const AERODB_POINT_CAP = 480;

export interface AeroDbSpec {
  machMin: number;
  machMax: number;
  machStep: number;
  alphaMin: number;
  alphaMax: number;
  alphaStep: number;
  betaMin: number;
  betaMax: number;
  betaStep: number;
  deltaMin: number;
  deltaMax: number;
  deltaStep: number;
  /** Panel method for the coefficient grid. Euler is allowed but slow. */
  method: AeroMethod;
  /** Extra symmetry-plane Euler slice at β = 0, δ = 0. */
  eulerSymmetry: boolean;
  /** Body-rate derivatives at the anchor (α≈design, β=0, δ≈0) of each Mach. */
  dynamicDerivatives: boolean;
  altKm: number;
}

export interface AeroDbMeta {
  format: "cuspis-aerodb-1";
  name: string;
  family: string;
  method: AeroMethod;
  designMach: number;
  altitude_km: number;
  sRef_m2: number;
  lRef_m: number;
  bRef_m: number;
  cg_m: [number, number, number];
  units: {
    angles: "deg";
    derivatives: "1/rad";
    force: "wind axes, CL CD CY";
    moment: "body axes, Cm Cn Cll, nondim by q·Sref·Lref";
  };
  frame: "X streamwise (tip at origin, flow +X), Y span, Z up";
  index: "flat i = ((im*nAlpha + ia)*nBeta + ib)*nDelta + id  (delta fastest)";
  generatedAt: string;
  assumptions: string[];
}

export interface AeroDbPoint {
  mach: number;
  alpha_deg: number;
  beta_deg: number;
  delta_deg: number;
  onDesign: boolean;
  cl: number;
  cd: number;
  cy: number;
  cm: number;
  cn: number;
  cll: number;
  ca: number;
  ld: number;
  /** Per radian. 0 when the stencil does not exist (axis edge or a skipped derivative). */
  cla: number;
  cda: number;
  cma: number;
  cyb: number;
  cnb: number;
  clb: number;
  cld: number;
  cmd: number;
  clq: number;
  cmq: number;
  clp: number;
  cnp: number;
  cyp: number;
  cnr: number;
  clr: number;
  cyr: number;
  euler_cl: number | null;
  euler_cd: number | null;
  euler_cm: number | null;
}

export interface AeroDbSummary {
  points: number;
  onDesign: number;
  offDesign: number;
  maxLd: number;
  maxLdAt: { mach: number; alpha_deg: number; beta_deg: number; delta_deg: number };
  clMin: number;
  clMax: number;
  cdMin: number;
  cdMax: number;
  cmAtDesign: number;
}

export interface AeroDbSlice {
  mach: number;
  alpha: { alpha: number; cl: number; cd: number; cm: number; ld: number }[];
}

export interface AeroDynamicRow {
  mach: number;
  alpha_deg: number;
  delta_deg: number;
  clq: number;
  cmq: number;
  clp: number;
  cnp: number;
  cyp: number;
  cnr: number;
  clr: number;
  cyr: number;
}

export interface AeroDatabase {
  meta: AeroDbMeta;
  axes: { mach: number[]; alpha_deg: number[]; beta_deg: number[]; delta_deg: number[] };
  points: AeroDbPoint[];
  dynamics: AeroDynamicRow[];
  summary: AeroDbSummary;
  slices: { design: AeroDbSlice; offDesign: AeroDbSlice };
}

export interface AeroDbProgress {
  done: number;
  total: number;
  label: string;
}

export function defaultAeroDbSpec(p: DesignParams): AeroDbSpec {
  const M = clamp(p.lockFlight ? p.mach : p.flightMach, 1.5, 24);
  return {
    machMin: clamp(M - 2, 1.5, 24),
    machMax: clamp(M + 2, 1.5, 26),
    machStep: 2,
    alphaMin: -4,
    alphaMax: 12,
    alphaStep: 4,
    betaMin: 0,
    betaMax: 0,
    betaStep: 4,
    deltaMin: 0,
    deltaMax: 0,
    deltaStep: 5,
    method: p.aeroMethod === "euler" ? "cbaero" : p.aeroMethod,
    eulerSymmetry: false,
    dynamicDerivatives: true,
    altKm: p.altKm,
  };
}

/** Inclusive breakpoints. Always contains endpoints and any `force` values inside the range. Caps at 21. */
export function axisBreaks(min: number, max: number, step: number, force: number[] = []): number[] {
  const lo = Math.min(min, max);
  const hi = Math.max(min, max);
  const span = hi - lo;
  const st = Math.max(Math.abs(step) || span || 1, 1e-4);
  const nRaw = Math.floor(span / st + 1e-6) + 1;
  const cap = 21;
  const raw: number[] = [];
  if (nRaw <= cap) {
    for (let i = 0; i < nRaw; i++) raw.push(lo + i * st);
    if (raw[raw.length - 1] < hi - st * 0.25) raw.push(hi);
  } else {
    for (let i = 0; i < cap; i++) raw.push(lo + (span * i) / (cap - 1));
  }
  for (const f of force) {
    if (f >= lo - 1e-8 && f <= hi + 1e-8) raw.push(clamp(f, lo, hi));
  }
  const uniq = [...new Set(raw.map((v) => Math.round(v * 1e6) / 1e6))];
  uniq.sort((a, b) => a - b);
  return uniq;
}

export function gridCount(spec: AeroDbSpec, designMach: number, designAlpha: number): { axes: AeroDatabase["axes"]; points: number } {
  const axes = {
    mach: axisBreaks(spec.machMin, spec.machMax, spec.machStep, [designMach]),
    alpha_deg: axisBreaks(spec.alphaMin, spec.alphaMax, spec.alphaStep, [designAlpha, 0]),
    beta_deg: axisBreaks(spec.betaMin, spec.betaMax, spec.betaStep, [0]),
    delta_deg: axisBreaks(spec.deltaMin, spec.deltaMax, spec.deltaStep, [0]),
  };
  return { axes, points: axes.mach.length * axes.alpha_deg.length * axes.beta_deg.length * axes.delta_deg.length };
}

function yieldThread() {
  return new Promise<void>((resolve) => setTimeout(resolve, 0));
}

interface Coeff {
  cl: number;
  cd: number;
  cy: number;
  cm: number;
  cn: number;
  cll: number;
  ca: number;
  ld: number;
}

function coeffOf(
  mesh: TriMesh,
  params: DesignParams,
  altKm: number,
  mach: number,
  alpha: number,
  beta: number,
  sRef: number,
  rates?: RateState,
): Coeff {
  const atm = atmosphere(altKm, mach, params.gamma);
  const flight: DesignParams = { ...params, lockFlight: false, flightMach: mach, aeroMethod: params.aeroMethod };
  const a = panelAero(mesh, flight, atm, alpha, beta, sRef, Math.max(params.length, 1e-6), rates);
  return { cl: a.cl, cd: a.cd, cy: a.cy, cm: a.cm, cn: a.cn, cll: a.cll, ca: a.ca, ld: a.ld };
}

function nearest(axis: number[], value: number): number {
  let best = axis[0];
  let err = Infinity;
  for (const v of axis) {
    const e = Math.abs(v - value);
    if (e < err) {
      err = e;
      best = v;
    }
  }
  return best;
}

export async function generateAeroDatabase(
  built: BuiltVehicle,
  spec: AeroDbSpec,
  opts?: { signal?: AbortSignal; onProgress?: (p: AeroDbProgress) => void },
): Promise<AeroDatabase> {
  const base = built.params;
  const designMach = base.mach;
  const { axes, points: nPoints } = gridCount(spec, designMach, base.alphaDeg);
  if (nPoints > AERODB_POINT_CAP) {
    throw new Error(
      `This grid is ${nPoints} points. The cap is ${AERODB_POINT_CAP} so the browser stays responsive — widen a step or shrink a range.`,
    );
  }
  if (nPoints < 1) throw new Error("Empty aero grid.");
  const method: AeroMethod = spec.method || "cbaero";
  const assumptions = [
    "Same geometry at every Mach. Design Mach shapes the vehicle; other Mach numbers are off-design flight conditions.",
    `Coefficient method: ${method}.`,
    "Control deflection rebuilds the OML (elevon rotation, or fin-ridge shear on missile / stack).",
    "Static derivatives are table differences, per radian. Edges without a two-sided stencil are 0.",
    "Dynamic derivatives: one body-rate perturbation (q̂, p̂, r̂ = 0.02) at α nearest the design angle, β = 0, δ nearest 0, per Mach.",
    spec.eulerSymmetry
      ? "Symmetry-plane Euler is an extra β = 0, δ = 0 column. Off the centerline the Euler kernel is not defined."
      : "Symmetry-plane Euler slice was not requested.",
    "Reference length is the geometric length. Moments use Sref·Lref for all three axes, matching the panel book.",
  ];

  const meshes = new Map<number, { mesh: TriMesh; sRef: number }>();
  const meshFor = (delta: number) => {
    const key = Math.round(delta * 1000);
    const hit = meshes.get(key);
    if (hit) return hit;
    const same = Math.abs(delta - (base.elevonDeg || 0)) < 1e-6;
    if (same) {
      const pack = { mesh: built.mesh, sRef: Math.max(built.aero.planformArea, 1e-6) };
      meshes.set(key, pack);
      return pack;
    }
    const vehicle = buildVehicle({ ...base, elevonDeg: delta });
    const pack = { mesh: vehicle.mesh, sRef: Math.max(vehicle.aero.planformArea, 1e-6) };
    meshes.set(key, pack);
    return pack;
  };

  const points: AeroDbPoint[] = [];
  let done = 0;
  const total = nPoints + (spec.dynamicDerivatives ? axes.mach.length * 3 : 0) + (spec.eulerSymmetry ? axes.mach.length * axes.alpha_deg.length : 0);
  const tick = async (label: string) => {
    if (opts?.signal?.aborted) throw new DOMException("Aero database cancelled", "AbortError");
    opts?.onProgress?.({ done, total, label });
    await yieldThread();
  };

  for (const mach of axes.mach) {
    for (const alpha of axes.alpha_deg) {
      for (const beta of axes.beta_deg) {
        for (const delta of axes.delta_deg) {
          await tick(`M ${mach.toFixed(2)}  α ${alpha.toFixed(1)}  β ${beta.toFixed(1)}  δ ${delta.toFixed(1)}`);
          const pack = meshFor(delta);
          const params = { ...base, aeroMethod: method, elevonDeg: delta };
          const c = coeffOf(pack.mesh, params, spec.altKm, mach, alpha, beta, pack.sRef);
          const onDesign = Math.abs(mach - designMach) <= Math.max(spec.machStep * 0.51, 0.05);
          points.push({
            mach,
            alpha_deg: alpha,
            beta_deg: beta,
            delta_deg: delta,
            onDesign,
            ...c,
            cla: 0,
            cda: 0,
            cma: 0,
            cyb: 0,
            cnb: 0,
            clb: 0,
            cld: 0,
            cmd: 0,
            clq: 0,
            cmq: 0,
            clp: 0,
            cnp: 0,
            cyp: 0,
            cnr: 0,
            clr: 0,
            cyr: 0,
            euler_cl: null,
            euler_cd: null,
            euler_cm: null,
          });
          done++;
        }
      }
    }
  }

  const nM = axes.mach.length;
  const nA = axes.alpha_deg.length;
  const nB = axes.beta_deg.length;
  const nD = axes.delta_deg.length;
  const at = (im: number, ia: number, ib: number, id: number) => points[((im * nA + ia) * nB + ib) * nD + id];

  for (let im = 0; im < nM; im++) {
    for (let ia = 0; ia < nA; ia++) {
      for (let ib = 0; ib < nB; ib++) {
        for (let id = 0; id < nD; id++) {
          const p = at(im, ia, ib, id);
          if (ia > 0 && ia < nA - 1) {
            const span = (axes.alpha_deg[ia + 1] - axes.alpha_deg[ia - 1]) * DEG;
            const hi = at(im, ia + 1, ib, id);
            const lo = at(im, ia - 1, ib, id);
            if (span !== 0) {
              p.cla = (hi.cl - lo.cl) / span;
              p.cda = (hi.cd - lo.cd) / span;
              p.cma = (hi.cm - lo.cm) / span;
            }
          }
          if (ib > 0 && ib < nB - 1) {
            const span = (axes.beta_deg[ib + 1] - axes.beta_deg[ib - 1]) * DEG;
            const hi = at(im, ia, ib + 1, id);
            const lo = at(im, ia, ib - 1, id);
            if (span !== 0) {
              p.cyb = (hi.cy - lo.cy) / span;
              p.cnb = (hi.cn - lo.cn) / span;
              p.clb = (hi.cll - lo.cll) / span;
            }
          }
          if (id > 0 && id < nD - 1) {
            const span = (axes.delta_deg[id + 1] - axes.delta_deg[id - 1]) * DEG;
            const hi = at(im, ia, ib, id + 1);
            const lo = at(im, ia, ib, id - 1);
            if (span !== 0) {
              p.cld = (hi.cl - lo.cl) / span;
              p.cmd = (hi.cm - lo.cm) / span;
            }
          }
        }
      }
    }
  }

  const dynamics: AeroDynamicRow[] = [];
  if (spec.dynamicDerivatives) {
    const alphaA = nearest(axes.alpha_deg, base.alphaDeg);
    const betaA = nearest(axes.beta_deg, 0);
    const deltaA = nearest(axes.delta_deg, 0);
    const ia = axes.alpha_deg.indexOf(alphaA);
    const ib = axes.beta_deg.indexOf(betaA);
    const id = axes.delta_deg.indexOf(deltaA);
    for (let im = 0; im < nM; im++) {
      const mach = axes.mach[im];
      await tick(`dynamic derivatives  M ${mach.toFixed(2)}`);
      const pack = meshFor(deltaA);
      const params = { ...base, aeroMethod: method, elevonDeg: deltaA };
      const atm = atmosphere(spec.altKm, mach, base.gamma);
      const V = Math.max(50, atm.V);
      const L = Math.max(base.length, 1e-6);
      const bRef = Math.max(base.span, 1e-6);
      const qHat = 0.02;
      const pHat = 0.02;
      const rHat = 0.02;
      const anchor = at(im, ia, ib, id);
      const qPair = coeffOf(pack.mesh, params, spec.altKm, mach, alphaA, betaA, pack.sRef, { p: 0, q: (qHat * 2 * V) / L, r: 0 });
      const pPair = coeffOf(pack.mesh, params, spec.altKm, mach, alphaA, betaA, pack.sRef, { p: (pHat * 2 * V) / bRef, q: 0, r: 0 });
      const rPair = coeffOf(pack.mesh, params, spec.altKm, mach, alphaA, betaA, pack.sRef, { p: 0, q: 0, r: (rHat * 2 * V) / bRef });
      done += 3;
      const row: AeroDynamicRow = {
        mach,
        alpha_deg: alphaA,
        delta_deg: deltaA,
        clq: (qPair.cl - anchor.cl) / qHat,
        cmq: (qPair.cm - anchor.cm) / qHat,
        clp: (pPair.cll - anchor.cll) / pHat,
        cnp: (pPair.cn - anchor.cn) / pHat,
        cyp: (pPair.cy - anchor.cy) / pHat,
        cnr: (rPair.cn - anchor.cn) / rHat,
        clr: (rPair.cll - anchor.cll) / rHat,
        cyr: (rPair.cy - anchor.cy) / rHat,
      };
      dynamics.push(row);
      anchor.clq = row.clq;
      anchor.cmq = row.cmq;
      anchor.clp = row.clp;
      anchor.cnp = row.cnp;
      anchor.cyp = row.cyp;
      anchor.cnr = row.cnr;
      anchor.clr = row.clr;
      anchor.cyr = row.cyr;
    }
  }

  if (spec.eulerSymmetry) {
    const deltaA = nearest(axes.delta_deg, 0);
    const betaA = nearest(axes.beta_deg, 0);
    const ib = axes.beta_deg.indexOf(betaA);
    const id = axes.delta_deg.indexOf(deltaA);
    const pack = meshFor(deltaA);
    for (let im = 0; im < nM; im++) {
      for (let ia = 0; ia < nA; ia++) {
        const mach = axes.mach[im];
        const alpha = axes.alpha_deg[ia];
        await tick(`Euler symmetry  M ${mach.toFixed(2)}  α ${alpha.toFixed(1)}`);
        const c = coeffOf(pack.mesh, { ...base, aeroMethod: "euler", elevonDeg: deltaA }, spec.altKm, mach, alpha, betaA, pack.sRef);
        const p = at(im, ia, ib, id);
        p.euler_cl = c.cl;
        p.euler_cd = c.cd;
        p.euler_cm = c.cm;
        done++;
      }
    }
  }

  const anchorDelta = nearest(axes.delta_deg, 0);
  const anchorBeta = nearest(axes.beta_deg, 0);
  const sliceAt = (mach: number): AeroDbSlice => ({
    mach,
    alpha: axes.alpha_deg.map((alpha) => {
      const p = points.find((q) => q.mach === mach && q.alpha_deg === alpha && q.beta_deg === anchorBeta && q.delta_deg === anchorDelta);
      return { alpha, cl: p?.cl ?? 0, cd: p?.cd ?? 0, cm: p?.cm ?? 0, ld: p?.ld ?? 0 };
    }),
  });
  let designM = axes.mach[0];
  let designErr = Infinity;
  let offM = axes.mach[0];
  let offErr = -1;
  for (const m of axes.mach) {
    const e = Math.abs(m - designMach);
    if (e < designErr) {
      designErr = e;
      designM = m;
    }
    if (e > offErr) {
      offErr = e;
      offM = m;
    }
  }

  let maxLd = -Infinity;
  let maxLdAt = { mach: designM, alpha_deg: 0, beta_deg: 0, delta_deg: 0 };
  let clMin = Infinity;
  let clMax = -Infinity;
  let cdMin = Infinity;
  let cdMax = -Infinity;
  let cmAtDesign = 0;
  let onN = 0;
  for (const p of points) {
    if (p.ld > maxLd) {
      maxLd = p.ld;
      maxLdAt = { mach: p.mach, alpha_deg: p.alpha_deg, beta_deg: p.beta_deg, delta_deg: p.delta_deg };
    }
    clMin = Math.min(clMin, p.cl);
    clMax = Math.max(clMax, p.cl);
    cdMin = Math.min(cdMin, p.cd);
    cdMax = Math.max(cdMax, p.cd);
    if (p.onDesign && Math.abs(p.alpha_deg - base.alphaDeg) < 1e-6 && p.beta_deg === anchorBeta && p.delta_deg === anchorDelta) {
      cmAtDesign = p.cm;
    }
    if (p.onDesign) onN++;
  }
  const cgx = clamp(base.cgFrac, 0.05, 0.95) * base.length;
  const sRef0 = Math.max(built.aero.planformArea, 1e-6);
  const db: AeroDatabase = {
    meta: {
      format: "cuspis-aerodb-1",
      name: base.name || "vehicle",
      family: base.family,
      method,
      designMach,
      altitude_km: spec.altKm,
      sRef_m2: sRef0,
      lRef_m: base.length,
      bRef_m: base.span,
      cg_m: [cgx, 0, 0],
      units: {
        angles: "deg",
        derivatives: "1/rad",
        force: "wind axes, CL CD CY",
        moment: "body axes, Cm Cn Cll, nondim by q·Sref·Lref",
      },
      frame: "X streamwise (tip at origin, flow +X), Y span, Z up",
      index: "flat i = ((im*nAlpha + ia)*nBeta + ib)*nDelta + id  (delta fastest)",
      generatedAt: new Date().toISOString(),
      assumptions,
    },
    axes,
    points,
    dynamics,
    summary: {
      points: points.length,
      onDesign: onN,
      offDesign: points.length - onN,
      maxLd: Number.isFinite(maxLd) ? maxLd : 0,
      maxLdAt,
      clMin: Number.isFinite(clMin) ? clMin : 0,
      clMax: Number.isFinite(clMax) ? clMax : 0,
      cdMin: Number.isFinite(cdMin) ? cdMin : 0,
      cdMax: Number.isFinite(cdMax) ? cdMax : 0,
      cmAtDesign,
    },
    slices: { design: sliceAt(designM), offDesign: sliceAt(offM) },
  };
  opts?.onProgress?.({ done: total, total, label: "done" });
  return db;
}

const CSV_COLS = [
  "mach",
  "alpha_deg",
  "beta_deg",
  "delta_deg",
  "on_design",
  "CL",
  "CD",
  "CY",
  "Cm",
  "Cn",
  "Cll",
  "CA",
  "LD",
  "CLa_per_rad",
  "CDa_per_rad",
  "Cma_per_rad",
  "CYb_per_rad",
  "Cnb_per_rad",
  "Clb_per_rad",
  "CLd_per_rad",
  "Cmd_per_rad",
  "CLq",
  "Cmq",
  "Clp",
  "Cnp",
  "Cyp",
  "Cnr",
  "Clr",
  "Cyr",
  "euler_CL",
  "euler_CD",
  "euler_Cm",
] as const;

function num(v: number | null): string {
  if (v === null || v === undefined || !Number.isFinite(v)) return "";
  return Math.abs(v) >= 1e4 || (Math.abs(v) > 0 && Math.abs(v) < 1e-4) ? v.toExponential(6) : v.toFixed(6);
}

export function aeroDbToCsv(db: AeroDatabase): string {
  const m = db.meta;
  const lines = [
    `# cuspis aerodynamic database ${m.format}`,
    `# name: ${m.name}`,
    `# family: ${m.family}`,
    `# method: ${m.method}`,
    `# design_mach: ${m.designMach}`,
    `# altitude_km: ${m.altitude_km}`,
    `# sref_m2: ${m.sRef_m2}`,
    `# lref_m: ${m.lRef_m}`,
    `# bref_m: ${m.bRef_m}`,
    `# cg_x_m: ${m.cg_m[0]}`,
    `# cg_y_m: ${m.cg_m[1]}`,
    `# cg_z_m: ${m.cg_m[2]}`,
    `# angles: deg    derivatives: per radian`,
    `# frame: ${m.frame}`,
    `# on_design marks Mach within half a Mach step of the inverse-design Mach`,
    `# ${m.assumptions.join(" | ")}`,
    CSV_COLS.join(","),
  ];
  for (const p of db.points) {
    lines.push(
      [
        num(p.mach),
        num(p.alpha_deg),
        num(p.beta_deg),
        num(p.delta_deg),
        p.onDesign ? "1" : "0",
        num(p.cl),
        num(p.cd),
        num(p.cy),
        num(p.cm),
        num(p.cn),
        num(p.cll),
        num(p.ca),
        num(p.ld),
        num(p.cla),
        num(p.cda),
        num(p.cma),
        num(p.cyb),
        num(p.cnb),
        num(p.clb),
        num(p.cld),
        num(p.cmd),
        num(p.clq),
        num(p.cmq),
        num(p.clp),
        num(p.cnp),
        num(p.cyp),
        num(p.cnr),
        num(p.clr),
        num(p.cyr),
        num(p.euler_cl),
        num(p.euler_cd),
        num(p.euler_cm),
      ].join(","),
    );
  }
  return lines.join("\n") + "\n";
}

export function aeroDbToJson(db: AeroDatabase): string {
  return JSON.stringify(db, null, 2) + "\n";
}

/** MATLAB script: n-D lookup tables for Simulink Aerospace Blockset or an n-D Lookup Table block. */
export function aeroDbToSimulink(db: AeroDatabase): string {
  const { axes, points, meta, dynamics } = db;
  const nM = axes.mach.length;
  const nA = axes.alpha_deg.length;
  const nB = axes.beta_deg.length;
  const nD = axes.delta_deg.length;
  const flat = (pick: (p: AeroDbPoint) => number) => points.map((p) => pick(p).toPrecision(8)).join(", ");
  const vec = (xs: number[]) => xs.map((v) => v.toPrecision(8)).join(", ");
  const dyn = (key: keyof AeroDynamicRow) => dynamics.map((d) => Number(d[key]).toPrecision(8)).join(", ");
  return `% cuspis_aero_db.m
% ${meta.name}  family ${meta.family}  method ${meta.method}
% Generated ${meta.generatedAt}
%
% Angles are degrees. Derivatives are per radian.
% Sref = ${meta.sRef_m2} m^2    Lref = ${meta.lRef_m} m    Bref = ${meta.bRef_m} m
% CG (m, body) = [${meta.cg_m.join(", ")}]
% Design Mach = ${meta.designMach}    altitude = ${meta.altitude_km} km
% Frame: ${meta.frame}
%
% MATLAB reshape is column-major and delta is the fastest index, so
%   reshape(CL, nDelta, nBeta, nAlpha, nMach)
% matches the file. Wire a Simulink n-D Lookup Table as
%   breakpoints {delta_deg, beta_deg, alpha_deg, mach}
% or permute to the Aerospace Blockset order you use.
%
% ${meta.assumptions.map((s) => "% " + s).join("\n")}
function aero = cuspis_aero_db()
aero.format = '${meta.format}';
aero.Sref = ${meta.sRef_m2};
aero.Lref = ${meta.lRef_m};
aero.Bref = ${meta.bRef_m};
aero.cg = [${meta.cg_m.join(", ")}];
aero.designMach = ${meta.designMach};
aero.alt_km = ${meta.altitude_km};
aero.method = '${meta.method}';
aero.mach = [${vec(axes.mach)}];
aero.alpha_deg = [${vec(axes.alpha_deg)}];
aero.beta_deg = [${vec(axes.beta_deg)}];
aero.delta_deg = [${vec(axes.delta_deg)}];
nM = ${nM}; nA = ${nA}; nB = ${nB}; nD = ${nD};
aero.CL  = reshape([${flat((p) => p.cl)}],  nD, nB, nA, nM);
aero.CD  = reshape([${flat((p) => p.cd)}],  nD, nB, nA, nM);
aero.CY  = reshape([${flat((p) => p.cy)}],  nD, nB, nA, nM);
aero.Cm  = reshape([${flat((p) => p.cm)}],  nD, nB, nA, nM);
aero.Cn  = reshape([${flat((p) => p.cn)}],  nD, nB, nA, nM);
aero.Cll = reshape([${flat((p) => p.cll)}], nD, nB, nA, nM);
aero.CLa = reshape([${flat((p) => p.cla)}], nD, nB, nA, nM);
aero.Cma = reshape([${flat((p) => p.cma)}], nD, nB, nA, nM);
aero.Cmq = reshape([${flat((p) => p.cmq)}], nD, nB, nA, nM);
aero.CLq = reshape([${flat((p) => p.clq)}], nD, nB, nA, nM);
% Dynamic derivatives vs Mach at alpha = design anchor, beta = 0, delta ~ 0.
aero.dyn.mach = [${dyn("mach")}];
aero.dyn.Cmq = [${dyn("cmq")}];
aero.dyn.CLq = [${dyn("clq")}];
aero.dyn.Clp = [${dyn("clp")}];
aero.dyn.Cnr = [${dyn("cnr")}];
aero.dyn.Cnp = [${dyn("cnp")}];
aero.dyn.Clr = [${dyn("clr")}];
end
`;
}
