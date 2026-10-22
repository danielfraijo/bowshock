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
 *
 * Independent control surfaces (controls.ts) add optional axes after δ: one per
 * entry of `spec.controlAxes` (channels de / da / dr / dbf / fsN_*, or a raw
 * surface id). The legacy δ axis is still main's full-span `elevonDeg`. Control
 * derivatives are PER DEGREE (the static ones stay per radian): a table central
 * difference on every swept control axis, plus an anchor block (α ≈ design,
 * β = 0, δ ≈ 0, other controls ≈ 0) per Mach and per channel by a ±h central
 * difference, h = `controlStepDeg`. Each deflection is a fresh `buildVehicle`, so
 * the panel geometry cache never sees a mutated mesh.
 */
import { atmosphere } from "./atmosphere";
import { buildVehicle } from "./generate";
import { DEG, clamp } from "./math";
import { panelAero, type RateState } from "./panel";
import { CONTROL_CHANNELS, controlChannels, listControlSurfaces, withControls, type ControlSurface } from "./controls";
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
  /** Extra control-deflection axes (deg), swept after δ. Channel = de | da | dr | dbf | fsN_de… | a surface id. */
  controlAxes?: AeroDbControlAxis[];
  /** Anchor control derivatives per Mach and channel (central difference). Default true. */
  controlDerivatives?: boolean;
  /** Half-step of the anchor central difference, deg. Default 2. */
  controlStepDeg?: number;
}

export interface AeroDbControlAxis {
  channel: string;
  min: number;
  max: number;
  step: number;
}

/** Control derivatives, per degree. Roll is main's Cll (about +X aft). */
export interface AeroControlDeriv {
  cl: number;
  cd: number;
  cy: number;
  cm: number;
  cn: number;
  cll: number;
}

export interface AeroControlRow extends AeroControlDeriv {
  mach: number;
  alpha_deg: number;
  beta_deg: number;
  channel: string;
  step_deg: number;
  /** Surfaces this channel moved (after mixing), for the record. */
  surfaces: string[];
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
    controlDerivatives: "1/deg";
  };
  frame: "X streamwise (tip at origin, flow +X), Y span, Z up";
  /** Flat index. Without control axes: ((im*nAlpha + ia)*nBeta + ib)*nDelta + id (delta fastest). */
  index: string;
  /** Sign conventions for the control channels and moments. */
  controlConvention: string[];
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
  /** Control-axis values at this point, deg, keyed by channel. Empty without control axes. */
  controls: Record<string, number>;
  /** Table central difference on each swept control axis, per degree. Absent at axis edges. */
  controlDerivs: Record<string, AeroControlDeriv>;
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
  axes: {
    mach: number[];
    alpha_deg: number[];
    beta_deg: number[];
    delta_deg: number[];
    /** Extra control axes in sweep order (last is fastest). */
    controls: { channel: string; deg: number[] }[];
  };
  points: AeroDbPoint[];
  dynamics: AeroDynamicRow[];
  /** Anchor control derivatives, per degree, one row per Mach × channel. */
  controlDerivatives: AeroControlRow[];
  /** The vehicle's control surfaces (controls.ts `listControlSurfaces`). */
  controlSurfaces: ControlSurface[];
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
    controlAxes: [],
    controlDerivatives: true,
    controlStepDeg: 2,
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
    controls: (spec.controlAxes ?? []).map((c) => ({ channel: c.channel, deg: axisBreaks(c.min, c.max, c.step, [0]) })),
  };
  let n = axes.mach.length * axes.alpha_deg.length * axes.beta_deg.length * axes.delta_deg.length;
  for (const c of axes.controls) n *= c.deg.length;
  return { axes, points: n };
}

/** Sign conventions written into every export. */
export const CONTROL_CONVENTION: string[] = [
  "delta_deg: main's full-span elevon (elevonDeg), TE down positive.",
  `de: ${CONTROL_CHANNELS.de.convention}.`,
  `da: ${CONTROL_CHANNELS.da.convention}. elevon_r = de + da, elevon_l = de − da.`,
  `dr: ${CONTROL_CHANNELS.dr.convention}.`,
  `dbf: ${CONTROL_CHANNELS.dbf.convention}.`,
  "Fins (set 0): fin_k = de·cos θk − dr·sin θk + da, θk from +Y toward +Z; fsN_de / fsN_da / fsN_dr drive set N. Positive fin = right-hand about its outward radial axis.",
  "Moments keep main's body axes: Cm about +Y (nose up +), Cn about +Z (nose left +), Cll about +X aft (right wing up +). β > 0 is flow toward +Y.",
  "Control derivatives are per degree; static and dynamic derivatives stay per radian.",
];

/**
 * A deflected copy of a built vehicle. `input` holds channel values (de, da, dr,
 * dbf, fsN_*) and/or raw surface ids, in degrees; they add to any deflections
 * already in `built.params.controls`. Always a fresh buildVehicle (new arrays,
 * the input mesh is never touched). Returns `built` itself when nothing moves.
 */
export function deflectVehicle(built: BuiltVehicle, input: Partial<Record<string, number>>): BuiltVehicle {
  const surfaces = listControlSurfaces(built.params);
  const ids = new Set([...surfaces.map((s) => s.id), ...controlChannels(built.params)]);
  for (const k of Object.keys(input)) {
    if (!ids.has(k)) throw new Error(`Unknown control "${k}" for ${built.params.family}. Known: ${[...ids].join(", ") || "none"}.`);
  }
  const anyInput = Object.values(input).some((v) => typeof v === "number" && Math.abs(v) > 1e-12);
  if (!anyInput) return built;
  return buildVehicle(withControls(built.params, input, surfaces));
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
  const surfaces = listControlSurfaces(built);
  const channels = controlChannels(base);
  const known = new Set([...channels, ...surfaces.map((s) => s.id)]);
  const seen = new Set<string>();
  for (const c of axes.controls) {
    if (!known.has(c.channel)) {
      throw new Error(`Control axis "${c.channel}" does not exist on ${base.family}. Available: ${[...known].join(", ") || "none"}.`);
    }
    if (seen.has(c.channel)) throw new Error(`Control axis "${c.channel}" is listed twice.`);
    seen.add(c.channel);
  }
  const ctrlStep = clamp(Math.abs(spec.controlStepDeg ?? 2) || 2, 0.05, 10);
  const wantCtrlDeriv = (spec.controlDerivatives ?? true) && channels.length > 0;
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
  if (surfaces.length) {
    assumptions.push(
      `Control surfaces: ${surfaces.map((s) => s.id).join(", ")}. Each deflection is a rebuilt OML (part-span shear before the LE fillet on waveriders and boost-glide, hinge swing of fin ridges on bodies of revolution).`,
      `Control derivatives are per degree: table central differences on swept control axes${wantCtrlDeriv ? `, plus a ±${ctrlStep}° central difference per channel at the anchor (α ≈ design, β = 0, δ ≈ 0) for every Mach` : ""}. Deflections beyond a surface's limits are clamped.`,
    );
  }

  const nM = axes.mach.length;
  const nA = axes.alpha_deg.length;
  const nB = axes.beta_deg.length;
  const nD = axes.delta_deg.length;
  // Control combinations, last axis fastest. Without control axes there is one empty combination.
  const ctrlAxes = axes.controls;
  const strides: number[] = new Array(ctrlAxes.length).fill(1);
  for (let k = ctrlAxes.length - 2; k >= 0; k--) strides[k] = strides[k + 1] * ctrlAxes[k + 1].deg.length;
  const nC = ctrlAxes.reduce((n, c) => n * c.deg.length, 1);
  const combos: Record<string, number>[] = [];
  for (let ic = 0; ic < nC; ic++) {
    const c: Record<string, number> = {};
    for (let k = 0; k < ctrlAxes.length; k++) c[ctrlAxes[k].channel] = ctrlAxes[k].deg[Math.floor(ic / strides[k]) % ctrlAxes[k].deg.length];
    combos.push(c);
  }
  const comboKey = (c: Record<string, number>) =>
    Object.keys(c)
      .sort()
      .filter((k) => Math.abs(c[k]) > 1e-9)
      .map((k) => `${k}=${Math.round(c[k] * 1000)}`)
      .join(",");

  const meshes = new Map<string, { mesh: TriMesh; sRef: number }>();
  const meshFor = (delta: number, ctrl: Record<string, number> = {}) => {
    const ck = comboKey(ctrl);
    const key = `${Math.round(delta * 1000)}|${ck}`;
    const hit = meshes.get(key);
    if (hit) return hit;
    const same = Math.abs(delta - (base.elevonDeg || 0)) < 1e-6 && ck === "";
    if (same) {
      const pack = { mesh: built.mesh, sRef: Math.max(built.aero.planformArea, 1e-6) };
      meshes.set(key, pack);
      return pack;
    }
    const p0 = { ...base, elevonDeg: delta };
    const vehicle = buildVehicle(ck === "" ? p0 : withControls(p0, ctrl, surfaces));
    const pack = { mesh: vehicle.mesh, sRef: Math.max(vehicle.aero.planformArea, 1e-6) };
    meshes.set(key, pack);
    return pack;
  };

  const points: AeroDbPoint[] = [];
  let done = 0;
  const total =
    nPoints +
    (spec.dynamicDerivatives ? nM * 3 : 0) +
    (spec.eulerSymmetry ? nM * nA : 0) +
    (wantCtrlDeriv ? nM * channels.length * 2 : 0);
  const tick = async (label: string) => {
    if (opts?.signal?.aborted) throw new DOMException("Aero database cancelled", "AbortError");
    opts?.onProgress?.({ done, total, label });
    await yieldThread();
  };
  const ctrlLabel = (c: Record<string, number>) =>
    Object.entries(c)
      .map(([k, v]) => `  ${k} ${v.toFixed(1)}`)
      .join("");

  for (const mach of axes.mach) {
    for (const alpha of axes.alpha_deg) {
      for (const beta of axes.beta_deg) {
        for (const delta of axes.delta_deg) {
          for (const ctrl of combos) {
            await tick(`M ${mach.toFixed(2)}  α ${alpha.toFixed(1)}  β ${beta.toFixed(1)}  δ ${delta.toFixed(1)}${ctrlLabel(ctrl)}`);
            const pack = meshFor(delta, ctrl);
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
              controls: { ...ctrl },
              controlDerivs: {},
            });
            done++;
          }
        }
      }
    }
  }

  const at = (im: number, ia: number, ib: number, id: number, ic = 0) => points[(((im * nA + ia) * nB + ib) * nD + id) * nC + ic];
  const nearestIdx = (axis: number[], value: number) => axis.indexOf(nearest(axis, value));
  const icAnchor = ctrlAxes.reduce((ic, c, k) => ic + nearestIdx(c.deg, 0) * strides[k], 0);

  for (let im = 0; im < nM; im++) {
    for (let ia = 0; ia < nA; ia++) {
      for (let ib = 0; ib < nB; ib++) {
        for (let id = 0; id < nD; id++) {
          for (let ic = 0; ic < nC; ic++) {
            const p = at(im, ia, ib, id, ic);
            if (ia > 0 && ia < nA - 1) {
              const span = (axes.alpha_deg[ia + 1] - axes.alpha_deg[ia - 1]) * DEG;
              const hi = at(im, ia + 1, ib, id, ic);
              const lo = at(im, ia - 1, ib, id, ic);
              if (span !== 0) {
                p.cla = (hi.cl - lo.cl) / span;
                p.cda = (hi.cd - lo.cd) / span;
                p.cma = (hi.cm - lo.cm) / span;
              }
            }
            if (ib > 0 && ib < nB - 1) {
              const span = (axes.beta_deg[ib + 1] - axes.beta_deg[ib - 1]) * DEG;
              const hi = at(im, ia, ib + 1, id, ic);
              const lo = at(im, ia, ib - 1, id, ic);
              if (span !== 0) {
                p.cyb = (hi.cy - lo.cy) / span;
                p.cnb = (hi.cn - lo.cn) / span;
                p.clb = (hi.cll - lo.cll) / span;
              }
            }
            if (id > 0 && id < nD - 1) {
              const span = (axes.delta_deg[id + 1] - axes.delta_deg[id - 1]) * DEG;
              const hi = at(im, ia, ib, id + 1, ic);
              const lo = at(im, ia, ib, id - 1, ic);
              if (span !== 0) {
                p.cld = (hi.cl - lo.cl) / span;
                p.cmd = (hi.cm - lo.cm) / span;
              }
            }
            for (let k = 0; k < ctrlAxes.length; k++) {
              const deg = ctrlAxes[k].deg;
              const j = Math.floor(ic / strides[k]) % deg.length;
              if (j <= 0 || j >= deg.length - 1) continue;
              const span = deg[j + 1] - deg[j - 1];
              if (span === 0) continue;
              const hi = at(im, ia, ib, id, ic + strides[k]);
              const lo = at(im, ia, ib, id, ic - strides[k]);
              p.controlDerivs[ctrlAxes[k].channel] = {
                cl: (hi.cl - lo.cl) / span,
                cd: (hi.cd - lo.cd) / span,
                cy: (hi.cy - lo.cy) / span,
                cm: (hi.cm - lo.cm) / span,
                cn: (hi.cn - lo.cn) / span,
                cll: (hi.cll - lo.cll) / span,
              };
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
    const ctrlA = combos[icAnchor];
    for (let im = 0; im < nM; im++) {
      const mach = axes.mach[im];
      await tick(`dynamic derivatives  M ${mach.toFixed(2)}`);
      const pack = meshFor(deltaA, ctrlA);
      const params = { ...base, aeroMethod: method, elevonDeg: deltaA };
      const atm = atmosphere(spec.altKm, mach, base.gamma);
      const V = Math.max(50, atm.V);
      const L = Math.max(base.length, 1e-6);
      const bRef = Math.max(base.span, 1e-6);
      const qHat = 0.02;
      const pHat = 0.02;
      const rHat = 0.02;
      const anchor = at(im, ia, ib, id, icAnchor);
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

  const controlDerivatives: AeroControlRow[] = [];
  if (wantCtrlDeriv) {
    const alphaA = nearest(axes.alpha_deg, base.alphaDeg);
    const betaA = nearest(axes.beta_deg, 0);
    const deltaA = nearest(axes.delta_deg, 0);
    const ctrlA = combos[icAnchor];
    const params = { ...base, aeroMethod: method, elevonDeg: deltaA };
    for (let im = 0; im < nM; im++) {
      const mach = axes.mach[im];
      for (const ch of channels) {
        await tick(`control derivatives  M ${mach.toFixed(2)}  ${ch}`);
        const c0 = ctrlA[ch] ?? 0;
        const hi = meshFor(deltaA, { ...ctrlA, [ch]: c0 + ctrlStep });
        const lo = meshFor(deltaA, { ...ctrlA, [ch]: c0 - ctrlStep });
        const a = coeffOf(hi.mesh, params, spec.altKm, mach, alphaA, betaA, hi.sRef);
        const b = coeffOf(lo.mesh, params, spec.altKm, mach, alphaA, betaA, lo.sRef);
        done += 2;
        const span = 2 * ctrlStep;
        controlDerivatives.push({
          mach,
          alpha_deg: alphaA,
          beta_deg: betaA,
          channel: ch,
          step_deg: ctrlStep,
          surfaces: surfaces.filter((s) => (s.mix[ch] ?? 0) !== 0).map((s) => s.id),
          cl: (a.cl - b.cl) / span,
          cd: (a.cd - b.cd) / span,
          cy: (a.cy - b.cy) / span,
          cm: (a.cm - b.cm) / span,
          cn: (a.cn - b.cn) / span,
          cll: (a.cll - b.cll) / span,
        });
      }
    }
  }

  if (spec.eulerSymmetry) {
    const deltaA = nearest(axes.delta_deg, 0);
    const betaA = nearest(axes.beta_deg, 0);
    const ib = axes.beta_deg.indexOf(betaA);
    const id = axes.delta_deg.indexOf(deltaA);
    const pack = meshFor(deltaA, combos[icAnchor]);
    for (let im = 0; im < nM; im++) {
      for (let ia = 0; ia < nA; ia++) {
        const mach = axes.mach[im];
        const alpha = axes.alpha_deg[ia];
        await tick(`Euler symmetry  M ${mach.toFixed(2)}  α ${alpha.toFixed(1)}`);
        const c = coeffOf(pack.mesh, { ...base, aeroMethod: "euler", elevonDeg: deltaA }, spec.altKm, mach, alpha, betaA, pack.sRef);
        const p = at(im, ia, ib, id, icAnchor);
        p.euler_cl = c.cl;
        p.euler_cd = c.cd;
        p.euler_cm = c.cm;
        done++;
      }
    }
  }

  const anchorDelta = nearest(axes.delta_deg, 0);
  const anchorBeta = nearest(axes.beta_deg, 0);
  const anchorCtrl = combos[icAnchor];
  const isAnchorCtrl = (p: AeroDbPoint) => ctrlAxes.every((c) => p.controls[c.channel] === anchorCtrl[c.channel]);
  const sliceAt = (mach: number): AeroDbSlice => ({
    mach,
    alpha: axes.alpha_deg.map((alpha) => {
      const p = points.find((q) => q.mach === mach && q.alpha_deg === alpha && q.beta_deg === anchorBeta && q.delta_deg === anchorDelta && isAnchorCtrl(q));
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
    if (p.onDesign && Math.abs(p.alpha_deg - base.alphaDeg) < 1e-6 && p.beta_deg === anchorBeta && p.delta_deg === anchorDelta && isAnchorCtrl(p)) {
      cmAtDesign = p.cm;
    }
    if (p.onDesign) onN++;
  }
  const cgx = clamp(base.cgFrac, 0.05, 0.95) * base.length;
  const sRef0 = Math.max(built.aero.planformArea, 1e-6);
  const index = ctrlAxes.length
    ? `flat i = (((im*nAlpha + ia)*nBeta + ib)*nDelta + id)*nCtrl + ic, ic = ${ctrlAxes
        .map((c, k) => `i_${c.channel}*${strides[k]}`)
        .join(" + ")}  (last control axis fastest)`
    : "flat i = ((im*nAlpha + ia)*nBeta + ib)*nDelta + id  (delta fastest)";
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
        controlDerivatives: "1/deg",
      },
      frame: "X streamwise (tip at origin, flow +X), Y span, Z up",
      index,
      controlConvention: CONTROL_CONVENTION,
      generatedAt: new Date().toISOString(),
      assumptions,
    },
    axes,
    points,
    dynamics,
    controlDerivatives,
    controlSurfaces: surfaces,
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

const CTRL_COEFFS = [
  ["CL", "cl"],
  ["CD", "cd"],
  ["CY", "cy"],
  ["Cm", "cm"],
  ["Cn", "cn"],
  ["Cll", "cll"],
] as const;

/** Anchor control derivatives as their own CSV (per degree). */
export function aeroDbControlDerivativesToCsv(db: AeroDatabase): string {
  const lines = [
    `# cuspis control derivatives ${db.meta.format}  per degree  anchor alpha ≈ design, beta = 0, delta ≈ 0`,
    ...db.meta.controlConvention.map((c) => `# ${c}`),
    ["mach", "alpha_deg", "beta_deg", "channel", "step_deg", ...CTRL_COEFFS.map(([n]) => `${n}_per_deg`), "surfaces"].join(","),
  ];
  for (const r of db.controlDerivatives) {
    lines.push(
      [num(r.mach), num(r.alpha_deg), num(r.beta_deg), r.channel, num(r.step_deg), ...CTRL_COEFFS.map(([, k]) => num(r[k])), r.surfaces.join(" ")].join(","),
    );
  }
  return lines.join("\n") + "\n";
}

export function aeroDbToCsv(db: AeroDatabase): string {
  const m = db.meta;
  const ctrl = db.axes.controls ?? [];
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
    ...(ctrl.length ? [`# control_axes: ${ctrl.map((c) => `${c.channel}_deg`).join(", ")} (columns after euler_Cm; control derivatives per degree)`] : []),
    ...(m.controlConvention ?? []).map((c) => `# control: ${c}`),
    ...(db.controlDerivatives?.length
      ? [
          `# ctrl_deriv,mach,alpha_deg,beta_deg,channel,step_deg,${CTRL_COEFFS.map(([n]) => `${n}_per_deg`).join(",")}`,
          ...db.controlDerivatives.map(
            (r) => `# ctrl_deriv,${num(r.mach)},${num(r.alpha_deg)},${num(r.beta_deg)},${r.channel},${num(r.step_deg)},${CTRL_COEFFS.map(([, k]) => num(r[k])).join(",")}`,
          ),
        ]
      : []),
    [
      ...CSV_COLS,
      ...ctrl.map((c) => `${c.channel}_deg`),
      ...ctrl.flatMap((c) => CTRL_COEFFS.map(([n]) => `${n}_${c.channel}_per_deg`)),
    ].join(","),
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
        ...ctrl.map((c) => num(p.controls?.[c.channel] ?? 0)),
        ...ctrl.flatMap((c) => CTRL_COEFFS.map(([, k]) => num(p.controlDerivs?.[c.channel]?.[k] ?? 0))),
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
  const ctrl = axes.controls ?? [];
  const flat = (pick: (p: AeroDbPoint) => number) => points.map((p) => pick(p).toPrecision(8)).join(", ");
  const vec = (xs: number[]) => xs.map((v) => v.toPrecision(8)).join(", ");
  const dyn = (key: keyof AeroDynamicRow) => dynamics.map((d) => Number(d[key]).toPrecision(8)).join(", ");
  const ident = (s: string) => s.replace(/[^A-Za-z0-9_]/g, "_");
  // MATLAB is column-major: the fastest index (last control axis, else delta) comes first.
  const dimNames = [...ctrl.map((c) => `n_${ident(c.channel)}`).reverse(), "nD", "nB", "nA", "nM"];
  const dims = dimNames.join(", ");
  const bpOrder = [...ctrl.map((c) => `${c.channel}_deg`).reverse(), "delta_deg", "beta_deg", "alpha_deg", "mach"].join(", ");
  const ctrlDecl = ctrl.map((c) => `aero.ctrl_axes.${ident(c.channel)}_deg = [${vec(c.deg)}];\nn_${ident(c.channel)} = ${c.deg.length};`).join("\n");
  const ctrlTables = ctrl
    .flatMap((c) =>
      CTRL_COEFFS.map(
        ([n, k]) => `aero.${n}_${ident(c.channel)} = reshape([${flat((p) => p.controlDerivs?.[c.channel]?.[k] ?? 0)}], ${dims}); % per deg`,
      ),
    )
    .join("\n");
  const rows = db.controlDerivatives ?? [];
  const chans = [...new Set(rows.map((r) => r.channel))];
  const ctrlAnchor = chans.length
    ? [
        `% Anchor control derivatives (per deg) vs Mach: alpha ~ design, beta = 0, delta ~ 0, central difference.`,
        `aero.ctrl.mach = [${vec(axes.mach)}];`,
        ...chans.flatMap((ch) =>
          CTRL_COEFFS.map(([n, k]) => {
            const vals = axes.mach.map((m) => rows.find((r) => r.channel === ch && r.mach === m)?.[k] ?? 0);
            return `aero.ctrl.${ident(ch)}.${n} = [${vec(vals)}];`;
          }),
        ),
      ].join("\n")
    : "% No control surfaces on this vehicle.";
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
% MATLAB reshape is column-major and the file's fastest index comes first, so
%   reshape(CL, ${dims})
% matches the file. Wire a Simulink n-D Lookup Table as
%   breakpoints {${bpOrder}}
% or permute to the Aerospace Blockset order you use.
%
% Control sign conventions:
${meta.controlConvention.map((c) => "%   " + c).join("\n")}
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
${ctrlDecl}
aero.CL  = reshape([${flat((p) => p.cl)}],  ${dims});
aero.CD  = reshape([${flat((p) => p.cd)}],  ${dims});
aero.CY  = reshape([${flat((p) => p.cy)}],  ${dims});
aero.Cm  = reshape([${flat((p) => p.cm)}],  ${dims});
aero.Cn  = reshape([${flat((p) => p.cn)}],  ${dims});
aero.Cll = reshape([${flat((p) => p.cll)}], ${dims});
aero.CLa = reshape([${flat((p) => p.cla)}], ${dims});
aero.Cma = reshape([${flat((p) => p.cma)}], ${dims});
aero.Cmq = reshape([${flat((p) => p.cmq)}], ${dims});
aero.CLq = reshape([${flat((p) => p.clq)}], ${dims});
${ctrlTables}
${ctrlAnchor}
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
