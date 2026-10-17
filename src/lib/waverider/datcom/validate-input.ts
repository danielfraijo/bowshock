/**
 * Input sanitization / validation for the DATCOM estimator.
 * Degenerate geometry and flight inputs are clamped to safe minima with warnings
 * rather than producing NaN. Truly empty schedules throw ValidationError.
 */
import type { BodySpec, FinSet, FlightSpec, VehicleConfig } from "./types.ts";
import { clamp } from "./util.ts";

export class ValidationError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "ValidationError";
  }
}

const EPS_L = 1e-6;
const EPS_A = 1e-12;

function fixBody(b: BodySpec | undefined, warnings: string[]): BodySpec | undefined {
  if (!b) return undefined;
  if (b.parametric) {
    const p = { ...b.parametric };
    if (!(p.noseDiameter > 0)) {
      warnings.push(`noseDiameter ${p.noseDiameter} → ${EPS_L}`);
      p.noseDiameter = EPS_L;
    }
    if (!(p.noseLength > 0)) {
      warnings.push(`noseLength ${p.noseLength} → ${EPS_L}`);
      p.noseLength = EPS_L;
    }
    if (!(p.centerLength >= 0)) p.centerLength = 0;
    if (!(p.aftLength >= 0)) p.aftLength = 0;
    if (p.centerDiameter !== undefined && !(p.centerDiameter > 0)) p.centerDiameter = p.noseDiameter;
    if (p.aftDiameter !== undefined && !(p.aftDiameter > 0)) p.aftDiameter = p.noseDiameter;
    if (p.noseBluntRadius !== undefined && p.noseBluntRadius < 0) p.noseBluntRadius = 0;
    if (p.power !== undefined) p.power = clamp(p.power, 0.05, 5);
    return { ...b, parametric: p };
  }
  if (b.stations) {
    const st = b.stations
      .filter((s) => Number.isFinite(s.x) && Number.isFinite(s.r) && s.r >= 0)
      .map((s) => ({ x: s.x, r: Math.max(s.r, 0) }));
    if (st.length < 2) {
      warnings.push("body stations < 2 — replaced with a unit cylinder stub");
      return { parametric: { noseKind: "cone", noseLength: 0.01, noseDiameter: 0.1, centerLength: 1, aftLength: 0 } };
    }
    // ensure strictly nondecreasing x
    for (let i = 1; i < st.length; i++) if (st[i].x < st[i - 1].x) st[i].x = st[i - 1].x + EPS_L;
    return { ...b, stations: st };
  }
  return b;
}

function fixSet(s: FinSet, warnings: string[]): FinSet {
  const out: FinSet = { ...s };
  out.nPanels = Math.max(1, Math.min(8, Math.round(out.nPanels || 1)));
  out.rootChord = Math.max(out.rootChord, EPS_L);
  out.tipChord = Math.max(out.tipChord, 0);
  out.semispan = Math.max(out.semispan, EPS_L);
  out.thickness = clamp(out.thickness || 0.04, 0, 0.5);
  if (out.flatFraction !== undefined) out.flatFraction = clamp(out.flatFraction, 0, 0.95);
  if (out.cfoc !== undefined) out.cfoc = clamp(out.cfoc, 0, 1);
  if (out.leRadius !== undefined && out.leRadius < 0) out.leRadius = 0;
  if (out.deflection) out.deflection = out.deflection.map((d) => (Number.isFinite(d) ? clamp(d, -60, 60) : 0));
  if (out.phiF) {
    out.phiF = out.phiF.slice(0, out.nPanels).map((p) => (Number.isFinite(p) ? ((p % 360) + 360) % 360 : 0));
    while (out.phiF.length < out.nPanels) out.phiF.push((360 / out.nPanels) * out.phiF.length);
  }
  if (!(out.xLE >= 0) || !Number.isFinite(out.xLE)) {
    warnings.push(`${s.name}: xLE invalid → 0`);
    out.xLE = 0;
  }
  if (!Number.isFinite(out.sweepLE)) out.sweepLE = 0;
  out.sweepLE = clamp(out.sweepLE, -80, 80);
  return out;
}

export function sanitizeConfig(cfg: VehicleConfig): VehicleConfig {
  const warnings: string[] = [...(cfg.notes ?? [])];
  const body = fixBody(cfg.body, warnings);
  const sets = (cfg.sets ?? []).map((s) => fixSet(s, warnings));
  const ref = { ...cfg.ref };
  // leave sRef/lRef undefined so resolveRefs recomputes; zero → undefined
  if (ref.sRef !== undefined && !(ref.sRef > EPS_A)) {
    warnings.push(`sRef ${ref.sRef} → auto`);
    ref.sRef = undefined;
  }
  if (ref.lRef !== undefined && !(ref.lRef > EPS_A)) {
    warnings.push(`lRef ${ref.lRef} → auto`);
    ref.lRef = undefined;
  }
  if (ref.latRef !== undefined && !(ref.latRef > EPS_A)) ref.latRef = undefined;
  if (!Number.isFinite(ref.xcg)) ref.xcg = 0;
  // elliptical / inlet / protuberance pass-through (sanitized in their modules)
  const out: VehicleConfig = {
    ...cfg,
    body,
    sets,
    ref,
    notes: warnings,
    inlets: cfg.inlets?.map(sanitizeInlet),
    protuberances: cfg.protuberances?.map(sanitizeProtuberance),
  };
  if (out.body?.elliptic) out.body = sanitizeElliptic(out.body);
  return out;
}

function sanitizeInlet(i: NonNullable<VehicleConfig["inlets"]>[number]) {
  return {
    ...i,
    captureArea: Math.max(i.captureArea, EPS_A),
    x: Number.isFinite(i.x) ? Math.max(i.x, 0) : 0,
    mfr: i.mfr !== undefined ? clamp(i.mfr, 0, 1.2) : 0.9,
    rampDeg: i.rampDeg !== undefined ? clamp(i.rampDeg, 0, 30) : undefined,
  };
}

function sanitizeProtuberance(p: NonNullable<VehicleConfig["protuberances"]>[number]) {
  return {
    ...p,
    length: Math.max(p.length, EPS_L),
    width: Math.max(p.width, EPS_L),
    height: Math.max(p.height, 0),
    x: Number.isFinite(p.x) ? Math.max(p.x, 0) : 0,
    count: Math.max(1, Math.round(p.count || 1)),
  };
}

function sanitizeElliptic(b: BodySpec): BodySpec {
  if (!b.elliptic) return b;
  const e = b.elliptic.map((s) => ({
    x: s.x,
    a: Math.max(s.a, 0),
    b: Math.max(s.b, 0),
  }));
  return { ...b, elliptic: e };
}

export function sanitizeFlight(f: FlightSpec): FlightSpec {
  const machs = (f.machs ?? []).filter((m) => Number.isFinite(m) && m > 0).map((m) => clamp(m, 0.01, 40));
  if (!machs.length) throw new ValidationError("flight.machs must contain at least one positive Mach number");
  let alphas = (f.alphas ?? []).filter((a) => Number.isFinite(a));
  if (!alphas.length) alphas = [0];
  // keep extreme alphas but clamp to ±180 for trig stability of the total-angle map
  alphas = alphas.map((a) => clamp(a, -180, 180));
  const altKm = Array.isArray(f.altKm)
    ? f.altKm.map((a) => clamp(Number.isFinite(a) ? a : 0, 0, 84.9))
    : clamp(typeof f.altKm === "number" && Number.isFinite(f.altKm) ? f.altKm : 0, 0, 84.9);
  return {
    ...f,
    machs,
    alphas,
    beta: Number.isFinite(f.beta ?? 0) ? clamp(f.beta ?? 0, -90, 90) : 0,
    phi: Number.isFinite(f.phi ?? 0) ? (((f.phi ?? 0) % 360) + 360) % 360 : 0,
    altKm,
    gamma: f.gamma !== undefined && f.gamma > 1.01 && f.gamma < 2 ? f.gamma : 1.4,
  };
}
