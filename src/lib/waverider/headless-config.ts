/**
 * [aerodb-headless] Config validation for the batch CLI (`npm run aerodb`, entry scripts/cuspis-eval.ts).
 * The CLI drives main's public APIs only: aerodb.ts generateAeroDatabase, mdo.ts evaluateDesign / runOptimizer /
 * parameterSweep / sensitivity, guidance.ts flyClosedLoop. Section shapes mirror those APIs' spec types
 * (AeroDbSpec, OptSpec, FlySpec); omitted fields take the APIs' own defaults (defaultAeroDbSpec, defaultFlySpec).
 * validateHeadlessConfig() collects every problem with its JSON path.
 */
import { AERODB_POINT_CAP, defaultAeroDbSpec, gridCount, type AeroDbSpec } from "./aerodb";
import { defaultFlySpec, type FlySpec } from "./guidance";
import { MDO_METRICS, numericDesignKeys, type MdoMetric, type OptSpec } from "./mdo";
import { PRESETS, sanitizeParams } from "./presets";
import { DEFAULT_PARAMS, type AeroMethod, type DesignParams } from "./types";

export const HEADLESS_MODES = ["aerodb", "eval", "mdo", "sweep", "sensitivity", "fly"] as const;
export type HeadlessMode = (typeof HEADLESS_MODES)[number];
const AERO_METHODS: AeroMethod[] = ["newtonian", "tangent", "mixed", "cbaero", "euler"];

export interface HeadlessConfig {
  mode?: HeadlessMode;
  name?: string;
  description?: string;
  geometry?: { preset?: string; params?: Partial<DesignParams> };
  aerodb?: Partial<AeroDbSpec>;
  mdo?: Partial<OptSpec>;
  sweep?: { key: keyof DesignParams; values: number[] };
  sensitivity?: { keys?: (keyof DesignParams)[]; metric?: MdoMetric; frac?: number };
  fly?: Partial<FlySpec> & { massKg?: number; ixx?: number; iyy?: number; izz?: number };
}

export interface ResolvedConfig {
  mode: HeadlessMode;
  name: string;
  params: DesignParams;
  aerodb?: AeroDbSpec;
  aerodbPoints?: number;
  mdo?: OptSpec;
  sweep?: { key: keyof DesignParams; values: number[] };
  sensitivity?: { keys: (keyof DesignParams)[]; metric: MdoMetric; frac: number };
  fly?: FlySpec;
  flyMass?: { massKg?: number; ixx?: number; iyy?: number; izz?: number };
  warnings: string[];
}

export class ConfigError extends Error {
  constructor(public readonly problems: string[]) {
    super(`invalid config:\n${problems.map((p) => `  - ${p}`).join("\n")}`);
    this.name = "ConfigError";
  }
}

const isNum = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v);
const isObj = (v: unknown): v is Record<string, unknown> => !!v && typeof v === "object" && !Array.isArray(v);
const TOP = new Set(["$schema", "$comment", "mode", "name", "description", "geometry", "aerodb", "mdo", "sweep", "sensitivity", "fly"]);

/** Check `obj` against the field types of `def` (a default spec). Unknown keys and wrong types are errors. */
function checkAgainst(obj: unknown, def: Record<string, unknown>, path: string, errs: string[], extra: string[] = []): void {
  if (obj === undefined) return;
  if (!isObj(obj)) {
    errs.push(`${path}: expected an object`);
    return;
  }
  for (const [k, v] of Object.entries(obj)) {
    if (extra.includes(k)) continue;
    if (!(k in def)) errs.push(`${path}.${k}: unknown key (known: ${Object.keys(def).join(", ")})`);
    else if (typeof def[k] === "number" && !isNum(v)) errs.push(`${path}.${k}: expected a finite number, got ${JSON.stringify(v)}`);
    else if (typeof def[k] !== "number" && typeof v !== typeof def[k]) errs.push(`${path}.${k}: expected ${typeof def[k]}, got ${JSON.stringify(v)}`);
  }
}

function resolveParams(g: unknown, errs: string[], warnings: string[]): DesignParams {
  let params: DesignParams = { ...DEFAULT_PARAMS };
  if (g === undefined) return sanitizeParams(params);
  if (!isObj(g)) {
    errs.push(`geometry: expected {"preset": "<id>"} and/or {"params": {…}}`);
    return params;
  }
  for (const k of Object.keys(g)) if (k !== "preset" && k !== "params") errs.push(`geometry.${k}: unknown key (use preset / params)`);
  if (g.preset !== undefined) {
    const p = PRESETS.find((x) => x.id === g.preset);
    if (!p) errs.push(`geometry.preset: unknown preset ${JSON.stringify(g.preset)} (known: ${PRESETS.map((x) => x.id).join(", ")})`);
    else params = { ...p.params };
  }
  if (g.params === undefined) return sanitizeParams(params);
  if (!isObj(g.params)) {
    errs.push("geometry.params: expected an object of DesignParams fields");
    return params;
  }
  const user = g.params;
  const def = DEFAULT_PARAMS as unknown as Record<string, unknown>;
  for (const [k, v] of Object.entries(user)) {
    if (k === "refinement" || k === "leBlend") continue;
    if (!(k in def)) errs.push(`geometry.params.${k}: unknown design parameter`);
    else if (typeof v !== typeof def[k] || (typeof v === "number" && !Number.isFinite(v))) errs.push(`geometry.params.${k}: expected ${typeof def[k]}, got ${JSON.stringify(v)}`);
  }
  const clean = sanitizeParams({ ...params, ...user }) as unknown as Record<string, unknown>;
  for (const [k, v] of Object.entries(user)) {
    if (!(k in def) || typeof v !== typeof def[k]) continue;
    if (typeof v === "number" && Number.isFinite(v) && clean[k] !== v) warnings.push(`geometry.params.${k}: ${v} clamped to ${clean[k]}`);
    else if (typeof v === "string" && clean[k] !== v) errs.push(`geometry.params.${k}: invalid value ${JSON.stringify(v)}`);
  }
  return clean as unknown as DesignParams;
}

/** Validate a parsed JSON config; `modeOverride` comes from --mode. Throws ConfigError listing every problem. */
export function validateHeadlessConfig(raw: unknown, modeOverride?: string): ResolvedConfig {
  const errs: string[] = [];
  const warnings: string[] = [];
  if (!isObj(raw)) throw new ConfigError(["config must be a JSON object"]);
  for (const k of Object.keys(raw)) if (!TOP.has(k)) errs.push(`${k}: unknown key`);
  const mode = (modeOverride ?? raw.mode ?? "aerodb") as HeadlessMode;
  if (!HEADLESS_MODES.includes(mode)) errs.push(`mode: expected one of ${HEADLESS_MODES.join(", ")}, got ${JSON.stringify(mode)}`);
  for (const k of ["name", "description"]) if (raw[k] !== undefined && typeof raw[k] !== "string") errs.push(`${k}: expected a string`);
  const params = resolveParams(raw.geometry, errs, warnings);
  const out: ResolvedConfig = { mode, name: (raw.name as string) || params.name || "vehicle", params, warnings };
  const numericKeys = numericDesignKeys() as string[];

  // aerodb
  const adef = defaultAeroDbSpec(params);
  const a0 = errs.length;
  checkAgainst(raw.aerodb, adef as unknown as Record<string, unknown>, "aerodb", errs);
  if (mode === "aerodb" && errs.length === a0) {
    const spec = { ...adef, ...(raw.aerodb as Partial<AeroDbSpec>) };
    if (!AERO_METHODS.includes(spec.method)) errs.push(`aerodb.method: expected one of ${AERO_METHODS.join(", ")}`);
    for (const ax of ["mach", "alpha", "beta", "delta"] as const) {
      const lo = spec[`${ax}Min`], hi = spec[`${ax}Max`], st = spec[`${ax}Step`];
      if (hi < lo) errs.push(`aerodb.${ax}Max (${hi}) < ${ax}Min (${lo})`);
      if (!(st > 0)) errs.push(`aerodb.${ax}Step: must be > 0`);
      if (st > 0 && (hi - lo) / st + 1 > 21) warnings.push(`aerodb.${ax}: ${Math.floor((hi - lo) / st + 1)} breakpoints requested; aerodb.ts axisBreaks resamples to 21 evenly spaced`);
    }
    if (spec.machMin < 1.05) errs.push(`aerodb.machMin: ${spec.machMin} < 1.05 (panel methods are supersonic)`);
    if (spec.altKm < 0 || spec.altKm > 84.9) errs.push(`aerodb.altKm: ${spec.altKm} outside the US76 range 0–84.9 km`);
    if (errs.length === a0) {
      const n = gridCount(spec, params.mach, params.alphaDeg).points;
      if (n > AERODB_POINT_CAP) errs.push(`aerodb: grid has ${n} points > AERODB_POINT_CAP ${AERODB_POINT_CAP} (aerodb.ts); widen a step or shrink a range`);
      out.aerodbPoints = n;
    }
    out.aerodb = spec;
  }

  // mdo
  if (raw.mdo !== undefined || mode === "mdo") {
    const m = (raw.mdo ?? {}) as Record<string, unknown>;
    const m0 = errs.length;
    if (!isObj(m)) errs.push("mdo: expected an object");
    else {
      for (const k of Object.keys(m)) if (!["variables", "objectives", "constraints", "population", "generations", "seed"].includes(k)) errs.push(`mdo.${k}: unknown key`);
      const vars = (m.variables ?? []) as unknown[];
      if (!Array.isArray(vars)) errs.push("mdo.variables: expected a list");
      else
        vars.forEach((v, i) => {
          const o = v as Record<string, unknown>;
          if (!isObj(o) || !numericKeys.includes(o.key as string)) errs.push(`mdo.variables[${i}].key: expected one of ${numericKeys.join(", ")}`);
          else if (!isNum(o.min) || !isNum(o.max) || !(o.max > o.min)) errs.push(`mdo.variables[${i}]: need finite min < max`);
        });
      if (mode === "mdo" && Array.isArray(vars) && vars.length === 0) errs.push("mdo.variables: at least one variable is required");
      const objs = (m.objectives ?? []) as unknown[];
      if (!Array.isArray(objs)) errs.push("mdo.objectives: expected a list");
      else
        objs.forEach((v, i) => {
          const o = v as Record<string, unknown>;
          if (!isObj(o) || !MDO_METRICS.includes(o.metric as MdoMetric)) errs.push(`mdo.objectives[${i}].metric: expected one of ${MDO_METRICS.join(", ")}`);
          else if (o.goal !== "min" && o.goal !== "max") errs.push(`mdo.objectives[${i}].goal: expected "min" or "max"`);
        });
      const cons = (m.constraints ?? []) as unknown[];
      if (!Array.isArray(cons)) errs.push("mdo.constraints: expected a list");
      else
        cons.forEach((v, i) => {
          const o = v as Record<string, unknown>;
          if (!isObj(o) || !MDO_METRICS.includes(o.metric as MdoMetric)) errs.push(`mdo.constraints[${i}].metric: expected one of ${MDO_METRICS.join(", ")}`);
          else if (o.op !== ">=" && o.op !== "<=") errs.push(`mdo.constraints[${i}].op: expected ">=" or "<="`);
          else if (!isNum(o.value)) errs.push(`mdo.constraints[${i}].value: expected a number`);
        });
      for (const k of ["population", "generations", "seed"]) if (m[k] !== undefined && !(isNum(m[k]) && (m[k] as number) >= 1)) errs.push(`mdo.${k}: expected a number ≥ 1`);
      if (errs.length === m0)
        out.mdo = {
          variables: vars as OptSpec["variables"],
          objectives: ((objs as OptSpec["objectives"]) ?? []).map((o) => ({ ...o, weight: o.weight ?? 1 })),
          constraints: cons as OptSpec["constraints"],
          population: (m.population as number) ?? 8,
          generations: (m.generations as number) ?? 5,
          seed: (m.seed as number) ?? 1,
        };
    }
  }

  // sweep
  if (raw.sweep !== undefined || mode === "sweep") {
    const s = raw.sweep as Record<string, unknown> | undefined;
    const s0 = errs.length;
    if (!isObj(s)) errs.push(`sweep: required for mode "sweep", {"key": "<DesignParams field>", "values": [...]}`);
    else {
      for (const k of Object.keys(s)) if (k !== "key" && k !== "values") errs.push(`sweep.${k}: unknown key`);
      if (!numericKeys.includes(s.key as string)) errs.push(`sweep.key: expected one of ${numericKeys.join(", ")}`);
      if (!Array.isArray(s.values) || s.values.length === 0 || !s.values.every(isNum)) errs.push("sweep.values: expected a non-empty list of numbers");
      if (errs.length === s0) out.sweep = { key: s.key as keyof DesignParams, values: s.values as number[] };
    }
  }

  // sensitivity
  if (raw.sensitivity !== undefined || mode === "sensitivity") {
    const s = (raw.sensitivity ?? {}) as Record<string, unknown>;
    const n0 = errs.length;
    if (!isObj(s)) errs.push("sensitivity: expected an object");
    else {
      for (const k of Object.keys(s)) if (!["keys", "metric", "frac"].includes(k)) errs.push(`sensitivity.${k}: unknown key`);
      const keys = (s.keys ?? numericKeys) as unknown[];
      if (!Array.isArray(keys) || keys.some((k) => !numericKeys.includes(k as string))) errs.push(`sensitivity.keys: expected a list drawn from ${numericKeys.join(", ")}`);
      const metric = (s.metric ?? "ld") as MdoMetric;
      if (!MDO_METRICS.includes(metric)) errs.push(`sensitivity.metric: expected one of ${MDO_METRICS.join(", ")}`);
      const frac = (s.frac ?? 0.05) as number;
      if (!(isNum(frac) && frac > 0 && frac < 1)) errs.push("sensitivity.frac: expected 0 < frac < 1");
      if (errs.length === n0) out.sensitivity = { keys: keys as (keyof DesignParams)[], metric, frac };
    }
  }

  // fly
  const fdef = defaultFlySpec(params);
  const f0 = errs.length;
  checkAgainst(raw.fly, fdef as unknown as Record<string, unknown>, "fly", errs, ["massKg", "ixx", "iyy", "izz"]);
  if ((raw.fly !== undefined || mode === "fly") && errs.length === f0) {
    const f = (raw.fly ?? {}) as Record<string, unknown>;
    const spec = { ...fdef, ...f } as FlySpec & Record<string, unknown>;
    if (!["boost-glide", "constant-q", "skip", "terminal"].includes(spec.mode)) errs.push(`fly.mode: expected boost-glide, constant-q, skip or terminal`);
    if (spec.law !== "pid" && spec.law !== "ndi") errs.push(`fly.law: expected "pid" or "ndi"`);
    for (const k of ["durationS", "dt", "tauAct", "deltaMaxDeg", "rateMaxDeg"]) if (!((spec[k] as number) > 0)) errs.push(`fly.${k}: must be > 0`);
    for (const k of ["massKg", "ixx", "iyy", "izz"]) if (f[k] !== undefined && !(isNum(f[k]) && (f[k] as number) > 0)) errs.push(`fly.${k}: expected a number > 0`);
    if (spec.dt < 0.01 || spec.dt > 0.2) warnings.push(`fly.dt ${spec.dt} is clamped to [0.01, 0.2] s by guidance.ts`);
    if (errs.length === f0) {
      const { massKg, ixx, iyy, izz, ...rest } = spec as FlySpec & { massKg?: number; ixx?: number; iyy?: number; izz?: number };
      out.fly = rest as FlySpec;
      out.flyMass = { massKg, ixx, iyy, izz };
    }
  }
  if (errs.length) throw new ConfigError(errs);
  return out;
}
