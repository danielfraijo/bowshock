import type { DesignParams } from "./types";
import { DEFAULT_PARAMS, FAMILY_META } from "./types";
import { validateRefinement } from "./refinement"; // [mesh-refinement]

export interface Preset {
  id: string;
  name: string;
  blurb: string;
  params: DesignParams;
}

export const PRESETS: Preset[] = [
  {
    id: "caret-m8",
    name: "Caret M8",
    blurb: "Classic Nonweiler, 4 m, cruise Mach 8",
    params: { ...DEFAULT_PARAMS, family: "caret", mach: 8, length: 4, span: 2.4, height: 0.48, shockDeg: 15.5, name: "caret_m8" },
  },
  {
    id: "caret-m5",
    name: "Caret M5",
    blurb: "Lower Mach, thicker wedge for inlet studies",
    params: { ...DEFAULT_PARAMS, family: "caret", mach: 5, length: 3.2, span: 2.0, height: 0.55, shockDeg: 22, nx: 36, name: "caret_m5" },
  },
  {
    id: "cone-m10",
    name: "Cone M10",
    blurb: "Spatular cone-derived, Taylor–Maccoll shock",
    params: {
      ...DEFAULT_PARAMS,
      family: "cone",
      mach: 10,
      length: 5,
      span: 2.6,
      height: 0.42,
      coneDeg: 7,
      shockDeg: 12,
      captureFrac: 0.48,
      name: "cone_m10",
    },
  },
  {
    id: "osc-elliptic",
    name: "Osculating cruise",
    blurb: "Elliptic capture, power-law planform — HyCAUSE-like",
    params: {
      ...DEFAULT_PARAMS,
      family: "osculating",
      mach: 6,
      length: 6,
      span: 3.4,
      height: 0.52,
      superN: 2.1,
      planform: "power",
      planformPower: 1.25,
      shockDeg: 16,
      name: "osc_cruise",
    },
  },
  {
    id: "viscopt",
    name: "Viscous-opt M8",
    blurb: "Bowcutt-style power-law, volume vs L/D",
    params: {
      ...DEFAULT_PARAMS,
      family: "viscopt",
      mach: 8,
      length: 5.5,
      span: 3.0,
      height: 0.5,
      superN: 1.8,
      planform: "power",
      planformPower: 1.35,
      name: "viscopt_m8",
    },
  },
  {
    id: "inward",
    name: "Inward scoop",
    blurb: "REST-like sidewalls, deep keel for capture",
    params: {
      ...DEFAULT_PARAMS,
      family: "inward",
      mach: 7,
      length: 5,
      span: 2.4,
      height: 0.62,
      wedgeFrac: 0.62,
      planform: "spatular",
      captureFrac: 0.22,
      name: "inward_scoop",
    },
  },
  {
    id: "star-4",
    name: "Star 4-fin",
    blurb: "Missile / entry body, four intersecting carets",
    params: { ...DEFAULT_PARAMS, family: "star", mach: 8, length: 3.5, span: 1.1, height: 0.38, fins: 4, shockDeg: 18, name: "star_4fin" },
  },
  {
    id: "liftbody",
    name: "Lifting body",
    blurb: "Elliptic HTV-class, entry heating study",
    params: {
      ...DEFAULT_PARAMS,
      family: "liftbody",
      mach: 10,
      length: 8,
      span: 3.2,
      height: 1.1,
      planform: "power",
      planformPower: 0.85,
      leRadius: 0.08,
      altKm: 45,
      alphaDeg: 12,
      twK: 1400,
      name: "lift_body",
    },
  },
  {
    id: "ramjet",
    name: "Ramjet 2-D",
    blurb: "Shock-on-lip ramps, H2, M6 / 28 km",
    params: {
      ...DEFAULT_PARAMS,
      family: "ramjet",
      mach: 6,
      length: 4.5,
      span: 0.9,
      height: 0.55,
      inletHeight: 0.16,
      rampDeg: 11,
      cowlFrac: 0.4,
      combustorFrac: 0.22,
      nozzleER: 5,
      fuel: "H2",
      phi: 0.85,
      altKm: 28,
      flowThrough: true,
      name: "ramjet_2d",
    },
  },
  {
    id: "integrated",
    name: "WR + ramjet",
    blurb: "Osculating cruise with underslung cowl",
    params: {
      ...DEFAULT_PARAMS,
      family: "integrated",
      mach: 7,
      length: 6.5,
      span: 3.2,
      height: 0.55,
      superN: 2.2,
      planform: "power",
      inletHeight: 0.12,
      cowlFrac: 0.42,
      wedgeFrac: 0.32,
      fuel: "H2",
      name: "wr_ramjet",
    },
  },
  {
    id: "wedgecone",
    name: "Wedge–cone",
    blurb: "Flat keel with conical outboard wings",
    params: {
      ...DEFAULT_PARAMS,
      family: "wedgecone",
      mach: 8,
      length: 5,
      span: 2.8,
      height: 0.5,
      wedgeFrac: 0.38,
      superN: 2.4,
      planform: "power",
      name: "wedge_cone",
    },
  },
  {
    id: "elliptic",
    name: "Elliptic cone",
    blurb: "Rasmussen elliptic shock, M8 cruise",
    params: {
      ...DEFAULT_PARAMS,
      family: "elliptic",
      mach: 8,
      length: 5,
      span: 2.6,
      height: 0.48,
      planform: "delta",
      name: "elliptic_m8",
    },
  },
  {
    id: "busemann",
    name: "Busemann scoop",
    blurb: "High-contraction inward inlet study",
    params: {
      ...DEFAULT_PARAMS,
      family: "busemann",
      mach: 7,
      length: 4.2,
      span: 1.6,
      height: 0.7,
      captureFrac: 0.16,
      name: "busemann_scoop",
    },
  },
  {
    id: "scramjet",
    name: "Scramjet 2-D",
    blurb: "X-43-like multi-ramp, H2, M7 / 30 km",
    params: {
      ...DEFAULT_PARAMS,
      family: "scramjet",
      mach: 7,
      length: 4.8,
      span: 1.0,
      height: 0.5,
      inletHeight: 0.14,
      rampDeg: 9,
      nRamps: 3,
      cowlFrac: 0.38,
      combustorFrac: 0.24,
      nozzleER: 6,
      fuel: "H2",
      phi: 0.8,
      altKm: 30,
      name: "scram_2d",
    },
  },
  {
    id: "caret-fins",
    name: "Caret + fins",
    blurb: "M8 caret with elevons and verticals for trim",
    params: {
      ...DEFAULT_PARAMS,
      family: "caret",
      mach: 8,
      length: 4.2,
      span: 2.5,
      height: 0.5,
      shockDeg: 15,
      finHeight: 0.55,
      elevonDeg: 2,
      dihedralDeg: 4,
      name: "caret_fins",
    },
  },
  {
    id: "capsule-m20",
    name: "Sphere-cone",
    blurb: "Apollo-class blunt cone, 5 m base, 20° flank",
    params: {
      ...DEFAULT_PARAMS,
      family: "capsule",
      mach: 20,
      length: 3.2,
      span: 3.9,
      height: 1.2,
      coneDeg: 20,
      leRadius: 0.35,
      nx: 36,
      ny: 28,
      cgFrac: 0.62,
      name: "sphere_cone",
    },
  },
  {
    id: "boostglide-m15",
    name: "Boost-glide",
    blurb: "Cranked planform loft, M15, 6 m",
    params: {
      ...DEFAULT_PARAMS,
      family: "boostglide",
      mach: 15,
      length: 6,
      span: 2.4,
      height: 0.55,
      planform: "double",
      wedgeFrac: 0.42,
      leRadius: 0.03,
      cgFrac: 0.62,
      name: "boost_glide",
    },
  },
  {
    id: "missile-m6",
    name: "Missile",
    blurb: "Ogive-cylinder with two fin sets",
    params: {
      ...DEFAULT_PARAMS,
      family: "missile",
      mach: 6,
      length: 4.5,
      span: 0.45,
      height: 0.2,
      wedgeFrac: 0.28,
      finHeight: 0.7,
      fins: 4,
      nRamps: 2,
      captureFrac: 0.7,
      name: "missile",
    },
  },
  {
    id: "stack-2",
    name: "Two-stage stack",
    blurb: "Approximate booster OML, not a staging sim",
    params: {
      ...DEFAULT_PARAMS,
      family: "multistage",
      mach: 8,
      length: 12,
      span: 1.4,
      height: 1.4,
      nRamps: 2,
      finHeight: 0.35,
      fins: 4,
      name: "stack",
    },
  },
];

const STORAGE_KEY = "cuspis.design.v1";

const BANNED_KEYS = new Set(["__proto__", "constructor", "prototype"]);

/** Ranges a saved design is allowed to restore. Outside this, the field falls back to the default — a hand-edited localStorage blob cannot ask for a million cells. */
const NUM_RANGE: Partial<Record<keyof DesignParams, readonly [number, number]>> = {
  mach: [1.05, 30],
  flightMach: [0.2, 30],
  gamma: [1.05, 1.67],
  length: [0.05, 80],
  span: [0.05, 40],
  height: [0.01, 20],
  shockDeg: [1, 60],
  coneDeg: [1, 45],
  superN: [1, 8],
  planformPower: [0.4, 4],
  fins: [0, 12],
  wedgeFrac: [0.05, 0.95],
  captureFrac: [0.05, 0.95],
  nx: [12, 160],
  ny: [10, 120],
  leRadius: [0, 2],
  dihedralDeg: [-30, 45],
  camber: [-0.5, 0.8],
  teSweepDeg: [-30, 60],
  elevonDeg: [-30, 30],
  finHeight: [0, 3],
  nRamps: [1, 6],
  alphaDeg: [-40, 40],
  betaDeg: [-30, 30],
  gammaDeg: [-30, 30],
  altKm: [0, 120],
  cgFrac: [0.05, 0.95],
  twK: [200, 4000],
  inletHeight: [0.01, 2],
  cowlFrac: [0.05, 0.95],
  combustorFrac: [0.02, 0.8],
  nozzleER: [1, 40],
  rampDeg: [0, 40],
  phi: [0, 3],
  rhoKgM3: [1, 20000],
  massKg: [0, 1e6],
};

const ENUMS: Partial<Record<keyof DesignParams, readonly string[]>> = {
  planform: ["delta", "power", "spatular", "rect", "double"],
  lid: ["top", "bottom"],
  cowlSide: ["belly", "dorsal"],
  unit: ["m", "mm", "in"],
  fuel: ["H2", "JP"],
  aeroMethod: ["newtonian", "tangent", "mixed", "cbaero", "euler"],
};

function safeDesignName(v: string): string {
  const cleaned = v.replace(/[\u0000-\u001f\u007f]/g, "").replace(/[\\/]/g, "_").trim().slice(0, 64);
  return cleaned || DEFAULT_PARAMS.name;
}

/**
 * [mesh-refinement] Saved designs come from an older build or a hand-edited localStorage: a null / NaN / wrong-typed field or an
 * unknown family must not reach the generator. Every field is checked against the type of its default; refinement is
 * re-validated (clamped / capped); anything wrong falls back to the default for that field.
 * Unknown keys are dropped (no prototype pollution, nothing extra reaches an export).
 */
export function sanitizeParams(parsed: unknown): DesignParams {
  const src = parsed && typeof parsed === "object" ? (parsed as Record<string, unknown>) : {};
  const out: Record<string, unknown> = { ...DEFAULT_PARAMS };
  const def = DEFAULT_PARAMS as unknown as Record<string, unknown>;
  for (const k of Object.keys(src)) {
    if (BANNED_KEYS.has(k) || k === "refinement") continue;
    const v = src[k];
    if (k === "leBlend") {
      if (v === "circular" || v === "g2") out.leBlend = v;
      continue;
    }
    if (!(k in def)) continue;
    if (k === "name" && typeof v === "string") {
      out.name = safeDesignName(v);
      continue;
    }
    const allowed = ENUMS[k as keyof DesignParams];
    if (allowed) {
      if (typeof v === "string" && allowed.includes(v)) out[k] = v;
      continue;
    }
    const d = def[k];
    if (typeof d === "number") {
      const range = NUM_RANGE[k as keyof DesignParams];
      if (typeof v === "number" && Number.isFinite(v)) {
        out[k] = range ? Math.min(range[1], Math.max(range[0], v)) : v;
      }
    } else if (typeof d === "boolean") {
      if (typeof v === "boolean") out[k] = v;
    } else if (typeof d === "string") {
      if (typeof v === "string" && v.length <= 64 && !/[\u0000-\u001f\u007f]/.test(v)) out[k] = v;
    }
  }
  if (!(String(out.family) in FAMILY_META)) out.family = DEFAULT_PARAMS.family;
  const ref = src.refinement;
  if (ref && typeof ref === "object") out.refinement = validateRefinement(ref as Parameters<typeof validateRefinement>[0]).settings;
  else delete out.refinement;
  return out as unknown as DesignParams;
}

export function loadSavedParams(): DesignParams | null {
  try {
    const raw =
      localStorage.getItem(STORAGE_KEY) ??
      localStorage.getItem("bowshock.design.v4") ??
      localStorage.getItem("bowshock.design.v3") ??
      localStorage.getItem("bowshock.design.v2") ??
      localStorage.getItem("bowshock.design.v1");
    if (!raw || raw.length > 400_000) return null;
    const parsed = JSON.parse(raw, (key, value) => (BANNED_KEYS.has(key) ? undefined : value)) as Partial<DesignParams>;
    const merged: DesignParams = sanitizeParams(parsed); // [mesh-refinement] was { ...DEFAULT_PARAMS, ...parsed }
    if (parsed.leBlunt === undefined) {
      merged.leBlunt = true;
      if (!(typeof parsed.leRadius === "number" && parsed.leRadius > 0)) {
        merged.leRadius = Math.max(0.005 * merged.length, DEFAULT_PARAMS.leRadius);
      }
    }
    return merged;
  } catch {
    return null;
  }
}

export function saveParams(p: DesignParams) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(p));
  } catch {
    /* ignore quota */
  }
}
