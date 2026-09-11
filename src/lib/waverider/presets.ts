import type { DesignParams } from "./types";
import { DEFAULT_PARAMS } from "./types";

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
];

const STORAGE_KEY = "bowshock.design.v4";

export function loadSavedParams(): DesignParams | null {
  try {
    const raw =
      localStorage.getItem(STORAGE_KEY) ??
      localStorage.getItem("bowshock.design.v3") ??
      localStorage.getItem("bowshock.design.v2") ??
      localStorage.getItem("bowshock.design.v1");
    if (!raw) return null;
    const parsed = JSON.parse(raw) as Partial<DesignParams>;
    return { ...DEFAULT_PARAMS, ...parsed };
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
