/**
 * Independent control surfaces: layout, listing, and channel mixing.
 *
 * Geometry is deflected where each family is built, so a deflected vehicle is
 * always a fresh `buildVehicle` (new positions / indices / surfaces arrays —
 * the panel WeakMap cache can never serve stale triangles):
 *
 * - Waverider families (generate.ts): part-span left / right elevons and a
 *   centre body flap extend the pre-fillet `applyElevon` shear
 *   z -= (x − x_h)·tan δ · W(y), with a spanwise partition of unity W. Rudders
 *   rotate the faired vertical-fin bump of `applyFins` about its vertical hinge
 *   line. Both are z-only maps of the lid / belly grids, so they cannot fold.
 * - Boost-glide (vehicles.ts): the same part-span shear on the planform loft.
 * - Missile / stack fin sets (vehicles.ts): each fin ridge rotates about its
 *   radial hinge line. The trailing part of the ridge swings circumferentially
 *   (Δθ on the body of revolution), so the round body is left exactly on its
 *   own surface and only the ridge flanks change incidence.
 * - Capsule, star, ramjet, scramjet: no control surfaces (main skips the
 *   elevon on ducts too).
 *
 * Sign conventions (positive deflection):
 * - elevons, body flap: trailing edge DOWN (same as main's `elevonDeg`).
 * - rudders: trailing edge LEFT (toward −Y; +Y is starboard in main's frame).
 * - missile fin k at roll angle θk (from +Y toward +Z): right-hand rotation about
 *   the outward radial axis. The +Y fin is then TE down, the +Z fin TE right.
 * - channels: δe = symmetric elevon (TE down); δa = right TE down / left TE up
 *   (elevon_r += δa, elevon_l −= δa); δr = rudder TE left; δbf = body flap TE
 *   down. Fin mixing (set 0): δk = δe·cos θk − δr·sin θk + δa.
 *
 * Main's moment axes are kept: Cm about +Y (nose up +), Cn about +Z (nose LEFT +),
 * Cll about +X (aft; right wing UP +). With those axes, +δa gives Cll > 0,
 * +δr gives CY > 0 and Cn > 0 on a tail rudder, +δe gives CL > 0 and Cm < 0
 * for an aft elevon.
 */
import { DEG, clamp } from "./math";
import {
  BODYFLAP_HINGE_FRAC,
  BOOSTGLIDE_HINGE_FRAC,
  ELEVON_ETA_BLEND,
  ELEVON_ETA_IN,
  ELEVON_HINGE_FRAC,
  finSets,
  finSurfaceId,
  spanWeights,
  type FinSet,
} from "./vehicles";
import type { BuiltVehicle, DesignParams, Vec3, WaveriderFamily } from "./types";

export type ControlKind = "elevon" | "bodyflap" | "rudder" | "fin";

export interface ControlSurface {
  /** Key into `DesignParams.controls`. */
  id: string;
  label: string;
  kind: ControlKind;
  side: "left" | "right" | "center";
  /** Hinge line endpoints, body frame (m). Origin at the tip, X streamwise, Y span, Z up. */
  hinge: { a: Vec3; b: Vec3 };
  /** Unit axis. Positive deflection is a right-hand rotation about it. */
  axis: Vec3;
  limitsDeg: [number, number];
  /** Mirror partner (left ↔ right, or fin at π − θ). */
  pair: string | null;
  /** Channel weights: surface deflection = Σ weight · channel. */
  mix: Partial<Record<string, number>>;
  /** Fin set index and roll angle (deg) for missile / stack fins. */
  set?: number;
  rollDeg?: number;
  note: string;
}

export interface ControlChannel {
  id: string;
  label: string;
  /** Symbol used in derivative names, e.g. "de" → Cm_de. */
  convention: string;
}

export const CONTROL_CHANNELS: Record<string, ControlChannel> = {
  de: { id: "de", label: "δe elevator", convention: "symmetric elevon / pitch fins, TE down positive" },
  da: { id: "da", label: "δa aileron", convention: "right TE down, left TE up (fins: all fins +δa, right-hand about outward radial)" },
  dr: { id: "dr", label: "δr rudder", convention: "TE left positive" },
  dbf: { id: "dbf", label: "δbf body flap", convention: "TE down positive" },
};

const DUCT: WaveriderFamily[] = ["ramjet", "scramjet"];
const NONE: WaveriderFamily[] = ["capsule", "star", ...DUCT];

export const RUDDER_HINGE_FRAC = 0.84;
export const FIN_LIMIT_DEG = 20;
export { BODYFLAP_HINGE_FRAC, BOOSTGLIDE_HINGE_FRAC, ELEVON_ETA_BLEND, ELEVON_ETA_IN, ELEVON_HINGE_FRAC, spanWeights };

/** Per-surface deflections (deg) actually requested for a set of ids. Zero or missing entries are omitted. */
export function activeControls(params: DesignParams, ids: string[]): Record<string, number> {
  const out: Record<string, number> = {};
  const c = params.controls;
  if (!c) return out;
  for (const id of ids) {
    const v = c[id];
    if (typeof v === "number" && Number.isFinite(v) && Math.abs(v) > 1e-9) out[id] = v;
  }
  return out;
}

function hasWaveriderControls(family: WaveriderFamily): boolean {
  return !NONE.includes(family) && family !== "boostglide" && family !== "missile" && family !== "multistage";
}

/** Waverider rudders exist when `applyFins` raises fins: same rule and stations as generate.ts. */
export function waveriderRudders(params: DesignParams): { id: string; yc: number }[] {
  const h = (params.finHeight || 0) * Math.max(params.height, 0.05);
  if (h < 1e-4 || !hasWaveriderControls(params.family)) return [];
  const yOff = params.span * 0.36;
  return params.halfModel
    ? [{ id: "rudder_r", yc: yOff }]
    : [
        { id: "rudder_l", yc: -yOff },
        { id: "rudder_r", yc: yOff },
      ];
}

function hingeZ(built: BuiltVehicle | undefined, x: number, y0: number, y1: number): number {
  if (!built) return 0;
  const p = built.mesh.positions;
  const lo = Math.min(y0, y1);
  const hi = Math.max(y0, y1);
  const tol = Math.max(built.params.length * 0.02, 1e-3);
  let zMin = Infinity;
  let zMax = -Infinity;
  for (let i = 0; i < p.length; i += 3) {
    if (Math.abs(p[i] - x) > tol || p[i + 1] < lo || p[i + 1] > hi) continue;
    zMin = Math.min(zMin, p[i + 2]);
    zMax = Math.max(zMax, p[i + 2]);
  }
  return Number.isFinite(zMin) ? 0.5 * (zMin + zMax) : 0;
}

function finLimit(p: DesignParams, s: FinSet, rb: (x: number) => number): number {
  // Δθ = (x − x_h)·sin δ / r_b. Ring order survives while max |dΔθ/dθ| = Δθ_max · n/2 < 0.8.
  let worst = 0;
  const cAft = Math.max(s.x1 - s.hinge, 1e-6);
  for (let k = 0; k <= 16; k++) {
    const x = s.hinge + (cAft * k) / 16;
    worst = Math.max(worst, (x - s.hinge) / Math.max(rb(x), 1e-4));
  }
  const sinMax = (0.8 * 2) / (s.n * Math.max(worst, 1e-9));
  const lim = sinMax >= 1 ? 90 : Math.asin(sinMax) / DEG;
  return Math.floor(Math.min(FIN_LIMIT_DEG, lim) * 10) / 10;
}

/**
 * The vehicle's control surfaces. Pass the built vehicle to get hinge-line Z
 * from the actual OML; with params only the hinge Z is 0 (the tip plane).
 */
export function listControlSurfaces(input: DesignParams | BuiltVehicle): ControlSurface[] {
  const built = "mesh" in input ? (input as BuiltVehicle) : undefined;
  const p: DesignParams = built ? built.params : (input as DesignParams);
  const L = Math.max(p.length, 1e-6);
  const fam = p.family;
  if (NONE.includes(fam)) return [];
  const out: ControlSurface[] = [];
  if (fam === "missile" || fam === "multistage") {
    const radius = Math.max(p.span / 2, 0.05);
    const sets = finSets(p, radius);
    const rb = (x: number) => {
      // body radius from the built ring nearest x (fin ridges excluded by taking the minimum)
      if (!built) return radius;
      const pos = built.mesh.positions;
      const tol = L / Math.max(4, p.nx);
      let r = Infinity;
      for (let i = 0; i < pos.length; i += 3) {
        if (Math.abs(pos[i] - x) > tol) continue;
        const rr = Math.hypot(pos[i + 1], pos[i + 2]);
        if (rr > 1e-6) r = Math.min(r, rr);
      }
      return Number.isFinite(r) ? r : radius;
    };
    sets.forEach((s, si) => {
      const lim = finLimit(p, s, rb);
      for (let k = 0; k < s.n; k++) {
        const th = (2 * Math.PI * k) / s.n;
        const er: Vec3 = [0, Math.cos(th), Math.sin(th)];
        const r0 = rb(s.hinge);
        const mirror = (((Math.PI - th) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
        const km = Math.round((mirror / (2 * Math.PI)) * s.n) % s.n;
        const exact = Math.abs((2 * Math.PI * km) / s.n - mirror) < 1e-6;
        const c = Math.cos(th);
        const sn = Math.sin(th);
        const mix: Partial<Record<string, number>> = {};
        const prefix = si === 0 ? "" : `fs${si}_`;
        if (Math.abs(c) > 1e-9) mix[`${prefix}de`] = c;
        mix[`${prefix}da`] = 1;
        if (Math.abs(sn) > 1e-9) mix[`${prefix}dr`] = -sn;
        out.push({
          id: finSurfaceId(si, k),
          label: `${si === 0 ? "Aft" : "Fwd"} fin ${k + 1} (θ ${((th / DEG) % 360).toFixed(0)}°)`,
          kind: "fin",
          side: Math.abs(c) < 1e-9 ? "center" : c > 0 ? "right" : "left",
          hinge: {
            a: [s.hinge, r0 * er[1], r0 * er[2]],
            b: [s.hinge, (r0 + s.height) * er[1], (r0 + s.height) * er[2]],
          },
          axis: er,
          limitsDeg: [-lim, lim],
          pair: exact && km !== k ? finSurfaceId(si, km) : null,
          mix,
          set: si,
          rollDeg: th / DEG,
          note: "Faired ridge. Aft of the hinge the ridge swings circumferentially by (x − x_h)·sin δ / r_body; the body of revolution is unchanged.",
        });
      }
    });
    return out;
  }
  const semi = Math.max(p.span / 2, 1e-6);
  const xE = (fam === "boostglide" ? BOOSTGLIDE_HINGE_FRAC : ELEVON_HINGE_FRAC) * L;
  const xB = BODYFLAP_HINGE_FRAC * L;
  const etaIn = ELEVON_ETA_IN * semi;
  const half = p.halfModel && fam !== "boostglide";
  const shearNote =
    fam === "boostglide"
      ? "Part-span shear z −= (x − x_h)·tan δ·W(y) on the boost-glide loft (same path as its elevon)."
      : "Part-span shear z −= (x − x_h)·tan δ·W(y), applied with main's applyElevon before the LE fillet.";
  if (!half) {
    out.push({
      id: "elevon_l",
      label: "Left elevon",
      kind: "elevon",
      side: "left",
      hinge: { a: [xE, -semi, hingeZ(built, xE, -semi, -etaIn)], b: [xE, -etaIn, hingeZ(built, xE, -semi, -etaIn)] },
      axis: [0, 1, 0],
      limitsDeg: [-20, 20],
      pair: "elevon_r",
      mix: { de: 1, da: -1 },
      note: shearNote,
    });
  }
  out.push({
    id: "elevon_r",
    label: "Right elevon",
    kind: "elevon",
    side: "right",
    hinge: { a: [xE, etaIn, hingeZ(built, xE, etaIn, semi)], b: [xE, semi, hingeZ(built, xE, etaIn, semi)] },
    axis: [0, 1, 0],
    limitsDeg: [-20, 20],
    pair: half ? null : "elevon_l",
    mix: { de: 1, da: 1 },
    note: shearNote,
  });
  out.push({
    id: "body_flap",
    label: "Body flap",
    kind: "bodyflap",
    side: "center",
    hinge: { a: [xB, half ? 0 : -etaIn, hingeZ(built, xB, -etaIn, etaIn)], b: [xB, etaIn, hingeZ(built, xB, -etaIn, etaIn)] },
    axis: [0, 1, 0],
    limitsDeg: [-10, 20],
    pair: null,
    mix: { dbf: 1 },
    note: shearNote,
  });
  for (const r of waveriderRudders(p)) {
    const zSign = p.lid === "bottom" ? -1 : 1;
    const xh = RUDDER_HINGE_FRAC * L;
    const z0 = hingeZ(built, xh, r.yc - 0.02 * semi, r.yc + 0.02 * semi);
    const h = (p.finHeight || 0) * Math.max(p.height, 0.05);
    out.push({
      id: r.id,
      label: r.id === "rudder_l" ? "Left rudder" : "Right rudder",
      kind: "rudder",
      side: r.id === "rudder_l" ? "left" : "right",
      hinge: { a: [xh, r.yc, z0], b: [xh, r.yc, z0 + zSign * h] },
      axis: [0, 0, -1],
      limitsDeg: [-20, 20],
      pair: r.id === "rudder_l" ? (p.halfModel ? null : "rudder_r") : p.halfModel ? null : "rudder_l",
      mix: { dr: 1 },
      note: "The faired fin bump of applyFins rotates about its vertical hinge line (z-only map of the lid grid). Resolution follows ny.",
    });
  }
  return out;
}

/** Channels a vehicle supports, in a stable order. */
export function controlChannels(input: DesignParams | BuiltVehicle): string[] {
  const set = new Set<string>();
  for (const s of listControlSurfaces(input)) for (const k of Object.keys(s.mix)) set.add(k);
  const order = ["de", "da", "dr", "dbf"];
  const rest = [...set].filter((k) => !order.includes(k)).sort();
  return [...order.filter((k) => set.has(k)), ...rest];
}

/**
 * Resolve channel values (de / da / dr / dbf / fsN_*) and raw surface ids into
 * per-surface deflections, clamped to each surface's limits. Raw surface ids add
 * on top of the mixed value.
 */
export function mixControls(
  surfaces: ControlSurface[],
  input: Partial<Record<string, number>>,
): { deflections: Record<string, number>; clamped: string[] } {
  const deflections: Record<string, number> = {};
  const clamped: string[] = [];
  for (const s of surfaces) {
    let d = 0;
    for (const [ch, w] of Object.entries(s.mix)) d += (w ?? 0) * (input[ch] ?? 0);
    d += input[s.id] ?? 0;
    if (!Number.isFinite(d) || Math.abs(d) < 1e-12) continue;
    const c = clamp(d, s.limitsDeg[0], s.limitsDeg[1]);
    if (c !== d) clamped.push(s.id);
    deflections[s.id] = c;
  }
  return { deflections, clamped };
}

/** DesignParams with channel / surface deflections resolved onto `controls` (merged with any already there). */
export function withControls(params: DesignParams, input: Partial<Record<string, number>>, surfaces = listControlSurfaces(params)): DesignParams {
  const merged: Partial<Record<string, number>> = { ...input };
  for (const [k, v] of Object.entries(params.controls ?? {})) merged[k] = (merged[k] ?? 0) + (v ?? 0);
  const { deflections } = mixControls(surfaces, merged);
  return { ...params, controls: deflections };
}
