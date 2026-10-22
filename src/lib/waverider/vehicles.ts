/**
 * Parametric vehicle classes beyond inverse-design waveriders.
 *
 * Capsule, missile, and multi-stage bodies are surfaces of revolution (plus
 * faired fin ridges). Boost-glide is a direct planform loft, not a shock-traced
 * waverider. Every builder returns a closed triangle mesh that the existing
 * panel methods and CAD export can consume.
 *
 * Assumptions (screening, not a CAD master model):
 * - Fins are finite-angle ridges faired into the OML so the skin stays a
 *   single watertight shell. They are not hinged CAD flaps. Elevon deflection
 *   shears the horizontal ridges about their hinge line.
 * - A multi-stage concept is the stacked outer mold line. Stages do not
 *   separate, and interstage gaps are not modeled.
 * - Reference area is base area πRb² for bodies of revolution and planform
 *   area for the boost-glide loft. Reference length stays the geometric length.
 * - The shock grid is a conical/ruled sheet for the viewer, not a captured bow shock.
 * - Half-model is ignored: these OMLs are full bodies. Symmetry-plane Euler
 *   still sees the +Z / −Z meridians.
 */
import { makeGrid, MeshBuilder } from "./mesh";
import { DEG, clamp } from "./math";
import type { DesignParams, SurfaceGrid, TriMesh, WaveriderFamily } from "./types";

export interface ClassVehicle {
  mesh: TriMesh;
  grids: SurfaceGrid[];
  shock: SurfaceGrid[];
  skipped: number;
  notes: string[];
  /** Reference area used by the panel bookkeeping, m². */
  sRef: number;
}

export const CLASS_FAMILIES: WaveriderFamily[] = ["capsule", "boostglide", "missile", "multistage"];

export function isClassFamily(family: string): boolean {
  return (CLASS_FAMILIES as string[]).includes(family);
}

const CLASS_NOTES: Partial<Record<WaveriderFamily, string>> = {
  capsule:
    "Sphere-cone. Span is the base diameter, cone half-angle is the flank, LE radius is the nose radius. Sref = πRb².",
  boostglide:
    "Parametric boost-glide loft (planform × elliptic section). Not an inverse waverider — the shock sheet is a ruled visualization.",
  missile:
    "Tangent-ogive / cylinder / boattail. Span is the diameter. Fin height grows cruciform ridges; elevon shears the horizontal pair. Sref = πRb².",
  multistage:
    "Stacked OML, nose to fat aft stage. Stage count is Ramps. Not a separating stack. Sref = πRb² of the base.",
};

export function classNote(family: WaveriderFamily): string | undefined {
  return CLASS_NOTES[family];
}

export function buildClassVehicle(params: DesignParams): ClassVehicle | null {
  switch (params.family) {
    case "capsule":
      return buildCapsule(params);
    case "boostglide":
      return buildBoostglide(params);
    case "missile":
      return buildMissile(params);
    case "multistage":
      return buildStack(params);
    default:
      return null;
  }
}

/** Spanwise station (|η| = |y|/semispan) where the elevons end and the body flap begins. */
export const ELEVON_ETA_IN = 0.3;
/** Width of the spanwise blend between elevon and body flap, in η. */
export const ELEVON_ETA_BLEND = 0.16;
export const ELEVON_HINGE_FRAC = 0.82;
export const BODYFLAP_HINGE_FRAC = 0.86;
export const BOOSTGLIDE_HINGE_FRAC = 0.78;

/** Spanwise weights (a partition of unity) for left elevon, right elevon and centre body flap at η = y / semispan. */
export function spanWeights(eta: number): { l: number; r: number; c: number } {
  const t = clamp((Math.abs(eta) - ELEVON_ETA_IN) / ELEVON_ETA_BLEND + 0.5, 0, 1);
  const out = t * t * (3 - 2 * t);
  return { l: eta < 0 ? out : 0, r: eta > 0 ? out : 0, c: 1 - out };
}

export interface FinSet {
  x0: number;
  x1: number;
  n: number;
  height: number;
  halfAng: number;
  hinge: number;
}

function wrapPi(d: number): number {
  let x = d;
  while (x > Math.PI) x -= 2 * Math.PI;
  while (x < -Math.PI) x += 2 * Math.PI;
  return x;
}

export function finSets(p: DesignParams, radius: number): FinSet[] {
  const h = Math.max(0, p.finHeight) * Math.max(p.height, radius, 0.05);
  if (h < 1e-4) return [];
  const L = Math.max(p.length, 0.2);
  const n = clamp(Math.round(p.fins || 4), 2, 8);
  const chord = clamp(p.combustorFrac || 0.22, 0.08, 0.4) * L;
  const sets: FinSet[] = [];
  const xAft = L * clamp(0.9 - chord / L, 0.35, 0.9);
  sets.push({
    x0: xAft,
    x1: Math.min(L * 0.985, xAft + chord),
    n,
    height: h,
    halfAng: 8 * DEG,
    hinge: xAft + chord * 0.72,
  });
  if ((p.nRamps || 1) >= 2) {
    const x2 = L * clamp(p.wedgeFrac, 0.2, 0.55);
    const c2 = chord * 0.75;
    if (x2 + c2 < xAft - 0.02 * L) {
      sets.push({
        x0: x2,
        x1: x2 + c2,
        n,
        height: h * 0.62,
        halfAng: 7 * DEG,
        hinge: x2 + c2 * 0.7,
      });
    }
  }
  return sets.filter((s) => s.x1 > s.x0 + 1e-4);
}

/** Control-surface id of fin k (0 at +Y, counter-clockwise toward +Z) in fin set s (0 = aft). */
export function finSurfaceId(set: number, k: number): string {
  return `fin${set}_${k}`;
}

/**
 * Ring stations θ ∈ [0, 2π] for a body of revolution. Without fins this is
 * main's uniform ring. With fins every sector between two fins gets the same
 * stations, mirrored about the fin centre, and each ridge is sampled at its
 * crest, its half-width and its foot, so all n fins are resolved identically.
 * (Uniform θ put the +Y fin on a vertex and the others between vertices:
 * a 4-fin missile had CY ≈ −0.026 and Cll ≈ 2e-3 at β = 0.)
 */
export function ringStations(nj: number, sets: FinSet[]): number[] {
  if (!sets.length) {
    const out: number[] = [];
    for (let j = 0; j < nj; j++) out.push((2 * Math.PI * j) / (nj - 1));
    return out;
  }
  const n = sets[0].n;
  const w = (2 * Math.PI) / n;
  const ha = Math.min(Math.max(...sets.map((s) => s.halfAng)), w * 0.3);
  const dUni = (2 * Math.PI) / Math.max(nj - 1, 1);
  const interior = Math.max(1, Math.ceil((w - 2 * ha) / dUni - 1e-9) - 1);
  const local: number[] = [0, ha * 0.5, ha];
  for (let k = 1; k <= interior; k++) local.push(ha + ((w - 2 * ha) * k) / (interior + 1));
  local.push(w - ha, w - ha * 0.5);
  const out: number[] = [];
  for (let f = 0; f < n; f++) for (const t of local) out.push(f * w + t);
  out.push(2 * Math.PI);
  return out;
}

/** Circumferential rotation (rad) of a ring vertex for per-fin hinge deflections. 0 when no fin is deflected. */
function finSwing(theta: number, x: number, rb: number, sets: FinSet[], defl: number[][], fades: number[]): number {
  let dth = 0;
  for (let si = 0; si < sets.length; si++) {
    const s = sets[si];
    const ds = defl[si];
    if (!ds) continue;
    if (x <= s.hinge) continue;
    const cAft = Math.max(s.x1 - s.hinge, 1e-6);
    // aft of the fin the swing fades back to 0 on the bare body (a pure Δθ leaves a body of revolution on itself)
    let u = Math.min(x - s.hinge, cAft);
    if (x > s.x1) {
      const t = clamp((x - s.x1) / Math.max(fades[si], 1e-9), 0, 1);
      u *= 1 - t * t * (3 - 2 * t);
      if (u <= 0) continue;
    }
    // ramp the hinge rotation in over the first quarter of the moving chord, so the skin has no kink at the hinge
    const ramp = (() => {
      const t = clamp(u / (0.25 * cAft), 0, 1);
      return t * t * (3 - 2 * t);
    })();
    const hw = Math.PI / s.n;
    for (let k = 0; k < s.n; k++) {
      const d = ds[k];
      if (!d) continue;
      const th0 = (2 * Math.PI * k) / s.n;
      const a = Math.abs(wrapPi(theta - th0));
      if (a >= hw) continue;
      const bell = 0.5 * (1 + Math.cos((a / hw) * Math.PI));
      // right-hand rotation about the outward radial axis moves the TE toward −e_θ
      dth -= (u * Math.sin(d * DEG * ramp) * bell) / Math.max(rb, 1e-6);
    }
  }
  return dth;
}

function axialEnvelope(x: number, x0: number, x1: number): number {
  if (x <= x0 || x >= x1) return 0;
  const u = (x - x0) / (x1 - x0);
  return Math.sin(Math.PI * u);
}

function finExtra(theta: number, x: number, sets: FinSet[]): number {
  let extra = 0;
  for (const s of sets) {
    const env = axialEnvelope(x, s.x0, s.x1);
    if (env <= 0) continue;
    for (let k = 0; k < s.n; k++) {
      const th0 = (2 * Math.PI * k) / s.n;
      const d = Math.abs(wrapPi(theta - th0));
      if (d >= s.halfAng) continue;
      const shape = 0.5 * (1 + Math.cos((d / s.halfAng) * Math.PI));
      extra = Math.max(extra, s.height * env * shape);
    }
  }
  return extra;
}

function horizShear(theta: number, x: number, sets: FinSet[]): { w: number; hinge: number } {
  let w = 0;
  let hinge = x;
  for (const s of sets) {
    const env = axialEnvelope(x, s.x0, s.x1);
    if (env <= 0) continue;
    for (let k = 0; k < s.n; k++) {
      const th0 = (2 * Math.PI * k) / s.n;
      if (Math.abs(Math.cos(th0)) < 0.7) continue;
      const d = Math.abs(wrapPi(theta - th0));
      if (d >= s.halfAng) continue;
      const shape = 0.5 * (1 + Math.cos((d / s.halfAng) * Math.PI));
      const ww = env * shape;
      if (ww > w) {
        w = ww;
        hinge = s.hinge;
      }
    }
  }
  return { w, hinge };
}

function meridian(name: string, xs: number[], zs: number[], yEps: number): SurfaceGrid {
  const ni = xs.length;
  const nj = 3;
  const xyz = new Float64Array(ni * nj * 3);
  for (let i = 0; i < ni; i++) {
    for (let j = 0; j < nj; j++) {
      const o = (i * nj + j) * 3;
      xyz[o] = xs[i];
      xyz[o + 1] = (j - 1) * yEps;
      xyz[o + 2] = zs[i];
    }
  }
  return { name, ni, nj, xyz };
}

function shockBeta(p: DesignParams): number {
  const M = Math.max(p.mach, 1.2);
  const mu = Math.asin(clamp(1 / M, 0, 1));
  return clamp(Math.max(p.shockDeg * DEG, mu + 1.5 * DEG), mu + 0.5 * DEG, 68 * DEG);
}

function revolve(
  p: DesignParams,
  radiusAt: (x: number) => number,
  sRef: number,
  notes: string[],
  withFins: boolean,
): ClassVehicle {
  const L = Math.max(p.length, 0.2);
  const ni = clamp(Math.round(p.nx) || 28, 16, 72);
  const nj = clamp(Math.round(p.ny) || 20, 12, 56);
  const b = new MeshBuilder(L);
  const sets = withFins ? finSets(p, Math.max(p.span / 2, 0.05)) : [];
  const defl = clamp((p.elevonDeg || 0) * DEG, -18 * DEG, 18 * DEG);
  const thetas = ringStations(nj, sets);
  const finDefl: number[][] = sets.map((s, si) => {
    const row: number[] = [];
    for (let k = 0; k < s.n; k++) {
      const v = p.controls?.[finSurfaceId(si, k)];
      row.push(typeof v === "number" && Number.isFinite(v) ? v : 0);
    }
    return row;
  });
  const anyFin = finDefl.some((r) => r.some((v) => Math.abs(v) > 1e-9));
  const fades = sets.map((s, si) => (si === 0 ? Math.max(L - s.x1, 1e-4) : Math.max(0.9 * (sets[0].x0 - s.x1), 1e-4)));
  const xs: number[] = [];
  const rs: number[] = [];
  const ids: number[][] = [];
  for (let i = 0; i < ni; i++) {
    const x = (L * i) / (ni - 1);
    const rb = Math.max(0, radiusAt(x));
    xs.push(x);
    rs.push(rb);
    const ring: number[] = [];
    for (let j = 0; j < thetas.length; j++) {
      const th0 = thetas[j];
      const rr = rb + finExtra(th0, x, sets);
      const th = anyFin ? th0 + finSwing(th0, x, rb, sets, finDefl, fades) : th0;
      let y = rr * Math.cos(th);
      let z = rr * Math.sin(th);
      if (sets.length && Math.abs(defl) > 1e-8) {
        const sh = horizShear(th, x, sets);
        z += Math.tan(defl) * (x - sh.hinge) * sh.w;
      }
      ring.push(b.vert(x, y, z));
    }
    ids.push(ring);
  }
  for (let i = 0; i < ni - 1; i++) {
    for (let j = 0; j < thetas.length - 1; j++) {
      const th = 0.5 * (thetas[j] + thetas[j + 1]);
      const surf = Math.sin(th) >= 0 ? "upper" : "lower";
      b.quad(ids[i][j], ids[i][j + 1], ids[i + 1][j + 1], ids[i + 1][j], surf);
    }
  }
  if (rs[ni - 1] > 1e-5) {
    const hub = b.vert(L, 0, 0);
    const last = ids[ni - 1];
    for (let j = 0; j < thetas.length - 1; j++) b.tri(hub, last[j], last[j + 1], "base");
  }
  if (rs[0] > 1e-4) {
    const hub = b.vert(0, 0, 0);
    const first = ids[0];
    for (let j = 0; j < thetas.length - 1; j++) b.tri(hub, first[j + 1], first[j], "leading");
  }
  const mesh = b.finish();
  const yEps = Math.max(p.span, L) * 2e-4;
  const upper = meridian("upper", xs, rs, yEps);
  const lower = meridian("lower", xs, rs.map((r) => -r), yEps);
  const nsi = Math.min(ni, 28);
  const nsj = Math.min(nj, 20);
  const beta = shockBeta(p);
  const shock = makeGrid("shock", nsi, nsj, (i, j) => {
    const x = (L * i) / (nsi - 1);
    const th = (2 * Math.PI * j) / (nsj - 1);
    const rsH = Math.max(radiusAt(x) * 1.04 + 0.01 * L, (x + 0.12 * L) * Math.tan(beta) * 0.45);
    return [x, rsH * Math.cos(th), rsH * Math.sin(th)];
  });
  if (sets.length) {
    notes.push(
      `${sets.length} fin set${sets.length > 1 ? "s" : ""} as faired ridges (half-angle ~8°). Elevon ${p.elevonDeg.toFixed(1)}° shears the horizontal pair about the hinge. Not a separate flap volume.`,
    );
  }
  if (anyFin) {
    const list = finDefl.flatMap((r, si) => r.map((v, k) => (v ? `${finSurfaceId(si, k)} ${v.toFixed(1)}°` : "")).filter(Boolean));
    notes.push(`Fin hinge deflections: ${list.join(", ")}. The ridge swings about its radial hinge; the round body is unchanged.`);
  }
  if (p.halfModel) notes.push("Half-model is not applied to bodies of revolution — the full OML is meshed.");
  return { mesh, grids: [upper, lower], shock: [shock], skipped: b.skipped, notes, sRef };
}

function buildCapsule(p: DesignParams): ClassVehicle {
  const L = Math.max(p.length, 0.2);
  const Rb = Math.max(0.05, p.span / 2);
  const theta = clamp(p.coneDeg || 20, 5, 55) * DEG;
  let Rn = p.leRadius > 1e-6 ? p.leRadius : Math.min(0.45 * Rb, 0.28 * L);
  Rn = clamp(Rn, 1e-3, Math.min(L * 0.9, Rb / Math.max(Math.cos(theta), 0.25)));
  const xt = Rn * (1 - Math.sin(theta));
  const rt = Rn * Math.cos(theta);
  const notes = [
    `Sphere-cone capsule. Rn = ${Rn.toFixed(3)} m, cone ${(theta / DEG).toFixed(1)}°, base diameter ${(2 * Rb).toFixed(3)} m. Sref = πRb² = ${(Math.PI * Rb * Rb).toFixed(4)} m².`,
    "Shoulder flare is added only if the cone has not reached the requested base by x = L. Heating still uses the LE radius on the nose.",
  ];
  const sphereCone = (x: number) => {
    const xc = clamp(x, 0, L);
    if (xc <= xt + 1e-9) {
      const c = clamp(1 - xc / Math.max(Rn, 1e-6), -1, 1);
      return Rn * Math.sqrt(Math.max(0, 1 - c * c));
    }
    return Math.max(0, rt + (xc - xt) * Math.tan(theta));
  };
  let radiusAt = sphereCone;
  if (sphereCone(L) > Rb * 1.002) {
    let xHit = L;
    for (let i = 0; i <= 40; i++) {
      const x = xt + ((L - xt) * i) / 40;
      if (sphereCone(x) >= Rb) {
        xHit = x;
        break;
      }
    }
    radiusAt = (x) => Math.min(Rb, sphereCone(x));
    notes.push(`Cone reaches the base diameter at x/L = ${(xHit / L).toFixed(2)}; the flank is cylindrical after that.`);
  } else if (sphereCone(L) < Rb * 0.98) {
    const xflare = Math.max(xt, L * 0.8);
    const r0 = Math.min(Rb, sphereCone(xflare));
    radiusAt = (x) => {
      if (x <= xflare) return Math.min(Rb, sphereCone(x));
      const u = (x - xflare) / Math.max(L - xflare, 1e-6);
      return r0 + (Rb - r0) * u;
    };
    notes.push("Cone is short of the requested diameter — a conical shoulder flare closes the base.");
  }
  return revolve(p, radiusAt, Math.PI * Rb * Rb, notes, false);
}

function buildMissile(p: DesignParams): ClassVehicle {
  const L = Math.max(p.length, 0.2);
  const Rb = Math.max(0.04, p.span / 2);
  const Ln = clamp(p.wedgeFrac || 0.28, 0.12, 0.65) * L;
  const Lbt = clamp(0.08 + (1 - clamp(p.cowlFrac || 0.7, 0.4, 0.95)) * 0.35, 0.06, 0.4) * L;
  const baseR = Rb * clamp(p.captureFrac || 0.72, 0.35, 1);
  const rho = (Rb * Rb + Ln * Ln) / (2 * Rb);
  const notes = [
    `Missile OML: tangent ogive Ln/L = ${(Ln / L).toFixed(2)}, diameter ${(2 * Rb).toFixed(3)} m, boattail to ${(2 * baseR).toFixed(3)} m. Sref = πRb².`,
    "Ridges stand in for fin sets. A second set appears when Ramps ≥ 2. Panel methods see the faired OML, not a thin-fin vortex lattice.",
  ];
  const radiusAt = (x: number) => {
    const xc = clamp(x, 0, L);
    if (xc <= Ln) {
      const dx = Ln - xc;
      return Math.max(0, Math.sqrt(Math.max(0, rho * rho - dx * dx)) - (rho - Rb));
    }
    if (xc >= L - Lbt) {
      const u = (xc - (L - Lbt)) / Math.max(Lbt, 1e-6);
      return Rb + (baseR - Rb) * u;
    }
    return Rb;
  };
  return revolve(p, radiusAt, Math.PI * Rb * Rb, notes, true);
}

function buildStack(p: DesignParams): ClassVehicle {
  const L = Math.max(p.length, 0.2);
  const n = clamp(Math.round(p.nRamps || 2), 2, 4);
  const Rb = Math.max(0.05, p.span / 2);
  const Ls = L / n;
  const stations: number[] = [];
  for (let i = 0; i <= n; i++) stations.push(Rb * (0.22 + 0.78 * (i / n)));
  stations[0] = 0;
  const notes = [
    `${n}-stage stack, approximate OML only (no separation, no interstage cavity). Base diameter ${(2 * Rb).toFixed(3)} m. Sref = πRb².`,
    "Stage radii grow smoothly from a pointed nose to the aft stage. Use fin height for aft control ridges.",
  ];
  const radiusAt = (x: number) => {
    const xc = clamp(x, 0, L);
    const u = xc / Ls;
    const i = Math.min(n - 1, Math.floor(u));
    const t = u - i;
    const s = t * t * (3 - 2 * t);
    return stations[i] + (stations[i + 1] - stations[i]) * s;
  };
  return revolve(p, radiusAt, Math.PI * Rb * Rb, notes, true);
}

function halfSpanAt(x: number, p: DesignParams): number {
  const L = Math.max(p.length, 0.2);
  const xi = clamp(x / L, 0, 1);
  const b = Math.max(p.span, 0.08) / 2;
  const pow = clamp(p.planformPower || 1, 0.45, 2.4);
  let y: number;
  switch (p.planform) {
    case "power":
      y = b * Math.pow(xi, pow);
      break;
    case "spatular":
      y = b * (0.32 + 0.68 * Math.pow(xi, 0.6));
      break;
    case "rect":
      y = b * Math.min(1, xi / 0.16);
      break;
    case "double": {
      const xk = clamp(p.wedgeFrac || 0.45, 0.25, 0.7);
      y = xi <= xk ? b * 0.62 * (xi / xk) : b * (0.62 + 0.38 * ((xi - xk) / (1 - xk)));
      break;
    }
    default:
      y = b * xi;
  }
  const rn = p.leBlunt === false ? 0 : Math.max(p.leRadius, 0.004 * L);
  if (rn > 0) y = Math.max(y, rn * Math.sqrt(Math.max(0, 1 - xi / 0.12)));
  return y;
}

function sectionThick(x: number, p: DesignParams): number {
  const L = Math.max(p.length, 0.2);
  const xi = clamp(x / L, 0, 1);
  const h = Math.max(p.height, 0.03);
  const cam = 1 + 0.4 * clamp(p.camber, -0.4, 0.6);
  const env = Math.max(0.16, Math.sin(Math.PI * Math.pow(xi, 0.72)) * (1 - 0.35 * xi) + 0.12);
  return h * cam * env;
}

interface PartSpan {
  elevon_l: number;
  elevon_r: number;
  body_flap: number;
}

function boostglideControls(p: DesignParams): PartSpan | null {
  const c = p.controls;
  if (!c) return null;
  const v = (k: string) => {
    const d = c[k];
    return typeof d === "number" && Number.isFinite(d) ? clamp(d, -25, 25) : 0;
  };
  const out = { elevon_l: v("elevon_l"), elevon_r: v("elevon_r"), body_flap: v("body_flap") };
  return out.elevon_l || out.elevon_r || out.body_flap ? out : null;
}

/**
 * Part-span elevon / body-flap shear for the boost-glide loft: the same
 * z −= (x − x_h)·tan δ as the full-span elevon, weighted by a spanwise
 * partition of unity (controls.ts `spanWeights`). z-only, so it cannot fold.
 */
function partSpanShear(x: number, L: number, d: PartSpan): (eta: number) => number {
  const xE = BOOSTGLIDE_HINGE_FRAC * L;
  const xB = BODYFLAP_HINGE_FRAC * L;
  const kL = x > xE ? (x - xE) * Math.tan(d.elevon_l * DEG) : 0;
  const kR = x > xE ? (x - xE) * Math.tan(d.elevon_r * DEG) : 0;
  const kB = x > xB ? (x - xB) * Math.tan(d.body_flap * DEG) : 0;
  return (eta: number) => {
    const w = spanWeights(eta);
    return -(kL * w.l + kR * w.r + kB * w.c);
  };
}

function buildBoostglide(p: DesignParams): ClassVehicle {
  const L = Math.max(p.length, 0.2);
  const ni = clamp(Math.round(p.nx) || 28, 14, 64);
  const nj = clamp(Math.round(p.ny) || 18, 10, 48);
  const b = new MeshBuilder(L);
  const dih = Math.tan(clamp(p.dihedralDeg || 0, -15, 25) * DEG);
  const notes = [
    `Boost-glide loft. Planform ${p.planform}, span ${p.span.toFixed(2)} m, height ${p.height.toFixed(2)} m. Direct parametric OML — the inverse-design shock is not enforced.`,
    "Section is a lee-side sheet with an elliptic belly. Nose-high incidence makes the belly windward at α = 0. Tips pinch. Sref is the projected planform.",
  ];
  if (p.halfModel) notes.push("Half-model is not applied — the full planform is meshed.");
  if (Math.abs(p.elevonDeg || 0) > 0.05) {
    notes.push(`Elevon ${p.elevonDeg.toFixed(1)}° shears the surface aft of 78% length (trailing edge down is positive). Same sign as the waverider elevon.`);
  }
  const partSpan = boostglideControls(p);
  const semiB = Math.max(p.span, 0.08) / 2;
  if (partSpan) {
    notes.push(
      `Control surfaces: ${Object.entries(partSpan)
        .filter(([, v]) => v)
        .map(([k, v]) => `${k} ${(v as number).toFixed(1)}°`)
        .join(", ")} (part-span shear, TE down positive).`,
    );
  }
  const upper: number[][] = [];
  const lower: number[][] = [];
  const xyzU = new Float64Array(ni * nj * 3);
  const xyzL = new Float64Array(ni * nj * 3);
  let sRef = 0;
  let prevY = halfSpanAt(0, p);
  for (let i = 0; i < ni; i++) {
    const x = (L * i) / (ni - 1);
    const yHalf = Math.max(halfSpanAt(x, p), 1e-5);
    if (i > 0) sRef += (prevY + yHalf) * (L / (ni - 1));
    prevY = yHalf;
    const thick = sectionThick(x, p);
    const xi = x / L;
    const camb = p.camber * p.height * 4 * xi * (1 - xi);
    // Nose sits high and the sheet slopes down aft. Volume hangs below it,
    // so the belly outward normal has an upstream component at α = 0.
    const incidence = 0.22 * p.height * (1 - xi);
    const elev = x > 0.78 * L ? -(x - 0.78 * L) * Math.tan(clamp(p.elevonDeg || 0, -18, 18) * DEG) : 0;
    const part = partSpan ? partSpanShear(x, L, partSpan) : null;
    const rowU: number[] = [];
    const rowL: number[] = [];
    for (let j = 0; j < nj; j++) {
      const t = j / (nj - 1);
      const y = -yHalf + 2 * yHalf * t;
      const eta = clamp(y / yHalf, -1, 1);
      const zSheet = incidence + camb + Math.abs(y) * dih;
      const drop = thick * Math.sqrt(Math.max(0, 1 - eta * eta));
      const ctl = part ? part(y / semiB) : 0;
      const zU = zSheet + elev + ctl;
      const zL = zSheet - drop + elev + ctl;
      rowL.push(b.vert(x, y, zL));
      rowU.push(b.vert(x, y, zU));
      const o = (i * nj + j) * 3;
      xyzU[o] = x;
      xyzU[o + 1] = y;
      xyzU[o + 2] = zU;
      xyzL[o] = x;
      xyzL[o + 1] = y;
      xyzL[o + 2] = zL;
    }
    upper.push(rowU);
    lower.push(rowL);
  }
  for (let i = 0; i < ni - 1; i++) {
    for (let j = 0; j < nj - 1; j++) {
      b.quad(upper[i][j], upper[i][j + 1], upper[i + 1][j + 1], upper[i + 1][j], "upper");
      b.quad(lower[i][j], lower[i + 1][j], lower[i + 1][j + 1], lower[i][j + 1], "lower");
    }
  }
  for (let j = 0; j < nj - 1; j++) {
    b.quad(upper[0][j], lower[0][j], lower[0][j + 1], upper[0][j + 1], "leading");
    b.quad(upper[ni - 1][j], upper[ni - 1][j + 1], lower[ni - 1][j + 1], lower[ni - 1][j], "base");
  }
  const mesh = b.finish();
  const beta = shockBeta(p);
  const shock = makeGrid("shock", Math.min(ni, 24), Math.min(nj, 16), (i, j) => {
    const x = (L * i) / Math.max(1, Math.min(ni, 24) - 1);
    const yHalf = halfSpanAt(x, p) * 1.08;
    const njj = Math.min(nj, 16);
    const y = -yHalf + (2 * yHalf * j) / Math.max(1, njj - 1);
    const zBelly = -0.04 * p.height - Math.max(0.015 * L, x * Math.tan(beta) * 0.22);
    return [x, y, zBelly];
  });
  return {
    mesh,
    grids: [
      { name: "upper", ni, nj, xyz: xyzU },
      { name: "lower", ni, nj, xyz: xyzL },
    ],
    shock: [shock],
    skipped: b.skipped,
    notes,
    sRef: Math.max(sRef, 1e-4),
  };
}
