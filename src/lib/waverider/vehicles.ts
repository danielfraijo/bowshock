/**
 * Parametric vehicle classes beyond inverse-design waveriders.
 *
 * Capsule, missile, and multi-stage bodies are surfaces of revolution plus
 * flat fins. Boost-glide is a direct planform loft, not a shock-traced
 * waverider. Every builder returns a closed triangle mesh that the existing
 * panel methods and CAD export can consume.
 *
 * Assumptions (screening, not a CAD master model):
 * - Fins are constant-thickness plates in the shell (cruciform, or n equally
 *   spaced). The tip is a straight line at one radius, so a boat-tail or
 *   interstage cannot kink it. Elevon is a flap: only the part aft of the
 *   hinge moves, trailing edge down positive. The fin ahead of the hinge stays
 *   fixed — swinging it throws the leading edge through the body.
 * - A multi-stage concept is cylindrical stages with a conical nose, conical
 *   interstages, and a short nozzle boat-tail. Stages do not separate.
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
    "Sphere-cone. Diameter is the base, cone half-angle is the flank, nose radius is the sphere. A length past the shoulder adds a cylinder; a shorter length truncates the cone. No flare. Sref = π Rbase².",
  boostglide:
    "Parametric boost-glide loft (planform × elliptic section). Not an inverse waverider — the shock sheet is a ruled visualization.",
  missile:
    "Tangent ogive, cylinder, boat-tail, and constant-thickness fins. Diameter is the cylinder. Nose fraction is the ogive. Base d/D is the boat-tail. Fin height is in body radii. Sref = π R².",
  multistage:
    "Conical nose, cylindrical stages, conical interstages, short nozzle. Upper stages are smaller. Fin height is in base radii. Stages do not separate. Sref = π Rbase².",
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

interface Plate {
  th0: number;
  height: number;
  thick: number;
  x0: number;
  x1: number;
  sweep: number;
  taper: number;
  hinge: number;
  horizontal: boolean;
  /** Full-height tip radius. Constant, so the planform stays straight. */
  rTip: number;
}

/** Shape defaults applied when the user picks a class, so a waverider's span is not reused as a diameter. */
export const CLASS_SHAPE: Partial<Record<WaveriderFamily, Partial<DesignParams>>> = {
  capsule: { length: 3.5, span: 3.9, height: 1.8, coneDeg: 25, leRadius: 0.6, leBlunt: true, finHeight: 0, fins: 0, nx: 48, ny: 36 },
  missile: {
    length: 5.2,
    span: 0.36,
    height: 0.36,
    wedgeFrac: 0.3,
    captureFrac: 0.86,
    leRadius: 0.012,
    leBlunt: true,
    finHeight: 0.85,
    fins: 4,
    nRamps: 1,
    nx: 64,
    ny: 28,
  },
  multistage: { length: 16, span: 1.8, height: 1.8, finHeight: 0.35, fins: 4, nRamps: 2, leRadius: 0.02, nx: 72, ny: 28 },
  boostglide: { length: 6, span: 2.4, height: 0.55, planform: "double", wedgeFrac: 0.42, leRadius: 0.03, leBlunt: true, finHeight: 0, nx: 40, ny: 24 },
};

function wrapPi(d: number): number {
  let x = d;
  while (x > Math.PI) x -= 2 * Math.PI;
  while (x < -Math.PI) x += 2 * Math.PI;
  return x;
}

function axisStations(L: number, ni: number, extra: number[]): number[] {
  const want = clamp(Math.round(ni) || 28, 16, 80);
  const xs = [0, L];
  for (let i = 1; i < want - 1; i++) xs.push((L * i) / (want - 1));
  for (const x of extra) xs.push(x);
  xs.sort((a, b) => a - b);
  const tol = Math.max(1e-4, 0.0012 * L);
  const out = [0];
  for (const x of xs) {
    const c = clamp(x, 0, L);
    if (c - out[out.length - 1] > tol) out.push(c);
  }
  if (L - out[out.length - 1] > tol * 0.25) out.push(L);
  else out[out.length - 1] = L;
  return out;
}

function plateSpan(f: Plate, x: number): number {
  const c = f.x1 - f.x0;
  if (!(c > 1e-6) || x <= f.x0 || x >= f.x1) return 0;
  const xLE = f.x0 + f.sweep * c;
  const tipC = clamp(f.taper, 0.2, 0.9) * c;
  const xTE = Math.min(f.x1 - c * 0.02, xLE + tipC);
  const sLE = x < xLE ? (f.height * (x - f.x0)) / Math.max(xLE - f.x0, 1e-6) : f.height;
  const sTE = x > xTE ? (f.height * (f.x1 - x)) / Math.max(f.x1 - xTE, 1e-6) : f.height;
  return Math.max(0, Math.min(sLE, sTE, f.height));
}

function radiusRuns(radiusAt: (x: number) => number, L: number): { x0: number; x1: number; r: number }[] {
  const n = 96;
  const xs: number[] = [];
  const rs: number[] = [];
  for (let i = 0; i < n; i++) {
    const x = (L * i) / (n - 1);
    xs.push(x);
    rs.push(Math.max(0, radiusAt(x)));
  }
  const quiet = (i: number) => {
    if (i <= 0 || i >= n - 1) return false;
    return Math.abs(rs[i] - rs[i - 1]) / Math.max(xs[i] - xs[i - 1], 1e-9) < 0.15;
  };
  const runs: { x0: number; x1: number; r: number }[] = [];
  let a = -1;
  const push = (i0: number, i1: number) => {
    if (xs[i1] - xs[i0] < 0.06 * L) return;
    let r = 0;
    for (let k = i0; k <= i1; k++) r = Math.max(r, rs[k]);
    runs.push({ x0: xs[i0], x1: xs[i1], r });
  };
  for (let i = 0; i < n; i++) {
    if (quiet(i)) {
      if (a < 0) a = Math.max(0, i - 1);
    } else if (a >= 0) {
      push(a, i - 1);
      a = -1;
    }
  }
  if (a >= 0) push(a, n - 1);
  return runs;
}

function makePlates(p: DesignParams, Rb: number, radiusAt: (x: number) => number): Plate[] {
  const hFrac = Math.max(0, p.finHeight);
  if (hFrac < 1e-3 || !(Rb > 1e-4)) return [];
  const L = Math.max(p.length, 0.2);
  const n = clamp(Math.round(p.fins || 4), 2, 8);
  const weld = 8e-4 * L;
  const runs = radiusRuns(radiusAt, L);
  const aft = runs.length ? runs[runs.length - 1] : { x0: 0.72 * L, x1: 0.98 * L, r: Math.max(0, radiusAt(0.9 * L)) };
  const chord = clamp(0.16 * L, 0.08 * L, Math.max(0.1 * L, 0.9 * (aft.x1 - aft.x0 + 0.06 * L)));
  // Sit on the cylindrical run and reach at most halfway into the boat-tail or nozzle behind it.
  let x1 = Math.min(0.985 * L, aft.x1 + 0.5 * Math.max(0, L - aft.x1));
  x1 = Math.max(x1, Math.min(0.985 * L, aft.x0 + 0.1 * L));
  let x0 = Math.max(0.06 * L, x1 - chord);
  if (x1 - x0 < 0.045 * L) return [];
  const rUnder = (a: number, b: number) => {
    let m = 0;
    for (let i = 0; i <= 12; i++) m = Math.max(m, Math.max(0, radiusAt(a + ((b - a) * i) / 12)));
    return m;
  };
  const slot = (Math.PI / n) * 0.62;
  const thickFor = (rRef: number, height: number) => {
    const r = Math.max(rRef, 0.02);
    let thick = Math.max(3.5 * weld, 0.045 * r);
    thick = Math.min(thick, 0.16 * r, Math.max(0.22 * height, 4 * weld));
    return Math.min(thick, 2 * r * Math.sin(slot));
  };
  const plates: Plate[] = [];
  const push = (xa: number, xb: number, height: number, rRef: number) => {
    if (!(xb > xa + 0.02 * L) || !(height > 1e-4)) return;
    const hinge = xa + 0.62 * (xb - xa);
    const thick = thickFor(rRef, height);
    for (let k = 0; k < n; k++) {
      const th0 = (2 * Math.PI * k) / n;
      plates.push({
        th0,
        height,
        thick,
        x0: xa,
        x1: xb,
        sweep: 0.18,
        taper: 0.5,
        hinge,
        horizontal: Math.abs(Math.cos(th0)) > 0.65,
        rTip: rUnder(xa, xb) + height,
      });
    }
  };
  const tailH = hFrac * Math.max(aft.r, 0.55 * Rb);
  push(x0, x1, tailH, Math.max(aft.r, 0.5 * Rb));
  const wantCanard = (p.family === "missile" || p.family === "multistage") && (p.nRamps || 1) >= 2;
  if (wantCanard) {
    const prev = runs.length >= 2 ? runs[runs.length - 2] : null;
    const separate = prev && prev.r > 0.4 * Rb && prev.x1 < x0 - 0.04 * L;
    const host = separate ? prev : aft;
    const room = Math.max(0.04 * L, host.x1 - host.x0);
    const c2 = Math.min(chord * 0.55, 0.72 * room);
    let xC1: number;
    let xC0: number;
    if (separate) {
      xC1 = host.x1 - 0.01 * L;
      xC0 = Math.max(host.x0 + 0.01 * L, xC1 - c2);
    } else {
      xC0 = Math.max(host.x0 + 0.01 * L, x0 - 0.05 * L - c2);
      xC1 = xC0 + c2;
    }
    if (xC1 <= x0 - 0.035 * L && xC1 > xC0 + 0.02 * L && xC0 > 0.04 * L) {
      const hC = tailH * 0.55 * (host.r / Math.max(aft.r, 1e-6));
      push(xC0, xC1, hC, host.r);
    }
  }
  return plates;
}

interface Spoke {
  th0: number;
  plates: Plate[];
  thick: number;
  horizontal: boolean;
}

/** One circumferential slot per azimuth. A canard and a tail at the same angle share it, so the ring never contains two fins on top of each other. */
function spokesOf(plates: Plate[]): Spoke[] {
  const map = new Map<number, Spoke>();
  for (const f of plates) {
    const key = Math.round(f.th0 * 1e6);
    let s = map.get(key);
    if (!s) {
      s = { th0: f.th0, plates: [], thick: f.thick, horizontal: f.horizontal };
      map.set(key, s);
    }
    s.plates.push(f);
    s.thick = Math.max(s.thick, f.thick);
  }
  return [...map.values()].sort((a, b) => a.th0 - b.th0);
}

function spokeAt(s: Spoke, x: number): { span: number; hinge: number; rTip: number; height: number; thick: number } {
  let span = 0;
  let hinge = x;
  let rTip = 0;
  let height = 1;
  let thick = s.thick;
  for (const f of s.plates) {
    const h = plateSpan(f, x);
    if (h > span) {
      span = h;
      hinge = f.hinge;
      rTip = f.rTip;
      height = f.height;
      thick = f.thick;
    }
  }
  return { span, hinge, rTip, height, thick };
}
function gapThetas(spokes: Spoke[], half: number[], nSlot: number[]): number[] {
  if (!spokes.length) return [];
  const ths: number[] = [];
  for (let i = 0; i < spokes.length; i++) {
    const k = spokes[i];
    const kn = spokes[(i + 1) % spokes.length];
    let a = k.th0 + half[i];
    let b = kn.th0 - half[(i + 1) % spokes.length];
    if (i === spokes.length - 1) b += 2 * Math.PI;
    const gap = Math.max(0, b - a);
    const n = nSlot[i] ?? 1;
    for (let s = 0; s < n; s++) ths.push(a + ((s + 0.5) * gap) / n);
  }
  return ths;
}

function cornerYZ(
  rb: number,
  th0: number,
  span: number,
  height: number,
  rTip: number,
  thick: number,
  corner: number,
  defl: number,
  hinge: number,
  x: number,
  horizontal: boolean,
): [number, number] {
  const out = corner === 1 || corner === 2;
  const sign = corner === 0 || corner === 1 ? -1 : 1;
  const frac = height > 1e-8 ? clamp(span / height, 0, 1) : 0;
  const rad = out ? rb + Math.max(0, rTip - rb) * frac : rb;
  const cy = Math.cos(th0);
  const sy = Math.sin(th0);
  const sMax = 0.42 * Math.max(rad, 1e-8);
  const s = sign * Math.min(thick * 0.5, sMax);
  let y = rad * cy + s * -sy;
  let z = rad * sy + s * cy;
  // Flap only. The fin ahead of the hinge is fixed.
  if (horizontal && out && span > 1e-8 && x > hinge + 1e-9 && Math.abs(defl) > 1e-8) z -= Math.tan(defl) * (x - hinge);
  return [y, z];
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
  plates: Plate[],
): ClassVehicle {
  const L = Math.max(p.length, 0.2);
  const niReq = clamp(Math.round(p.nx) || 28, 16, 72);
  const nBody = clamp(Math.round(p.ny) || 24, 12, 48);
  const b = new MeshBuilder(L);
  const defl = clamp((p.elevonDeg || 0) * DEG, -18 * DEG, 18 * DEG);
  const rRef = Math.max(p.span / 2, 1e-3);
  const spokes = spokesOf(plates);
  const half = spokes.map((s) => {
    const slot = (Math.PI / Math.max(spokes.length, 1)) * 0.62;
    const a = Math.atan(Math.min(s.thick * 0.5, 0.42 * rRef) / rRef);
    return Math.min(a * 1.15, slot);
  });
  const nSlot: number[] = [];
  if (spokes.length) {
    for (let i = 0; i < spokes.length; i++) {
      const kn = (i + 1) % spokes.length;
      let a0 = spokes[i].th0 + half[i];
      let b0 = spokes[kn].th0 - half[kn];
      if (i === spokes.length - 1) b0 += 2 * Math.PI;
      const gap = Math.max(0, b0 - a0);
      nSlot.push(Math.max(1, Math.round((nBody * gap) / (2 * Math.PI))));
    }
  }
  const thetas = spokes.length
    ? gapThetas(spokes, half, nSlot)
    : Array.from({ length: nBody }, (_, j) => (2 * Math.PI * j) / nBody);
  const extra: number[] = [];
  for (const f of plates) {
    const c = Math.max(f.x1 - f.x0, 1e-6);
    for (const u of [0, 0.1, 0.22, 0.4, 0.62, 0.78, 0.9, 1]) extra.push(f.x0 + u * c);
    extra.push(f.hinge);
  }
  const xs = axisStations(L, niReq, extra);
  const rs = xs.map((x) => Math.max(0, radiusAt(x)));
  const ids: number[][] = [];
  const yz: [number, number][][] = [];
  for (let i = 0; i < xs.length; i++) {
    const x = xs[i];
    const rb = rs[i];
    const ring: number[] = [];
    const yzr: [number, number][] = [];
    const thin = rb < Math.max(2.4 * (plates[0]?.thick ?? 0), 0.12 * rRef);
    const pushPt = (y: number, z: number) => {
      yzr.push([y, z]);
      ring.push(b.vert(x, y, z));
    };
    if (!spokes.length) {
      for (const th of thetas) pushPt(rb * Math.cos(th), rb * Math.sin(th));
    } else {
      let bi = 0;
      for (let ii = 0; ii < spokes.length; ii++) {
        const s = spokes[ii];
        const live = thin ? { span: 0, hinge: x, rTip: rb, height: 1, thick: s.thick } : spokeAt(s, x);
        if (live.span <= 1e-8) {
          const y = rb * Math.cos(s.th0);
          const z = rb * Math.sin(s.th0);
          for (let c = 0; c < 4; c++) pushPt(y, z);
        } else {
          for (let c = 0; c < 4; c++) {
            const [y, z] = cornerYZ(rb, s.th0, live.span, live.height, live.rTip, live.thick, c, defl, live.hinge, x, s.horizontal);
            pushPt(y, z);
          }
        }
        const n = nSlot[ii] ?? 1;
        for (let k = 0; k < n; k++) {
          const th = thetas[bi++];
          pushPt(rb * Math.cos(th), rb * Math.sin(th));
        }
      }
    }
    ids.push(ring);
    yz.push(yzr);
  }
  const nj = ids[0].length;
  for (let i = 0; i < xs.length - 1; i++) {
    for (let j = 0; j < nj; j++) {
      const j2 = (j + 1) % nj;
      const zc = (yz[i][j][1] + yz[i][j2][1] + yz[i + 1][j][1] + yz[i + 1][j2][1]) * 0.25;
      const surf = zc >= 0 ? "upper" : "lower";
      b.quad(ids[i][j], ids[i][j2], ids[i + 1][j2], ids[i + 1][j], surf);
    }
  }
  if (rs[rs.length - 1] > 1e-5) {
    const hub = b.vert(L, 0, 0);
    const last = ids[ids.length - 1];
    for (let j = 0; j < nj; j++) b.tri(hub, last[j], last[(j + 1) % nj], "base");
  }
  const mesh = b.finish();
  const yEps = Math.max(p.span, L) * 2e-4;
  const upper = meridian("upper", xs, rs, yEps);
  const lower = meridian("lower", xs, rs.map((r) => -r), yEps);
  const nsi = Math.min(xs.length, 28);
  const nsj = Math.min(Math.max(nBody, 12), 24);
  const beta = shockBeta(p);
  let finH = 0;
  for (const f of plates) finH = Math.max(finH, f.height);
  const shock = makeGrid("shock", nsi, nsj, (i, j) => {
    const x = (L * i) / (nsi - 1);
    const th = (2 * Math.PI * j) / nsj;
    const body = Math.max(0, radiusAt(x));
    const rsH = Math.max((body + finH) * 1.12, (x + 0.08 * L) * Math.tan(beta) * 0.35);
    return [x, rsH * Math.cos(th), rsH * Math.sin(th)];
  });
  if (plates.length) {
    const nFin = new Set(plates.map((f) => f.th0.toFixed(4))).size;
    const sets = new Set(plates.map((f) => f.x0.toFixed(3))).size;
    notes.push(
      `${sets} fin set${sets > 1 ? "s" : ""}, ${nFin} plates. Tail height ${(plates[0].height * 1000).toFixed(0)} mm, thickness ${(plates[0].thick * 1000).toFixed(0)} mm. Elevon ${p.elevonDeg.toFixed(1)}° moves only the flap aft of the hinge.`,
    );
  }
  if (p.halfModel) notes.push("Half-model is not applied to bodies of revolution — the full OML is meshed.");
  return { mesh, grids: [upper, lower], shock: [shock], skipped: b.skipped, notes, sRef };
}

/** Body radius of a capsule, missile, or stack. Null for other families. Fins are not included. */
export function revolutionRadius(p: DesignParams, x: number): number | null {
  if (p.family === "capsule") return capsuleRadius(p)(x);
  if (p.family === "missile") return missileRadius(p).r(x);
  if (p.family === "multistage") return stackRadius(p)(x);
  return null;
}

function capsuleRadius(p: DesignParams): (x: number) => number {
  const L = Math.max(p.length, 0.2);
  const Rb = Math.max(0.05, p.span / 2);
  const theta = clamp(p.coneDeg || 20, 5, 55) * DEG;
  let Rn = p.leRadius > 1e-6 ? p.leRadius : Math.min(0.45 * Rb, 0.28 * L);
  Rn = clamp(Rn, 1e-3, Math.min(L * 0.95, Rb / Math.max(Math.cos(theta), 0.2)));
  const xt = Rn * (1 - Math.sin(theta));
  const rt = Rn * Math.cos(theta);
  const tanT = Math.tan(theta);
  const Lgeom = xt + Math.max(0, Rb - rt) / Math.max(tanT, 1e-6);
  return (x) => {
    const xc = clamp(x, 0, L);
    if (xc <= 1e-12) return 0;
    if (xc <= xt + 1e-9 || xc <= xt) {
      const c = clamp(1 - xc / Math.max(Rn, 1e-6), -1, 1);
      return Rn * Math.sqrt(Math.max(0, 1 - c * c));
    }
    if (xc >= Lgeom) return Rb;
    return Math.max(0, rt + (xc - xt) * tanT);
  };
}

function buildCapsule(p: DesignParams): ClassVehicle {
  const L = Math.max(p.length, 0.2);
  const Rb = Math.max(0.05, p.span / 2);
  const theta = clamp(p.coneDeg || 20, 5, 55) * DEG;
  const rAt = capsuleRadius(p);
  const rBase = rAt(L);
  const xt = (() => {
    let Rn = p.leRadius > 1e-6 ? p.leRadius : Math.min(0.45 * Rb, 0.28 * L);
    Rn = clamp(Rn, 1e-3, Math.min(L * 0.95, Rb / Math.max(Math.cos(theta), 0.2)));
    return Rn * (1 - Math.sin(theta));
  })();
  const rt = rAt(Math.min(L, xt));
  const tanT = Math.tan(theta);
  const Lgeom = xt + Math.max(0, Rb - rt) / Math.max(tanT, 1e-6);
  const notes = [
    `Sphere-cone. Rn meets the ${(theta / DEG).toFixed(1)}° flank with a common tangent. Requested base diameter ${(2 * Rb).toFixed(3)} m, actual base ${(2 * rBase).toFixed(3)} m. Sref = π Rbase².`,
  ];
  if (Lgeom <= L + 1e-3) notes.push(`Cone reaches the base diameter at x/L = ${(Math.min(Lgeom, L) / L).toFixed(2)}${L > Lgeom + 0.02 * L ? "; the rest is a cylinder. No skirt flare." : "."}`);
  else notes.push(`Length ends before the cone reaches the requested diameter (would need ${(Lgeom).toFixed(2)} m). The base is the cone at this station — it is not flared out.`);
  const plates = makePlates(p, Math.max(rBase, 0.05), rAt);
  if (plates.length) notes.push("Flaps are plates on the skirt. Fin height is in base radii. Two is a pitch pair; more are equally spaced.");
  return revolve(p, rAt, Math.PI * rBase * rBase, notes, plates);
}

function missileRadius(p: DesignParams): { r: (x: number) => number; Ln: number; Rb: number; baseR: number } {
  const L = Math.max(p.length, 0.2);
  const Rb = Math.max(0.04, p.span / 2);
  const baseFrac = clamp(p.captureFrac > 0 ? p.captureFrac : 0.86, 0.35, 1);
  const baseR = Rb * baseFrac;
  const btFrac = baseFrac > 0.985 ? 0 : clamp(0.06 + 0.22 * (1 - baseFrac), 0.06, 0.22);
  let Ln = clamp(p.wedgeFrac || 0.3, 0.12, 0.6) * L;
  let Lbt = btFrac * L;
  if (Ln + Lbt > 0.9 * L) {
    const s = (0.9 * L) / (Ln + Lbt);
    Ln *= s;
    Lbt *= s;
  }
  const rho = (Rb * Rb + Ln * Ln) / (2 * Rb);
  let Rn = p.leBlunt === false ? 0 : Math.max(0, p.leRadius);
  Rn = clamp(Rn, 0, Math.min(0.4 * Rb, 0.3 * Ln));
  const sharp = (xs: number) => {
    const dx = Ln - xs;
    return Math.max(0, Math.sqrt(Math.max(0, rho * rho - dx * dx)) - (rho - Rb));
  };
  let xSc = Rn;
  let shift = 0;
  let tangV = 0;
  let useBlunt = false;
  if (Rn > 1e-5 * Rb) {
    const disc = (rho - Rn) * (rho - Rn) - (rho - Rb) * (rho - Rb);
    if (disc > 0) {
      const xsC = Ln - Math.sqrt(disc);
      const dist = Math.hypot(xsC - Ln, rho - Rb);
      const xNose = xsC - Rn;
      shift = -xNose;
      const tangSharp = Ln + (rho * (xsC - Ln)) / Math.max(dist, 1e-9);
      tangV = tangSharp + shift;
      xSc = xsC + shift;
      if (tangV > 1e-4 && tangV < Ln * 0.95 && xNose > -1e-6 && xNose < 0.5 * Ln) useBlunt = true;
    }
  }
  const r = (x: number) => {
    const xc = clamp(x, 0, L);
    if (xc <= Ln) {
      if (!useBlunt) return sharp(xc);
      if (xc <= tangV) return Math.sqrt(Math.max(0, Rn * Rn - (xc - xSc) * (xc - xSc)));
      const xs = xc - shift;
      if (xs >= Ln) return Rb;
      return sharp(Math.max(0, xs));
    }
    if (xc >= L - Lbt) {
      const u = (xc - (L - Lbt)) / Math.max(Lbt, 1e-6);
      return Rb + (baseR - Rb) * clamp(u, 0, 1);
    }
    return Rb;
  };
  return { r, Ln, Rb, baseR };
}

function buildMissile(p: DesignParams): ClassVehicle {
  const L = Math.max(p.length, 0.2);
  const prof = missileRadius(p);
  const notes = [
    `Missile OML: tangent ogive to x/L = ${(prof.Ln / L).toFixed(2)}, cylinder diameter ${(2 * prof.Rb).toFixed(3)} m, boat-tail to ${(2 * prof.baseR).toFixed(3)} m. Sref = πR².`,
    p.leBlunt === false || !(p.leRadius > 0)
      ? "Nose is sharp. A nose radius blends a sphere into the ogive with a common tangent."
      : `Nose sphere Rn = ${(Math.min(p.leRadius, 0.4 * prof.Rb) * 1000).toFixed(1)} mm, tangent to the ogive.`,
  ];
  const plates = makePlates(p, prof.Rb, prof.r);
  return revolve(p, prof.r, Math.PI * prof.Rb * prof.Rb, notes, plates);
}

function stackRadius(p: DesignParams): (x: number) => number {
  const L = Math.max(p.length, 0.2);
  const n = clamp(Math.round(p.nRamps || 2), 2, 4);
  const Rb = Math.max(0.05, p.span / 2);
  const Rs: number[] = [];
  for (let i = 0; i < n; i++) Rs.push(Rb * (n === 1 ? 1 : 0.46 + 0.54 * (i / (n - 1))));
  const noseL = clamp(Math.min(0.16 * L, 2.4 * Rs[0]), 0.06 * L, 0.28 * L);
  const nozzleL = 0.055 * L;
  const transL = 0.04 * L;
  const remain = Math.max(0.2 * L, L - noseL - nozzleL - transL * (n - 1));
  const stageL = remain / n;
  const cuts: { x0: number; x1: number; r0: number; r1: number }[] = [];
  let x = 0;
  cuts.push({ x0: 0, x1: noseL, r0: 0, r1: Rs[0] });
  x = noseL;
  for (let i = 0; i < n; i++) {
    const xC = x + stageL;
    cuts.push({ x0: x, x1: xC, r0: Rs[i], r1: Rs[i] });
    x = xC;
    if (i < n - 1) {
      cuts.push({ x0: x, x1: x + transL, r0: Rs[i], r1: Rs[i + 1] });
      x += transL;
    }
  }
  const rNoz = Rs[n - 1] * 0.78;
  cuts.push({ x0: x, x1: L, r0: Rs[n - 1], r1: rNoz });
  return (xq: number) => {
    const xc = clamp(xq, 0, L);
    for (const c of cuts) {
      if (xc <= c.x1 + 1e-9 || c === cuts[cuts.length - 1]) {
        const u = clamp((xc - c.x0) / Math.max(c.x1 - c.x0, 1e-9), 0, 1);
        return c.r0 + (c.r1 - c.r0) * u;
      }
    }
    return Rs[n - 1];
  };
}

function buildStack(p: DesignParams): ClassVehicle {
  const L = Math.max(p.length, 0.2);
  const n = clamp(Math.round(p.nRamps || 2), 2, 4);
  const Rb = Math.max(0.05, p.span / 2);
  const rAt = stackRadius(p);
  const notes = [
    `${n}-stage stack. Conical nose, cylindrical stages (upper smaller), conical interstages, nozzle to ${(0.78 * 2 * Rb).toFixed(3)} m. No separation, no cavity. Sref = π Rbase².`,
    "Fins, when the height is above zero, are plates on the aft stage.",
  ];
  const plates = makePlates(p, Rb, rAt);
  return revolve(p, rAt, Math.PI * Rb * Rb, notes, plates);
}

function sharpHalfSpan(x: number, p: DesignParams): number {
  const L = Math.max(p.length, 0.2);
  const xi = clamp(x / L, 0, 1);
  const b = Math.max(p.span, 0.08) / 2;
  const pow = clamp(p.planformPower || 1, 0.45, 2.4);
  switch (p.planform) {
    case "power":
      return b * Math.pow(xi, pow);
    case "spatular":
      return b * (0.32 + 0.68 * Math.pow(xi, 0.6));
    case "rect":
      return b * Math.min(1, xi / 0.16);
    case "double": {
      const xk = clamp(p.wedgeFrac || 0.45, 0.25, 0.7);
      return xi <= xk ? b * 0.62 * (xi / xk) : b * (0.62 + 0.38 * ((xi - xk) / (1 - xk)));
    }
    default:
      return b * xi;
  }
}

function halfSpanAt(x: number, p: DesignParams): number {
  const y = sharpHalfSpan(x, p);
  const L = Math.max(p.length, 0.2);
  const rn = p.leBlunt === false ? 0 : Math.max(p.leRadius, 0.004 * L);
  // Circular nose, center on the centerline at x = rn: y = sqrt(2 rn x − x²).
  // Used only when the sharp planform is already ≥ rn wide at x = rn, so the
  // arc meets it from inside. Both pieces increase, so span never falls.
  // The old bulb sqrt(1 − ξ/0.12) narrowed, then the planform widened — a shoulder.
  if (!(rn > 0) || x <= 0 || x >= rn) return y;
  if (sharpHalfSpan(rn, p) < rn * 0.999) return y;
  const yc = Math.sqrt(Math.max(0, 2 * rn * x - x * x));
  return Math.max(y, yc);
}

function sectionThick(x: number, p: DesignParams): number {
  const L = Math.max(p.length, 0.2);
  const xi = clamp(x / L, 0, 1);
  const h = Math.max(p.height, 0.03);
  const cam = 1 + 0.4 * clamp(p.camber, -0.4, 0.6);
  const env = Math.max(0.16, Math.sin(Math.PI * Math.pow(xi, 0.72)) * (1 - 0.35 * xi) + 0.12);
  return h * cam * env;
}

function buildBoostglide(p: DesignParams): ClassVehicle {
  const L = Math.max(p.length, 0.2);
  const ni = clamp(Math.round(p.nx) || 28, 14, 64);
  const nj = clamp(Math.round(p.ny) || 18, 10, 48);
  const b = new MeshBuilder(L);
  const dih = Math.tan(clamp(p.dihedralDeg || 0, -15, 25) * DEG);
  const notes = [
    `Boost-glide loft. Planform ${p.planform}, span ${p.span.toFixed(2)} m, height ${p.height.toFixed(2)} m. Direct parametric OML — the inverse-design shock is not enforced.`,
    "Section is a lee-side sheet with an elliptic belly. A pointed nose is one pole, not a spanwise row. Nose-high incidence makes the belly windward at α = 0. Tips pinch. Sref is the projected planform.",
  ];
  if (p.halfModel) notes.push("Half-model is not applied — the full planform is meshed.");
  if (Math.abs(p.elevonDeg || 0) > 0.05) {
    notes.push(`Elevon ${p.elevonDeg.toFixed(1)}° shears the surface aft of 78% length (trailing edge down is positive). Same sign as the waverider elevon.`);
  }
  const upper: number[][] = [];
  const lower: number[][] = [];
  const xyzU = new Float64Array(ni * nj * 3);
  const xyzL = new Float64Array(ni * nj * 3);
  let sRef = 0;
  let prevY = halfSpanAt(0, p);
  for (let i = 0; i < ni; i++) {
    const x = (L * i) / (ni - 1);
    const yRaw = halfSpanAt(x, p);
    // A pointed planform has zero span at the nose. A spanwise row there is a fan of
    // slivers (every "quad" has two corners on top of each other). Collapse it to one pole.
    const pole = yRaw <= 1e-4 * L;
    const yHalf = pole ? 0 : yRaw;
    if (i > 0) sRef += (prevY + yHalf) * (L / (ni - 1));
    prevY = yHalf;
    const thick = pole ? 0 : sectionThick(x, p);
    const xi = x / L;
    const camb = p.camber * p.height * 4 * xi * (1 - xi);
    // Nose sits high and the sheet slopes down aft. Volume hangs below it,
    // so the belly outward normal has an upstream component at α = 0.
    const incidence = 0.22 * p.height * (1 - xi);
    const elev = x > 0.78 * L ? -(x - 0.78 * L) * Math.tan(clamp(p.elevonDeg || 0, -18, 18) * DEG) : 0;
    const rowU: number[] = [];
    const rowL: number[] = [];
    for (let j = 0; j < nj; j++) {
      const t = j / (nj - 1);
      const y = -yHalf + 2 * yHalf * t;
      const eta = yHalf > 1e-8 ? clamp(y / yHalf, -1, 1) : 0;
      const zSheet = incidence + camb + Math.abs(y) * dih;
      const drop = thick * Math.sqrt(Math.max(0, 1 - eta * eta));
      const zU = zSheet + elev;
      const zL = zSheet - drop + elev;
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
