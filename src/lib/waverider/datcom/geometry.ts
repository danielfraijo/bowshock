/**
 * Geometry generation + derived quantities for bodies of revolution and
 * trapezoidal lifting-surface panels.
 *
 * Nose shape equations: standard (e.g. Crowell, "The Descriptive Geometry of
 * Nose Cones", 1996; Haack series per Sears/Haack, Eggers/Resnikoff/Dennis
 * NACA Rep. 1382 for the LV-Haack C=1/3 and von Karman C=0 cases).
 * Spherical blunting: sphere tangent to the profile (normal-intersects-axis
 * construction).
 */
import type { BodyParametric, BodySpec, BodyStation, FinSet, GeometrySummary, NoseKind, VehicleConfig } from "./types.ts";
import { DEG, clamp, simpson } from "./util.ts";

export interface BodyProfile {
  x: number[];
  r: number[];
  length: number;
  rMax: number;
  /** x where r first attains ~rMax (end of nose region) */
  xNoseEnd: number;
  baseRadius: number;
  /** body blunting radius applied at the tip (0 = sharp) */
  noseBluntRadius: number;
  /** radius of curvature of nose used for cap, for info */
  warnings: string[];
}

/** Sharp-nose radius at fraction xi=x/L of a nose of base radius R. */
export function noseRadius(kind: NoseKind, xi: number, R: number, L: number, power = 0.5): number {
  xi = clamp(xi, 0, 1);
  switch (kind) {
    case "cone":
      return R * xi;
    case "power":
      return R * Math.pow(xi, power);
    case "karman":
    case "haack": {
      const C = kind === "haack" ? 1 / 3 : 0;
      const th = Math.acos(1 - 2 * xi);
      const v = th - Math.sin(2 * th) / 2 + C * Math.pow(Math.sin(th), 3);
      return (R / Math.sqrt(Math.PI)) * Math.sqrt(Math.max(v, 0));
    }
    case "ogive":
    case "spherical":
    default: {
      // tangent ogive: rho = (R^2 + L^2) / (2R)
      const rho = (R * R + L * L) / (2 * R);
      const x = xi * L;
      return Math.sqrt(Math.max(rho * rho - (L - x) * (L - x), 0)) + R - rho;
    }
  }
}

function sampleParametric(p: BodyParametric): { x: number[]; r: number[]; blunt: number } {
  const xs: number[] = [];
  const rs: number[] = [];
  const R = p.noseDiameter / 2;
  const NN = 80;
  const kind = p.noseKind;
  const blunt = p.noseBluntRadius ?? (kind === "spherical" ? 0.15 * R : 0);
  for (let i = 0; i <= NN; i++) {
    // cosine-ish clustering at the tip
    const xi = Math.pow(i / NN, 1.6);
    xs.push(xi * p.noseLength);
    rs.push(noseRadius(kind, xi, R, p.noseLength, p.power ?? 0.5));
  }
  const Rc = (p.centerDiameter ?? p.noseDiameter) / 2;
  let xEnd = p.noseLength;
  if (Math.abs(Rc - R) > 1e-12 && p.centerLength > 0) {
    // step from nose base to center radius: treat as straight (cone) join over the centre section
    xs.push(xEnd + p.centerLength);
    rs.push(Rc);
    xEnd += p.centerLength;
  } else if (p.centerLength > 0) {
    xs.push(xEnd + p.centerLength);
    rs.push(R);
    xEnd += p.centerLength;
  }
  if (p.aftLength > 0) {
    const Ra = (p.aftDiameter ?? p.noseDiameter) / 2;
    const NA = 24;
    const a0 = rs[rs.length - 1];
    for (let i = 1; i <= NA; i++) {
      const t = i / NA;
      let rr: number;
      if (p.aftKind === "ogive") {
        // tangent ogive about the aft section: slope matches the cylinder at x_start (tangent), meets Ra at the end
        const Lh = p.aftLength;
        const dR = Ra - a0;
        // tangent-ogive segment with tangency at the cylinder end: r = a0 + dR*(1-(1-t)^2) for boattail (convex), 'flare' uses same form
        rr = a0 + dR * (1 - (1 - t) * (1 - t));
        void Lh;
      } else rr = a0 + (Ra - a0) * t;
      xs.push(xEnd + t * p.aftLength);
      rs.push(rr);
    }
  }
  return { x: xs, r: rs, blunt };
}

function densify(st: BodyStation[], per = 6): { x: number[]; r: number[] } {
  const xs: number[] = [];
  const rs: number[] = [];
  for (let i = 0; i < st.length - 1; i++) {
    for (let k = 0; k < per; k++) {
      const t = k / per;
      xs.push(st[i].x + (st[i + 1].x - st[i].x) * t);
      rs.push(st[i].r + (st[i + 1].r - st[i].r) * t);
    }
  }
  xs.push(st[st.length - 1].x);
  rs.push(st[st.length - 1].r);
  return { x: xs, r: rs };
}

/** Replace the tip of a sampled profile with a tangent spherical cap of radius rn. */
function applyBlunting(x: number[], r: number[], rn: number): { x: number[]; r: number[]; ok: boolean } {
  if (rn <= 0 || x.length < 3) return { x, r, ok: true };
  // slope-based search along the profile for tangent circle of radius rn
  const n = x.length;
  const x0 = x[0];
  let k = -1;
  for (let i = 1; i < n - 1; i++) {
    const dx = x[i + 1] - x[i - 1];
    const dr = r[i + 1] - r[i - 1];
    if (dx <= 0) continue;
    const slope = dr / dx;
    const rho = r[i] * Math.sqrt(1 + slope * slope);
    if (rho >= rn) {
      k = i;
      break;
    }
  }
  if (k < 0) return { x, r, ok: false };
  const dx = x[k + 1] - x[k - 1];
  const slope = (r[k + 1] - r[k - 1]) / dx;
  const xc = x[k] + r[k] * slope;
  const th0 = Math.atan2(r[k], x[k] - xc);
  const nx: number[] = [];
  const nr: number[] = [];
  const NS = 16;
  // sphere from the tip (angle pi) to the tangent angle. Station coordinates are NOT shifted: x=0 stays the
  // sharp-apex origin, so fin / cg stations keep their meaning; the blunt tip sits at xc - rn >= 0.
  const shift = 0;
  void x0;
  for (let i = 0; i <= NS; i++) {
    const th = Math.PI - ((Math.PI - th0) * i) / NS;
    nx.push(xc + rn * Math.cos(th) + shift);
    nr.push(Math.max(0, rn * Math.sin(th)));
  }
  for (let i = k + 1; i < n; i++) {
    nx.push(x[i] + shift);
    nr.push(r[i]);
  }
  return { x: nx, r: nr, ok: true };
}

export function buildBody(spec: BodySpec): BodyProfile {
  const warnings: string[] = [];
  let x: number[];
  let r: number[];
  let blunt = spec.noseBluntRadius ?? 0;
  if (spec.stations && spec.stations.length >= 2) {
    const st = [...spec.stations].sort((a, b) => a.x - b.x);
    const d = densify(st);
    x = d.x.map((v) => v - st[0].x);
    r = d.r;
  } else if (spec.parametric) {
    const s = sampleParametric(spec.parametric);
    x = s.x;
    r = s.r;
    blunt = spec.noseBluntRadius ?? s.blunt;
  } else {
    throw new Error("body needs stations or parametric definition");
  }
  if (blunt > 0) {
    const b = applyBlunting(x, r, blunt);
    if (!b.ok) warnings.push(`nose bluntness radius ${blunt} too large for the profile — ignored`);
    else {
      x = b.x;
      r = b.r;
    }
  }
  const length = x[x.length - 1] - x[0]; // actual body length (tip of the blunt cap to base)
  let rMax = 0;
  for (const v of r) rMax = Math.max(rMax, v);
  let xNoseEnd = x[x.length - 1];
  for (let i = 0; i < x.length; i++) {
    if (r[i] >= rMax * 0.9995) {
      xNoseEnd = x[i];
      break;
    }
  }
  return { x, r, length, rMax, xNoseEnd, baseRadius: r[r.length - 1], noseBluntRadius: blunt, warnings };
}

export function radiusAt(b: BodyProfile, x: number): number {
  const n = b.x.length;
  if (x <= b.x[0]) return b.r[0];
  if (x >= b.x[n - 1]) return b.r[n - 1];
  let lo = 0;
  let hi = n - 1;
  while (hi - lo > 1) {
    const m = (lo + hi) >> 1;
    if (b.x[m] <= x) lo = m;
    else hi = m;
  }
  const t = (x - b.x[lo]) / Math.max(b.x[hi] - b.x[lo], 1e-14);
  return b.r[lo] + t * (b.r[hi] - b.r[lo]);
}

export interface BodyIntegrals {
  volume: number;
  planform: number;
  wetted: number;
  baseArea: number;
  /** centroid of planform area from nose */
  xPlanform: number;
  /** centroid of volume */
  xVolume: number;
}
export function bodyIntegrals(b: BodyProfile): BodyIntegrals {
  let vol = 0;
  let pl = 0;
  let wet = 0;
  let mpl = 0;
  let mvol = 0;
  for (let i = 0; i < b.x.length - 1; i++) {
    const dx = b.x[i + 1] - b.x[i];
    if (dx <= 0) continue;
    const r0 = b.r[i];
    const r1 = b.r[i + 1];
    const xm = 0.5 * (b.x[i] + b.x[i + 1]);
    // frustum exact
    const v = (Math.PI * dx * (r0 * r0 + r0 * r1 + r1 * r1)) / 3;
    vol += v;
    mvol += v * xm;
    const p = (r0 + r1) * dx;
    pl += p;
    mpl += p * xm;
    wet += Math.PI * (r0 + r1) * Math.sqrt(dx * dx + (r1 - r0) * (r1 - r0));
  }
  return {
    volume: vol,
    planform: pl,
    wetted: wet,
    baseArea: Math.PI * b.baseRadius * b.baseRadius,
    xPlanform: pl > 0 ? mpl / pl : 0,
    xVolume: vol > 0 ? mvol / vol : 0,
  };
}

/* ---------------------------------------------------------------- fins */

export interface FinGeom {
  set: FinSet;
  rRoot: number;
  exposedSpan: number;
  /** exposed panel area (one panel) */
  area: number;
  /** aspect ratio of two exposed panels joined: (2 b_e)^2 / (2 S_e) */
  aspect: number;
  taper: number;
  tanLE: number;
  tanQC: number;
  tanHC: number;
  tanTE: number;
  mac: number;
  yMac: number;
  xMacLE: number;
  /** area-weighted spanwise centroid of the panel measured from the body axis */
  yBar: number;
  /** chord at span fraction eta in [0,1] along the exposed span */
  chordAt: (eta: number) => number;
  xLEAt: (eta: number) => number;
  /** tau = r/s (body radius over semispan from axis) */
  tau: number;
  panelPhi: number[];
  /** Integral of c(y) y^2 dy over the panel (y from the axis) for roll damping */
  cY2: number;
  warnings: string[];
}

export function defaultPhi(set: FinSet): number[] {
  if (set.phiF && set.phiF.length >= set.nPanels) return set.phiF.slice(0, set.nPanels);
  const n = set.nPanels;
  if (set.role === "vtail" || set.role === "fin" && n === 1) return [0];
  if (set.role === "wing" || set.role === "htail") {
    if (n === 2) return [90, 270];
  }
  if (n === 4) return [45, 135, 225, 315];
  if (n === 2) return [90, 270];
  return Array.from({ length: n }, (_, i) => (360 / n) * i);
}

export function finGeom(set: FinSet, body: BodyProfile | null): FinGeom {
  const warnings: string[] = [];
  const xMid = set.xLE + 0.5 * set.rootChord;
  const rRoot = body ? radiusAt(body, xMid) : 0;
  const be = set.semispan - rRoot;
  if (be <= 0) warnings.push(`${set.name}: semispan ${set.semispan} <= body radius ${rRoot.toFixed(4)} at the root — panel is buried`);
  const bE = Math.max(be, 1e-6);
  const cr = set.rootChord;
  const ct = set.tipChord;
  const tanLE = Math.tan(set.sweepLE * DEG);
  const chordAt = (eta: number) => cr + (ct - cr) * eta;
  const xLEAt = (eta: number) => set.xLE + tanLE * bE * eta;
  const area = (bE * (cr + ct)) / 2;
  const aspect = (2 * bE * bE) / Math.max(area, 1e-12);
  const taper = cr > 0 ? ct / cr : 0;
  let sc = 0;
  let sc2 = 0;
  let scx = 0;
  let scy = 0;
  let cY2 = 0;
  const N = 48;
  for (let i = 0; i < N; i++) {
    const eta = (i + 0.5) / N;
    const c = chordAt(eta);
    const y = rRoot + bE * eta;
    sc += c;
    sc2 += c * c;
    scx += xLEAt(eta) * c;
    scy += y * c;
    cY2 += c * y * y;
  }
  const dy = bE / N;
  const mac = sc2 / Math.max(sc, 1e-12);
  const xMacLE = scx / Math.max(sc, 1e-12);
  const yBar = scy / Math.max(sc, 1e-12);
  // y at MAC (chord == mac)
  const etaMac = cr !== ct ? clamp((mac - cr) / (ct - cr), 0, 1) : 0.5;
  return {
    set,
    rRoot,
    exposedSpan: bE,
    area,
    aspect,
    taper,
    tanLE,
    tanQC: tanLE - (0.25 * (cr - ct)) / bE,
    tanHC: tanLE - (0.5 * (cr - ct)) / bE,
    tanTE: tanLE - (cr - ct) / bE,
    mac,
    yMac: rRoot + bE * etaMac,
    xMacLE,
    yBar,
    chordAt,
    xLEAt,
    tau: clamp(rRoot / Math.max(set.semispan, 1e-9), 0, 0.99),
    panelPhi: defaultPhi(set),
    cY2: cY2 * dy,
    warnings,
  };
}

/** Section slope integral I = integral (dz/dxi)^2 dxi per surface, z in chord fractions (half thickness). */
export function sectionSlopeIntegral(set: FinSet): number {
  const t = set.thickness;
  switch (set.section) {
    case "flat":
      return 0;
    case "wedge":
      // symmetric wedge with blunt trailing edge: each surface slopes (t/2)/c over the whole chord
      return (t * t) / 4;
    case "biconvex":
      // z = 2 (t/2)... half-thickness parabola: z/c = 2(t/c) xi(1-xi) -> I = 4 t^2 / 3
      return (4 * t * t) / 3;
    case "naca4": {
      const tt = nacaThickness(set.naca) ?? t;
      const f = (xi: number) => {
        // d/dxi of NACA 4-digit half-thickness (closed-trailing-edge variant)
        const x = Math.max(xi, 1e-4);
        const d = (tt / 0.2) * (0.2969 * 0.5 / Math.sqrt(x) - 0.126 - 2 * 0.3516 * x + 3 * 0.2843 * x * x - 4 * 0.1036 * x * x * x);
        return d * d;
      };
      return simpson(f, 0.002, 1, 200) + 0.002 * f(0.002);
    }
    case "hex":
    default: {
      // hexagon: linear rise to xmax over (1 - flat)/2... symmetric about mid: leading ramp (1-flat)/2, flat, trailing ramp (1-flat)/2
      const fl = clamp(set.flatFraction ?? 0, 0, 0.9);
      const ramp = (1 - fl) / 2;
      const slope = t / 2 / ramp; // half thickness t/2 over ramp length
      return 2 * ramp * slope * slope;
    }
  }
}

/** Parse a NACA 4-digit designation like '0012' or '2412' -> thickness ratio. */
export function nacaThickness(d?: string): number | null {
  if (!d) return null;
  const m = d.replace(/[^0-9]/g, "");
  if (m.length < 4) return null;
  return parseInt(m.slice(-2), 10) / 100;
}

/* -------------------------------------------------------- summary */

export interface ResolvedRefs {
  sRef: number;
  lRef: number;
  latRef: number;
  xcg: number;
}

export function resolveRefs(cfg: VehicleConfig, body: BodyProfile | null, fins: FinGeom[]): ResolvedRefs {
  const r = cfg.ref;
  let sRef = r.sRef;
  let lRef = r.lRef;
  if (sRef === undefined) {
    if (body) sRef = Math.PI * body.rMax * body.rMax;
    else if (fins.length) sRef = Math.max(...fins.map((f) => f.area));
    else sRef = 1;
  }
  if (lRef === undefined) {
    if (body) lRef = 2 * body.rMax;
    else if (fins.length) lRef = fins[0].mac;
    else lRef = 1;
  }
  return { sRef, lRef, latRef: r.latRef ?? lRef, xcg: r.xcg };
}

export function summarizeGeometry(cfg: VehicleConfig, body: BodyProfile | null, fins: FinGeom[]): GeometrySummary {
  const refs = resolveRefs(cfg, body, fins);
  const bi = body ? bodyIntegrals(body) : null;
  const warnings: string[] = [];
  if (body) warnings.push(...body.warnings);
  fins.forEach((f) => warnings.push(...f.warnings));
  const L = body ? body.length : Math.max(0, ...fins.map((f) => f.set.xLE + f.set.rootChord));
  return {
    length: L,
    maxDiameter: body ? 2 * body.rMax : 0,
    sRef: refs.sRef,
    lRef: refs.lRef,
    latRef: refs.latRef,
    xcg: refs.xcg,
    baseArea: bi?.baseArea ?? 0,
    planformArea: bi?.planform ?? 0,
    volume: bi?.volume ?? 0,
    wettedArea: bi?.wetted ?? 0,
    noseFineness: body ? body.xNoseEnd / Math.max(2 * body.rMax, 1e-9) : 0,
    bodyFineness: body ? body.length / Math.max(2 * body.rMax, 1e-9) : 0,
    sets: fins.map((f) => ({
      name: f.set.name,
      role: f.set.role,
      nPanels: f.set.nPanels,
      exposedArea: f.area,
      exposedSpan: f.exposedSpan,
      aspectRatio: f.aspect,
      taper: f.taper,
      sweepLE: f.set.sweepLE,
      sweepQC: Math.atan(f.tanQC) / DEG,
      sweepHC: Math.atan(f.tanHC) / DEG,
      mac: f.mac,
      xMacLE: f.xMacLE,
      xAC: f.xMacLE + 0.25 * f.mac,
      bodyRadiusAtRoot: f.rRoot,
      tauRoverS: f.tau,
    })),
    warnings,
  };
}
