/**
 * Elliptical body, inlet, and protuberance engineering increments.
 *
 * Elliptical cross-section:
 *  - Area A = π a b; perimeter via Ramanujan's approximation
 *    P ≈ π [3(a+b) − √((3a+b)(a+3b))]  (Ramanujan 1914; error < 0.5% for moderate eccentricity).
 *  - Equivalent radius for interference / fin attach: r_eq = √(a b).
 *  - Crossflow drag of an ellipse at incidence φ to the minor axis uses the Hoerner-style
 *    engineering fit Cd_c(e,φ) ≈ Cd_cyl * (1 + k e² cos²φ) with k≈0.5 (NOT a published formula —
 *    flagged medium/low confidence). Slender-body potential CN uses base area as for circular.
 *
 * Inlet additive drag:
 *  - Farriols / standard inlet bookkeeping: C_D,add ≈ (2 / (γ M²)) * (1 − MFR) * (A_c / Sref)
 *    as a first-order estimate of the streamtube momentum deficit when MFR < 1
 *    (engineering form used in preliminary design notes; cf. Seddon & Goldsmith, Intake Aerodynamics,
 *    for the full control-volume treatment — we do NOT implement the full CV here).
 *  - Covered inlet: frontal Cd ≈ 1.0 * (A_c / Sref) (bluff face).
 *  - Ram drag is NOT double-counted into CA when the propulsion bookkeeping is external; we add
 *    only the additive/spillage contribution and a small CN from the capture-plane normal (ramp).
 *
 * Protuberance:
 *  - Cylinder (transverse): Cd ≈ 1.2, S = length * height (projected).
 *  - Wedge: Cd ≈ 2 sin²θ with θ = atan(h/(L/2)).
 *  - Hemisphere: Cd ≈ 0.4 (front) on π h².
 *  - Block: Cd ≈ 1.05 on width*height.
 *  - Fairing: Cd ≈ 0.1 on width*height.
 *  - Height-to-boundary-layer factor: if δ_bl ~ 0.37 L_x / Re_x^{0.2}, then
 *    f = clamp(h / max(δ,h), 0.3, 1) scales the dynamic pressure seen by the protuberance
 *    (engineering; Hoerner Fluid-Dynamic Drag ch. XIII style).
 * All increments are clearly engineering-level and carry method tags.
 */
import type { BodySpec, EllipticStation, InletSpec, ProtuberanceSpec } from "./types.ts";
import type { BodyProfile } from "./geometry.ts";
import type { Flow } from "./flow.ts";
import { DEG, clamp } from "./util.ts";

/** Ramanujan perimeter of an ellipse with semi-axes a, b. */
export function ellipsePerimeter(a: number, b: number): number {
  a = Math.abs(a);
  b = Math.abs(b);
  if (a < 1e-16 && b < 1e-16) return 0;
  return Math.PI * (3 * (a + b) - Math.sqrt(Math.max((3 * a + b) * (a + 3 * b), 0)));
}

export function ellipseArea(a: number, b: number): number {
  return Math.PI * Math.abs(a) * Math.abs(b);
}

/** Build a circular-equivalent BodyProfile from elliptic stations (r_eq = √(ab)). */
export function ellipticToProfile(stations: EllipticStation[]): BodyProfile {
  const sorted = [...stations].sort((p, q) => p.x - q.x);
  const x0 = sorted[0]?.x ?? 0;
  const x = sorted.map((s) => s.x - x0);
  const r = sorted.map((s) => Math.sqrt(Math.max(s.a * s.b, 0)));
  let rMax = 0;
  for (const v of r) rMax = Math.max(rMax, v);
  let xNoseEnd = x[x.length - 1] ?? 0;
  for (let i = 0; i < x.length; i++) {
    if (r[i] >= rMax * 0.999) {
      xNoseEnd = x[i];
      break;
    }
  }
  return {
    x,
    r,
    length: x[x.length - 1] ?? 0,
    rMax,
    xNoseEnd,
    baseRadius: r[r.length - 1] ?? 0,
    noseBluntRadius: 0,
    warnings: ["elliptic body: using equivalent radius √(a b) for the circular-methods path"],
  };
}

export interface EllipticMetrics {
  volume: number;
  planform: number; // top-view planform ≈ 2 ∫ a dx (width)
  sidePlanform: number; // side-view ≈ 2 ∫ b dx
  wetted: number;
  baseArea: number;
  xPlanform: number;
  xVolume: number;
  eccentricityMax: number;
}

export function ellipticIntegrals(stations: EllipticStation[]): EllipticMetrics {
  const st = [...stations].sort((p, q) => p.x - q.x);
  let vol = 0,
    pl = 0,
    side = 0,
    wet = 0,
    mpl = 0,
    mvol = 0,
    ecc = 0;
  for (let i = 0; i < st.length - 1; i++) {
    const dx = st[i + 1].x - st[i].x;
    if (dx <= 0) continue;
    const a0 = st[i].a,
      a1 = st[i + 1].a,
      b0 = st[i].b,
      b1 = st[i + 1].b;
    const am = 0.5 * (a0 + a1),
      bm = 0.5 * (b0 + b1);
    const xm = 0.5 * (st[i].x + st[i + 1].x);
    const v = ellipseArea(am, bm) * dx; // prism approximation
    vol += v;
    mvol += v * xm;
    const pW = 2 * am * dx;
    pl += pW;
    mpl += pW * xm;
    side += 2 * bm * dx;
    wet += ellipsePerimeter(am, bm) * dx;
    const aa = Math.max(am, bm),
      bb = Math.min(am, bm);
    if (aa > 1e-12) ecc = Math.max(ecc, Math.sqrt(1 - (bb * bb) / (aa * aa)));
  }
  const last = st[st.length - 1];
  return {
    volume: vol,
    planform: pl,
    sidePlanform: side,
    wetted: wet,
    baseArea: last ? ellipseArea(last.a, last.b) : 0,
    xPlanform: pl > 0 ? mpl / pl : 0,
    xVolume: vol > 0 ? mvol / vol : 0,
    eccentricityMax: ecc,
  };
}

/** Crossflow Cd multiplier for an ellipse vs a circular cylinder (engineering). */
export function ellipseCrossflowFactor(a: number, b: number, psi: number): number {
  // psi = crossflow azimuth (0 = lift in +z / "up"); a = half-width (y), b = half-height (z)
  const aa = Math.max(a, 1e-12),
    bb = Math.max(b, 1e-12);
  const e2 = 1 - Math.min(aa, bb) ** 2 / Math.max(aa, bb) ** 2;
  // more drag when crossflow faces the flatter side
  const flatness = Math.cos(psi) ** 2 * (bb / aa) + Math.sin(psi) ** 2 * (aa / bb);
  return clamp(0.85 + 0.3 * e2 + 0.15 * (flatness - 1), 0.7, 1.6);
}

export interface ExtraIncrement {
  CA: number;
  CN: number;
  CY: number;
  Cm: number;
  Cn: number;
  Cl: number;
  tag: string;
  notes: string[];
}

export function inletIncrements(inlets: InletSpec[] | undefined, flow: Flow, sRef: number, lRef: number, xcg: number): ExtraIncrement {
  const out: ExtraIncrement = { CA: 0, CN: 0, CY: 0, Cm: 0, Cn: 0, Cl: 0, tag: "inlet additive/spillage (engineering)", notes: [] };
  if (!inlets?.length) return out;
  const M = flow.mach;
  const g = flow.gamma;
  out.notes.push("Inlet additive drag is a first-order MFR-based estimate — not a full Seddon–Goldsmith control volume");
  for (const inl of inlets) {
    const Ac = Math.max(inl.captureArea, 0);
    const ratio = Ac / Math.max(sRef, 1e-12);
    if (inl.covered) {
      out.CA += 1.0 * ratio;
      continue;
    }
    const mfr = inl.mfr ?? 0.9;
    // Additive / spillage drag ~ (2/(γ M²)) (1 − MFR) (Ac/Sref) for M > 0.3
    if (M > 0.3) {
      const cda = ((2 / (g * M * M)) * Math.max(1 - mfr, 0) * ratio);
      out.CA += cda;
    }
    // Small normal force from an external ramp (pressure on ramp face)
    const ramp = (inl.rampDeg ?? 0) * DEG;
    if (ramp > 1e-6 && M > 1.05) {
      const cn = 2 * Math.sin(ramp) ** 2 * ratio * 0.5; // rough Newtonian on ramp area ~ Ac/sin?
      const phi = (inl.phi ?? 0) * DEG;
      out.CN += cn * Math.cos(phi);
      out.CY += cn * Math.sin(phi);
      const arm = (xcg - inl.x) / lRef;
      out.Cm += cn * Math.cos(phi) * arm;
      out.Cn += cn * Math.sin(phi) * arm;
    }
  }
  return out;
}

export function protuberanceIncrements(
  prots: ProtuberanceSpec[] | undefined,
  flow: Flow,
  sRef: number,
  lRef: number,
  xcg: number,
  bodyLength: number,
): ExtraIncrement {
  const out: ExtraIncrement = { CA: 0, CN: 0, CY: 0, Cm: 0, Cn: 0, Cl: 0, tag: "protuberance drag (Hoerner-style engineering)", notes: [] };
  if (!prots?.length) return out;
  const ReL = Math.max(flow.renPerM * Math.max(bodyLength, 1e-3), 1e3);
  for (const p of prots) {
    const n = Math.max(1, p.count ?? 1);
    // turbulent flat-plate δ/x ≈ 0.37 Re_x^{-1/5}
    const x = Math.max(p.x, 1e-3);
    const Rex = Math.max(flow.renPerM * x, 1e3);
    const delta = 0.37 * x * Math.pow(Rex, -0.2);
    const fBl = clamp(p.height / Math.max(delta, p.height, 1e-6), 0.3, 1);
    let Cd = 1.0;
    let S = p.width * p.height;
    switch (p.kind) {
      case "cylinder":
        Cd = 1.2;
        S = p.length * p.height;
        break;
      case "wedge": {
        const th = Math.atan2(p.height, Math.max(p.length / 2, 1e-6));
        Cd = 2 * Math.sin(th) ** 2;
        S = p.width * p.height;
        break;
      }
      case "hemisphere":
        Cd = 0.4;
        S = Math.PI * p.height * p.height;
        break;
      case "fairing":
        Cd = 0.1;
        S = p.width * p.height;
        break;
      case "block":
      default:
        Cd = 1.05;
        S = p.width * p.height;
        break;
    }
    // compressibility fade of bluff Cd above M~0.8 (engineering)
    const cdM = Cd * (1 - 0.25 * clamp((flow.mach - 0.8) / 1.2, 0, 1));
    const dCA = (n * cdM * fBl * S) / Math.max(sRef, 1e-12);
    out.CA += dCA;
    // tiny normal-force if offset vertically — ignored unless phi0 set with height
    void ReL;
    void xcg;
    void lRef;
  }
  out.notes.push("Protuberance Cd from Hoerner-style bluff-body values; BL height factor is engineering");
  return out;
}

/** True if the body spec should use the elliptic path. */
export function isElliptic(body?: BodySpec): boolean {
  return !!body?.elliptic && body.elliptic.length >= 2;
}
