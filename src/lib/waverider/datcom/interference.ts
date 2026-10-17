/**
 * Body / fin interference.
 *
 * K_W(B), K_B(W): ratios of the lift on the exposed panels in the presence of the body (W(B)) and of the
 * lift carried onto the body by the panels (B(W)) to the lift of the exposed panels alone
 * (Pitts, Nielsen & Kaattari, NACA Rep. 1307, 1957 — slender-body theory, Eq. 14; strip theory with
 * body upwash, Eq. 15-17).
 *  - exact slender-body identity K_W(B) + K_B(W) = (1 + tau)^2, tau = r/s (body radius over semispan
 *    measured from the axis). Verified here by re-deriving the Joukowski-map added-mass result
 *    (see tests): lift of wing+body carry-over = rho V^2 alpha pi s^2 (1 - tau^2)^2 relative to the
 *    exposed-wing-alone lift rho V^2 alpha pi (s - r)^2.
 *  - K_W(B)_slender: lift on the exposed panels, integral of the cross-flow potential jump
 *      4 * Int_r^s sqrt(s'^2 - (y + r^2/y)^2) dy / (pi (s - r)^2),  s' = s + r^2/s
 *    (the closed form is NACA 1307 Eq. 14; the integral is evaluated numerically here).
 *  - K_W(B)_strip: strip theory with the body upwash a_v = a (1 + r^2/y^2) (NACA 1307 Eq. 15-17):
 *      Int c (1 + r^2/y^2) dy / Int c dy,  reducing to 1 + tau for a rectangular panel.
 *  - The two are blended on the effective aspect ratio A*beta (slender for A beta < 1, strip for > 4).
 *    The blend weights are an engineering choice (NOT from the report).
 *
 * Tail-in-wake downwash: one Lamb–Oseen tip vortex per forward panel (NACA 1307 / Missile DATCOM
 * single rolled-up vortex assumption) with circle-theorem image, viscous core growth, optional
 * exponential Γ decay, and a mild α-dependent radial migration (engineering).
 */
import type { FinGeom } from "./geometry.ts";
import { DEG, clamp, simpson, smoothstep } from "./util.ts";

export function slenderKW(tau: number): number {
  if (tau < 1e-6) return 1;
  // normalise s = 1, r = tau
  const s = 1;
  const r = clamp(tau, 0, 0.995);
  const sp = s + (r * r) / s;
  // y = s - (s - r) u^2
  const f = (u: number) => {
    const y = s - (s - r) * u * u;
    const sig = y + (r * r) / y;
    const v = Math.max(sp * sp - sig * sig, 0);
    return u * Math.sqrt(v);
  };
  const integ = 2 * (s - r) * simpson(f, 0, 1, 400);
  return (4 * integ) / (Math.PI * (s - r) * (s - r));
}

export function stripKW(g: FinGeom): number {
  const r = g.rRoot;
  const be = g.exposedSpan;
  const N = 48;
  let num = 0;
  let den = 0;
  for (let i = 0; i < N; i++) {
    const eta = (i + 0.5) / N;
    const c = g.chordAt(eta);
    const y = r + be * eta;
    num += c * (1 + (r * r) / (y * y));
    den += c;
  }
  return num / Math.max(den, 1e-12);
}

export interface KFactors {
  KWB: number;
  KBW: number;
  slenderWeight: number;
}

export function interferenceFactors(g: FinGeom, M: number): KFactors {
  const tau = g.tau;
  if (tau < 1e-6) return { KWB: 1, KBW: 0, slenderWeight: 0 };
  const beta = Math.sqrt(Math.abs(1 - M * M));
  const Ab = g.aspect * beta;
  const wSl = 1 - smoothstep(1, 4, Ab);
  const KWB = wSl * slenderKW(tau) + (1 - wSl) * stripKW(g);
  const total = (1 + tau) * (1 + tau);
  return { KWB, KBW: Math.max(total - KWB, 0), slenderWeight: wSl };
}

export interface Vortex {
  /** position in the cross-flow plane (up, starboard) [m] */
  up: number;
  stb: number;
  /** image position inside the body */
  iup: number;
  istb: number;
  /** circulation / V  [m] (signed: sign = sign of panel force along +t) */
  gamma: number;
  /** station where shed (trailing edge at the vortex span) [m] */
  xShed: number;
  /** body radius for the image */
  rBody: number;
  /** length scale for decay [m] */
  decayLen: number;
  core: number;
  /** unit tangent of the shedding panel (up, stb) */
  tUp: number;
  tStb: number;
  /** radial unit vector of the panel */
  rUp: number;
  rStb: number;
}

/** Make the tip vortex of one panel. Fp = force coefficient along +t on the panel (Sref basis, with K_WB). */
export function makeVortex(g: FinGeom, phiDeg: number, Fp: number, sRef: number, decaySemis: number, alphaAbs = 0): Vortex | null {
  if (Math.abs(Fp) < 1e-12 || g.exposedSpan <= 0) return null;
  const phi = phiDeg * DEG;
  const rUp = Math.cos(phi);
  const rStb = Math.sin(phi);
  const tUp = -Math.sin(phi);
  const tStb = Math.cos(phi);
  const r = g.rRoot;
  const rv = vortexRadialStation(r, g.exposedSpan, alphaAbs);
  const ell = Math.max(rv - (r * r) / Math.max(rv, 1e-9), 0.05 * Math.max(g.exposedSpan, 1e-6));
  const gamma = (sRef * Fp) / (2 * ell); // Gamma / V
  const riv = r > 0 ? (r * r) / rv : 0;
  return {
    up: rv * rUp,
    stb: rv * rStb,
    iup: riv * rUp,
    istb: riv * rStb,
    gamma,
    xShed: g.set.xLE + g.set.rootChord,
    rBody: r,
    decayLen: decaySemis > 0 && Number.isFinite(decaySemis) ? decaySemis * Math.max(g.exposedSpan, 1e-6) : Infinity,
    core: Math.max(0.05 * Math.max(g.exposedSpan, 1e-6), 1e-4),
    tUp,
    tStb,
    rUp,
    rStb,
  };
}

/**
 * Induced cross-flow velocity / V at (up, stb), station x from vortex v.
 *
 * Core: Lamb–Oseen vortex (Lamb 1932 / Saffman) with viscous core growth
 *   r_c²(x) = r_c0² + 4 ν_t (x − x_shed) / V ,  ν_t / (V b) ≈ 0.0015 (engineering constant),
 *   v_θ (r) = (Γ / (2 π r)) (1 − exp(−r² / r_c²)).
 * Image vortex inside the body via the circle theorem (Milne-Thomson).
 * Optional exponential decay of Γ with streamwise distance (engineering; not in NACA 1307).
 * Lateral vortex position is taken at the generating span station (no α-dependent migration
 * in this revision — migration would follow Hemsch / Pitts charts; flagged as a limitation).
 */
export function inducedVelocity(v: Vortex, up: number, stb: number, x: number): { up: number; stb: number } {
  const dx = x - v.xShed;
  if (dx < 0) return { up: 0, stb: 0 };
  const dec = Number.isFinite(v.decayLen) ? Math.exp(-dx / Math.max(v.decayLen, 1e-9)) : 1;
  // Lamb–Oseen core growth: rc^2 = rc0^2 + 4 * (0.0015 * be) * dx
  const be = Math.max(v.core / 0.05, v.core * 20, 1e-3); // recover a span-ish scale
  const rc2 = v.core * v.core + 4 * 0.0015 * be * Math.max(dx, 0);
  const lo = (px: number, py: number, vx: number, vy: number) => {
    const dX = px - vx;
    const dY = py - vy;
    const r2 = dX * dX + dY * dY;
    // r = sqrt(r2); Lamb–Oseen uses r2 directly
    const damp = 1 - Math.exp(-r2 / Math.max(rc2, 1e-18));
    const mag = damp / Math.max(r2, 1e-18); // (1-exp)/r^2  →  v = Γ/(2π) * mag * (-dY, dX)
    return { up: -dY * mag, stb: dX * mag };
  };
  const inboardUp = v.up - 0.1 * v.rUp * Math.max(Math.hypot(v.up, v.stb), 1e-6);
  const inboardStb = v.stb - 0.1 * v.rStb * Math.max(Math.hypot(v.up, v.stb), 1e-6);
  const pr = lo(inboardUp, inboardStb, v.up, v.stb);
  const dot = pr.up * v.tUp + pr.stb * v.tStb;
  const kappa = (dot > 0 ? -1 : 1) * Math.sign(v.gamma || 1) * (Math.abs(v.gamma) / (2 * Math.PI)) * dec;
  const a = lo(up, stb, v.up, v.stb);
  let b = { up: 0, stb: 0 };
  if (v.rBody > 0) {
    const bb = lo(up, stb, v.iup, v.istb);
    b = { up: -bb.up, stb: -bb.stb }; // opposite image circulation
  }
  return { up: kappa * (a.up + b.up), stb: kappa * (a.stb + b.stb) };
}

/**
 * Shed-vortex radial station vs angle of attack (engineering): the tip vortex migrates
 * slightly outboard with α. Hemsch / Pitts-style charts give O(10%) shifts; we use
 *   r_v = r_root + (π/4) b_e * (1 + 0.15 sin|α|)
 * which is an engineering fit, NOT a published closed form — medium/low confidence.
 */
export function vortexRadialStation(rRoot: number, exposedSpan: number, alphaAbs: number): number {
  return rRoot + (Math.PI / 4) * exposedSpan * (1 + 0.15 * Math.sin(Math.min(alphaAbs, Math.PI / 2)));
}
