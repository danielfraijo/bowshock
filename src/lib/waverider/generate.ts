import type { AeroEstimate, BuiltVehicle, DesignParams, SurfaceGrid, Vec3 } from "./types";
import {
  DEG,
  RAD,
  betaFromThetaM,
  chordFractions,
  clamp,
  cosineSpace,
  lerp,
  maxTheta,
  obliqueShock,
  solveConeShock,
  thetaFromBetaM,
} from "./math";
import { MeshBuilder, analyzeMesh, compactMesh, enforceMonotoneChord, gridPoint, makeGrid, orientOutward, setGridPoint, stitchGrid, zipperSharpEdges } from "./mesh";
import { applyCowlLip, applyLeadingFillet, bluntStarNose, effectiveLeRadius, filletTipCap } from "./blunt";
import { bindCenterline } from "./sym-euler";
import { collapseTipRows, orientConsistent, removeDoubleFaces, tidyTrailingEdge } from "./topology";
import { repairAftIfDefective } from "./mesh-repair"; // [mesh-refinement]
import { clusterFractionsToLE } from "./refinement"; // [mesh-refinement]
import { curvatureStations } from "./curvature"; // [mesh-refinement]

function liftStations(ny: number, half: boolean): number[] {
  // ±1 so the elliptic section closes. Cluster at the sides, where curvature is highest.
  if (half) {
    const n = Math.max(8, Math.ceil(ny / 2));
    const ys: number[] = [];
    for (let j = 0; j < n; j++) ys.push(Math.sin((j / Math.max(n - 1, 1)) * (Math.PI / 2)));
    ys[0] = 0;
    ys[n - 1] = 1;
    return ys;
  }
  let n = Math.max(9, ny);
  if (n % 2 === 0) n += 1;
  const ys: number[] = [];
  for (let j = 0; j < n; j++) ys.push(-Math.cos((j / (n - 1)) * Math.PI));
  ys[0] = -1;
  ys[n - 1] = 1;
  ys[(n - 1) >> 1] = 0;
  return ys;
}

function baseSpanStations(ny: number, s: number, half: boolean): number[] {
  // Stay off y = ±s (zero-chord pole) but keep the tip close to the planform
  // edge so the TE does not look chopped. Always include y = 0 for Pointwise.
  if (half) {
    const n = Math.max(6, Math.ceil(ny / 2));
    const yMax = s * (1 - 0.12 / n);
    const ys: number[] = [];
    for (let j = 0; j < n; j++) ys.push(yMax * cosineSpace(j, n));
    ys[0] = 0;
    ys[n - 1] = yMax;
    return ys;
  }
  let n = Math.max(7, ny);
  if (n % 2 === 0) n += 1;
  const yMax = s * (1 - 0.12 / n);
  const ys: number[] = [];
  for (let j = 0; j < n; j++) ys.push(-yMax + 2 * yMax * cosineSpace(j, n));
  ys[0] = -yMax;
  ys[n - 1] = yMax;
  ys[(n - 1) >> 1] = 0;
  return ys;
}

/**
 * Span stations. With `nose` set (a rounded planform vertex of half-width `w` at y = 0)
 * the same number of stations is re-distributed so the vertex region is sampled at
 * about `dy` (adaptive sampling on the high-curvature part of the planform); the tips
 * and y = 0 stay fixed and the station count is unchanged.
 */
function spanStations(ny: number, s: number, half: boolean, nose?: { w: number; dy: number }): number[] {
  const base = baseSpanStations(ny, s, half);
  if (!nose || !(nose.w > 0) || !(nose.dy > 0)) return base;
  const n = base.length;
  // work on the half range [0, yMax] and mirror
  const mid = half ? 0 : (n - 1) >> 1;
  const pos = base.slice(mid).map((y) => Math.abs(y));
  const m = pos.length;
  if (m < 4) return base;
  const dy0 = pos[1] - pos[0];
  const B = Math.min(14, Math.max(0, dy0 / nose.dy - 1));
  if (B < 0.05) return base;
  const yMax = pos[m - 1];
  // density = base density (1/spacing, piecewise) * (1 + B g(y))
  const NS = 4000;
  const cum = new Float64Array(NS + 1);
  let k = 0;
  for (let i = 1; i <= NS; i++) {
    const y = (yMax * (i - 0.5)) / NS;
    while (k < m - 2 && pos[k + 1] < y) k++;
    const rho = 1 / Math.max(pos[k + 1] - pos[k], 1e-12);
    const g = Math.exp(-((y / nose.w) ** 2));
    cum[i] = cum[i - 1] + (rho * (1 + B * g) * yMax) / NS;
  }
  const total = cum[NS];
  const out: number[] = [0];
  let c = 0;
  for (let j = 1; j < m - 1; j++) {
    const target = (total * j) / (m - 1);
    while (c < NS - 1 && cum[c + 1] < target) c++;
    const u = (target - cum[c]) / Math.max(cum[c + 1] - cum[c], 1e-300);
    out.push((yMax * (c + clamp(u, 0, 1))) / NS);
  }
  out.push(yMax);
  if (half) return out;
  const res: number[] = new Array(n);
  for (let j = 0; j < m; j++) {
    res[mid + j] = out[j];
    res[mid - j] = -out[j];
  }
  res[mid] = 0;
  return res;
}

function xLeading(
  y: number,
  L: number,
  s: number,
  planform: DesignParams["planform"],
  p: number,
  spatular: number,
) {
  const yn = clamp(Math.abs(y) / Math.max(s, 1e-12), 0, 1);
  let x: number;
  if (planform === "rect") {
    x = 0;
  } else if (planform === "spatular") {
    const nose = clamp(spatular, 0.04, 0.45);
    const blunt = 1 - Math.sqrt(Math.max(0, 1 - yn * yn));
    x = L * lerp(nose * blunt, yn, 0.35);
  } else if (planform === "double") {
    const kink = 0.42;
    if (yn < kink) x = L * 0.1 * (yn / kink);
    else x = L * (0.1 + 0.9 * ((yn - kink) / (1 - kink)) ** 1.05);
  } else {
    const pow = planform === "delta" ? 1 : clamp(p, 0.6, 2.4);
    x = L * yn ** pow;
  }
  return clamp(x, 0, L);
}

function xTrailing(y: number, L: number, teTan: number, xl: number) {
  // Tiny finite chord at the tip so the grid is not a pole — never long enough
  // to push the TE past x = L (that was the aft spike / indent).
  const minC = 0.0045 * L;
  const xt = L - Math.abs(y) * teTan;
  return clamp(Math.max(xt, xl + minC), xl + 1e-6 * L, L);
}

function superZ(y: number, s: number, h: number, n: number) {
  const yn = clamp(Math.abs(y) / Math.max(s, 1e-12), 0, 1);
  return -h * (1 - yn ** n) ** (1 / n);
}

function closeVehicle(
  b: MeshBuilder,
  upper: SurfaceGrid,
  lower: SurfaceGrid,
  half: boolean,
  flowThrough = false,
  leading?: SurfaceGrid | null,
) {
  stitchGrid(b, upper, "upper", false, 0);
  stitchGrid(b, lower, "lower", true, 1);
  if (leading && leading.ni >= 2 && leading.nj >= 2) stitchGrid(b, leading, "leading", false);
  if (leading && leading.ni >= 3) {
    const lead = leading;
    const cap = (j: number, kind: "leading" | "symmetry") => {
      const loop: number[] = [];
      for (let k = 0; k < lead.ni; k++) loop.push(b.vertP(gridPoint(lead, k, j)));
      b.fan(loop, kind);
    };
    if (half) {
      cap(0, "symmetry");
      cap(lead.nj - 1, "leading");
    } else {
      cap(0, "leading");
      cap(lead.nj - 1, "leading");
    }
  }

  const nj = upper.nj;
  if (!flowThrough) {
    const teU: number[] = [];
    const teL: number[] = [];
    for (let j = 0; j < nj; j++) teU.push(b.vertP(gridPoint(upper, upper.ni - 1, j)));
    for (let j = 0; j < nj; j++) teL.push(b.vertP(gridPoint(lower, lower.ni - 1, j)));
    for (let j = 0; j < nj - 1; j++) {
      if (teU[j] === teL[j] && teU[j + 1] === teL[j + 1]) continue;
      b.quad(teU[j], teL[j], teL[j + 1], teU[j + 1], "base");
    }
  }

  const sideJs = half ? [nj - 1] : [0, nj - 1];
  for (const j of sideJs) {
    for (let i = 0; i < upper.ni - 1; i++) {
      const u0 = b.vertP(gridPoint(upper, i, j));
      const u1 = b.vertP(gridPoint(upper, i + 1, j));
      const l0 = b.vertP(gridPoint(lower, i, j));
      const l1 = b.vertP(gridPoint(lower, i + 1, j));
      if (u0 === l0 && u1 === l1) continue;
      if (j === 0) b.quad(u0, l0, l1, u1, "leading");
      else b.quad(u0, u1, l1, l0, "leading");
    }
  }

  if (half) {
    const su: number[] = [];
    const sl: number[] = [];
    for (let i = 0; i < upper.ni; i++) su.push(b.vertP(gridPoint(upper, i, 0)));
    for (let i = 0; i < lower.ni; i++) sl.push(b.vertP(gridPoint(lower, i, 0)));
    const n = Math.min(su.length, sl.length);
    for (let i = 0; i < n - 1; i++) {
      if (su[i] === sl[i] && su[i + 1] === sl[i + 1]) continue;
      b.quad(su[i], su[i + 1], sl[i + 1], sl[i], "symmetry");
    }
  }

  if (!flowThrough && !(leading && leading.ni >= 2)) {
    const leU: number[] = [];
    const leL: number[] = [];
    for (let j = 0; j < nj; j++) leU.push(b.vertP(gridPoint(upper, 0, j)));
    for (let j = 0; j < nj; j++) leL.push(b.vertP(gridPoint(lower, 0, j)));
    for (let j = 0; j < nj - 1; j++) {
      if (leU[j] === leL[j] && leU[j + 1] === leL[j + 1]) continue;
      b.quad(leU[j], leU[j + 1], leL[j + 1], leL[j], "inlet");
    }
  }
}

function closeDuct(
  b: MeshBuilder,
  cowl: SurfaceGrid,
  floor: SurfaceGrid,
  half: boolean,
  flowThrough: boolean,
) {
  stitchGrid(b, cowl, "cowl", false, 0);
  stitchGrid(b, floor, "lower", true, 1);
  const nj = Math.min(cowl.nj, floor.nj);
  const ni = Math.min(cowl.ni, floor.ni);
  const sideJs = half ? [nj - 1] : [0, nj - 1];
  for (const j of sideJs) {
    for (let i = 0; i < ni - 1; i++) {
      const u0 = b.vertP(gridPoint(cowl, i, j));
      const u1 = b.vertP(gridPoint(cowl, i + 1, j));
      const l0 = b.vertP(gridPoint(floor, i, j));
      const l1 = b.vertP(gridPoint(floor, i + 1, j));
      if (u0 === l0 && u1 === l1) continue;
      if (j === 0) b.quad(u0, l0, l1, u1, "leading");
      else b.quad(u0, u1, l1, l0, "leading");
    }
  }
  if (half) {
    for (let i = 0; i < ni - 1; i++) {
      const su0 = b.vertP(gridPoint(cowl, i, 0));
      const su1 = b.vertP(gridPoint(cowl, i + 1, 0));
      const sl0 = b.vertP(gridPoint(floor, i, 0));
      const sl1 = b.vertP(gridPoint(floor, i + 1, 0));
      if (su0 === sl0 && su1 === sl1) continue;
      b.quad(su0, su1, sl1, sl0, "symmetry");
    }
  }
  const capSpan = (i: number, kind: "inlet" | "nozzle") => {
    const u: number[] = [];
    const l: number[] = [];
    for (let j = 0; j < nj; j++) {
      u.push(b.vertP(gridPoint(cowl, i, j)));
      l.push(b.vertP(gridPoint(floor, i, j)));
    }
    for (let j = 0; j < nj - 1; j++) {
      if (u[j] === l[j] && u[j + 1] === l[j + 1]) continue;
      if (kind === "inlet") b.quad(u[j], u[j + 1], l[j + 1], l[j], "inlet");
      else b.quad(u[j], l[j], l[j + 1], u[j + 1], "nozzle");
    }
  };
  if (!flowThrough) {
    capSpan(0, "inlet");
    capSpan(ni - 1, "nozzle");
  }
}

function aeroFrom(
  params: DesignParams,
  volume: number,
  area: number,
  planformArea: number,
  theta: number,
  beta: number,
  attached: boolean,
  notes: string[],
): AeroEstimate {
  const sh = attached ? obliqueShock(params.mach, beta, params.gamma) : null;
  const mu = Math.asin(clamp(1 / params.mach, 0, 1));
  const p2 = sh?.p2p1 ?? 1;
  const S = Math.max(planformArea, 1e-12);
  const nLo = Math.cos(theta);
  const axial = Math.max(1e-6, Math.sin(theta));
  const qInf = 0.5 * params.gamma * params.mach * params.mach;
  const cl = attached ? ((p2 - 1) * nLo) / qInf : 0;
  const cdWave = attached ? ((p2 - 1) * axial) / qInf : 0;
  const cdBase = 2 / (params.gamma * params.mach * params.mach);
  const cd = cdWave + cdBase * 0.55;
  const ld = cd > 1e-8 ? cl / cd : 0;
  return {
    thetaDeg: theta * RAD,
    betaDeg: beta * RAD,
    muDeg: mu * RAD,
    coneDeg: params.coneDeg,
    pressureRatio: sh?.p2p1 ?? 1,
    temperatureRatio: sh?.t2t1 ?? 1,
    densityRatio: sh?.r2r1 ?? 1,
    m2: sh?.M2 ?? params.mach,
    planformArea: S,
    wettedArea: area,
    volume,
    cl,
    cd,
    ld,
    volumetricEfficiency: S > 0 ? volume / S ** 1.5 : 0,
    attached,
    notes,
  };
}

function resolveShock(params: DesignParams): { theta: number; beta: number; attached: boolean; notes: string[] } {
  const notes: string[] = [];
  const thetaGeom = Math.atan(params.height / Math.max(params.length, 1e-9));
  const cap = maxTheta(params.mach, params.gamma);
  let theta = thetaGeom;
  let attached = theta < cap.theta * 0.98;
  if (!attached) {
    notes.push("Requested height detaches the shock at this Mach — using the maximum attached wedge.");
    theta = cap.theta * 0.96;
  }
  let beta = betaFromThetaM(params.mach, theta, params.gamma);
  if (!Number.isFinite(beta)) {
    beta = cap.beta;
    attached = false;
  }
  const userBeta = params.shockDeg * DEG;
  if (params.family === "caret" || params.family === "star") {
    if (userBeta > Math.asin(clamp(1 / params.mach, 0, 1)) + 0.2 * DEG && userBeta < 70 * DEG) {
      const tUser = thetaFromBetaM(params.mach, userBeta, params.gamma);
      if (tUser > 0) {
        beta = userBeta;
        theta = tUser;
        attached = true;
      }
    }
  }
  return { theta, beta, attached, notes };
}

/**
 * Round the planform vertex (the V where a delta LE meets the centerline) with a circle of
 * radius rho in plan view: morphological opening of the planform region by a disk, which
 * leaves straight LE segments untouched and replaces the corner by an exact circular arc.
 * The fillet that follows then rolls over a smooth LE curve, so the nose has no crease.
 * Returns the original function when the LE has no real vertex (smooth or blunt planforms).
 */
function roundPlanformVertex(g: (t: number) => number, s: number, rho: number): (y: number) => number {
  const N = 1200;
  const dt = s / N;
  const gs = new Float64Array(N + 1);
  for (let i = 0; i <= N; i++) gs[i] = g(i * dt);
  const k0 = (gs[2] - gs[0]) / (2 * dt);
  if (!(rho > 4 * dt) || !(k0 > 0.2)) return (y) => g(Math.abs(y));
  // inward offset (right branch): points (xo, yo)
  const xo = new Float64Array(N + 1);
  const yo = new Float64Array(N + 1);
  for (let i = 0; i <= N; i++) {
    const a = Math.max(0, i - 1);
    const b = Math.min(N, i + 1);
    const gp = (gs[b] - gs[a]) / ((b - a) * dt);
    const nn = Math.sqrt(1 + gp * gp);
    xo[i] = gs[i] + rho / nn;
    yo[i] = i * dt - (rho * gp) / nn;
  }
  const slope0 = k0;
  const hR = (yc: number): number => {
    if (yc <= yo[0]) return xo[0] + slope0 * (yc - yo[0]);
    if (yc >= yo[N]) return xo[N] + ((xo[N] - xo[N - 1]) / (yo[N] - yo[N - 1])) * (yc - yo[N]);
    let lo = 0;
    let hi = N;
    while (hi - lo > 1) {
      const m = (lo + hi) >> 1;
      if (yo[m] <= yc) lo = m;
      else hi = m;
    }
    const u = (yc - yo[lo]) / Math.max(yo[hi] - yo[lo], 1e-300);
    return xo[lo] + u * (xo[hi] - xo[lo]);
  };
  const M = N;
  const h = new Float64Array(2 * M + 1); // yc from -s..s
  for (let m = -M; m <= M; m++) {
    const yc = m * dt;
    h[m + M] = Math.max(hR(yc), hR(-yc));
  }
  const W = Math.ceil(rho / dt);
  const out = new Float64Array(N + 1);
  for (let i = 0; i <= N; i++) {
    let best = Infinity;
    for (let w = -W; w <= W; w++) {
      const m = i + w;
      if (m < -M || m > M) continue;
      const dy = w * dt;
      const v = h[m + M] - Math.sqrt(Math.max(rho * rho - dy * dy, 0));
      if (v < best) best = v;
    }
    out[i] = best;
  }
  // far from the vertex the opening reproduces g; pin that exactly so tips are untouched
  const tCut = 2.2 * rho;
  return (y: number) => {
    const t = Math.min(Math.abs(y), s);
    if (t >= tCut) return g(t);
    const f = t / dt;
    const i0 = Math.min(Math.floor(f), N - 1);
    const u = f - i0;
    const xr = out[i0] + u * (out[i0 + 1] - out[i0]);
    const wgt = t <= 1.4 * rho ? 1 : 1 - (t - 1.4 * rho) / (0.8 * rho);
    return wgt * xr + (1 - wgt) * g(t);
  };
}

/** Target max (row-to-row edge)/(chord gap) of loft cells; rows needing more use fewer chord points. */
const CELL_ASPECT = 3.5;
/** First chord gap at the LE is at least (row-to-row edge)/MIN_GAP_DIV. */
const MIN_GAP_DIV = 5;
const GAP_BLUR = 3;
/** Max change in distinct chord points between neighbouring rows. */
const M_SLOPE = 6;

function lofts(
  params: DesignParams,
  zBase: (y: number) => number,
  planform: DesignParams["planform"],
  power: number,
  spat: number,
  shockScale = 1.4,
  zExp = 1,
): { upper: SurfaceGrid; lower: SurfaceGrid; shock: SurfaceGrid[] } {
  const L = params.length;
  const s = params.span / 2;
  const Rle = effectiveLeRadius(params);
  const rhoPlan = Rle > 0 && planform !== "rect" ? Math.min(3.5 * Rle, 0.04 * L, 0.2 * s) : 0;
  const xleRaw = (t: number) => xLeading(t, L, s, planform, power, spat);
  const xle = rhoPlan > 0 ? roundPlanformVertex(xleRaw, s, rhoPlan) : xleRaw;
  const ys0 = spanStations(params.ny, s, params.halfModel, rhoPlan > 0 ? { w: rhoPlan, dy: rhoPlan / 12 } : undefined);
  // [mesh-refinement] curvature-adaptive spanwise stations (strength 0 / undefined = the unchanged stations)
  const ys = curvatureStations(ys0, params.refinement?.surface.curvature ?? 0, (y) => [xle(y), y, zBase(y)]);
  const nx = Math.max(8, params.nx);
  const nj = ys.length;
  const zPow = Math.max(0.6, zExp);
  const dih = Math.tan((params.dihedralDeg || 0) * DEG);
  const cam = (params.camber || 0) * params.height;
  const teTan = Math.tan((params.teSweepDeg || 0) * DEG);
  const rowInfo = ys.map((y, j) => {
    const xl = xle(y);
    const xt = xTrailing(y, L, teTan, xl);
    let edge = 0;
    if (j > 0) edge = Math.max(edge, Math.hypot(xle(ys[j]) - xle(ys[j - 1]), ys[j] - ys[j - 1]));
    if (j + 1 < nj) edge = Math.max(edge, Math.hypot(xle(ys[j + 1]) - xle(ys[j]), ys[j + 1] - ys[j]));
    const dyJ = Math.max(j > 0 ? ys[j] - ys[j - 1] : 0, j + 1 < nj ? ys[j + 1] - ys[j] : 0);
    return { chord: xt - xl, edge, jLen: Math.max(0.55 * edge + 0.45 * dyJ, 1e-12) };
  });
  // Rows whose uniform chord gap is far below the row-to-row edge would give needle cells (swept tips).
  // Such rows use fewer distinct chord points. The grid index still maps onto them monotonically, so the
  // extra index cells collapse to zero width (welded away) instead of becoming slivers. The distinct-point
  // count is Lipschitz-limited across rows so neighbouring rows never differ wildly (no twisted cells).
  const mRow = rowInfo.map((r, j) => (Math.abs(ys[j]) < 2.5 * rhoPlan ? nx : clamp(Math.floor((r.chord * CELL_ASPECT) / r.jLen) + 1, Math.min(5, nx), nx)));
  for (let pass = 0; pass < 2; pass++) {
    for (let j = 1; j < nj; j++) mRow[j] = Math.min(mRow[j], mRow[j - 1] + M_SLOPE);
    for (let j = nj - 2; j >= 0; j--) mRow[j] = Math.min(mRow[j], mRow[j + 1] + M_SLOPE);
  }
  // The LE-end gap target varies smoothly across rows (log-domain blur), otherwise neighbouring rows
  // switch between cosine and geometric grading and the cells between them twist into slivers.
  let lg = rowInfo.map((r) => Math.log(Math.max(r.edge, 1e-9) / MIN_GAP_DIV));
  for (let pass = 0; pass < GAP_BLUR; pass++) lg = lg.map((v, j) => (lg[Math.max(j - 1, 0)] + 2 * v + lg[Math.min(j + 1, nj - 1)]) / 4);
  const fracs = rowInfo.map((r, j) => {
    const m = mRow[j];
    const base = chordFractions(m, r.chord, Math.exp(lg[j]));
    let out = base;
    if (m < nx) {
      out = new Float64Array(nx);
      for (let i = 0; i < nx; i++) out[i] = base[Math.round((i * (m - 1)) / (nx - 1))];
    }
    // [mesh-refinement] optional extra stretching toward the LE / nose; strength 0 returns the same array (unchanged).
    return clusterFractionsToLE(out, params.refinement?.surface.leClustering ?? 0);
  });
  const sample = (i: number, j: number, isLower: boolean): Vec3 => {
    const y = ys[j];
    const xl = xle(y);
    const xt = xTrailing(y, L, teTan, xl);
    const chord = xt - xl;
    const zOff = Math.abs(y) * dih;
    if (chord <= 1e-12 * L) return [xl, y, zOff];
    const frac = fracs[j][i];
    const x = lerp(xl, xt, frac);
    const zLid = 4 * cam * frac * (1 - frac);
    const z = isLower ? zBase(y) * frac ** zPow : 0;
    return [x, y, z + zLid + zOff];
  };
  const upper = makeGrid("upper", nx, nj, (i, j) => sample(i, j, false));
  const lower = makeGrid("lower", nx, nj, (i, j) => sample(i, j, true));
  const shock = [
    makeGrid("shock", nx, nj, (i, j) => {
      const y = ys[j];
      const xl = xle(y);
      const xt = xTrailing(y, L, teTan, xl);
      const chord = xt - xl;
      if (chord <= 1e-12 * L) return [xl, y, Math.abs(y) * dih];
      const frac = i / Math.max(nx - 1, 1);
      const x = lerp(xl, xt, frac);
      const z = zBase(y) * shockScale * frac ** zPow;
      return [x, y, z + Math.abs(y) * dih];
    }),
  ];
  return { upper, lower, shock };
}

function buildCaret(params: DesignParams) {
  const { theta, beta } = resolveShock(params);
  const h = params.length * Math.tan(theta);
  const s = params.span / 2;
  const k = Math.tan(theta) / Math.max(Math.tan(beta), 1e-6);
  return lofts(params, (y) => -h * (1 - clamp(Math.abs(y) / Math.max(s, 1e-12), 0, 1)), "delta", 1, 0, 1 / Math.max(k, 0.2));
}

function buildOsculating(params: DesignParams) {
  const { theta, beta } = resolveShock(params);
  const k = Math.tan(theta) / Math.max(Math.tan(beta), 1e-6);
  const s = params.span / 2;
  const n = clamp(params.superN, 1, 8);
  return lofts(
    params,
    (y) => superZ(y, s, params.height, n),
    params.planform,
    params.planformPower,
    params.captureFrac,
    1 / Math.max(k, 0.2),
  );
}

function buildCone(params: DesignParams) {
  const s = params.span / 2;
  const n = 2.15;
  return lofts(
    params,
    (y) => superZ(y, s, params.height, n),
    "spatular",
    1,
    params.captureFrac,
    1.35,
  );
}

function buildWedgeCone(params: DesignParams) {
  const s = params.span / 2;
  const h = params.height;
  const yw = clamp(params.wedgeFrac, 0.08, 0.9) * s;
  const n = clamp(params.superN, 1.2, 6);
  return lofts(
    params,
    (y) => {
      const ay = Math.abs(y);
      if (ay <= yw) return -h;
      const t = clamp((ay - yw) / Math.max(s - yw, 1e-12), 0, 1);
      return -h * (1 - t ** n) ** (1 / n);
    },
    params.planform,
    params.planformPower,
    0.2,
    1.35,
  );
}

function buildViscopt(params: DesignParams) {
  const s = params.span / 2;
  const n = clamp(params.superN, 1.05, 5);
  const m = clamp(params.planformPower, 0.8, 2.2);
  return lofts(
    params,
    (y) => superZ(y, s, params.height, n),
    params.planform === "rect" ? "power" : params.planform,
    params.planformPower,
    params.captureFrac,
    1.3,
    m,
  );
}

function buildInward(params: DesignParams) {
  const s = params.span / 2;
  const h = params.height;
  const wall = clamp(params.wedgeFrac, 0.35, 0.88);
  return lofts(
    params,
    (y) => {
      const yn = clamp(Math.abs(y) / Math.max(s, 1e-12), 0, 1);
      if (yn <= wall) return -h;
      const t = (yn - wall) / Math.max(1 - wall, 1e-9);
      return -h * (1 - t * t);
    },
    params.planform,
    params.planformPower,
    params.captureFrac,
    1.25,
    1.05,
  );
}

function buildElliptic(params: DesignParams) {
  const s = params.span / 2;
  const h = params.height;
  return lofts(
    params,
    (y) => {
      const yn = clamp(Math.abs(y) / Math.max(s, 1e-12), 0, 1);
      return -h * Math.sqrt(Math.max(0, 1 - yn * yn));
    },
    params.planform === "rect" ? "delta" : params.planform,
    params.planformPower,
    params.captureFrac,
    1.22,
    1,
  );
}

function buildBusemann(params: DesignParams) {
  const s = params.span / 2;
  const h = params.height;
  return lofts(
    params,
    (y) => {
      const yn = clamp(Math.abs(y) / Math.max(s, 1e-12), 0, 1);
      const circ = Math.sqrt(Math.max(0, 1 - yn * yn));
      return -h * (0.28 + 0.72 * circ);
    },
    "spatular",
    1,
    clamp(params.captureFrac, 0.08, 0.35),
    1.12,
    0.72,
  );
}

function numDeriv(f: (x: number) => number, x: number): number {
  const h = Math.max(1e-6, 1e-4 * Math.abs(x));
  return (f(x + h) - f(x - h)) / (2 * h);
}

/** Hemisphere of radius Rn, then a monotone cubic into the aft OML. Slope is continuous; the cap cannot neck down into a lip. */
function semiAxis(x: number, Rn: number, xJoin: number, body: (x: number) => number, L: number): number {
  if (x <= 1e-12) return 0;
  const circ = (u: number) => Math.sqrt(Math.max(0, 2 * Rn * u - u * u));
  if (x <= Rn) return circ(x);
  if (x >= xJoin) return Math.max(circ(Rn), body(x));
  const dx = Math.max(xJoin - Rn, 1e-9);
  const p0 = Rn;
  const p1 = Math.max(body(xJoin), p0);
  let m1 = numDeriv(body, xJoin) * dx;
  if (!(m1 > 0)) m1 = 0;
  const evalH = (mm: number, t: number) => {
    const t2 = t * t;
    const t3 = t2 * t;
    const h00 = 2 * t3 - 3 * t2 + 1;
    const h10 = t3 - 2 * t2 + t;
    const h01 = -2 * t3 + 3 * t2;
    const h11 = t3 - t2;
    return h00 * p0 + h10 * 0 + h01 * p1 + h11 * mm;
  };
  let lo = 0;
  let hi = Math.max(m1, 0);
  let use = 0;
  for (let it = 0; it < 12; it++) {
    const mid = 0.5 * (lo + hi);
    let prev = p0;
    let ok = true;
    for (let k = 1; k <= 10; k++) {
      const a = evalH(mid, k / 10);
      if (a + 1e-7 < prev) ok = false;
      prev = a;
    }
    if (ok) {
      use = mid;
      lo = mid;
    } else hi = mid;
  }
  const t = clamp((x - Rn) / dx, 0, 1);
  return Math.max(0, evalH(use, t));
}

function buildLiftbody(params: DesignParams) {
  const L = params.length;
  const s = params.span / 2;
  const h = params.height;
  const nx = Math.max(28, params.nx);
  const ys = liftStations(params.ny, params.halfModel);
  const nj = ys.length;
  const pow = params.planform === "delta" ? 0.62 : clamp(params.planformPower * 0.62, 0.48, 1.15);
  const dih = Math.tan((params.dihedralDeg || 0) * DEG);
  const cam = (params.camber || 0) * h;
  const teTan = Math.tan((params.teSweepDeg || 0) * DEG);
  const requested = Math.max(effectiveLeRadius(params), 0.008 * L);
  const bu = 0.58 * h;
  const bl = 0.42 * h;
  const Rn = clamp(requested, 0.006 * L, 0.9 * Math.min(s, bu, bl));
  const fuseA = (x: number) => {
    const xi = clamp(x / L, 0, 1);
    return s * Math.pow(Math.max(xi, 1e-6), pow) * (1 - 0.07 * xi * xi);
  };
  const fuseU = (x: number) => {
    const xi = clamp(x / L, 0, 1);
    return bu * Math.pow(Math.max(xi, 1e-6), 0.55) * (1 - 0.06 * xi);
  };
  const fuseL = (x: number) => {
    const xi = clamp(x / L, 0, 1);
    return bl * Math.pow(Math.max(xi, 1e-6), 0.55) * (1 - 0.04 * xi);
  };
  let xJoin = Math.min(0.62 * L, Rn + Math.max(0.18 * L, 5 * Rn));
  for (let k = 0; k < 12; k++) {
    if (fuseA(xJoin) >= Rn && fuseU(xJoin) >= Rn && fuseL(xJoin) >= Rn) break;
    xJoin = Math.min(0.8 * L, xJoin + 0.04 * L);
  }
  const nCap = Math.max(18, Math.round(nx * 0.45));
  const xs: number[] = [];
  for (let i = 0; i < nCap; i++) {
    const phi = (Math.PI / 2) * (i / Math.max(nCap - 1, 1));
    xs.push(Rn * (1 - Math.cos(phi)));
  }
  const nAft = Math.max(12, nx - nCap);
  for (let i = 1; i <= nAft; i++) {
    const t = i / nAft;
    const u = t * t * (3 - 2 * t);
    xs.push(Rn + (L - Rn) * u);
  }
  xs[0] = 0;
  xs[xs.length - 1] = L;
  for (let i = 1; i < xs.length; i++) if (xs[i] <= xs[i - 1]) xs[i] = xs[i - 1] + 1e-6 * L;
  xs[xs.length - 1] = L;
  const ni = xs.length;
  const sample = (i: number, j: number, isLower: boolean): Vec3 => {
    if (i === 0) return [0, 0, 0];
    const x = xs[i];
    const a = semiAxis(x, Rn, xJoin, fuseA, L);
    const b = semiAxis(x, Rn, xJoin, isLower ? fuseL : fuseU, L);
    const yn = ys[j];
    const y = yn * a;
    const zEll = Math.sqrt(Math.max(0, 1 - yn * yn));
    const xi = clamp(x / L, 0, 1);
    const z = (isLower ? -1 : 1) * b * zEll + 4 * cam * xi * (1 - xi) + Math.abs(y) * dih;
    const xOut = i === ni - 1 ? Math.max(x - Math.abs(y) * teTan, Rn) : x;
    return [xOut, y, z];
  };
  const upper = makeGrid("upper", ni, nj, (i, j) => sample(i, j, false));
  const lower = makeGrid("lower", ni, nj, (i, j) => sample(i, j, true));
  return { upper, lower, shock: [] as SurfaceGrid[], noseR: Rn, noseReq: requested };
}

/**
 * 2-D ramjet / scramjet OML (X-43 / Hyper-X class).
 * Both surfaces run x = 0 → L so the capture plane (x=0) and nozzle (x=L)
 * are real rectangular faces, not collapsed lips.
 *
 * Floor: n ramps (internal compression, floor rises) → isolator → SERN drop.
 * Cowl: constant capture height then expanding nozzle.
 * Shock-on-lip: last-ramp shock aimed at the cowl at x_ramp.
 */
function buildDuct(params: DesignParams, scram: boolean) {
  const L = params.length;
  const s = params.span / 2;
  const hIn = Math.max(0.05, params.inletHeight);
  const th = params.rampDeg * DEG;
  const nR = clamp(Math.round(params.nRamps || 2), 1, 3);
  const xRamp = L * clamp(params.cowlFrac, 0.22, 0.58);
  const xComb = Math.min(L * (scram ? 0.82 : 0.86), xRamp + L * clamp(params.combustorFrac, 0.12, 0.38));
  const ER = Math.max(1.4, params.nozzleER);
  const throatT = clamp(scram ? 0.52 * hIn : 0.36 * hIn, 0.24 * hIn, 0.72 * hIn);
  const rise = clamp(hIn - throatT, 0.08 * hIn, 0.78 * hIn);
  const throat = hIn - rise;
  const hExit = scram ? throat * Math.sqrt(ER) : throat * ER;
  const expand = Math.max(0, hExit - throat);
  const cowlUp = 0.35 * expand;
  const floorDrop = 0.65 * expand;
  const dxR = xRamp / nR;
  const weights: number[] = [];
  for (let k = 0; k < nR; k++) weights.push(Math.tan((th * (k + 1)) / nR));
  const wsum = weights.reduce((a, b) => a + b, 0) || 1;
  const zFloor = (x: number) => {
    if (x <= 0) return 0;
    if (x <= xRamp) {
      let z = 0;
      let x0 = 0;
      for (let k = 0; k < nR; k++) {
        const x1 = dxR * (k + 1);
        const dz = rise * (weights[k] / wsum);
        if (x <= x1 + 1e-12) return z + ((x - x0) / Math.max(x1 - x0, 1e-12)) * dz;
        z += dz;
        x0 = x1;
      }
      return rise;
    }
    if (x <= xComb) return rise;
    const u = (x - xComb) / Math.max(L - xComb, 1e-9);
    return rise - floorDrop * u * u;
  };
  const zCowl = (x: number) => {
    if (x <= xComb) return hIn;
    const u = (x - xComb) / Math.max(L - xComb, 1e-9);
    return hIn + cowlUp * u;
  };
  const ys = spanStations(params.ny, s, params.halfModel);
  const nx = Math.max(16, params.nx);
  const nj = ys.length;
  const xs: number[] = [];
  const n1 = Math.max(6, Math.round(nx * (xRamp / L)));
  const n2 = Math.max(4, Math.round(nx * ((xComb - xRamp) / L)));
  const n3 = Math.max(6, nx - n1 - n2);
  for (let i = 0; i < n1; i++) xs.push((xRamp * i) / Math.max(n1 - 1, 1));
  for (let i = 1; i <= n2; i++) xs.push(xRamp + ((xComb - xRamp) * i) / n2);
  for (let i = 1; i <= n3; i++) xs.push(xComb + ((L - xComb) * i) / n3);
  xs[0] = 0;
  xs[xs.length - 1] = L;
  const ni = xs.length;
  const lower = makeGrid("lower", ni, nj, (i, j) => [xs[i], ys[j], zFloor(xs[i])]);
  const upper = makeGrid("cowl", ni, nj, (i, j) => [xs[i], ys[j], zCowl(xs[i])]);
  const inlet = makeGrid("inlet", 2, nj, (i, j) => (i === 0 ? gridPoint(lower, 0, j) : gridPoint(upper, 0, j)));
  const nozzle = makeGrid("nozzle", 2, nj, (i, j) =>
    i === 0 ? gridPoint(lower, ni - 1, j) : gridPoint(upper, ni - 1, j),
  );
  let beta = betaFromThetaM(params.mach, th, params.gamma);
  if (!Number.isFinite(beta)) beta = th + 8 * DEG;
  const sh = [
    makeGrid("shock", Math.max(8, n1), nj, (i, j) => {
      const x = xRamp * (i / Math.max(n1 - 1, 1));
      return [x, ys[j], hIn - x * Math.tan(beta)];
    }),
  ];
  return { upper, lower, shock: sh, extra: [inlet, nozzle] as SurfaceGrid[] };
}

function buildRamjet(params: DesignParams) {
  return buildDuct(params, false);
}

function buildScramjet(params: DesignParams) {
  return buildDuct(params, true);
}

function buildIntegrated(params: DesignParams) {
  const base = buildOsculating(params);
  const L = params.length;
  const s = params.span / 2;
  const yw = clamp(params.wedgeFrac, 0.12, 0.55) * s;
  const xC = L * clamp(params.cowlFrac, 0.28, 0.7);
  const hIn = Math.max(0.03, params.inletHeight);
  const dorsal = params.cowlSide === "dorsal";
  const morph = (g: SurfaceGrid, sign: number) => {
    for (let i = 0; i < g.ni; i++) {
      for (let j = 0; j < g.nj; j++) {
        const o = (i * g.nj + j) * 3;
        const x = g.xyz[o];
        const y = g.xyz[o + 1];
        if (Math.abs(y) > yw || x < xC) continue;
        const u = (x - xC) / Math.max(0.08 * L, 1e-9);
        const sU = u < 1 ? u * u * (3 - 2 * u) : 1;
        const yf = 1 - (Math.abs(y) / yw) ** 2;
        g.xyz[o + 2] += sign * hIn * sU * yf;
      }
    }
  };
  if (dorsal) morph(base.upper, 1);
  else morph(base.lower, -1);
  return base;
}

function applyElevon(upper: SurfaceGrid, lower: SurfaceGrid, deg: number, L: number) {
  if (Math.abs(deg) < 1e-3) return;
  const hinge = 0.82 * L;
  const k = Math.tan(deg * DEG);
  const apply = (g: SurfaceGrid) => {
    for (let i = 0; i < g.ni; i++) {
      for (let j = 0; j < g.nj; j++) {
        const o = (i * g.nj + j) * 3;
        const x = g.xyz[o];
        if (x > hinge) g.xyz[o + 2] -= (x - hinge) * k;
      }
    }
  };
  apply(upper);
  apply(lower);
}

/**
 * Collapse the wingtip chord onto the mid-surface so the planform edge is
 * not a chopped slab. Leave i = 0 alone when a fillet is attached: pinching
 * that arc to one point in a single span step is the corner fan.
 */
function sharpenTips(upper: SurfaceGrid, lower: SurfaceGrid, leading: SurfaceGrid | null, half: boolean) {
  const ni = Math.min(upper.ni, lower.ni);
  const nj = Math.min(upper.nj, lower.nj);
  if (nj < 3) return;
  const js = half ? [nj - 1] : [0, nj - 1];
  const i0 = leading ? 1 : 0;
  for (const j of js) {
    for (let i = i0; i < ni; i++) {
      const u = gridPoint(upper, i, j);
      const l = gridPoint(lower, i, j);
      const m: Vec3 = [(u[0] + l[0]) * 0.5, (u[1] + l[1]) * 0.5, (u[2] + l[2]) * 0.5];
      setGridPoint(upper, i, j, m);
      setGridPoint(lower, i, j, m);
    }
  }
}

function applyFins(lid: SurfaceGrid, params: DesignParams, zSign: number) {
  const h = (params.finHeight || 0) * Math.max(params.height, 0.05);
  if (h < 1e-4) return;
  const L = params.length;
  const yOff = params.span * 0.36;
  const yFins = params.halfModel ? [yOff] : [-yOff, yOff];
  const x0 = L * 0.68;
  const halfT = Math.max(params.span * 0.018, 0.01);
  for (let i = 0; i < lid.ni; i++) {
    for (let j = 0; j < lid.nj; j++) {
      const o = (i * lid.nj + j) * 3;
      const x = lid.xyz[o];
      const y = lid.xyz[o + 1];
      if (x < x0) continue;
      let dy = Infinity;
      for (const yc of yFins) dy = Math.min(dy, Math.abs(y - yc));
      if (dy >= halfT) continue;
      const along = (x - x0) / Math.max(L - x0, 1e-9);
      const across = 1 - (dy / halfT) ** 2;
      lid.xyz[o + 2] += zSign * h * along * across;
    }
  }
}

function flipGridsZ(grids: SurfaceGrid[]) {
  for (const g of grids) {
    for (let i = 2; i < g.xyz.length; i += 3) g.xyz[i] *= -1;
  }
}

function flipMeshZ(mesh: { positions: Float64Array }) {
  for (let i = 2; i < mesh.positions.length; i += 3) mesh.positions[i] *= -1;
}

function buildStar(params: DesignParams): {
  mesh: ReturnType<MeshBuilder["finish"]>;
  grids: SurfaceGrid[];
  shock: SurfaceGrid[];
  skipped: number;
} {
  const L = params.length;
  const { theta } = resolveShock(params);
  const h = L * Math.tan(theta);
  const rOut = params.span / 2;
  const rIn = Math.max(0.08 * rOut, rOut - h);
  const fins = clamp(Math.round(params.fins), 3, 8);
  const nx = Math.max(8, params.nx);
  const nPer = Math.max(4, Math.round(params.ny / fins));
  const b = new MeshBuilder(L);
  const grids: SurfaceGrid[] = [];

  const angle = (k: number, inner: boolean) => {
    const a0 = (2 * Math.PI * k) / fins - Math.PI / 2;
    return inner ? a0 + Math.PI / fins : a0;
  };
  const point = (x: number, k: number, inner: boolean): Vec3 => {
    const r = (x / L) * (inner ? rIn : rOut);
    const a = angle(k, inner);
    return [x, r * Math.cos(a), r * Math.sin(a)];
  };

  for (let f = 0; f < fins; f++) {
    const k0 = f;
    const k1 = (f + 1) % fins;
    const gA = makeGrid(`fin${f}-a`, nx, nPer, (i, j) => {
      const x = (i / (nx - 1)) * L;
      const t = j / (nPer - 1);
      const a = point(x, k0, false);
      const c = point(x, k0, true);
      return [x, lerp(a[1], c[1], t), lerp(a[2], c[2], t)];
    });
    const gB = makeGrid(`fin${f}-b`, nx, nPer, (i, j) => {
      const x = (i / (nx - 1)) * L;
      const t = j / (nPer - 1);
      const c = point(x, k0, true);
      const a = point(x, k1, false);
      return [x, lerp(c[1], a[1], t), lerp(c[2], a[2], t)];
    });
    grids.push(gA, gB);
  }
  const Rn = effectiveLeRadius(params);
  if (Rn > 0) bluntStarNose(grids, Rn, L);
  for (let f = 0; f < fins; f++) {
    stitchGrid(b, grids[2 * f], "upper", false, f * 2);
    stitchGrid(b, grids[2 * f + 1], "lower", false, f * 2 + 1);
  }

  const outline: number[] = [];
  for (let f = 0; f < fins; f++) {
    const gA = grids[2 * f];
    const gB = grids[2 * f + 1];
    for (let j = 0; j < gA.nj; j++) outline.push(b.vertP(gridPoint(gA, gA.ni - 1, j)));
    for (let j = 1; j < gB.nj; j++) outline.push(b.vertP(gridPoint(gB, gB.ni - 1, j)));
  }
  const core = b.vert(L, 0, 0);
  for (let i = 0; i < outline.length; i++) {
    b.tri(core, outline[i], outline[(i + 1) % outline.length], "base");
  }

  return { mesh: b.finish(), grids, shock: [], skipped: b.skipped };
}

function projectHalfModel(grids: SurfaceGrid[], half: boolean) {
  if (!half) {
    for (const g of grids) {
      for (let j = 0; j < g.nj; j++) {
        let yMean = 0;
        for (let i = 0; i < g.ni; i++) yMean += gridPoint(g, i, j)[1];
        yMean /= Math.max(g.ni, 1);
        if (Math.abs(yMean) > 1e-9) continue;
        for (let i = 0; i < g.ni; i++) {
          const p = gridPoint(g, i, j);
          p[1] = 0;
          setGridPoint(g, i, j, p);
        }
      }
    }
    return;
  }
  for (const g of grids) {
    for (let i = 0; i < g.ni; i++) {
      for (let j = 0; j < g.nj; j++) {
        const p = gridPoint(g, i, j);
        p[1] = j === 0 || Math.abs(p[1]) < 1e-6 ? 0 : Math.max(p[1], 0);
        setGridPoint(g, i, j, p);
      }
    }
  }
}

/**
 * Nose of the exported solid at (0,0,0). Origin is the most-forward
 * USED triangle vertex (the solid Pointwise sees), not unused builder
 * verts. Grids/shock ride the same shift. Half-model never shifts Y.
 */
function lockFrame(mesh: { positions: Float64Array; indices?: Uint32Array } | null, grids: SurfaceGrid[], shock: SurfaceGrid[], half: boolean) {
  const xs: number[] = [];
  const ys: number[] = [];
  const zs: number[] = [];
  const consider = (x: number, y: number, z: number) => {
    if (!Number.isFinite(x + y + z)) return;
    xs.push(x);
    ys.push(y);
    zs.push(z);
  };
  if (mesh?.indices && mesh.indices.length) {
    const p = mesh.positions;
    for (let t = 0; t < mesh.indices.length; t++) {
      const i = mesh.indices[t];
      consider(p[i * 3], p[i * 3 + 1], p[i * 3 + 2]);
    }
  } else {
    for (const g of grids) {
      for (let i = 0; i < g.xyz.length; i += 3) consider(g.xyz[i], g.xyz[i + 1], g.xyz[i + 2]);
    }
  }
  if (!xs.length) return;
  let xmin = Infinity;
  for (const x of xs) if (x < xmin) xmin = x;
  const band = 1e-3;
  let bestAy = Infinity;
  let yN = 0;
  let zN = 0;
  for (let i = 0; i < xs.length; i++) {
    if (xs[i] > xmin + band) continue;
    const ay = Math.abs(ys[i]);
    if (ay < bestAy) {
      bestAy = ay;
      yN = ys[i];
      zN = zs[i];
    }
  }
  const dy = half ? 0 : yN;
  const shift = (a: Float64Array) => {
    for (let i = 0; i < a.length; i += 3) {
      a[i] -= xmin;
      a[i + 1] -= dy;
      a[i + 2] -= zN;
    }
  };
  if (mesh) shift(mesh.positions);
  for (const g of grids) shift(g.xyz);
  for (const g of shock) shift(g.xyz);
  projectHalfModel(grids, half);
  if (half && mesh) {
    const p = mesh.positions;
    for (let i = 1; i < p.length; i += 3) {
      if (Math.abs(p[i]) < 1e-6) p[i] = 0;
    }
  }
}

function planformAreaOf(upper: SurfaceGrid): number {
  let A = 0;
  for (let i = 0; i < upper.ni - 1; i++) {
    for (let j = 0; j < upper.nj - 1; j++) {
      const p = gridPoint(upper, i, j);
      const q = gridPoint(upper, i + 1, j);
      const r = gridPoint(upper, i + 1, j + 1);
      const s = gridPoint(upper, i, j + 1);
      A += 0.5 * Math.abs((q[0] - p[0]) * (s[1] - p[1]) - (q[1] - p[1]) * (s[0] - p[0]));
      A += 0.5 * Math.abs((r[0] - p[0]) * (s[1] - p[1]) - (r[1] - p[1]) * (s[0] - p[0]));
    }
  }
  return A;
}

export function buildVehicle(params: DesignParams): BuiltVehicle {
  const notes: string[] = [];
  const shockSolve = resolveShock(params);
  let mesh;
  let skipped = 0;
  let grids: SurfaceGrid[] = [];
  let shock: SurfaceGrid[] = [];
  let noseMeta: { noseR?: number; noseReq?: number } | null = null;

  if (params.family === "star") {
    const star = buildStar(params);
    mesh = star.mesh;
    grids = star.grids;
    shock = star.shock;
    skipped = star.skipped;
    if (params.lid === "bottom") {
      flipMeshZ(mesh);
      flipGridsZ(grids);
      orientOutward(mesh);
    }
  } else {
    const built =
      params.family === "caret"
        ? buildCaret(params)
        : params.family === "cone"
          ? buildCone(params)
          : params.family === "wedgecone"
            ? buildWedgeCone(params)
            : params.family === "viscopt"
              ? buildViscopt(params)
              : params.family === "inward"
                ? buildInward(params)
                : params.family === "elliptic"
                  ? buildElliptic(params)
                  : params.family === "busemann"
                    ? buildBusemann(params)
                    : params.family === "liftbody"
                      ? buildLiftbody(params)
                      : params.family === "ramjet"
                        ? buildRamjet(params)
                        : params.family === "scramjet"
                          ? buildScramjet(params)
                          : params.family === "integrated"
                            ? buildIntegrated(params)
                            : buildOsculating(params);
    if (params.family === "liftbody") {
      const meta = built as { noseR?: number; noseReq?: number };
      noseMeta = { noseR: meta.noseR, noseReq: meta.noseReq };
    }
    if (params.lid === "bottom") {
      flipGridsZ([built.upper, built.lower, ...built.shock]);
    }
    const isDuct = params.family === "ramjet" || params.family === "scramjet";
    const isLift = params.family === "liftbody";
    const R = effectiveLeRadius(params);
    // deflect the elevon BEFORE the fillet is built so the LE fillet and tip caps follow the same rows
    if (!isDuct) applyElevon(built.upper, built.lower, params.elevonDeg || 0, params.length);
    let leading: SurfaceGrid | null = null;
    const tips: SurfaceGrid[] = [];
    if (!isDuct && !isLift) {
      if (R > 0) leading = applyLeadingFillet(built.upper, built.lower, R, params.length, { blend: params.leBlend });
      if (params.family !== "liftbody") {
        sharpenTips(built.upper, built.lower, leading, params.halfModel);
      }
      if (leading) {
        const a = filletTipCap(leading, 0, "tip_l");
        const c = filletTipCap(leading, leading.nj - 1, "tip_r");
        if (a) tips.push(a);
        if (c) tips.push(c);
      }
    } else if (isDuct && R > 0 && params.flowThrough) {
      const lip = applyCowlLip(built.upper, R, params.length);
      if (lip) {
        const extra = ((built as { extra?: SurfaceGrid[] }).extra ??= []);
        extra.push(lip);
      }
    }
    const zSign = params.lid === "bottom" ? -1 : 1;
    if (!isDuct) applyFins(built.upper, params, zSign);
    if (!isDuct) zipperSharpEdges(built.upper, built.lower, params.length);
    if (!isDuct) {
      tidyTrailingEdge(built.upper, built.lower, params.length);
      collapseTipRows(built.upper, built.lower, leading, params.length);
      enforceMonotoneChord(built.upper);
      enforceMonotoneChord(built.lower);
    }
    grids = [built.upper, built.lower];
    const extra = (built as { extra?: SurfaceGrid[] }).extra;
    if (extra) grids.push(...extra);
    if (leading) grids.push(leading);
    if (tips.length) grids.push(...tips);
    shock = built.shock;
    projectHalfModel(grids, params.halfModel);
    lockFrame(null, grids, shock, params.halfModel);
    const b = new MeshBuilder(params.length);
    if (isDuct) {
      closeDuct(b, built.upper, built.lower, params.halfModel, params.flowThrough);
      const extras = (built as { extra?: SurfaceGrid[] }).extra ?? [];
      for (const g of extras) {
        if (g.name === "cowl_lip") stitchGrid(b, g, "leading", false);
      }
    } else closeVehicle(b, built.upper, built.lower, params.halfModel, false, leading);
    mesh = b.finish();
    skipped = b.skipped;
    if (isDuct) {
      const u0 = gridPoint(built.upper, 0, 0);
      const l0 = gridPoint(built.lower, 0, 0);
      const uN = gridPoint(built.upper, built.upper.ni - 1, 0);
      const lN = gridPoint(built.lower, built.lower.ni - 1, 0);
      const hCap = Math.abs(u0[2] - l0[2]);
      const hNoz = Math.abs(uN[2] - lN[2]);
      notes.push(
        params.flowThrough
          ? `2-D ramjet/scramjet: rectangular INLET at x=0 (capture ${hCap.toFixed(3)} m) and NOZZLE at x=L (${hNoz.toFixed(3)} m) are OPEN for internal CFD. Floor ramps compress to the throat; cowl is constant then expands. Sidewalls are wetted walls.`
          : `2-D ramjet/scramjet: inlet (x=0, h=${hCap.toFixed(3)} m) and nozzle (x=L, h=${hNoz.toFixed(3)} m) capped. Enable flow-through for an open duct.`,
      );
    }
  }

  mesh = removeDoubleFaces(compactMesh(mesh));
  // [mesh-refinement] The grid-level TE tidy + removeDoubleFaces above is the primary closure. Only when that
  // leaves open / non-manifold edges (aft tips at higher ny) fall back to the tolerance-aware aft weld.
  // Open-ended flow-through ducts are intentionally open, so they are left alone.
  if (!(params.flowThrough && (params.family === "ramjet" || params.family === "scramjet"))) mesh = repairAftIfDefective(mesh, params.length);
  orientOutward(mesh);
  orientConsistent(mesh);
  lockFrame(mesh, grids, shock, params.halfModel);
  const usedX = (() => {
    let m = Infinity;
    const p = mesh.positions;
    for (let t = 0; t < mesh.indices.length; t++) m = Math.min(m, p[mesh.indices[t] * 3]);
    return m;
  })();
  notes.push(
    params.halfModel
      ? `Frame: most-forward point at (0,0,0). Half-model: Y = 0 is a planar symmetry face (Pointwise DC).`
      : `Frame: most-forward point at (0,0,0). X stream, Y span, Z up.`,
  );
  if (usedX > 1e-6) notes.push("STL used-vertex xmin > 0 — raise LE resolution.");

  const quality = analyzeMesh(mesh, skipped);
  const ductOpen = (params.family === "ramjet" || params.family === "scramjet") && params.flowThrough;
  const Rn = effectiveLeRadius(params);
  if (Rn > 0 && (params.family === "ramjet" || params.family === "scramjet")) {
    notes.push(`Lip radius R = ${(Rn * 1000).toFixed(1)} mm.`);
  } else if (Rn > 0 && params.family === "liftbody") {
    const geo = noseMeta?.noseR;
    const req = noseMeta?.noseReq;
    if (geo && req && geo < req * 0.92) {
      notes.push(
        `Nose cap is a tangent elliptic ogive, R = ${(geo * 1000).toFixed(1)} mm (requested ${(req * 1000).toFixed(1)} mm does not fit this OML without a lip). Pole is at the origin.`,
      );
    } else if (geo) {
      notes.push(`Nose cap is a tangent elliptic ogive, R = ${(geo * 1000).toFixed(1)} mm, pole welded at the origin.`);
    }
  } else if (Rn > 0 && grids.some((g) => g.name === "leading" || g.name === "cowl_lip")) {
    notes.push(
      `Leading-edge radius R = ${(Rn * 1000).toFixed(1)} mm, tessellated where the arc is at least 1/12 of the local span step.`,
    );
  } else if (Rn > 0 && params.family !== "star") {
    notes.push(
      `Heating nose radius R = ${(Rn * 1000).toFixed(1)} mm. The fillet arc is shorter than the span step, so the mesh keeps the sharp seam.`,
    );
  } else if (Rn > 0) {
    notes.push(`Nose radius R = ${(Rn * 1000).toFixed(1)} mm on the heating calculation.`);
  }
  if (!quality.watertight && !ductOpen) notes.push(`Mesh has ${quality.openEdges} open edges — raise streamwise/spanwise points.`);
  if (!shockSolve.attached) notes.push(...shockSolve.notes);
  if (params.lid === "bottom") notes.push("Lid on the belly — compression surface is the upper face.");

  const pArea =
    params.family === "star"
      ? 0.5 * params.span * params.length * 0.55
      : params.family === "ramjet" || params.family === "scramjet"
        ? params.span * params.length
        : planformAreaOf(grids[0]);
  const aero = aeroFrom(
    params,
    quality.volume,
    quality.area,
    pArea,
    shockSolve.theta,
    shockSolve.beta,
    shockSolve.attached,
    notes,
  );
  if (params.family === "cone") {
    const betaC = solveConeShock(params.mach, params.coneDeg * DEG, params.gamma);
    if (Number.isFinite(betaC)) aero.betaDeg = betaC * RAD;
  }

  const upperG = grids.find((g) => g.name === "upper" || g.name === "cowl");
  const lowerG = grids.find((g) => g.name === "lower");
  if (upperG && lowerG) bindCenterline(mesh, upperG, lowerG, params.span);

  return { params, mesh, grids, shockGrids: shock, quality, aero };
}
