import type { AeroEstimate, BuiltVehicle, DesignParams, SurfaceGrid, Vec3 } from "./types";
import {
  DEG,
  RAD,
  betaFromThetaM,
  clamp,
  cluster,
  cosineSpace,
  lerp,
  maxTheta,
  obliqueShock,
  solveConeShock,
  thetaFromBetaM,
} from "./math";
import { MeshBuilder, analyzeMesh, compactMesh, gridPoint, makeGrid, orientOutward, setGridPoint, stitchGrid, zipperSharpEdges } from "./mesh";
import { applyCowlLip, applyLeadingFillet, bluntStarNose, effectiveLeRadius, filletTipCap } from "./blunt";

function spanStations(ny: number, s: number, half: boolean): number[] {
  // Never sit on y = ±s (zero-chord delta pole) and always include y = 0
  // so Pointwise can pick a planar symmetry / centerline.
  if (half) {
    const n = Math.max(6, Math.ceil(ny / 2));
    const yMax = s * (1 - 0.4 / n);
    const ys: number[] = [];
    for (let j = 0; j < n; j++) ys.push(yMax * cosineSpace(j, n));
    ys[0] = 0;
    ys[n - 1] = yMax;
    return ys;
  }
  let n = Math.max(7, ny);
  if (n % 2 === 0) n += 1;
  const yMax = s * (1 - 0.4 / n);
  const ys: number[] = [];
  for (let j = 0; j < n; j++) ys.push(-yMax + 2 * yMax * cosineSpace(j, n));
  ys[0] = -yMax;
  ys[n - 1] = yMax;
  ys[(n - 1) >> 1] = 0;
  return ys;
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
  const xt = L - Math.abs(y) * teTan;
  return xt > xl ? xt : xl;
}

function superZ(y: number, s: number, h: number, n: number) {
  const yn = clamp(Math.abs(y) / Math.max(s, 1e-12), 0, 1);
  return -h * (1 - yn ** n) ** (1 / n);
}

function chordCluster(i: number, nx: number) {
  return cluster(i, nx, 1.35);
}

function closeVehicle(
  b: MeshBuilder,
  upper: SurfaceGrid,
  lower: SurfaceGrid,
  half: boolean,
  flowThrough = false,
  leading?: SurfaceGrid | null,
) {
  stitchGrid(b, upper, "upper", false);
  stitchGrid(b, lower, "lower", true);
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
  stitchGrid(b, cowl, "cowl", false);
  stitchGrid(b, floor, "lower", true);
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
  const ys = spanStations(params.ny, s, params.halfModel);
  const nx = Math.max(8, params.nx);
  const nj = ys.length;
  const xle = (y: number) => xLeading(y, L, s, planform, power, spat);
  const zPow = Math.max(0.6, zExp);
  const dih = Math.tan((params.dihedralDeg || 0) * DEG);
  const cam = (params.camber || 0) * params.height;
  const teTan = Math.tan((params.teSweepDeg || 0) * DEG);
  const sample = (i: number, j: number, isLower: boolean): Vec3 => {
    const y = ys[j];
    const xl = xle(y);
    const xt = xTrailing(y, L, teTan, xl);
    const chord = xt - xl;
    const zOff = Math.abs(y) * dih;
    if (chord <= 1e-12 * L) return [xl, y, zOff];
    const frac = chordCluster(i, nx);
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

function buildLiftbody(params: DesignParams) {
  const L = params.length;
  const s = params.span / 2;
  const h = params.height;
  const nx = Math.max(8, params.nx);
  const ys = spanStations(params.ny, 1, params.halfModel);
  const nj = ys.length;
  const pow = params.planform === "delta" ? 1 : clamp(params.planformPower, 0.55, 1.6);
  const dih = Math.tan((params.dihedralDeg || 0) * DEG);
  const upper = makeGrid("upper", nx, nj, (i, j) => {
    const t = chordCluster(i, nx);
    const x = t * L;
    const a = s * Math.max(t, 0.04) ** (pow === 1 ? 0.85 : pow * 0.7);
    const b = 0.5 * h * Math.max(t, 0.06) ** 0.55;
    const y = ys[j] * a;
    const yn = clamp(Math.abs(ys[j]), 0, 1);
    const z = b * Math.sqrt(Math.max(0, 1 - yn * yn));
    return [x, y, z + Math.abs(y) * dih];
  });
  const lower = makeGrid("lower", nx, nj, (i, j) => {
    const t = chordCluster(i, nx);
    const x = t * L;
    const a = s * Math.max(t, 0.04) ** (pow === 1 ? 0.85 : pow * 0.7);
    const b = 0.55 * h * Math.max(t, 0.06) ** 0.55;
    const y = ys[j] * a;
    const yn = clamp(Math.abs(ys[j]), 0, 1);
    const z = -b * Math.sqrt(Math.max(0, 1 - yn * yn));
    return [x, y, z + Math.abs(y) * dih];
  });
  return { upper, lower, shock: [] as SurfaceGrid[] };
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

/** Collapse the wingtip station to a seam so the planform edge is not a chopped slab. */
function sharpenTips(upper: SurfaceGrid, lower: SurfaceGrid, leading: SurfaceGrid | null, half: boolean) {
  const ni = Math.min(upper.ni, lower.ni);
  const nj = Math.min(upper.nj, lower.nj);
  if (nj < 3) return;
  const js = half ? [nj - 1] : [0, nj - 1];
  for (const j of js) {
    for (let i = 0; i < ni; i++) {
      const u = gridPoint(upper, i, j);
      const l = gridPoint(lower, i, j);
      const m: Vec3 = [(u[0] + l[0]) * 0.5, (u[1] + l[1]) * 0.5, (u[2] + l[2]) * 0.5];
      setGridPoint(upper, i, j, m);
      setGridPoint(lower, i, j, m);
    }
    if (leading) {
      const p = gridPoint(upper, 0, j);
      for (let k = 0; k < leading.ni; k++) setGridPoint(leading, k, j, p);
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
    stitchGrid(b, grids[2 * f], "upper", false);
    stitchGrid(b, grids[2 * f + 1], "lower", false);
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
    if (params.lid === "bottom") {
      flipGridsZ([built.upper, built.lower, ...built.shock]);
    }
    const isDuct = params.family === "ramjet" || params.family === "scramjet";
    const R = effectiveLeRadius(params);
    let leading: SurfaceGrid | null = null;
    const tips: SurfaceGrid[] = [];
    if (!isDuct) {
      if (R > 0) leading = applyLeadingFillet(built.upper, built.lower, R, params.length);
      if (params.family !== "liftbody") {
        sharpenTips(built.upper, built.lower, leading, params.halfModel);
      }
      if (leading) {
        const a = filletTipCap(leading, 0, "tip_l");
        const c = filletTipCap(leading, leading.nj - 1, "tip_r");
        if (a) tips.push(a);
        if (c) tips.push(c);
      }
      applyElevon(built.upper, built.lower, params.elevonDeg || 0, params.length);
    } else if (R > 0 && params.flowThrough) {
      const lip = applyCowlLip(built.upper, R, params.length);
      if (lip) {
        const extra = ((built as { extra?: SurfaceGrid[] }).extra ??= []);
        extra.push(lip);
      }
    }
    const zSign = params.lid === "bottom" ? -1 : 1;
    if (!isDuct) applyFins(built.upper, params, zSign);
    if (!isDuct) zipperSharpEdges(built.upper, built.lower, params.length);
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

  mesh = compactMesh(mesh);
  orientOutward(mesh);
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
  if (Rn > 0) notes.push(`Leading-edge radius R = ${(Rn * 1000).toFixed(1)} mm (circular fillet, G1 to the wetted sheets).`);
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

  return { params, mesh, grids, shockGrids: shock, quality, aero };
}
