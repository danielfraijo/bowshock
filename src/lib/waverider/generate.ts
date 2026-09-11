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
import { MeshBuilder, analyzeMesh, gridPoint, makeGrid, orientOutward, stitchGrid, zipperSharpEdges } from "./mesh";

function halfSpanList(ny: number, s: number, half: boolean): number[] {
  const n = Math.max(half ? 4 : 5, half ? Math.ceil(ny / 2) : ny);
  const ys: number[] = [];
  for (let j = 0; j < n; j++) {
    const t = cosineSpace(j, n);
    ys.push(half ? s * t : -s + 2 * s * t);
  }
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
  return Math.min(Math.max(x, 0), L);
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
) {
  stitchGrid(b, upper, "upper", false);
  stitchGrid(b, lower, "lower", true);

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

  if (!flowThrough) {
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

function applyBlunt(upper: SurfaceGrid, lower: SurfaceGrid, radius: number, L: number) {
  if (radius <= 1e-8) return;
  const r = Math.min(radius, 0.08 * L);
  const nj = upper.nj;
  for (let j = 0; j < nj; j++) {
    const u0 = gridPoint(upper, 0, j);
    const u1 = gridPoint(upper, Math.min(1, upper.ni - 1), j);
    const l1 = gridPoint(lower, Math.min(1, lower.ni - 1), j);
    const dxU = u1[0] - u0[0];
    const dzU = u1[2] - u0[2];
    const dxL = l1[0] - u0[0];
    const dzL = l1[2] - u0[2];
    const nU = Math.hypot(dxU, dzU) || 1;
    const nL = Math.hypot(dxL, dzL) || 1;
    const bisX = dxU / nU + dxL / nL;
    const bisZ = dzU / nU + dzL / nL;
    const bn = Math.hypot(bisX, bisZ) || 1;
    const o = j * 3;
    upper.xyz[o] = u0[0] + (bisX / bn) * r;
    upper.xyz[o + 2] = u0[2] + (bisZ / bn) * r;
    lower.xyz[o] = u0[0] + (bisX / bn) * r;
    lower.xyz[o + 2] = u0[2] + (bisZ / bn) * r;
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
  const ys = halfSpanList(params.ny, s, params.halfModel);
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
  const ys = halfSpanList(params.ny, 1, params.halfModel);
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

function rampFloor(x: number, L: number, th: number, nRamps: number, xEnd: number) {
  const n = clamp(Math.round(nRamps || 2), 1, 3);
  if (x <= 0) return 0;
  const dx = xEnd / n;
  let z = 0;
  let x0 = 0;
  for (let k = 0; k < n; k++) {
    const x1 = dx * (k + 1);
    const slope = Math.tan((th * (k + 1)) / n);
    if (x <= x1 + 1e-12) return z - (x - x0) * slope;
    z -= (x1 - x0) * slope;
    x0 = x1;
  }
  return z - (x - xEnd) * 0;
}

function buildDuct(params: DesignParams, scram: boolean) {
  const L = params.length;
  const s = params.span / 2;
  const hIn = Math.max(0.04, params.inletHeight);
  const hMax = Math.max(hIn * 1.2, params.height);
  const th = params.rampDeg * DEG;
  const nR = clamp(Math.round(params.nRamps || 2), 1, 3);
  const xRamp = L * (0.1 + 0.07 * nR);
  const x2 = L * clamp(params.cowlFrac, 0.28, 0.58);
  const x3 = Math.min(L * (scram ? 0.88 : 0.92), x2 + L * clamp(params.combustorFrac, 0.1, 0.4));
  const ER = Math.max(1.4, params.nozzleER);
  const zIso = rampFloor(xRamp, L, th, nR, xRamp) - 1e-4;
  const zCombEnd = scram ? zIso * 1.08 : zIso;
  const zExit = zIso * ER;
  const zBot = (x: number) => {
    if (x <= xRamp) return rampFloor(x, L, th, nR, xRamp);
    if (x <= x2) return zIso;
    if (x <= x3) {
      const u = (x - x2) / Math.max(x3 - x2, 1e-9);
      return lerp(zIso, zCombEnd, u);
    }
    const u = (x - x3) / Math.max(L - x3, 1e-9);
    return lerp(zCombEnd, Math.min(zExit, -hMax), u);
  };
  const zTop = (x: number) => {
    const cowl = 0.012 * hMax;
    if (x < x2) return cowl + hIn * 0.15 * (1 - x / Math.max(x2, 1e-9));
    if (x < x3) return cowl;
    const u = (x - x3) / Math.max(L - x3, 1e-9);
    return lerp(cowl, cowl + 0.25 * hIn * (ER - 1), u);
  };
  const ys = halfSpanList(params.ny, s, params.halfModel);
  const nx = Math.max(10, params.nx);
  const nj = ys.length;
  const upper = makeGrid("upper", nx, nj, (i, j) => {
    const x = (i / (nx - 1)) * L;
    return [x, ys[j], zTop(x)];
  });
  const lower = makeGrid("lower", nx, nj, (i, j) => {
    const x = (i / (nx - 1)) * L;
    return [x, ys[j], zBot(x)];
  });
  return { upper, lower, shock: [] as SurfaceGrid[] };
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
    stitchGrid(b, gA, "upper", false);
    stitchGrid(b, gB, "lower", false);
    grids.push(gA, gB);
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

function snapNoseToOrigin(mesh: { positions: Float64Array }, grids: SurfaceGrid[], shock: SurfaceGrid[]) {
  const p = mesh.positions;
  let xmin = Infinity;
  for (let i = 0; i < p.length; i += 3) if (p[i] < xmin) xmin = p[i];
  let bestAbsY = Infinity;
  let zTip = 0;
  for (let i = 0; i < p.length; i += 3) {
    if (p[i] - xmin > 1e-8) continue;
    const ay = Math.abs(p[i + 1]);
    if (ay < bestAbsY) {
      bestAbsY = ay;
      zTip = p[i + 2];
    }
  }
  const shift = (a: Float64Array) => {
    for (let i = 0; i < a.length; i += 3) {
      a[i] -= xmin;
      a[i + 2] -= zTip;
    }
  };
  shift(p);
  for (const g of grids) shift(g.xyz);
  for (const g of shock) shift(g.xyz);
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
    applyBlunt(built.upper, built.lower, params.leRadius, params.length);
    applyElevon(built.upper, built.lower, params.elevonDeg || 0, params.length);
    const duct = params.flowThrough && (params.family === "ramjet" || params.family === "scramjet");
    if (!duct) zipperSharpEdges(built.upper, built.lower, params.length);
    const zSign = params.lid === "bottom" ? -1 : 1;
    applyFins(built.upper, params, zSign);
    if (!duct) zipperSharpEdges(built.upper, built.lower, params.length);
    const b = new MeshBuilder(params.length);
    closeVehicle(b, built.upper, built.lower, params.halfModel, duct);
    mesh = b.finish();
    skipped = b.skipped;
    grids = [built.upper, built.lower];
    shock = built.shock;
    if (duct) notes.push("Flow-through duct — inlet and nozzle left open for internal CFD.");
  }

  snapNoseToOrigin(mesh, grids, shock);
  notes.push("Tip snapped to the origin: X stream, Y span, Z up.");

  const quality = analyzeMesh(mesh, skipped);
  if (!quality.watertight) notes.push(`Mesh has ${quality.openEdges} open edges — raise streamwise/spanwise points.`);
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
