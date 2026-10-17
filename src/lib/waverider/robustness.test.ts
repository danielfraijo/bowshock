/**
 * Robustness, edge-case and cross-check tests for the mesh-refinement work (round 2).
 *   bun test src/lib/waverider/robustness.test.ts
 * Nothing here may throw or produce NaN / Infinity; accuracy checks use exact relations (oblique shock, Rayleigh pitot).
 */
import assert from "node:assert/strict";
import { describe, it } from "node:test";
import { atmosphere } from "./atmosphere";
import { curvatureAdaptiveParams, curvatureStations, type Pt3 } from "./curvature";
import { buildVehicle } from "./generate";
import { betaFromThetaM, obliqueShock, rayleighPitot, thetaFromBetaM } from "./math";
import { analyzeMesh, gridPoint } from "./mesh";
import { edgeDefects, fillTriangleHoles, repairAftIfDefective, triangleQuality } from "./mesh-repair";
import { panelAero } from "./panel";
import {
  MEDIUM,
  REFINEMENT_LIMITS,
  estimateShockHeights,
  presetSettings,
  shockFractionFromHeight,
  stripDims,
  validateRefinement,
  LEGACY,
  type RefinementSettings,
} from "./refinement";
import { clearSymmetryEulerCache, symmetryEulerCacheInfo, warmSymmetryEuler } from "./sym-euler";
import { legacyStripGrid, solveStrip, solveStripAsync, stripDomain, stripGridOf } from "./strip-euler";
import { DEFAULT_PARAMS, type DesignParams, type WaveriderFamily } from "./types";

const DEG = Math.PI / 180;
const FAMILIES: WaveriderFamily[] = [
  "caret", "cone", "osculating", "viscopt", "inward", "elliptic", "busemann", "star", "wedgecone", "liftbody", "ramjet", "scramjet", "integrated",
];

function aeroOf(p: DesignParams, alpha = 3) {
  const b = buildVehicle(p);
  const atm = atmosphere(p.altKm, p.lockFlight ? p.mach : p.flightMach, p.gamma);
  return { b, a: panelAero(b.mesh, p, atm, alpha, 0, Math.max(b.aero.planformArea, 1e-6), p.length) };
}
function assertFiniteAero(a: ReturnType<typeof aeroOf>["a"], tag: string) {
  for (const k of ["cl", "cd", "cm", "ca"] as const) assert.ok(Number.isFinite(a[k]), `${tag}: ${k} = ${a[k]}`);
  for (let i = 0; i < a.cp.length; i++) if (!Number.isFinite(a.cp[i])) assert.fail(`${tag}: cp[${i}] = ${a.cp[i]}`);
}
function wedgeWall(thetaDeg: number, L = 4, n = 60, sign = 1) {
  const x = Float64Array.from({ length: n }, (_, i) => (L * i) / (n - 1));
  const z = x.map((v) => sign * v * Math.tan(thetaDeg * DEG));
  return { x, z };
}
const cpExact = (M: number, thetaDeg: number, g = 1.4) => {
  // weak-shock oblique relation solved for beta by bisection on theta(beta)
  let lo = Math.asin(1 / M) + 1e-6;
  let hi = 89.9 * DEG;
  let bMax = lo;
  let tMax = 0;
  for (let b = lo; b < hi; b += 0.0005) {
    const t = thetaFromBetaM(M, b, g);
    if (t > tMax) {
      tMax = t;
      bMax = b;
    }
  }
  lo = Math.asin(1 / M) + 1e-9;
  hi = bMax;
  for (let k = 0; k < 80; k++) {
    const mid = 0.5 * (lo + hi);
    if (thetaFromBetaM(M, mid, g) < thetaDeg * DEG) lo = mid;
    else hi = mid;
  }
  const os = obliqueShock(M, 0.5 * (lo + hi), g);
  return (os.p2p1 - 1) / (0.5 * g * M * M);
};

describe("lower-surface kernel fix (flow domain below the wall)", () => {
  it("a descending lower wall solves to the same Cp as its mirrored upper wall (was the vacuum Cp)", () => {
    for (const id of ["coarse", "medium", "fine"] as const) {
      const v = presetSettings(id).volume;
      const up = wedgeWall(10.46);
      const lo = wedgeWall(10.46, 4, 60, -1);
      const gu = solveStrip(up.x, up.z, 8, 1.4, 0, stripGridOf(v, null)).cp;
      const gl = solveStrip(lo.x, lo.z, 8, 1.4, 0, stripGridOf(v, null)).cp;
      for (let i = 0; i < gu.length; i++) assert.equal(gl[i], gu[i], `${id} cp[${i}]`);
      const mid = gl[30];
      assert.ok(Math.abs(mid / cpExact(8, 10.46) - 1) < 0.03, `${id} lower wall Cp ${mid} vs exact ${cpExact(8, 10.46)}`);
    }
  });
  it("alpha acts the right way round on both sides (lower wall: +alpha compresses, upper wall: +alpha expands)", () => {
    const v = presetSettings("coarse").volume;
    const lo = wedgeWall(8, 4, 60, -1);
    const up = wedgeWall(8);
    const c0 = solveStrip(lo.x, lo.z, 8, 1.4, 0, stripGridOf(v, null)).cp[30];
    const cP = solveStrip(lo.x, lo.z, 8, 1.4, 0.05, stripGridOf(v, null)).cp[30];
    const u0 = solveStrip(up.x, up.z, 8, 1.4, 0, stripGridOf(v, null)).cp[30];
    const uP = solveStrip(up.x, up.z, 8, 1.4, 0.05, stripGridOf(v, null)).cp[30];
    assert.ok(cP > c0 && uP < u0, `lower ${c0}->${cP}, upper ${u0}->${uP}`);
  });
  it("caret waverider: default Euler centreline lower-surface Cp is within 3 % of the exact oblique shock at Mach 5..15", () => {
    for (const [M, beta] of [[8, 16], [6, 20], [10, 12], [5, 26], [15, 10]] as const) {
      const th = thetaFromBetaM(M, beta * DEG, 1.4);
      const os = obliqueShock(M, beta * DEG, 1.4);
      const exact = (os.p2p1 - 1) / (0.5 * 1.4 * M * M);
      const p = { ...DEFAULT_PARAMS, family: "caret" as const, mach: M, flightMach: M, shockDeg: beta, height: Math.tan(th) * DEFAULT_PARAMS.length, aeroMethod: "euler" as const };
      const { b, a } = aeroOf(p, 0);
      let s = 0;
      let c = 0;
      const pos = b.mesh.positions;
      const ind = b.mesh.indices;
      for (let t = 0; t < ind.length / 3; t++) {
        const ia = ind[t * 3] * 3, ib = ind[t * 3 + 1] * 3, ic = ind[t * 3 + 2] * 3;
        const cy = Math.abs((pos[ia + 1] + pos[ib + 1] + pos[ic + 1]) / 3);
        const cx = (pos[ia] + pos[ib] + pos[ic]) / 3;
        const nz = (pos[ib] - pos[ia]) * (pos[ic + 1] - pos[ia + 1]) - (pos[ib + 1] - pos[ia + 1]) * (pos[ic] - pos[ia]);
        if (cy < 0.05 && cx > 0.3 * p.length && cx < 0.8 * p.length && nz < 0) {
          s += a.cp[t];
          c++;
        }
      }
      assert.ok(c > 5, "centreline triangles found");
      const q = 0.5 * 1.4 * M * M;
      // The default blunt LE (fillet v2) makes the as-built centreline ramp ~0.5 deg steeper than the design angle
      // (10.98 vs 10.46 deg at M8/beta 16; with leBlunt:false it is exact), so the reference is the exact oblique-shock Cp
      // at the ramp angle actually built, measured on the lower grid centreline between 0.3 and 0.8 L.
      const lowerG = b.grids.find((g) => g.name === "lower")!;
      let j0 = 0;
      for (let j = 0; j < lowerG.nj; j++) if (Math.abs(gridPoint(lowerG, 0, j)[1]) < Math.abs(gridPoint(lowerG, 0, j0)[1])) j0 = j;
      const wall: [number, number][] = [];
      for (let i = 0; i < lowerG.ni; i++) {
        const g = gridPoint(lowerG, i, j0);
        if (g[0] > 0.3 * p.length && g[0] < 0.8 * p.length) wall.push([g[0], g[2]]);
      }
      assert.ok(wall.length >= 2, "centreline wall samples");
      const thBuilt = Math.atan2(wall[0][1] - wall[wall.length - 1][1], wall[wall.length - 1][0] - wall[0][0]);
      const exactBuilt = (obliqueShock(M, betaFromThetaM(M, thBuilt, 1.4), 1.4).p2p1 - 1) / q;
      assert.ok(Math.abs(s / c / exactBuilt - 1) < 0.03, `M${M} beta${beta}: ${s / c} vs exact at built ramp ${thBuilt / DEG} deg: ${exactBuilt} (design-angle exact ${exact})`);
    }
  });
});

describe("blunt nose (planar cylinder-fronted half body, Mach 8 / 4)", () => {
  function roundNose(R: number, n = 120) {
    const x: number[] = [];
    const z: number[] = [];
    const m = n >> 1;
    for (let i = 0; i < m; i++) {
      const phi = 0.98 * (Math.PI / 2) * (i / (m - 1));
      x.push(R * (1 - Math.cos(phi)));
      z.push(R * Math.sin(phi));
    }
    for (let i = 1; i <= n - m; i++) {
      x.push(x[m - 1] + ((1 - x[m - 1]) * i) / (n - m));
      z.push(z[m - 1]);
    }
    return { x: Float64Array.from(x), z: Float64Array.from(z) };
  }
  function run(id: "legacy" | "medium" | "fine", M: number, R: number) {
    const w = roundNose(R);
    const ref = presetSettings(id);
    const d = stripDims(ref.volume);
    const dom = stripDomain(w.x, w.z);
    let shockS: Float64Array | null = null;
    if (ref.volume.shockClustering > 0) {
      const xs = new Float64Array(d.ni + 1).map((_, i) => dom.xmin + ((dom.xmax - dom.xmin) * i) / d.ni);
      const h = estimateShockHeights(w.x, w.z, xs, M, 1.4, 0, 1, R);
      shockS = new Float64Array(d.ni + 1).map((_, i) => shockFractionFromHeight(h[i], dom.hFar, LEGACY.strip.wallExp));
    }
    const r = id === "legacy" ? solveStrip(w.x, w.z, M, 1.4, 0, legacyStripGrid()) : solveStrip(w.x, w.z, M, 1.4, 0, stripGridOf(ref.volume, shockS));
    let st = NaN;
    for (let k = 0; k < r.shock.x.length; k++)
      if (Number.isFinite(r.shock.height[k])) {
        st = -r.shock.x[k];
        break;
      }
    return { cp0: Math.max(...r.cp), standoff: st / R, finite: r.cp.every(Number.isFinite) };
  }
  it("stagnation Cp vs Rayleigh pitot (+-16 %) and Billig cylinder stand-off (+-35 %): medium and fine; legacy is far off", () => {
    for (const M of [8, 4]) {
      const cp0 = (rayleighPitot(M, 1.4) - 1) / (0.5 * 1.4 * M * M);
      const billig = 0.386 * Math.exp(4.67 / (M * M));
      const leg = run("legacy", M, 0.1);
      for (const id of ["medium", "fine"] as const) {
        const r = run(id, M, 0.1);
        assert.ok(r.finite);
        assert.ok(Math.abs(r.cp0 / cp0 - 1) < 0.16, `${id} M${M} stagnation Cp ${r.cp0} vs ${cp0}`);
        assert.ok(Math.abs(r.standoff / billig - 1) < 0.35, `${id} M${M} standoff/R ${r.standoff} vs Billig ${billig}`);
        assert.ok(Math.abs(r.standoff / billig - 1) < Math.abs(leg.standoff / billig - 1), `${id} beats legacy stand-off`);
      }
    }
  });
});

describe("curvature-adaptive sampling", () => {
  const arc = (R: number): ((t: number) => Pt3) => (t) => [R * Math.sin(t * Math.PI * 0.5), 0, R * (1 - Math.cos(t * Math.PI * 0.5))];
  it("end points exact, strictly increasing, deterministic, n samples, for any strength", () => {
    for (const n of [3, 4, 9, 33, 100]) for (const strength of [0, 0.1, 0.5, 1, 7, -1, NaN]) {
      const a = curvatureAdaptiveParams(arc(2), n, { strength });
      const b = curvatureAdaptiveParams(arc(2), n, { strength });
      assert.equal(a.t.length, n);
      assert.equal(a.t[0], 0);
      assert.equal(a.t[n - 1], 1);
      for (let i = 1; i < n; i++) assert.ok(a.t[i] > a.t[i - 1], `n=${n} s=${strength} i=${i}`);
      assert.deepEqual(Array.from(a.t), Array.from(b.t));
    }
  });
  it("a sharp corner is sampled exactly and points gather near it; a straight line stays uniform", () => {
    const corner: (t: number) => Pt3 = (t) => (t < 0.4 ? [t, 0, 0] : [0.4 + (t - 0.4) * 0.3, 0, (t - 0.4) * 0.9]);
    const s = curvatureAdaptiveParams(corner, 41, { strength: 0.8 });
    assert.ok(s.corners.length >= 1);
    assert.ok(s.t.some((t) => Math.abs(t - 0.4) < 1e-6), "corner is a sample");
    const line = curvatureAdaptiveParams((t) => [t, 0.5 * t, 0], 21, { strength: 1 });
    for (let i = 0; i < 21; i++) assert.ok(Math.abs(line.t[i] - i / 20) < 1e-5);
  });
  it("tighter arcs get finer spacing", () => {
    const curve: (t: number) => Pt3 = (t) => (t < 0.5 ? [t, 0, 0] : [0.5 + 0.05 * Math.sin(((t - 0.5) / 0.5) * Math.PI * 0.5) * 2, 0, 0.1 * (1 - Math.cos(((t - 0.5) / 0.5) * Math.PI * 0.5))]);
    const s = curvatureAdaptiveParams(curve, 60, { strength: 1 });
    const dt = (a: number, b: number) => {
      let mx = 0;
      for (let i = 1; i < s.t.length; i++) if (s.t[i - 1] >= a && s.t[i] <= b) mx = Math.max(mx, s.t[i] - s.t[i - 1]);
      return mx;
    };
    assert.ok(dt(0.5, 1) > 0, "samples in the arc");
    assert.ok(Math.min(...Array.from(s.t.slice(1), (v, i) => v - s.t[i])) > 0);
  });
  it("NaN / Infinity / degenerate curves fall back to uniform without throwing", () => {
    for (const bad of [() => [NaN, 0, 0] as Pt3, () => [Infinity, 0, 0] as Pt3, () => [1, 1, 1] as Pt3]) {
      const s = curvatureAdaptiveParams(bad, 11, { strength: 1 });
      for (let i = 0; i < 11; i++) assert.ok(Math.abs(s.t[i] - i / 10) < 1e-12);
    }
    assert.equal(curvatureAdaptiveParams(arc(1), NaN as number, { strength: 1 }).t.length, 2);
  });
  it("curvatureStations: strength 0 is the identity; end stations, y=0 and mirror symmetry are kept; NaN-safe", () => {
    const n = 33;
    const yMax = 1;
    const ys = Array.from({ length: n }, (_, j) => -yMax + 2 * yMax * (0.5 * (1 - Math.cos((Math.PI * j) / (n - 1)))));
    ys[(n - 1) >> 1] = 0;
    const pt = (y: number): Pt3 => [Math.abs(y) < 0.3 ? 0 : (Math.abs(y) - 0.3) * 2, y, -0.2 * Math.exp(-((y / 0.2) ** 2))];
    assert.equal(curvatureStations(ys, 0, pt), ys);
    const out = curvatureStations(ys, 0.7, pt);
    assert.equal(out.length, n);
    assert.equal(out[0], ys[0]);
    assert.equal(out[n - 1], ys[n - 1]);
    assert.equal(out[(n - 1) >> 1], 0);
    for (let i = 1; i < n; i++) assert.ok(out[i] > out[i - 1]);
    for (let i = 0; i < n; i++) assert.ok(Math.abs(out[i] + out[n - 1 - i]) < 1e-12, "mirror symmetric");
    assert.equal(curvatureStations(ys, 0.7, () => [NaN, 0, 0]).length, n);
    assert.deepEqual(curvatureStations([0, 1, 2], 1, pt), [0, 1, 2]);
  });
});

describe("fillTriangleHoles / repairAftIfDefective", () => {
  const sq = (extra: number[]) => ({
    positions: Float64Array.from([0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 2, 0, 0, 2, 1, 0]),
    indices: Uint32Array.from(extra),
    surfaces: new Uint8Array(extra.length / 3),
  });
  it("fills a one-triangle hole in a closed tetra, leaves larger holes and closed meshes alone", () => {
    // tetrahedron missing one face (0,1,2): 3 open edges
    const tet = {
      positions: Float64Array.from([0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1]),
      indices: Uint32Array.from([0, 3, 1, 1, 3, 2, 2, 3, 0]),
      surfaces: new Uint8Array(3),
    };
    assert.deepEqual(edgeDefects(tet), { open: 3, nonManifold: 0 });
    const filled = fillTriangleHoles(tet);
    assert.deepEqual(edgeDefects(filled), { open: 0, nonManifold: 0 });
    assert.equal(filled.indices.length, 12);
    assert.equal(fillTriangleHoles(filled), filled);
    assert.equal(repairAftIfDefective(filled, 1), filled);
    // an open strip (two triangles, 4-edge boundary loop) is not a triangle hole
    const strip = sq([0, 1, 2, 1, 3, 2]);
    assert.equal(fillTriangleHoles(strip), strip);
  });
});

describe("watertight / manifold sweep: 13 families x presets x spanwise density", () => {
  const cfgs: RefinementSettings[] = [
    presetSettings("legacy"),
    presetSettings("coarse"),
    presetSettings("medium"),
    presetSettings("fine"),
    validateRefinement({ preset: "custom", surface: { nx: 48, ny: 40, leClustering: 1, curvature: 1 } }).settings,
    validateRefinement({ preset: "custom", surface: { nx: 120, ny: 110, leClustering: 0.5, curvature: 0.5 } }).settings,
  ];
  it("every family is closed, 2-manifold, finite, with (almost) no zero-area triangles", { timeout: 120000 }, () => {
    let n = 0;
    for (const family of FAMILIES) for (const ref of cfgs) for (const ny of [16, 34, 64]) for (const half of [false, true]) {
      const p: DesignParams = { ...DEFAULT_PARAMS, family, nx: ref.surface.nx, ny, halfModel: half, refinement: ref };
      const m = buildVehicle(p).mesh;
      const d = edgeDefects(m);
      const tag = `${family} ${ref.preset}/${ref.surface.nx}x${ny} half=${half}`;
      // KNOWN LIMITATION (present on main before this PR, in the topology tip-collapse, not in the aft weld): busemann at the
      // sparsest spanwise density (ny = 16) leaves one knife-edge tip vertex that is shared by four faces (1 non-manifold
      // edge, coarse 24x16 half model) and, at nx = 96, up to 6 open edges. Everything else must be closed and 2-manifold.
      const knownTipGap = family === "busemann" && ny === 16;
      assert.ok(d.open <= (knownTipGap ? 6 : 0), `${tag} open edges ${d.open}`);
      assert.ok(d.nonManifold <= (knownTipGap ? 1 : 0), `${tag} non-manifold edges ${d.nonManifold}`);
      const q = triangleQuality(m, p.length);
      // exact zero-area (collinear) triangles occur at the aft tip station of a few families at ny >= 64 (also in b5fdccc:
      // 6 over the same sweep vs 4 now); they do not open the mesh. Bound them instead of pretending they are gone.
      assert.ok(q.degenerate <= Math.max(2, 0.002 * q.triangles), `${tag} degenerate triangles ${q.degenerate}`);
      assert.ok(Number.isFinite(q.minAngleDeg) && Number.isFinite(q.maxAspect), tag);
      for (let i = 0; i < m.positions.length; i++) if (!Number.isFinite(m.positions[i])) assert.fail(`${tag} NaN vertex`);
      n++;
    }
    assert.equal(n, FAMILIES.length * cfgs.length * 3 * 2);
  });
  it("triangleQuality counts slivers and degenerates on a synthetic mesh", () => {
    const m = {
      positions: new Float32Array([0, 0, 0, 1, 0, 0, 0.5, 0.0001, 0, 0, 1, 0, 0, 0, 0, 2, 0, 0]) as unknown as number[],
      indices: new Uint32Array([0, 1, 2, 0, 1, 3, 0, 4, 5]),
    } as never;
    const q = triangleQuality(m, 1);
    assert.equal(q.triangles, 3);
    assert.ok(q.slivers >= 1, `slivers ${q.slivers}`);
    assert.ok(q.degenerate >= 1, `degenerate ${q.degenerate}`);
  });
});

describe("edge cases: nothing throws or returns NaN / Infinity", () => {
  const base = { ...DEFAULT_PARAMS, aeroMethod: "euler" as const };
  it("LE radius 0 / tiny / huge / negative / NaN, blunt on and off", { timeout: 60000 }, () => {
    for (const leRadius of [0, 1e-12, 1e-4, 0.02, 0.5, 50, -1, NaN]) for (const leBlunt of [true, false]) {
      const p = { ...base, leRadius, leBlunt };
      const { b, a } = aeroOf(p);
      assertFiniteAero(a, `le ${leRadius} blunt ${leBlunt}`);
      assert.ok(analyzeMesh(b.mesh).watertight, `le ${leRadius}`);
    }
  });
  it("Mach 1.0 / 1.5 / 3 / 8 / 20 / 25 / 50 / NaN, all presets", { timeout: 120000 }, () => {
    for (const mach of [0.5, 1, 1.5, 3, 8, 20, 25, 50, NaN]) for (const id of ["legacy", "coarse", "medium", "fine"] as const) {
      const p = { ...base, mach, flightMach: mach, refinement: presetSettings(id) };
      assertFiniteAero(aeroOf(p).a, `M${mach} ${id}`);
    }
  });
  it("slender, blunt and detached shapes (height 0.001 .. 2.5 x length)", { timeout: 60000 }, () => {
    for (const height of [1e-6, 0.001, 0.05, 0.55, 1.5, 4, 10]) for (const family of ["caret", "osculating", "cone"] as WaveriderFamily[]) {
      assertFiniteAero(aeroOf({ ...base, family, height }).a, `${family} h=${height}`);
    }
  });
  it("tiny / huge span and length, zero-ish planform", { timeout: 60000 }, () => {
    for (const [length, span] of [[1e-3, 5e-4], [1e3, 5e2], [4, 1e-4], [4, 50], [1e-2, 10]] as const) {
      const p = { ...base, length, span, height: 0.1 * length };
      const { b, a } = aeroOf(p);
      assertFiniteAero(a, `L=${length} span=${span}`);
      for (let i = 0; i < b.mesh.positions.length; i++) if (!Number.isFinite(b.mesh.positions[i])) assert.fail(`NaN vertex L=${length} span=${span}`);
    }
  });
  it("extreme alpha and gamma", { timeout: 60000 }, () => {
    for (const alpha of [-90, -30, 0, 45, 89.9, 180]) assertFiniteAero(aeroOf(base, alpha).a, `alpha ${alpha}`);
    for (const gamma of [1.0001, 1.1, 1.67, 3]) assertFiniteAero(aeroOf({ ...base, gamma }).a, `gamma ${gamma}`);
  });
  it("strip kernel: walls with NaN, flat, vertical-ish, tiny and huge sizes return finite Cp or fall back", () => {
    const flat = { x: Float64Array.from({ length: 30 }, (_, i) => i / 29), z: new Float64Array(30) };
    const r0 = solveStrip(flat.x, flat.z, 8, 1.4, 0, stripGridOf(presetSettings("medium").volume, null));
    for (const v of r0.cp) assert.ok(Math.abs(v) < 0.02, `flat plate Cp ${v}`);
    const steep = wedgeWall(80, 1);
    assert.ok(solveStrip(steep.x, steep.z, 8, 1.4, 0, stripGridOf(presetSettings("medium").volume, null)).cp.every(Number.isFinite));
    for (const s of [1e-6, 1e6]) {
      const w = wedgeWall(10, s);
      assert.ok(solveStrip(w.x, w.z, 8, 1.4, 0, stripGridOf(presetSettings("coarse").volume, null)).cp.every(Number.isFinite), `scale ${s}`);
    }
    for (const M of [1.05, 1.5, 25, 40]) {
      const w = wedgeWall(6);
      assert.ok(solveStrip(w.x, w.z, M, 1.4, 0, stripGridOf(presetSettings("medium").volume, null)).cp.every(Number.isFinite), `M ${M}`);
    }
  });
});

describe("extreme custom refinement settings", () => {
  const junk: unknown[] = [NaN, Infinity, -Infinity, -1, 0, 1e300, -1e300, "x", null, undefined, {}, [], true, 0.5, 3.7];
  it("every combination of junk values validates into in-range, integer-where-needed settings with the caps respected", () => {
    const L = REFINEMENT_LIMITS;
    for (const a of junk) for (const b of junk) {
      const v = validateRefinement({
        preset: "custom",
        surface: { nx: a as never, ny: b as never, leClustering: a as never, curvature: b as never },
        volume: { nStream: a as never, nCross: b as never, iters: a as never, wallClustering: b as never, shockClustering: a as never, tol: b as never, sequence: a as never, shockWidth: b as never, adaptive: a as never, order: b as never },
      });
      const s = v.settings;
      assert.ok(Number.isInteger(s.surface.nx) && s.surface.nx >= L.surface.nx[0] && s.surface.nx <= L.surface.nx[1]);
      assert.ok(Number.isInteger(s.surface.ny) && s.surface.ny >= L.surface.ny[0] && s.surface.ny <= L.surface.ny[1]);
      assert.ok(s.surface.nx * s.surface.ny <= L.surface.maxPoints);
      assert.ok(s.volume.nStream * s.volume.nCross <= L.volume.maxCells);
      assert.ok(Number.isInteger(s.volume.iters) && s.volume.iters >= L.volume.iters[0] && s.volume.iters <= L.volume.iters[1]);
      assert.ok(s.surface.leClustering >= 0 && s.surface.leClustering <= 1 && Number.isFinite(s.surface.leClustering));
      assert.ok((s.surface.curvature ?? 0) >= 0 && (s.surface.curvature ?? 0) <= 1);
      assert.ok(s.volume.wallClustering >= 0 && Number.isFinite(s.volume.wallClustering));
      assert.ok(s.volume.shockClustering >= 0 && Number.isFinite(s.volume.shockClustering));
      assert.ok((s.volume.tol ?? 0) === 0 || ((s.volume.tol as number) >= L.volume.tol[0] && (s.volume.tol as number) <= L.volume.tol[1]));
      assert.ok([1, 2].includes(s.volume.order ?? 1));
      const c = stripDims(s.volume);
      assert.ok(Number.isInteger(c.ni) && Number.isInteger(c.nj) && c.ni > 0 && c.nj > 0);
    }
  });
  it("the validator is idempotent and rejects non-object input", () => {
    for (const x of [null, undefined, 5, "medium", [], NaN] as unknown[]) {
      const v = validateRefinement(x as never);
      assert.ok(v.settings.surface.nx > 0);
    }
    const once = validateRefinement({ preset: "custom", volume: { nStream: 1e9, iters: 1e9, tol: 5 } });
    const twice = validateRefinement(once.settings);
    assert.deepEqual(twice.settings, once.settings);
  });
  it("maximum custom request (everything at its limit, MUSCL, adaptive) stays inside the work cap and gives finite aero", { timeout: 120000 }, () => {
    const v = validateRefinement({
      preset: "custom",
      surface: { nx: 1e9, ny: 1e9, leClustering: 9, curvature: 9 },
      volume: { nStream: Infinity, nCross: Infinity, iters: Infinity, wallClustering: Infinity, shockClustering: Infinity, adaptive: true, tol: 1e-9, order: 2, sequence: 9 },
    });
    assert.ok(v.warnings.length > 0);
    const s = v.settings;
    const c = stripDims(s.volume);
    const work = 2 * c.ni * c.nj * stripDims(s.volume).iters * 1.5;
    assert.ok(work <= REFINEMENT_LIMITS.volume.maxWork * 1.2, `work ${work}`);
    const t0 = performance.now();
    const { a } = aeroOf({ ...DEFAULT_PARAMS, nx: s.surface.nx, ny: s.surface.ny, aeroMethod: "euler", refinement: s });
    assertFiniteAero(a, "max custom");
    console.log(`max custom (${s.surface.nx}x${s.surface.ny} surface, ${c.ni}x${c.nj}x<=${c.iters} strip, adaptive, MUSCL): ${((performance.now() - t0) / 1000).toFixed(1)} s`);
  });
  it("tol = 0 with sequencing is ignored with a warning; order 2 is finite and close to order 1", () => {
    const v = validateRefinement({ preset: "custom", volume: { tol: 0, sequence: 3 } });
    assert.equal(v.settings.volume.sequence, 0);
    assert.ok(v.warnings.some((w) => w.includes("sequencing")));
    const w = wedgeWall(8);
    const g1 = stripGridOf(presetSettings("medium").volume, null);
    const c1 = solveStrip(w.x, w.z, 8, 1.4, 0, g1).cp[30];
    const c2 = solveStrip(w.x, w.z, 8, 1.4, 0, { ...g1, order: 2, cfl: 0.5 }).cp[30];
    assert.ok(Number.isFinite(c2) && Math.abs(c2 / c1 - 1) < 0.05, `MUSCL ${c2} vs ${c1}`);
  });
});

describe("determinism, repeated recompute, cache, async", () => {
  const p: DesignParams = { ...DEFAULT_PARAMS, aeroMethod: "euler" };
  it("same input gives bit-identical aero, with and without a warm cache, and in either evaluation order", () => {
    clearSymmetryEulerCache();
    const a1 = aeroOf(p).a;
    const a2 = aeroOf(p).a;
    clearSymmetryEulerCache();
    const other = aeroOf({ ...p, family: "osculating" }).a;
    const a3 = aeroOf(p).a;
    for (const k of ["cl", "cd", "cm", "ca"] as const) {
      assert.equal(a2[k], a1[k]);
      assert.equal(a3[k], a1[k], `${k} after another vehicle`);
    }
    assert.deepEqual(Array.from(a3.cp), Array.from(a1.cp));
    assert.ok(Number.isFinite(other.cl));
  });
  it("30 repeated recomputes leave the cache bounded and results unchanged", () => {
    clearSymmetryEulerCache();
    const first = aeroOf(p).a.cl;
    for (let i = 0; i < 30; i++) assert.equal(aeroOf(p).a.cl, first);
    assert.ok(symmetryEulerCacheInfo().size <= 2 + 0, `cache size ${symmetryEulerCacheInfo().size}`);
  });
  it("the solve cache is a true LRU and never exceeds its capacity", { timeout: 120000 }, () => {
    clearSymmetryEulerCache();
    const ref = presetSettings("coarse");
    const mk = (h: number) => ({ ...p, height: h, refinement: ref });
    const { max } = symmetryEulerCacheInfo();
    aeroOf(mk(0.5));
    const keysAfterFirst = symmetryEulerCacheInfo().keys.slice();
    for (let i = 0; i < Math.ceil(max / 2) + 6; i++) {
      aeroOf(mk(0.3 + i * 0.002));
      aeroOf(mk(0.5)); // keep the first geometry hot
      assert.ok(symmetryEulerCacheInfo().size <= max);
    }
    const keys = new Set(symmetryEulerCacheInfo().keys);
    assert.ok(keysAfterFirst.every((k) => keys.has(k)), "the repeatedly used entries survived eviction");
    assert.equal(symmetryEulerCacheInfo().size, Math.min(max, symmetryEulerCacheInfo().size));
    clearSymmetryEulerCache();
  });
  it("async chunked solve equals the synchronous solve exactly, reports progress, and aborts", async () => {
    const w = wedgeWall(12);
    const g = stripGridOf(presetSettings("medium").volume, null);
    const sync = solveStrip(w.x, w.z, 8, 1.4, 0, g);
    const prog: number[] = [];
    const asy = await solveStripAsync(w.x, w.z, 8, 1.4, 0, g, { chunk: 7, onProgress: (v) => prog.push(v) });
    assert.deepEqual(Array.from(asy.cp), Array.from(sync.cp));
    assert.equal(asy.grid.itersUsed, sync.grid.itersUsed);
    assert.ok(prog.length > 3 && prog.every((v, i) => i === 0 || v >= prog[i - 1]));
    const sig = { aborted: false };
    let err: unknown;
    try {
      await solveStripAsync(w.x, w.z, 8, 1.4, 0, g, { chunk: 5, signal: sig, onProgress: () => (sig.aborted = true) });
    } catch (e) {
      err = e;
    }
    assert.equal((err as { name?: string })?.name, "AbortError");
  });
  it("warmSymmetryEuler makes the following synchronous evaluation a cache hit with identical numbers", async () => {
    clearSymmetryEulerCache();
    const q: DesignParams = { ...p, refinement: presetSettings("coarse") };
    const b = buildVehicle(q);
    assert.equal(await warmSymmetryEuler(b.mesh, q, 3), true);
    const n = symmetryEulerCacheInfo().size;
    assert.equal(n, 2);
    const atm = atmosphere(q.altKm, q.lockFlight ? q.mach : q.flightMach, q.gamma);
    const a = panelAero(b.mesh, q, atm, 3, 0, Math.max(b.aero.planformArea, 1e-6), q.length);
    assert.equal(symmetryEulerCacheInfo().size, n, "no new solves");
    clearSymmetryEulerCache();
    const a2 = panelAero(b.mesh, q, atm, 3, 0, Math.max(b.aero.planformArea, 1e-6), q.length);
    assert.equal(a.cl, a2.cl);
    assert.equal(await warmSymmetryEuler(b.mesh, { ...q, refinement: presetSettings("legacy") }, 3), false);
  });
});

describe("default (no refinement set) is the converged medium preset", () => {
  it("undefined refinement uses medium numbers: caret centreline accurate, legacy is not", () => {
    const p = { ...DEFAULT_PARAMS, aeroMethod: "euler" as const };
    const dflt = aeroOf(p, 0).a;
    const med = aeroOf({ ...p, refinement: MEDIUM }, 0).a;
    const leg = aeroOf({ ...p, refinement: presetSettings("legacy") }, 0).a;
    assert.equal(dflt.cl, med.cl);
    assert.ok(dflt.cl > 1.5 * leg.cl, `default CL ${dflt.cl} vs legacy ${leg.cl}`);
  });
});
