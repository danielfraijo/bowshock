/**
 * Mesh-refinement tests. node:test + node:assert, so they run under
 *   bun test src/lib/waverider/refinement.test.ts
 *   node --experimental-strip-types --test src/lib/waverider/refinement.test.ts   (Node >= 22.6)
 */
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { describe, it } from "node:test";
import { eulerWedge, wedgeShockBias } from "./euler2d";
import { EULER_WASM_B64 } from "./euler-wasm";
import { buildVehicle } from "./generate";
import { analyzeMesh } from "./mesh";
import { atmosphere } from "./atmosphere";
import { panelAero } from "./panel";
import {
  LEGACY,
  LEGACY_SETTINGS,
  MEDIUM,
  REFINEMENT_LIMITS,
  clusterCoordinates,
  clusterFractionsToLE,
  estimateCost,
  isBaseline,
  presetSettings,
  spacingStats,
  stripDims,
  validateRefinement,
  type VolumeRefinement,
} from "./refinement";
import { stripStudy, surfaceStudy, wedgeStudy } from "./refinement-study";
import { legacyStripGrid, solveStrip } from "./strip-euler";
import { legacyStripGridRef, solveStripRef } from "./strip-euler-ref";
import { DEFAULT_PARAMS, type WaveriderFamily } from "./types";

const fixture = JSON.parse(
  readFileSync(new URL("./__baseline__/baseline-topology-physics-45e7c83.json", import.meta.url), "utf8"),
);

function fnv(arr: ArrayLike<number>): string {
  const f = new Float64Array(arr.length);
  for (let i = 0; i < arr.length; i++) f[i] = arr[i];
  const u = new Uint8Array(f.buffer);
  let h = 0x811c9dc5;
  for (let i = 0; i < u.length; i++) {
    h ^= u[i];
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h.toString(16).padStart(8, "0");
}

describe("validation and clamping", () => {
  it("named presets are authoritative and unknown presets fall back to medium", () => {
    const a = validateRefinement({ preset: "fine", volume: { nStream: 9999 } });
    assert.deepEqual(a.settings, presetSettings("fine"));
    assert.equal(a.clamped, false);
    const b = validateRefinement({ preset: "ultra" as never });
    assert.equal(b.settings.preset, "medium");
    assert.ok(b.warnings.some((w) => w.includes("Unknown preset")));
    assert.deepEqual(validateRefinement(undefined).settings, MEDIUM);
    assert.deepEqual(validateRefinement(null).settings, MEDIUM);
  });

  it("custom values are clamped to the limits and rounded", () => {
    const lim = REFINEMENT_LIMITS;
    const lo = validateRefinement({
      preset: "custom",
      surface: { nx: 1, ny: 1, leClustering: -3 },
      volume: {
        nStream: 2,
        nCross: 1,
        iters: 1,
        wallClustering: -1,
        shockClustering: -1,
        shockWidth: 0,
      },
    });
    assert.equal(lo.settings.surface.nx, lim.surface.nx[0]);
    assert.equal(lo.settings.surface.ny, lim.surface.ny[0]);
    assert.equal(lo.settings.surface.leClustering, 0);
    assert.equal(lo.settings.volume.nStream, lim.volume.nStream[0]);
    assert.equal(lo.settings.volume.nCross, lim.volume.nCross[0]);
    assert.equal(lo.settings.volume.iters, lim.volume.iters[0]);
    assert.equal(lo.settings.volume.wallClustering, 0);
    assert.equal(lo.settings.volume.shockClustering, 0);
    assert.equal(lo.settings.volume.shockWidth, lim.volume.shockWidth[0]);
    assert.equal(lo.clamped, true);
    const hi = validateRefinement({
      preset: "custom",
      volume: { wallClustering: 99, shockClustering: 99, shockWidth: 5 },
    });
    assert.equal(hi.settings.volume.wallClustering, lim.volume.wallClustering[1]);
    assert.equal(hi.settings.volume.shockClustering, lim.volume.shockClustering[1]);
    assert.equal(hi.settings.volume.shockWidth, lim.volume.shockWidth[1]);
    const r = validateRefinement({
      preset: "custom",
      volume: { nStream: 100.6, nCross: 30.2, iters: 301.5 },
    });
    assert.equal(r.settings.volume.nStream, 101);
    assert.equal(r.settings.volume.nCross, 30);
    assert.equal(r.settings.volume.iters, 302);
  });

  it("NaN / Infinity / wrong types fall back to the medium value with a warning", () => {
    const v = validateRefinement({
      preset: "custom",
      surface: { nx: NaN },
      volume: { nStream: Infinity, iters: "x" as never },
    });
    assert.equal(v.settings.surface.nx, LEGACY.surface.nx);
    // +Infinity means "as much as allowed" (clamped, not garbage); NaN / strings fall back to the default
    assert.ok(v.settings.volume.nStream <= REFINEMENT_LIMITS.volume.nStream[1]);
    assert.ok(v.settings.volume.nStream > LEGACY.wedge.ni);
    assert.ok(Number.isInteger(v.settings.volume.iters) && v.settings.volume.iters > 0);
    assert.equal(validateRefinement({ preset: "custom", volume: { iters: "x" as never } }).settings.volume.iters, MEDIUM.volume.iters);
    assert.ok(v.warnings.length >= 2);
  });

  it("empty custom input equals medium numbers", () => {
    const v = validateRefinement({ preset: "custom" });
    assert.equal(v.settings.preset, "custom");
    assert.deepEqual(v.settings.volume, { ...MEDIUM.volume, order: 1 });
    assert.deepEqual(v.settings.surface, { ...MEDIUM.surface, curvature: 0 });
    assert.ok(!isBaseline(v.settings), "medium is no longer the legacy run");
  });
});

describe("caps", () => {
  it("total volume cells and total work stay under the hard caps for any request", () => {
    const L = REFINEMENT_LIMITS.volume;
    const asks: Partial<VolumeRefinement>[] = [
      { nStream: 400, nCross: 160, iters: 4000 },
      { nStream: 400, nCross: 160, iters: 20 },
      { nStream: 300, nCross: 100, iters: 4000, adaptive: true },
      { nStream: 24, nCross: 8, iters: 4000 },
      { nStream: 250, nCross: 90, iters: 1200, adaptive: true },
    ];
    for (const ask of asks) {
      const v = validateRefinement({ preset: "custom", volume: ask });
      const c = estimateCost(v.settings);
      assert.ok(c.wedgeCells <= L.maxCells, `cells ${c.wedgeCells} for ${JSON.stringify(ask)}`);
      assert.ok(
        Math.max(c.wedgeWork, c.stripWork) <= L.maxWork,
        `work ${Math.max(c.wedgeWork, c.stripWork)} for ${JSON.stringify(ask)}`,
      );
      assert.ok(c.withinCaps);
      assert.ok(
        v.settings.volume.nStream >= L.nStream[0] &&
          v.settings.volume.nCross >= L.nCross[0] &&
          v.settings.volume.iters >= L.iters[0],
      );
    }
    const big = validateRefinement({ preset: "custom", volume: { nStream: 400, nCross: 160 } });
    assert.ok(big.clamped);
    assert.ok(big.warnings.some((w) => /cap/i.test(w)));
  });

  it("surface points per sheet are capped and reported", () => {
    const v = validateRefinement({ preset: "custom", surface: { nx: 160, ny: 120 } });
    assert.ok(v.settings.surface.nx * v.settings.surface.ny <= REFINEMENT_LIMITS.surface.maxPoints);
    assert.ok(v.warnings.some((w) => w.includes("capped")));
  });

  it("the three presets are inside every cap and their estimates are ordered", () => {
    const c = (["coarse", "medium", "fine"] as const).map((id) => estimateCost(presetSettings(id)));
    for (const x of c) assert.ok(x.withinCaps);
    assert.ok(c[0].wedgeCells < c[1].wedgeCells && c[1].wedgeCells < c[2].wedgeCells);
    assert.ok(
      c[0].surfaceTrianglesApprox < c[1].surfaceTrianglesApprox &&
        c[1].surfaceTrianglesApprox < c[2].surfaceTrianglesApprox,
    );
    assert.ok(c[1].stripSeconds < c[2].stripSeconds);
  });

  it("legacy maps to the shipped kernel grid exactly; medium keeps the cell counts but converges", () => {
    assert.deepEqual(stripDims(LEGACY_SETTINGS.volume), { ni: 88, nj: 28, iters: 42 });
    assert.equal(LEGACY_SETTINGS.volume.nStream, 72);
    assert.equal(LEGACY_SETTINGS.volume.nCross, 24);
    assert.equal(LEGACY_SETTINGS.volume.iters, 280);
    assert.ok(isBaseline(LEGACY_SETTINGS) && !isBaseline(MEDIUM));
    const md = stripDims(MEDIUM.volume);
    assert.equal(md.ni, 88);
    assert.equal(md.nj, 28);
    assert.ok(md.iters > 42 && (MEDIUM.volume.tol ?? 0) > 0);
    assert.equal(MEDIUM.surface.nx, DEFAULT_PARAMS.nx);
    assert.equal(MEDIUM.surface.ny, DEFAULT_PARAMS.ny);
  });
});

describe("clustering", () => {
  it("is strictly increasing with exact endpoints for a sweep of specs, and off is uniform", () => {
    for (const n of [8, 24, 48, 160, 400]) {
      for (const wall of [0, 1, 4, 12]) {
        for (const shock of [0, 3, 10, 30]) {
          for (const pos of [0.04, 0.3, 0.6, 0.92]) {
            for (const w of [0.02, 0.09, 0.3]) {
              const c = clusterCoordinates(n, { wall, shock, shockPos: pos, shockWidth: w });
              assert.equal(c.length, n + 1);
              assert.equal(c[0], 0);
              assert.equal(c[n], 1);
              assert.ok(
                spacingStats(c).strictlyIncreasing,
                `n=${n} wall=${wall} shock=${shock} pos=${pos} w=${w}`,
              );
            }
          }
        }
      }
    }
    const u = clusterCoordinates(30, { wall: 0, shock: 0, shockPos: 0.5, shockWidth: 0.1 });
    for (let j = 0; j <= 30; j++) assert.equal(u[j], j / 30);
  });

  it("puts finer spacing at the wall and at the shock, with smooth growth", () => {
    const n = 48;
    const c = clusterCoordinates(n, { wall: 4, shock: 8, shockPos: 0.4, shockWidth: 0.08 });
    const d = (j: number) => c[j + 1] - c[j];
    const uniform = 1 / n;
    assert.ok(d(0) < 0.6 * uniform, `wall spacing ${d(0)}`);
    let jShock = 0;
    while (c[jShock + 1] < 0.4) jShock++;
    assert.ok(d(jShock) < 0.4 * uniform, `shock spacing ${d(jShock)}`);
    assert.ok(d(n - 1) > uniform, "far field is coarser than uniform");
    assert.ok(
      spacingStats(c).maxAdjacentRatio < 1.7,
      `adjacent ratio ${spacingStats(c).maxAdjacentRatio}`,
    );
  });

  it("preset clustering stays smooth (adjacent cells differ < 2x); extreme custom values draw a warning", () => {
    const f = presetSettings("fine").volume;
    const c = clusterCoordinates(f.nCross, {
      wall: f.wallClustering,
      shock: f.shockClustering,
      shockPos: 0.3,
      shockWidth: f.shockWidth,
    });
    assert.ok(
      spacingStats(c).maxAdjacentRatio < 2,
      `fine preset ratio ${spacingStats(c).maxAdjacentRatio}`,
    );
    assert.ok(
      !validateRefinement({
        preset: "custom",
        volume: { nCross: 32, shockClustering: 6, wallClustering: 2 },
      }).warnings.some((w) => w.includes("Aggressive")),
    );
    const x = validateRefinement({
      preset: "custom",
      volume: { nCross: 32, shockClustering: 30, shockWidth: 0.02 },
    });
    assert.ok(x.warnings.some((w) => w.includes("Aggressive")));
  });

  it("wall-only clustering is monotone in strength", () => {
    let prev = Infinity;
    for (const wall of [0, 1, 2, 4, 8]) {
      const c = clusterCoordinates(32, { wall, shock: 0, shockPos: 0.5, shockWidth: 0.1 });
      assert.ok(c[1] - c[0] < prev + 1e-15);
      prev = c[1] - c[0];
    }
  });

  it("LE stretching keeps fractions strictly increasing, endpoints exact, and is finer at the LE", () => {
    const n = 40;
    const base = new Float64Array(n);
    for (let i = 0; i < n; i++) base[i] = 0.5 * (1 - Math.cos((Math.PI * i) / (n - 1)));
    assert.equal(
      clusterFractionsToLE(base, 0),
      base,
      "strength 0 returns the input array untouched",
    );
    let prev = base[1] - base[0];
    for (const s of [0.2, 0.5, 0.8, 1]) {
      const f = clusterFractionsToLE(base, s);
      assert.equal(f[0], 0);
      assert.equal(f[n - 1], 1);
      assert.ok(spacingStats(f).strictlyIncreasing);
      assert.ok(f[1] - f[0] < prev, `LE gap should shrink with strength (s=${s})`);
      prev = f[1] - f[0];
      // finer near the LE than the same index of the original, coarser toward the TE
      assert.ok(f[3] < base[3]);
    }
  });
});

describe("legacy == baseline (captured from 45e7c83 = topology + physics-datapack, before the refinement hardening)", () => {
  it("wedge anchor: default and explicit-legacy results are bitwise identical to the base", () => {
    for (const key of Object.keys(fixture.wedge)) {
      const [M, th] = key.split("|").map(Number);
      const want = fixture.wedge[key];
      for (const vol of [undefined, LEGACY_SETTINGS.volume]) {
        const r = eulerWedge(M, th, 1.4, vol);
        assert.equal(r.p2p1, want.p2p1, key);
        assert.equal(r.relErr, want.relErr, key);
        assert.equal(r.cp, want.cp, key);
        assert.equal(r.residual, want.residual, key);
        assert.equal(r.iters, want.iters, key);
      }
    }
  });

  it("vehicles: mesh (any refinement) and Euler aero (legacy preset) are bitwise identical to the base", () => {
    for (const key of Object.keys(fixture.vehicles)) {
      const [family, alpha] = key.split("|");
      const want = fixture.vehicles[key];
      for (const refinement of [undefined, presetSettings("legacy")]) {
        const params = {
          ...DEFAULT_PARAMS,
          family: family as WaveriderFamily,
          alphaDeg: Number(alpha),
          refinement,
        };
        const b = buildVehicle(params);
        const q = analyzeMesh(b.mesh);
        if (want.manifold === false) {
          // The only intended change to a default mesh: the base build was NON-manifold here (aft-tip weld,
          // wedgecone). weldAftRobust repairs it; everything else about the case must stay close.
          assert.equal(q.manifold, true, `${key} is repaired`);
          assert.ok(Math.abs(q.triangles - want.triangles) <= 0.03 * want.triangles, `${key} triangles`);
          if (!refinement) continue;
          const atm0 = atmosphere(params.altKm, params.mach, params.gamma);
          const a0 = panelAero(b.mesh, params, atm0, Number(alpha), 0, Math.max(b.aero.planformArea, 1e-6), params.length);
          assert.ok(Math.abs(a0.cl - want.cl) <= 0.03 * Math.abs(want.cl) + 1e-4, `${key} cl ${a0.cl} vs ${want.cl}`);
          assert.ok(Math.abs(a0.cd - want.cd) <= 0.03 * Math.abs(want.cd) + 1e-4, `${key} cd ${a0.cd} vs ${want.cd}`);
          continue;
        }
        assert.equal(q.triangles, want.triangles, key);
        assert.equal(q.vertices, want.vertices, key);
        assert.equal(q.watertight, want.watertight, key);
        assert.equal(q.manifold, want.manifold, key);
        assert.equal(fnv(b.mesh.positions), want.posHash, `${key} positions`);
        assert.equal(fnv(Array.from(b.mesh.indices)), want.idxHash, `${key} indices`);
        if (!refinement) continue; // default = medium: the mesh is identical, the Euler aero is now converged (see below)
        const atm = atmosphere(params.altKm, params.mach, params.gamma);
        const a = panelAero(
          b.mesh,
          params,
          atm,
          Number(alpha),
          0,
          Math.max(b.aero.planformArea, 1e-6),
          params.length,
        );
        // Some base results depend on what ran earlier in the process (a coarse cache in math.ts's cone-shock model, plus the
        // base's own colliding Euler cache key - see NOTES.md), so osculating / cone are compared to 2 % (they differ from a
        // fresh-process run by up to ~1 % in measured cases); caret and liftbody are history-free and must match bit for bit.
        const exact = family === "caret" || family === "liftbody";
        const same = (got: number, ref: number, what: string) =>
          exact ? assert.equal(got, ref, `${key} ${what}`) : assert.ok(Math.abs(got - ref) <= 2e-2 * Math.abs(ref) + 1e-9, `${key} ${what} ${got} vs ${ref}`);
        same(a.cl, want.cl, "cl");
        same(a.cd, want.cd, "cd");
        same(a.cm, want.cm, "cm");
        same(a.ca, want.ca, "ca");
        if (exact) assert.equal(fnv(Array.from(a.cp)), want.cpHash, `${key} cp`);
        assert.equal(a.method, want.method);
        assert.ok(
          !a.notes.some((n) => n.startsWith("Mesh refinement")),
          "legacy adds no refinement note",
        );
      }
    }
  });

  it("reference TypeScript port reproduces the shipped wasm (1e-12); the optimised kernel too on upper walls", () => {
    const bin = atob(EULER_WASM_B64);
    const bytes = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    const ex = new WebAssembly.Instance(new WebAssembly.Module(bytes), {}).exports as unknown as {
      euler_alloc: (n: number) => number;
      euler_solve: (p: number, n: number, m: number, g: number, a: number) => number;
      memory: WebAssembly.Memory;
    };
    const n = 60;
    const L = 4;
    const walls: [string, (x: number) => number, number][] = [
      ["wedge8 upper", (x) => x * Math.tan((8 * Math.PI) / 180), 0],
      ["wedge8 lower a=.05", (x) => -x * Math.tan((8 * Math.PI) / 180), 0.05],
      ["curved", (x) => 0.075 * (x / L) ** 1.5 * L, 0.03],
    ];
    for (const [name, f, alpha] of walls) {
      const x = Array.from({ length: n }, (_, i) => (L * i) / (n - 1));
      const z = x.map(f);
      const ptr = ex.euler_alloc((n * 3 + 16) * 8);
      const view = new Float64Array(ex.memory.buffer, ptr, n * 2);
      for (let i = 0; i < n; i++) {
        view[i * 2] = x[i];
        view[i * 2 + 1] = z[i];
      }
      const res = ex.euler_solve(ptr, n, 8, 1.4, alpha);
      const cp = new Float64Array(ex.memory.buffer, ptr + n * 16, n);
      const ref = solveStripRef(x, z, 8, 1.4, alpha, legacyStripGridRef());
      const ts = solveStrip(x, z, 8, 1.4, alpha, legacyStripGrid());
      const upper = z[n >> 1] >= 0;
      for (let i = 0; i < n; i++) {
        assert.ok(Math.abs(cp[i] - ref.cp[i]) < 1e-12, `${name} cp[${i}] wasm ${cp[i]} ref ${ref.cp[i]}`);
        // the optimised kernel mirrors lower walls (the wasm / reference do not: that is the fixed bug), so compare upper walls
        if (upper) assert.ok(Math.abs(cp[i] - ts.cp[i]) < 1e-12, `${name} cp[${i}] wasm ${cp[i]} ts ${ts.cp[i]}`);
      }
      assert.ok(Math.abs(res - ref.residual) < 1e-9 * Math.max(1, Math.abs(res)), name);
      if (upper) assert.ok(Math.abs(res - ts.residual) < 1e-9 * Math.max(1, Math.abs(res)), name);
    }
  });

  it("cross-language goldens match the Rust crate tests", () => {
    const c = clusterCoordinates(10, { wall: 2, shock: 6, shockPos: 0.3, shockWidth: 0.09 });
    const want = [
      0, 0.0913287796753181, 0.19645591394704617, 0.24861662385687627, 0.2831088829975087,
      0.3140255579974827, 0.3483595736980215, 0.40178301370276004, 0.5627519821242225,
      0.7805946023081516, 1,
    ];
    for (let j = 0; j <= 10; j++) assert.ok(Math.abs(c[j] - want[j]) < 1e-12, `j=${j}`);
    const n = 60;
    const x = Array.from({ length: n }, (_, i) => (4 * i) / (n - 1));
    const z = x.map((v) => v * Math.tan((8 * Math.PI) / 180));
    const ni = 64;
    const shockS = new Float64Array(ni + 1).map((_, i) => 0.1 + (0.4 * i) / ni);
    const r = solveStrip(x, z, 8, 1.4, 0, {
      ...legacyStripGrid(),
      ni,
      nj: 20,
      iters: 150,
      wallClustering: 2,
      shockClustering: 6,
      shockWidth: 0.09,
      shockS,
    });
    assert.ok(Math.abs(r.cp[5] - 0.07180176310553892) < 1e-9);
    assert.ok(Math.abs(r.cp[30] - 0.06338922395989954) < 1e-9);
    assert.ok(Math.abs(r.cp[55] - 0.06307605241013038) < 1e-9);
    assert.ok(Math.abs(r.residual - 0.3041611260342723) < 1e-9);
  });
});

describe("surface presets", () => {
  it("triangle counts grow with the preset; watertight everywhere; manifold for the default family", () => {
    const rows = surfaceStudy(["caret", "osculating", "cone"], ["coarse", "medium", "fine"]);
    for (const fam of ["caret", "osculating", "cone"]) {
      const r = rows.filter((x) => x.family === fam);
      assert.equal(r.length, 3);
      assert.ok(
        r[0].triangles < r[1].triangles && r[1].triangles < r[2].triangles,
        `${fam} ${r.map((x) => x.triangles)}`,
      );
      for (const x of r) {
        assert.ok(x.watertight, `${fam} ${x.preset} watertight`);
        assert.equal(x.openEdges, 0);
        assert.ok(x.manifold, `${fam} ${x.preset} manifold (nonManifold ${x.nonManifoldEdges})`);
      }
    }
    // medium row equals the baseline fixture
    const med = rows.find((x) => x.family === "caret" && x.preset === "medium")!;
    assert.equal(med.triangles, fixture.vehicles["caret|0"].triangles);
  });

  it("LE clustering makes the first streamwise cell finer and keeps the mesh closed", () => {
    for (const family of [
      "caret",
      "osculating",
      "cone",
      "viscopt",
      "elliptic",
      "inward",
      "busemann",
    ] as WaveriderFamily[]) {
      const firstGap = (le: number) => {
        const r = presetSettings("fine");
        r.surface.leClustering = le;
        r.surface.curvature = 0; // isolate the chordwise effect
        const b = buildVehicle({ ...DEFAULT_PARAMS, family, nx: 56, ny: 28, refinement: r });
        const g = b.grids[0];
        const j = Math.min(g.nj - 2, ((g.nj - 1) >> 1) + 4);
        const q = analyzeMesh(b.mesh);
        assert.ok(q.watertight && q.manifold, `${family} le=${le} watertight/manifold`);
        return g.xyz[(1 * g.nj + j) * 3] - g.xyz[(0 * g.nj + j) * 3];
      };
      const g0 = firstGap(0);
      const g6 = firstGap(0.6);
      const g1 = firstGap(1);
      assert.ok(g6 < 0.7 * g0 && g1 < g6, `${family}: ${g0} ${g6} ${g1}`);
    }
  });

  it("families that do not use the loft stations (liftbody, star, ducts) are unchanged by LE clustering", () => {
    for (const family of ["liftbody", "star", "ramjet"] as WaveriderFamily[]) {
      const base = buildVehicle({ ...DEFAULT_PARAMS, family, flowThrough: family === "ramjet" });
      const r = presetSettings("medium");
      r.surface.leClustering = 1;
      const clustered = buildVehicle({
        ...DEFAULT_PARAMS,
        family,
        flowThrough: family === "ramjet",
        refinement: r,
      });
      assert.equal(fnv(clustered.mesh.positions), fnv(base.mesh.positions), family);
    }
  });
});

describe("grid convergence vs exact oblique shock (M=8)", () => {
  const rows = wedgeStudy();
  const find = (th: number, id: string) => rows.find((r) => r.thetaDeg === th && r.preset === id)!;
  for (const r of rows) {
    console.log(
      `wedge ${r.thetaDeg} deg ${r.preset.padEnd(6)} ${r.grid.padEnd(12)} cells=${String(r.cells).padEnd(5)} ${r.ms.toFixed(0).padStart(5)} ms  p2/p1=${r.p2p1.toFixed(3)} (exact ${r.exactP.toFixed(3)}) err=${r.pErrPct.toFixed(2)}%  beta=${r.shockDeg.toFixed(2)} (exact ${r.exactShockDeg.toFixed(2)}) err=${r.shockErrDeg.toFixed(2)} deg`,
    );
  }
  it("fine is within 1 % in pressure ratio and 0.5 deg in shock angle on both wedges, and no worse than medium", () => {
    for (const th of [8, 30]) {
      const fine = find(th, "fine");
      const med = find(th, "medium");
      const leg = find(th, "legacy");
      assert.ok(fine.pErrPct < 1, `${th} deg fine p err ${fine.pErrPct}`);
      assert.ok(Math.abs(fine.shockErrDeg) < 0.5, `${th} deg fine beta err ${fine.shockErrDeg}`);
      assert.ok(fine.pErrPct <= leg.pErrPct, `${th} deg fine ${fine.pErrPct} vs legacy ${leg.pErrPct}`);
      assert.ok(med.pErrPct < 1.5 && med.pErrPct < leg.pErrPct / 5, `${th} deg medium ${med.pErrPct} vs legacy ${leg.pErrPct}`);
      assert.ok(fine.cells > med.cells);
    }
  });
  it("legacy reproduces the known baseline error (13.5 % / 36.2 %) - the legacy grid is under-resolved", () => {
    assert.ok(Math.abs(find(8, "legacy").pErrPct - 13.518) < 0.01);
    assert.ok(Math.abs(find(30, "legacy").pErrPct - 36.23) < 0.01);
  });
  it("fine stops on the convergence test, well before the iteration cap", () => {
    for (const th of [8, 30]) {
      const r = eulerWedge(8, th, 1.4, presetSettings("fine").volume);
      assert.ok(r.iters < presetSettings("fine").volume.iters, `${th} deg used ${r.iters} sweeps`);
      assert.ok(r.relErr * 100 < 1, `${th} err ${r.relErr * 100}`);
    }
  });
  it(
    "shock clustering at the legacy cell count already beats the uniform legacy grid",
    { timeout: 60000 },
    () => {
      const vol: VolumeRefinement = { ...MEDIUM.volume, iters: 1600, shockClustering: 6 };
      for (const th of [8, 30]) {
        const r = eulerWedge(8, th, 1.4, vol);
        assert.equal(r.ni, 72);
        assert.equal(r.nj, 24);
        assert.ok(r.relErr * 100 < 1, `${th} deg 72x24 clustered err ${r.relErr * 100}`);
        assert.ok(r.relErr * 100 < find(th, "legacy").pErrPct / 10);
      }
    },
  );
  it(
    "adaptive re-cluster recovers a badly placed static shock estimate",
    { timeout: 120000 },
    () => {
      const vol: VolumeRefinement = {
        nStream: 72,
        nCross: 24,
        iters: 1600,
        wallClustering: 0,
        shockClustering: 8,
        shockWidth: 0.06,
        adaptive: false,
      };
      for (const th of [8, 30]) {
        for (const bias of [0.5, 1.8]) {
          wedgeShockBias.value = bias;
          const st = eulerWedge(8, th, 1.4, vol);
          const ad = eulerWedge(8, th, 1.4, { ...vol, adaptive: true });
          wedgeShockBias.value = 1;
          assert.equal(ad.passes, 2);
          assert.equal(st.passes, 1);
          assert.ok(ad.relErr * 100 < 0.5, `${th} bias ${bias}: adaptive err ${ad.relErr * 100}`);
          assert.ok(Math.abs((ad.shockAngleDeg ?? NaN) - (ad.exactShockAngleDeg ?? 0)) < 0.2);
          // Adaptive is not always better than a lucky static guess; it must never be bad, and must beat a bad one.
          assert.ok(
            ad.relErr <= Math.max(st.relErr, 0.003),
            `${th} bias ${bias}: adaptive ${ad.relErr} static ${st.relErr}`,
          );
        }
      }
    },
  );
});

describe("symmetry-plane kernel convergence vs exact oblique shock (M=8)", () => {
  const rows = stripStudy();
  for (const r of rows) {
    console.log(
      `strip ${r.thetaDeg} deg ${r.preset.padEnd(6)} ${r.grid.padEnd(12)} cells=${String(r.cells).padEnd(5)} ${r.ms.toFixed(0).padStart(5)} ms  Cp=${r.cp.toFixed(4)} (exact ${r.exactCp.toFixed(4)}) err=${r.cpErrPct.toFixed(1)}%  beta=${r.shockDeg.toFixed(2)} (exact ${r.exactShockDeg.toFixed(2)})`,
    );
  }
  it("coarse is within 3 %, medium and fine within 2 % and 0.3 deg, the shipped 88x28x42 run is not", () => {
    for (const th of [8, 30]) {
      const f = rows.find((r) => r.thetaDeg === th && r.preset === "fine")!;
      const c = rows.find((r) => r.thetaDeg === th && r.preset === "coarse")!;
      const m = rows.find((r) => r.thetaDeg === th && r.preset === "medium")!;
      const l = rows.find((r) => r.thetaDeg === th && r.preset === "legacy")!;
      assert.ok(f.cpErrPct < 2 && Math.abs(f.shockDeg - f.exactShockDeg) < 0.3, `${th} fine ${f.cpErrPct}`);
      assert.ok(m.cpErrPct < 2 && Math.abs(m.shockDeg - m.exactShockDeg) < 0.3, `${th} medium ${m.cpErrPct}`);
      assert.ok(c.cpErrPct < 3, `${th} coarse ${c.cpErrPct}`);
      assert.ok(l.cpErrPct > 20, `${th} legacy (unconverged) ${l.cpErrPct}`);
    }
  });
});
