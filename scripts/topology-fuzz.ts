// Run: bun scripts/topology-fuzz.ts [--n 3000] [--seed 1] [--exports 6] [--json out.json] [--only family] [--quiet]
// Seeded fuzz / stress of all 13 families: exceptions, NaN/Inf, manifold/winding/degenerates/volume,
// export validity (STL/Plot3D/IGES/STEP) and runtime blowups. Defects are grouped by root-cause signature.
import { writeFileSync } from "node:fs";
import { buildVehicle } from "../src/lib/waverider/generate";
import { topoMetrics, checkBinaryStl, checkIges, checkPlot3d, checkStep, stepBrepVolume, ductFlowVolume } from "../src/lib/waverider/geom-quality";
import { meshToBinaryStl } from "../src/lib/waverider/export-stl";
import { gridsToPlot3d } from "../src/lib/waverider/export-plot3d";
import { gridsToIges } from "../src/lib/waverider/export-iges";
import { gridsToNurbsStep } from "../src/lib/waverider/export-step";
import { refineGrids } from "../src/lib/waverider/mesh";
import { DEFAULT_PARAMS, type DesignParams, type WaveriderFamily, type PlanformKind } from "../src/lib/waverider/types";

export const FAMILIES: WaveriderFamily[] = ["caret", "cone", "osculating", "viscopt", "elliptic", "wedgecone", "star", "liftbody", "ramjet", "scramjet", "busemann", "inward", "integrated"];
export const DUCT_FAMILIES = ["ramjet", "scramjet"];
export const STEP_FAMILIES: WaveriderFamily[] = ["caret", "cone", "osculating", "viscopt", "elliptic", "wedgecone", "star", "liftbody", "busemann", "inward", "integrated", "ramjet", "scramjet"];

export function rng(seed: number) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const PLANFORMS: PlanformKind[] = ["delta", "power", "spatular", "rect", "double"];

/** one random parameter set; `extreme` snaps each knob to an end of its (widened-UI) range with prob 0.4 */
export function randomParams(r: () => number, family: WaveriderFamily, extreme: boolean): Partial<DesignParams> {
  const u = (lo: number, hi: number) => lo + (hi - lo) * r();
  const lg = (lo: number, hi: number) => Math.exp(u(Math.log(lo), Math.log(hi)));
  const pick = <T,>(a: T[]) => a[Math.floor(r() * a.length) % a.length];
  const ex = (lo: number, hi: number, mid: () => number) => (extreme && r() < 0.4 ? (r() < 0.5 ? lo : hi) : mid());
  const p: Partial<DesignParams> = {
    family,
    flowThrough: DUCT_FAMILIES.includes(family),
    mach: ex(2, 25, () => u(3, 16)),
    gamma: ex(1.1, 1.67, () => u(1.2, 1.67)),
    length: ex(0.2, 30, () => lg(0.5, 12)),
    span: ex(0.05, 16, () => lg(0.15, 12)),
    height: ex(0.02, 6, () => lg(0.05, 4)),
    shockDeg: ex(4, 50, () => u(8, 40)),
    coneDeg: ex(2, 30, () => u(3, 20)),
    superN: ex(0.8, 8, () => u(1, 6)),
    planform: pick(PLANFORMS),
    planformPower: ex(0.5, 3, () => u(0.7, 2.2)),
    fins: Math.round(ex(2, 10, () => u(3, 8))),
    wedgeFrac: ex(0.05, 0.95, () => u(0.1, 0.85)),
    captureFrac: ex(0.1, 0.9, () => u(0.15, 0.8)),
    nx: Math.round(ex(6, 200, () => lg(12, 160))),
    ny: Math.round(ex(6, 140, () => lg(10, 120))),
    leRadius: ex(0, 0.5, () => (r() < 0.1 ? 0 : lg(0.0005, 0.2))),
    leBlend: r() < 0.25 ? "g2" : "circular",
    leBlunt: r() < 0.85,
    halfModel: r() < 0.2,
    lid: r() < 0.2 ? "bottom" : "top",
    dihedralDeg: ex(-20, 30, () => (r() < 0.5 ? 0 : u(-8, 12))),
    camber: ex(-0.4, 0.6, () => (r() < 0.4 ? 0 : u(-0.25, 0.4))),
    teSweepDeg: ex(-30, 45, () => (r() < 0.4 ? 0 : u(-5, 25))),
    elevonDeg: ex(-30, 30, () => (r() < 0.5 ? 0 : u(-12, 12))),
    finHeight: ex(0, 2.5, () => (r() < 0.5 ? 0 : u(0, 1.4))),
    nRamps: Math.round(u(1, 4)),
    cowlSide: r() < 0.5 ? "belly" : "dorsal",
    inletHeight: ex(0.02, 1.0, () => u(0.04, 0.6)),
    cowlFrac: u(0.2, 0.6),
    combustorFrac: u(0.2, 0.5),
    nozzleER: u(2, 8),
    rampDeg: u(4, 14),
  };
  return p;
}

export interface FuzzCase { id: number; family: WaveriderFamily; params: Partial<DesignParams>; exports: boolean }
export interface Defect { id: number; family: string; cause: string; detail: string; params: Partial<DesignParams> }

export function runCase(c: FuzzCase, slowMs = 6000): Defect[] {
  const out: Defect[] = [];
  const add = (cause: string, detail: string) => out.push({ id: c.id, family: c.family, cause, detail, params: c.params });
  const duct = DUCT_FAMILIES.includes(c.family);
  const params: DesignParams = { ...DEFAULT_PARAMS, ...c.params } as DesignParams;
  let built;
  const t0 = performance.now();
  try {
    built = buildVehicle(params);
  } catch (e) {
    add("exception", String((e as Error).message).slice(0, 120));
    return out;
  }
  const ms = performance.now() - t0;
  if (ms > slowMs) add("slow", `${ms.toFixed(0)} ms`);
  const mesh = built.mesh;
  let bad = 0;
  for (let i = 0; i < mesh.positions.length; i++) if (!Number.isFinite(mesh.positions[i])) bad++;
  for (const g of built.grids) for (let i = 0; i < g.xyz.length; i++) if (!Number.isFinite(g.xyz[i])) bad++;
  if (bad) {
    add("nonfinite", `${bad} NaN/Inf coordinates`);
    return out;
  }
  if (mesh.indices.length < 12) {
    add("empty-mesh", `${mesh.indices.length / 3} triangles`);
    return out;
  }
  const m = topoMetrics(mesh);
  if (!duct && m.openEdges) add("open-edges", `${m.openEdges}`);
  if (m.nonManifoldEdges) add("non-manifold", `${m.nonManifoldEdges}`);
  if (m.inconsistentEdges) add("inconsistent-winding", `${m.inconsistentEdges}`);
  if (m.degenerateTris) add("degenerate-tris", `${m.degenerateTris}`);
  if (m.duplicateVerts) add("duplicate-verts", `${m.duplicateVerts}`);
  if (!(m.signedVolume > 0)) add("bad-volume", `${m.signedVolume}`);
  if (!c.exports) return out;
  try {
    const nt = mesh.indices.length / 3;
    const stl = checkBinaryStl(meshToBinaryStl(mesh, "f"), nt);
    if (!stl.ok) add("export-stl", stl.errors.join("; ").slice(0, 100));
    const light = built.grids.reduce((s, g) => s + g.ni * g.nj, 0) < 6000;
    // what export-kit hands the surface writers
    const grids = light ? refineGrids(built.grids, 3, 2) : built.grids;
    const p3 = checkPlot3d(gridsToPlot3d(grids));
    if (!p3.ok) add("export-plot3d", p3.errors.join("; ").slice(0, 100));
    const iges = checkIges(gridsToIges(grids, "f", "m"));
    if (!iges.ok) add("export-iges", iges.errors.join("; ").slice(0, 100));
    if (light && STEP_FAMILIES.includes(c.family)) {
      const step = gridsToNurbsStep(grids, "f", "m");
      const cs = checkStep(step);
      if (!cs.ok) add("export-step", cs.errors.join("; ").slice(0, 100));
      else {
        const ref = duct ? ductFlowVolume(built.grids) : m.signedVolume;
        if (ref > 0) {
          const v = stepBrepVolume(step, 24);
          if (!(v > 0) || Math.abs(v - ref) > 0.08 * ref) add("export-step-volume", `${v.toFixed(4)} vs ${duct ? "duct flow path" : "mesh"} ${ref.toFixed(4)}`);
        }
      }
    }
  } catch (e) {
    add("export-exception", String((e as Error).message).slice(0, 120));
  }
  return out;
}

if (import.meta.main) {
  const a = process.argv.slice(2);
  const arg = (k: string, d: string) => (a.includes(k) ? a[a.indexOf(k) + 1] : d);
  const n = +arg("--n", "3000");
  const seed = +arg("--seed", "1");
  const expEvery = +arg("--exports", "6");
  const only = arg("--only", "");
  const json = arg("--json", "");
  const quiet = a.includes("--quiet");
  const r = rng(seed);
  const fams = only ? FAMILIES.filter((f) => f === only) : FAMILIES;
  const defects: Defect[] = [];
  const casesBad = new Set<number>();
  const perFam: Record<string, { n: number; bad: number }> = {};
  let maxMs = 0;
  const t0 = Date.now();
  for (let id = 0; id < n; id++) {
    const family = fams[id % fams.length];
    const extreme = Math.floor(id / fams.length) % 2 === 0;
    const c: FuzzCase = { id, family, params: randomParams(r, family, extreme), exports: expEvery > 0 && id % expEvery === 0 };
    const ts = performance.now();
    const d = runCase(c);
    maxMs = Math.max(maxMs, performance.now() - ts);
    perFam[family] ??= { n: 0, bad: 0 };
    perFam[family].n++;
    if (d.length) {
      perFam[family].bad++;
      casesBad.add(id);
      defects.push(...d);
      if (!quiet) console.log(`#${id} ${family}: ${d.map((x) => `${x.cause}(${x.detail})`).join(", ")}`);
    }
  }
  const byCause: Record<string, { n: number; fams: Record<string, number>; first: number }> = {};
  for (const d of defects) {
    const g = (byCause[d.cause] ??= { n: 0, fams: {}, first: d.id });
    g.n++;
    g.fams[d.family] = (g.fams[d.family] ?? 0) + 1;
  }
  console.log(`\ncases tried: ${n} (seed ${seed}); cases with defects: ${casesBad.size}; defect records: ${defects.length}; slowest case ${maxMs.toFixed(0)} ms; wall ${((Date.now() - t0) / 1000).toFixed(0)} s`);
  for (const [k, g] of Object.entries(byCause).sort((x, y) => y[1].n - x[1].n)) console.log(`  ${k.padEnd(22)} ${String(g.n).padStart(5)}  first #${g.first}  ${JSON.stringify(g.fams)}`);
  console.log("per family (cases / with defects):", Object.entries(perFam).map(([f, v]) => `${f} ${v.n}/${v.bad}`).join(", "));
  if (json) writeFileSync(json, JSON.stringify({ n, seed, casesBad: [...casesBad], byCause, defects }, null, 1));
  process.exitCode = defects.length ? 1 : 0;
}
