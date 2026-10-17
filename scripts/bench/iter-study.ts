/**
 * Error-vs-time study for the symmetry-plane Euler kernel (wasm), on straight wedge walls where the exact
 * oblique-shock Cp is known:  tsx scripts/bench/iter-study.ts [--quick]
 * Metric: mean wall Cp over 0.3 < x < 0.8 vs exact; time = median of 3 warmed wasm calls.
 */
import { DEG, betaFromThetaM, obliqueShock } from "../../src/lib/waverider/math";
import { LEGACY, estimateShockHeights, shockFractionFromHeight } from "../../src/lib/waverider/refinement";
import { stripDomain } from "../../src/lib/waverider/strip-euler";
import { solveWallEx, solveWallRaw, eulerSync, hasSolveEx } from "../../src/lib/perf/euler-kernel";

eulerSync();
if (!hasSolveEx()) throw new Error("wasm has no euler_solve_ex: rebuild with npm run build:wasm");
const quick = process.argv.includes("--quick");
const n = 80;
const x = Float64Array.from({ length: n }, (_, i) => i / (n - 1));

interface Cfg {
  name: string;
  ni: number;
  nj: number;
  iters: number;
  tol: number;
  seq: number;
  wall: number;
  shock: number;
  fast?: boolean; // plain fixed-iteration kernel (euler_solve_grid), no clustering
}

function run(c: Cfg, mach: number, thetaDeg: number): { cp: number; ms: number } {
  const z = x.map((v) => v * Math.tan(thetaDeg * DEG));
  const reps = 3;
  let best: number[] = [];
  let out: Float64Array | null = null;
  for (let r = 0; r < reps + 1; r++) {
    const t0 = performance.now();
    if (c.fast) out = solveWallRaw(x, z, mach, 1.4, 0, { grid: { ni: c.ni, nj: c.nj, iters: c.iters } });
    else {
      let shockS: Float64Array | null = null;
      if (c.shock > 0) {
        const dom = stripDomain(x, z);
        const xs = Float64Array.from({ length: c.ni + 1 }, (_, i) => dom.xmin + ((dom.xmax - dom.xmin) * i) / c.ni);
        const h = estimateShockHeights(x, z, xs, mach, 1.4, 0, 1, 0);
        shockS = h.map((v) => shockFractionFromHeight(v, dom.hFar, LEGACY.strip.wallExp));
      }
      out = solveWallEx(x, z, mach, 1.4, 0, { ni: c.ni, nj: c.nj, iters: c.iters, wallAmp: c.wall, shockAmp: c.shock, shockWidth: 0.09, tol: c.tol, sequence: c.seq, cfl: 0, shockS });
    }
    const dt = performance.now() - t0;
    if (r > 0) best.push(dt); // first call = warm-up
  }
  best.sort((a, b) => a - b);
  let s = 0;
  let k = 0;
  for (let i = 0; i < n; i++) if (x[i] > 0.3 && x[i] < 0.8) (s += out![i]), k++;
  return { cp: s / k, ms: best[1] };
}

const M = 8;
const thetas = quick ? [8, 30] : [8, 15, 30];
const cfgs: Cfg[] = [];
const fixed = (name: string, ni: number, nj: number, iters: number, fast = true): Cfg => ({ name, ni, nj, iters, tol: 0, seq: 0, wall: 0, shock: 0, fast });
for (const it of [42, 200]) cfgs.push(fixed(`88x28 fixed ${it}`, 88, 28, it));
for (const it of [200]) cfgs.push(fixed(`117x37 fixed ${it}`, 117, 37, it));
const conv = (name: string, ni: number, nj: number, iters: number, tol: number, seq: number, wall = 2, shock = 6): Cfg => ({ name, ni, nj, iters, tol, seq, wall, shock });
for (const [tol, seq] of [[1e-2, 2], [3e-3, 2], [1e-3, 2]] as [number, number][]) cfgs.push(conv(`88x28 clus tol${tol} seq${seq}`, 88, 28, 2500, tol, seq));
for (const [tol, seq] of [[3e-3, 2]] as [number, number][]) cfgs.push(conv(`117x37 clus tol${tol} seq${seq}`, 117, 37, 2500, tol, seq));
cfgs.push(conv("44x14 clus tol1e-2 seq1 (preview)", 44, 14, 400, 1e-2, 1));
cfgs.push(conv("44x14 clus tol3e-3 seq1", 44, 14, 800, 3e-3, 1));
cfgs.push(conv("44x14 clus tol3e-2 seq1", 44, 14, 400, 3e-2, 1));
cfgs.push(conv("35x11 clus tol1e-2 seq1", 35, 11, 400, 1e-2, 1));
cfgs.push(conv("35x11 clus tol3e-2 seq0", 35, 11, 400, 3e-2, 0));
cfgs.push(conv("52x17 clus tol1e-2 seq1", 52, 17, 600, 1e-2, 1));
cfgs.push(conv("59x19 clus tol1e-2 seq1", 59, 19, 600, 1e-2, 1));
cfgs.push(conv("59x19 clus tol1e-2 seq2", 59, 19, 600, 1e-2, 2));
cfgs.push(conv("59x19 clus tol3e-3 seq1", 59, 19, 800, 3e-3, 1));

const exact = (th: number) => (obliqueShock(M, betaFromThetaM(M, th * DEG)).p2p1 - 1) / (0.5 * 1.4 * M * M);
const rows: { name: string; cells: string; err: Record<number, number>; ms: Record<number, number> }[] = [];
console.log(`wall-Cp error vs exact oblique shock, M=${M}, gamma=1.4 (mean Cp 0.3<x<0.8); wasm time per solve (median of 3, warmed)`);
console.log("config".padEnd(36), ...thetas.map((t) => `th${t} err%`.padStart(10)), ...thetas.map((t) => `th${t} ms`.padStart(9)));
for (const c of cfgs) {
  const err: Record<number, number> = {};
  const ms: Record<number, number> = {};
  for (const th of thetas) {
    const r = run(c, M, th);
    err[th] = (Math.abs(r.cp - exact(th)) / exact(th)) * 100;
    ms[th] = r.ms;
  }
  rows.push({ name: c.name, cells: `${c.ni}x${c.nj}`, err, ms });
  console.log(c.name.padEnd(36), ...thetas.map((t) => err[t].toFixed(2).padStart(10)), ...thetas.map((t) => ms[t].toFixed(1).padStart(9)));
}
