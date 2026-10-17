/**
 * Wall-Cp agreement of the Euler quality tiers with the `reference` tier on REAL vehicle symmetry-plane walls
 * (4 families x 3 flight conditions x upper/lower). Errors are on the wall-mean Cp (i >= 5, nose excluded) and
 * RMS over the wall, in Cp units and relative to the mean |Cp| of the reference. Time = avg per wall solve.
 *   tsx scripts/bench/iter-study-vehicle.ts
 */
import { buildVehicle } from "../../src/lib/waverider/generate";
import { wallsOfMesh } from "../../src/lib/waverider/sym-euler";
import { DEFAULT_PARAMS, type WaveriderFamily } from "../../src/lib/waverider/types";
import { eulerSync, kernelFlags, solveWallRaw, type EulerQuality } from "../../src/lib/perf/euler-kernel";
import { cachedWallCp } from "../../src/lib/perf/euler-kernel";
import { qualitySettings } from "../../src/lib/perf/quality";
import { refinedPlan } from "../../src/lib/waverider/sym-euler";
import { solveWallEx } from "../../src/lib/perf/euler-kernel";

eulerSync();
void cachedWallCp;
type Wall = { x: Float64Array; z: Float64Array };
function solveTier(wall: Wall, mach: number, alpha: number, q: EulerQuality | "unc88x28x400" | "unc88x28x200" | string): { cp: Float64Array; ms: number } {
  const t0 = performance.now();
  let cp: Float64Array;
  if (q === "legacy") cp = solveWallRaw(wall.x, wall.z, mach, 1.4, alpha);
  else if (q === "unc88x28x400") cp = solveWallRaw(wall.x, wall.z, mach, 1.4, alpha, { grid: { ni: 88, nj: 28, iters: 400 } });
  else if (q === "unc88x28x200") cp = solveWallRaw(wall.x, wall.z, mach, 1.4, alpha, { grid: { ni: 88, nj: 28, iters: 200 } });
  else {
    let set = qualitySettings(q as EulerQuality)!;
    if (q.startsWith("c:")) {
      const [, ns, nc, tl, sq] = q.split(":");
      set = { ...qualitySettings("converged")!, volume: { ...qualitySettings("converged")!.volume, nStream: +ns, nCross: +nc, iters: 3000, tol: +tl, sequence: +sq } } as never;
    }
    const { v, d, shockS } = refinedPlan(wall as never, mach, 1.4, alpha, set, 0.02);
    cp = solveWallEx(wall.x, wall.z, mach, 1.4, alpha, { ni: d.ni, nj: d.nj, iters: d.iters, wallAmp: v.wallClustering, shockAmp: shockS ? v.shockClustering : 0, shockWidth: v.shockWidth, tol: v.tol ?? 0, sequence: v.sequence ?? 0, cfl: 0, shockS })!;
  }
  return { cp, ms: performance.now() - t0 };
}
void kernelFlags;
const fams: WaveriderFamily[] = ["caret", "osculating", "liftbody", "cone"];
const conds: [number, number][] = [[8, 2], [5, 3], [12, 0]];
const walls = new Map<WaveriderFamily, ReturnType<typeof wallsOfMesh>>();
for (const f of fams) walls.set(f, wallsOfMesh(buildVehicle({ ...DEFAULT_PARAMS, family: f }).mesh));
const refs = new Map<string, Float64Array>();
const key = (f: string, m: number, a: number, s: string) => `${f}|${m}|${a}|${s}`;
for (const f of fams) for (const [m, a] of conds) for (const s of ["upper", "lower"] as const) refs.set(key(f, m, a, s), solveTier(walls.get(f)![s], m, (a * Math.PI) / 180, "reference").cp);
console.log("tier".padEnd(15), "mean-Cp err (Cp units: mean / worst)".padEnd(38), "rel to mean|Cp| (mean / worst)".padEnd(32), "RMS dCp (mean / worst)".padEnd(26), "ms/solve");
for (const q of ["legacy", "unc88x28x200", "unc88x28x400", "preview", "standard", "c:72:24:3e-2:2", "c:72:24:1e-2:1", "c:72:24:3e-2:1", "c:72:24:1e-2:2"] as string[]) {
  const em: number[] = [];
  const er: number[] = [];
  const rms: number[] = [];
  let ms = 0;
  let cnt = 0;
  for (const f of fams)
    for (const [m, a] of conds)
      for (const s of ["upper", "lower"] as const) {
        const w = walls.get(f)![s];
        const r = refs.get(key(f, m, a, s))!;
        solveTier(w, m, (a * Math.PI) / 180, q); // warm
        const t = solveTier(w, m, (a * Math.PI) / 180, q);
        ms += t.ms;
        cnt++;
        let mr = 0;
        let mt = 0;
        let k = 0;
        let sq = 0;
        for (let i = 5; i < r.length; i++) {
          mr += r[i];
          mt += t.cp[i];
          sq += (t.cp[i] - r[i]) ** 2;
          k++;
        }
        mr /= k;
        mt /= k;
        em.push(Math.abs(mt - mr));
        er.push(Math.abs(mt - mr) / Math.max(Math.abs(mr), 1e-4));
        rms.push(Math.sqrt(sq / k));
      }
  const st = (a: number[]) => `${(a.reduce((x, y) => x + y, 0) / a.length).toExponential(2)} / ${Math.max(...a).toExponential(2)}`;
  const pc = (a: number[]) => `${((a.reduce((x, y) => x + y, 0) / a.length) * 100).toFixed(1)}% / ${(Math.max(...a) * 100).toFixed(1)}%`;
  console.log(q.padEnd(15), st(em).padEnd(38), pc(er).padEnd(32), st(rms).padEnd(26), (ms / cnt).toFixed(1));
}
