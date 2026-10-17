/**
 * Study-level error and cost of the Euler quality tiers (what a user sees: CL, CD, L/D, Cm_alpha) vs the
 * `reference` tier, over families x flight conditions, euler aero method.
 *   tsx scripts/bench/iter-study-study.ts [--quick]
 */
import { Pipeline } from "../../src/lib/perf/pipeline";
import type { EulerQuality } from "../../src/lib/perf/euler-kernel";
import { DEFAULT_PARAMS, type WaveriderFamily } from "../../src/lib/waverider/types";

const quick = process.argv.includes("--quick");
const fams: WaveriderFamily[] = quick ? ["caret", "liftbody"] : ["caret", "osculating", "liftbody", "cone"];
const conds: [number, number][] = quick ? [[8, 2]] : [[8, 2], [5, 3]];
const tiers: EulerQuality[] = ["legacy", "preview", "standard", "converged"];
const field = (s: any) => ({ cl: s.aero.cl, cd: s.aero.cd, ld: s.aero.ld, cma: s.stab.cma });
const ref = new Map<string, ReturnType<typeof field>>();
const plRef = new Pipeline("reference");
const mk = (f: WaveriderFamily, m: number, a: number) => ({ ...DEFAULT_PARAMS, family: f, aeroMethod: "euler" as const, flightMach: m, mach: m, alphaDeg: a, lockFlight: false });
for (const f of fams) for (const [m, a] of conds) ref.set(`${f}|${m}|${a}`, field(plRef.run(mk(f, m, a), "full").study));
console.log("tier".padEnd(10), "mean |dCL|/ref".padStart(15), "worst".padStart(8), "mean |dCD|/ref".padStart(15), "worst".padStart(8), "mean |dL/D|/ref".padStart(16), "worst".padStart(8), "  full-study ms (median/p95)  quick-study ms");
for (const t of tiers) {
  const pl = new Pipeline(t);
  const e: Record<string, number[]> = { cl: [], cd: [], ld: [] };
  const times: number[] = [];
  const qt: number[] = [];
  for (const f of fams)
    for (const [m, a] of conds) {
      const p = mk(f, m, a);
      pl.build(p); // geometry outside the timing
      const t0 = performance.now();
      const s = pl.run({ ...p, name: "t" + Math.random() }, "full"); // cosmetic key change forces... study cache hit! use alpha jitter instead
      void s;
      pl.clear();
      const p2 = { ...p, alphaDeg: a + 1e-9 };
      pl.build(p2);
      const t1 = performance.now();
      const r = pl.run(p2, "full");
      times.push(performance.now() - t1);
      void t0;
      const q0 = performance.now();
      pl.run({ ...p2, alphaDeg: a + 2e-9 }, "quick");
      qt.push(performance.now() - q0);
      const x = field(r.study);
      const rf = ref.get(`${f}|${m}|${a}`)!;
      for (const k of ["cl", "cd", "ld"] as const) e[k].push(Math.abs(x[k] - rf[k]) / Math.max(Math.abs(rf[k]), 1e-6));
    }
  const st = (a: number[]) => [a.reduce((s, v) => s + v, 0) / a.length, Math.max(...a)];
  const md = (a: number[]) => {
    const s = [...a].sort((x, y) => x - y);
    return [s[s.length >> 1], s[Math.min(s.length - 1, Math.ceil(0.95 * s.length) - 1)]];
  };
  const [cl, cd, ld] = [st(e.cl), st(e.cd), st(e.ld)];
  console.log(t.padEnd(10), `${(cl[0] * 100).toFixed(2)}%`.padStart(15), `${(cl[1] * 100).toFixed(1)}%`.padStart(8), `${(cd[0] * 100).toFixed(2)}%`.padStart(15), `${(cd[1] * 100).toFixed(1)}%`.padStart(8), `${(ld[0] * 100).toFixed(2)}%`.padStart(16), `${(ld[1] * 100).toFixed(1)}%`.padStart(8), `   ${md(times)[0].toFixed(0)} / ${md(times)[1].toFixed(0)}        ${md(qt)[0].toFixed(0)}`);
}
