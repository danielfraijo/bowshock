/**
 * Timing of the analysis pipeline (median of repeats, warm) + cold-start of the first call.
 * Run: node --import ./scripts/ts-register.mjs scripts/perf.ts
 * Works on any commit that has these exports (used for before/after).
 */
import { atmosphere } from "../src/lib/waverider/atmosphere";
import { clearDataPackCache, computeDataPackAsync } from "../src/lib/waverider/datapack";
import { buildVehicle } from "../src/lib/waverider/generate";
import { panelAero } from "../src/lib/waverider/panel";
import { studyVehicle } from "../src/lib/waverider/study";
import { DEFAULT_PARAMS, type AeroMethod, type DesignParams } from "../src/lib/waverider/types";

const med = (a: number[]) => [...a].sort((x, y) => x - y)[Math.floor(a.length / 2)];
function time(fn: () => void, reps = 7): number {
  const t: number[] = [];
  for (let i = 0; i < reps; i++) {
    const s = performance.now();
    fn();
    t.push(performance.now() - s);
  }
  return med(t);
}
const out: string[] = [];
for (const family of ["caret", "cone", "liftbody", "osculating"] as const) {
  const p: DesignParams = { ...DEFAULT_PARAMS, family };
  const b = buildVehicle(p);
  const tb = time(() => buildVehicle(p));
  const row = [`${family} (${b.mesh.indices.length / 3} tris) build ${tb.toFixed(1)} ms`];
  for (const m of ["newtonian", "cbaero", "tangent", "mixed", "euler"] as AeroMethod[]) {
    const q = { ...p, aeroMethod: m };
    const tm = time(() => panelAero(b.mesh, q, atmosphere(q.altKm, q.flightMach, q.gamma), 3, 0, b.aero.planformArea, q.length));
    row.push(`${m} ${tm.toFixed(1)}`);
  }
  out.push(row.join(" | "));
}
console.log(out.join("\n"));
const built = buildVehicle(DEFAULT_PARAMS);
console.log(`studyVehicle quick (euler): median ${time(() => studyVehicle(built, { quick: true })).toFixed(1)} ms; full: ${time(() => studyVehicle(built), 3).toFixed(1)} ms`);
for (const aeroMethod of ["euler", "mixed"] as const) {
  const ts: number[] = [];
  const worstL: number[] = [];
  for (let rep = 0; rep < 3; rep++) {
    clearDataPackCache();
    let last = performance.now();
    let worst = 0;
    const t0 = last;
    await computeDataPackAsync({ ...DEFAULT_PARAMS, aeroMethod }, undefined, {
      onProgress: () => {
        const now = performance.now();
        worst = Math.max(worst, now - last);
        last = now;
      },
    });
    ts.push(performance.now() - t0);
    worstL.push(worst);
  }
  console.log(`datapack ${aeroMethod}: total ${ts.map((x) => x.toFixed(0)).join("/")} ms (1st = cold, includes 2-D HLLC anchor); longest slice ${worstL.map((x) => x.toFixed(0)).join("/")} ms`);
}
