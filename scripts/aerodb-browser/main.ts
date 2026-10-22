// [aerodb-perf] Browser check: Web Worker pool vs TS in real Chrome. Run: node scripts/aerodb-browser/run.mjs
import { buildVehicle } from "@/lib/waverider/generate";
import { DEFAULT_PARAMS } from "@/lib/waverider/types";
import { generateAeroDatabase, aeroDbPointCap } from "@/lib/waverider/aerodb";
const out = document.getElementById("out")!;
(async () => {
  try {
    const built = buildVehicle({ ...DEFAULT_PARAMS, family: "caret", nx: 24, ny: 16, mach: 8 });
    const spec = { machMin: 3, machMax: 12, machStep: 4.5, alphaMin: -4, alphaMax: 12, alphaStep: 4, betaMin: -4, betaMax: 4, betaStep: 4, deltaMin: -10, deltaMax: 10, deltaStep: 10, method: "cbaero" as const, eulerSymmetry: false, dynamicDerivatives: true, altKm: 30 };
    const t0 = performance.now();
    const ts = await generateAeroDatabase(built, spec, { engine: "ts" });
    const t1 = performance.now();
    // frame gaps while the pool runs = main-thread responsiveness
    let maxGap = 0, last = performance.now(), on = true;
    const raf = () => { const n = performance.now(); maxGap = Math.max(maxGap, n - last); last = n; if (on) requestAnimationFrame(raf); };
    requestAnimationFrame(raf);
    let prog = 0;
    const pool = await generateAeroDatabase(built, spec, { onProgress: (p) => (prog = p.done) });
    const t2 = performance.now();
    on = false;
    let worst = 0;
    ts.points.forEach((p, i) => { for (const k of ["cl","cd","cm","cy","cn","cll"] as const) { const a = p[k], b = pool.points[i][k]; worst = Math.max(worst, Math.abs(a-b)/(Math.max(Math.abs(a),Math.abs(b))+1e-9)); } });
    // abort
    const ac = new AbortController();
    let aborted = "no";
    const run = generateAeroDatabase(built, spec, { signal: ac.signal, chunkSize: 4, onProgress: (p) => { if (p.done > 8) ac.abort(); } });
    try { await run; } catch (e) { aborted = (e as Error).name; }
    const after = await generateAeroDatabase(built, { ...spec, method: "newtonian" });
    out.textContent = JSON.stringify({ ok: true, engine: pool.meta.engine, cap: aeroDbPointCap("cbaero"), pts: pool.points.length, worst, tsMs: t1 - t0, poolMs: t2 - t1, maxFrameGapMs: maxGap, prog, aborted, afterEngine: after.meta.engine, hc: navigator.hardwareConcurrency });
  } catch (e) { out.textContent = JSON.stringify({ ok: false, err: String((e as Error)?.stack ?? e) }); }
})();
