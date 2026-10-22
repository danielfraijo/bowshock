/**
 * Regression tests for "the site crashes when I change shape": the cause was a multi-second main-thread freeze (converged Euler
 * solves run synchronously inside a React render on every change), not an exception. These cover the non-blocking pipeline,
 * the never-throw wrappers and the saved-parameter sanitiser. bun test src/lib/waverider/shape-change.test.ts
 */
import assert from "node:assert/strict";
import { describe, it } from "node:test";
import { buildVehicle } from "./generate";
import { instantStudy, safeBuild, safeStudy, studyProgressive, usesEuler } from "./progressive-study";
import { presetSettings, previewOf, sweepOf, validateRefinement } from "./refinement";
import { sanitizeParams } from "./presets";
import { studyVehicle } from "./study";
import { beginEulerCollect, chunkFor, clearSymmetryEulerCache, endEulerCollect, solveEulerJobs, symmetryEulerReport } from "./sym-euler";
import { DEFAULT_PARAMS, FAMILY_META, type DesignParams } from "./types";

const coarse = (over: Partial<DesignParams> = {}): DesignParams => ({ ...DEFAULT_PARAMS, refinement: presetSettings("coarse"), ...over });
const finite = (x: number) => Number.isFinite(x);

describe("shape change never throws", () => {
  it("every family: safeBuild + instantStudy (quick and full) are finite, with the default converged preset", () => {
    for (const family of Object.keys(FAMILY_META)) {
      const { built, error } = safeBuild({ ...DEFAULT_PARAMS, family: family as DesignParams["family"] });
      assert.equal(error, undefined, family);
      for (const quick of [true, false]) {
        const { study } = instantStudy(built, quick);
        for (const v of [study.aero.cl, study.aero.cd, study.aero.cm, study.aero.ld]) assert.ok(finite(v), `${family} quick=${quick}: finite aero`);
      }
    }
  });
  it("instantStudy defers the Euler solve (fast, CBAERO numbers) and leaves nothing recording", () => {
    const built = buildVehicle(coarse());
    clearSymmetryEulerCache();
    const t0 = performance.now();
    const { study } = instantStudy(built, true);
    assert.ok(performance.now() - t0 < 400, "instant quick study is cheap");
    assert.ok(study.aero.notes.some((n) => /pending/i.test(n)), "says the Euler solve is pending");
    // collector is off again: a plain study now solves for real (no 'pending' note)
    const real = studyVehicle(built, { quick: true });
    assert.ok(!real.aero.notes.some((n) => /pending/i.test(n)));
  });
  it("safeBuild keeps the last good vehicle when the generator throws", () => {
    const good = buildVehicle(DEFAULT_PARAMS);
    const bad = new Proxy({ ...DEFAULT_PARAMS }, { get: () => { throw new Error("boom"); } }) as DesignParams;
    const r = safeBuild(bad, good);
    assert.equal(r.built, good);
    assert.match(r.error ?? "", /boom/);
  });
  it("safeStudy falls back to CBAERO when the selected method throws", () => {
    const built = buildVehicle(coarse());
    let calls = 0;
    const hostile = { ...built, params: new Proxy(built.params, { get(t, k, r) { if (k === "aeroMethod" && calls++ === 0) throw new Error("solver exploded"); return Reflect.get(t, k, r); } }) };
    const r = safeStudy(hostile as typeof built, true);
    assert.match(r.warning ?? "", /solver exploded/);
    assert.ok(finite(r.study.aero.cl));
  });
});

describe("progressive study", () => {
  it("converges to exactly the synchronous result (coarse, full study)", { timeout: 60000 }, async () => {
    const built = buildVehicle(coarse());
    clearSymmetryEulerCache();
    const sync = studyVehicle(built);
    clearSymmetryEulerCache();
    const stages: string[] = [];
    const { study, status } = await studyProgressive(built, { onUpdate: (_s, st) => stages.push(st.stage) });
    assert.equal(study.aero.cd, sync.aero.cd);
    assert.equal(study.aero.cl, sync.aero.cl);
    assert.equal(study.stab.cma, sync.stab.cma);
    assert.equal(status.stage, "final");
    assert.equal(stages[0], "preview", "first update is the instant preview");
    const rep = symmetryEulerReport();
    assert.ok(rep && !rep.pending && rep.upper && rep.lower, "convergence report is filled");
    assert.ok(rep!.lower!.iters > 0 && finite(rep!.lower!.residual));
  });
  it("a stale solve is cancelled between chunks (AbortError) and a fresh one still completes", { timeout: 60000 }, async () => {
    const built = buildVehicle(coarse({ alphaDeg: 2.5 }));
    clearSymmetryEulerCache();
    let reads = 0;
    const token = { get aborted() { return ++reads > 30; } }; // aborts after 30 checks, deterministic
    await assert.rejects(studyProgressive(built, { signal: token }), (e: unknown) => e instanceof DOMException && e.name === "AbortError");
    const ok = await studyProgressive(built, { quick: true });
    assert.ok(finite(ok.study.aero.cl));
  });
  it("the event loop keeps running while a converged solve is in progress", { timeout: 60000 }, async () => {
    const built = buildVehicle({ ...DEFAULT_PARAMS, refinement: presetSettings("medium"), alphaDeg: 1.7 });
    clearSymmetryEulerCache();
    beginEulerCollect({ primaryAlphaDeg: built.params.alphaDeg });
    try { studyVehicle(built, { quick: true }); } finally { /* collected below */ }
    const jobs = endEulerCollect();
    assert.ok(jobs.length >= 2, "both walls were recorded");
    let ticks = 0;
    let worst = 0;
    let last = performance.now();
    const iv = setInterval(() => { const n = performance.now(); worst = Math.max(worst, n - last); last = n; ticks++; }, 5);
    await solveEulerJobs(jobs);
    clearInterval(iv);
    assert.ok(ticks >= 3, `timer ticked ${ticks} times during the solve`);
    assert.ok(worst < 250, `longest gap between ticks ${worst.toFixed(0)} ms`);
  });
  it("legacy / cbaero are not deferred (cheap already)", () => {
    assert.equal(usesEuler(buildVehicle({ ...DEFAULT_PARAMS, refinement: presetSettings("legacy") })), false);
    assert.equal(usesEuler(buildVehicle({ ...DEFAULT_PARAMS, aeroMethod: "cbaero" })), false);
    assert.equal(usesEuler(buildVehicle(DEFAULT_PARAMS)), true);
  });
  it("preview / sweep fidelity never exceed what was asked and never touch legacy", () => {
    const fine = presetSettings("fine");
    assert.deepEqual(previewOf(fine).volume, presetSettings("coarse").volume);
    assert.deepEqual(sweepOf(fine).volume, presetSettings("medium").volume);
    const lg = presetSettings("legacy");
    assert.equal(previewOf(lg), lg);
    assert.equal(sweepOf(lg), lg);
    const tiny = validateRefinement({ preset: "custom", volume: { nStream: 20, nCross: 8, iters: 50 } }).settings;
    assert.equal(previewOf(tiny), tiny);
    assert.ok(chunkFor(1e6) >= 1 && chunkFor(1) <= 400);
  });
});

describe("saved-parameter sanitiser", () => {
  it("replaces null / NaN / wrong-typed fields and unknown families; clamps refinement", () => {
    const s = sanitizeParams({ length: null, mach: "fast", alphaDeg: NaN, family: "nope", nx: Infinity, refinement: { preset: "custom", volume: { iters: 1e12, tol: NaN } } });
    assert.equal(s.length, DEFAULT_PARAMS.length);
    assert.equal(s.mach, DEFAULT_PARAMS.mach);
    assert.equal(s.alphaDeg, DEFAULT_PARAMS.alphaDeg);
    assert.equal(s.family, DEFAULT_PARAMS.family);
    assert.equal(s.nx, DEFAULT_PARAMS.nx);
    assert.ok(s.refinement && s.refinement.volume.iters <= 4000);
    assert.ok(finite(buildVehicle(s).aero.cd));
  });
  it("keeps valid values and tolerates non-objects", () => {
    assert.equal(sanitizeParams({ length: 3.5, family: "caret" }).length, 3.5);
    assert.deepEqual(sanitizeParams(null), DEFAULT_PARAMS);
    assert.deepEqual(sanitizeParams("garbage"), DEFAULT_PARAMS);
  });
  it("drops prototype keys, path characters and out-of-range grids", () => {
    const polluted = JSON.parse('{"__proto__":{"admin":true},"constructor":{"x":1},"length":3,"nx":100000,"name":"../etc/passwd\\n","leBlend":"g2","extra":"nope"}');
    const s = sanitizeParams(polluted);
    assert.equal(s.length, 3);
    assert.equal(s.nx, 160);
    assert.equal(s.name, ".._etc_passwd");
    assert.equal(s.leBlend, "g2");
    assert.equal("extra" in s, false);
    assert.equal("admin" in {}, false);
    assert.equal(({} as { admin?: boolean }).admin, undefined);
  });
});
