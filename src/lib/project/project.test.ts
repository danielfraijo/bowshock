// [p7-project] npm run test:project  (node --experimental-strip-types --test src/lib/project/project.test.ts)
import { describe, it } from "node:test";
import assert from "node:assert/strict";
import { DEFAULT_PARAMS, type DesignParams } from "../waverider/types.ts";
import type { AeroDatabase, AeroDbSpec } from "../waverider/aerodb.ts";
import {
  type AeroDbSlot,
  alphaCurve,
  buildCompare,
  compareCandidates,
  importIterateLists,
  type VariantMetrics,
  aeroDbStale,
  paramsHash,
  MIGRATIONS,
  PROJECT_SCHEMA_VERSION,
  ProjectFileError,
  ProjectStore,
  branchFromSnapshot,
  canonicalJson,
  commitSnapshot,
  deleteSnapshot,
  diffStates,
  exportHistory,
  importHistory,
  listSnapshots,
  memoryKv,
  newProject,
  parseHistoryExport,
  parseProject,
  projectFileName,
  restoreSnapshot,
  sameDesign,
  serializeProject,
  summarizeDiff,
  type ProjectState,
} from "./index.ts";

const T0 = "2026-10-22T20:00:00.000Z";
const at = (min: number) => new Date(Date.parse(T0) + min * 60_000).toISOString();
const withParams = (s: ProjectState, p: Partial<DesignParams>): ProjectState => ({ ...s, params: { ...s.params, ...p } });

/** Stand-in for presets.ts sanitizeParams (that file has extensionless imports, so strip-types cannot load it). */
const sanitize = (p: unknown): DesignParams => {
  const src = p as Record<string, unknown>;
  const out: Record<string, unknown> = { ...DEFAULT_PARAMS };
  for (const k of Object.keys(src)) if (k in out && typeof src[k] === typeof out[k]) out[k] = src[k];
  out.nx = Math.min(160, Math.max(12, out.nx as number));
  return out as unknown as DesignParams;
};

const SPEC: AeroDbSpec = {
  machMin: 6, machMax: 10, machStep: 2, alphaMin: -4, alphaMax: 8, alphaStep: 4, betaMin: 0, betaMax: 0, betaStep: 4,
  deltaMin: 0, deltaMax: 0, deltaStep: 5, method: "cbaero", eulerSymmetry: false, dynamicDerivatives: false, altKm: 30,
};
/** Minimal AeroDatabase (main's aerodb.ts shape): CL = k·0.02·α, CD = 0.01 + 0.001·α². */
function fakeDb(k: number, machs = [6, 8, 10], alphas = [-4, 0, 4, 8]): AeroDbSlot {
  const zero = { cy: 0, cm: -0.001, cn: 0, cll: 0, ca: 0, cla: 0, cda: 0, cma: 0, cyb: 0, cnb: 0, clb: 0, cld: 0, cmd: 0, clq: 0, cmq: 0, clp: 0, cnp: 0, cyp: 0, cnr: 0, clr: 0, cyr: 0, euler_cl: null, euler_cd: null, euler_cm: null };
  const points = machs.flatMap((mach) =>
    alphas.map((a) => {
      const cl = k * 0.02 * a, cd = 0.01 + 0.001 * a * a;
      return { mach, alpha_deg: a, beta_deg: 0, delta_deg: 0, onDesign: mach === 8, cl, cd, ld: cl / cd, ...zero };
    }),
  );
  const database = {
    meta: { format: "cuspis-aerodb-1", name: "t", family: "caret", method: "cbaero", designMach: 8, altitude_km: 30, sRef_m2: 4.4, lRef_m: 4, bRef_m: 2.2, cg_m: [2.3, 0, 0], generatedAt: T0, assumptions: [] },
    axes: { mach: machs, alpha_deg: alphas, beta_deg: [0], delta_deg: [0] },
    points,
    dynamics: [],
    summary: { points: points.length, onDesign: alphas.length, offDesign: points.length - alphas.length, maxLd: 0, maxLdAt: { mach: 8, alpha_deg: 4, beta_deg: 0, delta_deg: 0 }, clMin: 0, clMax: 0, cdMin: 0, cdMax: 0, cmAtDesign: 0 },
    slices: { design: { mach: 8, alpha: [] }, offDesign: { mach: 10, alpha: [] } },
  } as unknown as AeroDatabase;
  return { spec: { ...SPEC }, database };
}

function sampleDoc() {
  let doc = newProject({ ...DEFAULT_PARAMS, refinement: { preset: "fine" } as DesignParams["refinement"] }, { name: "Caret M8 study", nowISO: T0, id: "p_test" });
  doc.state.ui = { viewer: { wireframe: true, shock: false, grid: true, origin: false, color: "cp" }, tab: "aero" };
  doc.state.aeroDb = fakeDb(1);
  doc = commitSnapshot(doc, doc.state, "baseline", { nowISO: at(1), id: "s1" }).doc;
  const s2 = withParams(doc.state, { mach: 10, span: 2.5 });
  doc = commitSnapshot(doc, s2, "Mach 10", { nowISO: at(2), id: "s2" }).doc;
  doc.state = s2;
  return doc;
}

describe("project schema: round trip", () => {
  it("serialize → parse is lossless (state, ui, aero DB, history)", () => {
    const doc = sampleDoc();
    const text = serializeProject(doc);
    const { doc: back, warnings, migrated } = parseProject(text);
    assert.equal(migrated, false);
    assert.deepEqual(warnings, []);
    assert.equal(back.schemaVersion, PROJECT_SCHEMA_VERSION);
    const strip = (d: typeof doc) => canonicalJson({ ...d, checksum: undefined });
    assert.equal(strip(back), strip(doc));
    // and a second round trip is byte-identical
    assert.equal(serializeProject(back), text);
  });

  it("round trip through the app sanitizer keeps in-range values untouched", () => {
    const doc = sampleDoc();
    const { doc: back, warnings } = parseProject(serializeProject(doc), { sanitizeParams: sanitize });
    assert.ok(sameDesign(back.state, { ...doc.state, params: sanitize(doc.state.params) }));
    assert.equal(warnings.filter((w) => /reset\/clamped/.test(w) && !/refinement/.test(w)).length, 0);
  });

  it("persists through the ProjectStore + autosave workspace", async () => {
    const store = new ProjectStore(memoryKv());
    const doc = sampleDoc();
    await store.saveProject(doc);
    const working = withParams(doc.state, { alphaDeg: 4 });
    await store.saveWorkspace({ projectId: doc.id, doc, working, updatedISO: at(3) });
    const list = await store.listProjects();
    assert.deepEqual(list.map((p) => [p.id, p.name, p.snapshots]), [["p_test", "Caret M8 study", 2]]);
    const ws = await store.loadWorkspace();
    assert.ok(ws);
    assert.equal(ws.working.params.alphaDeg, 4);
    assert.equal(ws.doc.history.snapshots.length, 2);
    assert.equal(sameDesign(ws.working, ws.doc.state), false, "unsaved change survives reload");
    assert.equal(canonicalJson((await store.loadProject("p_test"))!.state), canonicalJson(doc.state));
  });

  it("file name is safe", () => {
    assert.equal(projectFileName("Caret / M8 study"), "Caret_M8_study.bowshock.json");
  });
});

describe("project schema: migration", () => {
  it("v1 (linear snapshots, no branches) migrates to current", () => {
    const v1 = {
      format: "bowshock-project",
      schemaVersion: 1,
      name: "old project",
      savedAt: T0,
      params: { ...DEFAULT_PARAMS, mach: 9 },
      view: { viewer: { wireframe: true }, tab: "heat" },
      snapshots: [
        { id: "a", time: at(1), message: "first", params: { ...DEFAULT_PARAMS, mach: 7 } },
        { id: "b", time: at(2), message: "second", params: { ...DEFAULT_PARAMS, mach: 9 } },
      ],
    };
    const r = parseProject(JSON.stringify(v1));
    assert.equal(r.fromVersion, 1);
    assert.equal(r.migrated, true);
    assert.match(r.warnings.join(" "), /Migrated from schemaVersion 1 to 2/);
    const d = r.doc;
    assert.equal(d.schemaVersion, 2);
    assert.equal(d.state.params.mach, 9);
    assert.equal(d.state.ui.viewer.wireframe, true);
    assert.equal(d.state.ui.viewer.shock, true, "missing viewer fields default");
    assert.equal(d.state.ui.tab, "heat");
    assert.equal(d.state.aeroDb, null);
    assert.deepEqual(d.history.snapshots.map((s) => [s.id, s.parentId, s.branch]), [["a", null, "main"], ["b", "a", "main"]]);
    assert.equal(d.history.head, "b");
    // the migrated doc saves as current schema and reloads without migrating again
    const again = parseProject(serializeProject(d));
    assert.equal(again.migrated, false);
  });

  it("v0 (bare design-params JSON, the old localStorage blob) imports", () => {
    const r = parseProject(JSON.stringify({ ...DEFAULT_PARAMS, name: "legacy", span: 3 }));
    assert.equal(r.fromVersion, 0);
    assert.equal(r.doc.state.params.span, 3);
    assert.equal(r.doc.name, "legacy");
    assert.equal(r.doc.history.snapshots.length, 0);
  });

  it("every version below current has a migration", () => {
    for (let v = 0; v < PROJECT_SCHEMA_VERSION; v++) assert.equal(typeof MIGRATIONS[v], "function", `missing ${v}→${v + 1}`);
  });

  it("a file from a newer app is refused with a clear message", () => {
    const doc = JSON.parse(serializeProject(sampleDoc()));
    doc.schemaVersion = PROJECT_SCHEMA_VERSION + 1;
    assert.throws(() => parseProject(JSON.stringify(doc)), /schemaVersion 3 is newer than this app supports \(2\)/);
  });
});

describe("version history", () => {
  it("restore of a snapshot returns its exact state and moves head", () => {
    const doc = sampleDoc();
    const { doc: d2, state } = restoreSnapshot(doc, "s1");
    assert.equal(state.params.mach, 8);
    assert.equal(state.params.span, DEFAULT_PARAMS.span);
    assert.equal(canonicalJson(state), canonicalJson(doc.history.snapshots[0]!.state));
    assert.equal(d2.history.head, "s1");
    // restored state is a copy: mutating it cannot corrupt history
    state.params.mach = 99;
    assert.equal(d2.history.snapshots[0]!.state.params.mach, 8);
    // snapshot after restore is parented on the restored snapshot
    const { snapshot } = commitSnapshot(d2, withParams(state, { mach: 6 }), "M6 from baseline", { nowISO: at(5), id: "s3" });
    assert.equal(snapshot.parentId, "s1");
    assert.equal(snapshot.summary, "mach 8→6");
  });

  it("restore survives save/load of the file", () => {
    const back = parseProject(serializeProject(sampleDoc())).doc;
    assert.equal(restoreSnapshot(back, "s1").state.params.mach, 8);
    assert.throws(() => restoreSnapshot(back, "nope"), /not found/);
  });

  it("diff summary, list, delete (re-parent), branch, export/import", () => {
    let doc = sampleDoc();
    const s2 = doc.history.snapshots[1]!;
    assert.equal(s2.summary, "mach 8→10, span 2.2→2.5");
    assert.equal(s2.changes, 2);
    assert.deepEqual(listSnapshots(doc).map((s) => s.id), ["s2", "s1"]);

    const b = branchFromSnapshot(doc, "s1", "low-mach", { nowISO: at(3), id: "s1b" });
    doc = b.doc;
    assert.equal(doc.history.branch, "low-mach");
    assert.deepEqual(doc.history.branches, ["main", "low-mach"]);
    assert.equal(b.state.params.mach, 8);
    doc = commitSnapshot(doc, withParams(b.state, { mach: 6 }), "M6", { nowISO: at(4), id: "s4" }).doc;
    assert.deepEqual(listSnapshots(doc, "low-mach").map((s) => s.id), ["s4", "s1b"]);

    const exported = exportHistory(doc, at(5));
    doc = deleteSnapshot(doc, "s1b");
    const s4 = doc.history.snapshots.find((s) => s.id === "s4")!;
    assert.equal(s4.parentId, "s1");
    assert.equal(s4.summary, "mach 8→6");

    const { data } = parseHistoryExport(exported);
    const merged = importHistory(doc, data, "merge");
    assert.equal(merged.added, 1);
    assert.equal(merged.doc.history.snapshots.length, 4);
    const fresh = newProject(DEFAULT_PARAMS, { nowISO: T0 });
    const replaced = importHistory(fresh, data, "replace").doc;
    assert.equal(replaced.history.snapshots.length, 4);

    const d = diffStates(doc.history.snapshots[0]!.state, withParams(doc.history.snapshots[0]!.state, { fins: 2, aeroMethod: "newtonian" }));
    assert.deepEqual(d.map((c) => c.path), ["params.aeroMethod", "params.fins"]);
    assert.equal(summarizeDiff(d), "aeroMethod euler→newtonian, fins 4→2");
  });

  it("UI/view state does not count as a design change", () => {
    const doc = sampleDoc();
    const s = { ...doc.state, ui: { ...doc.state.ui, tab: "cad" } };
    assert.ok(sameDesign(s, doc.state));
    assert.equal(diffStates(doc.state, s).length, 0);
  });
});

describe("rejects corrupted / invalid files with a clear error", () => {
  const good = () => JSON.parse(serializeProject(sampleDoc()));
  const cases: [string, () => string, RegExp][] = [
    ["truncated JSON", () => serializeProject(sampleDoc()).slice(0, 300), /not valid JSON .*truncated or corrupted/],
    ["empty file", () => "   ", /is empty/],
    ["not an object", () => "[1,2,3]", /top level must be a JSON object/],
    ["unrelated JSON", () => JSON.stringify({ hello: "world" }), /not a Bowshock project/],
    ["wrong format", () => JSON.stringify({ ...good(), format: "something-else" }), /at format: expected format "bowshock-project"/],
    ["history export opened as project", () => exportHistory(sampleDoc()), /version-history export; use History → Import/],
    ["bad schemaVersion", () => JSON.stringify({ ...good(), schemaVersion: "2" }), /schemaVersion must be a positive integer/],
    ["params missing", () => { const g = good(); delete g.state.params; return JSON.stringify(g); }, /at state\.params: design parameters must be an object/],
    ["family missing", () => { const g = good(); delete g.state.params.family; return JSON.stringify(g); }, /at state\.params\.family: missing "family"/],
    ["param wrong type", () => { const g = good(); g.state.params.mach = [8]; return JSON.stringify(g); }, /at state\.params\.mach: expected a number, string or boolean, got array/],
    ["snapshot bad timestamp", () => { const g = good(); g.history.snapshots[0].createdISO = "yesterday"; return JSON.stringify(g); }, /at history\.snapshots\[0\]\.createdISO: expected an ISO-8601 timestamp/],
    ["duplicate snapshot id", () => { const g = good(); g.history.snapshots[1].id = "s1"; return JSON.stringify(g); }, /duplicate snapshot id "s1"/],
    ["snapshot state corrupt", () => { const g = good(); g.history.snapshots[1].state.params = "oops"; return JSON.stringify(g); }, /at history\.snapshots\[1\]\.state\.params/],
    ["aero DB points not array", () => { const g = good(); g.state.aeroDb.database.points = {}; return JSON.stringify(g); }, /at state\.aeroDb\.database\.points: points must be an array/],
    ["aero DB point corrupt", () => { const g = good(); g.state.aeroDb.database.points[2].cl = "x"; return JSON.stringify(g); }, /at state\.aeroDb\.database\.points\[2\]: each point needs numeric/],
    ["aero DB spec corrupt", () => { const g = good(); delete g.state.aeroDb.spec.machStep; return JSON.stringify(g); }, /at state\.aeroDb\.spec\.machStep: expected a finite number/],
    ["snapshots not array", () => { const g = good(); g.history.snapshots = "x"; return JSON.stringify(g); }, /at history\.snapshots: snapshots must be an array/],
  ];
  for (const [name, make, re] of cases) {
    it(name, () => {
      assert.throws(() => parseProject(make()), (e: unknown) => {
        assert.ok(e instanceof ProjectFileError, `expected ProjectFileError, got ${String(e)}`);
        assert.match(e.message, re);
        return true;
      });
    });
  }

  it("prototype-pollution keys are dropped, not applied", () => {
    const text = serializeProject(sampleDoc()).replace('"family"', '"__proto__": {"polluted": true}, "family"');
    const { doc } = parseProject(text);
    assert.equal(({} as Record<string, unknown>).polluted, undefined);
    assert.equal(Object.prototype.hasOwnProperty.call(doc.state.params, "__proto__"), false);
  });

  it("hand edits load with a checksum warning; out-of-range values are clamped by the sanitizer", () => {
    const g = JSON.parse(serializeProject(sampleDoc()));
    g.state.params.nx = 100000;
    const r = parseProject(JSON.stringify(g), { sanitizeParams: sanitize });
    assert.equal(r.doc.state.params.nx, 160);
    assert.match(r.warnings.join(" "), /Checksum mismatch/);
    assert.match(r.warnings.join(" "), /state\.params: .*nx/);
  });

  it("history import rejects a project file", () => {
    assert.throws(() => parseHistoryExport(serializeProject(sampleDoc())), /this is a project file; use Open/);
  });
});

describe("variant compare data builder", () => {
  const metricsOf = (p: DesignParams): VariantMetrics => ({
    ld: 10 / p.mach, cl: 0.1, cd: 0.01 * p.mach, cm: 0, staticMargin: 0.05, volume: p.span, sRef: p.span * p.length,
    volEff: 0.1, mass: 100 * p.span, qStag: p.mach * 10, rangeKm: 1000 / p.mach, method: "cbaero",
  });

  it("lists working copy, branch tips and snapshots", () => {
    let doc = sampleDoc();
    doc = branchFromSnapshot(doc, "s1", "alt", { nowISO: at(3), id: "sb" }).doc;
    const c = compareCandidates(doc, doc.state);
    assert.deepEqual(c.map((e) => e.id), ["working", "tip:main", "tip:alt", "sb", "s2", "s1"]);
  });

  it("param diff rows, metrics with best markers, identical designs evaluated once", () => {
    const doc = sampleDoc();
    const [s1, s2] = doc.history.snapshots;
    let calls = 0;
    const evaluate = (p: DesignParams) => (calls++, metricsOf(p));
    const s3 = { id: "x", label: "copy of s2", state: s2!.state };
    const d = buildCompare([{ id: "a", label: "s1", state: s1!.state }, { id: "b", label: "s2", state: s2!.state }, s3], { evaluate });
    assert.deepEqual(d.params.map((r) => [r.path, r.values]), [["params.mach", [8, 10, 10]], ["params.span", [2.2, 2.5, 2.5]]]);
    assert.equal(calls, 2, "s2 and its copy share one evaluation");
    const ld = d.metrics!.find((m) => m.key === "ld")!;
    assert.deepEqual(ld.values, [1.25, 1, 1]);
    assert.equal(ld.best, 0);
    const cl = d.metrics!.find((m) => m.key === "cl")!;
    assert.equal(cl.best, null, "no 'better' direction → no best");
    assert.equal(cl.spread, 0);
  });

  it("a failed evaluation shows as null, not a number", () => {
    const doc = sampleDoc();
    const d = buildCompare(doc.history.snapshots.map((s) => ({ id: s.id, label: s.id, state: s.state })), {
      evaluate: (p) => (p.mach === 10 ? { ...metricsOf(p), error: "build failed" } : metricsOf(p)),
    });
    assert.deepEqual(d.metrics!.find((m) => m.key === "ld")!.values, [1.25, null]);
  });

  it("aero DB overlay: CL/CD/LD vs α per version at a common Mach; versions without a DB are listed", () => {
    const doc = sampleDoc();
    const a = { id: "a", label: "k1", state: doc.history.snapshots[0]!.state };
    const b = { id: "b", label: "k2", state: { ...a.state, params: { ...a.state.params, mach: 9 }, aeroDb: fakeDb(2, [7, 9], [0, 4, 12]) } };
    const c = { id: "c", label: "no db", state: { ...a.state, params: { ...a.state.params, span: 3 }, aeroDb: null } };
    const d = buildCompare([a, b, c]);
    assert.equal(d.metrics, null, "metrics only when an evaluator is supplied");
    const cv = d.curves!;
    assert.equal(cv.mach, 8);
    assert.deepEqual(cv.series.map((s) => [s.entryId, s.mach]), [["a", 8], ["b", 7]]);
    assert.deepEqual(cv.missing, ["no db"]);
    assert.deepEqual(cv.rows.map((r) => r.alpha), [-4, 0, 4, 8, 12]);
    const r4 = cv.rows.find((r) => r.alpha === 4)!;
    assert.ok(Math.abs(r4.cl0! - 0.08) < 1e-12 && Math.abs(r4.cl1! - 0.16) < 1e-12);
    assert.equal(cv.rows.find((r) => r.alpha === 12)!.cl0, null);
    assert.ok(Math.abs(r4.ld0! - 0.08 / 0.026) < 1e-12);
    assert.deepEqual(cv.stale, [], "untagged databases are not flagged");
  });

  it("flags a database carried over to a different design as stale", () => {
    const base = sampleDoc().history.snapshots[0]!.state;
    const tagged = { ...base, aeroDb: { ...fakeDb(1), builtFor: paramsHash(base.params) } };
    const moved = { ...tagged, params: { ...tagged.params, span: 3.1 } };
    assert.equal(aeroDbStale(tagged.aeroDb, tagged.params), false);
    assert.equal(aeroDbStale(moved.aeroDb, moved.params), true);
    const cv = buildCompare([{ id: "a", label: "fresh", state: tagged }, { id: "b", label: "moved", state: moved }]).curves!;
    assert.deepEqual(cv.series.map((s) => s.stale), [false, true]);
    assert.deepEqual(cv.stale, ["moved"]);
    // builtFor survives the file round trip
    const d = newProject(moved.params, { aeroDb: moved.aeroDb, nowISO: T0 });
    assert.equal(parseProject(serializeProject(d)).doc.state.aeroDb!.builtFor, paramsHash(base.params));
  });

  it("alphaCurve picks β≈0, δ≈0 and the nearest Mach", () => {
    const db = fakeDb(1).database!;
    db.points.push({ ...db.points[0]!, beta_deg: 4, cl: 99 });
    const c = alphaCurve(db, 8.9);
    assert.equal(c.mach, 8);
    assert.equal(c.points.length, 4);
    assert.ok(c.points.every((p) => p.cl !== 99));
  });

  it("rejects fewer than 2, more than 4, or duplicate picks", () => {
    const s = sampleDoc().history.snapshots[0]!.state;
    const e = (id: string) => ({ id, label: id, state: s });
    assert.throws(() => buildCompare([e("a")]), /Pick 2 to 4/);
    assert.throws(() => buildCompare([e("a"), e("b"), e("c"), e("d"), e("e")]), /Pick 2 to 4/);
    assert.throws(() => buildCompare([e("a"), e("a")]), /picked twice/);
  });
});

describe("builds on the Iterate lab's browser lists", () => {
  it("imports cuspis.history.v1 / cuspis.variants.v1 as an 'iterate' branch, idempotently", () => {
    const doc = sampleDoc();
    const history = [{ at: Date.parse(at(10)), name: "h1", params: { ...DEFAULT_PARAMS, mach: 12 } }];
    const variants = [{ id: "v1", at: Date.parse(at(11)), name: "v", ld: 4.2, params: { ...DEFAULT_PARAMS, mach: 7 } }, { bogus: true }];
    const r = importIterateLists(doc, { history, variants }, doc.state, sanitize);
    assert.equal(r.added, 2);
    const it2 = listSnapshots(r.doc, "iterate");
    assert.deepEqual(it2.map((s) => s.message), ["Iterate variant · v (L/D 4.20)", "Iterate history · h1"]);
    assert.equal(it2[0]!.summary, "mach 12→7");
    assert.ok(r.doc.history.branches.includes("iterate"));
    assert.equal(importIterateLists(r.doc, { history, variants }, doc.state).added, 0);
    // and it survives a file round trip
    assert.equal(parseProject(serializeProject(r.doc)).doc.history.snapshots.length, 4);
  });
});
