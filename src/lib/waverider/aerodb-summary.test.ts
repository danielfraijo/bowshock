/**
 * Aero database summary, plot series, list axes, and the zip bundle.
 * Run: npm run test:aerodb-ui
 */
import assert from "node:assert/strict";
import { generateAeroDatabase, gridCount, listBreaks, type AeroDatabase, type AeroDbPoint, type AeroDbSpec } from "./aerodb";
import {
  aeroDbCoefficientCsvs,
  aeroDbZip,
  coefRanges,
  coefVsAlphaByMach,
  coefVsDeltaByMach,
  machKey,
  parseAxisList,
  summarizeAeroDb,
  trimCrossing,
} from "./aerodb-summary";
import { buildVehicle } from "./generate";
import { DEFAULT_PARAMS } from "./types";

const DEG = Math.PI / 180;
const close = (a: number | null | undefined, b: number, tol = 1e-9, msg = "") => {
  assert.ok(a !== null && a !== undefined && Math.abs(a - b) <= tol * Math.max(1, Math.abs(b)), `${msg} got ${a}, want ${b}`);
};
let passed = 0;
const ok = (name: string) => {
  passed++;
  console.log(`ok ${passed} - ${name}`);
};

// ---------- trim crossing ----------
{
  const t = trimCrossing([-4, 0, 4, 8], [0.02 + 0.02, 0.02, 0, -0.02]);
  close(t?.alpha, 4, 1e-12, "exact zero");
  const u = trimCrossing([-4, 0, 4, 8], [-4, 0, 4, 8].map((a) => 0.01 - 0.004 * a));
  close(u?.alpha, 2.5, 1e-12, "interpolated");
  close(u?.slope, -0.004, 1e-12, "slope");
  assert.equal(trimCrossing([0, 4, 8], [0.1, 0.05, 0.01]), null);
  ok("trim α: exact zero, linear interpolation, none in range");
}

// ---------- synthetic table with analytic CL(α), Cm(α) ----------
function synthetic(opts: { tableDerivs: boolean; cm0: (m: number) => number }): AeroDatabase {
  const axes = { mach: [4, 6], alpha_deg: [-4, 0, 4, 8], beta_deg: [0], delta_deg: [-5, 0, 5] };
  const points: AeroDbPoint[] = [];
  for (const mach of axes.mach)
    for (let ia = 0; ia < axes.alpha_deg.length; ia++)
      for (const beta of axes.beta_deg)
        for (const delta of axes.delta_deg) {
          const a = axes.alpha_deg[ia];
          const cl = 0.05 * a + 0.004 * delta;
          const cd = 0.02 + 0.001 * a * a;
          const cm = opts.cm0(mach) - 0.01 * a - 0.002 * delta;
          const interior = ia > 0 && ia < axes.alpha_deg.length - 1;
          points.push({
            mach, alpha_deg: a, beta_deg: beta, delta_deg: delta, onDesign: mach === 6,
            cl, cd, cy: 0, cm, cn: 0, cll: 0, ca: cd, ld: cl / cd,
            cla: opts.tableDerivs && interior ? 0.05 / DEG : 0, cda: 0, cma: opts.tableDerivs && interior ? -0.01 / DEG : 0,
            cyb: 0, cnb: 0, clb: 0, cld: 0, cmd: 0, clq: 0, cmq: 0, clp: 0, cnp: 0, cyp: 0, cnr: 0, clr: 0, cyr: 0,
            euler_cl: null, euler_cd: null, euler_cm: null,
          });
        }
  let best = points[0];
  for (const p of points) if (p.ld > best.ld) best = p;
  return {
    meta: {
      format: "cuspis-aerodb-1", name: "synthetic", family: "caret", method: "newtonian", designMach: 6, altitude_km: 30,
      sRef_m2: 2, lRef_m: 3, bRef_m: 1.5, cg_m: [1.8, 0, 0],
      units: { angles: "deg", derivatives: "1/rad", force: "wind axes, CL CD CY", moment: "body axes, Cm Cn Cll, nondim by q·Sref·Lref" },
      frame: "X streamwise (tip at origin, flow +X), Y span, Z up",
      index: "flat i = ((im*nAlpha + ia)*nBeta + ib)*nDelta + id  (delta fastest)",
      generatedAt: "2026-10-22T00:00:00.000Z", assumptions: ["synthetic"],
    },
    axes, points, dynamics: [],
    summary: { points: points.length, onDesign: 12, offDesign: 12, maxLd: best.ld, maxLdAt: { mach: best.mach, alpha_deg: best.alpha_deg, beta_deg: 0, delta_deg: best.delta_deg }, clMin: 0, clMax: 0, cdMin: 0, cdMax: 0, cmAtDesign: 0 },
    slices: { design: { mach: 6, alpha: [] }, offDesign: { mach: 4, alpha: [] } },
  };
}

{
  // M4: Cm = 0.03 − 0.01α → trim at 3°. M6: Cm = 0.2 − 0.01α → no crossing in [−4, 8].
  for (const tableDerivs of [true, false]) {
    const db = synthetic({ tableDerivs, cm0: (m) => (m === 4 ? 0.03 : 0.2) });
    const s = summarizeAeroDb(db, 2);
    assert.deepEqual(s.anchor, { beta_deg: 0, delta_deg: 0 });
    close(s.trim[0].alphaTrim, 3, 1e-12, "M4 trim");
    close(s.trim[0].dCmdAlphaPerDeg, -0.01, 1e-12, "M4 slope");
    assert.equal(s.trim[1].alphaTrim, null, "M6 none in range");
    for (const m of s.margin) {
      close(m.smFrac, 0.2, 1e-9, `SM frac M${m.mach}`);
      close(m.sm_m, 0.6, 1e-9, `SM metres M${m.mach}`);
      close(m.claPerRad, 0.05 / DEG, 1e-9, "CLα per rad");
      assert.equal(m.source, tableDerivs ? "table derivative" : "finite difference");
    }
    // untrimmed Mach falls back to the node nearest the design α
    assert.equal(s.margin[1].alpha_deg, 0);
  }
  ok("static margin = −Cmα/CLα (fraction and metres), from table derivatives and by finite difference");
}

{
  const db = synthetic({ tableDerivs: true, cm0: (m) => (m === 4 ? 0.03 : 0.2) });
  const r = coefRanges(db.points);
  close(r.cl?.min, 0.05 * -4 + 0.004 * -5, 1e-12, "CL min");
  close(r.cl?.max, 0.05 * 8 + 0.004 * 5, 1e-12, "CL max");
  close(r.cm?.min, 0.03 - 0.08 - 0.01, 1e-12, "Cm min");
  close(r.cd?.min, 0.02, 1e-12, "CD min");
  assert.ok(r.ld && r.cy && r.cn && r.cll && r.ca, "all ranges present");
  ok("coefficient ranges cover every point");

  const ld = coefVsAlphaByMach(db, "ld");
  assert.deepEqual(ld.keys, [machKey(4), machKey(6)]);
  assert.deepEqual(ld.rows.map((x) => x.x), [-4, 0, 4, 8]);
  close(ld.rows[2][machKey(6)] as number, (0.05 * 4) / (0.02 + 0.016), 1e-12, "L/D at α 4, δ 0");
  const cmd = coefVsDeltaByMach(db, "cm", 3.1);
  assert.equal(cmd.alpha_deg, 4);
  assert.deepEqual(cmd.rows.map((x) => x.x), [-5, 0, 5]);
  close(cmd.rows[0][machKey(4)] as number, 0.03 - 0.04 + 0.01, 1e-12, "Cm at δ −5");
  ok("L/D vs α per Mach and Cm vs δ per Mach series");

  const csvs = aeroDbCoefficientCsvs(db);
  assert.equal(csvs.length, 8);
  const cl = csvs.find((f) => f.name === "coefficients/CL.csv")!;
  const rows = cl.text.trim().split("\n").filter((l) => !l.startsWith("#"));
  assert.equal(rows[0], "mach,alpha_deg,beta_deg,delta_-5,delta_0,delta_5");
  assert.equal(rows.length, 1 + 2 * 4 * 1);
  const m6a8 = rows.find((l) => l.startsWith("6,8,0,"))!.split(",").map(Number);
  close(m6a8[3], 0.38, 1e-9, "CL(6,8,0,-5)");
  close(m6a8[5], 0.42, 1e-9, "CL(6,8,0,5)");
  ok("per-coefficient CSV: one column per δ, values in flat-index order");

  const z = aeroDbZip(db, 2);
  const bytes = new Uint8Array(await z.blob.arrayBuffer());
  assert.deepEqual([...bytes.slice(0, 4)], [0x50, 0x4b, 0x03, 0x04], "zip local header");
  const eocd = bytes.length - 22;
  assert.deepEqual([...bytes.slice(eocd, eocd + 4)], [0x50, 0x4b, 0x05, 0x06], "end of central directory");
  const count = bytes[eocd + 10] | (bytes[eocd + 11] << 8);
  assert.equal(count, z.entries.length);
  const text = new TextDecoder().decode(bytes);
  for (const n of ["aerodb.csv", "aerodb.json", "cuspis_aero_db.m", "coefficients/Cm.csv", "README.txt"]) {
    assert.ok(z.entries.includes(n) && text.includes(n), `zip has ${n}`);
  }
  assert.ok(text.includes("trim α 3.00°") && text.includes("none in range"), "README carries the trim summary");
  // JSON round-trips through the stored (uncompressed) entry
  const jStart = text.indexOf('{\n  "meta"');
  const jEnd = text.indexOf("PK\u0003\u0004", jStart);
  const back = JSON.parse(text.slice(jStart, jEnd)) as AeroDatabase;
  assert.equal(back.points.length, db.points.length);
  assert.deepEqual(back.axes, db.axes);
  ok(`zip bundle: ${z.entries.length} entries (CSV, JSON, .m, per-coefficient CSVs, README)`);
}

// ---------- list axes ----------
{
  assert.deepEqual(parseAxisList("−4, 0,4  8;12"), [-4, 0, 4, 8, 12]);
  assert.deepEqual(parseAxisList("  "), []);
  assert.throws(() => parseAxisList("1, two"));
  assert.deepEqual(listBreaks([8, 4, 4, 12], [6]), [4, 6, 8, 12]);
  assert.deepEqual(listBreaks([8, 4], [20]), [4, 8], "force outside the span is ignored");
  const base: AeroDbSpec = {
    machMin: 4, machMax: 8, machStep: 2, alphaMin: -4, alphaMax: 12, alphaStep: 4, betaMin: 0, betaMax: 0, betaStep: 4,
    deltaMin: 0, deltaMax: 0, deltaStep: 5, method: "newtonian", eulerSymmetry: false, dynamicDerivatives: false, altKm: 30,
  };
  const g = gridCount({ ...base, machList: [5, 9, 7], alphaList: [1, 3], deltaList: [-10, 10] }, 6, 2);
  assert.deepEqual(g.axes.mach, [5, 6, 7, 9]);
  assert.deepEqual(g.axes.alpha_deg, [1, 2, 3]);
  assert.deepEqual(g.axes.beta_deg, [0]);
  assert.deepEqual(g.axes.delta_deg, [-10, 0, 10]);
  assert.equal(g.points, 4 * 3 * 1 * 3);
  const r = gridCount({ ...base, machList: [] }, 6, 2);
  assert.deepEqual(r.axes.mach, [4, 6, 8], "empty list keeps the range");
  ok("list axes: parse, sort/dedupe, design values forced in, empty list falls back to min/max/step");
}

// ---------- end-to-end on the real kernel ----------
{
  const built = buildVehicle({ ...DEFAULT_PARAMS, family: "caret", aeroMethod: "newtonian", nx: 18, ny: 14, mach: 8, length: 3, span: 1.6, height: 0.4, shockDeg: 14 });
  const db = await generateAeroDatabase(built, {
    machMin: 0, machMax: 0, machStep: 1, alphaMin: 0, alphaMax: 0, alphaStep: 1, betaMin: 0, betaMax: 0, betaStep: 1,
    deltaMin: 0, deltaMax: 0, deltaStep: 1, method: "newtonian", eulerSymmetry: false, dynamicDerivatives: false, altKm: 30,
    machList: [6, 10], alphaList: [-2, 2, 6, 10], deltaList: [-5, 5],
  });
  assert.deepEqual(db.axes.mach, [6, 8, 10]);
  assert.deepEqual(db.axes.delta_deg, [-5, 0, 5]);
  const tagged = db.points.filter((p) => p.onDesign).map((p) => p.mach);
  assert.ok(tagged.length > 0 && tagged.every((m) => m === 8), "only the design Mach is on-design with a list axis");
  const s = summarizeAeroDb(db, built.params.alphaDeg);
  assert.equal(s.trim.length, 3);
  assert.equal(s.margin.length, 3);
  for (const m of s.margin) assert.ok(m.smFrac !== null && Number.isFinite(m.smFrac) && Number.isFinite(m.sm_m!), `finite SM M${m.mach}`);
  const cmd = coefVsDeltaByMach(db, "cm", built.params.alphaDeg);
  assert.ok(cmd.rows.every((r) => cmd.keys.every((k) => Number.isFinite(r[k] as number))), "Cm vs δ finite");
  const z = aeroDbZip(db, built.params.alphaDeg);
  assert.equal(z.entries.length, 3 + 8 + 1);
  console.log(
    `  kernel: ${db.points.length} pts, maxL/D ${db.summary.maxLd.toFixed(2)}, ` +
      s.trim.map((t, i) => `M${t.mach} trim ${t.alphaTrim === null ? "none" : t.alphaTrim.toFixed(2)} SM ${(s.margin[i].smFrac! * 100).toFixed(1)}%`).join(", "),
  );
  ok("generateAeroDatabase with list axes → summary, Cm vs δ, zip");
}

console.log(`aerodb-ui: ${passed} passed`);
