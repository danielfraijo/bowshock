/**
 * Data package tests. Run: node --import ./scripts/ts-register.mjs src/lib/waverider/datapack.test.ts
 *  - key: identical inputs → identical key; every non-cosmetic input changes it; name/unit do not
 *  - relevant tables change when a geometry / flight variable changes (and only then)
 *  - determinism, memoization, async == sync
 *  - CSV / JSON round trips
 *  - three families end to end (caret, cone, liftbody)
 */
import assert from "node:assert/strict";
import {
  clearDataPackCache,
  computeDataPack,
  computeDataPackAsync,
  datapackKey,
  packFiles,
  packToJson,
  parseCsv,
  tableToCsv,
  type DataPack,
  type DataPackOptions,
  type DataTable,
} from "./datapack";
import { DEFAULT_PARAMS, type DesignParams } from "./types";

const NOW = "2026-01-01T00:00:00.000Z";
// small, fast options for the one-at-a-time tests
const FAST: DataPackOptions = {
  alphaSweep: { min: 0, max: 4, n: 3 },
  machSweep: { min: 6, max: 10, n: 2 },
  carpet: { alphaMin: 0, alphaMax: 4, na: 2, machMin: 6, machMax: 10, nm: 2 },
  stations: 8,
  panels: false,
  now: NOW,
};
const base: DesignParams = { ...DEFAULT_PARAMS, family: "caret", aeroMethod: "mixed", nx: 24, ny: 16 };
const tbl = (p: DataPack, id: string): DataTable => {
  const t = p.tables.find((x) => x.id === id);
  assert.ok(t, `table ${id}`);
  return t;
};
const val = (p: DataPack, table: string, name: string, col = 1): number | string | boolean | null => {
  const t = tbl(p, table);
  const r = t.rows.find((x) => x[0] === name);
  assert.ok(r, `${table}.${name}`);
  return r[col];
};

/* ---- key ---- */
{
  const k0 = datapackKey(base, FAST);
  assert.equal(datapackKey({ ...base }, { ...FAST }), k0, "same inputs, same key");
  assert.equal(datapackKey({ ...base, name: "other", unit: "mm" }, FAST), k0, "name/unit are cosmetic");
  assert.equal(datapackKey({ ...base, name: "other" }, { ...FAST, now: "2030-01-01", gitHash: "abc" }), k0, "provenance does not enter the key");
  const alt: Record<string, unknown> = {
    family: "cone",
    planform: "arrow",
    lid: "bottom",
    cowlSide: "top",
    aeroMethod: "tangent",
    fuel: "JP10",
    unit: "mm",
    name: "x",
  };
  const changed: string[] = [];
  const missed: string[] = [];
  for (const k of Object.keys(DEFAULT_PARAMS) as (keyof DesignParams)[]) {
    if (k === "name" || k === "unit") continue;
    const cur = base[k];
    let next: unknown;
    if (typeof cur === "number") next = cur * 1.07 + 0.013;
    else if (typeof cur === "boolean") next = !cur;
    else next = alt[k] ?? `${String(cur)}_x`;
    const kk = datapackKey({ ...base, [k]: next } as DesignParams, FAST);
    if (kk === k0) missed.push(k);
    else changed.push(k);
  }
  assert.deepEqual(missed, [], `inputs that did not change the key: ${missed.join(",")}`);
  console.log(`key: ${changed.length} inputs each change the key; name/unit/provenance do not`);
  // option changes
  assert.notEqual(datapackKey(base, { ...FAST, stations: 9 }), k0);
  assert.notEqual(datapackKey(base, { ...FAST, alphaSweep: { min: 0, max: 5, n: 3 } }), k0);
  assert.notEqual(datapackKey(base, { ...FAST, parametric: { variable: "span", min: 1, max: 2, n: 3 } }), k0);
}

/* ---- relevant tables change ---- */
{
  clearDataPackCache();
  const p0 = computeDataPack(base, FAST);
  const t0 = performance.now();
  const again = computeDataPack({ ...base, name: "renamed" }, FAST);
  assert.equal(again, p0, "cosmetic change is a cache hit (same object)");
  assert.ok(performance.now() - t0 < 25, "cache hit is instant");

  type Case = { label: string; patch: Partial<DesignParams>; table: string; row: string };
  const cases: Case[] = [
    { label: "shockDeg", patch: { shockDeg: 18 }, table: "geometry", row: "generating_wedge_theta_deg" },
    { label: "span", patch: { span: 2.6 }, table: "geometry", row: "planform_area_m2" },
    { label: "teSweepDeg", patch: { teSweepDeg: 10 }, table: "geometry", row: "volume_m3" },
    { label: "length", patch: { length: 5 }, table: "geometry", row: "bbox_max_x_m" },
    { label: "leRadius", patch: { leRadius: 0.04 }, table: "results_design_point", row: "q_stag_W_cm2" },
    { label: "alphaDeg", patch: { alphaDeg: 3 }, table: "results_design_point", row: "CL" },
    { label: "altKm", patch: { altKm: 40 }, table: "flight_conditions", row: "p_inf_Pa" },
    { label: "twK", patch: { twK: 1200 }, table: "results_design_point", row: "q_max_W_cm2" },
    { label: "cgFrac", patch: { cgFrac: 0.7 }, table: "results_design_point", row: "Cm" },
    { label: "flightMach", patch: { lockFlight: false, flightMach: 6 }, table: "flight_conditions", row: "flight_mach" },
    { label: "aeroMethod", patch: { aeroMethod: "tangent" }, table: "results_design_point", row: "method" },
    { label: "gamma", patch: { gamma: 1.3 }, table: "flight_conditions", row: "gamma" },
  ];
  for (const c of cases) {
    const p1 = computeDataPack({ ...base, ...c.patch }, FAST);
    assert.notEqual(p1.key, p0.key, `${c.label}: key`);
    const a = val(p0, c.table, c.row);
    const b = val(p1, c.table, c.row);
    assert.notEqual(a, b, `${c.label}: ${c.table}.${c.row} should change (${a} vs ${b})`);
    console.log(`live: ${c.label.padEnd(10)} → ${c.table}.${c.row}: ${typeof a === "number" ? a.toPrecision(5) : a} → ${typeof b === "number" ? b.toPrecision(5) : b}`);
  }
  // a sweep table responds too
  const pa = computeDataPack({ ...base, shockDeg: 18 }, FAST);
  assert.notDeepEqual(tbl(pa, "sweep_alpha").rows, tbl(p0, "sweep_alpha").rows, "α sweep follows geometry");
  assert.notDeepEqual(tbl(pa, "stations_streamwise").rows, tbl(p0, "stations_streamwise").rows, "stations follow geometry");
}

/* ---- determinism, async == sync ---- */
{
  clearDataPackCache();
  const a = computeDataPack(base, FAST);
  const ja = packToJson(a);
  clearDataPackCache();
  const b = computeDataPack(base, FAST);
  assert.equal(packToJson(b), ja, "recompute is bit-identical");
  assert.equal(a.key, b.key);
  clearDataPackCache();
  const c = await computeDataPackAsync(base, FAST);
  assert.equal(packToJson(c), ja, "async == sync");
  const ctl = new AbortController();
  clearDataPackCache();
  const pr = computeDataPackAsync(base, FAST, { signal: ctl.signal });
  ctl.abort();
  await assert.rejects(pr, /aborted/);
  console.log("determinism: sync/async/recompute identical; abort works");
}

/* ---- round trips ---- */
function numClose(a: number, b: number) {
  return Math.abs(a - b) <= 1e-8 * Math.max(1, Math.abs(a), Math.abs(b));
}
{
  const p = computeDataPack({ ...base, nx: 16, ny: 10 }, { ...FAST, panels: true });
  let cells = 0;
  for (const t of p.tables) {
    const rows = parseCsv(tableToCsv(t));
    assert.equal(rows.length, t.rows.length + 1, `${t.id}: row count`);
    assert.deepEqual(rows[0], t.columns.map((c) => c.key), `${t.id}: header`);
    for (let i = 0; i < t.rows.length; i++) {
      assert.equal(rows[i + 1].length, t.columns.length, `${t.id} row ${i}: width`);
      for (let j = 0; j < t.columns.length; j++) {
        const o = t.rows[i][j];
        const s = rows[i + 1][j];
        cells++;
        if (typeof o === "number") {
          if (Number.isFinite(o)) assert.ok(numClose(Number(s), o), `${t.id}[${i}][${j}] ${s} vs ${o}`);
          else assert.equal(s, "");
        } else if (typeof o === "boolean") assert.equal(s, o ? "TRUE" : "FALSE");
        else if (o === null) assert.equal(s, "");
        else assert.equal(s, o);
      }
    }
  }
  const j = JSON.parse(packToJson(p)) as { format: string; key: string; tables: { id: string; columns: unknown[]; rows: unknown[][] }[] };
  assert.equal(j.key, p.key);
  assert.equal(j.tables.length, p.tables.length);
  for (const t of p.tables) {
    const jt = j.tables.find((x) => x.id === t.id);
    assert.ok(jt);
    assert.equal(jt.rows.length, t.rows.length);
    assert.equal(jt.columns.length, t.columns.length);
    for (let i = 0; i < t.rows.length; i++)
      for (let c = 0; c < t.columns.length; c++) {
        const o = t.rows[i][c];
        const v: unknown = jt.rows[i][c];
        if (typeof o === "number" && !Number.isFinite(o)) assert.equal(v, null);
        else assert.ok(v === o, `json ${t.id}[${i}][${c}] ${String(v)} vs ${String(o)}`);
      }
  }
  const files = packFiles(p);
  assert.ok(files.some((f) => f.name === "data/README.txt") && files.some((f) => f.name === "data/datapack.json"));
  assert.equal(files.filter((f) => f.name.endsWith(".csv")).length, p.tables.length);
  // commas / quotes survive
  const tricky: DataTable = { id: "t", title: "t", description: "d", columns: [{ key: "a", unit: "-", desc: "" }, { key: "b", unit: "-", desc: "" }], rows: [["he said \"hi\", ok", "line1\nline2"], [1.5, null]] };
  const back = parseCsv(tableToCsv(tricky));
  assert.deepEqual(back[1], ["he said \"hi\", ok", "line1\nline2"]);
  console.log(`round trip: ${p.tables.length} tables, ${cells} CSV cells, JSON identical`);
}

/* ---- every unit is in the dictionary; every table is non-empty and finite-or-null ---- */
for (const family of ["caret", "cone", "liftbody"] as const) {
  clearDataPackCache();
  const p = computeDataPack({ ...DEFAULT_PARAMS, family, aeroMethod: "mixed", nx: 24, ny: 16 }, { ...FAST, panels: true, now: NOW });
  for (const id of ["flight_conditions", "surface_panels", "stations_streamwise", "stations_spanwise", "sweep_alpha", "sweep_mach", "sweep_mach_alpha", "heating_peaks", "shock_standoff", "stability", "validation", "provenance", "results_design_point", "geometry", "design_inputs"]) {
    const t = tbl(p, id);
    assert.ok(t.rows.length > 0, `${family}.${id} non-empty`);
    for (const r of t.rows) assert.equal(r.length, t.columns.length, `${family}.${id} ragged`);
  }
  const panels = tbl(p, "surface_panels");
  const iCp = panels.columns.findIndex((c) => c.key === "Cp");
  assert.ok(panels.rows.every((r) => Number.isFinite(r[iCp] as number)), `${family}: Cp finite`);
  const iQ = panels.columns.findIndex((c) => c.key === "q_heat_W_cm2");
  assert.ok(panels.rows.every((r) => Number.isFinite(r[iQ] as number) && (r[iQ] as number) >= 0), `${family}: q finite`);
  const v = tbl(p, "validation");
  const iPass = v.columns.findIndex((c) => c.key === "pass");
  const iScope = v.columns.findIndex((c) => c.key === "scope");
  const codeFail = v.rows.filter((r) => r[iScope] === "code" && r[iPass] === false);
  assert.equal(codeFail.length, 0, `${family}: code checks pass`);
  const designFail = v.rows.filter((r) => r[iScope] === "design" && r[iPass] === false).map((r) => r[0]);
  assert.deepEqual(designFail.filter((x) => String(x).startsWith("design-sum") || x === "design-closure"), [], `${family}: table-vs-integral consistency`);
  console.log(`${family}: ${p.tables.length} tables, ${panels.rows.length} panels, ${p.validation.pass}/${p.validation.n} checks, design-check failures: [${designFail.join(", ")}], ${Math.round(p.elapsedMs)} ms`);
}

/* ---- parametric sweep feeds the same schema ---- */
{
  const p = computeDataPack(base, { ...FAST, parametric: { variable: "span", min: 1.8, max: 2.6, n: 3 } });
  const t = tbl(p, "sweep_parametric");
  assert.equal(t.rows.length, 3);
  assert.equal(t.columns[0].key, "span");
  assert.ok(Number(t.rows[2][9]) > Number(t.rows[0][9]), "planform area grows with span");
  assert.equal(parseCsv(tableToCsv(t)).length, 4);
}
console.log("datapack tests passed");
