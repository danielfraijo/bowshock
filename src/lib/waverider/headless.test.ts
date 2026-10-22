/** [aerodb-headless] Tests for the headless/batch CLI (headless-config.ts, headless-cli.ts, scripts/cuspis-eval.ts). */
import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { existsSync, mkdtempSync, readFileSync, readdirSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { dirname, join, resolve } from "node:path";
import test from "node:test";
import { fileURLToPath } from "node:url";
import { AERODB_POINT_CAP } from "./aerodb";
import { ConfigError, validateHeadlessConfig } from "./headless-config";
import { EXIT, UsageError, hashHex, parseArgs, runCli } from "./headless-cli";

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), "../../..");
const CSV_HEADER =
  "mach,alpha_deg,beta_deg,delta_deg,on_design,CL,CD,CY,Cm,Cn,Cll,CA,LD,CLa_per_rad,CDa_per_rad,Cma_per_rad,CYb_per_rad,Cnb_per_rad,Clb_per_rad,CLd_per_rad,Cmd_per_rad,CLq,Cmq,Clp,Cnp,Cyp,Cnr,Clr,Cyr,euler_CL,euler_CD,euler_Cm";

const small = (extra: Record<string, unknown> = {}) => ({
  name: "tiny",
  geometry: { preset: "caret-m8", params: { nx: 12, ny: 8 } },
  aerodb: { machMin: 6, machMax: 8, machStep: 2, alphaMin: 0, alphaMax: 4, alphaStep: 2, betaMin: -2, betaMax: 2, betaStep: 2, deltaMin: 0, deltaMax: 0, deltaStep: 1, method: "newtonian", dynamicDerivatives: true, altKm: 30 },
  ...extra,
});

function tmp(): string {
  return mkdtempSync(join(tmpdir(), "aerodb-headless-"));
}
function capture() {
  const o = { err: "", out: "" };
  return { o, io: { stderr: (s: string) => void (o.err += s), stdout: (s: string) => void (o.out += s) } };
}
async function runWith(cfg: unknown, args: string[] = [], signal?: AbortSignal) {
  const dir = tmp();
  const cfgPath = join(dir, "cfg.json");
  writeFileSync(cfgPath, JSON.stringify(cfg));
  const out = join(dir, "out");
  const c = capture();
  const code = await runCli(["--config", cfgPath, "--out", out, ...args], c.io, signal);
  return { code, out, ...c };
}

test("parseArgs: flags, legacy positional, usage errors", () => {
  assert.deepEqual(parseArgs(["design.json"]).legacy, "design.json");
  const a = parseArgs(["--config", "c.json", "--out", "o", "--mode", "fly", "--reproducible", "--quiet"]);
  assert.equal(a.mode, "fly");
  assert.ok(a.reproducible && a.quiet);
  assert.ok(parseArgs(["--config", "c.json", "--validate-only"]).validateOnly);
  assert.ok(parseArgs(["--help"]).help);
  for (const bad of [["--out", "o"], ["--config", "c.json"], ["--config"], ["--config", "c", "--out", "o", "--mode", "cfd"], ["--frob"]]) assert.throws(() => parseArgs(bad), UsageError, bad.join(" "));
});

test("validateHeadlessConfig: shipped examples are valid in every mode they target", () => {
  for (const f of ["aerodb-caret", "mdo-caret", "fly-caret"]) {
    const raw = JSON.parse(readFileSync(join(ROOT, "examples", `${f}.json`), "utf8"));
    const r = validateHeadlessConfig(raw);
    assert.ok(r.params.name, f);
  }
  const fly = JSON.parse(readFileSync(join(ROOT, "examples/fly-caret.json"), "utf8"));
  for (const m of ["sweep", "sensitivity", "eval"]) assert.equal(validateHeadlessConfig(fly, m).mode, m);
  const r = validateHeadlessConfig(small());
  assert.equal(r.mode, "aerodb");
  assert.ok(r.aerodbPoints && r.aerodbPoints > 0 && r.aerodbPoints <= AERODB_POINT_CAP);
});

test("validateHeadlessConfig: reports every problem with its path", () => {
  const bad = {
    bogus: 1,
    mode: "aerodb",
    geometry: { preset: "caret-m8", params: { span: "wide", nope: 3 } },
    aerodb: { machMin: 0.5, method: "cfd", alphaStep: 0, altKm: 120 },
    mdo: { variables: [{ key: "notAKey", min: 0, max: 1 }], objectives: [{ metric: "ld", goal: "up" }], constraints: [{ metric: "mass", op: "==", value: 1 }] },
    sweep: { key: "span", values: [] },
    fly: { law: "lqr", dt: -1 },
  };
  let problems: string[] = [];
  try {
    validateHeadlessConfig(bad);
  } catch (e) {
    assert.ok(e instanceof ConfigError);
    problems = e.problems;
  }
  const want = ["bogus", "geometry.params.span", "geometry.params.nope", "aerodb.method", "aerodb.alphaStep", "aerodb.machMin", "aerodb.altKm", "mdo.variables[0].key", "mdo.objectives[0].goal", "mdo.constraints[0].op", "sweep.values"];
  for (const w of want) assert.ok(problems.some((p) => p.startsWith(w)), `missing ${w} in:\n${problems.join("\n")}`);
  assert.throws(() => validateHeadlessConfig({ aerodb: { extra: true } }), /aerodb.extra: unknown key/);
  assert.throws(() => validateHeadlessConfig(small(), "sweep"), /sweep: required/);
  assert.throws(() => validateHeadlessConfig(small(), "mdo"), /mdo.variables: at least one/);
  const huge = small();
  Object.assign(huge.aerodb, { alphaMin: -10, alphaMax: 20, alphaStep: 1.5, betaMin: -4, betaMax: 4, betaStep: 1, machMax: 12, machStep: 1 });
  assert.throws(() => validateHeadlessConfig(huge), /AERODB_POINT_CAP/);
});

test("CLI aerodb: layout, main's formats, manifest hashes, reproducible byte-identical output", { timeout: 120000 }, async () => {
  const a = await runWith(small(), ["--reproducible"]);
  assert.equal(a.code, EXIT.ok, a.o.err);
  assert.deepEqual(readdirSync(a.out).sort(), ["aerodb.csv", "aerodb.json", "config.resolved.json", "cuspis_aero_db.m", "manifest.json"]);
  assert.match(a.o.err, /\[aerodb\] \d+\/\d+/);
  const csv = readFileSync(join(a.out, "aerodb.csv"), "utf8");
  assert.ok(csv.startsWith("# cuspis aerodynamic database"));
  assert.ok(csv.split("\n").includes(CSV_HEADER), "CSV header is main's aeroDbToCsv header");
  const db = JSON.parse(readFileSync(join(a.out, "aerodb.json"), "utf8"));
  assert.equal(db.meta.format, "cuspis-aerodb-1");
  assert.equal(db.meta.generatedAt, "1970-01-01T00:00:00.000Z");
  const n = db.axes.mach.length * db.axes.alpha_deg.length * db.axes.beta_deg.length * db.axes.delta_deg.length;
  assert.equal(db.points.length, n);
  assert.equal(db.dynamics.length, db.axes.mach.length);
  assert.match(readFileSync(join(a.out, "cuspis_aero_db.m"), "utf8"), /function .*cuspis_aero_db/);
  const man = JSON.parse(readFileSync(join(a.out, "manifest.json"), "utf8"));
  assert.equal(man.mode, "aerodb");
  assert.match(man.configHash, /^c-[0-9a-f]{18}$/);
  assert.ok(typeof man.gitSha === "string" && man.gitSha.length > 0);
  assert.equal(man.counts.points, n);
  assert.deepEqual(man.timings_ms, { run: 0, total: 0 });
  assert.ok(Array.isArray(man.aerodb.assumptions) && man.aerodb.assumptions.length > 0);
  for (const f of man.files) assert.equal(hashHex(readFileSync(join(a.out, f.name), "utf8")), f.hash, f.name);
  assert.ok(!readdirSync(a.out).some((f) => f.includes(".tmp-")), "no temp files left");
  const b = await runWith(small(), ["--reproducible", "--quiet"]);
  assert.equal(b.code, EXIT.ok);
  assert.ok(!/\[aerodb\] \d+\//.test(b.o.err), "--quiet suppresses progress");
  for (const f of readdirSync(a.out)) assert.equal(readFileSync(join(b.out, f), "utf8"), readFileSync(join(a.out, f), "utf8"), `${f} differs between reproducible runs`);
  const c = await runWith(small());
  const manC = JSON.parse(readFileSync(join(c.out, "manifest.json"), "utf8"));
  assert.notEqual(manC.createdISO, "1970-01-01T00:00:00.000Z");
  assert.equal(manC.configHash, man.configHash, "config hash is independent of --reproducible");
});

test("CLI eval / sweep / sensitivity / mdo / fly: outputs over main's public APIs", { timeout: 300000 }, async () => {
  const base = { geometry: { preset: "caret-m8", params: { nx: 12, ny: 8 } } };
  const ev = await runWith({ ...base, mode: "eval" }, ["--quiet"]);
  assert.equal(ev.code, EXIT.ok, ev.o.err);
  const m = JSON.parse(readFileSync(join(ev.out, "metrics.json"), "utf8"));
  for (const k of ["ld", "rangeKm", "qStag", "mass", "staticMargin", "volume", "cd", "cm"]) assert.ok(Number.isFinite(m[k]), k);

  const sw = await runWith({ ...base, mode: "sweep", sweep: { key: "alphaDeg", values: [0, 3] } }, ["--quiet"]);
  assert.equal(sw.code, EXIT.ok, sw.o.err);
  const swl = readFileSync(join(sw.out, "sweep.csv"), "utf8").trim().split("\n");
  assert.equal(swl.length, 3);
  assert.ok(swl[0].startsWith("alphaDeg,ld,"));

  const se = await runWith({ ...base, mode: "sensitivity", sensitivity: { keys: ["span"], metric: "ld" } }, ["--quiet"]);
  assert.equal(se.code, EXIT.ok, se.o.err);
  const sj = JSON.parse(readFileSync(join(se.out, "sensitivity.json"), "utf8"));
  assert.equal(sj.rows.length, 1);
  assert.equal(sj.rows[0].key, "span");

  const mdo = await runWith({ ...base, mode: "mdo", mdo: { variables: [{ key: "span", min: 1.8, max: 2.4 }], objectives: [{ metric: "ld", goal: "max" }], population: 4, generations: 1, seed: 3 } }, ["--quiet"]);
  assert.equal(mdo.code, EXIT.ok, mdo.o.err);
  const pl = readFileSync(join(mdo.out, "pareto.csv"), "utf8").trim().split("\n");
  assert.ok(pl[0].startsWith("rank,span,ld,"));
  assert.ok(pl.length >= 2, `pareto.csv: ${JSON.stringify(pl)} ${mdo.o.err}`);
  assert.ok(existsSync(join(mdo.out, "history.csv")) && existsSync(join(mdo.out, "mdo.json")), "mdo files");

  const fly = await runWith({ ...base, mode: "fly", fly: { durationS: 3, dt: 0.05 } }, ["--quiet"]);
  assert.equal(fly.code, EXIT.ok, fly.o.err);
  const tl = readFileSync(join(fly.out, "trajectory.csv"), "utf8").trim().split("\n").filter((l) => !l.startsWith("#"));
  assert.ok(tl[0].startsWith("t_s,alpha_deg,alpha_cmd_deg,"), tl[0]);
  assert.ok(tl.length > 10, `trajectory rows ${tl.length}`);
});

test("CLI: aborted signal → exit 130, nothing written; bad config → exit 2", async () => {
  const ac = new AbortController();
  ac.abort(new DOMException("test", "AbortError"));
  const r = await runWith(small(), ["--quiet"], ac.signal);
  assert.equal(r.code, EXIT.interrupted, r.o.err);
  assert.ok(!existsSync(r.out), "no output directory on abort");
  const b = await runWith({ aerodb: { machMin: "six" } });
  assert.equal(b.code, EXIT.usage);
  assert.match(b.o.err, /aerodb\.machMin: expected a finite number/);
  assert.ok(!existsSync(b.out));
  // main's cuspis-eval hardening carried over: regular file under 2 MB, prototype keys dropped
  const d = tmp();
  const c = capture();
  assert.equal(await runCli(["--config", d, "--out", join(d, "o")], c.io), EXIT.usage);
  assert.match(c.o.err, /under 2 MB/);
  const polluted = await runWith(JSON.parse('{"mode":"eval","__proto__":{"polluted":1},"geometry":{"preset":"caret-m8","params":{"nx":12,"ny":10}}}'), ["--quiet", "--validate-only"]);
  assert.equal(polluted.code, EXIT.ok, polluted.o.err);
  assert.equal(({} as Record<string, unknown>).polluted, undefined);
});

function spawnCli(args: string[]) {
  return spawn(process.execPath, ["--import", "./scripts/ts-register.mjs", "scripts/cuspis-eval.ts", ...args], { cwd: ROOT, stdio: ["ignore", "pipe", "pipe"] });
}
const done = (c: ReturnType<typeof spawnCli>) => new Promise<number | null>((res) => c.on("close", (code) => res(code)));

test("CLI process: SIGINT aborts the run, exits 130 and writes no manifest", { timeout: 120000 }, async () => {
  const dir = tmp();
  const cfg = small();
  Object.assign(cfg.geometry.params, { nx: 40, ny: 28 });
  Object.assign(cfg.aerodb, { method: "cbaero", machMin: 4, machMax: 12, machStep: 2, alphaMin: -4, alphaMax: 16, alphaStep: 2, betaMin: -4, betaMax: 4, betaStep: 4 });
  writeFileSync(join(dir, "cfg.json"), JSON.stringify(cfg));
  const out = join(dir, "out");
  const child = spawnCli(["--config", join(dir, "cfg.json"), "--out", out]);
  let err = "";
  let sent = false;
  child.stderr.on("data", (d) => {
    err += d;
    if (!sent && /\[aerodb\] \d+\/\d+/.test(err)) {
      sent = true;
      child.kill("SIGINT");
    }
  });
  const code = await done(child);
  assert.ok(sent, `no progress line seen:\n${err}`);
  assert.equal(code, 130, err);
  assert.ok(!existsSync(join(out, "manifest.json")));
});

test("CLI process: bad config exits 2; legacy positional design.json still prints metrics", { timeout: 120000 }, async () => {
  const dir = tmp();
  writeFileSync(join(dir, "bad.json"), JSON.stringify({ mode: "nope" }));
  const bad = spawnCli(["--config", join(dir, "bad.json"), "--out", join(dir, "o")]);
  let err = "";
  bad.stderr.on("data", (d) => (err += d));
  assert.equal(await done(bad), 2);
  assert.match(err, /mode: expected one of/);

  writeFileSync(join(dir, "design.json"), JSON.stringify({ params: { preset: "caret-m8", nx: 12, ny: 8 } }));
  const leg = spawnCli([join(dir, "design.json")]);
  let out = "";
  leg.stdout.on("data", (d) => (out += d));
  assert.equal(await done(leg), 0);
  const m = JSON.parse(out);
  assert.ok(Number.isFinite(m.ld) && Number.isFinite(m.mass));
});
