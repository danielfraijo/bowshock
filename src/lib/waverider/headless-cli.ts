/**
 * [aerodb-headless] Batch CLI over main's public APIs (entry: scripts/cuspis-eval.ts, `npm run aerodb`).
 *
 *   npm run aerodb -- --config cfg.json --out dir [--mode aerodb|eval|mdo|sweep|sensitivity|fly] [--reproducible] [--quiet]
 *   node --import ./scripts/ts-register.mjs scripts/cuspis-eval.ts design.json      (legacy: metrics JSON on stdout)
 *
 * Progress → stderr; SIGINT/SIGTERM → AbortSignal (wired in scripts/cuspis-eval.ts); every file is written to
 * `<name>.tmp-<pid>` then renamed; manifest.json (provenance, hashes, timings) is written last.
 * Exit codes: 0 ok, 1 run failure, 2 usage/config error, 130 interrupted.
 */
import { execFileSync } from "node:child_process";
import { mkdirSync, readFileSync, renameSync, statSync, writeFileSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { aeroDbToCsv, aeroDbToJson, aeroDbToSimulink, generateAeroDatabase } from "./aerodb";
import { buildVehicle } from "./generate";
import { flyClosedLoop, samplesToCsv } from "./guidance";
import { MDO_METRICS, evaluateDesign, parameterSweep, runOptimizer, sensitivity, type MdoMetrics } from "./mdo";
import { sanitizeParams } from "./presets";
import { ConfigError, HEADLESS_MODES, validateHeadlessConfig, type ResolvedConfig } from "./headless-config";

export const EXIT = { ok: 0, failure: 1, usage: 2, interrupted: 130 } as const;
export const LAYOUT_VERSION = "cuspis-headless-1";

export const USAGE = `Usage: npm run aerodb -- --config <cfg.json> --out <dir> [options]

Modes (config "mode" or --mode; default aerodb):
  aerodb       generateAeroDatabase → aerodb.csv, aerodb.json, cuspis_aero_db.m
  eval         evaluateDesign → metrics.json
  mdo          runOptimizer (NSGA-II) → mdo.json, pareto.csv, history.csv
  sweep        parameterSweep → sweep.csv, sweep.json
  sensitivity  sensitivity → sensitivity.csv, sensitivity.json
  fly          flyClosedLoop → trajectory.csv, fly.json
Every mode also writes config.resolved.json and manifest.json.

Options:
  --config <file>   JSON config; see examples/aerodb-caret.json and docs/aerodb.md
  --out <dir>       output directory (created; files replaced atomically)
  --mode <m>        override the config's mode
  --reproducible    fixed timestamps (SOURCE_DATE_EPOCH or 1970-01-01) and zero timings → byte-identical output
  --quiet           no progress on stderr
  --validate-only   check the config, print a summary, write nothing
  -h, --help        this text
Legacy: scripts/cuspis-eval.ts <design.json> prints evaluateDesign metrics as JSON.
Exit codes: 0 ok, 1 run failed, 2 usage/config error, 130 interrupted.`;

export interface CliArgs {
  config?: string;
  out?: string;
  mode?: string;
  reproducible: boolean;
  quiet: boolean;
  validateOnly: boolean;
  help: boolean;
  legacy?: string;
}

export class UsageError extends Error {}

/** Max size of a design/config JSON (same limit as main's cuspis-eval hardening). */
export const MAX_INPUT_BYTES = 2_000_000;

/** Read a JSON input file: regular file under MAX_INPUT_BYTES, prototype-polluting keys dropped. */
export function readJsonInput(path: string): unknown {
  const st = statSync(path);
  if (!st.isFile() || st.size > MAX_INPUT_BYTES) throw new Error("input must be a JSON file under 2 MB");
  return JSON.parse(readFileSync(path, "utf8"), (key, value) => (key === "__proto__" || key === "constructor" || key === "prototype" ? undefined : value));
}

export function parseArgs(argv: string[]): CliArgs {
  const a: CliArgs = { reproducible: false, quiet: false, validateOnly: false, help: false };
  if (argv.length === 1 && !argv[0].startsWith("-")) return { ...a, legacy: argv[0] };
  for (let i = 0; i < argv.length; i++) {
    const k = argv[i];
    const val = () => {
      const v = argv[++i];
      if (v === undefined || v.startsWith("--")) throw new UsageError(`${k} needs a value`);
      return v;
    };
    if (k === "--config") a.config = val();
    else if (k === "--out") a.out = val();
    else if (k === "--mode") {
      a.mode = val();
      if (!(HEADLESS_MODES as readonly string[]).includes(a.mode)) throw new UsageError(`--mode must be one of ${HEADLESS_MODES.join(", ")}`);
    } else if (k === "--reproducible") a.reproducible = true;
    else if (k === "--quiet") a.quiet = true;
    else if (k === "--validate-only") a.validateOnly = true;
    else if (k === "-h" || k === "--help") a.help = true;
    else throw new UsageError(`unknown argument ${k}`);
  }
  if (!a.help && !a.config) throw new UsageError("--config is required");
  if (!a.help && !a.validateOnly && !a.out) throw new UsageError("--out is required");
  return a;
}

const REPO_ROOT = resolve(dirname(fileURLToPath(import.meta.url)), "../../..");

export function gitInfo(cwd = REPO_ROOT): { sha: string; dirty: boolean | null } {
  try {
    const sha = execFileSync("git", ["rev-parse", "HEAD"], { cwd, stdio: ["ignore", "pipe", "ignore"] }).toString().trim();
    const st = execFileSync("git", ["status", "--porcelain", "--untracked-files=no"], { cwd, stdio: ["ignore", "pipe", "ignore"] }).toString().trim();
    return { sha, dirty: st.length > 0 };
  } catch {
    return { sha: "unknown", dirty: null };
  }
}

function appVersion(): string {
  try {
    return (JSON.parse(readFileSync(join(REPO_ROOT, "package.json"), "utf8")) as { version?: string }).version ?? "unknown";
  } catch {
    return "unknown";
  }
}

/** cyrb53 (same algorithm as datapack.ts) → 18 hex chars. Cache/provenance key, not cryptographic. */
export function hashHex(str: string): string {
  const h = (seed: number) => {
    let h1 = 0xdeadbeef ^ seed;
    let h2 = 0x41c6ce57 ^ seed;
    for (let i = 0; i < str.length; i++) {
      const ch = str.charCodeAt(i);
      h1 = Math.imul(h1 ^ ch, 2654435761);
      h2 = Math.imul(h2 ^ ch, 1597334677);
    }
    h1 = Math.imul(h1 ^ (h1 >>> 16), 2246822507) ^ Math.imul(h2 ^ (h2 >>> 13), 3266489909);
    h2 = Math.imul(h2 ^ (h2 >>> 16), 2246822507) ^ Math.imul(h1 ^ (h1 >>> 13), 3266489909);
    return 4294967296 * (2097151 & h2) + (h1 >>> 0);
  };
  return h(0).toString(16).padStart(14, "0") + (h(7919) & 0xffff).toString(16).padStart(4, "0");
}

/** JSON with recursively sorted keys (stable hashes and byte-stable files). */
export function stableJson(v: unknown, indent = 1): string {
  const norm = (x: unknown): unknown => {
    if (typeof x === "number") return Number.isFinite(x) ? x : String(x);
    if (Array.isArray(x)) return x.map(norm);
    if (x && typeof x === "object") {
      const o = x as Record<string, unknown>;
      const out: Record<string, unknown> = {};
      for (const k of Object.keys(o).sort()) if (o[k] !== undefined) out[k] = norm(o[k]);
      return out;
    }
    return x;
  };
  return JSON.stringify(norm(v), null, indent) + "\n";
}

const cell = (v: unknown) => (typeof v === "number" ? (Number.isFinite(v) ? String(Number(v.toPrecision(12))) : "") : v === undefined || v === null ? "" : /[",\n]/.test(String(v)) ? `"${String(v).replace(/"/g, '""')}"` : String(v));
const METRIC_COLS = [...MDO_METRICS, "cl", "method"] as const;
const metricCells = (m: MdoMetrics | undefined) => METRIC_COLS.map((k) => cell(m ? (m as unknown as Record<string, unknown>)[k] : undefined));

export interface CliIo {
  stderr: (s: string) => void;
  stdout: (s: string) => void;
}

function reporter(io: CliIo, quiet: boolean) {
  let last = 0;
  return (done: number, total: number, label: string) => {
    if (quiet) return;
    const now = Date.now();
    if (done < total && now - last < 500) return;
    last = now;
    const pct = total > 0 ? ((100 * done) / total).toFixed(0) : "100";
    io.stderr(`[aerodb] ${done}/${total} (${pct}%) ${label}\n`);
  };
}

/** Run one mode; returns [file, content] pairs (manifest excluded) plus counts for the manifest. */
async function runMode(cfg: ResolvedConfig, signal: AbortSignal | undefined, progress: ReturnType<typeof reporter>, createdISO: string | null) {
  const files: [string, string][] = [];
  const counts: Record<string, number> = {};
  const extra: Record<string, unknown> = {};
  const p = cfg.params;
  if (cfg.mode === "aerodb") {
    const built = buildVehicle(p);
    const db = await generateAeroDatabase(built, cfg.aerodb!, { signal, onProgress: (x) => progress(x.done, x.total, x.label) });
    if (createdISO) db.meta.generatedAt = createdISO;
    files.push(["aerodb.csv", aeroDbToCsv(db)], ["aerodb.json", aeroDbToJson(db)], ["cuspis_aero_db.m", aeroDbToSimulink(db)]);
    counts.points = db.points.length;
    counts.dynamics = db.dynamics.length;
    extra.aerodb = { format: db.meta.format, method: db.meta.method, axes: db.axes, sRef_m2: db.meta.sRef_m2, lRef_m: db.meta.lRef_m, bRef_m: db.meta.bRef_m, cg_m: db.meta.cg_m, units: db.meta.units, assumptions: db.meta.assumptions, summary: db.summary };
  } else if (cfg.mode === "eval") {
    const ev = evaluateDesign(p);
    if (ev.error) throw new Error(`evaluateDesign: ${ev.error}`);
    files.push(["metrics.json", stableJson(ev.metrics)]);
    counts.evaluations = 1;
  } else if (cfg.mode === "mdo") {
    const res = await runOptimizer(p, cfg.mdo!, { signal, onProgress: (x) => progress(x.generation, x.generations, `${x.label} (${x.evaluations} evals)`) });
    const keys = cfg.mdo!.variables.map((v) => String(v.key));
    const rows = [["rank", ...keys, ...METRIC_COLS, "violation", "feasible", "error"].join(",")];
    res.pareto.forEach((e, i) => rows.push([String(i), ...keys.map((k) => cell((e.params as unknown as Record<string, unknown>)[k])), ...metricCells(e.metrics), cell(e.violation), e.feasible ? "true" : "false", cell(e.error)].join(",")));
    files.push(["mdo.json", stableJson(res)], ["pareto.csv", rows.join("\n") + "\n"], ["history.csv", ["generation,bestLd,feasible", ...res.history.map((h) => [h.generation, h.bestLd, h.feasible].map(cell).join(","))].join("\n") + "\n"]);
    counts.evaluations = res.evaluations;
    counts.pareto = res.pareto.length;
    extra.notes = res.notes;
  } else if (cfg.mode === "sweep") {
    const s = cfg.sweep!;
    const rows = await parameterSweep(p, s.key, s.values, { signal, onProgress: (d, t) => progress(d, t, `${String(s.key)} sweep`) });
    files.push(["sweep.csv", [[String(s.key), ...METRIC_COLS, "error"].join(","), ...rows.map((r) => [cell(r.value), ...metricCells(r.metrics), cell(r.error)].join(","))].join("\n") + "\n"], ["sweep.json", stableJson({ key: s.key, rows })]);
    counts.evaluations = rows.length;
  } else if (cfg.mode === "sensitivity") {
    const s = cfg.sensitivity!;
    progress(0, s.keys.length, `sensitivity of ${s.metric}`);
    const res = await sensitivity(p, s.keys, s.metric, s.frac, { signal });
    files.push(["sensitivity.csv", ["key,base,plus,minus,dMetric,elasticity", ...res.rows.map((r) => [r.key, r.base, r.plus, r.minus, r.dMetric, r.elasticity].map(cell).join(","))].join("\n") + "\n"], ["sensitivity.json", stableJson({ metric: s.metric, frac: s.frac, ...res })]);
    counts.rows = res.rows.length;
  } else if (cfg.mode === "fly") {
    const built = buildVehicle(p);
    const res = await flyClosedLoop(built, cfg.fly!, { signal, onProgress: (x) => progress(x.done, x.total, x.label), ...cfg.flyMass });
    files.push(["trajectory.csv", samplesToCsv(res)], ["fly.json", stableJson(res)]);
    counts.samples = res.samples.length;
    extra.fly = { maxQ_Pa: res.maxQ, maxN_g: res.maxN, rangeKm: res.rangeKm, finalH_km: res.finalH, timeS: res.timeS, notes: res.notes };
  }
  return { files, counts, extra };
}

/** Run the CLI; returns the exit code. `signal` is wired to SIGINT by scripts/cuspis-eval.ts. */
export async function runCli(argv: string[], io: CliIo, signal?: AbortSignal): Promise<number> {
  const tStart = Date.now();
  let args: CliArgs;
  try {
    args = parseArgs(argv);
  } catch (e) {
    io.stderr(`aerodb: ${(e as Error).message}\n\n${USAGE}\n`);
    return EXIT.usage;
  }
  if (args.help) {
    io.stdout(USAGE + "\n");
    return EXIT.ok;
  }
  const base = process.env.INIT_CWD || process.cwd(); // `npm run` moves cwd to the repo root
  if (args.legacy) {
    try {
      const raw = readJsonInput(resolve(base, args.legacy)) as { params?: unknown };
      const ev = evaluateDesign(sanitizeParams(raw.params ?? raw));
      if (ev.error) {
        io.stderr(ev.error + "\n");
        return EXIT.failure;
      }
      io.stdout(JSON.stringify(ev.metrics, null, 2) + "\n");
      return EXIT.ok;
    } catch (e) {
      io.stderr(`cuspis-eval: ${(e as Error).message}\n`);
      return EXIT.usage;
    }
  }
  const cfgPath = resolve(base, args.config as string);
  let raw: unknown;
  try {
    raw = readJsonInput(cfgPath);
  } catch (e) {
    io.stderr(`aerodb: cannot read config ${cfgPath}: ${(e as Error).message}\n`);
    return EXIT.usage;
  }
  let cfg: ResolvedConfig;
  try {
    cfg = validateHeadlessConfig(raw, args.mode);
  } catch (e) {
    io.stderr(`aerodb: ${(e as Error).message}\n`);
    return e instanceof ConfigError ? EXIT.usage : EXIT.failure;
  }
  for (const w of cfg.warnings) io.stderr(`aerodb: warning: ${w}\n`);
  const configHash = `c-${hashHex(stableJson({ mode: cfg.mode, params: cfg.params, aerodb: cfg.aerodb, mdo: cfg.mdo, sweep: cfg.sweep, sensitivity: cfg.sensitivity, fly: cfg.fly, flyMass: cfg.flyMass }, 0))}`;
  if (args.validateOnly) {
    io.stdout(`config OK: mode ${cfg.mode}${cfg.aerodbPoints ? `, ${cfg.aerodbPoints} aerodb points` : ""}, config hash ${configHash}\n`);
    return EXIT.ok;
  }
  const epoch = process.env.SOURCE_DATE_EPOCH;
  const fixedISO = args.reproducible ? new Date(epoch && /^\d+$/.test(epoch) ? Number(epoch) * 1000 : 0).toISOString() : null;
  const progress = reporter(io, args.quiet);
  if (!args.quiet) io.stderr(`[aerodb] mode ${cfg.mode}; config ${configHash}\n`);
  const tRun = Date.now();
  let res;
  try {
    res = await runMode(cfg, signal, progress, fixedISO);
  } catch (e) {
    if (signal?.aborted || (e as Error).name === "AbortError") {
      io.stderr("aerodb: interrupted, nothing written\n");
      return EXIT.interrupted;
    }
    io.stderr(`aerodb: ${cfg.mode} failed: ${(e as Error).message}\n`);
    return EXIT.failure;
  }
  const run_ms = Date.now() - tRun;
  const git = gitInfo();
  const resolvedConfig = { mode: cfg.mode, name: cfg.name, params: cfg.params, aerodb: cfg.aerodb, mdo: cfg.mdo, sweep: cfg.sweep, sensitivity: cfg.sensitivity, fly: cfg.fly, flyMass: cfg.flyMass };
  res.files.push(["config.resolved.json", stableJson(resolvedConfig)]);
  const manifest = {
    tool: "bowshock headless CLI (scripts/cuspis-eval.ts)",
    layoutVersion: LAYOUT_VERSION,
    mode: cfg.mode,
    name: cfg.name,
    appVersion: appVersion(),
    gitSha: git.sha,
    gitDirty: git.dirty,
    configHash,
    geometryHash: `g-${hashHex(stableJson({ ...cfg.params, name: undefined }, 0))}`,
    createdISO: fixedISO ?? new Date().toISOString(),
    counts: res.counts,
    configWarnings: cfg.warnings,
    ...res.extra,
    runtime: args.reproducible ? undefined : { node: process.version, platform: process.platform },
    timings_ms: args.reproducible ? { run: 0, total: 0 } : { run: run_ms, total: Date.now() - tStart },
    files: res.files.map(([name, content]) => ({ name, bytes: new TextEncoder().encode(content).length, hash: hashHex(content) })),
  };
  res.files.push(["manifest.json", stableJson(manifest)]);
  try {
    const out = resolve(base, args.out as string);
    mkdirSync(out, { recursive: true });
    for (const [name, content] of res.files) {
      const target = join(out, name);
      const tmp = `${target}.tmp-${process.pid}`;
      writeFileSync(tmp, content);
      renameSync(tmp, target);
    }
    io.stderr(`[aerodb] wrote ${res.files.length} files to ${out}\n`);
  } catch (e) {
    io.stderr(`aerodb: writing output failed: ${(e as Error).message}\n`);
    return EXIT.failure;
  }
  return EXIT.ok;
}
