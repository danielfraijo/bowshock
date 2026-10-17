/**
 * Tiny benchmark harness: warm-up, then time `run(i)` until either `maxRuns` or `budgetMs`
 * is reached (never fewer than `minRuns`). Reports median / p95 / min / mean in ms.
 * `run` gets the iteration index so benchmarks can vary their inputs and defeat memo caches
 * (a slider drag never repeats exactly the same value).
 */
import os from "node:os";
import { execSync } from "node:child_process";

export interface BenchResult {
  name: string;
  group: string;
  n: number;
  median: number;
  p95: number;
  min: number;
  mean: number;
  max: number;
  note?: string;
}

export interface BenchSpec {
  name: string;
  group: string;
  /** Called once (untimed) – returns the function to time. */
  setup: () => (i: number) => unknown | Promise<unknown>;
  warmup?: number;
  minRuns?: number;
  maxRuns?: number;
  budgetMs?: number;
  note?: string;
}

export function percentile(sorted: number[], p: number): number {
  if (!sorted.length) return NaN;
  const idx = Math.min(sorted.length - 1, Math.max(0, Math.ceil(p * sorted.length) - 1));
  return sorted[idx];
}

let sink: unknown;
export async function runBench(spec: BenchSpec, scale = 1): Promise<BenchResult> {
  const run = spec.setup();
  const warm = spec.warmup ?? 3;
  const minRuns = spec.minRuns ?? 7;
  const maxRuns = Math.max(minRuns, Math.round((spec.maxRuns ?? 60) * scale));
  const budget = (spec.budgetMs ?? 1500) * scale;
  let k = 0;
  for (let i = 0; i < warm; i++) sink = await run(k++);
  const times: number[] = [];
  const t0 = performance.now();
  while (times.length < maxRuns) {
    const a = performance.now();
    sink = await run(k++);
    times.push(performance.now() - a);
    if (times.length >= minRuns && performance.now() - t0 > budget) break;
  }
  void sink;
  const sorted = times.slice().sort((a, b) => a - b);
  const mean = times.reduce((s, v) => s + v, 0) / times.length;
  return {
    name: spec.name,
    group: spec.group,
    n: times.length,
    median: percentile(sorted, 0.5),
    p95: percentile(sorted, 0.95),
    min: sorted[0],
    mean,
    max: sorted[sorted.length - 1],
    note: spec.note,
  };
}

export function machineInfo() {
  const cpus = os.cpus();
  let commit = "unknown";
  let branch = "unknown";
  try {
    commit = execSync("git rev-parse --short HEAD", { stdio: ["ignore", "pipe", "ignore"] }).toString().trim();
    branch = execSync("git rev-parse --abbrev-ref HEAD", { stdio: ["ignore", "pipe", "ignore"] }).toString().trim();
  } catch {
    /* not a git checkout */
  }
  return {
    node: process.version,
    v8: process.versions.v8,
    platform: `${os.platform()} ${os.release()} ${os.arch()}`,
    cpu: cpus[0]?.model ?? "unknown",
    cores: cpus.length,
    memGB: Math.round((os.totalmem() / 1024 ** 3) * 10) / 10,
    commit,
    branch,
    date: new Date().toISOString(),
  };
}
