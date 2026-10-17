/**
 * ComputeClient: async API in front of the compute worker(s).
 *
 *   const c = getComputeClient();
 *   const r = await c.study(params, "preview");      // coarse Euler, fast lane  (slider drag)
 *   const r = await c.study(params, "full");         // full study, slow lane    (slider released)
 *   const files = await c.exportFiles("kit", params, sources);
 *
 * Three lanes (three workers): "fast" for preview/quick studies (drag), "slow" for full studies and exports,
 * "refine" for the converged background refinement (preemptible: a newer request kills it), so a long
 * release-time study never blocks the next drag frame and a refinement never delays a new interaction. Each lane runs ONE job at a time and
 * keeps at most ONE queued job per request type: a newer study request replaces (and rejects with
 * SupersededError) an older one that has not started -> latest-wins while a slider drags. A job that is
 * already running is not interrupted (a study is <= ~250 ms); its result is still delivered and callers
 * drop it if they have moved on (see useLiveVehicle).
 *
 * Without Workers (Node, tests, SSR, old browsers) the same lanes run in-process on a macrotask
 * (`LoopbackTransport`), with identical protocol and coalescing, so callers never branch.
 */
import type { DesignParams } from "../waverider/types";
import { computeDataPackAsync, datapackCached, primeDataPack, type DataPack, type DataPackOptions } from "../waverider/datapack";
import { runCases } from "../waverider/datcom/model";
import type { FlightSpec, RunResult, VehicleConfig } from "../waverider/datcom/types";
import type { StudyMode } from "./pipeline";
import type { ExportSources, StudyReply, WorkerReply, WorkerRequest } from "./protocol";
import { WorkerCore } from "./worker-core";

export class SupersededError extends Error {
  constructor() {
    super("superseded by a newer request");
    this.name = "SupersededError";
  }
}

export interface Transport {
  post(req: WorkerRequest): void;
  onreply: (r: WorkerReply) => void;
  terminate(): void;
}

/** In-process transport: same protocol, runs on a macrotask so callers can still coalesce. */
export class LoopbackTransport implements Transport {
  onreply: (r: WorkerReply) => void = () => {};
  private core: WorkerCore;
  private alive = true;
  constructor(private clone = false) {
    this.core = new WorkerCore((r) => {
      if (!this.alive) return;
      // emulate structured clone so loopback behaves like a real worker (no shared mutable state)
      this.onreply(this.clone ? structuredClone(r) : r);
    });
  }
  post(req: WorkerRequest) {
    const msg = this.clone ? structuredClone(req) : req;
    setTimeout(() => {
      if (this.alive) void this.core.handle(msg);
    }, 0);
  }
  terminate() {
    this.alive = false;
  }
}

class WebWorkerTransport implements Transport {
  onreply: (r: WorkerReply) => void = () => {};
  private w: Worker;
  constructor(wasmUrl?: string) {
    this.w = new Worker(new URL("./compute.worker.ts", import.meta.url), { type: "module" });
    this.w.onmessage = (e: MessageEvent<WorkerReply>) => this.onreply(e.data);
    // Default: the worker instantiates the wasm embedded in its bundle (54 KB: ~0.6 ms to compile+instantiate,
    // no extra request). Pass `wasmUrl` to stream a hosted copy instead (see NOTES.md for why that is opt-in).
    if (wasmUrl) this.w.postMessage({ type: "init", wasmUrl } satisfies WorkerRequest);
  }
  post(req: WorkerRequest) {
    this.w.postMessage(req);
  }
  terminate() {
    this.w.terminate();
  }
}

interface Job {
  req: WorkerRequest & { id: number };
  coalesce: boolean;
  resolve: (r: WorkerReply) => void;
  reject: (e: unknown) => void;
}

class Lane {
  private running: Job | null = null;
  private queue: Job[] = [];
  private t: Transport | null = null;
  /**
   * preempt: a new coalescing request terminates the worker that is still running an older one (true
   * cancellation of a long job; the lane's caches are lost, so only use it for the background refine lane).
   */
  constructor(
    readonly name: string,
    private make: () => Transport,
    private preempt = false,
  ) {}
  get busy() {
    return this.running !== null || this.queue.length > 0;
  }
  private transport(): Transport {
    if (!this.t) {
      this.t = this.make();
      this.t.onreply = (r) => this.onReply(r);
    }
    return this.t;
  }
  submit(job: Job) {
    if (this.preempt && job.coalesce && this.running?.coalesce) {
      const old = this.running;
      this.running = null;
      this.t?.terminate();
      this.t = null;
      old.reject(new SupersededError());
    }
    if (job.coalesce) {
      for (const q of this.queue.filter((q) => q.coalesce && q.req.type === job.req.type)) {
        this.queue.splice(this.queue.indexOf(q), 1);
        q.reject(new SupersededError());
      }
    }
    this.queue.push(job);
    this.pump();
  }
  private pump() {
    if (this.running || !this.queue.length) return;
    this.running = this.queue.shift()!;
    this.transport().post(this.running.req);
  }
  private onReply(r: WorkerReply) {
    if (r.type === "ready") return;
    const job = this.running;
    if (!job || ("id" in r && r.id !== job.req.id)) return;
    this.running = null;
    if (r.type === "error") job.reject(new Error(r.message));
    else job.resolve(r);
    this.pump();
  }
  /** Kill whatever is running/queued here (the worker is respawned lazily on the next submit). */
  cancel() {
    this.t?.terminate();
    this.t = null;
    for (const j of [this.running, ...this.queue]) j?.reject(new SupersededError());
    this.running = null;
    this.queue = [];
  }

  dispose() {
    this.t?.terminate();
    this.t = null;
    for (const j of [this.running, ...this.queue]) j?.reject(new Error("compute client disposed"));
    this.running = null;
    this.queue = [];
  }
}

export interface ComputeClientOptions {
  /** "worker" | "inline"; default: worker when available. */
  runner?: "worker" | "inline";
  wasmUrl?: string;
  /** structuredClone messages in inline mode (tests) */
  clone?: boolean;
}

export function workersAvailable(): boolean {
  return typeof Worker !== "undefined" && typeof window !== "undefined";
}

export class ComputeClient {
  readonly runner: "worker" | "inline";
  private fast: Lane;
  private slow: Lane;
  private refine: Lane;
  private bulk: Lane;
  private seq = 1;

  constructor(opts: ComputeClientOptions = {}) {
    this.runner = opts.runner ?? (workersAvailable() ? "worker" : "inline");
    const make = (): Transport => (this.runner === "worker" ? new WebWorkerTransport(opts.wasmUrl) : new LoopbackTransport(opts.clone));
    this.fast = new Lane("fast", make);
    this.slow = new Lane("slow", make);
    this.refine = new Lane("refine", make, true);
    // datapack tables + DATCOM runs: own worker so they never queue behind (or in front of) the live study lanes;
    // a newer datapack request preempts a running one (terminate + respawn), DATCOM runs are never dropped
    this.bulk = new Lane("bulk", make, true);
  }

  get busy() {
    return this.fast.busy || this.slow.busy || this.refine.busy || this.bulk.busy;
  }

  /** Run a study. Rejects with SupersededError if a newer request on the same lane replaced it before it started. */
  study(params: DesignParams, mode: StudyMode, have?: string): Promise<StudyReply> {
    const lane = mode === "refine" ? this.refine : mode === "full" ? this.slow : this.fast;
    return new Promise<StudyReply>((resolve, reject) => {
      lane.submit({
        req: { type: "study", id: this.seq++, params, mode, have },
        coalesce: true,
        resolve: resolve as (r: WorkerReply) => void,
        reject,
      });
    });
  }

  exportFiles(kind: string, params: DesignParams, sources: ExportSources): Promise<{ name: string; blob: Blob }[]> {
    return new Promise((resolve, reject) => {
      this.slow.submit({
        req: { type: "export", id: this.seq++, kind, params, sources },
        coalesce: false,
        resolve: (r) => resolve((r as Extract<WorkerReply, { type: "export" }>).files),
        reject,
      });
    });
  }

  /** Data tab tables in the worker; a newer request replaces an older queued/running one (rejects SupersededError). */
  datapack(params: DesignParams, opts?: DataPackOptions): Promise<{ pack: DataPack; ms: number }> {
    return new Promise((resolve, reject) => {
      this.bulk.submit({
        req: { type: "datapack", id: this.seq++, params, opts },
        coalesce: true,
        resolve: (r) => resolve(r as never),
        reject,
      });
    });
  }

  /** DATCOM case sweep in the worker. */
  datcom(config: VehicleConfig, flight: FlightSpec): Promise<{ result: RunResult; ms: number }> {
    return new Promise((resolve, reject) => {
      this.bulk.submit({
        req: { type: "datcom", id: this.seq++, config, flight },
        coalesce: false,
        resolve: (r) => resolve(r as never),
        reject,
      });
    });
  }

  /** Abort a running/queued background refinement (called on every new interaction). */
  cancelRefine() {
    this.refine.cancel();
  }

  /** Pipeline cache stats of the fast-lane worker (plus how its wasm was loaded). */
  stats(): Promise<unknown> {
    return new Promise((resolve, reject) => {
      this.fast.submit({
        req: { type: "stats", id: this.seq++ },
        coalesce: false,
        resolve: (r) => resolve((r as Extract<WorkerReply, { type: "stats" }>).stats),
        reject,
      });
    });
  }

  dispose() {
    this.fast.dispose();
    this.slow.dispose();
    this.refine.dispose();
    this.bulk.dispose();
  }
}

let shared: ComputeClient | null = null;
/** Process-wide client. In the browser it spawns its workers lazily on first use. */
export function getComputeClient(): ComputeClient {
  if (!shared) {
    shared = new ComputeClient();
    // introspection hook for smoke tests / the console: await __bowshockCompute.stats()
    if (typeof window !== "undefined") (window as unknown as Record<string, unknown>).__bowshockCompute = shared;
  }
  return shared;
}

const OFFTHREAD_KINDS = new Set(["stl", "stl-ascii", "stl-regions", "step", "step-nurbs", "step-tess", "step-facet", "iges", "plot3d", "obj", "kit"]);

/**
 * Build a CAD export in the worker (export-density rebuild + quick study + file text + zip).
 * Returns null when it should be done inline instead (no Workers, or a kind the worker does not handle).
 */
export async function exportOffThread(kind: string, params: DesignParams, sources: ExportSources): Promise<{ name: string; blob: Blob }[] | null> {
  if (!OFFTHREAD_KINDS.has(kind) || !workersAvailable()) return null;
  try {
    return await getComputeClient().exportFiles(kind, params, sources);
  } catch (err) {
    console.warn("worker export failed, exporting on the main thread", err);
    return null;
  }
}

/**
 * Data tab tables: worker when available (result is also stored in the main-thread datapack cache so
 * `datapackCached` keeps working), else the time-sliced main-thread computation.
 */
export async function computeDataPackOffThread(
  params: DesignParams,
  opts: DataPackOptions | undefined,
  hooks: { signal?: AbortSignal; onProgress?: (p: { frac: number; label: string }) => void } = {},
): Promise<DataPack> {
  if (!workersAvailable()) return computeDataPackAsync(params, opts, hooks);
  const hit = datapackCached(params, opts);
  if (hit) return hit;
  hooks.onProgress?.({ frac: 0, label: "computing in worker" });
  try {
    const { pack } = await getComputeClient().datapack(params, opts);
    if (hooks.signal?.aborted) throw new Error("aborted");
    primeDataPack(pack);
    return pack;
  } catch (err) {
    if (err instanceof SupersededError) throw new Error("aborted");
    if (hooks.signal?.aborted) throw err;
    console.warn("worker datapack failed, computing on the main thread", err);
    return computeDataPackAsync(params, opts, hooks);
  }
}

/** DATCOM sweep off the main thread (falls back to inline). */
export async function runCasesOffThread(config: VehicleConfig, flight: FlightSpec): Promise<RunResult> {
  if (!workersAvailable()) return runCases(config, flight);
  try {
    return (await getComputeClient().datcom(config, flight)).result;
  } catch (err) {
    console.warn("worker DATCOM run failed, running on the main thread", err);
    return runCases(config, flight);
  }
}
