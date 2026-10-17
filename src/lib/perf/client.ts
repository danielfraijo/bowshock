/**
 * ComputeClient: async API in front of the compute worker(s).
 *
 *   const c = getComputeClient();
 *   const r = await c.study(params, "preview");      // coarse Euler, fast lane  (slider drag)
 *   const r = await c.study(params, "full");         // full study, slow lane    (slider released)
 *   const files = await c.exportFiles("kit", params, sources);
 *
 * Two lanes (two workers): "fast" for preview/quick studies and "slow" for full studies and exports,
 * so a long release-time study never blocks the next drag frame. Each lane runs ONE job at a time and
 * keeps at most ONE queued job per request type: a newer study request replaces (and rejects with
 * SupersededError) an older one that has not started -> latest-wins while a slider drags. A job that is
 * already running is not interrupted (a study is <= ~250 ms); its result is still delivered and callers
 * drop it if they have moved on (see useLiveVehicle).
 *
 * Without Workers (Node, tests, SSR, old browsers) the same lanes run in-process on a macrotask
 * (`LoopbackTransport`), with identical protocol and coalescing, so callers never branch.
 */
import type { DesignParams } from "../waverider/types";
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
    this.w.postMessage({ type: "init", wasmUrl } satisfies WorkerRequest);
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
  constructor(
    readonly name: string,
    private make: () => Transport,
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
  private seq = 1;

  constructor(opts: ComputeClientOptions = {}) {
    this.runner = opts.runner ?? (workersAvailable() ? "worker" : "inline");
    const make = (): Transport => (this.runner === "worker" ? new WebWorkerTransport(opts.wasmUrl) : new LoopbackTransport(opts.clone));
    this.fast = new Lane("fast", make);
    this.slow = new Lane("slow", make);
  }

  get busy() {
    return this.fast.busy || this.slow.busy;
  }

  /** Run a study. Rejects with SupersededError if a newer request on the same lane replaced it before it started. */
  study(params: DesignParams, mode: StudyMode, have?: string): Promise<StudyReply> {
    const lane = mode === "full" ? this.slow : this.fast;
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
  }
}

let shared: ComputeClient | null = null;
/** Process-wide client. In the browser it spawns its workers lazily on first use. */
export function getComputeClient(): ComputeClient {
  if (!shared) shared = new ComputeClient({ wasmUrl: "/cuspis_euler.wasm" });
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
