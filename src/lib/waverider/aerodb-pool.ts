/**
 * Worker pool for aero-database batches: chunks the points of every (mesh, params) case across workers,
 * writes results back by index (deterministic order whatever finishes first), reports progress per chunk
 * and cancels on AbortSignal (workers are terminated and respawned lazily on the next run).
 *
 * Transports: Web Workers in the browser (`browserSpawn`), worker_threads in Node (aerodb-pool-node.ts),
 * or `loopbackPort` (in-process, same messages; the fallback when neither exists).
 */
import { STRIDE_OUT, STRIDE_PT, type BatchCase, type BatchEngine } from "./aerodb-batch";
import { AeroWorkerCore, type AeroWorkerReply, type AeroWorkerRequest } from "./aerodb-worker-core";
import type { TriMesh } from "./types";

export interface WorkerPort {
  post(msg: AeroWorkerRequest, transfer?: Transferable[]): void;
  onReply: ((r: AeroWorkerReply) => void) | null;
  onFail: ((err: Error) => void) | null;
  terminate(): void;
}

export type SpawnPort = (index: number) => WorkerPort;

export interface PoolJob {
  case: BatchCase;
  /** packed points (STRIDE_PT f64 each) */
  points: Float64Array;
}

export interface PoolProgress {
  done: number;
  total: number;
}

export interface PoolRunOptions {
  signal?: AbortSignal;
  onProgress?: (p: PoolProgress) => void;
  /** points per chunk; default ≈ total / (4·workers), clamped to [4, 96] */
  chunkSize?: number;
  engine?: BatchEngine;
}

export interface PoolRunResult {
  outs: Float64Array[];
  /** "wasm" when every chunk ran in the kernel */
  engine: BatchEngine | "mixed";
  chunks: number;
  workers: number;
}

/** In-process port: the same core and messages, replies on a macrotask so the caller's event loop keeps turning. */
export function loopbackPort(): WorkerPort {
  let dead = false;
  const port: WorkerPort = {
    onReply: null,
    onFail: null,
    post(msg) {
      setTimeout(() => {
        if (!dead) core.handle(msg);
      }, 0);
    },
    terminate() {
      dead = true;
    },
  };
  const core = new AeroWorkerCore((r) => {
    if (!dead) port.onReply?.(r);
  });
  return port;
}

/** Web Worker transport, or null outside a browser. */
export function browserSpawn(): SpawnPort | null {
  if (typeof Worker === "undefined" || typeof window === "undefined") return null;
  return () => {
    const w = new Worker(new URL("./aerodb.worker.ts", import.meta.url), { type: "module", name: "aerodb" });
    const port: WorkerPort = {
      onReply: null,
      onFail: null,
      post: (msg, transfer) => w.postMessage(msg, transfer ?? []),
      terminate: () => w.terminate(),
    };
    w.onmessage = (e: MessageEvent<AeroWorkerReply>) => port.onReply?.(e.data);
    w.onerror = (e) => {
      e.preventDefault?.();
      port.onFail?.(new Error(`aerodb worker failed: ${e.message || "load error"}`));
    };
    return port;
  };
}

export function defaultPoolSize(): number {
  const hc = typeof navigator !== "undefined" && navigator.hardwareConcurrency ? navigator.hardwareConcurrency : 4;
  return Math.max(1, Math.min(8, hc - 1));
}

const meshIds = new WeakMap<TriMesh, number>();
let nextMeshId = 1;
function meshId(m: TriMesh): number {
  let id = meshIds.get(m);
  if (!id) {
    id = nextMeshId++;
    meshIds.set(m, id);
  }
  return id;
}

interface Slot {
  port: WorkerPort;
  meshes: Set<number>;
}

function abortError() {
  return new DOMException("Aero database cancelled", "AbortError");
}

export class AeroPool {
  readonly size: number;
  private spawn: SpawnPort;
  private slots: Slot[] = [];
  private seq = 1;
  private running = false;

  constructor(opts: { size?: number; spawn?: SpawnPort } = {}) {
    this.size = Math.max(1, Math.floor(opts.size ?? defaultPoolSize()));
    this.spawn = opts.spawn ?? browserSpawn() ?? (() => loopbackPort());
  }

  private ensure() {
    while (this.slots.length < this.size) this.slots.push({ port: this.spawn(this.slots.length), meshes: new Set() });
  }

  /** Kill every worker; the next run respawns them. */
  terminate() {
    for (const s of this.slots) s.port.terminate();
    this.slots = [];
  }

  run(jobs: PoolJob[], opts: PoolRunOptions = {}): Promise<PoolRunResult> {
    if (this.running) return Promise.reject(new Error("AeroPool.run is already in progress"));
    if (opts.signal?.aborted) return Promise.reject(abortError());
    this.ensure();
    const engine = opts.engine ?? "wasm";
    const outs = jobs.map((j) => new Float64Array(Math.floor(j.points.length / STRIDE_PT) * STRIDE_OUT));
    const total = jobs.reduce((s, j) => s + Math.floor(j.points.length / STRIDE_PT), 0);
    const chunk = Math.max(1, Math.floor(opts.chunkSize ?? Math.min(96, Math.max(4, Math.ceil(total / (4 * this.size))))));
    const queue: { job: number; start: number; end: number }[] = [];
    jobs.forEach((j, ji) => {
      const n = Math.floor(j.points.length / STRIDE_PT);
      for (let s = 0; s < n; s += chunk) queue.push({ job: ji, start: s, end: Math.min(n, s + chunk) });
    });
    const nChunks = queue.length;
    if (total === 0) return Promise.resolve({ outs, engine, chunks: 0, workers: this.size });
    this.running = true;

    return new Promise<PoolRunResult>((resolve, reject) => {
      let done = 0;
      let settled = false;
      let allWasm = true;
      let anyWasm = false;
      const pending = new Map<number, { slot: Slot; job: number; start: number }>();
      const used = new Set<number>();

      const finish = (err: unknown | null) => {
        if (settled) return;
        settled = true;
        this.running = false;
        opts.signal?.removeEventListener("abort", onAbort);
        for (const s of this.slots) {
          s.port.onReply = null;
          s.port.onFail = null;
        }
        if (err) {
          // a worker may still be busy on a cancelled chunk: kill and respawn next time
          this.terminate();
          reject(err);
          return;
        }
        for (const s of this.slots) {
          for (const id of used) {
            if (s.meshes.has(id)) s.port.post({ type: "drop", id: this.seq++, meshId: id });
          }
          for (const id of used) s.meshes.delete(id);
        }
        resolve({ outs, engine: allWasm ? "wasm" : anyWasm ? "mixed" : "ts", chunks: nChunks, workers: this.size });
      };
      const onAbort = () => finish(abortError());
      opts.signal?.addEventListener("abort", onAbort, { once: true });

      const dispatch = (slot: Slot) => {
        if (settled) return;
        const next = queue.shift();
        if (!next) {
          if (pending.size === 0) finish(null);
          return;
        }
        const job = jobs[next.job];
        const c = job.case;
        const mid = meshId(c.mesh);
        used.add(mid);
        if (!slot.meshes.has(mid)) {
          slot.port.post({ type: "mesh", id: this.seq++, meshId: mid, mesh: c.mesh });
          slot.meshes.add(mid);
        }
        const id = this.seq++;
        pending.set(id, { slot, job: next.job, start: next.start });
        slot.port.post({
          type: "eval",
          id,
          meshId: mid,
          params: c.params,
          altKm: c.altKm,
          sRef: c.sRef,
          lRef: c.lRef,
          points: job.points.slice(next.start * STRIDE_PT, next.end * STRIDE_PT),
          engine,
        });
      };

      for (const slot of this.slots) {
        slot.port.onFail = (err) => finish(err);
        slot.port.onReply = (r) => {
          if (settled) return;
          if (r.type === "error") return finish(new Error(r.message));
          if (r.type !== "result") return;
          const p = pending.get(r.id);
          if (!p) return;
          pending.delete(r.id);
          outs[p.job].set(r.out, p.start * STRIDE_OUT);
          if (r.engine === "wasm") anyWasm = true;
          else allWasm = false;
          done += r.out.length / STRIDE_OUT;
          opts.onProgress?.({ done, total });
          dispatch(slot);
        };
      }
      for (const slot of this.slots) dispatch(slot);
    });
  }
}

let shared: AeroPool | null = null;

/** Shared browser pool (Web Workers). Null where Workers do not exist (Node, SSR). */
export function sharedAeroPool(): AeroPool | null {
  if (shared) return shared;
  const spawn = browserSpawn();
  if (!spawn) return null;
  shared = new AeroPool({ spawn });
  return shared;
}
