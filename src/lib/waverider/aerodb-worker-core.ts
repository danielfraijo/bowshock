/**
 * Transport-independent aero-database worker logic. `aerodb.worker.ts` wires it to `self.onmessage`
 * (browser), `aerodb-pool-node.ts` to worker_threads (Node), and `loopbackPort` in aerodb-pool.ts calls it
 * in-process (tests, no-Worker environments).
 *
 * Pattern reused from feat/performance (PR #12, src/lib/perf/worker-core.ts at a110aba): one core per
 * worker, a `post` callback injected by the transport, every request answered by id, errors as replies.
 * Each core owns one cuspis-panel wasm instance and the meshes the pool has sent it.
 */
import { evalCase, type BatchEngine } from "./aerodb-batch";
import { panelKernel } from "./panel-kernel";
import type { DesignParams, TriMesh } from "./types";

export type AeroWorkerRequest =
  | { type: "init"; id: number }
  | { type: "mesh"; id: number; meshId: number; mesh: TriMesh }
  | { type: "drop"; id: number; meshId: number }
  | {
      type: "eval";
      id: number;
      meshId: number;
      params: DesignParams;
      altKm: number;
      sRef: number;
      lRef: number;
      /** packed points, STRIDE_PT f64 each (aerodb-batch.ts) */
      points: Float64Array;
      engine: BatchEngine;
    };

export type AeroWorkerReply =
  | { type: "ready"; id: number; wasm: boolean }
  | { type: "ok"; id: number }
  | { type: "result"; id: number; out: Float64Array; engine: BatchEngine; ms: number }
  | { type: "error"; id: number; message: string };

export class AeroWorkerCore {
  private meshes = new Map<number, TriMesh>();

  constructor(private post: (r: AeroWorkerReply, transfer?: Transferable[]) => void) {}

  handle(req: AeroWorkerRequest) {
    try {
      if (req.type === "init") return this.post({ type: "ready", id: req.id, wasm: panelKernel() !== null });
      if (req.type === "mesh") {
        this.meshes.set(req.meshId, req.mesh);
        return this.post({ type: "ok", id: req.id });
      }
      if (req.type === "drop") {
        const m = this.meshes.get(req.meshId);
        if (m) panelKernel()?.drop(m);
        this.meshes.delete(req.meshId);
        return this.post({ type: "ok", id: req.id });
      }
      if (req.type === "eval") {
        const mesh = this.meshes.get(req.meshId);
        if (!mesh) throw new Error(`aerodb worker: mesh ${req.meshId} was not sent`);
        const t0 = performance.now();
        const r = evalCase({ mesh, params: req.params, altKm: req.altKm, sRef: req.sRef, lRef: req.lRef }, req.points, req.engine);
        return this.post({ type: "result", id: req.id, out: r.out, engine: r.engine, ms: performance.now() - t0 }, [r.out.buffer]);
      }
      throw new Error(`aerodb worker: unknown request ${(req as { type?: string }).type}`);
    } catch (err) {
      this.post({ type: "error", id: (req as { id?: number }).id ?? -1, message: err instanceof Error ? err.message : String(err) });
    }
  }
}
