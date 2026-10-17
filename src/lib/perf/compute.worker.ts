/// <reference lib="webworker" />
/**
 * Compute worker entry (Vite bundles this with `new Worker(new URL("./compute.worker.ts", import.meta.url), { type: "module" })`).
 * All heavy numerics run here; the main thread only posts parameters and receives results.
 */
import { WorkerCore } from "./worker-core";
import type { WorkerRequest } from "./protocol";

const core = new WorkerCore((reply, transfer) => (self as unknown as Worker).postMessage(reply, transfer ?? []));
self.onmessage = (e: MessageEvent<WorkerRequest>) => {
  void core.handle(e.data);
};
