/**
 * Node transport for AeroPool: worker_threads running AeroWorkerCore (tests, benchmarks, headless CLI).
 * Not imported by browser code. Workers load TypeScript through scripts/ts-register.mjs.
 */
import { cpus } from "node:os";
import { fileURLToPath } from "node:url";
import { Worker } from "node:worker_threads";
import { AeroPool, type SpawnPort, type WorkerPort } from "./aerodb-pool";
import type { AeroWorkerReply } from "./aerodb-worker-core";

const entry = new URL("./aerodb-worker-node.ts", import.meta.url);
const register = fileURLToPath(new URL("../../../scripts/ts-register.mjs", import.meta.url));

export const nodeSpawn: SpawnPort = () => {
  const w = new Worker(entry, { execArgv: ["--import", register] });
  const port: WorkerPort = {
    onReply: null,
    onFail: null,
    post: (msg, transfer) => w.postMessage(msg, (transfer ?? []) as never),
    terminate: () => void w.terminate(),
  };
  w.on("message", (r: AeroWorkerReply) => port.onReply?.(r));
  w.on("error", (e) => port.onFail?.(e instanceof Error ? e : new Error(String(e))));
  return port;
};

export function createNodeAeroPool(size = Math.max(1, cpus().length - 1)): AeroPool {
  return new AeroPool({ size, spawn: nodeSpawn });
}
