/** worker_threads entry for aerodb-pool-node.ts. */
import { parentPort } from "node:worker_threads";
import { AeroWorkerCore, type AeroWorkerRequest } from "./aerodb-worker-core";

const port = parentPort;
if (!port) throw new Error("aerodb-worker-node must run in a worker thread");
const core = new AeroWorkerCore((r, transfer) => port.postMessage(r, (transfer ?? []) as never));
port.on("message", (req: AeroWorkerRequest) => core.handle(req));
