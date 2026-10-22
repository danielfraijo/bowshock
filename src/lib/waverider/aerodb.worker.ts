/** Browser Web Worker entry for the aero-database pool (aerodb-pool.ts). */
import { AeroWorkerCore, type AeroWorkerReply, type AeroWorkerRequest } from "./aerodb-worker-core";

const scope = self as unknown as {
  postMessage: (m: AeroWorkerReply, transfer: Transferable[]) => void;
  onmessage: ((e: MessageEvent<AeroWorkerRequest>) => void) | null;
};
const core = new AeroWorkerCore((r, transfer) => scope.postMessage(r, transfer ?? []));
scope.onmessage = (e) => core.handle(e.data);
