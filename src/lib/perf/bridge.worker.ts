/** Worker-thread half of bridge.test.ts: loads the shipped wasm from its bytes, solves with euler_solve_ex, returns result hashes. */
import { parentPort } from "node:worker_threads";
import { readFileSync } from "node:fs";
import { eulerFromBytes, solveWallEx, wasmSource } from "./euler-kernel";
import { hashFloats } from "./hash";
import { presetSettings } from "../waverider/refinement";
import { refinedPlan } from "../waverider/sym-euler";

parentPort!.on("message", async (m: { jobs: { x: Float64Array; z: Float64Array; mach: number; alpha: number }[]; preset: "medium" }) => {
  try {
    await eulerFromBytes(readFileSync(new URL("../../../public/cuspis_euler.wasm", import.meta.url)));
    const s = presetSettings(m.preset);
    const hashes = m.jobs.map((j) => {
      const { v, d, shockS } = refinedPlan(j as never, j.mach, 1.4, j.alpha, s, 0.02);
      const out = solveWallEx(j.x, j.z, j.mach, 1.4, j.alpha, { ni: d.ni, nj: d.nj, iters: d.iters, wallAmp: v.wallClustering, shockAmp: shockS ? v.shockClustering : 0, shockWidth: v.shockWidth, tol: v.tol ?? 0, sequence: v.sequence ?? 0, cfl: 0, shockS });
      if (!out) throw new Error("euler_solve_ex returned null in worker");
      return hashFloats(out);
    });
    parentPort!.postMessage({ hashes, source: wasmSource });
  } catch (e) {
    parentPort!.postMessage({ error: String((e as Error).stack ?? e) });
  }
});
