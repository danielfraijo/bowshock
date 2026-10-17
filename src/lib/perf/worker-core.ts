/**
 * Transport-independent worker logic. `compute.worker.ts` wires it to `self.onmessage`; the loopback
 * runner (tests, Node, no-Worker environments) calls it directly. Keeps one Pipeline (and therefore
 * one set of caches + one wasm instance) per worker.
 */
import { cfdZip, fileBlobs } from "../waverider/export-kit";
import { studyJson } from "../waverider/study";
import { eulerAsync, eulerLoaded } from "./euler-kernel";
import type { DesignParams } from "../waverider/types";
import { Pipeline } from "./pipeline";
import type { ExportSources, WorkerReply, WorkerRequest } from "./protocol";

export const CAD_KINDS = new Set(["stl", "stl-ascii", "stl-regions", "step", "step-nurbs", "step-tess", "step-facet", "iges", "plot3d", "obj", "kit"]);

export class WorkerCore {
  readonly pipeline = new Pipeline();

  constructor(private post: (r: WorkerReply, transfer?: Transferable[]) => void) {}

  async init(wasmUrl?: string) {
    let how: "streaming" | "embedded" = "embedded";
    try {
      if (!eulerLoaded() && wasmUrl) {
        await eulerAsync(wasmUrl);
        how = "streaming";
      } else {
        await eulerAsync();
      }
    } catch {
      await eulerAsync();
    }
    this.post({ type: "ready", wasm: how });
  }

  async handle(req: WorkerRequest) {
    try {
      if (req.type === "init") return await this.init(req.wasmUrl);
      if (req.type === "stats") return this.post({ type: "stats", id: req.id, stats: this.pipeline.stats() });
      if (req.type === "study") {
        const out = this.pipeline.run(req.params, req.mode);
        this.post({
          type: "study",
          id: req.id,
          mode: req.mode,
          geomKey: out.geomKey,
          studyKey: out.studyKey,
          built: req.have === out.geomKey ? undefined : { ...out.built, params: req.params },
          study: out.study,
          hit: out.hit,
          ms: out.ms,
        });
        return;
      }
      if (req.type === "export") return await this.exportJob(req.id, req.kind, req.params, req.sources);
    } catch (err) {
      this.post({ type: "error", id: (req as { id?: number }).id ?? -1, message: err instanceof Error ? err.message : String(err) });
    }
  }

  private async exportJob(id: number, kind: string, params: DesignParams, sources: ExportSources) {
    const t0 = performance.now();
    if (!CAD_KINDS.has(kind)) throw new Error(`unsupported export kind ${kind}`);
    // Same recipe as Designer.save(): export-density mesh + a quick study for the per-triangle fields.
    const dense = { ...params, nx: Math.max(params.nx, 96), ny: Math.max(params.ny, 72) };
    const out = this.pipeline.run(dense, "quick");
    const src = { ...out.built, params: dense };
    const aero = out.study.aero;
    const fields = { cp: aero.cp, heat: aero.heat, twEq: aero.twEq, stanton: aero.stanton, machE: aero.machE, cf: aero.cf, impact: aero.impact };
    const files = fileBlobs(src, fields);
    const n = files.name;
    const one = (name: string, blob: Blob) => ({ name, blob });
    let list: { name: string; blob: Blob }[];
    switch (kind) {
      case "stl":
        list = [one(`${n}.stl`, files.stlBin)];
        break;
      case "stl-ascii":
        list = [one(`${n}_ascii.stl`, files.stlAscii)];
        break;
      case "stl-regions":
        list = [one(`${n}_regions.stl`, files.stlRegions)];
        break;
      case "step":
      case "step-nurbs":
        list = [one(`${n}_nurbs.step`, files.stepNurbs)];
        break;
      case "step-tess":
        list = [one(`${n}_tess.step`, files.stepTess)];
        break;
      case "step-facet":
        list = [one(`${n}_faceted.step`, files.stepFacet)];
        break;
      case "iges":
        list = [one(`${n}.igs`, files.iges)];
        break;
      case "plot3d":
        list = [one(`${n}.x`, files.plot3d)];
        break;
      case "obj":
        list = [one(`${n}.obj`, files.obj)];
        break;
      default: {
        const zip = await cfdZip(src, sources.python, sources.cSrc, studyJson(src, out.study), sources.cppSrc, fields, sources.kernelSource);
        list = [one(`${n}_cfd_kit.zip`, zip)];
      }
    }
    this.post({ type: "export", id, files: list, ms: performance.now() - t0 });
  }
}
