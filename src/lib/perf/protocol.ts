/** Message protocol between ComputeClient and the compute worker (also used by the loopback runner). */
import type { StudyResult } from "../waverider/study";
import type { BuiltVehicle, DesignParams } from "../waverider/types";
import type { StudyMode } from "./pipeline";

export interface ExportSources {
  python: string;
  cSrc: string;
  cppSrc: string;
  kernelSource: string;
}

export type WorkerRequest =
  | { type: "study"; id: number; params: DesignParams; mode: StudyMode; /** geometry key the caller already holds */ have?: string }
  | { type: "export"; id: number; kind: string; params: DesignParams; sources: ExportSources }
  | { type: "init"; wasmUrl?: string }
  | { type: "stats"; id: number };

export interface StudyReply {
  type: "study";
  id: number;
  mode: StudyMode;
  geomKey: string;
  studyKey: string;
  /** absent when `have` matched geomKey */
  built?: BuiltVehicle;
  study: StudyResult;
  hit: { geometry: boolean; study: boolean };
  ms: { geometry: number; study: number; total: number };
}

export type WorkerReply =
  | StudyReply
  | { type: "export"; id: number; files: { name: string; blob: Blob }[]; ms: number }
  | { type: "stats"; id: number; stats: unknown }
  | { type: "error"; id: number; message: string }
  | { type: "ready"; wasm: "none" | "streaming" | "fetch" | "file" | "embedded" };
