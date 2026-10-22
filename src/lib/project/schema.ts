/**
 * [p7-project] Bowshock project file schema (.bowshock.json), versioned with forward migrations.
 *
 * A project = the designer state (DesignParams: geometry + flight/analysis settings such as aeroMethod, alpha,
 * altitude, cg), the viewer/UI state, an optional aero-database slot (grid spec + results, AeroDB contract
 * 1.x) and the version history (snapshots on named branches).
 *
 * Pure TypeScript, no DOM and no extensionless imports, so it runs under `node --experimental-strip-types`.
 *
 * Schema history
 *   v0  bare DesignParams object (what the app kept in localStorage `cuspis.design.v1` / `bowshock.design.v*`).
 *   v1  { format, schemaVersion: 1, name, savedAt, params, view?, snapshots?: [{ id, time, message, params }] }
 *       (first draft: linear history, no branches, no aero database, no ui/tab).
 *   v2  current: { format, schemaVersion: 2, id, name, createdISO, modifiedISO, app, state, history, checksum }.
 *       state.aeroDb uses main's aerodb.ts types (AeroDbSpec + AeroDatabase "cuspis-aerodb-1").
 */
import type { AeroDatabase, AeroDbSpec } from "../waverider/aerodb.ts";
import type { DesignParams } from "../waverider/types.ts";

export const PROJECT_FORMAT = "bowshock-project";
export const HISTORY_FORMAT = "bowshock-history";
export const PROJECT_SCHEMA_VERSION = 2;
export const PROJECT_FILE_EXT = ".bowshock.json";
/** Hard cap for a file we are willing to parse (a big aero database fits; a runaway file does not). */
export const MAX_PROJECT_BYTES = 32 * 1024 * 1024;
export const MAX_SNAPSHOTS = 500;

export interface ViewerSettings {
  wireframe: boolean;
  shock: boolean;
  grid: boolean;
  origin: boolean;
  color: string;
}

/** View state. Saved and restored, but NOT part of the unsaved-changes check or the param diff. */
export interface ProjectUi {
  viewer: ViewerSettings;
  tab: string;
}

/**
 * Aero-database slot: the Iterate → Aero DB grid spec and, if it was run, the generated table.
 * Types are main's src/lib/waverider/aerodb.ts (AeroDbSpec / AeroDatabase, meta.format "cuspis-aerodb-1").
 */
export interface AeroDbSlot {
  spec: AeroDbSpec;
  database?: AeroDatabase | null;
  /** paramsHash() of the design the database was generated for; differs from the current params ⇒ stale. */
  builtFor?: string;
}

export interface ProjectState {
  params: DesignParams;
  ui: ProjectUi;
  aeroDb?: AeroDbSlot | null;
}

export interface Snapshot {
  id: string;
  parentId: string | null;
  branch: string;
  createdISO: string;
  message: string;
  /** One-line change summary vs the parent snapshot (or "Initial snapshot"). */
  summary: string;
  changes: number;
  state: ProjectState;
}

export interface ProjectHistory {
  /** Current branch: new snapshots go here. */
  branch: string;
  branches: string[];
  /** Snapshot the working state descends from (last commit or restore). */
  head: string | null;
  snapshots: Snapshot[];
}

export interface ProjectDoc {
  format: typeof PROJECT_FORMAT;
  schemaVersion: typeof PROJECT_SCHEMA_VERSION;
  id: string;
  name: string;
  createdISO: string;
  modifiedISO: string;
  app: { name: string; schema: number };
  state: ProjectState;
  history: ProjectHistory;
  /** FNV-1a of the canonical state + history; a mismatch is reported as a warning (hand edits are allowed). */
  checksum?: string;
}

export const DEFAULT_UI: ProjectUi = {
  viewer: { wireframe: false, shock: true, grid: true, origin: true, color: "surface" },
  tab: "geom",
};

export class ProjectFileError extends Error {
  readonly path: string;
  constructor(message: string, path = "") {
    super(path ? `Invalid project file at ${path}: ${message}` : `Invalid project file: ${message}`);
    this.name = "ProjectFileError";
    this.path = path;
  }
}

export const BANNED_KEYS = new Set(["__proto__", "constructor", "prototype"]);

// ───────────────────────── helpers ─────────────────────────

export function isObj(v: unknown): v is Record<string, unknown> {
  return !!v && typeof v === "object" && !Array.isArray(v);
}

/** Deterministic JSON (sorted keys) for hashing, equality and the dirty check. */
export function canonicalJson(v: unknown): string {
  return JSON.stringify(v, (_k, val) => {
    if (isObj(val)) {
      const out: Record<string, unknown> = {};
      for (const k of Object.keys(val).sort()) out[k] = val[k];
      return out;
    }
    return val;
  });
}

export function fnv1a(s: string): string {
  let h = 0x811c9dc5;
  for (let i = 0; i < s.length; i++) {
    h ^= s.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h.toString(16).padStart(8, "0");
}

export function deepClone<T>(v: T): T {
  return v === undefined ? v : (JSON.parse(JSON.stringify(v)) as T);
}

export function newId(prefix = "p"): string {
  const c = (globalThis as { crypto?: { randomUUID?: () => string } }).crypto;
  const r = c?.randomUUID ? c.randomUUID().replace(/-/g, "").slice(0, 16) : Math.random().toString(16).slice(2, 18).padEnd(16, "0");
  return `${prefix}_${r}`;
}

/** The part of the state that counts as "the design" (dirty check + diff). UI/view state is excluded. */
export function designOf(s: ProjectState): { params: DesignParams; aeroDb: AeroDbSlot | null } {
  return { params: s.params, aeroDb: s.aeroDb ?? null };
}

/** Hash of the design parameters (used to tag which design an aero database belongs to). */
export function paramsHash(p: DesignParams): string {
  return fnv1a(canonicalJson(p));
}

/** True when the slot holds a database generated for different params than `p` (untagged ⇒ not stale). */
export function aeroDbStale(slot: AeroDbSlot | null | undefined, p: DesignParams): boolean {
  return !!slot?.database && !!slot.builtFor && slot.builtFor !== paramsHash(p);
}

export function stateHash(s: ProjectState): string {
  return fnv1a(canonicalJson(designOf(s)));
}

export function sameDesign(a: ProjectState, b: ProjectState): boolean {
  return canonicalJson(designOf(a)) === canonicalJson(designOf(b));
}

export function docChecksum(doc: Pick<ProjectDoc, "state" | "history">): string {
  return fnv1a(canonicalJson({ state: doc.state, history: doc.history }));
}

export function safeName(v: unknown, fallback = "Untitled project"): string {
  if (typeof v !== "string") return fallback;
  // eslint-disable-next-line no-control-regex -- strip control characters from user-supplied names
  const cleaned = v.replace(/[\u0000-\u001f\u007f]/g, "").trim().slice(0, 96);
  return cleaned || fallback;
}

export function projectFileName(name: string): string {
  const base = safeName(name).replace(/[^A-Za-z0-9._-]+/g, "_").replace(/^_+|_+$/g, "") || "project";
  return `${base}${PROJECT_FILE_EXT}`;
}

export function newProject(params: DesignParams, opts: { name?: string; ui?: ProjectUi; aeroDb?: AeroDbSlot | null; nowISO?: string; id?: string } = {}): ProjectDoc {
  const now = opts.nowISO ?? new Date().toISOString();
  return {
    format: PROJECT_FORMAT,
    schemaVersion: PROJECT_SCHEMA_VERSION,
    id: opts.id ?? newId("p"),
    name: safeName(opts.name),
    createdISO: now,
    modifiedISO: now,
    app: { name: "bowshock", schema: PROJECT_SCHEMA_VERSION },
    state: { params: deepClone(params), ui: deepClone(opts.ui ?? DEFAULT_UI), aeroDb: deepClone(opts.aeroDb ?? null) },
    history: { branch: "main", branches: ["main"], head: null, snapshots: [] },
  };
}
