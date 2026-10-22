/**
 * [p7-project] Serialize / parse .bowshock.json. parseProject never returns a half-valid doc: it either returns a
 * fully validated current-schema ProjectDoc or throws ProjectFileError naming the bad path.
 */
import { migrate } from "./migrate.ts";
import type { HistoryExport } from "./history.ts";
import {
  BANNED_KEYS,
  DEFAULT_UI,
  HISTORY_FORMAT,
  MAX_PROJECT_BYTES,
  MAX_SNAPSHOTS,
  PROJECT_FORMAT,
  PROJECT_SCHEMA_VERSION,
  ProjectFileError,
  canonicalJson,
  docChecksum,
  isObj,
  safeName,
  type AeroDbSlot,
  type ProjectDoc,
  type ProjectHistory,
  type ProjectState,
  type ProjectUi,
  type Snapshot,
} from "./schema.ts";
import type { DesignParams } from "../waverider/types.ts";

export interface ParseOptions {
  /** App-side param normaliser (the designer passes presets.ts sanitizeParams: clamps ranges, fills defaults). */
  sanitizeParams?: (p: unknown) => DesignParams;
}

export interface ParseResult {
  doc: ProjectDoc;
  /** schemaVersion found in the file (0 = bare params). */
  fromVersion: number;
  migrated: boolean;
  warnings: string[];
}

export function serializeProject(doc: ProjectDoc): string {
  const out: ProjectDoc = { ...doc, format: PROJECT_FORMAT, schemaVersion: PROJECT_SCHEMA_VERSION };
  delete out.checksum;
  out.checksum = docChecksum(out);
  return JSON.stringify(out, null, 2);
}

function parseJson(text: string, what: string): unknown {
  if (typeof text !== "string") throw new ProjectFileError(`${what} must be text`);
  if (text.length > MAX_PROJECT_BYTES) throw new ProjectFileError(`${what} is ${(text.length / 1e6).toFixed(1)} MB, over the ${MAX_PROJECT_BYTES / 1024 / 1024} MB limit`);
  if (!text.trim()) throw new ProjectFileError(`${what} is empty`);
  try {
    return JSON.parse(text, (k, v) => (BANNED_KEYS.has(k) ? undefined : v));
  } catch (e) {
    throw new ProjectFileError(`not valid JSON (${e instanceof Error ? e.message : String(e)}); the file may be truncated or corrupted`);
  }
}

const ISO_RE = /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d+)?(Z|[+-]\d{2}:\d{2})$/;
function iso(v: unknown, path: string): string {
  if (typeof v !== "string" || !ISO_RE.test(v) || Number.isNaN(Date.parse(v))) throw new ProjectFileError(`expected an ISO-8601 timestamp, got ${JSON.stringify(v)}`, path);
  return v;
}
function str(v: unknown, path: string, max = 200): string {
  if (typeof v !== "string") throw new ProjectFileError(`expected a string, got ${typeof v}`, path);
  if (v.length > max) throw new ProjectFileError(`string longer than ${max} characters`, path);
  return v;
}

function checkParams(v: unknown, path: string, opts: ParseOptions, warnings: string[]): DesignParams {
  if (!isObj(v)) throw new ProjectFileError("design parameters must be an object", path);
  if (typeof v.family !== "string") throw new ProjectFileError('missing "family" (string)', `${path}.family`);
  for (const [k, val] of Object.entries(v)) {
    if (k === "refinement") {
      if (val !== undefined && val !== null && !isObj(val)) throw new ProjectFileError("expected an object", `${path}.refinement`);
      continue;
    }
    const t = typeof val;
    if (!(t === "string" || t === "boolean" || (t === "number" && Number.isFinite(val as number)) || val === null)) {
      throw new ProjectFileError(`expected a number, string or boolean, got ${Array.isArray(val) ? "array" : t}`, `${path}.${k}`);
    }
  }
  if (!opts.sanitizeParams) return v as unknown as DesignParams;
  const clean = opts.sanitizeParams(v);
  const changed = Object.keys(v).filter((k) => canonicalJson((clean as unknown as Record<string, unknown>)[k]) !== canonicalJson(v[k]));
  if (changed.length) warnings.push(`${path}: ${changed.length} value(s) out of range or unknown were reset/clamped (${changed.slice(0, 6).join(", ")}${changed.length > 6 ? ", …" : ""})`);
  return clean;
}

function checkUi(v: unknown): ProjectUi {
  if (!isObj(v)) return structuredCloneUi(DEFAULT_UI);
  const viewer = isObj(v.viewer) ? v.viewer : {};
  const d = DEFAULT_UI.viewer;
  const b = (x: unknown, dflt: boolean) => (typeof x === "boolean" ? x : dflt);
  return {
    viewer: {
      wireframe: b(viewer.wireframe, d.wireframe),
      shock: b(viewer.shock, d.shock),
      grid: b(viewer.grid, d.grid),
      origin: b(viewer.origin, d.origin),
      color: typeof viewer.color === "string" && viewer.color.length < 32 ? viewer.color : d.color,
    },
    tab: typeof v.tab === "string" && v.tab.length < 32 ? v.tab : DEFAULT_UI.tab,
  };
}
function structuredCloneUi(u: ProjectUi): ProjectUi {
  return { viewer: { ...u.viewer }, tab: u.tab };
}

const SPEC_NUM = ["machMin", "machMax", "machStep", "alphaMin", "alphaMax", "alphaStep", "betaMin", "betaMax", "betaStep", "deltaMin", "deltaMax", "deltaStep", "altKm"] as const;

function checkAeroDb(v: unknown, path: string): AeroDbSlot | null {
  if (v === undefined || v === null) return null;
  if (!isObj(v)) throw new ProjectFileError("expected an object or null", path);
  const spec = v.spec;
  if (!isObj(spec)) throw new ProjectFileError("aero-database grid spec must be an object", `${path}.spec`);
  for (const k of SPEC_NUM) {
    if (typeof spec[k] !== "number" || !Number.isFinite(spec[k])) throw new ProjectFileError(`expected a finite number`, `${path}.spec.${k}`);
  }
  if (typeof spec.method !== "string") throw new ProjectFileError("expected a method name", `${path}.spec.method`);
  const db = v.database;
  if (db !== undefined && db !== null) {
    if (!isObj(db)) throw new ProjectFileError("expected an object or null", `${path}.database`);
    if (!isObj(db.meta)) throw new ProjectFileError("missing meta block", `${path}.database.meta`);
    if (!Array.isArray(db.points)) throw new ProjectFileError("points must be an array", `${path}.database.points`);
    if (!isObj(db.axes)) throw new ProjectFileError("missing axes", `${path}.database.axes`);
    db.points.forEach((pt, i) => {
      if (!isObj(pt) || typeof pt.mach !== "number" || typeof pt.alpha_deg !== "number" || typeof pt.cl !== "number" || typeof pt.cd !== "number") {
        throw new ProjectFileError("each point needs numeric mach, alpha_deg, cl, cd", `${path}.database.points[${i}]`);
      }
    });
  }
  const out: AeroDbSlot = { spec: spec as unknown as AeroDbSlot["spec"], database: (db as unknown as AeroDbSlot["database"]) ?? null };
  if (typeof v.builtFor === "string" && v.builtFor.length <= 16) out.builtFor = v.builtFor;
  return out;
}

function checkState(v: unknown, path: string, opts: ParseOptions, warnings: string[]): ProjectState {
  if (!isObj(v)) throw new ProjectFileError("expected an object", path);
  return {
    params: checkParams(v.params, `${path}.params`, opts, warnings),
    ui: checkUi(v.ui),
    aeroDb: checkAeroDb(v.aeroDb, `${path}.aeroDb`),
  };
}

export function checkHistory(v: unknown, path: string, opts: ParseOptions, warnings: string[]): ProjectHistory {
  if (!isObj(v)) throw new ProjectFileError("expected an object", path);
  if (!Array.isArray(v.snapshots)) throw new ProjectFileError("snapshots must be an array", `${path}.snapshots`);
  if (v.snapshots.length > MAX_SNAPSHOTS) throw new ProjectFileError(`more than ${MAX_SNAPSHOTS} snapshots`, `${path}.snapshots`);
  const seen = new Set<string>();
  const snapshots: Snapshot[] = v.snapshots.map((s, i) => {
    const p = `${path}.snapshots[${i}]`;
    if (!isObj(s)) throw new ProjectFileError("expected an object", p);
    const id = str(s.id, `${p}.id`, 80);
    if (!id) throw new ProjectFileError("empty id", `${p}.id`);
    if (seen.has(id)) throw new ProjectFileError(`duplicate snapshot id "${id}"`, `${p}.id`);
    seen.add(id);
    if (s.parentId !== null && typeof s.parentId !== "string") throw new ProjectFileError("expected a string or null", `${p}.parentId`);
    return {
      id,
      parentId: (s.parentId as string | null) ?? null,
      branch: str(s.branch, `${p}.branch`, 64) || "main",
      createdISO: iso(s.createdISO, `${p}.createdISO`),
      message: str(s.message, `${p}.message`),
      summary: typeof s.summary === "string" ? s.summary.slice(0, 400) : "",
      changes: typeof s.changes === "number" && Number.isFinite(s.changes) ? s.changes : 0,
      state: checkState(s.state, `${p}.state`, opts, warnings),
    };
  });
  for (const s of snapshots) {
    if (s.parentId && !seen.has(s.parentId)) {
      warnings.push(`${path}: snapshot ${s.id} referenced a missing parent; it is now a root`);
      s.parentId = null;
    }
  }
  const branch = typeof v.branch === "string" && v.branch ? v.branch : "main";
  const listed = Array.isArray(v.branches) ? v.branches.filter((b): b is string => typeof b === "string" && !!b) : [];
  const branches = [...new Set(["main", ...listed, branch, ...snapshots.map((s) => s.branch)])];
  let head: string | null = typeof v.head === "string" ? v.head : null;
  if (head && !seen.has(head)) {
    warnings.push(`${path}.head pointed at a missing snapshot; reset`);
    head = null;
  }
  return { branch, branches, head, snapshots };
}

/** Parse + migrate + validate a .bowshock.json (or a bare design-params JSON). Throws ProjectFileError. */
export function parseProject(text: string, opts: ParseOptions = {}): ParseResult {
  const parsed = parseJson(text, "project file");
  const { raw, from, steps } = migrate(parsed);
  const warnings: string[] = [];
  if (from === 0) warnings.push("Imported a bare design-parameter file (schema v0) as a new project");
  else if (steps.length) warnings.push(`Migrated from schemaVersion ${from} to ${PROJECT_SCHEMA_VERSION}`);
  const now = new Date().toISOString();
  const doc: ProjectDoc = {
    format: PROJECT_FORMAT,
    schemaVersion: PROJECT_SCHEMA_VERSION,
    id: str(raw.id, "id", 80) || "p_imported",
    name: safeName(raw.name),
    createdISO: raw.createdISO === undefined ? now : iso(raw.createdISO, "createdISO"),
    modifiedISO: raw.modifiedISO === undefined ? now : iso(raw.modifiedISO, "modifiedISO"),
    app: { name: "bowshock", schema: PROJECT_SCHEMA_VERSION },
    state: checkState(raw.state, "state", opts, warnings),
    history: checkHistory(raw.history, "history", opts, warnings),
  };
  if (typeof raw.checksum === "string" && !steps.length) {
    const expect = docChecksum({ state: raw.state as ProjectState, history: raw.history as ProjectHistory });
    if (expect !== raw.checksum) warnings.push("Checksum mismatch: the file was edited outside Bowshock (loaded anyway after validation)");
  }
  return { doc, fromVersion: from, migrated: steps.length > 0, warnings };
}

/** Parse + validate a history export (History → Export). */
export function parseHistoryExport(text: string, opts: ParseOptions = {}): { data: HistoryExport; warnings: string[] } {
  const raw = parseJson(text, "history file");
  if (!isObj(raw)) throw new ProjectFileError("top level must be a JSON object");
  if (raw.format === PROJECT_FORMAT) throw new ProjectFileError("this is a project file; use Open instead of History → Import", "format");
  if (raw.format !== HISTORY_FORMAT) throw new ProjectFileError(`expected format "${HISTORY_FORMAT}", got ${JSON.stringify(raw.format)}`, "format");
  if (typeof raw.schemaVersion !== "number" || raw.schemaVersion > PROJECT_SCHEMA_VERSION || raw.schemaVersion < 2) {
    throw new ProjectFileError(`unsupported history schemaVersion ${JSON.stringify(raw.schemaVersion)}`, "schemaVersion");
  }
  const warnings: string[] = [];
  const history = checkHistory(raw.history, "history", opts, warnings);
  return {
    data: {
      format: HISTORY_FORMAT,
      schemaVersion: PROJECT_SCHEMA_VERSION,
      projectId: typeof raw.projectId === "string" ? raw.projectId : "",
      projectName: safeName(raw.projectName),
      exportedISO: typeof raw.exportedISO === "string" ? raw.exportedISO : "",
      history,
    },
    warnings,
  };
}
