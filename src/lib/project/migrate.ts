/**
 * [p7-project] Forward migrations. MIGRATIONS[n] upgrades a schemaVersion-n document to n+1.
 * Migrations only reshape; validation (io.ts) runs on the result, so a migration never has to be defensive
 * about values, only about shape.
 */
import { DEFAULT_UI, PROJECT_FORMAT, PROJECT_SCHEMA_VERSION, ProjectFileError, isObj, newId, safeName } from "./schema.ts";

type Raw = Record<string, unknown>;

/** v0 (bare DesignParams, e.g. the old localStorage blob or a hand-saved params JSON) → v1. */
function v0to1(raw: Raw): Raw {
  return {
    format: PROJECT_FORMAT,
    schemaVersion: 1,
    name: typeof raw.name === "string" ? raw.name : "Imported design",
    savedAt: new Date(0).toISOString(),
    params: raw,
    snapshots: [],
  };
}

/** v1 (linear snapshots of params, optional view) → v2 (state/ui/aeroDb, branches, head, summaries). */
function v1to2(raw: Raw): Raw {
  const savedAt = typeof raw.savedAt === "string" ? raw.savedAt : new Date(0).toISOString();
  const view = isObj(raw.view) ? raw.view : {};
  const ui = { viewer: { ...DEFAULT_UI.viewer, ...(isObj(view.viewer) ? view.viewer : {}) }, tab: typeof view.tab === "string" ? view.tab : DEFAULT_UI.tab };
  const old = Array.isArray(raw.snapshots) ? raw.snapshots : [];
  let parentId: string | null = null;
  const snapshots = old.map((s, i) => {
    const o = isObj(s) ? s : {};
    const id = typeof o.id === "string" && o.id ? o.id : `s_v1_${i}`;
    const snap = {
      id,
      parentId,
      branch: "main",
      createdISO: typeof o.time === "string" ? o.time : savedAt,
      message: typeof o.message === "string" ? o.message : `Snapshot ${i + 1}`,
      summary: i === 0 ? "Initial snapshot" : "Migrated from schema v1",
      changes: 0,
      state: { params: o.params, ui, aeroDb: null },
    };
    parentId = id;
    return snap;
  });
  return {
    format: PROJECT_FORMAT,
    schemaVersion: 2,
    id: typeof raw.id === "string" && raw.id ? raw.id : newId("p"),
    name: safeName(raw.name),
    createdISO: savedAt,
    modifiedISO: savedAt,
    app: { name: "bowshock", schema: 2 },
    state: { params: raw.params, ui, aeroDb: null },
    history: { branch: "main", branches: ["main"], head: parentId, snapshots },
  };
}

export const MIGRATIONS: Record<number, (raw: Raw) => Raw> = { 0: v0to1, 1: v1to2 };

/** Version of a raw parsed document: 0 for a bare DesignParams object. Throws on anything unrecognisable. */
export function detectVersion(raw: unknown): number {
  if (!isObj(raw)) throw new ProjectFileError("top level must be a JSON object");
  if (raw.format === undefined && raw.schemaVersion === undefined) {
    if (typeof raw.family === "string") return 0;
    throw new ProjectFileError('not a Bowshock project (no "format" field and not a bare design-parameter object)');
  }
  if (raw.format === "bowshock-history") throw new ProjectFileError("this is a version-history export; use History → Import instead of Open", "format");
  if (raw.format !== PROJECT_FORMAT) throw new ProjectFileError(`expected format "${PROJECT_FORMAT}", got ${JSON.stringify(raw.format)}`, "format");
  const v = raw.schemaVersion;
  if (typeof v !== "number" || !Number.isInteger(v) || v < 1) throw new ProjectFileError(`schemaVersion must be a positive integer, got ${JSON.stringify(v)}`, "schemaVersion");
  if (v > PROJECT_SCHEMA_VERSION) {
    throw new ProjectFileError(`schemaVersion ${v} is newer than this app supports (${PROJECT_SCHEMA_VERSION}); update Bowshock`, "schemaVersion");
  }
  return v;
}

/** Upgrade to the current schema. Returns the raw (still unvalidated) document and the versions it went through. */
export function migrate(raw: unknown): { raw: Raw; from: number; steps: number[] } {
  const from = detectVersion(raw);
  let cur = raw as Raw;
  const steps: number[] = [];
  for (let v = from; v < PROJECT_SCHEMA_VERSION; v++) {
    const m = MIGRATIONS[v];
    if (!m) throw new ProjectFileError(`no migration from schemaVersion ${v}`, "schemaVersion");
    cur = m(cur);
    steps.push(v + 1);
  }
  return { raw: cur, from, steps };
}
