/**
 * [p7-project] Version history: snapshots (commits) on named branches. All functions are pure and return a new doc.
 * Snapshots store a full copy of the state (simple, robust restore; a design is a few KB). An attached aero
 * database is stored once per snapshot too, so very large databases make history heavy (see NOTES limits).
 */
import { diffStates, summarizeDiff } from "./diff.ts";
import {
  HISTORY_FORMAT,
  MAX_SNAPSHOTS,
  PROJECT_SCHEMA_VERSION,
  ProjectFileError,
  deepClone,
  newId,
  type ProjectDoc,
  type ProjectHistory,
  type ProjectState,
  type Snapshot,
} from "./schema.ts";

export function findSnapshot(doc: ProjectDoc, id: string): Snapshot | undefined {
  return doc.history.snapshots.find((s) => s.id === id);
}

/** Newest first; optionally one branch. */
export function listSnapshots(doc: ProjectDoc, branch?: string): Snapshot[] {
  return doc.history.snapshots
    .filter((s) => !branch || s.branch === branch)
    .slice()
    .sort((a, b) => (a.createdISO < b.createdISO ? 1 : a.createdISO > b.createdISO ? -1 : 0));
}

function summaryFor(parent: Snapshot | undefined, state: ProjectState) {
  if (!parent) return { summary: "Initial snapshot", changes: 0 };
  const d = diffStates(parent.state, state);
  return { summary: summarizeDiff(d), changes: d.length };
}

/** Commit the given working state as a snapshot on the current branch (parent = head). */
export function commitSnapshot(
  doc: ProjectDoc,
  working: ProjectState,
  message: string,
  opts: { nowISO?: string; id?: string } = {},
): { doc: ProjectDoc; snapshot: Snapshot } {
  if (doc.history.snapshots.length >= MAX_SNAPSHOTS) throw new Error(`History is full (${MAX_SNAPSHOTS} snapshots). Delete some first.`);
  const parent = doc.history.head ? findSnapshot(doc, doc.history.head) : undefined;
  const { summary, changes } = summaryFor(parent, working);
  const snapshot: Snapshot = {
    id: opts.id ?? newId("s"),
    parentId: parent?.id ?? null,
    branch: doc.history.branch,
    createdISO: opts.nowISO ?? new Date().toISOString(),
    message: message.trim().slice(0, 200) || "Snapshot",
    summary,
    changes,
    state: deepClone(working),
  };
  return {
    doc: { ...doc, history: { ...doc.history, head: snapshot.id, snapshots: [...doc.history.snapshots, snapshot] } },
    snapshot,
  };
}

/** Restore: returns the snapshot's state as the new working state; head + current branch follow the snapshot. */
export function restoreSnapshot(doc: ProjectDoc, id: string): { doc: ProjectDoc; state: ProjectState } {
  const s = findSnapshot(doc, id);
  if (!s) throw new Error(`Snapshot ${id} not found`);
  return { doc: { ...doc, history: { ...doc.history, head: s.id, branch: s.branch } }, state: deepClone(s.state) };
}

/** Delete a snapshot; its children are re-parented to its parent and their summaries recomputed. */
export function deleteSnapshot(doc: ProjectDoc, id: string): ProjectDoc {
  const victim = findSnapshot(doc, id);
  if (!victim) throw new Error(`Snapshot ${id} not found`);
  const rest = doc.history.snapshots.filter((s) => s.id !== id);
  const byId = new Map(rest.map((s) => [s.id, s]));
  const grandParent = victim.parentId ? byId.get(victim.parentId) : undefined;
  const snapshots = rest.map((s) => {
    if (s.parentId !== id) return s;
    return { ...s, parentId: victim.parentId, ...summaryFor(grandParent, s.state) };
  });
  const branches = doc.history.branches.filter((b) => b === "main" || b === doc.history.branch || snapshots.some((s) => s.branch === b));
  const head = doc.history.head === id ? victim.parentId : doc.history.head;
  return { ...doc, history: { ...doc.history, snapshots, branches, head } };
}

function uniqueBranch(h: ProjectHistory, name: string): string {
  // eslint-disable-next-line no-control-regex -- strip control characters from user-supplied names
  const base = name.trim().replace(/[\u0000-\u001f\u007f]/g, "").slice(0, 48) || "branch";
  let n = base;
  for (let i = 2; h.branches.includes(n); i++) n = `${base}-${i}`;
  return n;
}

/**
 * Duplicate / branch: copy a snapshot onto a new branch (parent = the source) and make it head.
 * The returned state is the copy, so the caller can load it as the working state.
 */
export function branchFromSnapshot(
  doc: ProjectDoc,
  id: string,
  branchName: string,
  opts: { nowISO?: string; id?: string } = {},
): { doc: ProjectDoc; snapshot: Snapshot; state: ProjectState } {
  const src = findSnapshot(doc, id);
  if (!src) throw new Error(`Snapshot ${id} not found`);
  if (doc.history.snapshots.length >= MAX_SNAPSHOTS) throw new Error(`History is full (${MAX_SNAPSHOTS} snapshots). Delete some first.`);
  const branch = uniqueBranch(doc.history, branchName);
  const snapshot: Snapshot = {
    id: opts.id ?? newId("s"),
    parentId: src.id,
    branch,
    createdISO: opts.nowISO ?? new Date().toISOString(),
    message: `Branch "${branch}" from "${src.message}"`,
    summary: "No design changes",
    changes: 0,
    state: deepClone(src.state),
  };
  return {
    doc: {
      ...doc,
      history: { branch, branches: [...doc.history.branches, branch], head: snapshot.id, snapshots: [...doc.history.snapshots, snapshot] },
    },
    snapshot,
    state: deepClone(src.state),
  };
}

/** Switch the current branch (new snapshots go there). */
export function switchBranch(doc: ProjectDoc, branch: string): ProjectDoc {
  if (!doc.history.branches.includes(branch)) throw new Error(`No branch "${branch}"`);
  const tip = listSnapshots(doc, branch)[0];
  return { ...doc, history: { ...doc.history, branch, head: tip?.id ?? doc.history.head } };
}

export interface HistoryExport {
  format: typeof HISTORY_FORMAT;
  schemaVersion: number;
  projectId: string;
  projectName: string;
  exportedISO: string;
  history: ProjectHistory;
}

export function exportHistory(doc: ProjectDoc, nowISO = new Date().toISOString()): string {
  const out: HistoryExport = {
    format: HISTORY_FORMAT,
    schemaVersion: PROJECT_SCHEMA_VERSION,
    projectId: doc.id,
    projectName: doc.name,
    exportedISO: nowISO,
    history: doc.history,
  };
  return JSON.stringify(out, null, 2);
}

/**
 * Import a history export. "merge" (default) adds snapshots whose id is not present yet (parents that do not
 * exist become roots); "replace" swaps the whole history. Validate the text with io.ts parseHistoryExport first.
 */
export function importHistory(
  doc: ProjectDoc,
  parsed: HistoryExport,
  mode: "merge" | "replace" = "merge",
): { doc: ProjectDoc; added: number } {
  if (parsed.format !== HISTORY_FORMAT) throw new ProjectFileError(`expected format "${HISTORY_FORMAT}"`, "format");
  const inc = parsed.history;
  if (mode === "replace") return { doc: { ...doc, history: deepClone(inc) }, added: inc.snapshots.length };
  const have = new Set(doc.history.snapshots.map((s) => s.id));
  const fresh = inc.snapshots.filter((s) => !have.has(s.id));
  if (doc.history.snapshots.length + fresh.length > MAX_SNAPSHOTS) throw new Error(`Import would exceed ${MAX_SNAPSHOTS} snapshots`);
  const all = [...doc.history.snapshots, ...deepClone(fresh)];
  const ids = new Set(all.map((s) => s.id));
  const snapshots = all.map((s) => (s.parentId && !ids.has(s.parentId) ? { ...s, parentId: null } : s));
  const branches = [...new Set([...doc.history.branches, ...snapshots.map((s) => s.branch)])];
  return { doc: { ...doc, history: { ...doc.history, branches, snapshots } }, added: fresh.length };
}

/**
 * Build on the Iterate lab's own browser lists (main e586e26): `cuspis.history.v1` geometry snapshots and
 * `cuspis.variants.v1` saved variants (params + one panel evaluation). They become snapshots on branch "iterate"
 * so they can be restored, compared and exported with the project. Entries already imported (same id) are skipped.
 */
export function importIterateLists(
  doc: ProjectDoc,
  lists: { history?: unknown; variants?: unknown },
  baseState: ProjectState,
  sanitize?: (p: unknown) => ProjectState["params"],
): { doc: ProjectDoc; added: number } {
  const rows: { id: string; at: number; message: string; params: unknown }[] = [];
  const h = Array.isArray(lists.history) ? lists.history : [];
  const v = Array.isArray(lists.variants) ? lists.variants : [];
  for (const r of h) {
    const o = r as { at?: unknown; name?: unknown; params?: unknown };
    if (typeof o?.at === "number" && o.params && typeof o.params === "object") rows.push({ id: `s_iterh_${o.at}`, at: o.at, message: `Iterate history · ${String(o.name ?? "")}`.trim(), params: o.params });
  }
  for (const r of v) {
    const o = r as { id?: unknown; at?: unknown; name?: unknown; params?: unknown; ld?: unknown };
    if (typeof o?.at === "number" && o.params && typeof o.params === "object") {
      const ld = typeof o.ld === "number" ? ` (L/D ${o.ld.toFixed(2)})` : "";
      rows.push({ id: `s_iterv_${String(o.id ?? o.at)}`, at: o.at, message: `Iterate variant · ${String(o.name ?? "")}${ld}`, params: o.params });
    }
  }
  const have = new Set(doc.history.snapshots.map((s) => s.id));
  const fresh = rows.filter((r) => !have.has(r.id)).sort((a, b) => a.at - b.at);
  if (!fresh.length) return { doc, added: 0 };
  if (doc.history.snapshots.length + fresh.length > MAX_SNAPSHOTS) throw new Error(`Import would exceed ${MAX_SNAPSHOTS} snapshots`);
  let prev: Snapshot | undefined;
  const snaps: Snapshot[] = fresh.map((r) => {
    const state: ProjectState = { ...deepClone(baseState), params: sanitize ? sanitize(deepClone(r.params)) : (deepClone(r.params) as ProjectState["params"]), aeroDb: null };
    const s: Snapshot = {
      id: r.id,
      parentId: prev?.id ?? null,
      branch: "iterate",
      createdISO: new Date(r.at).toISOString(),
      message: r.message.slice(0, 200),
      ...summaryFor(prev, state),
      state,
    };
    prev = s;
    return s;
  });
  const branches = doc.history.branches.includes("iterate") ? doc.history.branches : [...doc.history.branches, "iterate"];
  return { doc: { ...doc, history: { ...doc.history, branches, snapshots: [...doc.history.snapshots, ...snaps] } }, added: snaps.length };
}
