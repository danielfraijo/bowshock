/**
 * [p7-project] Project session hook: binds the designer's live state (params, viewer, tab, Iterate aero DB) to a
 * ProjectDoc with version history, autosaves the session to IndexedDB (localStorage fallback) and exposes the
 * New / Save / Save As / Open / Snapshot / Restore / Branch / Delete / Export / Import actions.
 */
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { sanitizeParams } from "@/lib/waverider/presets";
import { DEFAULT_PARAMS, type ColorMode, type DesignParams } from "@/lib/waverider/types";
import type { ViewerOpts } from "@/components/viewer";
import { downloadBlob } from "@/lib/utils";
import {
  ProjectStore,
  branchFromSnapshot,
  commitSnapshot,
  deleteSnapshot,
  exportHistory,
  importHistory,
  importIterateLists,
  newId,
  newProject,
  openDefaultKv,
  paramsHash,
  parseHistoryExport,
  parseProject,
  projectFileName,
  restoreSnapshot,
  sameDesign,
  serializeProject,
  switchBranch,
  type AeroDbSlot,
  type ProjectDoc,
  type ProjectState,
  type ProjectSummary,
} from "@/lib/project/index";

const PARSE = { sanitizeParams };

export interface ProjectSession {
  ready: boolean;
  backend: string;
  doc: ProjectDoc;
  working: ProjectState;
  dirty: boolean;
  /** Bumped whenever a load/restore replaces the state, so panels with local state (Iterate) can remount. */
  rev: number;
  aeroDb: AeroDbSlot | null;
  setAeroDb: (s: AeroDbSlot | null) => void;
  message: { text: string; warn: boolean } | null;
  library: ProjectSummary[];
  refreshLibrary: () => Promise<void>;
  rename: (name: string) => void;
  newProject: () => void;
  save: () => Promise<void>;
  saveAs: (name: string) => Promise<void>;
  openFile: (file: File) => Promise<void>;
  openStored: (id: string) => Promise<void>;
  deleteStored: (id: string) => Promise<void>;
  download: () => void;
  snapshot: (message: string) => Promise<void>;
  restore: (id: string) => void;
  branch: (id: string, name: string) => Promise<void>;
  switchTo: (branch: string) => void;
  remove: (id: string) => Promise<void>;
  exportHistory: () => void;
  importHistory: (file: File) => Promise<void>;
  importIterate: () => Promise<void>;
  iterateCount: number;
}

function readIterate(key: string): unknown[] {
  try {
    const v = JSON.parse(localStorage.getItem(key) ?? "[]");
    return Array.isArray(v) ? v : [];
  } catch {
    return [];
  }
}

export function useProjectSession(o: {
  params: DesignParams;
  setParams: (p: DesignParams) => void;
  viewer: ViewerOpts;
  setViewer: (v: ViewerOpts) => void;
  tab: string;
  setTab: (t: string) => void;
  hydrated: boolean;
}): ProjectSession {
  const [doc, setDoc] = useState<ProjectDoc>(() => newProject(DEFAULT_PARAMS, { name: "Untitled project", id: "p_boot" }));
  const [aeroDb, setAeroDb] = useState<AeroDbSlot | null>(null);
  const [rev, setRev] = useState(0);
  const [ready, setReady] = useState(false);
  const [message, setMessage] = useState<{ text: string; warn: boolean } | null>(null);
  const [library, setLibrary] = useState<ProjectSummary[]>([]);
  const [iterateCount, setIterateCount] = useState(0);
  const store = useRef<ProjectStore | null>(null);
  const paramsRef = useRef(o.params);
  paramsRef.current = o.params;
  /** Iterate reports spec/results; a NEW database is tagged with the params it was generated for. */
  const setAeroDbTagged = useCallback((s: AeroDbSlot | null) => {
    setAeroDb((prev) => {
      if (!s) return null;
      const builtFor = s.database ? (prev?.database === s.database && prev?.builtFor ? prev.builtFor : paramsHash(paramsRef.current)) : undefined;
      return builtFor ? { ...s, builtFor } : { spec: s.spec, database: s.database ?? null };
    });
  }, []);

  const working: ProjectState = useMemo(
    () => ({ params: o.params, ui: { viewer: { ...o.viewer }, tab: o.tab }, aeroDb }),
    [o.params, o.viewer, o.tab, aeroDb],
  );
  const dirty = ready && !sameDesign(working, doc.state);

  const say = (text: string, warn = false) => setMessage({ text, warn });

  const apply = useCallback(
    (s: ProjectState) => {
      o.setParams(s.params);
      o.setViewer({ ...o.viewer, ...s.ui.viewer, color: s.ui.viewer.color as ColorMode });
      o.setTab(s.ui.tab);
      setAeroDb(s.aeroDb ?? null);
      setRev((r) => r + 1);
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [o.setParams, o.setViewer, o.setTab, o.viewer],
  );

  const refreshLibrary = useCallback(async () => {
    if (store.current) setLibrary(await store.current.listProjects());
  }, []);

  // boot: open the KV store and restore the autosaved session (after the designer hydrated its params)
  useEffect(() => {
    if (!o.hydrated || store.current) return;
    let live = true;
    (async () => {
      const kv = await openDefaultKv();
      const s = new ProjectStore(kv, PARSE);
      store.current = s;
      const ws = await s.loadWorkspace().catch(() => null);
      if (!live) return;
      if (ws) {
        setDoc(ws.doc);
        apply(ws.working);
      } else {
        setDoc(newProject(o.params, { name: o.params.name || "Untitled project", ui: { viewer: { ...o.viewer }, tab: o.tab } }));
      }
      setIterateCount(readIterate("cuspis.history.v1").length + readIterate("cuspis.variants.v1").length);
      setLibrary(await s.listProjects());
      setReady(true);
    })();
    return () => {
      live = false;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [o.hydrated]);

  // autosave the live session (debounced)
  useEffect(() => {
    if (!ready || !store.current) return;
    const t = setTimeout(() => {
      store.current?.saveWorkspace({ projectId: doc.id, doc, working, updatedISO: new Date().toISOString() }).catch((e) => console.warn("[project] autosave", e));
    }, 600);
    return () => clearTimeout(t);
  }, [ready, doc, working]);

  const persist = async (d: ProjectDoc) => {
    await store.current?.saveProject(d);
    await refreshLibrary();
  };
  /** History edits are written to the stored project immediately, without saving the working state. */
  const persistHistory = async (d: ProjectDoc) => {
    if (library.some((p) => p.id === d.id)) await persist(d);
  };
  const guard = (what: string) => !dirty || window.confirm(`Discard unsaved changes and ${what}?`);

  const nowISO = () => new Date().toISOString();

  return {
    ready,
    backend: store.current?.kv.kind ?? "…",
    doc,
    working,
    dirty,
    rev,
    aeroDb,
    setAeroDb: setAeroDbTagged,
    message,
    library,
    refreshLibrary,
    iterateCount,
    rename: (name) => setDoc((d) => ({ ...d, name: name.slice(0, 96) })),
    newProject: () => {
      if (!guard("start a new project")) return;
      const d = newProject(DEFAULT_PARAMS, { name: "Untitled project" });
      setDoc(d);
      apply(d.state);
      say("New project");
    },
    save: async () => {
      const d = { ...doc, state: structuredClone(working), modifiedISO: nowISO() };
      setDoc(d);
      await persist(d);
      say(`Saved "${d.name}" (${store.current?.kv.kind})`);
    },
    saveAs: async (name) => {
      const now = nowISO();
      const d = { ...doc, id: newId("p"), name: name.trim() || `${doc.name} copy`, createdISO: now, modifiedISO: now, state: structuredClone(working) };
      setDoc(d);
      await persist(d);
      say(`Saved as "${d.name}"`);
    },
    openFile: async (file) => {
      if (!guard(`open ${file.name}`)) return;
      try {
        const r = parseProject(await file.text(), PARSE);
        setDoc(r.doc);
        apply(r.doc.state);
        say(`Opened ${file.name}${r.warnings.length ? ` · ${r.warnings.join(" · ")}` : ""}`, r.warnings.length > 0);
      } catch (e) {
        say(e instanceof Error ? e.message : String(e), true);
      }
    },
    openStored: async (id) => {
      if (!guard("open another project")) return;
      try {
        const d = await store.current?.loadProject(id);
        if (!d) return say("Project not found", true);
        setDoc(d);
        apply(d.state);
        say(`Opened "${d.name}"`);
      } catch (e) {
        say(e instanceof Error ? e.message : String(e), true);
      }
    },
    deleteStored: async (id) => {
      if (!window.confirm("Delete this saved project from the browser? Download it first if you want to keep it.")) return;
      await store.current?.deleteProject(id);
      await refreshLibrary();
    },
    download: () => {
      const d = { ...doc, state: working, modifiedISO: nowISO() };
      downloadBlob(new Blob([serializeProject(d)], { type: "application/json" }), projectFileName(d.name));
    },
    snapshot: async (msg) => {
      try {
        const r = commitSnapshot(doc, working, msg);
        setDoc(r.doc);
        await persistHistory(r.doc);
        say(`Snapshot "${r.snapshot.message}" · ${r.snapshot.summary}`);
      } catch (e) {
        say(e instanceof Error ? e.message : String(e), true);
      }
    },
    restore: (id) => {
      if (!guard("restore this snapshot")) return;
      const r = restoreSnapshot(doc, id);
      setDoc(r.doc);
      apply(r.state);
      say("Snapshot restored into the working copy (Save to keep it)");
    },
    branch: async (id, name) => {
      const r = branchFromSnapshot(doc, id, name);
      setDoc(r.doc);
      apply(r.state);
      await persistHistory(r.doc);
      say(`On branch "${r.snapshot.branch}"`);
    },
    switchTo: (b) => setDoc((d) => switchBranch(d, b)),
    remove: async (id) => {
      if (!window.confirm("Delete this snapshot? Its children are re-attached to its parent.")) return;
      const d = deleteSnapshot(doc, id);
      setDoc(d);
      await persistHistory(d);
    },
    exportHistory: () => {
      downloadBlob(new Blob([exportHistory(doc)], { type: "application/json" }), projectFileName(doc.name).replace(/\.bowshock\.json$/, ".history.json"));
    },
    importHistory: async (file) => {
      try {
        const { data, warnings } = parseHistoryExport(await file.text(), PARSE);
        const r = importHistory(doc, data, "merge");
        setDoc(r.doc);
        await persistHistory(r.doc);
        say(`Imported ${r.added} snapshot(s)${warnings.length ? ` · ${warnings.join(" · ")}` : ""}`, warnings.length > 0);
      } catch (e) {
        say(e instanceof Error ? e.message : String(e), true);
      }
    },
    importIterate: async () => {
      const r = importIterateLists(doc, { history: readIterate("cuspis.history.v1"), variants: readIterate("cuspis.variants.v1") }, working, sanitizeParams);
      setDoc(r.doc);
      await persistHistory(r.doc);
      say(r.added ? `Imported ${r.added} Iterate entries onto branch "iterate"` : "Nothing new to import from Iterate");
    },
  };
}
