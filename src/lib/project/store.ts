/**
 * [p7-project] Client-side persistence. IndexedDB (database "bowshock-projects", store "kv") is the primary
 * backend because a project with an aero database and a long history can exceed localStorage's ~5 MB quota;
 * when IndexedDB is unavailable (private mode, old browser, SSR) it falls back to localStorage, then memory.
 *
 * Keys
 *   project:<id>       serialized ProjectDoc (the last *saved* version, i.e. what Save wrote)
 *   workspace:current  { projectId, working: ProjectState, doc: ProjectDoc, updatedISO } (autosave of the live session)
 */
import { parseProject, serializeProject, type ParseOptions } from "./io.ts";
import type { ProjectDoc, ProjectState } from "./schema.ts";

export interface KV {
  readonly kind: "indexeddb" | "localstorage" | "memory";
  get(key: string): Promise<string | null>;
  set(key: string, value: string): Promise<void>;
  del(key: string): Promise<void>;
  keys(prefix: string): Promise<string[]>;
}

export function memoryKv(): KV {
  const m = new Map<string, string>();
  return {
    kind: "memory",
    get: async (k) => m.get(k) ?? null,
    set: async (k, v) => void m.set(k, v),
    del: async (k) => void m.delete(k),
    keys: async (p) => [...m.keys()].filter((k) => k.startsWith(p)),
  };
}

const LS_PREFIX = "bowshock.project.";
export function localStorageKv(ls: Storage): KV {
  return {
    kind: "localstorage",
    get: async (k) => ls.getItem(LS_PREFIX + k),
    set: async (k, v) => ls.setItem(LS_PREFIX + k, v),
    del: async (k) => ls.removeItem(LS_PREFIX + k),
    keys: async (p) => {
      const out: string[] = [];
      for (let i = 0; i < ls.length; i++) {
        const k = ls.key(i);
        if (k && k.startsWith(LS_PREFIX + p)) out.push(k.slice(LS_PREFIX.length));
      }
      return out;
    },
  };
}

function req<T>(r: IDBRequest<T>): Promise<T> {
  return new Promise((res, rej) => {
    r.onsuccess = () => res(r.result);
    r.onerror = () => rej(r.error);
  });
}

export async function indexedDbKv(idb: IDBFactory, dbName = "bowshock-projects"): Promise<KV> {
  const open = idb.open(dbName, 1);
  open.onupgradeneeded = () => {
    if (!open.result.objectStoreNames.contains("kv")) open.result.createObjectStore("kv");
  };
  const db = await req(open);
  const tx = (mode: IDBTransactionMode) => db.transaction("kv", mode).objectStore("kv");
  return {
    kind: "indexeddb",
    get: async (k) => ((await req(tx("readonly").get(k))) as string | undefined) ?? null,
    set: async (k, v) => void (await req(tx("readwrite").put(v, k))),
    del: async (k) => void (await req(tx("readwrite").delete(k))),
    keys: async (p) => ((await req(tx("readonly").getAllKeys())) as IDBValidKey[]).map(String).filter((k) => k.startsWith(p)),
  };
}

/** Best available backend in this environment. */
export async function openDefaultKv(): Promise<KV> {
  const g = globalThis as { indexedDB?: IDBFactory; localStorage?: Storage };
  if (g.indexedDB) {
    try {
      return await indexedDbKv(g.indexedDB);
    } catch {
      /* fall through */
    }
  }
  try {
    if (g.localStorage) {
      g.localStorage.setItem("bowshock.project.__probe", "1");
      g.localStorage.removeItem("bowshock.project.__probe");
      return localStorageKv(g.localStorage);
    }
  } catch {
    /* fall through */
  }
  return memoryKv();
}

export interface ProjectSummary {
  id: string;
  name: string;
  modifiedISO: string;
  snapshots: number;
}

export interface Workspace {
  projectId: string;
  /** Last saved doc (+ history, which is saved immediately on snapshot/delete/restore). */
  doc: ProjectDoc;
  /** Live (possibly unsaved) state. */
  working: ProjectState;
  updatedISO: string;
}

export class ProjectStore {
  readonly kv: KV;
  readonly parseOpts: ParseOptions;
  constructor(kv: KV, parseOpts: ParseOptions = {}) {
    this.kv = kv;
    this.parseOpts = parseOpts;
  }

  async saveProject(doc: ProjectDoc): Promise<void> {
    await this.kv.set(`project:${doc.id}`, serializeProject(doc));
  }

  async loadProject(id: string): Promise<ProjectDoc | null> {
    const raw = await this.kv.get(`project:${id}`);
    return raw ? parseProject(raw, this.parseOpts).doc : null;
  }

  async deleteProject(id: string): Promise<void> {
    await this.kv.del(`project:${id}`);
  }

  async listProjects(): Promise<ProjectSummary[]> {
    const out: ProjectSummary[] = [];
    for (const k of await this.kv.keys("project:")) {
      try {
        const raw = await this.kv.get(k);
        if (!raw) continue;
        const d = JSON.parse(raw) as ProjectDoc;
        out.push({ id: d.id, name: d.name, modifiedISO: d.modifiedISO, snapshots: d.history?.snapshots?.length ?? 0 });
      } catch {
        /* skip an unreadable record */
      }
    }
    return out.sort((a, b) => (a.modifiedISO < b.modifiedISO ? 1 : -1));
  }

  async saveWorkspace(ws: Workspace): Promise<void> {
    await this.kv.set("workspace:current", JSON.stringify({ ...ws, doc: JSON.parse(serializeProject(ws.doc)) }));
  }

  /** Restores the autosaved session. A corrupt record is dropped (returns null) rather than breaking startup. */
  async loadWorkspace(): Promise<Workspace | null> {
    const raw = await this.kv.get("workspace:current");
    if (!raw) return null;
    try {
      const w = JSON.parse(raw) as { projectId?: unknown; doc?: unknown; working?: unknown; updatedISO?: unknown };
      const doc = parseProject(JSON.stringify(w.doc), this.parseOpts).doc;
      const working = parseProject(JSON.stringify({ ...(w.doc as object), state: w.working, history: { snapshots: [] } }), this.parseOpts).doc.state;
      return { projectId: doc.id, doc, working, updatedISO: typeof w.updatedISO === "string" ? w.updatedISO : doc.modifiedISO };
    } catch {
      await this.kv.del("workspace:current");
      return null;
    }
  }
}
