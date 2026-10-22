/**
 * [p7-project] Projects menu: header chip (name + unsaved indicator) and a HUD drawer with
 * Project (New / Save / Save As / Open / Download / library), History (Snapshot, list, Restore, Branch,
 * Delete, Export / Import) and Compare (2–4 versions).
 */
import { useEffect, useRef, useState } from "react";
import { FolderOpen, GitBranch, History, Save, X } from "lucide-react";
import { Button } from "@/components/ui/button";
import { Seg } from "@/components/fields";
import { CompareView } from "@/components/project-compare";
import type { ProjectSession } from "@/components/use-project";
import { aeroDbStale, listSnapshots } from "@/lib/project/index";

type Pane = "project" | "history" | "compare";

function when(iso: string) {
  const d = new Date(iso);
  return Number.isNaN(d.getTime()) ? iso : d.toLocaleString(undefined, { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" });
}

export function ProjectsMenu({ project }: { project: ProjectSession }) {
  const [open, setOpen] = useState(false);
  const [pane, setPane] = useState<Pane>("project");
  const [msg, setMsg] = useState("");
  const fileRef = useRef<HTMLInputElement>(null);
  const histRef = useRef<HTMLInputElement>(null);
  const { doc, dirty } = project;

  useEffect(() => {
    if (!open) return;
    const onKey = (e: KeyboardEvent) => e.key === "Escape" && setOpen(false);
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [open]);

  // Ctrl/Cmd+S saves the project
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "s") {
        e.preventDefault();
        void project.save();
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [project]);

  const snaps = listSnapshots(doc);
  const head = doc.history.head;

  return (
    <>
      <button
        type="button"
        data-testid="projects-open"
        onClick={() => setOpen(true)}
        className="hud-chip-info flex max-w-[16rem] items-center gap-2 px-2.5 py-1.5 text-left"
        title={dirty ? "Unsaved changes" : "All changes saved"}
      >
        <FolderOpen className="size-4 shrink-0 text-accent" />
        <span className="min-w-0">
          <span className="hud-label block !text-[9px] !text-muted">Project</span>
          <span className="block truncate text-xs text-fg">{doc.name}</span>
        </span>
        <span
          data-testid="unsaved-indicator"
          data-dirty={String(dirty)}
          aria-label={dirty ? "Unsaved changes" : "Saved"}
          className={`ml-1 size-2 shrink-0 rounded-full ${dirty ? "bg-warn shadow-[0_0_6px_var(--color-warn)]" : "bg-surface-3 shadow-[inset_0_0_0_1px_var(--color-line)]"}`}
        />
      </button>

      {open ? (
        <div className="fixed inset-0 z-50 flex justify-end bg-black/55" onClick={() => setOpen(false)}>
          <div
            role="dialog"
            aria-label="Projects"
            data-testid="projects-panel"
            className="hud-panel flex h-full w-full max-w-[880px] flex-col overflow-hidden bg-surface-2"
            onClick={(e) => e.stopPropagation()}
          >
            <div className="flex items-center justify-between gap-3 border-b border-line px-4 py-3">
              <div className="min-w-0">
                <p className="hud-wordmark text-base">PROJECTS</p>
                <p className="hud-label !text-muted">
                  {dirty ? <span className="text-warn">● Unsaved changes</span> : "Saved"} · autosave {project.backend} · branch {doc.history.branch}
                </p>
              </div>
              <Button variant="ghost" size="icon" aria-label="Close" onClick={() => setOpen(false)}>
                <X />
              </Button>
            </div>
            <div className="border-b border-line px-4 py-2">
              <Seg value={pane} onChange={setPane} options={[{ id: "project", label: "Project" }, { id: "history", label: `History (${snaps.length})` }, { id: "compare", label: "Compare" }]} />
            </div>
            {project.message ? (
              <p data-testid="project-message" className={`border-b border-line px-4 py-1.5 text-[11px] leading-snug ${project.message.warn ? "text-warn" : "text-muted"}`}>
                {project.message.text}
              </p>
            ) : null}
            <input ref={fileRef} type="file" accept=".json,.bowshock.json,application/json" className="hidden" onChange={(e) => { const f = e.target.files?.[0]; if (f) void project.openFile(f); e.target.value = ""; }} />
            <input ref={histRef} type="file" accept=".json,application/json" className="hidden" onChange={(e) => { const f = e.target.files?.[0]; if (f) void project.importHistory(f); e.target.value = ""; }} />

            <div className="min-h-0 flex-1 overflow-y-auto p-4">
              {pane === "project" ? (
                <div className="space-y-4">
                  <label className="block">
                    <span className="hud-label mb-1 block !text-muted">Name</span>
                    <input data-testid="project-name" value={doc.name} onChange={(e) => project.rename(e.target.value)} className="h-9 w-full bg-surface px-2 text-sm text-fg" />
                  </label>
                  <div className="grid grid-cols-2 gap-2 sm:grid-cols-3">
                    <Button size="sm" variant="secondary" onClick={project.newProject}>New</Button>
                    <Button size="sm" onClick={() => void project.save()}><Save /> Save</Button>
                    <Button size="sm" variant="secondary" onClick={() => { const n = window.prompt("Save as", `${doc.name} copy`); if (n !== null) void project.saveAs(n); }}>Save As…</Button>
                    <Button size="sm" variant="secondary" onClick={() => fileRef.current?.click()}><FolderOpen /> Open file…</Button>
                    <Button size="sm" variant="secondary" onClick={project.download}>Download .bowshock.json</Button>
                    <Button size="sm" variant="secondary" onClick={() => setPane("history")}><History /> Snapshot…</Button>
                  </div>
                  <p className="text-[11px] leading-relaxed text-subtle">
                    A project holds the design + flight/analysis parameters, the view, the Iterate aero-database spec and results, and the version
                    history. Save writes it to this browser ({project.backend}); Download gives a portable <span className="text-fg">.bowshock.json</span>.
                    The live session autosaves continuously.
                  </p>
                  <div>
                    <p className="hud-label hud-label-tick mb-2">Saved in this browser</p>
                    {project.library.length === 0 ? <p className="text-xs text-muted">Nothing saved yet.</p> : null}
                    <div className="grid gap-1.5">
                      {project.library.map((p) => (
                        <div key={p.id} className="hud-card flex items-center gap-2 rounded-md px-3 py-2" aria-pressed={p.id === doc.id}>
                          <button type="button" className="hud-plain min-w-0 flex-1 text-left" onClick={() => void project.openStored(p.id)}>
                            <span className="hud-card-title block truncate text-sm">{p.name}</span>
                            <span className="block text-[11px] text-muted">{when(p.modifiedISO)} · {p.snapshots} snapshot(s)</span>
                          </button>
                          <Button size="sm" variant="ghost" onClick={() => void project.deleteStored(p.id)}>Delete</Button>
                        </div>
                      ))}
                    </div>
                  </div>
                </div>
              ) : null}

              {pane === "history" ? (
                <div className="space-y-3">
                  <div className="flex gap-2">
                    <input
                      value={msg}
                      placeholder="Snapshot message"
                      onChange={(e) => setMsg(e.target.value)}
                      onKeyDown={(e) => { if (e.key === "Enter") { void project.snapshot(msg); setMsg(""); } }}
                      className="h-9 min-w-0 flex-1 bg-surface px-2 text-sm text-fg"
                    />
                    <Button size="sm" data-testid="snapshot-btn" onClick={() => { void project.snapshot(msg); setMsg(""); }}>Snapshot</Button>
                  </div>
                  <div className="flex flex-wrap items-center gap-2">
                    <span className="hud-label !text-muted">Branch</span>
                    {doc.history.branches.map((b) => (
                      <button key={b} type="button" aria-pressed={b === doc.history.branch} className="hud-toggle hud-toggle--sm rounded-sm" onClick={() => project.switchTo(b)}>
                        {b}
                      </button>
                    ))}
                    <span className="flex-1" />
                    <Button size="sm" variant="ghost" onClick={project.exportHistory}>Export history</Button>
                    <Button size="sm" variant="ghost" onClick={() => histRef.current?.click()}>Import history…</Button>
                    {project.iterateCount ? (
                      <Button size="sm" variant="ghost" onClick={() => void project.importIterate()} title="Iterate → Compare keeps its own variant/history lists in this browser">
                        Import Iterate lists ({project.iterateCount})
                      </Button>
                    ) : null}
                  </div>
                  {snaps.length === 0 ? <p className="text-xs text-muted">No snapshots yet. A snapshot freezes the current design with a message.</p> : null}
                  <ol className="grid gap-1.5" data-testid="history-list">
                    {snaps.map((s) => (
                      <li key={s.id} className="hud-card rounded-md px-3 py-2" aria-pressed={s.id === head}>
                        <div className="flex items-start gap-2">
                          <div className="min-w-0 flex-1">
                            <p className="hud-card-title truncate text-sm">
                              {s.message}
                              {s.id === head ? <span className="hud-label ml-2 !text-accent">HEAD</span> : null}
                            </p>
                            <p className="text-[11px] text-muted">
                              {when(s.createdISO)} · <GitBranch className="inline size-3" /> {s.branch}
                              {s.state.aeroDb?.database ? (aeroDbStale(s.state.aeroDb, s.state.params) ? " · aero DB (stale)" : " · aero DB") : ""}
                            </p>
                            <p className="truncate font-mono text-[11px] text-subtle" title={s.summary}>{s.summary}</p>
                          </div>
                          <div className="flex shrink-0 gap-1">
                            <Button size="sm" variant="secondary" onClick={() => project.restore(s.id)}>Restore</Button>
                            <Button size="sm" variant="ghost" onClick={() => { const n = window.prompt("New branch name", `${s.branch}-alt`); if (n) void project.branch(s.id, n); }}>Branch</Button>
                            <Button size="sm" variant="ghost" onClick={() => void project.remove(s.id)}>Delete</Button>
                          </div>
                        </div>
                      </li>
                    ))}
                  </ol>
                </div>
              ) : null}

              {pane === "compare" ? <CompareView project={project} /> : null}
            </div>
          </div>
        </div>
      ) : null}
    </>
  );
}
