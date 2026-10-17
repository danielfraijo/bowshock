/**
 * Data tab: live data package for the current Geom-tab configuration.
 * Recomputes when (and only when) `datapackKey(params, opts)` changes: debounced, run in slices so the
 * main thread is never held for more than one panelAero call, and cancelled if the design changes
 * again mid-compute.
 */
import { useEffect, useMemo, useRef, useState } from "react";
import { Button } from "@/components/ui/button";
import { Stat } from "@/components/fields";
import {
  DEFAULT_OPTIONS,
  computeDataPackAsync,
  datapackCached,
  datapackKey,
  dataDictionary,
  packFiles,
  packToJson,
  tableToCsv,
  formatCell,
  type DataPack,
  type DataPackOptions,
} from "@/lib/waverider/datapack";
import { buildZip, utf8 } from "@/lib/waverider/zip";
import { downloadBlob } from "@/lib/utils";
import type { DesignParams } from "@/lib/waverider/types";

const PARAM_CHOICES: (keyof DesignParams)[] = ["shockDeg", "coneDeg", "span", "height", "length", "leRadius", "mach", "flightMach", "altKm", "twK", "cgFrac", "alphaDeg"];
const DEBOUNCE_MS = 350;
const PREVIEW_ROWS = 60;

function Num({ label, value, onChange, step = 1 }: { label: string; value: number; onChange: (v: number) => void; step?: number }) {
  return (
    <label className="flex items-center justify-between gap-2 text-xs text-muted">
      <span>{label}</span>
      <input
        type="number"
        value={value}
        step={step}
        onChange={(e) => {
          const v = Number(e.target.value);
          if (Number.isFinite(v)) onChange(v);
        }}
        className="h-8 w-20 rounded-sm bg-surface-2 px-2 text-right font-mono text-xs text-fg shadow-[0_0_0_1px_rgba(255,255,255,0.08)]"
      />
    </label>
  );
}

export function DataPanel({ params, paused }: { params: DesignParams; paused: boolean }) {
  const [opts, setOpts] = useState<DataPackOptions>({
    alphaSweep: { ...DEFAULT_OPTIONS.alphaSweep },
    machSweep: { ...DEFAULT_OPTIONS.machSweep },
    carpet: { ...DEFAULT_OPTIONS.carpet },
    stations: DEFAULT_OPTIONS.stations,
    panels: true,
  });
  const key = useMemo(() => datapackKey(params, opts), [params, opts]);
  const [pack, setPack] = useState<DataPack | null>(() => datapackCached(params, opts));
  const [busy, setBusy] = useState<{ frac: number; label: string } | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [tableId, setTableId] = useState("results_design_point");
  const latest = useRef({ params, opts });
  latest.current = { params, opts };

  useEffect(() => {
    if (paused) return; // do not compete with a slider drag; resumes on release
    const hit = datapackCached(latest.current.params, latest.current.opts);
    if (hit) {
      setPack(hit);
      setBusy(null);
      return;
    }
    const ctl = new AbortController();
    const timer = window.setTimeout(() => {
      setBusy({ frac: 0, label: "queued" });
      setErr(null);
      computeDataPackAsync(latest.current.params, latest.current.opts, {
        signal: ctl.signal,
        onProgress: (p) => !ctl.signal.aborted && setBusy(p),
      })
        .then((p) => {
          if (ctl.signal.aborted) return;
          setPack(p);
          setBusy(null);
        })
        .catch((e: unknown) => {
          if (ctl.signal.aborted) return;
          setErr(e instanceof Error ? e.message : "failed");
          setBusy(null);
        });
    }, DEBOUNCE_MS);
    return () => {
      ctl.abort();
      window.clearTimeout(timer);
    };
  }, [key, paused]);

  const stale = !pack || pack.key !== key;
  const table = pack?.tables.find((t) => t.id === tableId) ?? pack?.tables[0];
  const designFails = pack ? pack.tables.find((t) => t.id === "validation")?.rows.filter((r) => r[1] === "design" && r[9] === false).length ?? 0 : 0;

  const dl = {
    csv: () => table && downloadBlob(new Blob([tableToCsv(table)], { type: "text/csv" }), `${params.name}_${table.id}.csv`),
    json: () => pack && downloadBlob(new Blob([packToJson(pack, true)], { type: "application/json" }), `${params.name}_datapack.json`),
    zip: () => {
      if (!pack) return;
      const files = packFiles(pack, "data").map((f) => ({ name: f.name, data: utf8(f.content) }));
      downloadBlob(buildZip(files), `${params.name}_datapack.zip`);
    },
    readme: () => pack && downloadBlob(new Blob([dataDictionary(pack)], { type: "text/plain" }), `${params.name}_datapack_README.txt`),
  };

  const set = (patch: Partial<DataPackOptions>) => setOpts((o) => ({ ...o, ...patch }));

  return (
    <div className="space-y-3">
      <div>
        <Stat k="Data pack" v={busy ? `computing ${Math.round(busy.frac * 100)}% · ${busy.label}` : stale ? (paused ? "paused while dragging" : "waiting") : "up to date"} ok={busy || stale ? undefined : true} />
        {pack ? <Stat k="Input hash" v={pack.key.slice(0, 15)} /> : null}
        {pack ? <Stat k="Validation" v={`${pack.validation.pass}/${pack.validation.n} pass${designFails ? ` · ${designFails} design check(s) FAIL` : ""}`} ok={pack.validation.ok} /> : null}
        {pack ? <Stat k="Compute time" v={`${Math.round(pack.elapsedMs)} ms`} /> : null}
        {err ? <Stat k="Error" v={err} ok={false} /> : null}
      </div>

      <details className="hud-panel hud-panel--flat p-2">
        <summary className="cursor-pointer text-xs text-muted">Sweeps</summary>
        <div className="mt-2 space-y-1.5">
          <Num label="α min °" value={opts.alphaSweep!.min} onChange={(min) => set({ alphaSweep: { ...opts.alphaSweep!, min } })} />
          <Num label="α max °" value={opts.alphaSweep!.max} onChange={(max) => set({ alphaSweep: { ...opts.alphaSweep!, max } })} />
          <Num label="α points" value={opts.alphaSweep!.n} onChange={(n) => set({ alphaSweep: { ...opts.alphaSweep!, n: Math.max(2, Math.min(41, Math.round(n))) } })} />
          <Num label="Mach min" value={opts.machSweep!.min} step={0.5} onChange={(min) => set({ machSweep: { ...opts.machSweep!, min: Math.max(1.5, min) } })} />
          <Num label="Mach max" value={opts.machSweep!.max} step={0.5} onChange={(max) => set({ machSweep: { ...opts.machSweep!, max } })} />
          <Num label="Mach points" value={opts.machSweep!.n} onChange={(n) => set({ machSweep: { ...opts.machSweep!, n: Math.max(2, Math.min(25, Math.round(n))) } })} />
          <Num label="Stations" value={opts.stations ?? 20} onChange={(n) => set({ stations: Math.max(4, Math.min(80, Math.round(n))) })} />
          <label className="flex items-center justify-between gap-2 text-xs text-muted">
            <span>Parametric sweep</span>
            <select
              className="h-8 rounded-sm bg-surface-2 px-1 text-xs text-fg"
              value={opts.parametric?.variable ?? ""}
              onChange={(e) => {
                const variable = e.target.value as keyof DesignParams;
                if (!variable) return set({ parametric: null });
                const cur = Number(params[variable]);
                const span = Math.abs(cur) > 1e-9 ? Math.abs(cur) * 0.3 : 1;
                set({ parametric: { variable, min: +(cur - span).toPrecision(4), max: +(cur + span).toPrecision(4), n: 7 } });
              }}
            >
              <option value="">off</option>
              {PARAM_CHOICES.map((k) => (
                <option key={k} value={k}>
                  {k}
                </option>
              ))}
            </select>
          </label>
          {opts.parametric ? (
            <>
              <Num label="  from" value={opts.parametric.min} step={0.1} onChange={(min) => set({ parametric: { ...opts.parametric!, min } })} />
              <Num label="  to" value={opts.parametric.max} step={0.1} onChange={(max) => set({ parametric: { ...opts.parametric!, max } })} />
              <Num label="  points" value={opts.parametric.n} onChange={(n) => set({ parametric: { ...opts.parametric!, n: Math.max(2, Math.min(21, Math.round(n))) } })} />
            </>
          ) : null}
          <label className="flex items-center justify-between text-xs text-muted">
            <span>Include per-panel table</span>
            <input type="checkbox" checked={opts.panels ?? true} onChange={(e) => set({ panels: e.target.checked })} />
          </label>
        </div>
      </details>

      <div className="flex flex-wrap gap-1.5">
        <Button size="sm" variant="secondary" disabled={!pack} onClick={dl.zip}>
          ZIP (all CSV + JSON)
        </Button>
        <Button size="sm" variant="outline" disabled={!pack} onClick={dl.json}>
          JSON
        </Button>
        <Button size="sm" variant="outline" disabled={!table} onClick={dl.csv}>
          CSV (this table)
        </Button>
        <Button size="sm" variant="ghost" disabled={!pack} onClick={dl.readme}>
          Data dictionary
        </Button>
      </div>

      {pack ? (
        <div className="flex flex-wrap gap-1">
          {pack.tables.map((t) => (
            <button
              key={t.id}
              type="button"
              onClick={() => setTableId(t.id)}
              aria-pressed={t.id === table?.id}
              className={`rounded-sm px-2 py-1 text-[11px] ${t.id === table?.id ? "bg-accent text-accent-fg" : "bg-surface-2 text-muted hover:text-fg"}`}
            >
              {t.id.replace(/_/g, " ")}
            </button>
          ))}
        </div>
      ) : null}

      {table ? (
        <div>
          <p className="text-xs text-fg">{table.title}</p>
          <p className="mb-2 text-[11px] leading-snug text-subtle">{table.description}</p>
          <div className={`max-h-[46vh] overflow-auto rounded-sm border border-border/80 ${stale ? "opacity-60" : ""}`}>
            <table className="w-full border-collapse font-mono text-[10px]">
              <thead className="sticky top-0 bg-surface-2">
                <tr>
                  {table.columns.map((c) => (
                    <th key={c.key} title={`${c.desc} [${c.unit}]`} className="whitespace-nowrap px-1.5 py-1 text-left font-medium text-muted">
                      {c.key}
                    </th>
                  ))}
                </tr>
              </thead>
              <tbody>
                {table.rows.slice(0, PREVIEW_ROWS).map((r, i) => (
                  <tr key={i} className="border-t border-border/60">
                    {r.map((c, j) => {
                      const text = formatCell(c);
                      const isPass = table.columns[j]?.key === "pass";
                      return (
                        <td key={j} className={`max-w-[18rem] truncate px-1.5 py-0.5 ${isPass ? (c ? "text-ok" : "text-warn") : "text-fg"}`} title={text}>
                          {text.length > 14 && typeof c === "number" ? text.slice(0, 12) : text}
                        </td>
                      );
                    })}
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
          {table.rows.length > PREVIEW_ROWS ? (
            <p className="mt-1 text-[11px] text-subtle">
              Showing {PREVIEW_ROWS} of {table.rows.length} rows. Download the CSV for all of them.
            </p>
          ) : null}
        </div>
      ) : (
        <p className="text-xs text-muted">{busy ? "Computing…" : "No data yet."}</p>
      )}
    </div>
  );
}
