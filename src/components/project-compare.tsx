/** [p7-project] Variant compare: 2–4 versions side by side (param diff, on-demand metrics, aero-DB α overlay). */
import { useMemo, useState } from "react";
import { CartesianGrid, Legend, Line, LineChart, ResponsiveContainer, Tooltip, XAxis, YAxis } from "recharts";
import { Button } from "@/components/ui/button";
import { Seg } from "@/components/fields";
import { evaluateVariant } from "@/components/project-eval";
import { fmt } from "@/lib/waverider/math";
import { buildCompare, compareCandidates, type CompareData } from "@/lib/project/index";
import type { ProjectSession } from "@/components/use-project";

const SERIES = ["#ff1744", "#ffffff", "#ff8fa0", "#ff4d6d"];

function show(v: unknown): string {
  if (typeof v === "number") return fmt(v, Math.abs(v) >= 100 ? 1 : 4);
  if (typeof v === "boolean") return v ? "on" : "off";
  if (v === undefined || v === null) return "—";
  if (typeof v === "string") return v;
  return JSON.stringify(v).slice(0, 40);
}

export function CompareView({ project }: { project: ProjectSession }) {
  const candidates = useMemo(() => compareCandidates(project.doc, project.working), [project.doc, project.working]);
  const [picked, setPicked] = useState<string[]>([]);
  const [data, setData] = useState<CompareData | null>(null);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const [coef, setCoef] = useState<"cl" | "cd" | "ld">("ld");
  const sel = picked.map((id) => candidates.find((c) => c.id === id)).filter((c): c is NonNullable<typeof c> => !!c);

  const toggle = (id: string) =>
    setPicked((p) => (p.includes(id) ? p.filter((x) => x !== id) : p.length >= 4 ? p : [...p, id]));

  const run = (withMetrics: boolean) => {
    setErr(null);
    setBusy(withMetrics);
    // yield a frame so the busy state paints before the (synchronous) panel evaluations
    setTimeout(() => {
      try {
        setData(buildCompare(sel, withMetrics ? { evaluate: evaluateVariant } : {}));
      } catch (e) {
        setErr(e instanceof Error ? e.message : String(e));
      } finally {
        setBusy(false);
      }
    }, 30);
  };

  return (
    <div className="space-y-4" data-testid="project-compare">
      <div>
        <p className="hud-label hud-label-tick mb-2">Pick 2–4 versions</p>
        {candidates.length < 2 ? <p className="text-xs text-muted">Take at least one snapshot to compare against the working copy.</p> : null}
        <div className="grid max-h-44 gap-1 overflow-y-auto pr-1 sm:grid-cols-2">
          {candidates.map((c) => {
            const i = picked.indexOf(c.id);
            return (
              <button key={c.id} type="button" aria-pressed={i >= 0} onClick={() => toggle(c.id)} className="hud-card flex items-center gap-2 rounded-md px-2 py-1.5 text-left">
                <span className="inline-block size-2.5 shrink-0" style={{ background: i >= 0 ? SERIES[i] : "transparent", boxShadow: "inset 0 0 0 1px var(--color-line)" }} />
                <span className="truncate text-xs text-fg">{c.label}</span>
              </button>
            );
          })}
        </div>
        <div className="mt-2 flex flex-wrap gap-2">
          <Button size="sm" variant="secondary" disabled={sel.length < 2} onClick={() => run(false)}>
            Diff
          </Button>
          <Button size="sm" disabled={sel.length < 2 || busy} onClick={() => run(true)}>
            {busy ? "Evaluating…" : "Diff + metrics"}
          </Button>
          <span className="self-center text-[11px] text-subtle">Metrics run the panel method once per distinct design, in this browser.</span>
        </div>
        {err ? <p className="mt-2 text-xs text-warn">{err}</p> : null}
      </div>

      {data ? (
        <>
          <div className="hud-panel overflow-x-auto px-3 py-2">
            <table className="w-full text-[11px]">
              <thead>
                <tr className="text-left text-muted">
                  <th className="py-1 pr-2 font-display">Param</th>
                  {data.entries.map((e, i) => (
                    <th key={e.id} className="py-1 pr-2 font-display" style={{ color: SERIES[i] }}>
                      {e.label}
                    </th>
                  ))}
                </tr>
              </thead>
              <tbody className="font-mono">
                {data.params.length === 0 ? (
                  <tr>
                    <td colSpan={data.entries.length + 1} className="py-1 text-muted">
                      Identical design parameters.
                    </td>
                  </tr>
                ) : null}
                {data.params.map((r) => (
                  <tr key={r.path}>
                    <td className="py-0.5 pr-2 text-muted">{r.path.replace(/^params\./, "")}</td>
                    {r.values.map((v, i) => (
                      <td key={i} className={`py-0.5 pr-2 ${show(v) !== show(r.values[0]) ? "text-accent" : "text-fg"}`}>
                        {show(v)}
                      </td>
                    ))}
                  </tr>
                ))}
              </tbody>
            </table>
          </div>

          {data.metrics ? (
            <div className="hud-panel overflow-x-auto px-3 py-2">
              <p className="hud-label mb-1">Key metrics · panel method at each version's flight condition</p>
              <table className="w-full text-[11px]">
                <tbody className="font-mono">
                  {data.metrics.map((m) => (
                    <tr key={m.key}>
                      <td className="py-0.5 pr-2 font-display text-muted">
                        {m.label}
                        {m.unit ? <span className="text-subtle"> {m.unit}</span> : null}
                      </td>
                      {m.values.map((v, i) => (
                        <td key={i} className={`py-0.5 pr-2 tabular-nums ${m.best === i ? "text-accent" : "text-fg"}`}>
                          {v === null ? "—" : fmt(v, m.digits)}
                          {m.best === i ? " ▲" : ""}
                        </td>
                      ))}
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          ) : null}

          {data.curves ? (
            <div className="hud-panel px-3 py-2">
              <div className="mb-2 flex items-center justify-between gap-2">
                <p className="hud-label">Aero DB · vs α · M≈{fmt(data.curves.mach, 2)} · β≈0 δ≈0</p>
                <div className="w-40">
                  <Seg value={coef} onChange={setCoef} options={[{ id: "cl", label: "CL" }, { id: "cd", label: "CD" }, { id: "ld", label: "L/D" }]} />
                </div>
              </div>
              <div className="h-56">
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart data={data.curves.rows} margin={{ top: 4, right: 8, bottom: 4, left: -8 }}>
                    <CartesianGrid stroke="rgba(255,23,68,0.16)" strokeDasharray="2 4" />
                    <XAxis dataKey="alpha" type="number" domain={["dataMin", "dataMax"]} tick={{ fill: "#ffd4da", fontSize: 10 }} unit="°" />
                    <YAxis tick={{ fill: "#ffd4da", fontSize: 10 }} width={48} />
                    <Tooltip contentStyle={{ background: "#080003", border: "1px solid #ff1744", fontSize: 11, color: "#ffffff" }} />
                    <Legend wrapperStyle={{ fontSize: 10 }} />
                    {data.curves.series.map((s, i) => {
                      const color = SERIES[data.entries.findIndex((e) => e.id === s.entryId)] ?? SERIES[i];
                      return <Line key={s.entryId} dataKey={`${coef}${i}`} name={`${s.label} (M${fmt(s.mach, 2)}${s.stale ? ", stale DB" : ""})`} strokeDasharray={s.stale ? "4 3" : undefined} stroke={color} dot={{ r: 2 }} strokeWidth={1.6} connectNulls isAnimationActive={false} />;
                    })}
                  </LineChart>
                </ResponsiveContainer>
              </div>
              {data.curves.stale.length ? <p className="mt-1 text-[11px] text-warn">Stale (dashed): {data.curves.stale.join(", ")} carry a database generated for a different design. Rebuild it in Iterate.</p> : null}
              {data.curves.missing.length ? <p className="mt-1 text-[11px] text-subtle">No aero DB stored: {data.curves.missing.join(", ")}. Run Iterate → Aero DB, then snapshot.</p> : null}
            </div>
          ) : (
            <p className="text-[11px] text-subtle">None of these versions has an aero database stored. Run Iterate → Aero DB, then snapshot, to overlay CL/CD/L/D vs α.</p>
          )}
        </>
      ) : null}
    </div>
  );
}
