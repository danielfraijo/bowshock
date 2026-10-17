/**
 * DATCOM lab tab: config editor, presets, run, tables, plots, deck paste, downloads.
 * Pure client-side — nothing is uploaded.
 */
import { useMemo, useState } from "react";
import { CartesianGrid, Legend, Line, LineChart, ResponsiveContainer, Tooltip, XAxis, YAxis } from "recharts";
import { Stat } from "@/components/fields";
import { Button } from "@/components/ui/button";
import {
  PRESETS,
  cloneConfig,
  fromWaveriderDesign,
  type Preset,
} from "@/lib/waverider/datcom/presets";
import { runCases } from "@/lib/waverider/datcom/model";
import {
  downloadName,
  format,
  type OutputFormat,
} from "@/lib/waverider/datcom/output";
import { parseDeck, generateMissileDeck } from "@/lib/waverider/datcom/deck";
import type { FlightSpec, RunResult, VehicleConfig } from "@/lib/waverider/datcom/types";
import type { DesignParams } from "@/lib/waverider/types";
import { fmt } from "@/lib/waverider/math";
import { downloadBlob } from "@/lib/utils";

type PlotX = "alpha" | "mach";
type PlotY = "CN" | "CA" | "Cm" | "CL" | "CD" | "CY" | "Cl" | "Cn" | "LD";

const YOPTS: { id: PlotY; label: string }[] = [
  { id: "CN", label: "CN" },
  { id: "CA", label: "CA" },
  { id: "Cm", label: "Cm" },
  { id: "CL", label: "CL" },
  { id: "CD", label: "CD" },
  { id: "CY", label: "CY" },
  { id: "Cl", label: "Cl" },
  { id: "Cn", label: "Cn" },
  { id: "LD", label: "L/D" },
];

function downloadText(name: string, text: string) {
  downloadBlob(new Blob([text], { type: "text/plain;charset=utf-8" }), name);
}

function Num({
  label,
  value,
  onChange,
  step = 0.01,
  min,
  max,
}: {
  label: string;
  value: number;
  onChange: (v: number) => void;
  step?: number;
  min?: number;
  max?: number;
}) {
  return (
    <label className="flex items-center justify-between gap-2 py-0.5">
      <span className="text-[11px] text-muted">{label}</span>
      <input
        type="number"
        className="h-7 w-[5.5rem] rounded-sm border-0 bg-surface-2 px-1.5 text-right font-mono text-xs tabular-nums text-fg outline-none focus:bg-surface"
        value={Number.isFinite(value) ? value : 0}
        step={step}
        min={min}
        max={max}
        onChange={(e) => {
          const n = parseFloat(e.target.value);
          if (Number.isFinite(n)) onChange(n);
        }}
      />
    </label>
  );
}

export function DatcomBlock({ params }: { params: DesignParams }) {
  const [presetId, setPresetId] = useState("finner");
  const [config, setConfig] = useState<VehicleConfig>(() => cloneConfig(PRESETS[0].config));
  const [flight, setFlight] = useState<FlightSpec>(() => ({ ...PRESETS[0].flight }));
  const [result, setResult] = useState<RunResult | null>(null);
  const [busy, setBusy] = useState(false);
  const [deckText, setDeckText] = useState("");
  const [deckWarn, setDeckWarn] = useState<string[]>([]);
  const [plotX, setPlotX] = useState<PlotX>("alpha");
  const [plotY, setPlotY] = useState<PlotY>("CN");
  const [machFocus, setMachFocus] = useState(0);
  const [outFmt, setOutFmt] = useState<OutputFormat>("missile");
  const [err, setErr] = useState<string | null>(null);

  function loadPreset(id: string) {
    const p = PRESETS.find((x: Preset) => x.id === id);
    if (!p) return;
    setPresetId(id);
    setConfig(cloneConfig(p.config));
    setFlight({ ...p.flight });
    setResult(null);
    setDeckWarn([]);
    setErr(null);
  }

  function loadFromWaverider() {
    const cfg = fromWaveriderDesign({
      length: params.length,
      span: params.span,
      height: params.height,
      coneDeg: params.coneDeg,
      elevonDeg: params.elevonDeg,
      finHeight: params.finHeight,
      fins: params.fins,
      cgFrac: params.cgFrac,
      name: `waverider:${params.family}`,
    });
    setPresetId("waverider");
    setConfig(cfg);
    setFlight({
      machs: [params.flightMach, Math.max(params.flightMach * 0.7, 1.5), Math.min(params.flightMach * 1.3, 12)],
      alphas: [-4, -2, 0, 2, 4, 6, 8, 10, 12],
      altKm: params.altKm,
      beta: params.betaDeg,
    });
    setResult(null);
  }

  function run() {
    setBusy(true);
    setErr(null);
    try {
      const t0 = performance.now();
      const r = runCases(config, flight);
      (r as RunResult & { _ms?: number })._ms = performance.now() - t0;
      setResult(r);
      setMachFocus(0);
    } catch (e) {
      setErr((e as Error).message);
      setResult(null);
    } finally {
      setBusy(false);
    }
  }

  function pasteDeck() {
    try {
      const p = parseDeck(deckText);
      setConfig(p.config);
      setFlight(p.flight);
      setDeckWarn([
        ...p.warnings,
        ...p.unsupported.map((u) => `UNSUPPORTED: ${u}`),
        `supported: ${p.supported.join(", ") || "(none)"}`,
      ]);
      setPresetId("deck");
      setResult(null);
      setErr(null);
    } catch (e) {
      setErr(`deck parse: ${(e as Error).message}`);
    }
  }

  const chartData = useMemo(() => {
    if (!result) return [];
    if (plotX === "alpha") {
      const c = result.cases[Math.min(machFocus, result.cases.length - 1)];
      if (!c) return [];
      return c.points.map((p) => ({ x: p.alpha, y: p[plotY], mach: c.mach }));
    }
    // vs mach at nearest alpha=0 (or first)
    return result.cases.map((c) => {
      const p = c.points.find((q) => Math.abs(q.alpha) < 1e-6) ?? c.points[0];
      return { x: c.mach, y: p?.[plotY] ?? 0, alpha: p?.alpha ?? 0 };
    });
  }, [result, plotX, plotY, machFocus]);

  const case0 = result?.cases[Math.min(machFocus, (result?.cases.length ?? 1) - 1)];

  function patchBody(p: Partial<NonNullable<VehicleConfig["body"]>["parametric"]>) {
    setConfig((c) => {
      const n = cloneConfig(c);
      if (!n.body) n.body = { parametric: { noseKind: "ogive", noseLength: 0.3, noseDiameter: 0.1, centerLength: 1, aftLength: 0 } };
      if (!n.body.parametric) n.body.parametric = { noseKind: "ogive", noseLength: 0.3, noseDiameter: 0.1, centerLength: 1, aftLength: 0 };
      Object.assign(n.body.parametric, p);
      return n;
    });
  }

  const body = config.body?.parametric;

  return (
    <div className="space-y-4">
      <p className="text-xs leading-relaxed text-muted">
        Engineering-level DATCOM / Missile DATCOM substitute. Body + fin/wing/canard synthesis with
        slender-body, Allen–Perkins, linear theory, shock-expansion and Newtonian methods. Not a
        certified DATCOM replacement — see method tags and confidence notes on every point.
      </p>

      <div>
        <span className="mb-1.5 block text-xs font-medium text-muted">Preset</span>
        <div className="flex flex-wrap gap-1.5">
          {PRESETS.map((p) => (
            <button
              key={p.id}
              type="button"
              onClick={() => loadPreset(p.id)}
              className={`h-8 rounded-sm px-2.5 text-[11px] ${presetId === p.id ? "bg-accent text-accent-fg" : "bg-surface-2 text-muted"}`}
            >
              {p.name}
            </button>
          ))}
          <button
            type="button"
            onClick={loadFromWaverider}
            className={`h-8 rounded-sm px-2.5 text-[11px] ${presetId === "waverider" ? "bg-accent text-accent-fg" : "bg-surface-2 text-muted"}`}
          >
            From waverider
          </button>
        </div>
        <p className="mt-1.5 text-[11px] leading-snug text-subtle">
          {PRESETS.find((p) => p.id === presetId)?.blurb ??
            (presetId === "waverider"
              ? "Cone-cylinder proxy of the current Geom tab (qualitative only)."
              : presetId === "deck"
                ? "Loaded from a pasted DATCOM deck."
                : "")}
        </p>
      </div>

      {body ? (
        <div className="rounded-sm bg-surface-2/60 p-3">
          <p className="mb-2 text-[10px] font-medium tracking-[0.14em] text-subtle uppercase">Body (parametric)</p>
          <div className="grid grid-cols-2 gap-x-3">
            <Num label="Nose L" value={body.noseLength} onChange={(noseLength) => patchBody({ noseLength })} step={0.01} />
            <Num label="Nose D" value={body.noseDiameter} onChange={(noseDiameter) => patchBody({ noseDiameter })} step={0.01} />
            <Num label="Centre L" value={body.centerLength} onChange={(centerLength) => patchBody({ centerLength })} step={0.01} />
            <Num label="Aft L" value={body.aftLength} onChange={(aftLength) => patchBody({ aftLength })} step={0.01} />
            <Num label="Xcg" value={config.ref.xcg} onChange={(xcg) => setConfig((c) => ({ ...c, ref: { ...c.ref, xcg } }))} step={0.01} />
            <label className="flex items-center justify-between gap-2 py-0.5">
              <span className="text-[11px] text-muted">Nose</span>
              <select
                className="h-7 rounded-sm border-0 bg-surface px-1 font-mono text-[11px] text-fg"
                value={body.noseKind}
                onChange={(e) => patchBody({ noseKind: e.target.value as typeof body.noseKind })}
              >
                {["cone", "ogive", "karman", "haack", "power", "spherical"].map((k) => (
                  <option key={k} value={k}>
                    {k}
                  </option>
                ))}
              </select>
            </label>
          </div>
          <p className="mt-2 text-[10px] text-subtle">
            {config.sets.length} surface set{config.sets.length === 1 ? "" : "s"}
            {config.sets.map((s) => ` · ${s.name}(${s.nPanels})`).join("")}
          </p>
        </div>
      ) : null}

      <div className="rounded-sm bg-surface-2/60 p-3">
        <p className="mb-2 text-[10px] font-medium tracking-[0.14em] text-subtle uppercase">Flight</p>
        <label className="block">
          <span className="text-[11px] text-muted">Mach list</span>
          <input
            className="mt-1 h-8 w-full rounded-sm border-0 bg-surface px-2 font-mono text-xs text-fg outline-none"
            value={flight.machs.join(", ")}
            onChange={(e) => {
              const machs = e.target.value
                .split(/[,\s]+/)
                .map(parseFloat)
                .filter((n) => Number.isFinite(n) && n > 0);
              if (machs.length) setFlight((f) => ({ ...f, machs }));
            }}
          />
        </label>
        <label className="mt-2 block">
          <span className="text-[11px] text-muted">Alpha list [deg]</span>
          <input
            className="mt-1 h-8 w-full rounded-sm border-0 bg-surface px-2 font-mono text-xs text-fg outline-none"
            value={flight.alphas.join(", ")}
            onChange={(e) => {
              const alphas = e.target.value
                .split(/[,\s]+/)
                .map(parseFloat)
                .filter((n) => Number.isFinite(n));
              if (alphas.length) setFlight((f) => ({ ...f, alphas }));
            }}
          />
        </label>
        <div className="mt-1 grid grid-cols-3 gap-x-3">
          <Num label="Beta" value={flight.beta ?? 0} onChange={(beta) => setFlight((f) => ({ ...f, beta }))} step={0.5} />
          <Num label="Phi" value={flight.phi ?? 0} onChange={(phi) => setFlight((f) => ({ ...f, phi }))} step={5} />
          <Num
            label="Alt km"
            value={typeof flight.altKm === "number" ? flight.altKm : flight.altKm?.[0] ?? 0}
            onChange={(altKm) => setFlight((f) => ({ ...f, altKm }))}
            step={1}
          />
        </div>
      </div>

      <div className="flex flex-wrap gap-2">
        <Button type="button" onClick={run} disabled={busy} className="h-9">
          {busy ? "Running…" : "Run DATCOM"}
        </Button>
        <Button
          type="button"
          variant="outline"
          className="h-9"
          onClick={() => {
            const t = generateMissileDeck(config, flight, "m");
            setDeckText(t);
            navigator.clipboard?.writeText(t).catch(() => undefined);
          }}
        >
          Copy deck
        </Button>
      </div>
      {err ? <p className="text-xs text-warn">{err}</p> : null}

      {result ? (
        <>
          <div className="space-y-0.5">
            <Stat k="Cases" v={`${result.cases.length} Mach × ${result.cases[0]?.points.length ?? 0} α`} />
            <Stat k="Sref" v={`${fmt(result.geometry.sRef, 4)} m²`} />
            <Stat k="Lref" v={`${fmt(result.geometry.lRef, 3)} m`} />
            <Stat k="Xcg" v={`${fmt(result.geometry.xcg, 3)} m`} />
            <Stat
              k="SM @ M₀,α₀"
              v={`${fmt(result.cases[0]?.points.find((p) => Math.abs(p.alpha) < 1e-9)?.derivs.staticMargin ?? result.cases[0]?.points[0]?.derivs.staticMargin ?? 0, 2)} cal`}
            />
            <Stat k="Elapsed" v={`${fmt((result as RunResult & { _ms?: number })._ms ?? 0, 0)} ms`} />
            <Stat k="Confidence" v={result.cases[0]?.points[0]?.validity.confidence ?? "—"} />
          </div>
          <p className="text-[11px] leading-snug text-subtle">{result.cases[0]?.points[0]?.validity.method}</p>
          {(result.warnings.length > 0 || (result.cases[0]?.points[0]?.validity.notes.length ?? 0) > 0) && (
            <ul className="space-y-1 text-[11px] leading-snug text-muted">
              {result.warnings.map((w) => (
                <li key={w}>• {w}</li>
              ))}
              {(result.cases[0]?.points[0]?.validity.notes ?? []).map((n) => (
                <li key={n}>• {n}</li>
              ))}
            </ul>
          )}

          <div>
            <div className="mb-2 flex flex-wrap items-center gap-2">
              <span className="text-[11px] text-muted">Plot</span>
              <select
                className="h-7 rounded-sm bg-surface-2 px-1 font-mono text-[11px] text-fg"
                value={plotY}
                onChange={(e) => setPlotY(e.target.value as PlotY)}
              >
                {YOPTS.map((o) => (
                  <option key={o.id} value={o.id}>
                    {o.label}
                  </option>
                ))}
              </select>
              <span className="text-[11px] text-muted">vs</span>
              <select
                className="h-7 rounded-sm bg-surface-2 px-1 font-mono text-[11px] text-fg"
                value={plotX}
                onChange={(e) => setPlotX(e.target.value as PlotX)}
              >
                <option value="alpha">alpha</option>
                <option value="mach">Mach</option>
              </select>
              {plotX === "alpha" && result.cases.length > 1 ? (
                <select
                  className="h-7 rounded-sm bg-surface-2 px-1 font-mono text-[11px] text-fg"
                  value={machFocus}
                  onChange={(e) => setMachFocus(parseInt(e.target.value, 10))}
                >
                  {result.cases.map((c, i) => (
                    <option key={c.mach} value={i}>
                      M={c.mach}
                    </option>
                  ))}
                </select>
              ) : null}
            </div>
            <div className="h-48 w-full">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={chartData} margin={{ top: 8, right: 12, left: 0, bottom: 0 }}>
                  <CartesianGrid stroke="currentColor" strokeOpacity={0.12} />
                  <XAxis dataKey="x" tick={{ fontSize: 10 }} stroke="currentColor" strokeOpacity={0.4} />
                  <YAxis tick={{ fontSize: 10 }} stroke="currentColor" strokeOpacity={0.4} width={40} />
                  <Tooltip contentStyle={{ background: "#111", border: "1px solid #333", fontSize: 11 }} />
                  <Legend wrapperStyle={{ fontSize: 11 }} />
                  <Line type="monotone" dataKey="y" name={plotY} stroke="currentColor" strokeWidth={1.5} dot={false} />
                </LineChart>
              </ResponsiveContainer>
            </div>
          </div>

          {case0 ? (
            <div className="overflow-x-auto">
              <p className="mb-1 text-[10px] font-medium tracking-[0.14em] text-subtle uppercase">
                M = {case0.mach.toFixed(3)} · α table
              </p>
              <table className="w-full min-w-[36rem] border-collapse font-mono text-[10px] tabular-nums">
                <thead>
                  <tr className="text-left text-subtle">
                    {["ALPHA", "CN", "CM", "CA", "CL", "CD", "XCP", "CNA", "CMA", "SM"].map((h) => (
                      <th key={h} className="border-b border-border px-1 py-1 font-medium">
                        {h}
                      </th>
                    ))}
                  </tr>
                </thead>
                <tbody>
                  {case0.points.map((p) => (
                    <tr key={p.alpha} className="text-fg">
                      <td className="border-b border-border/50 px-1 py-0.5">{p.alpha.toFixed(1)}</td>
                      <td className="border-b border-border/50 px-1 py-0.5">{p.CN.toFixed(4)}</td>
                      <td className="border-b border-border/50 px-1 py-0.5">{p.Cm.toFixed(4)}</td>
                      <td className="border-b border-border/50 px-1 py-0.5">{p.CA.toFixed(4)}</td>
                      <td className="border-b border-border/50 px-1 py-0.5">{p.CL.toFixed(4)}</td>
                      <td className="border-b border-border/50 px-1 py-0.5">{p.CD.toFixed(4)}</td>
                      <td className="border-b border-border/50 px-1 py-0.5">{(p.xcp / result.geometry.lRef).toFixed(3)}</td>
                      <td className="border-b border-border/50 px-1 py-0.5">{(p.derivs.CNa * (Math.PI / 180)).toFixed(4)}</td>
                      <td className="border-b border-border/50 px-1 py-0.5">{(p.derivs.Cma * (Math.PI / 180)).toFixed(4)}</td>
                      <td className="border-b border-border/50 px-1 py-0.5">{p.derivs.staticMargin.toFixed(2)}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
              <p className="mt-1 text-[10px] text-subtle">CNA / CMA shown per degree (Missile DATCOM default).</p>
            </div>
          ) : null}

          <div className="flex flex-wrap items-center gap-2">
            <select
              className="h-8 rounded-sm bg-surface-2 px-2 font-mono text-[11px] text-fg"
              value={outFmt}
              onChange={(e) => setOutFmt(e.target.value as OutputFormat)}
            >
              <option value="missile">Missile for006</option>
              <option value="usaf">USAF for004</option>
              <option value="csv">CSV</option>
              <option value="json">JSON</option>
              <option value="sixdof">6DOF JSON</option>
            </select>
            <Button
              type="button"
              variant="outline"
              className="h-8"
              onClick={() => downloadText(downloadName(outFmt), format(result, outFmt))}
            >
              Download
            </Button>
            <Button
              type="button"
              variant="outline"
              className="h-8"
              onClick={() => navigator.clipboard?.writeText(format(result, outFmt))}
            >
              Copy
            </Button>
          </div>
        </>
      ) : null}

      <div>
        <p className="mb-1.5 text-[10px] font-medium tracking-[0.14em] text-subtle uppercase">Paste DATCOM deck</p>
        <textarea
          className="h-28 w-full resize-y rounded-sm border-0 bg-surface-2 p-2 font-mono text-[10px] leading-snug text-fg outline-none focus:bg-surface"
          placeholder={"$FLTCON NMACH=2., MACH=1.5,2.0, NALPHA=3., ALPHA=0.,5.,10.,$\n$AXIBOD ...$\n$FINSET1 ...$"}
          value={deckText}
          onChange={(e) => setDeckText(e.target.value)}
        />
        <div className="mt-2 flex gap-2">
          <Button type="button" variant="outline" className="h-8" onClick={pasteDeck} disabled={!deckText.trim()}>
            Parse & load
          </Button>
        </div>
        {deckWarn.length ? (
          <ul className="mt-2 space-y-1 text-[11px] leading-snug text-muted">
            {deckWarn.map((w) => (
              <li key={w}>• {w}</li>
            ))}
          </ul>
        ) : null}
      </div>
    </div>
  );
}
