import { useMemo, useRef, useState } from "react";
import { CartesianGrid, Legend, Line, LineChart, ResponsiveContainer, Tooltip, XAxis, YAxis } from "recharts";
import { Formula, NumberField, Seg, Stat } from "@/components/fields";
import {
  AERODB_POINT_CAP,
  aeroDbToCsv,
  aeroDbToJson,
  aeroDbToSimulink,
  defaultAeroDbSpec,
  generateAeroDatabase,
  gridCount,
  type AeroDatabase,
  type AeroDbSpec,
} from "@/lib/waverider/aerodb";
import {
  AERODB_COEFS,
  COEF_LABEL,
  aeroDbZip,
  coefVsAlphaByMach,
  coefVsDeltaByMach,
  parseAxisList,
  summarizeAeroDb,
  type SeriesRow,
} from "@/lib/waverider/aerodb-summary";
import { genericDeck, openFoamFiles, python6dofSource, su2Config } from "@/lib/waverider/export-cfd";
import {
  defaultFlySpec,
  flyClosedLoop,
  samplesToCsv,
  type FlyResult,
  type FlySpec,
  type GuidanceMode,
  type ControlLaw,
} from "@/lib/waverider/guidance";
import { massProperties } from "@/lib/waverider/mass";
import {
  evaluateDesign,
  numericDesignKeys,
  parameterSweep,
  runOptimizer,
  sensitivity,
  type OptResult,
  type SensitivityRow,
  type SweepRow,
} from "@/lib/waverider/mdo";
import { inletOperability, type OperabilityRow } from "@/lib/waverider/operability";
import { applyMassToParams, massBudget, sizeTps, TPS_LIB, type MassBudget, type TpsMaterialId, type TpsSizing } from "@/lib/waverider/tps";
import { domainOf, type AeroMethod, type BuiltVehicle, type DesignParams } from "@/lib/waverider/types";
import { sanitizeParams } from "@/lib/waverider/presets";
import { fmt } from "@/lib/waverider/math";
import { downloadBlob } from "@/lib/utils";

type Sub = "db" | "opt" | "fly" | "mass" | "compare";
type ListAxis = "machList" | "alphaList" | "betaList" | "deltaList";

const MACH_COLORS = ["#ff2d2d", "#f3efe6", "#ff8a5c", "#b9b3a8", "#c94f4f", "#ffd9c7", "#7a4545"];

const HIST_KEY = "cuspis.history.v1";
const VAR_KEY = "cuspis.variants.v1";

interface SavedVariant {
  id: string;
  name: string;
  at: number;
  params: DesignParams;
  ld: number;
  cd: number;
  q: number;
  mass: number;
  sm: number;
  rangeKm: number;
}

const STORE_MAX = 1_500_000;

function dropProto(key: string, value: unknown) {
  return key === "__proto__" || key === "constructor" || key === "prototype" ? undefined : value;
}

function readList<T extends { params?: unknown; name?: unknown }>(key: string, limit: number): T[] {
  try {
    const raw = localStorage.getItem(key);
    if (!raw || raw.length > STORE_MAX) return [];
    const parsed = JSON.parse(raw, dropProto);
    if (!Array.isArray(parsed)) return [];
    const out: T[] = [];
    for (const row of parsed) {
      if (!row || typeof row !== "object") continue;
      const rec = row as T;
      const params = sanitizeParams(rec.params);
      const name = typeof rec.name === "string" ? rec.name.replace(/[\u0000-\u001f\u007f]/g, "").slice(0, 64) : params.name;
      out.push({ ...rec, params, name });
      if (out.length >= limit) break;
    }
    return out;
  } catch {
    return [];
  }
}

function chartTip(label: string) {
  return {
    contentStyle: { background: "#101010", border: "1px solid #7a4545", fontSize: 11, color: "#f3efe6" },
    labelFormatter: () => label,
  };
}

export function IteratePanel({
  params,
  built,
  onParams,
}: {
  params: DesignParams;
  built: BuiltVehicle;
  onParams: (p: DesignParams) => void;
}) {
  const [sub, setSub] = useState<Sub>("db");
  const [spec, setSpec] = useState<AeroDbSpec>(() => defaultAeroDbSpec(params));
  const [db, setDb] = useState<AeroDatabase | null>(null);
  const [dbErr, setDbErr] = useState<string | null>(null);
  const [dbProg, setDbProg] = useState({ done: 0, total: 0, label: "" });
  const [busy, setBusy] = useState<string | null>(null);
  const abortRef = useRef<AbortController | null>(null);
  const [axisMode, setAxisMode] = useState<"range" | "list">("range");
  const [listText, setListText] = useState<Record<ListAxis, string>>({ machList: "", alphaList: "", betaList: "", deltaList: "" });
  const [listErr, setListErr] = useState<string | null>(null);

  const [flySpec, setFlySpec] = useState<FlySpec>(() => defaultFlySpec(params));
  const [fly, setFly] = useState<FlyResult | null>(null);
  const [opt, setOpt] = useState<OptResult | null>(null);
  const [vars, setVars] = useState<string[]>(["shockDeg", "span", "cgFrac"]);
  const [pop, setPop] = useState(8);
  const [gens, setGens] = useState(4);
  const [sweepKey, setSweepKey] = useState<keyof DesignParams>("shockDeg");
  const [sweep, setSweep] = useState<SweepRow[] | null>(null);
  const [sens, setSens] = useState<SensitivityRow[] | null>(null);
  const [material, setMaterial] = useState<TpsMaterialId>("pica");
  const [heatTime, setHeatTime] = useState(120);
  const [payload, setPayload] = useState(40);
  const [fuelFrac, setFuelFrac] = useState(0.15);
  const [tps, setTps] = useState<TpsSizing | null>(null);
  const [budget, setBudget] = useState<MassBudget | null>(null);
  const [ops, setOps] = useState<OperabilityRow[] | null>(null);
  const [variants, setVariants] = useState<SavedVariant[]>(() => readList<SavedVariant>(VAR_KEY, 8));
  const [history, setHistory] = useState<{ at: number; name: string; params: DesignParams }[]>(() => readList(HIST_KEY, 12));

  const preview = useMemo(() => gridCount(spec, params.mach, params.alphaDeg), [spec, params.mach, params.alphaDeg]);
  const internal = domainOf(params.family) === "internal";
  const extra = useMemo(() => (db ? summarizeAeroDb(db, params.alphaDeg) : null), [db, params.alphaDeg]);
  const ldSeries = useMemo(() => (db ? coefVsAlphaByMach(db, "ld") : null), [db]);
  const cmDelta = useMemo(() => (db ? coefVsDeltaByMach(db, "cm", params.alphaDeg) : null), [db, params.alphaDeg]);

  const setList = (key: ListAxis, text: string) => {
    const nextText = { ...listText, [key]: text };
    setListText(nextText);
    try {
      const next = { ...spec };
      for (const k of Object.keys(nextText) as ListAxis[]) next[k] = parseAxisList(nextText[k]);
      setSpec(next);
      setListErr(null);
    } catch (err) {
      setListErr(err instanceof Error ? err.message : String(err));
    }
  };
  const pickAxisMode = (mode: "range" | "list") => {
    setAxisMode(mode);
    setListErr(null);
    if (mode === "range") setSpec({ ...spec, machList: [], alphaList: [], betaList: [], deltaList: [] });
    else {
      const next = { ...spec };
      for (const k of Object.keys(listText) as ListAxis[]) {
        try {
          next[k] = parseAxisList(listText[k]);
        } catch {
          next[k] = [];
        }
      }
      setSpec(next);
    }
  };

  const stop = () => abortRef.current?.abort();

  const runDb = async () => {
    stop();
    const ac = new AbortController();
    abortRef.current = ac;
    setBusy("db");
    setDbErr(null);
    try {
      const next = await generateAeroDatabase({ ...built, params }, spec, {
        signal: ac.signal,
        onProgress: setDbProg,
      });
      setDb(next);
    } catch (err) {
      if (err instanceof DOMException && err.name === "AbortError") setDbErr("Cancelled.");
      else setDbErr(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(null);
    }
  };

  const runFly = async () => {
    stop();
    const ac = new AbortController();
    abortRef.current = ac;
    setBusy("fly");
    setDbErr(null);
    try {
      const next = await flyClosedLoop({ ...built, params }, flySpec, {
        signal: ac.signal,
        onProgress: (p) => setDbProg(p),
        massKg: budget?.total,
        ixx: budget?.ixx,
        iyy: budget?.iyy,
        izz: budget?.izz,
      });
      setFly(next);
    } catch (err) {
      if (!(err instanceof DOMException && err.name === "AbortError")) setDbErr(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(null);
    }
  };

  const runOpt = async () => {
    stop();
    const ac = new AbortController();
    abortRef.current = ac;
    setBusy("opt");
    setDbErr(null);
    try {
      const next = await runOptimizer(
        params,
        {
          variables: vars.map((key) => ({ key: key as keyof DesignParams, ...bounds(key) })),
          objectives: [
            { metric: "ld", goal: "max", weight: 1 },
            { metric: "qStag", goal: "min", weight: 1 },
          ],
          constraints: [{ metric: "staticMargin", op: ">=", value: 0 }],
          population: pop,
          generations: gens,
          seed: 7,
        },
        { signal: ac.signal, onProgress: (p) => setDbProg({ done: p.generation, total: p.generations, label: p.label }) },
      );
      setOpt(next);
    } catch (err) {
      if (!(err instanceof DOMException && err.name === "AbortError")) setDbErr(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(null);
    }
  };

  const runSweep = async () => {
    setBusy("sweep");
    setDbErr(null);
    try {
      const base = Number(params[sweepKey]);
      const values = [0.85, 0.95, 1, 1.05, 1.15].map((f) => (Number.isFinite(base) ? base * f : f));
      const rows = await parameterSweep(params, sweepKey, values, {
        onProgress: (d, t) => setDbProg({ done: d, total: t, label: `sweep ${sweepKey}` }),
      });
      setSweep(rows);
    } catch (err) {
      setDbErr(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(null);
    }
  };

  const runSens = async () => {
    setBusy("sens");
    try {
      const keys = vars.map((k) => k as keyof DesignParams);
      const out = await sensitivity(params, keys, "ld", 0.05);
      setSens(out.rows);
    } catch (err) {
      setDbErr(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(null);
    }
  };

  const sizeMass = () => {
    const ev = evaluateDesign({ ...params, massKg: 0 });
    const area = Math.max(built.quality.area * 0.45, built.aero.planformArea);
    const sized = sizeTps({ qWcm2: ev.metrics.qStag, areaM2: area, timeS: heatTime, material });
    const mp = massProperties(built.mesh, params.rhoKgM3 || 160, 0);
    const b = massBudget({
      params: sanitizeParams(params),
      volume: built.quality.volume,
      length: params.length,
      height: params.height,
      structMass: mp.mass,
      structCgX: mp.cg[0],
      ixx: mp.Ixx,
      iyy: mp.Iyy,
      izz: mp.Izz,
      tpsMass: sized.mass,
      payloadKg: payload,
      fuelFrac,
      fuelRho: params.fuel === "H2" ? 71 : 800,
    });
    setTps(sized);
    setBudget(b);
  };

  const saveVariant = () => {
    const ev = evaluateDesign(params);
    const row: SavedVariant = {
      id: `${Date.now()}`,
      name: params.name || params.family,
      at: Date.now(),
      params: sanitizeParams(params),
      ld: ev.metrics.ld,
      cd: ev.metrics.cd,
      q: ev.metrics.qStag,
      mass: ev.metrics.mass,
      sm: ev.metrics.staticMargin,
      rangeKm: ev.metrics.rangeKm,
    };
    const next = [row, ...variants].slice(0, 8);
    setVariants(next);
    try {
      localStorage.setItem(VAR_KEY, JSON.stringify(next));
    } catch {
      setDbErr("This browser refused to store the variant.");
    }
  };

  const snapshot = () => {
    const row = { at: Date.now(), name: params.name || params.family, params: sanitizeParams(params) };
    const next = [row, ...history].slice(0, 12);
    setHistory(next);
    try {
      localStorage.setItem(HIST_KEY, JSON.stringify(next));
    } catch {
      setDbErr("This browser refused to store the snapshot.");
    }
  };

  const pct = dbProg.total > 0 ? Math.round((100 * dbProg.done) / dbProg.total) : 0;

  return (
    <div className="space-y-4">
      <Seg
        value={sub}
        onChange={setSub}
        options={[
          { id: "db", label: "Aero DB" },
          { id: "opt", label: "Optimize" },
          { id: "fly", label: "Fly" },
          { id: "mass", label: "TPS" },
          { id: "compare", label: "Compare" },
        ]}
      />

      {dbErr ? <p className="text-xs leading-relaxed text-warn">{dbErr}</p> : null}
      {busy ? (
        <div>
          <div className="flex items-center justify-between gap-2">
            <p className="truncate text-[11px] text-muted">{dbProg.label || busy}</p>
            <button type="button" className="text-xs text-accent" onClick={stop}>
              Cancel
            </button>
          </div>
          <div className="mt-1 h-1.5 overflow-hidden rounded-sm bg-surface-2">
            <div className="h-full bg-accent" style={{ width: `${pct}%` }} />
          </div>
        </div>
      ) : null}

      {sub === "db" ? (
        <div className="space-y-3">
          <p className="text-xs leading-relaxed text-subtle">
            One table covers the design Mach and the off-design Mach numbers on the same shape. Deflection rebuilds the elevon or the fin ridges.
            Static derivatives are central differences of the table, one-sided at the ends. Dynamic derivatives are central body-rate differences at β = 0.
          </p>
          <Seg
            value={axisMode}
            onChange={pickAxisMode}
            options={[
              { id: "range", label: "Range + step" },
              { id: "list", label: "List" },
            ]}
          />
          {axisMode === "list" ? (
            <div className="space-y-2">
              <AxisList label="Mach" value={listText.machList} placeholder={`blank = ${spec.machMin}…${spec.machMax} step ${spec.machStep}`} onChange={(t) => setList("machList", t)} />
              <AxisList label="α deg" value={listText.alphaList} placeholder={`blank = ${spec.alphaMin}…${spec.alphaMax} step ${spec.alphaStep}`} onChange={(t) => setList("alphaList", t)} />
              <AxisList label="β deg" value={listText.betaList} placeholder={`blank = ${spec.betaMin}…${spec.betaMax} step ${spec.betaStep}`} onChange={(t) => setList("betaList", t)} />
              <AxisList label="δ deg" value={listText.deltaList} placeholder={`blank = ${spec.deltaMin}…${spec.deltaMax} step ${spec.deltaStep}`} onChange={(t) => setList("deltaList", t)} />
              <p className="text-[11px] leading-relaxed text-subtle">Comma-separated breakpoints. A blank axis keeps its range and step.</p>
              {listErr ? <p className="text-[11px] text-warn">{listErr}</p> : null}
            </div>
          ) : null}
          {axisMode === "range" ? (
            <>
              <AxisRange label="Mach" a={spec.machMin} b={spec.machMax} s={spec.machStep} onA={(machMin) => setSpec({ ...spec, machMin })} onB={(machMax) => setSpec({ ...spec, machMax })} onS={(machStep) => setSpec({ ...spec, machStep })} />
              <AxisRange label="α deg" a={spec.alphaMin} b={spec.alphaMax} s={spec.alphaStep} onA={(alphaMin) => setSpec({ ...spec, alphaMin })} onB={(alphaMax) => setSpec({ ...spec, alphaMax })} onS={(alphaStep) => setSpec({ ...spec, alphaStep })} />
              <AxisRange label="β deg" a={spec.betaMin} b={spec.betaMax} s={spec.betaStep} onA={(betaMin) => setSpec({ ...spec, betaMin })} onB={(betaMax) => setSpec({ ...spec, betaMax })} onS={(betaStep) => setSpec({ ...spec, betaStep })} />
              <AxisRange label="δ deg" a={spec.deltaMin} b={spec.deltaMax} s={spec.deltaStep} onA={(deltaMin) => setSpec({ ...spec, deltaMin })} onB={(deltaMax) => setSpec({ ...spec, deltaMax })} onS={(deltaStep) => setSpec({ ...spec, deltaStep })} />
            </>
          ) : null}
          <div>
            <span className="hud-label mb-2 block !text-muted">Method</span>
            <Seg
              value={spec.method}
              onChange={(method: AeroMethod) => setSpec({ ...spec, method })}
              options={[
                { id: "newtonian", label: "Newton" },
                { id: "tangent", label: "Tangent" },
                { id: "cbaero", label: "CBAERO" },
                { id: "mixed", label: "Mixed" },
              ]}
            />
          </div>
          <label className="flex items-center justify-between gap-2 text-xs text-muted">
            Symmetry-plane Euler slice (β = 0, δ = 0)
            <input type="checkbox" checked={spec.eulerSymmetry} onChange={(e) => setSpec({ ...spec, eulerSymmetry: e.target.checked })} />
          </label>
          <label className="flex items-center justify-between gap-2 text-xs text-muted">
            Dynamic derivatives
            <input type="checkbox" checked={spec.dynamicDerivatives} onChange={(e) => setSpec({ ...spec, dynamicDerivatives: e.target.checked })} />
          </label>
          <p className="text-[11px] text-muted">
            {preview.points} points {preview.points > AERODB_POINT_CAP ? `(over the ${AERODB_POINT_CAP} cap)` : ""} · design Mach {fmt(params.mach, 1)} is forced onto the axis
          </p>
          <button type="button" className="hud-card w-full rounded-md px-3 py-2 text-sm text-fg" onClick={runDb} disabled={!!busy || !!listErr}>
            {busy === "db" ? "Building table…" : "Build aero database"}
          </button>
          {db ? (
            <div className="space-y-3">
              <div className="grid grid-cols-2 gap-2">
                <Stat k="Points" v={String(db.summary.points)} />
                <Stat k="On design" v={String(db.summary.onDesign)} />
                <Stat k="Off design" v={String(db.summary.offDesign)} />
                <Stat k="Max L/D" v={fmt(db.summary.maxLd, 2)} />
                <Stat k="CL span" v={`${fmt(db.summary.clMin, 3)}…${fmt(db.summary.clMax, 3)}`} />
                <Stat k="Cm @ design α" v={fmt(db.summary.cmAtDesign, 4)} />
              </div>
              <p className="text-[11px] leading-relaxed text-subtle">
                Peak L/D {fmt(db.summary.maxLd, 2)} at M {fmt(db.summary.maxLdAt.mach, 2)}, α {fmt(db.summary.maxLdAt.alpha_deg, 1)}°, β {fmt(db.summary.maxLdAt.beta_deg, 1)}°, δ {fmt(db.summary.maxLdAt.delta_deg, 1)}°.
                Sref {fmt(db.meta.sRef_m2, 3)} m², Lref {fmt(db.meta.lRef_m, 3)} m, CG x {fmt(db.meta.cg_m[0], 3)} m.
              </p>
              <div className="h-40">
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart data={mergeSlices(db)} margin={{ top: 4, right: 8, left: 0, bottom: 0 }}>
                    <CartesianGrid stroke="rgba(243,239,230,0.08)" />
                    <XAxis dataKey="alpha" tick={{ fill: "#b9b3a8", fontSize: 10 }} />
                    <YAxis tick={{ fill: "#b9b3a8", fontSize: 10 }} />
                    <Tooltip {...chartTip("α deg")} />
                    <Legend wrapperStyle={{ fontSize: 10 }} />
                    <Line type="monotone" dataKey="cl" stroke="#ff2d2d" dot={false} name={`CL M${fmt(db.slices.design.mach, 1)}`} strokeWidth={1.5} />
                    <Line type="monotone" dataKey="clOff" stroke="#f3efe6" dot={false} name={`CL M${fmt(db.slices.offDesign.mach, 1)}`} strokeWidth={1.25} />
                    <Line type="monotone" dataKey="cm" stroke="#7a4545" dot={false} name="Cm design" strokeWidth={1.25} />
                  </LineChart>
                </ResponsiveContainer>
              </div>
              {extra ? (
                <div className="space-y-2">
                  <span className="hud-label block !text-muted">
                    Trim and static margin · β {fmt(extra.anchor.beta_deg, 1)}°, δ {fmt(extra.anchor.delta_deg, 1)}°
                  </span>
                  <table className="w-full font-mono text-[11px] tabular-nums text-muted">
                    <thead>
                      <tr className="text-subtle">
                        <th className="text-left font-normal">M</th>
                        <th className="text-right font-normal">trim α</th>
                        <th className="text-right font-normal">SM %L</th>
                        <th className="text-right font-normal">SM m</th>
                      </tr>
                    </thead>
                    <tbody>
                      {extra.trim.map((t) => {
                        const m = extra.margin.find((r) => r.mach === t.mach);
                        return (
                          <tr key={t.mach}>
                            <td className="text-left text-fg">{fmt(t.mach, 2)}</td>
                            <td className="text-right">{t.alphaTrim === null ? "none in range" : `${fmt(t.alphaTrim, 2)}°`}</td>
                            <td className={`text-right ${m?.smFrac != null && m.smFrac < 0 ? "text-warn" : ""}`}>
                              {m?.smFrac == null ? "—" : fmt(m.smFrac * 100, 2)}
                            </td>
                            <td className="text-right">{m?.sm_m == null ? "—" : fmt(m.sm_m, 3)}</td>
                          </tr>
                        );
                      })}
                    </tbody>
                  </table>
                  <span className="hud-label block !text-muted">Coefficient ranges</span>
                  <table className="w-full font-mono text-[11px] tabular-nums text-muted">
                    <thead>
                      <tr className="text-subtle">
                        <th className="text-left font-normal">coef</th>
                        <th className="text-right font-normal">min</th>
                        <th className="text-right font-normal">max</th>
                      </tr>
                    </thead>
                    <tbody>
                      {AERODB_COEFS.map((c) => {
                        const r = extra.ranges[c];
                        return (
                          <tr key={c}>
                            <td className="text-left text-fg">{COEF_LABEL[c]}</td>
                            <td className="text-right">{r ? fmt(tidy(r.min), 4) : "—"}</td>
                            <td className="text-right">{r ? fmt(tidy(r.max), 4) : "—"}</td>
                          </tr>
                        );
                      })}
                    </tbody>
                  </table>
                  <p className="text-[11px] leading-relaxed text-subtle">
                    Trim α is the first Cm = 0 crossing along α, linear between breakpoints. SM = −Cmα/CLα at trim α (or the node nearest the design α if untrimmed), as a
                    fraction of Lref {fmt(db.meta.lRef_m, 3)} m; × Lref is x_np − x_cg. Positive is stable.
                  </p>
                </div>
              ) : null}
              {ldSeries ? <MachChart title="L/D vs α" xLabel="α deg" series={ldSeries} /> : null}
              {cmDelta && cmDelta.rows.length > 1 ? (
                <MachChart title={`Cm vs δ · α ${fmt(cmDelta.alpha_deg, 1)}°`} xLabel="δ deg" series={cmDelta} />
              ) : (
                <p className="text-[11px] text-subtle">Cm vs δ needs more than one δ breakpoint.</p>
              )}
              <div className="grid grid-cols-4 gap-2">
                <button type="button" className="hud-card rounded-md px-2 py-2 text-xs" onClick={() => downloadBlob(new Blob([aeroDbToCsv(db)], { type: "text/csv" }), `${params.name}_aerodb.csv`)}>CSV</button>
                <button type="button" className="hud-card rounded-md px-2 py-2 text-xs" onClick={() => downloadBlob(new Blob([aeroDbToJson(db)], { type: "application/json" }), `${params.name}_aerodb.json`)}>JSON</button>
                <button type="button" className="hud-card rounded-md px-2 py-2 text-xs" onClick={() => downloadBlob(new Blob([aeroDbToSimulink(db)], { type: "text/plain" }), "cuspis_aero_db.m")}>Simulink</button>
                <button type="button" className="hud-card rounded-md px-2 py-2 text-xs" onClick={() => downloadBlob(aeroDbZip(db, params.alphaDeg).blob, `${params.name}_aerodb.zip`)}>Zip</button>
              </div>
              <Formula
                name="What the files contain"
                expr="CL, CD, CY, Cm, Cn, Cll + CLα, Cmα, Cnβ, CLδ, Cmq, Clp, …"
                note="CSV has a # metadata header (Sref, Lref, CG, units, method). Zip adds one CSV per coefficient (rows M, α, β; a column per δ) and a README with this summary. The .m file reshapes to n-D lookups, delta fastest, for a Simulink n-D Lookup Table. JSON is what cuspis_6dof.py reads."
              />
            </div>
          ) : null}
          {internal ? (
            <div className="space-y-2">
              <button
                type="button"
                className="hud-card w-full rounded-md px-3 py-2 text-sm"
                onClick={() => {
                  const machs = [params.mach - 1, params.mach, params.mach + 1].map((m) => Math.max(2.5, m));
                  setOps(inletOperability(params, machs, [0.6, 0.9, 1.1]));
                }}
              >
                Inlet start map
              </button>
              {ops ? (
                <div className="space-y-1">
                  {ops.map((r) => (
                    <p key={`${r.mach}-${r.phi}`} className="text-[11px] text-muted">
                      M {fmt(r.mach, 1)} φ {fmt(r.phi, 2)} — {r.started ? "started" : "unstart risk"} · Kantrowitz {fmt(r.kantrowitz, 3)} · thrust {fmt(r.thrust, 0)} N
                    </p>
                  ))}
                  <p className="text-[11px] leading-relaxed text-subtle">1-D Kantrowitz screen from the cycle deck. Not a bleed or unstart CFD.</p>
                </div>
              ) : null}
            </div>
          ) : null}
        </div>
      ) : null}

      {sub === "opt" ? (
        <div className="space-y-3">
          <p className="text-xs leading-relaxed text-subtle">
            NSGA-II on the panel kernel. Objectives: maximize L/D and minimize stagnation heat, with static margin ≥ 0. Euler is run as CBAERO inside the loop so a population can finish.
          </p>
          <div className="flex flex-wrap gap-1.5">
            {numericDesignKeys().slice(0, 12).map((key) => {
              const on = vars.includes(key);
              return (
                <button
                  key={key}
                  type="button"
                  aria-pressed={on}
                  onClick={() => setVars(on ? vars.filter((v) => v !== key) : [...vars, key].slice(0, 4))}
                  className={`rounded-sm px-2 py-1 text-[11px] ${on ? "bg-accent text-accent-fg" : "bg-surface-2 text-muted"}`}
                >
                  {key}
                </button>
              );
            })}
          </div>
          <NumberField label="Population" value={pop} min={4} max={16} step={1} digits={0} onChange={(v) => setPop(Math.round(v))} />
          <NumberField label="Generations" value={gens} min={2} max={10} step={1} digits={0} onChange={(v) => setGens(Math.round(v))} />
          <button type="button" className="hud-card w-full rounded-md px-3 py-2 text-sm" disabled={!!busy} onClick={runOpt}>
            {busy === "opt" ? "Searching…" : "Run optimizer"}
          </button>
          {opt ? (
            <div className="space-y-2">
              <p className="text-[11px] text-muted">{opt.evaluations} evaluations · {opt.pareto.length} on the Pareto front</p>
              {opt.pareto.slice(0, 6).map((d, i) => (
                <button key={i} type="button" className="hud-card w-full rounded-md px-3 py-2 text-left" onClick={() => onParams(sanitizeParams(d.params))}>
                  <span className="block text-xs text-fg">L/D {fmt(d.metrics.ld, 2)} · q {fmt(d.metrics.qStag, 2)} W/cm² · SM {fmt(d.metrics.staticMargin, 3)}</span>
                  <span className="block text-[11px] text-muted">{d.feasible ? "feasible" : "infeasible"} · load this design</span>
                </button>
              ))}
              {opt.notes.map((n) => (
                <p key={n.slice(0, 24)} className="text-[11px] leading-relaxed text-subtle">{n}</p>
              ))}
            </div>
          ) : null}
          <div>
            <span className="hud-label mb-2 block !text-muted">Sweep variable</span>
            <select
              className="h-9 w-full rounded-sm bg-surface-2 px-2 text-xs text-fg"
              value={sweepKey}
              onChange={(e) => setSweepKey(e.target.value as keyof DesignParams)}
            >
              {numericDesignKeys().map((k) => (
                <option key={k} value={k}>{k}</option>
              ))}
            </select>
          </div>
          <div className="grid grid-cols-2 gap-2">
            <button type="button" className="hud-card rounded-md px-2 py-2 text-xs" disabled={!!busy} onClick={runSweep}>Sweep ±15%</button>
            <button type="button" className="hud-card rounded-md px-2 py-2 text-xs" disabled={!!busy} onClick={runSens}>Sensitivity</button>
          </div>
          {sweep ? (
            <div className="h-36">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={sweep.map((r) => ({ x: r.value, ld: r.metrics.ld, q: r.metrics.qStag }))}>
                  <CartesianGrid stroke="rgba(243,239,230,0.08)" />
                  <XAxis dataKey="x" tick={{ fill: "#b9b3a8", fontSize: 10 }} />
                  <YAxis tick={{ fill: "#b9b3a8", fontSize: 10 }} />
                  <Tooltip {...chartTip(String(sweepKey))} />
                  <Line type="monotone" dataKey="ld" stroke="#ff2d2d" dot={false} name="L/D" strokeWidth={1.5} />
                </LineChart>
              </ResponsiveContainer>
            </div>
          ) : null}
          {sens ? (
            <div className="space-y-1">
              {sens.map((r) => (
                <p key={r.key} className="text-[11px] text-muted">
                  {r.key}: d(L/D)/dx = {fmt(r.dMetric, 3)} · elasticity {fmt(r.elasticity, 2)}
                </p>
              ))}
            </div>
          ) : null}
          <Formula
            name="External driver"
            expr="node --import ./scripts/ts-register.mjs scripts/cuspis-eval.ts design.json"
            note="Prints the same evaluateDesign metrics as JSON. Point OpenMDAO or a Python subprocess at that command. Do not scrape this panel."
          />
        </div>
      ) : null}

      {sub === "fly" ? (
        <div className="space-y-3">
          <p className="text-xs leading-relaxed text-subtle">
            Nonlinear 6DOF on a Mach–α–δ panel table, with guidance, a control law, a lag actuator, and optional α noise. Uses the TPS mass if you sized it.
          </p>
          <Seg
            value={flySpec.mode}
            onChange={(mode: GuidanceMode) => setFlySpec({ ...flySpec, mode })}
            options={[
              { id: "boost-glide", label: "Glide" },
              { id: "constant-q", label: "Const q" },
              { id: "skip", label: "Skip" },
              { id: "terminal", label: "Terminal" },
            ]}
          />
          <Seg
            value={flySpec.law}
            onChange={(law: ControlLaw) => setFlySpec({ ...flySpec, law })}
            options={[
              { id: "pid", label: "PID" },
              { id: "ndi", label: "NDI" },
            ]}
          />
          <NumberField label="Duration" value={flySpec.durationS} min={10} max={200} step={5} unit="s" digits={0} onChange={(durationS) => setFlySpec({ ...flySpec, durationS })} />
          <NumberField label="q target" value={flySpec.qTargetKpa} min={5} max={120} step={1} unit="kPa" digits={0} onChange={(qTargetKpa) => setFlySpec({ ...flySpec, qTargetKpa })} />
          <NumberField label="Actuator τ" value={flySpec.tauAct} min={0.02} max={0.4} step={0.01} unit="s" onChange={(tauAct) => setFlySpec({ ...flySpec, tauAct })} />
          <NumberField label="Sensor σ" value={flySpec.sensorSigmaDeg} min={0} max={2} step={0.05} unit="deg" onChange={(sensorSigmaDeg) => setFlySpec({ ...flySpec, sensorSigmaDeg })} />
          <button type="button" className="hud-card w-full rounded-md px-3 py-2 text-sm" disabled={!!busy} onClick={runFly}>
            {busy === "fly" ? "Flying…" : "Fly closed loop"}
          </button>
          {fly ? (
            <div className="space-y-2">
              <Stat k="Range" v={`${fmt(fly.rangeKm, 1)} km`} />
              <Stat k="Final h" v={`${fmt(fly.finalH, 2)} km`} />
              <Stat k="Peak q" v={`${fmt(fly.maxQ / 1000, 1)} kPa`} />
              <Stat k="Peak n" v={`${fmt(fly.maxN, 2)} g`} />
              <div className="h-36">
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart data={fly.samples}>
                    <CartesianGrid stroke="rgba(243,239,230,0.08)" />
                    <XAxis dataKey="xKm" tick={{ fill: "#b9b3a8", fontSize: 10 }} />
                    <YAxis tick={{ fill: "#b9b3a8", fontSize: 10 }} />
                    <Tooltip {...chartTip("downrange km")} />
                    <Line type="monotone" dataKey="hKm" stroke="#ff2d2d" dot={false} name="h km" strokeWidth={1.5} />
                  </LineChart>
                </ResponsiveContainer>
              </div>
              <button type="button" className="hud-card w-full rounded-md px-3 py-2 text-xs" onClick={() => downloadBlob(new Blob([samplesToCsv(fly)], { type: "text/csv" }), `${params.name}_traj.csv`)}>
                Export trajectory CSV
              </button>
              {fly.notes.map((n) => (
                <p key={n.slice(0, 32)} className="text-[11px] leading-relaxed text-subtle">{n}</p>
              ))}
            </div>
          ) : null}
        </div>
      ) : null}

      {sub === "mass" ? (
        <div className="space-y-3">
          <p className="text-xs leading-relaxed text-subtle">
            Insulation thickness holds the backface under 0.5 W/cm². If the peak exceeds reradiation at Tmax, an ablator thickness is added from Q*. Structure stays the smeared density; TPS, fuel, and payload are added.
          </p>
          <label className="block">
            <span className="hud-label mb-1 block !text-muted">TPS material</span>
            <select className="h-9 w-full rounded-sm bg-surface-2 px-2 text-xs text-fg" value={material} onChange={(e) => setMaterial(e.target.value as TpsMaterialId)}>
              {Object.values(TPS_LIB).map((m) => (
                <option key={m.id} value={m.id}>{m.label}</option>
              ))}
            </select>
          </label>
          <NumberField label="Heat pulse" value={heatTime} min={10} max={600} step={10} unit="s" digits={0} onChange={setHeatTime} />
          <NumberField label="Payload" value={payload} min={0} max={2000} step={5} unit="kg" digits={0} onChange={setPayload} />
          <NumberField label="Fuel fill" value={fuelFrac} min={0} max={0.7} step={0.01} onChange={setFuelFrac} />
          <button type="button" className="hud-card w-full rounded-md px-3 py-2 text-sm" onClick={sizeMass}>Size TPS and mass</button>
          {tps && budget ? (
            <div className="space-y-1">
              <Stat k="Thickness" v={`${fmt(tps.thicknessMm, 1)} mm`} ok={!tps.overTemperature} />
              <Stat k="TPS mass" v={`${fmt(tps.mass, 1)} kg`} />
              <Stat k="Rad margin" v={fmt(tps.radMargin, 2)} ok={tps.radMargin > 0} />
              <Stat k="Total mass" v={`${fmt(budget.total, 1)} kg`} />
              <Stat k="Fuel" v={`${fmt(budget.fuel, 1)} kg`} />
              <Stat k="CG x/L" v={fmt(budget.cgFrac, 3)} />
              <button type="button" className="hud-card mt-2 w-full rounded-md px-3 py-2 text-sm" onClick={() => onParams(sanitizeParams(applyMassToParams(params, budget)))}>
                Use this mass and CG
              </button>
              {tps.notes.concat(budget.notes).map((n) => (
                <p key={n.slice(0, 40)} className="text-[11px] leading-relaxed text-subtle">{n}</p>
              ))}
            </div>
          ) : null}
        </div>
      ) : null}

      {sub === "compare" ? (
        <div className="space-y-3">
          <div className="grid grid-cols-2 gap-2">
            <button type="button" className="hud-card rounded-md px-2 py-2 text-xs" onClick={saveVariant}>Save variant</button>
            <button type="button" className="hud-card rounded-md px-2 py-2 text-xs" onClick={snapshot}>Snapshot history</button>
          </div>
          <p className="text-[11px] text-subtle">Variants keep the metrics from one panel evaluation. History restores a geometry. Both stay in this browser.</p>
          {variants.length === 0 ? <p className="text-xs text-muted">No saved variants yet.</p> : null}
          {variants.map((v) => (
            <div key={v.id} className="hud-card rounded-md px-3 py-2">
              <button type="button" className="w-full text-left" onClick={() => onParams(sanitizeParams(v.params))}>
                <span className="block text-sm text-fg">{v.name}</span>
                <span className="block text-[11px] text-muted">
                  L/D {fmt(v.ld, 2)} · CD {fmt(v.cd, 3)} · q {fmt(v.q, 2)} · m {fmt(v.mass, 0)} kg · SM {fmt(v.sm, 3)} · R {fmt(v.rangeKm, 0)} km
                </span>
              </button>
            </div>
          ))}
          {history.length ? <p className="hud-label mt-2">History</p> : null}
          {history.map((h) => (
            <button key={h.at} type="button" className="block w-full text-left text-[11px] text-muted" onClick={() => onParams(sanitizeParams(h.params))}>
              {new Date(h.at).toLocaleString()} — {h.name}
            </button>
          ))}
          <button
            type="button"
            className="hud-card w-full rounded-md px-3 py-2 text-xs"
            onClick={() => {
              const deck = genericDeck(built);
              const su2 = su2Config(built);
              const foam = openFoamFiles(built).map((f) => `===== ${f.name} =====\n${f.content}`).join("\n");
              const py = python6dofSource();
              downloadBlob(new Blob([deck, "\n\n", su2, "\n\n", foam, "\n\n", py], { type: "text/plain" }), `${params.name}_cfd_deck.txt`);
            }}
          >
            Download SU2 / OpenFOAM / Python deck
          </button>
        </div>
      ) : null}
    </div>
  );
}

function AxisRange({
  label,
  a,
  b,
  s,
  onA,
  onB,
  onS,
}: {
  label: string;
  a: number;
  b: number;
  s: number;
  onA: (v: number) => void;
  onB: (v: number) => void;
  onS: (v: number) => void;
}) {
  return (
    <div>
      <span className="hud-label mb-1 block !text-muted">{label}</span>
      <div className="grid grid-cols-3 gap-1.5">
        <AxisBox k="min" value={a} onChange={onA} />
        <AxisBox k="max" value={b} onChange={onB} />
        <AxisBox k="step" value={s} onChange={onS} />
      </div>
    </div>
  );
}

function AxisBox({ k, value, onChange }: { k: string; value: number; onChange: (v: number) => void }) {
  return (
    <label className="block">
      <span className="mb-0.5 block font-display text-[9px] font-semibold tracking-[0.12em] text-subtle uppercase">{k}</span>
      <input
        type="number"
        inputMode="decimal"
        step="any"
        value={Number.isFinite(value) ? value : 0}
        onChange={(e) => {
          const n = parseFloat(e.target.value);
          if (Number.isFinite(n)) onChange(n);
        }}
        className="h-8 w-full rounded-sm border-0 bg-surface-2 px-1.5 text-right font-mono text-xs tabular-nums text-fg"
      />
    </label>
  );
}

/** Round-off (|v| < 1e-9, e.g. CY at β = 0) prints as 0. */
function tidy(v: number): number {
  return Math.abs(v) < 1e-9 ? 0 : v;
}

function AxisList({ label, value, placeholder, onChange }: { label: string; value: string; placeholder: string; onChange: (text: string) => void }) {
  return (
    <label className="block">
      <span className="hud-label mb-1 block !text-muted">{label}</span>
      <input
        type="text"
        inputMode="decimal"
        value={value}
        placeholder={placeholder}
        onChange={(e) => onChange(e.target.value)}
        className="h-8 w-full rounded-sm border-0 bg-surface-2 px-1.5 font-mono text-xs tabular-nums text-fg placeholder:text-subtle"
      />
    </label>
  );
}

function MachChart({ title, xLabel, series }: { title: string; xLabel: string; series: { rows: SeriesRow[]; keys: string[] } }) {
  return (
    <div>
      <span className="hud-label mb-1 block !text-muted">{title}</span>
      <div className="h-40">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={series.rows} margin={{ top: 4, right: 8, left: 0, bottom: 0 }}>
            <CartesianGrid stroke="rgba(243,239,230,0.08)" />
            <XAxis dataKey="x" type="number" domain={["dataMin", "dataMax"]} tick={{ fill: "#b9b3a8", fontSize: 10 }} />
            <YAxis tick={{ fill: "#b9b3a8", fontSize: 10 }} width={44} tickFormatter={(v: number) => fmt(v, 2)} />
            <Tooltip {...chartTip(xLabel)} formatter={(v: number) => fmt(v, 4)} />
            <Legend wrapperStyle={{ fontSize: 10 }} />
            {series.keys.map((k, i) => (
              <Line key={k} type="monotone" dataKey={k} stroke={MACH_COLORS[i % MACH_COLORS.length]} dot={false} name={k} strokeWidth={1.5} connectNulls />
            ))}
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}

function bounds(key: string): { min: number; max: number } {
  const table: Record<string, [number, number]> = {
    mach: [4, 12],
    length: [2, 8],
    span: [0.8, 4],
    height: [0.15, 1.2],
    shockDeg: [8, 24],
    coneDeg: [4, 16],
    superN: [1.2, 4],
    planformPower: [0.8, 1.8],
    fins: [3, 6],
    wedgeFrac: [0.2, 0.7],
    captureFrac: [0.3, 0.8],
    dihedralDeg: [-4, 10],
    camber: [-0.1, 0.25],
    elevonDeg: [-6, 6],
    finHeight: [0, 0.8],
    cgFrac: [0.45, 0.7],
    leRadius: [0.005, 0.08],
    alphaDeg: [-2, 8],
    altKm: [20, 50],
  };
  const pair = table[key] ?? [0, 1];
  return { min: pair[0], max: pair[1] };
}

function mergeSlices(db: AeroDatabase) {
  return db.slices.design.alpha.map((row, i) => ({
    alpha: row.alpha,
    cl: row.cl,
    cm: row.cm,
    clOff: db.slices.offDesign.alpha[i]?.cl ?? row.cl,
  }));
}
