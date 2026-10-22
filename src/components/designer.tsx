import { useEffect, useMemo, useRef, useState } from "react";
import {
  Box,
  Check,
  ChevronDown,
  Download,
  FileCode2,
  Grid3x3,
  Layers,
  Octagon,
  RotateCcw,
} from "lucide-react";
import { Button } from "@/components/ui/button";
import { Switch } from "@/components/ui/switch";
import { WaveriderViewer, fieldScale, type ViewerOpts } from "@/components/viewer";
import { NumberField, Seg, Stat } from "@/components/fields";
import { RefinementControls } from "@/components/refinement-controls"; // [mesh-refinement]
import { AeroBlock, ChecksBlock, CycleBlock, FrontierBlock, HeatBlock, MassBlock, ShocksBlock, SixDofBlock, StabBlock, TrajBlock } from "@/components/analysis-panel";
import { DataPanel } from "@/components/data-panel";
import { DatcomBlock } from "@/components/datcom-panel";
import { HudTabs } from "@/components/ui/hud-tabs"; // [ui-redesign]
import { Readout, ViewportChrome } from "@/components/ui/hud"; // [ui-redesign]
import { computeDataPackAsync } from "@/lib/waverider/datapack";
import { buildVehicle } from "@/lib/waverider/generate";
import { safeBuild, studyProgressive } from "@/lib/waverider/progressive-study"; // [mesh-refinement]
import { PanelBoundary } from "@/components/error-boundary"; // [mesh-refinement]
import { useStudy } from "@/components/use-study"; // [mesh-refinement]
import { cfdZip, fileBlobs } from "@/lib/waverider/export-kit";
import kernelSource from "../../native/bowshock_kernel.c?raw";
import { fmt } from "@/lib/waverider/math";
import { studyJson, studyVehicle, type StudyResult } from "@/lib/waverider/study";
import { loadSavedParams, PRESETS, saveParams } from "@/lib/waverider/presets";
import {
  COLOR_MODES,
  DEFAULT_PARAMS,
  FAMILY_GROUPS,
  FAMILY_META,
  domainOf,
  type AeroMethod,
  type ColorMode,
  type CowlSide,
  type DesignParams,
  type FlowDomain,
  type BuiltVehicle,
  type FuelKind,
  type LengthUnit,
  type LidFace,
  type PlanformKind,
  type WaveriderFamily,
} from "@/lib/waverider/types";
import { downloadBlob } from "@/lib/utils";

type Tab = "geom" | "flight" | "aero" | "heat" | "frontier" | "stab" | "sixdof" | "traj" | "cycle" | "shocks" | "checks" | "data" | "cad" | "datcom";

function LiveField(
  props: Omit<React.ComponentProps<typeof NumberField>, "onDragStart" | "onDragEnd"> & {
    setDragging: (v: boolean) => void;
  },
) {
  const { setDragging, ...rest } = props;
  return <NumberField {...rest} onDragStart={() => setDragging(true)} onDragEnd={() => setDragging(false)} />;
}

function tabsFor(domain: FlowDomain): { id: Tab; label: string }[] {
  if (domain === "internal") {
    return [
      { id: "geom", label: "Geom" },
      { id: "cycle", label: "Cycle" },
      { id: "shocks", label: "Shocks" },
      { id: "heat", label: "Heat" },
      { id: "frontier", label: "Frontier" },
      { id: "sixdof", label: "6DOF" },
      { id: "datcom", label: "DATCOM" },
      { id: "checks", label: "Checks" },
      { id: "data", label: "Data" },
      { id: "cad", label: "CAD" },
    ];
  }
  return [
    { id: "geom", label: "Geom" },
    { id: "flight", label: "Flight" },
    { id: "aero", label: "Aero" },
    { id: "heat", label: "Heat" },
    { id: "frontier", label: "Frontier" },
    { id: "stab", label: "Stab" },
    { id: "sixdof", label: "6DOF" },
    { id: "datcom", label: "DATCOM" },
    { id: "traj", label: "Traj" },
    { id: "checks", label: "Checks" },
    { id: "data", label: "Data" },
    { id: "cad", label: "CAD" },
  ];
}

function solveCaption(
  method: string,
  eulerOn: boolean,
  status: {
    busy: boolean;
    stage: string;
    fraction: number;
    error?: string;
    warnings: string[];
    report: { pending: boolean; lower?: { residual: number; converged: boolean } } | null;
  },
): { text: string; warn: boolean } {
  if (status.error) return { text: `Solve failed (${status.error}). Instant numbers kept.`, warn: true };
  if (status.warnings.length) return { text: status.warnings[0]!, warn: true };
  if (eulerOn && (status.busy || status.stage === "preview" || status.stage === "solving" || status.report?.pending)) {
    if (status.stage === "solving") return { text: `Euler ${Math.round(status.fraction * 100)}% · CBAERO until it finishes`, warn: false };
    return { text: "Euler queued · CBAERO on screen", warn: false };
  }
  const lo = status.report?.lower;
  if (eulerOn && lo && Number.isFinite(lo.residual)) {
    return { text: `Euler · res ${lo.residual.toExponential(1)}${lo.converged ? "" : " · open"}`, warn: !lo.converged };
  }
  const names: Record<string, string> = {
    newtonian: "Newtonian",
    tangent: "Tangent wedge",
    mixed: "Mixed",
    cbaero: "CBAERO",
    euler: "Euler",
  };
  return { text: names[method] ?? method, warn: false };
}

function ScaleLegend({
  study,
  color,
  surfaces,
}: {
  study: StudyResult;
  color: ColorMode;
  surfaces: Uint8Array;
}) {
  const scale = fieldScale(study, color, surfaces);
  const fmtScale = (v: number) => (v >= 10 ? v.toFixed(0) : v >= 1 ? v.toFixed(2) : v.toFixed(3));
  return (
    <div className="flex min-w-0 items-center gap-2" title={scale.unit || color}>
      <span className="font-mono text-[10px] text-muted">{fmtScale(scale.lo)}</span>
      <div className="h-2 w-28 shrink-0 overflow-hidden rounded-sm bg-bg">
        <span className="jet-ramp-h block h-full w-full" />
      </div>
      <span className="truncate font-mono text-[10px] text-muted">
        {fmtScale(scale.hi)} {scale.unit}
        {scale.log ? " log" : ""}
      </span>
    </div>
  );
}

function geomKey(p: DesignParams) {
  return [
    p.family,
    p.mach,
    p.gamma,
    p.length,
    p.span,
    p.height,
    p.shockDeg,
    p.coneDeg,
    p.superN,
    p.planform,
    p.planformPower,
    p.fins,
    p.wedgeFrac,
    p.captureFrac,
    p.nx,
    p.ny,
    JSON.stringify(p.refinement ?? null), // [mesh-refinement] re-build (and re-solve) when the refinement changes
    p.leRadius,
    p.leBlunt,
    p.leBlend ?? "circular",
    p.halfModel,
    p.lid,
    p.dihedralDeg,
    p.camber,
    p.teSweepDeg,
    p.elevonDeg,
    p.finHeight,
    p.nRamps,
    p.cowlSide,
    p.flowThrough,
    p.inletHeight,
    p.cowlFrac,
    p.combustorFrac,
    p.nozzleER,
    p.rampDeg,
  ].join("|");
}

export function Designer() {
  const [params, setParams] = useState<DesignParams>(DEFAULT_PARAMS);
  const [hydrated, setHydrated] = useState(false);
  const [opts, setOpts] = useState<ViewerOpts>({ wireframe: false, shock: true, grid: true, origin: true, color: "surface" });
  const [busy, setBusy] = useState<string | null>(null);
  const [python, setPython] = useState<string>("");
  const [cSrc, setCSrc] = useState<string>("");
  const [cppSrc, setCppSrc] = useState<string>("");
  const [presetOpen, setPresetOpen] = useState(false);
  const [tab, setTab] = useState<Tab>("geom");
  const [dragging, setDragging] = useState(false);
  const pending = useRef<Partial<DesignParams> | null>(null);
  const raf = useRef(0);

  useEffect(() => {
    const saved = loadSavedParams();
    if (saved) setParams(saved);
    setHydrated(true);
    fetch("/waverider_cad.py")
      .then((r) => r.text())
      .then(setPython)
      .catch(() => setPython(""));
    fetch("/bowshock_aero.c")
      .then((r) => r.text())
      .then(setCSrc)
      .catch(() => setCSrc(""));
    fetch("/bowshock.cpp")
      .then((r) => r.text())
      .then(setCppSrc)
      .catch(() => setCppSrc(""));
  }, []);

  useEffect(() => {
    if (hydrated) saveParams(params);
  }, [params, hydrated]);

  useEffect(() => {
    const ids = tabsFor(domainOf(params.family)).map((t) => t.id);
    if (!ids.includes(tab)) setTab("geom");
  }, [params.family, tab]);

  const patch = (p: Partial<DesignParams>) => {
    pending.current = { ...pending.current, ...p };
    if (raf.current) return;
    raf.current = requestAnimationFrame(() => {
      raf.current = 0;
      const next = pending.current;
      pending.current = null;
      if (next) setParams((prev) => ({ ...prev, ...next }));
    });
  };

  const gKey = geomKey(params);
  // [mesh-refinement] safe build (a shape the generator rejects keeps the last good vehicle) + non-blocking study
  const lastGood = useRef<BuiltVehicle | null>(null);
  const builtRes = useMemo(() => safeBuild(params, lastGood.current), [gKey]); // eslint-disable-line react-hooks/exhaustive-deps
  const built = builtRes.built;
  if (!builtRes.error) lastGood.current = built;
  const studyDeps = [
    params.flightMach,
    params.lockFlight,
    params.alphaDeg,
    params.betaDeg,
    params.altKm,
    params.cgFrac,
    params.aeroMethod,
    params.twK,
    params.fuel,
    params.phi,
    params.gammaDeg,
    params.rhoKgM3,
    params.massKg,
  ];
  // built.params is frozen at the last *geometry* change; alpha, altitude, Mach, method etc. are not in geomKey,
  // so hand the study the live params or it (and the Data tab) read stale values.
  // eslint-disable-next-line react-hooks/exhaustive-deps
  const studyBuilt = useMemo(() => ({ ...built, params }), [built, ...studyDeps]);
  const { study, status: studyStatus } = useStudy(
    studyBuilt,
    studyDeps,
    dragging,
    hydrated,
  );
  const q = built.quality;
  const a = built.aero;
  const fam = FAMILY_META[params.family];
  const domain = domainOf(params.family);
  const solveLine = solveCaption(study.aero.method, params.aeroMethod === "euler", studyStatus);
  const TABS = tabsFor(domain);

  function goDomain(d: FlowDomain) {
    if (d === domain) return;
    const pick = d === "external" ? PRESETS.find((p) => p.id === "caret-m8") : PRESETS.find((p) => p.id === "ramjet");
    if (pick) {
      setParams({
        ...pick.params,
        flowThrough: d === "internal" && (pick.params.family === "ramjet" || pick.params.family === "scramjet"),
      });
    }
    setTab("geom");
  }

  function pickFamily(id: WaveriderFamily) {
    const meta = FAMILY_META[id];
    patch({
      family: id,
      name: `${id}_${params.mach}`,
      flowThrough: meta.domain === "internal" && (id === "ramjet" || id === "scramjet"),
    });
  }

  async function save(kind: string) {
    setBusy(kind);
    try {
      const cadKinds = new Set(["stl", "stl-ascii", "stl-regions", "step", "step-nurbs", "step-tess", "step-facet", "iges", "plot3d", "obj", "kit"]);
      const src =
        cadKinds.has(kind)
          ? safeBuild({ ...params, nx: Math.max(params.nx, 96), ny: Math.max(params.ny, 72) }, built).built
          : built;
      const aero = cadKinds.has(kind) && src !== built ? (await studyProgressive(src, { quick: true })).study.aero : study.aero; // [mesh-refinement] non-blocking
      const files = fileBlobs(src, {
        cp: aero.cp,
        heat: aero.heat,
        twEq: aero.twEq,
        stanton: aero.stanton,
        machE: aero.machE,
        cf: aero.cf,
        impact: aero.impact,
      });
      const n = files.name;
      if (kind === "stl") downloadBlob(files.stlBin, `${n}.stl`);
      else if (kind === "stl-ascii") downloadBlob(files.stlAscii, `${n}_ascii.stl`);
      else if (kind === "stl-regions") downloadBlob(files.stlRegions, `${n}_regions.stl`);
      else if (kind === "step") downloadBlob(files.stepNurbs, `${n}_nurbs.step`);
      else if (kind === "step-nurbs") downloadBlob(files.stepNurbs, `${n}_nurbs.step`);
      else if (kind === "step-tess") downloadBlob(files.stepTess, `${n}_tess.step`);
      else if (kind === "step-facet") downloadBlob(files.stepFacet, `${n}_faceted.step`);
      else if (kind === "iges") downloadBlob(files.iges, `${n}.igs`);
      else if (kind === "plot3d") downloadBlob(files.plot3d, `${n}.x`);
      else if (kind === "obj") downloadBlob(files.obj, `${n}.obj`);
      else if (kind === "vtk") downloadBlob(files.vtk, `${n}.vtk`);
      else if (kind === "glyph") downloadBlob(files.glyph, `${n}.glf`);
      else if (kind === "json") downloadBlob(files.json, `${n}.json`);
      else if (kind === "analysis") downloadBlob(new Blob([studyJson(built, study)], { type: "application/json" }), `${n}_analysis.json`);
      else if (kind === "python") {
        downloadBlob(files.runner, `run_case.py`);
        if (python) downloadBlob(new Blob([python], { type: "text/x-python" }), "waverider_cad.py");
        if (cSrc) downloadBlob(new Blob([cSrc], { type: "text/x-csrc" }), "bowshock_aero.c");
        if (kernelSource) downloadBlob(new Blob([kernelSource], { type: "text/x-csrc" }), "bowshock_kernel.c");
        if (cppSrc) downloadBlob(new Blob([cppSrc], { type: "text/x-c++src" }), "bowshock.cpp");
      } else if (kind === "kit") {
        const pack = await computeDataPackAsync(params);
        const zip = await cfdZip(
          src,
          python,
          cSrc,
          studyJson(src, (await studyProgressive(src, { quick: true })).study),
          cppSrc,
          {
            cp: aero.cp,
            heat: aero.heat,
            twEq: aero.twEq,
            stanton: aero.stanton,
            machE: aero.machE,
            cf: aero.cf,
            impact: aero.impact,
          },
          kernelSource,
          pack,
        );
        downloadBlob(zip, `${n}_cfd_kit.zip`);
      }
    } catch (err) {
      console.error("[export]", err);
      window.alert(`Export failed: ${err instanceof Error ? err.message : String(err)}`); // [mesh-refinement]
    } finally {
      setBusy(null);
    }
  }

  return (
    <div className="flex h-dvh max-h-dvh flex-col overflow-hidden bg-bg text-fg">
      <header className="hud-header flex shrink-0 flex-wrap items-center justify-between gap-3 px-4 py-3 sm:px-6">
        <div className="min-w-0">
          <p className="hud-wordmark text-xl">CUSPIS</p>
          <p className="hud-label truncate !text-muted">
            {domain === "external" ? "Roma · hypersonic CAD · external" : "Roma · hypersonic CAD · internal"}
          </p>
        </div>
        <div className="flex shrink-0 items-center gap-2">
          <Button variant="ghost" size="sm" className="hidden sm:inline-flex" onClick={() => setParams(DEFAULT_PARAMS)}>
            <RotateCcw />
            Reset
          </Button>
          <Button onClick={() => save("kit")} disabled={!!busy}>
            <Download />
            {busy === "kit" ? "Packing…" : "CFD kit"}
          </Button>
        </div>
      </header>

      <div className="grid min-h-0 w-full flex-1 grid-cols-1 grid-rows-[minmax(0,1.15fr)_minmax(0,0.85fr)_minmax(0,1fr)] overflow-hidden lg:grid-cols-[272px_minmax(0,1fr)_360px] lg:grid-rows-1">
        <aside className="hud-rail order-2 min-h-0 overflow-y-auto overscroll-contain border-t border-line lg:order-1 lg:border-t-0 lg:border-r">
          <div className="p-4 sm:p-5">
            <p className="hud-label hud-label-tick mb-2">Toolkit</p>
            <Seg
              value={domain}
              onChange={(d: FlowDomain) => goDomain(d)}
              options={[
                { id: "external", label: "External" },
                { id: "internal", label: "Internal" },
              ]}
            />
            <p className="hud-label hud-label-tick mt-4 mb-2">
              {domain === "external" ? "Vehicles" : "Inlets / engines"}
            </p>
            <div className="flex gap-2 overflow-x-auto pb-1 lg:flex-col lg:overflow-visible">
              {(FAMILY_GROUPS.find((g) => g.id === domain)?.families ?? []).map((id) => {
                const meta = FAMILY_META[id];
                const on = params.family === id;
                return (
                  <button
                    key={id}
                    type="button"
                    onClick={() => pickFamily(id)}
                    aria-pressed={on}
                    className="hud-card min-h-11 min-w-[9.5rem] shrink-0 rounded-md px-3 py-2 transition-colors lg:min-w-0"
                  >
                    <span className="hud-card-title block text-sm">{meta.label}</span>
                    <span className="mt-0.5 block text-[11px] leading-snug text-muted">
                      {meta.generating}
                    </span>
                  </button>
                );
              })}
            </div>

            <button
              type="button"
              className="hud-label hud-plain mt-5 flex min-h-9 w-full items-center justify-between"
              aria-expanded={presetOpen}
              onClick={() => setPresetOpen((v) => !v)}
            >
              Presets
              <ChevronDown className={`size-4 transition-transform ${presetOpen ? "rotate-180" : ""}`} />
            </button>
            {presetOpen ? (
              <div className="mt-2 grid gap-2">
                {PRESETS.filter((pr) => domainOf(pr.params.family) === domain).map((pr) => (
                  <button
                    key={pr.id}
                    type="button"
                    onClick={() => setParams({ ...pr.params })}
                    className="hud-card rounded-md px-3 py-2"
                  >
                    <span className="block text-sm text-fg">{pr.name}</span>
                    <span className="block text-[11px] text-muted">{pr.blurb}</span>
                  </button>
                ))}
              </div>
            ) : null}
          </div>
        </aside>

        <section className="hud-viewport relative order-1 flex min-h-0 flex-col overflow-hidden lg:order-2">
          <div className="z-10 flex shrink-0 flex-col gap-1.5 border-b border-line bg-bg/95 px-3 py-2">
            <div className="flex min-w-0 flex-wrap items-center gap-x-2 gap-y-1.5">
              <div className="hud-chip-info flex min-w-0 items-center gap-1 px-2.5 py-1 font-mono text-[11px] text-muted">
                <span className="hud-led mr-1.5 shrink-0" aria-hidden />
                <span className="font-display font-semibold tracking-[0.1em] text-accent uppercase">{fam.label}</span>
                <span className="mx-1.5 text-subtle">/</span>
                {domain}
                <span className="mx-1.5 text-subtle">/</span>
                M {params.lockFlight ? params.mach : params.flightMach}
              </div>
              <p
                data-testid="study-status"
                className={`min-w-0 flex-1 truncate text-[11px] leading-snug ${solveLine.warn ? "text-warn" : "text-muted"}`}
                title={solveLine.text}
              >
                {solveLine.text}
              </p>
            </div>
            {builtRes.error ? (
              <p className="min-w-0 text-[11px] leading-snug text-warn">{builtRes.error} Showing the last valid shape.</p>
            ) : null}
            <div className="flex gap-1 overflow-x-auto" role="group" aria-label="View modes">
              {(
                [
                  ["wireframe", "Wires", opts.wireframe],
                  ["shock", "Shock", opts.shock],
                  ["grid", "Grid", opts.grid],
                  ["origin", "Origin", opts.origin],
                ] as const
              ).map(([key, label, on]) => (
                <button
                  key={key}
                  type="button"
                  onClick={() => setOpts((o) => ({ ...o, [key]: !o[key] }))}
                  aria-pressed={on}
                  className="hud-toggle shrink-0 rounded-sm"
                >
                  {label}
                </button>
              ))}
            </div>
          </div>
          <div className="relative min-h-0 flex-1">
            <PanelBoundary label="3D viewer" resetKey={built}>
              <WaveriderViewer built={built} study={study} opts={opts} />
            </PanelBoundary>
            <ViewportChrome />
          </div>
          <div className="z-10 flex shrink-0 flex-col gap-2 border-t border-line bg-bg/95 px-3 py-2">
            <div className="flex gap-1 overflow-x-auto" role="group" aria-label="Surface colour mode">
              {COLOR_MODES.map((c) => (
                <button
                  key={c.id}
                  type="button"
                  onClick={() => setOpts((o) => ({ ...o, color: c.id }))}
                  aria-pressed={opts.color === c.id}
                  className="hud-toggle hud-toggle--sm shrink-0 rounded-sm"
                >
                  {c.label}
                </button>
              ))}
            </div>
            <div className="flex items-end justify-between gap-3">
              {opts.color !== "surface" ? (
                <ScaleLegend study={study} color={opts.color} surfaces={built.mesh.surfaces} />
              ) : (
                <span className="text-[11px] text-subtle">Surface colour</span>
              )}
              <div className="hud-chip-info flex shrink-0 items-end gap-4 px-3 py-1.5">
                <Readout label="L/D" value={fmt(study.aero.ld, 2)} />
                <Readout size="sm" label="CoP" value={fmt(study.aero.xCp / Math.max(params.length, 1e-8), 2)} unit="L" />
                {hydrated ? <Readout size="sm" label="Solve" value={fmt(study.elapsedMs, 0)} unit="ms" /> : null}
              </div>
            </div>
          </div>
        </section>

        <aside className="hud-rail order-3 flex min-h-0 flex-col overflow-hidden border-t border-line lg:border-t-0 lg:border-l">
          <HudTabs tabs={TABS} value={tab} onChange={setTab} />
          <div className="min-h-0 flex-1 overflow-y-auto overscroll-contain p-4 sm:p-5">
            {!hydrated ? (
              <p className="text-sm leading-relaxed text-muted">{fam.blurb}</p>
            ) : (
              <PanelBoundary label="This panel" resetKey={`${tab}|${gKey}`}>
            {tab === "geom" ? (
              <div className="space-y-4">
                <p className="hidden text-sm leading-relaxed text-muted lg:block">{fam.blurb}</p>
                <LiveField setDragging={setDragging} label="Design Mach" value={params.mach} min={3} max={16} step={0.1} onChange={(mach) => patch({ mach })} digits={1} />
                <LiveField setDragging={setDragging} label="Length" value={params.length} min={0.4} max={20} step={0.1} unit="m" onChange={(length) => patch({ length })} />
                <LiveField setDragging={setDragging} label="Span" value={params.span} min={0.15} max={12} step={0.05} unit="m" onChange={(span) => patch({ span })} />
                <LiveField setDragging={setDragging} label="Height" value={params.height} min={0.05} max={4} step={0.01} unit="m" onChange={(height) => patch({ height })} />
                {domain === "external" || params.family === "integrated" || params.family === "inward" || params.family === "busemann" ? (
                  <>
                <div>
                  <span className="hud-label mb-2 block !text-muted">Flat lid</span>
                  <Seg
                    value={params.lid}
                    onChange={(lid: LidFace) => patch({ lid })}
                    options={[
                      { id: "top", label: "On top" },
                      { id: "bottom", label: "On bottom" },
                    ]}
                  />
                </div>
                <LiveField setDragging={setDragging}
                  label="Dihedral"
                  value={params.dihedralDeg}
                  min={-8}
                  max={18}
                  step={0.5}
                  unit="deg"
                  digits={1}
                  onChange={(dihedralDeg) => patch({ dihedralDeg })}
                />
                <LiveField setDragging={setDragging}
                  label="Lid camber"
                  value={params.camber}
                  min={-0.25}
                  max={0.4}
                  step={0.01}
                  onChange={(camber) => patch({ camber })}
                />
                <LiveField setDragging={setDragging}
                  label="TE sweep"
                  value={params.teSweepDeg}
                  min={-5}
                  max={25}
                  step={0.5}
                  unit="deg"
                  digits={1}
                  onChange={(teSweepDeg) => patch({ teSweepDeg })}
                />
                <LiveField setDragging={setDragging}
                  label="Elevon"
                  value={params.elevonDeg}
                  min={-12}
                  max={12}
                  step={0.25}
                  unit="deg"
                  digits={1}
                  onChange={(elevonDeg) => patch({ elevonDeg })}
                />
                <LiveField setDragging={setDragging}
                  label="Vert. fins h/H"
                  value={params.finHeight}
                  min={0}
                  max={1.4}
                  step={0.05}
                  onChange={(finHeight) => patch({ finHeight })}
                />
                  </>
                ) : null}
                <LiveField setDragging={setDragging} label="γ gas" value={params.gamma} min={1.2} max={1.67} step={0.01} onChange={(gamma) => patch({ gamma })} />
                {params.family === "caret" || params.family === "star" ? (
                  <LiveField setDragging={setDragging} label="Shock β" value={params.shockDeg} min={8} max={40} step={0.1} unit="deg" onChange={(shockDeg) => patch({ shockDeg })} digits={1} />
                ) : null}
                {params.family === "cone" ? (
                  <>
                    <LiveField setDragging={setDragging} label="Cone half-angle" value={params.coneDeg} min={3} max={20} step={0.1} unit="deg" onChange={(coneDeg) => patch({ coneDeg })} digits={1} />
                    <LiveField setDragging={setDragging} label="Capture cut" value={params.captureFrac} min={0.15} max={0.8} step={0.01} onChange={(captureFrac) => patch({ captureFrac })} />
                  </>
                ) : null}
                {params.family === "osculating" ||
                params.family === "wedgecone" ||
                params.family === "viscopt" ||
                params.family === "inward" ||
                params.family === "elliptic" ||
                params.family === "integrated" ||
                params.family === "liftbody" ? (
                  <>
                    {params.family !== "liftbody" && params.family !== "elliptic" ? (
                      <LiveField setDragging={setDragging} label="Superellipse n" value={params.superN} min={1} max={6} step={0.05} onChange={(superN) => patch({ superN })} />
                    ) : null}
                    <div>
                      <span className="hud-label mb-2 block !text-muted">Planform</span>
                      <div className="grid grid-cols-3 gap-1.5">
                        {(["delta", "power", "spatular", "rect", "double"] as PlanformKind[]).map((pl) => (
                          <button
                            key={pl}
                            type="button"
                            onClick={() => patch({ planform: pl })}
                            aria-pressed={params.planform === pl}
                            className={`h-9 rounded-sm text-xs capitalize ${params.planform === pl ? "bg-accent text-accent-fg" : "bg-surface-2 text-muted"}`}
                          >
                            {pl}
                          </button>
                        ))}
                      </div>
                    </div>
                    {params.planform === "power" || params.planform === "double" || params.family === "viscopt" ? (
                      <LiveField setDragging={setDragging} label="LE / stream power" value={params.planformPower} min={0.7} max={2.2} step={0.05} onChange={(planformPower) => patch({ planformPower })} />
                    ) : null}
                  </>
                ) : null}
                {params.family === "wedgecone" || params.family === "inward" || params.family === "integrated" ? (
                  <LiveField setDragging={setDragging} label="Center width" value={params.wedgeFrac} min={0.1} max={0.85} step={0.01} onChange={(wedgeFrac) => patch({ wedgeFrac })} />
                ) : null}
                {params.family === "star" ? (
                  <LiveField setDragging={setDragging} label="Fins" value={params.fins} min={3} max={8} step={1} digits={0} onChange={(fins) => patch({ fins: Math.round(fins) })} />
                ) : null}
                {params.family === "ramjet" || params.family === "scramjet" || params.family === "integrated" ? (
                  <>
                    <LiveField setDragging={setDragging} label="Inlet height" value={params.inletHeight} min={0.04} max={0.6} step={0.01} unit="m" onChange={(inletHeight) => patch({ inletHeight })} />
                    <LiveField setDragging={setDragging} label="Ramp" value={params.rampDeg} min={4} max={18} step={0.5} unit="deg" digits={1} onChange={(rampDeg) => patch({ rampDeg })} />
                    <LiveField setDragging={setDragging}
                      label="Ramps"
                      value={params.nRamps}
                      min={1}
                      max={3}
                      step={1}
                      digits={0}
                      onChange={(nRamps) => patch({ nRamps: Math.round(nRamps) })}
                    />
                    <LiveField setDragging={setDragging} label="Cowl x/L" value={params.cowlFrac} min={0.2} max={0.7} step={0.01} onChange={(cowlFrac) => patch({ cowlFrac })} />
                    <LiveField setDragging={setDragging} label="Combustor L/L" value={params.combustorFrac} min={0.1} max={0.45} step={0.01} onChange={(combustorFrac) => patch({ combustorFrac })} />
                    <LiveField setDragging={setDragging} label="Nozzle ER" value={params.nozzleER} min={1.5} max={12} step={0.1} onChange={(nozzleER) => patch({ nozzleER })} digits={1} />
                    {params.family === "integrated" ? (
                      <div>
                        <span className="hud-label mb-2 block !text-muted">Cowl</span>
                        <Seg
                          value={params.cowlSide}
                          onChange={(cowlSide: CowlSide) => patch({ cowlSide })}
                          options={[
                            { id: "belly", label: "Belly" },
                            { id: "dorsal", label: "Dorsal" },
                          ]}
                        />
                      </div>
                    ) : null}
                    {params.family === "ramjet" || params.family === "scramjet" ? (
                      <div className="flex items-center justify-between gap-3 pt-1">
                        <span className="hud-label !text-muted">Flow-through (open duct)</span>
                        <Switch checked={params.flowThrough} onCheckedChange={(flowThrough) => patch({ flowThrough })} />
                      </div>
                    ) : null}
                  </>
                ) : null}
                <LiveField setDragging={setDragging} label="Streamwise pts" value={params.nx} min={12} max={160} step={1} digits={0} onChange={(nx) => patch({ nx: Math.round(nx) })} />
                <LiveField setDragging={setDragging} label="Spanwise pts" value={params.ny} min={10} max={120} step={1} digits={0} onChange={(ny) => patch({ ny: Math.round(ny) })} />
                <div>
                  <span className="hud-label mb-2 block !text-muted">Resolution</span>
                  <Seg
                    value={params.nx >= 90 ? "cad" : params.nx >= 56 ? "hi" : params.nx <= 24 ? "fast" : "std"}
                    onChange={(r: "fast" | "std" | "hi" | "cad") => {
                      if (r === "fast") patch({ nx: 20, ny: 14 });
                      else if (r === "hi") patch({ nx: 64, ny: 48 });
                      else if (r === "cad") patch({ nx: 96, ny: 72 });
                      else patch({ nx: 40, ny: 28 });
                    }}
                    options={[
                      { id: "fast", label: "Fast" },
                      { id: "std", label: "Std" },
                      { id: "hi", label: "Hi" },
                      { id: "cad", label: "CAD" },
                    ]}
                  />
                </div>
                <RefinementControls params={params} patch={patch} setDragging={setDragging} />
                <div className="flex items-center justify-between gap-3 pt-1">
                  <span className="hud-label !text-muted">Rounded leading edge</span>
                  <Switch
                    checked={params.leBlunt !== false}
                    onCheckedChange={(leBlunt) =>
                      patch({
                        leBlunt,
                        leRadius: leBlunt ? Math.max(params.leRadius, 0.005 * params.length) : 0,
                      })
                    }
                  />
                </div>
                {params.leBlunt !== false ? (
                  <>
                    <LiveField setDragging={setDragging}
                      label="Nose / LE radius"
                      value={params.leRadius}
                      min={0.001 * params.length}
                      max={Math.max(0.08, params.length * 0.04)}
                      step={0.001}
                      unit="m"
                      digits={3}
                      onChange={(leRadius) => patch({ leRadius, leBlunt: true })}
                    />
                    <div>
                      <span className="hud-label mb-2 block !text-muted">LE blend</span>
                      <Seg
                        value={params.leBlend ?? "circular"}
                        onChange={(leBlend: "circular" | "g2") => patch({ leBlend, leBlunt: true })}
                        options={[
                          { id: "circular", label: "Circular" },
                          { id: "g2", label: "G2" },
                        ]}
                      />
                      <p className="mt-1.5 text-[11px] leading-snug text-subtle">
                        Circular is the default fillet (G1 at the seam). G2 matches curvature too, but on a slender wedge the nose stays sharper than the radius you set.
                      </p>
                    </div>
                  </>
                ) : (
                  <p className="text-[11px] text-subtle">Sharp knife-edge — inviscid theory only. Pointwise needs a radius.</p>
                )}
                <div className="flex items-center justify-between gap-3 pt-1">
                  <span className="hud-label !text-muted">Half-model (Y≥0)</span>
                  <Switch checked={params.halfModel} onCheckedChange={(halfModel) => patch({ halfModel })} />
                </div>
              </div>
            ) : null}

            {tab === "flight" ? (
              <div className="space-y-4">
                <div className="flex items-center justify-between gap-3">
                  <span className="hud-label !text-muted">Lock flight Mach = design</span>
                  <Switch checked={params.lockFlight} onCheckedChange={(lockFlight) => patch({ lockFlight })} />
                </div>
                <LiveField setDragging={setDragging}
                  label="Flight Mach"
                  value={params.lockFlight ? params.mach : params.flightMach}
                  min={2}
                  max={20}
                  step={0.1}
                  digits={1}
                  onChange={(flightMach) => patch({ flightMach, lockFlight: false })}
                />
                <LiveField setDragging={setDragging} label="α" value={params.alphaDeg} min={-12} max={22} step={0.05} unit="deg" digits={2} onChange={(alphaDeg) => patch({ alphaDeg })} />
                <LiveField setDragging={setDragging} label="β" value={params.betaDeg} min={-8} max={8} step={0.05} unit="deg" digits={2} onChange={(betaDeg) => patch({ betaDeg })} />
                <LiveField setDragging={setDragging} label="Altitude" value={params.altKm} min={0} max={80} step={0.5} unit="km" digits={1} onChange={(altKm) => patch({ altKm })} />
                <LiveField setDragging={setDragging}
                  label="Flight path γ"
                  value={params.gammaDeg}
                  min={-8}
                  max={4}
                  step={0.1}
                  unit="deg"
                  digits={1}
                  onChange={(gammaDeg) => patch({ gammaDeg })}
                />
                <LiveField setDragging={setDragging} label="CG x/L" value={params.cgFrac} min={0.3} max={0.8} step={0.005} onChange={(cgFrac) => patch({ cgFrac })} digits={3} />
                <LiveField setDragging={setDragging} label="Wall T" value={params.twK} min={250} max={1800} step={10} unit="K" digits={0} onChange={(twK) => patch({ twK })} />
                <LiveField setDragging={setDragging} label="ρ structure" value={params.rhoKgM3} min={40} max={400} step={5} unit="kg/m³" digits={0} onChange={(rhoKgM3) => patch({ rhoKgM3 })} />
                <LiveField setDragging={setDragging} label="Mass override" value={params.massKg} min={0} max={8000} step={10} unit="kg" digits={0} onChange={(massKg) => patch({ massKg })} />
                <div>
                  <span className="hud-label mb-2 block !text-muted">Panel method</span>
                  <Seg
                    value={params.aeroMethod}
                    onChange={(aeroMethod: AeroMethod) => patch({ aeroMethod })}
                    options={[
                      { id: "newtonian", label: "Newton" },
                      { id: "tangent", label: "Tangent" },
                      { id: "mixed", label: "Mixed" },
                      { id: "cbaero", label: "CBAERO" },
                      { id: "euler", label: "Euler" },
                    ]}
                  />
                </div>
                <p className="text-xs leading-relaxed text-subtle">
                  Euler is the step between CBAERO and Cart3D: a Rust HLLC solve on the symmetry plane
                  captures the bow shock and the expansions tangent-wedge misses, then blends back to
                  CBAERO off the centerline. It is not a 3-D volume mesh. CBAERO stays the fast
                  engineering method (Dahlem–Buck, Newton–Busemann, tangent-wedge/cone).
                </p>
              </div>
            ) : null}

            {tab === "aero" ? <AeroBlock study={study} /> : null}
            {tab === "heat" ? <HeatBlock study={study} /> : null}
            {tab === "frontier" ? <FrontierBlock study={study} /> : null}
            {tab === "stab" ? <StabBlock study={study} /> : null}
            {tab === "sixdof" ? <SixDofBlock study={study} /> : null}
            {tab === "datcom" ? <DatcomBlock params={params} /> : null}

            {tab === "cycle" ? (
              <div className="space-y-4">
                <div>
                  <span className="hud-label mb-2 block !text-muted">Fuel</span>
                  <Seg
                    value={params.fuel}
                    onChange={(fuel: FuelKind) => patch({ fuel })}
                    options={[
                      { id: "H2", label: "H₂" },
                      { id: "JP", label: "JP" },
                    ]}
                  />
                </div>
                <LiveField setDragging={setDragging} label="Equivalence φ" value={params.phi} min={0.3} max={1.3} step={0.05} onChange={(phi) => patch({ phi })} />
                <LiveField setDragging={setDragging}
                  label="Flight Mach"
                  value={params.lockFlight ? params.mach : params.flightMach}
                  min={2}
                  max={20}
                  step={0.1}
                  digits={1}
                  onChange={(flightMach) => patch({ flightMach, lockFlight: false })}
                />
                <LiveField setDragging={setDragging} label="Altitude" value={params.altKm} min={0} max={80} step={0.5} unit="km" digits={1} onChange={(altKm) => patch({ altKm })} />
                <CycleBlock study={study} />
              </div>
            ) : null}

            {tab === "shocks" ? <ShocksBlock study={study} /> : null}
            {tab === "checks" ? <ChecksBlock study={study} /> : null}
            {tab === "data" ? <DataPanel params={params} paused={dragging} /> : null}

            {tab === "traj" ? (
              <div className="space-y-4">
                <MassBlock study={study} />
                <TrajBlock study={study} />
              </div>
            ) : null}

            {tab === "cad" ? (
              <div>
                <Stat k="Tip" v="(0, 0, 0)" />
                <Stat k="BBox min X" v={fmt(q.bbox.min[0], 4)} />
                <Stat k="Deflection θ" v={`${fmt(a.thetaDeg, 2)}°`} />
                <Stat k="Shock β" v={`${fmt(a.betaDeg, 2)}°`} />
                <Stat k="Volume" v={`${fmt(q.volume, 4)} m³`} />
                <Stat k="Planform" v={`${fmt(a.planformArea, 3)} m²`} />
                <Stat k="Triangles" v={String(q.triangles)} />
                <Stat
                  k="Watertight"
                  v={q.watertight ? "yes" : `${q.openEdges} open`}
                  ok={q.watertight || ((params.family === "ramjet" || params.family === "scramjet") && params.flowThrough)}
                />
                <Stat k="Manifold" v={q.manifold ? "yes" : "check"} ok={q.manifold} />
                <Stat k="Non-manifold edges" v={String(q.nonManifoldEdges)} ok={q.nonManifoldEdges === 0} />
                <Stat k="Degenerate" v={String(q.skippedDegenerate)} ok={q.skippedDegenerate === 0} />
                <div className="mt-4">
                  <span className="hud-label mb-2 block !text-muted">Export unit</span>
                  <Seg
                    value={params.unit}
                    onChange={(unit: LengthUnit) => patch({ unit })}
                    options={[
                      { id: "m", label: "m" },
                      { id: "mm", label: "mm" },
                      { id: "in", label: "in" },
                    ]}
                  />
                </div>
                {a.notes.length ? <p className="mt-3 text-xs leading-relaxed text-warn">{a.notes.join(" ")}</p> : null}
                <div className="mt-4 grid grid-cols-2 gap-2">
                  <Button variant="secondary" size="sm" onClick={() => save("stl")}>
                    <Box /> STL
                  </Button>
                  <Button variant="secondary" size="sm" onClick={() => save("plot3d")}>
                    <Grid3x3 /> Plot3D .x
                  </Button>
                  <Button variant="secondary" size="sm" onClick={() => save("step")}>
                    <Layers /> NURBS STEP
                  </Button>
                  <Button variant="secondary" size="sm" onClick={() => save("step-tess")}>
                    Mesh STEP
                  </Button>
                  <Button variant="secondary" size="sm" onClick={() => save("iges")}>
                    IGES
                  </Button>
                  <Button variant="secondary" size="sm" onClick={() => save("obj")}>
                    OBJ
                  </Button>
                  <Button variant="secondary" size="sm" onClick={() => save("vtk")}>
                    VTK
                  </Button>
                  <Button variant="secondary" size="sm" onClick={() => save("glyph")}>
                    Pointwise .glf
                  </Button>
                  <Button variant="secondary" size="sm" onClick={() => save("python")}>
                    <FileCode2 /> Python+C++
                  </Button>
                  <Button variant="secondary" size="sm" onClick={() => save("analysis")}>
                    Analysis JSON
                  </Button>
                </div>
                <Button className="mt-2 w-full" onClick={() => save("kit")} disabled={!!busy}>
                  {busy === "kit" ? "Packing…" : "Download full CFD kit (.zip)"}
                </Button>
                <p className="mt-3 text-[11px] leading-relaxed text-subtle">
                  <span className="font-medium text-fg">Pointwise:</span> export uses a dense CAD grid (at least 96×72)
                  with healed trailing edges. STL, IGES, and NURBS STEP are written in the export unit selected above
                  (IGES global unit flag included). Import the <span className="text-fg">binary STL</span> (File → Import → STL)
                  or <span className="text-fg">IGES</span> type-128 / <span className="text-fg">NURBS STEP</span> /
                  Plot3D <span className="text-fg">.x</span>. Nose is a circular fillet. Do not import STEP as XYZ
                  points. Frame: most-forward point at <span className="text-fg">(0, 0, 0)</span>. Half-model: Y = 0 is
                  the symmetry face. Use the CAD resolution preset before you mesh if you want even denser poles.
                  The kit also includes bowshock_kernel.c.
                  {params.family === "ramjet" || params.family === "scramjet"
                    ? " Ramjet: rectangular inlet at x=0 and nozzle at x=L. Flow-through leaves both OPEN."
                    : ""}
                </p>
              </div>
            ) : null}
              </PanelBoundary>
            )}
          </div>
        </aside>
      </div>

      <footer className="hud-statusbar hidden shrink-0 items-center justify-between px-6 py-2 text-[11px] text-muted sm:flex">
        <span className="inline-flex items-center gap-1.5">
          {q.watertight || ((params.family === "ramjet" || params.family === "scramjet") && params.flowThrough) ? (
            <Check className="size-3 text-accent" />
          ) : (
            <Octagon className="size-3 text-warn" />
          )}
          {q.watertight
            ? "Closed manifold solid"
            : (params.family === "ramjet" || params.family === "scramjet") && params.flowThrough
              ? "Flow-through duct — inlet & nozzle open"
              : "Open mesh — raise resolution"}
          <span className="mx-2 text-subtle" aria-hidden>·</span>
          CL {fmt(study.aero.cl, 3)} · L/D {fmt(study.aero.ld, 2)} · q̇s {fmt(study.aero.qStag, 2)} W/cm²
          <span className="mx-2 text-subtle" aria-hidden>·</span>
          {fmt(study.mass.mass, 0)} kg
        </span>
        <span className="font-mono tabular-nums">
          {q.vertices} vtx · {q.triangles} tri{hydrated ? ` · ${fmt(study.elapsedMs, 0)} ms` : ""}
        </span>
      </footer>
    </div>
  );
}
