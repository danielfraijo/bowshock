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
import { WaveriderViewer, type ViewerOpts } from "@/components/viewer";
import { NumberField, Seg, Stat } from "@/components/fields";
import { AeroBlock, ChecksBlock, CycleBlock, FrontierBlock, HeatBlock, MassBlock, ShocksBlock, SixDofBlock, StabBlock, TrajBlock } from "@/components/analysis-panel";
import { buildVehicle } from "@/lib/waverider/generate";
import { cfdZip, fileBlobs } from "@/lib/waverider/export-kit";
import { fmt } from "@/lib/waverider/math";
import { studyJson, studyVehicle } from "@/lib/waverider/study";
import { loadSavedParams, PRESETS, saveParams } from "@/lib/waverider/presets";
import {
  COLOR_MODES,
  DEFAULT_PARAMS,
  FAMILY_GROUPS,
  FAMILY_META,
  domainOf,
  type AeroMethod,
  type CowlSide,
  type DesignParams,
  type FlowDomain,
  type FuelKind,
  type LengthUnit,
  type LidFace,
  type PlanformKind,
  type WaveriderFamily,
} from "@/lib/waverider/types";
import { downloadBlob } from "@/lib/utils";

type Tab = "geom" | "flight" | "aero" | "heat" | "frontier" | "stab" | "sixdof" | "traj" | "cycle" | "shocks" | "checks" | "cad";

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
      { id: "checks", label: "Checks" },
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
    { id: "traj", label: "Traj" },
    { id: "checks", label: "Checks" },
    { id: "cad", label: "CAD" },
  ];
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
    p.leRadius,
    p.leBlunt,
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
  const built = useMemo(() => buildVehicle(params), [gKey]); // eslint-disable-line react-hooks/exhaustive-deps
  const study = useMemo(
    () => studyVehicle(built, { quick: dragging || !hydrated }),
    [
      built,
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
      dragging,
      hydrated,
    ],
  );
  const q = built.quality;
  const a = built.aero;
  const fam = FAMILY_META[params.family];
  const domain = domainOf(params.family);
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
          ? buildVehicle({ ...params, nx: Math.max(params.nx, 96), ny: Math.max(params.ny, 72) })
          : built;
      const aero = cadKinds.has(kind) && src !== built ? studyVehicle(src, { quick: true }).aero : study.aero;
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
        if (cppSrc) downloadBlob(new Blob([cppSrc], { type: "text/x-c++src" }), "bowshock.cpp");
      } else if (kind === "kit") {
        const zip = await cfdZip(src, python, cSrc, studyJson(src, studyVehicle(src, { quick: true })), cppSrc, {
          cp: aero.cp,
          heat: aero.heat,
          twEq: aero.twEq,
          stanton: aero.stanton,
          machE: aero.machE,
          cf: aero.cf,
          impact: aero.impact,
        });
        downloadBlob(zip, `${n}_cfd_kit.zip`);
      }
    } finally {
      setBusy(null);
    }
  }

  return (
    <div className="flex min-h-dvh flex-col bg-bg text-fg">
      <header className="flex items-center justify-between gap-3 border-b border-border px-4 py-3 sm:px-6">
        <div className="min-w-0">
          <p className="font-display text-xl font-semibold tracking-[0.22em] text-fg">CUSPIS</p>
          <p className="truncate text-xs text-muted">
            {domain === "external" ? "Roma · hypersonic CAD · external" : "Roma · hypersonic CAD · internal"}
          </p>
        </div>
        <div className="flex items-center gap-2">
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

      <div className="mx-auto grid w-full max-w-[1680px] flex-1 grid-cols-1 lg:grid-cols-[260px_minmax(0,1fr)_320px]">
        <aside className="order-2 border-b border-border lg:order-none lg:border-r lg:border-b-0">
          <div className="p-4 sm:p-5">
            <p className="mb-2 text-[11px] font-medium tracking-[0.16em] text-subtle uppercase">Toolkit</p>
            <Seg
              value={domain}
              onChange={(d: FlowDomain) => goDomain(d)}
              options={[
                { id: "external", label: "External" },
                { id: "internal", label: "Internal" },
              ]}
            />
            <p className="mt-3 mb-2 text-[11px] font-medium tracking-[0.16em] text-subtle uppercase">
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
                    className={`min-h-11 min-w-[9.5rem] shrink-0 rounded-md px-3 py-2 text-left shadow-[0_0_0_1px_rgba(255,255,255,0.08)] transition-colors lg:min-w-0 ${
                      on ? "bg-accent text-accent-fg" : "bg-surface-2 text-fg hover:bg-surface"
                    }`}
                  >
                    <span className="block text-sm font-medium">{meta.label}</span>
                    <span className={`mt-0.5 block text-[11px] leading-snug ${on ? "text-accent-fg/70" : "text-muted"}`}>
                      {meta.generating}
                    </span>
                  </button>
                );
              })}
            </div>

            <button
              type="button"
              className="mt-4 flex w-full items-center justify-between text-[11px] font-medium tracking-[0.16em] text-subtle uppercase"
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
                    className="rounded-md bg-surface-2 px-3 py-2 text-left shadow-[0_0_0_1px_rgba(255,255,255,0.07)] hover:bg-surface"
                  >
                    <span className="block text-sm text-fg">{pr.name}</span>
                    <span className="block text-[11px] text-muted">{pr.blurb}</span>
                  </button>
                ))}
              </div>
            ) : null}
          </div>
        </aside>

        <section className="relative order-1 flex min-h-[280px] flex-col lg:order-none lg:min-h-0">
          <div className="relative h-[46vh] min-h-[280px] flex-1 lg:h-auto lg:min-h-[420px]">
            <WaveriderViewer built={built} study={study} opts={opts} />
            <div className="pointer-events-none absolute inset-x-0 top-0 h-16 bg-gradient-to-b from-bg/70 to-transparent" />
            <div className="absolute top-3 left-3 rounded-md bg-bg/70 px-2.5 py-1.5 text-[11px] text-muted backdrop-blur-sm">
              <span className="font-medium text-fg">{fam.label}</span>
              <span className="mx-1.5 text-subtle">/</span>
              {domain}
              <span className="mx-1.5 text-subtle">/</span>
              M {params.lockFlight ? params.mach : params.flightMach}
              <span className="mx-1.5 text-subtle">/</span>
              tip @ 0
            </div>
            <div className="absolute top-3 right-3 flex flex-wrap justify-end gap-1">
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
                  className={`h-9 rounded-sm px-2.5 text-[11px] font-medium ${
                    on ? "bg-accent text-accent-fg" : "bg-bg/70 text-muted backdrop-blur-sm"
                  }`}
                >
                  {label}
                </button>
              ))}
            </div>
            <div className="absolute bottom-3 left-3 flex flex-col gap-1.5">
              <div className="flex max-w-[26rem] flex-wrap gap-1">
                {COLOR_MODES.map((c) => (
                  <button
                    key={c.id}
                    type="button"
                    onClick={() => setOpts((o) => ({ ...o, color: c.id }))}
                    className={`h-8 rounded-sm px-2 text-[10px] font-medium uppercase ${
                      opts.color === c.id ? "bg-accent text-accent-fg" : "bg-bg/70 text-muted backdrop-blur-sm"
                    }`}
                  >
                    {c.label}
                  </button>
                ))}
              </div>
              {opts.color !== "surface" ? (
                <div className="flex h-2 w-40 overflow-hidden rounded-sm bg-bg" title={opts.color === "cp" ? "Cp" : "heat"}>
                  <span className="jet-ramp-h h-full w-full" />
                </div>
              ) : null}
            </div>
            <div className="absolute right-3 bottom-3 font-mono text-[10px] tracking-wider text-subtle">
              {hydrated ? `${fmt(study.elapsedMs, 0)} ms · ` : null}L/D {fmt(study.aero.ld, 2)}
              <span className="mx-1.5 text-subtle">·</span>
              CoP {fmt(study.aero.xCp / Math.max(params.length, 1e-8), 2)}L
            </div>
          </div>
        </section>

        <aside className="order-3 flex flex-col border-t border-border lg:order-none lg:border-t-0 lg:border-l">
          <div className="flex gap-px overflow-x-auto border-b border-border bg-surface-2 p-1">
            {TABS.map((t) => (
              <button
                key={t.id}
                type="button"
                onClick={() => setTab(t.id)}
                className={`h-9 shrink-0 rounded-sm px-2.5 text-xs font-medium ${
                  tab === t.id ? "bg-accent text-accent-fg" : "text-muted hover:text-fg"
                }`}
              >
                {t.label}
              </button>
            ))}
          </div>
          <div className="flex-1 overflow-y-auto p-4 sm:p-5">
            {!hydrated ? (
              <p className="text-sm leading-relaxed text-muted">{fam.blurb}</p>
            ) : (
              <>
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
                  <span className="mb-2 block text-xs font-medium text-muted">Flat lid</span>
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
                      <span className="mb-2 block text-xs font-medium text-muted">Planform</span>
                      <div className="grid grid-cols-3 gap-1.5">
                        {(["delta", "power", "spatular", "rect", "double"] as PlanformKind[]).map((pl) => (
                          <button
                            key={pl}
                            type="button"
                            onClick={() => patch({ planform: pl })}
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
                        <span className="mb-2 block text-xs font-medium text-muted">Cowl</span>
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
                        <span className="text-xs font-medium text-muted">Flow-through (open duct)</span>
                        <Switch checked={params.flowThrough} onCheckedChange={(flowThrough) => patch({ flowThrough })} />
                      </div>
                    ) : null}
                  </>
                ) : null}
                <LiveField setDragging={setDragging} label="Streamwise pts" value={params.nx} min={12} max={160} step={1} digits={0} onChange={(nx) => patch({ nx: Math.round(nx) })} />
                <LiveField setDragging={setDragging} label="Spanwise pts" value={params.ny} min={10} max={120} step={1} digits={0} onChange={(ny) => patch({ ny: Math.round(ny) })} />
                <div>
                  <span className="mb-2 block text-xs font-medium text-muted">Resolution</span>
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
                <div className="flex items-center justify-between gap-3 pt-1">
                  <span className="text-xs font-medium text-muted">Rounded leading edge</span>
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
                ) : (
                  <p className="text-[11px] text-subtle">Sharp knife-edge — inviscid theory only. Pointwise needs a radius.</p>
                )}
                <div className="flex items-center justify-between gap-3 pt-1">
                  <span className="text-xs font-medium text-muted">Half-model (Y≥0)</span>
                  <Switch checked={params.halfModel} onCheckedChange={(halfModel) => patch({ halfModel })} />
                </div>
              </div>
            ) : null}

            {tab === "flight" ? (
              <div className="space-y-4">
                <div className="flex items-center justify-between gap-3">
                  <span className="text-xs font-medium text-muted">Lock flight Mach = design</span>
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
                  <span className="mb-2 block text-xs font-medium text-muted">Panel method</span>
                  <Seg
                    value={params.aeroMethod}
                    onChange={(aeroMethod: AeroMethod) => patch({ aeroMethod })}
                    options={[
                      { id: "newtonian", label: "Newton" },
                      { id: "tangent", label: "Tangent" },
                      { id: "mixed", label: "Mixed" },
                      { id: "cbaero", label: "CBAERO" },
                    ]}
                  />
                </div>
                <p className="text-xs leading-relaxed text-subtle">
                  CBAERO is the closest engineering method to Euler/NS CFD: Dahlem–Buck + Newton–Busemann +
                  tangent-wedge/cone, Prandtl–Meyer leeward, Eckert–Zoby heating on the post-shock edge.
                  +α heats the belly; −α heats the lid. Type a number or drag the slider.
                </p>
              </div>
            ) : null}

            {tab === "aero" ? <AeroBlock study={study} /> : null}
            {tab === "heat" ? <HeatBlock study={study} /> : null}
            {tab === "frontier" ? <FrontierBlock study={study} /> : null}
            {tab === "stab" ? <StabBlock study={study} /> : null}
            {tab === "sixdof" ? <SixDofBlock study={study} /> : null}

            {tab === "cycle" ? (
              <div className="space-y-4">
                <div>
                  <span className="mb-2 block text-xs font-medium text-muted">Fuel</span>
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
                <div className="mt-4">
                  <span className="mb-2 block text-xs font-medium text-muted">Export unit</span>
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
                  with healed trailing edges. Import the <span className="text-fg">binary STL</span> (File → Import → STL)
                  or <span className="text-fg">IGES</span> type-128 / <span className="text-fg">NURBS STEP</span> /
                  Plot3D <span className="text-fg">.x</span>. Nose is a circular fillet. Do not import STEP as XYZ
                  points. Frame: most-forward point at <span className="text-fg">(0, 0, 0)</span>. Half-model: Y = 0 is
                  the symmetry face. Use the CAD resolution preset before you mesh if you want even denser poles.
                  {params.family === "ramjet" || params.family === "scramjet"
                    ? " Ramjet: rectangular inlet at x=0 and nozzle at x=L. Flow-through leaves both OPEN."
                    : ""}
                </p>
              </div>
            ) : null}
              </>
            )}
          </div>
        </aside>
      </div>

      <footer className="hidden items-center justify-between border-t border-border px-6 py-2 text-[11px] text-subtle sm:flex">
        <span className="inline-flex items-center gap-1.5">
          {q.watertight || ((params.family === "ramjet" || params.family === "scramjet") && params.flowThrough) ? (
            <Check className="size-3 text-fg" />
          ) : (
            <Octagon className="size-3 text-muted" />
          )}
          {q.watertight
            ? "Closed manifold solid"
            : (params.family === "ramjet" || params.family === "scramjet") && params.flowThrough
              ? "Flow-through duct — inlet & nozzle open"
              : "Open mesh — raise resolution"}
          <span className="mx-2 text-border">·</span>
          CL {fmt(study.aero.cl, 3)} · L/D {fmt(study.aero.ld, 2)} · q̇s {fmt(study.aero.qStag, 2)} W/cm²
          <span className="mx-2 text-border">·</span>
          {fmt(study.mass.mass, 0)} kg
        </span>
        <span className="font-mono tabular-nums">
          {q.vertices} vtx · {q.triangles} tri{hydrated ? ` · ${fmt(study.elapsedMs, 0)} ms` : ""}
        </span>
      </footer>
    </div>
  );
}
