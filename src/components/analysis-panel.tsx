import { CartesianGrid, Line, LineChart, ResponsiveContainer, Tooltip, XAxis, YAxis } from "recharts";
import { fmt } from "@/lib/waverider/math";
import type { StudyResult } from "@/lib/waverider/study";
import { Formula, Stat } from "@/components/fields";

export function AeroBlock({ study }: { study: StudyResult }) {
  const a = study.aero;
  const atm = study.atm;
  const b = study.bench;
  return (
    <div>
      <Stat k="Flight M" v={fmt(study.flightMach, 2)} />
      <Stat k="q∞" v={`${fmt(atm.q / 1000, 2)} kPa`} />
      <Stat k="T∞" v={`${fmt(atm.T, 1)} K`} />
      <Stat k="Re/L" v={`${fmt(atm.ReL, 3)} /m`} />
      <Stat k="CL" v={fmt(a.cl, 4)} />
      <Stat k="CD" v={fmt(a.cd, 4)} />
      <Stat k="  wave" v={fmt(a.cdWave, 4)} />
      <Stat k="  friction" v={fmt(a.cdFric, 4)} />
      <Stat k="  base" v={fmt(a.cdBase, 4)} />
      <Stat k="L/D" v={fmt(a.ld, 2)} />
      <Stat k="Cm (pitch)" v={fmt(a.cm, 4)} />
      <Stat k="Cn (yaw)" v={fmt(a.cn, 4)} />
      <Stat k="cot θ (wedge)" v={fmt(b.wedgeCot, 2)} />
      <Stat k="L/D / cot θ" v={fmt(b.ratio, 2)} ok={b.ratio > 0.35 && b.ratio < 1.15} />
      <Stat k="Method" v={a.method} />
      <p className="mt-3 text-xs leading-relaxed text-subtle">{b.note}</p>
      <p className="mt-2 text-xs leading-relaxed text-muted">
        Lit. L/D {study.literature.ld[0]}–{study.literature.ld[1]}
        {study.literature.inBand ? " — in band." : " — outside typical band (off-design or inviscid/viscous mix)."}
      </p>
      <p className="mt-1 text-xs leading-relaxed text-subtle">{study.literature.note}</p>
      <div className="mt-4 space-y-2">
        <Formula
          name="θ-β-M"
          expr="tan θ = 2 cot β (M² sin²β − 1) / (M² (γ + cos 2β) + 2)"
          note="Weak root used. Detached if θ > θ_max(M)."
        />
        <Formula name="Inviscid wedge" expr="L/D = cot θ" note="Exact for a 2-D wedge with p_lid = p∞ and no friction." />
        <Formula
          name="Modified Newtonian"
          expr="Cp = Cp_max sin²θ    Cp_max = (p₀₂/p∞ − 1) / (½ γ M²)"
          note="Lees. Used on the windward blend."
        />
        <Formula name="Prandtl–Meyer (leeward)" expr="ν(M) = √((γ+1)/(γ−1)) tan⁻¹√[…] − tan⁻¹√(M²−1)" />
        <Formula
          name="Mixed panel"
          expr="attached: tangent-wedge / cone Cp    detached: Cp_max sin²θ    leeward: PM"
          note="CBAERO-class. Base Love Cp = −1/M² (NACA TN 3819 high-M)."
        />
        <Formula name="Skin friction" expr="Cf = Cfi / Fc     van Driest II, Cfi = 0.455/(log₁₀ Re)²" note="Hopkins–Inouye. Compressibility via Taw, Tw." />
      </div>
    </div>
  );
}

export function HeatBlock({ study }: { study: StudyResult }) {
  const a = study.aero;
  return (
    <div>
      <Stat k="Stag. heat" v={`${fmt(a.qStag, 3)} W/cm²`} />
      <Stat k="Radiative" v={`${fmt(a.qRad, 3)} W/cm²`} />
      <Stat k="Peak panel" v={`${fmt(a.qMax, 3)} W/cm²`} />
      <Stat k="Mean windward" v={`${fmt(a.qMeanWind, 3)} W/cm²`} />
      <Stat k="Rn used" v="LE radius or 0.15% L" />
      <p className="mt-3 text-xs leading-relaxed text-subtle">
        Sutton–Graves stagnation (TR R-802), Tauber laminar/turbulent along running length (TP-2914),
        Tauber–Sutton radiative (JSR 1991, Earth). Recovery uses r=√Pr laminar / Pr⅓ turbulent. Not a CFD heat flux — color the view by Heat.
      </p>
      <div className="mt-3 space-y-2">
        <Formula name="Sutton–Graves" expr="q_s = 1.83×10⁻⁸ √(ρ/Rn) V³ (1 − hw/h0)    W/cm²" note="Sutton & Graves NASA TR R-802; Tauber TP-2914 units." />
        <Formula name="Tauber–Sutton radiative" expr="q_r = 4.736×10⁸ Rn^1.072 ρ^1.22 (V/10⁴)^8.5    (V ≳ 2.5 km/s)" />
      </div>
    </div>
  );
}

export function StabBlock({ study }: { study: StudyResult }) {
  const s = study.stab;
  return (
    <div>
      <Stat k="CLα ( /rad )" v={fmt(s.cla, 3)} />
      <Stat k="Cmα ( /rad )" v={fmt(s.cma, 3)} ok={s.cma < 0} />
      <Stat k="Cnβ ( /rad )" v={fmt(s.cnb, 3)} ok={s.cnb > 0} />
      <Stat k="Clβ ( /rad )" v={fmt(s.clb, 3)} />
      <Stat k="Static margin" v={`${fmt(s.staticMarginPct, 1)} % L`} ok={s.staticMargin > 0} />
      <Stat k="Trim α" v={`${fmt(s.trimAlpha, 2)}°`} />
      <Stat k="Long. stable" v={s.longitudinallyStable ? "yes" : "no"} ok={s.longitudinallyStable} />
      <Stat k="Dir. stable" v={s.directionallyStable ? "yes" : "no"} ok={s.directionallyStable} />
      <div className="mt-4 h-36">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={s.polar} margin={{ top: 4, right: 8, left: 0, bottom: 0 }}>
            <CartesianGrid stroke="rgba(255,255,255,0.08)" />
            <XAxis dataKey="a" tick={{ fill: "#9a9a9a", fontSize: 10 }} />
            <YAxis tick={{ fill: "#9a9a9a", fontSize: 10 }} />
            <Tooltip
              contentStyle={{ background: "#111111", border: "1px solid #2c2c2c", fontSize: 11, color: "#f4f4f4" }}
              labelFormatter={(v) => `α ${v}°`}
            />
            <Line type="monotone" dataKey="cl" stroke="#f2f2f2" dot={false} strokeWidth={1.5} name="CL" />
            <Line type="monotone" dataKey="cm" stroke="#8a8a8a" dot={false} strokeWidth={1.5} name="Cm" />
          </LineChart>
        </ResponsiveContainer>
      </div>
      <p className="mt-2 text-xs text-subtle">Finite-difference ±1.5°. Elevon on Geom tab for trim. CG at Flight x/L.</p>
    </div>
  );
}

export function PropBlock({ study }: { study: StudyResult }) {
  return <CycleBlock study={study} />;
}

export function CycleBlock({ study }: { study: StudyResult }) {
  const p = study.prop;
  if (!p) {
    return (
      <p className="text-xs leading-relaxed text-muted">
        Internal toolkit: pick Ramjet, Scramjet, Busemann, REST, or WR + ramjet. The cycle is 1-D
        gas dynamics (θ-β-M ramps, Rankine–Hugoniot, Rayleigh, isentropic nozzle) — not a CFD dump.
      </p>
    );
  }
  const ic = p.internal;
  return (
    <div>
      <Stat k="Mode" v={p.kind} />
      <Stat k="Thrust" v={`${fmt(p.thrust / 1000, 2)} kN`} />
      <Stat k="Isp" v={`${fmt(p.isp, 0)} s`} />
      <Stat k="ṁ air" v={`${fmt(p.mdot, 2)} kg/s`} />
      <Stat k="Tt4" v={`${fmt(p.Tt4, 0)} K`} />
      <Stat k="Ue / U0" v={fmt(p.Ue / Math.max(p.Ue0, 1), 3)} />
      <Stat k="πd (inlet)" v={fmt(ic?.piD ?? 0, 3)} />
      <Stat k="Kantrowitz At/Ac" v={fmt(ic?.kantrowitz ?? 0, 3)} />
      <Stat k="Self-start" v={ic?.started ? "yes" : "maybe not"} ok={ic?.started} />
      <Stat k="η thermal" v={fmt(p.etaThermal, 3)} />
      <p className="mt-3 text-xs leading-relaxed text-subtle">{p.notes.join(" ")}</p>
      {ic ? (
        <div className="mt-4 space-y-2">
          {ic.equations.slice(0, 4).map((e) => (
            <Formula key={e.name} name={e.name} expr={e.expr} />
          ))}
        </div>
      ) : null}
    </div>
  );
}

export function ShocksBlock({ study }: { study: StudyResult }) {
  const ic = study.prop?.internal;
  if (!ic) {
    return <p className="text-xs leading-relaxed text-muted">No internal shock train on this family.</p>;
  }
  return (
    <div>
      <p className="mb-3 text-xs leading-relaxed text-muted">
        Each ramp uses the exact θ-β-M weak root, then Rankine–Hugoniot on Mn = M sin β. Stations are
        1-D mass / energy consistent.
      </p>
      {ic.shocks.map((s) => (
        <div key={s.name} className="mb-3 border-b border-border/80 pb-2">
          <p className="text-xs font-medium text-fg">{s.name}</p>
          <Stat k="θ" v={`${fmt(s.thetaDeg, 2)}°`} />
          <Stat k="β" v={`${fmt(s.betaDeg, 2)}°`} />
          <Stat k="M1 → M2" v={`${fmt(s.M1, 3)} → ${fmt(s.M2, 3)}`} />
          <Stat k="p2/p1" v={fmt(s.p2p1, 3)} />
          <Stat k="T2/T1" v={fmt(s.T2T1, 3)} />
        </div>
      ))}
      <p className="mb-2 text-[10px] font-medium tracking-[0.14em] text-subtle uppercase">Stations</p>
      {ic.stations.map((st) => (
        <Stat key={st.name} k={`${st.name}  M`} v={fmt(st.M, 3)} />
      ))}
    </div>
  );
}

export function ChecksBlock({ study }: { study: StudyResult }) {
  const checks = study.checks ?? [];
  const n = checks.length;
  const pass = checks.filter((c) => c.pass).length;
  const domain = study.domain;
  const shown = checks.filter((c) => c.domain === "gas" || c.domain === domain || c.domain === "external");
  return (
    <div>
      <Stat k="Kernel" v={`${pass}/${n} pass`} ok={pass === n} />
      <p className="mt-2 mb-3 text-xs leading-relaxed text-muted">
        Closed-form identities and published tables (Anderson, NACA 1135, Sims, US76, Kantrowitz). These
        do not use the mesh — they test the same solvers the vehicle uses.
      </p>
      {shown.map((c) => (
        <div key={c.id} className="border-b border-border/80 py-2 last:border-0">
          <div className="flex items-baseline justify-between gap-2">
            <span className="text-xs text-fg">{c.name}</span>
            <span className={`font-mono text-[11px] ${c.pass ? "text-ok" : "text-warn"}`}>{c.pass ? "PASS" : "FAIL"}</span>
          </div>
          <p className="mt-1 font-mono text-[10px] leading-snug text-subtle">{c.formula}</p>
          <p className="mt-0.5 font-mono text-[10px] text-muted">
            got {fmt(c.got, 4)} · expect {fmt(c.expected, 4)} · {c.source}
          </p>
        </div>
      ))}
    </div>
  );
}

export function MassBlock({ study }: { study: StudyResult }) {
  const m = study.mass;
  return (
    <div>
      <Stat k="Mass" v={`${fmt(m.mass, 1)} kg`} />
      <Stat k="ρ used" v={`${fmt(m.rho, 0)} kg/m³`} />
      <Stat k="CG x" v={`${fmt(m.cg[0], 3)} m`} />
      <Stat k="Ixx" v={`${fmt(m.Ixx, 1)} kg·m²`} />
      <Stat k="Iyy (pitch)" v={`${fmt(m.Iyy, 1)} kg·m²`} />
      <Stat k="Izz" v={`${fmt(m.Izz, 1)} kg·m²`} />
      <Stat k="Ballistic β" v={`${fmt(m.ballistic, 0)} kg/m²`} />
      <p className="mt-3 text-xs leading-relaxed text-subtle">
        Volume from the closed mesh × structural density (or a mass override). β = m / (CD S). For a
        hollow tank-like airframe 80–200 kg/m³ of enclosed volume is a screening guess, not a FEM.
      </p>
    </div>
  );
}

export function TrajBlock({ study }: { study: StudyResult }) {
  const t = study.traj;
  const data = t.samples.filter((_, i) => i % 2 === 0);
  return (
    <div>
      <Stat k="Range" v={`${fmt(t.rangeKm, 1)} km`} />
      <Stat k="Time" v={`${fmt(t.timeS, 0)} s`} />
      <Stat k="Max q" v={`${fmt(t.maxQ / 1000, 2)} kPa`} />
      <Stat k="Peak heat" v={`${fmt(t.maxHeat, 2)} W/cm²`} />
      <Stat k="Heat load" v={`${fmt(t.heatLoad, 1)} J/cm²`} />
      <Stat k="Final V" v={`${fmt(t.finalV, 0)} m/s`} />
      <Stat k="Final h" v={`${fmt(t.finalH, 1)} km`} />
      <div className="mt-4 h-36">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data} margin={{ top: 4, right: 8, left: 0, bottom: 0 }}>
            <CartesianGrid stroke="rgba(255,255,255,0.08)" />
            <XAxis dataKey="xKm" tick={{ fill: "#9a9a9a", fontSize: 10 }} />
            <YAxis tick={{ fill: "#9a9a9a", fontSize: 10 }} />
            <Tooltip
              contentStyle={{ background: "#111111", border: "1px solid #2c2c2c", fontSize: 11, color: "#f4f4f4" }}
              labelFormatter={(v) => `${Number(v).toFixed(0)} km`}
            />
            <Line type="monotone" dataKey="hKm" stroke="#f2f2f2" dot={false} strokeWidth={1.5} name="h km" />
          </LineChart>
        </ResponsiveContainer>
      </div>
      <p className="mt-2 text-xs leading-relaxed text-subtle">{t.notes[0]}</p>
    </div>
  );
}
