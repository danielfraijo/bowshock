import { CartesianGrid, Line, LineChart, ResponsiveContainer, Tooltip, XAxis, YAxis } from "recharts";
import type { ReactNode } from "react";
import { detraKempRiddell, fmt } from "@/lib/waverider/math";
import type { StudyResult } from "@/lib/waverider/study";
import { Formula, Stat } from "@/components/fields";
import { Readout } from "@/components/ui/hud"; // [ui-redesign]

function Lead({ children }: { children: ReactNode }) {
  return <p className="mb-3 text-xs leading-relaxed text-muted">{children}</p>;
}

function withUnit(n: number, digits: number, unit: string) {
  return Number.isFinite(n) ? `${fmt(n, digits)} ${unit}` : "—";
}

export function AeroBlock({ study }: { study: StudyResult }) {
  const a = study.aero;
  const atm = study.atm;
  const b = study.bench;
  return (
    <div>
      <Lead>Forces at this Mach, alpha, and altitude. Wave, friction, and base are split so a finer CFD run has something to move.</Lead>
      <div className="hud-panel mb-3 grid grid-cols-3 gap-3 px-3 py-3">
        <Readout label="L/D" value={fmt(a.ld, 2)} tone="accent" />
        <Readout label="CL" value={fmt(a.cl, 4)} size="sm" />
        <Readout label="CD" value={fmt(a.cd, 4)} size="sm" />
      </div>
      <Stat k="Flight M" v={fmt(study.flightMach, 2)} />
      <Stat k="q∞" v={`${fmt(atm.q / 1000, 2)} kPa`} />
      <Stat k="T∞" v={`${fmt(atm.T, 1)} K`} />
      <Stat k="Re/L" v={`${fmt(atm.ReL, 3)} /m`} />
      <Stat k="  wave" v={fmt(a.cdWave, 4)} />
      <Stat k="  friction" v={fmt(a.cdFric, 4)} />
      <Stat k="  base" v={fmt(a.cdBase, 4)} />
      <Stat k="Cm (pitch)" v={fmt(a.cm, 4)} />
      <Stat k="Cn (yaw)" v={fmt(a.cn, 4)} />
      <Stat k="CoP x/L" v={fmt(a.xCp / Math.max(a.lRef, 1e-8), 3)} />
      <Stat k="CG x/L" v={fmt(a.cg[0] / Math.max(a.lRef, 1e-8), 3)} />
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
          name="CBAERO / HABP"
          expr="windward: 0.55 tangent + 0.25 Dahlem–Buck + 0.20 Newton–Busemann    leeward: PM    base: −1/M²"
          note="Pick CBAERO on the Flight tab. Tangent-wedge recomputes every panel from freestream."
        />
        <Formula
          name="Shock-expansion strip"
          expr="nose: oblique shock or Taylor–Maccoll    later turns: Prandtl–Meyer"
          note="Mixed marches strips. Base Love Cp = −1/M²."
        />
        <Formula name="Skin friction" expr="Cf = Cfi / Fc     van Driest II, Cfi = 0.455/(log₁₀ Re)^2.58" note="Hopkins–Inouye. Compressibility via Taw, Tw." />
      </div>
    </div>
  );
}

export function HeatBlock({ study }: { study: StudyResult }) {
  const a = study.aero;
  const atm = study.atm;
  const Rn = Math.max(0.0015 * Math.max(a.lRef, 1), 0.004);
  const qDkr = detraKempRiddell(atm.rho, atm.V, Rn);
  return (
    <div>
      <Lead>Stagnation heating from three correlations. Size the TPS on Iterate from these, not from a flight measurement.</Lead>
      <Stat k="Stag. heat" v={`${fmt(a.qStag, 3)} W/cm²`} />
      <Stat k="Fay–Riddell" v={`${fmt(a.qFay, 3)} W/cm²`} />
      <Stat k="Sutton–Graves" v={`${fmt(a.qSG, 3)} W/cm²`} />
      <Stat k="DKR stag." v={`${fmt(qDkr, 3)} W/cm²`} />
      <Stat k="Radiative" v={`${fmt(a.qRad, 3)} W/cm²`} />
      <Stat k="Peak panel" v={`${fmt(a.qMax, 3)} W/cm²`} />
      <Stat k="Mean windward" v={`${fmt(a.qMeanWind, 3)} W/cm²`} />
      <Stat k="Mean belly" v={`${fmt(a.qMeanLower, 3)} W/cm²`} />
      <Stat k="Mean lid" v={`${fmt(a.qMeanUpper, 3)} W/cm²`} />
      <Stat k="Tw eq (peak)" v={`${fmt(a.twMax, 0)} K`} />
      <Stat k="Rn used" v="LE radius or 0.15% L" />
      <p className="mt-3 text-xs leading-relaxed text-subtle">
        Heating is Eckert–Zoby (NASA TP-1374 / MINIVER / CBAERO) on the post-shock edge state, blended with
        Tauber running-length and Lees p/p_s. That is the engineering method that tracks Navier–Stokes heat
        flux. +α (nose up) raises belly q̇; −α raises lid q̇. Stagnation is Fay–Riddell with Billig standoff,
        plus Sutton–Graves and Detra–Kemp–Riddell checks. Color the view by Heat, q/qs, p/p∞, Tw, Me, St, Cf or θ.
        Running length is s = x − x_LE(y) so the swept leading edge stays hot.
      </p>
      <div className="mt-3 space-y-2">
        <Formula name="Fay–Riddell" expr="q = 0.763 Pr⁻⁰·⁶ (ρeμe)⁰·⁴ (ρwμw)⁰·¹ √(due/ds) (h0−hw)" note="Sphere stagnation, Lewis=1. due/ds = Rn⁻¹ √(2(pe−p∞)/ρe)." />
        <Formula name="Zoby / Eckert" expr="q = St ρe Ue (hr − hw)    St from T* reference enthalpy" note="Edge (pe, Ue, Te) from the local shock or Prandtl–Meyer expansion. Closest panel heating to CFD." />
        <Formula name="Sutton–Graves" expr="q_s = 1.7415×10⁻⁸ √(ρ/Rn) V³ (1 − hw/h0)    W/cm²" note="Sutton & Graves NASA TR R-376 (Earth air; 1.83×10⁻⁸ is the CO₂ value)." />
        <Formula name="Detra–Kemp–Riddell" expr="q = 5.21×10⁴ √(ρ/Rn) (V/10⁴)^3.15    W/m²" note="DKR stagnation; reported as Tw_eq from the SG flux." />
        <Formula name="Radiation equilibrium" expr="σ ε Tw⁴ = q_conv    ε = 0.8" />
        <Formula name="Tauber–Sutton radiative" expr="q_r = 4.736×10⁸ Rn^1.072 ρ^1.22 (V/10⁴)^8.5    (V ≳ 2.5 km/s)" />
      </div>
    </div>
  );
}

export function FrontierBlock({ study }: { study: StudyResult }) {
  const f = study.frontier;
  return (
    <div>
      <Stat k="Regime" v={f.regime} ok={f.regime === "continuum"} />
      <Stat k="Knudsen Kn" v={f.kn.toExponential(2)} />
      <Stat k="Mean free path" v={`${fmt(f.lambda * 1e6, 2)} μm`} />
      <Stat k="γ_vib (T₂)" v={fmt(f.gammaEq, 3)} />
      <Stat k="γ_eff (Da)" v={fmt(f.gammaEff, 3)} />
      <Stat k="γ_real (IDG)" v={fmt(f.gammaReal, 3)} />
      <Stat k="T₂ post-NS" v={`${fmt(f.T2, 0)} K`} />
      <Stat k="Da_vib" v={f.daVib.toExponential(2)} />
      <Stat k="τ_v" v={`${fmt(f.tauVib * 1e6, 1)} μs`} />
      <Stat k="α_O₂ / α_N₂" v={`${fmt(f.alphaO2, 3)} / ${fmt(f.alphaN2, 3)}`} />
      <Stat k="ρL binary" v={`${f.rhoL.toExponential(2)} kg/m²`} />
      <Stat k="Billig Δ/Rn" v={fmt(f.billigDelta, 3)} />
      <Stat k="Van Dyke K" v={fmt(f.vanDykeK, 2)} />
      <Stat k="χ̄ visc. int." v={fmt(f.chiBar, 2)} />
      <Stat k="p / p_inv" v={fmt(f.pVisc, 3)} />
      <Stat k="Fay–Riddell" v={`${fmt(f.qFay, 2)} W/cm²`} />
      <Stat k="Sutton–Graves" v={`${fmt(f.qSG, 2)} W/cm²`} />
      <Stat k="DKR" v={`${fmt(f.qDkr, 2)} W/cm²`} />
      <Stat k="Tw FR" v={`${fmt(f.twFay, 0)} K`} />
      <Stat k="LE sweep Λ" v={`${fmt(f.sweepDeg, 1)}°`} />
      <Stat k="Sweep q/q₀" v={fmt(f.sweepFactor, 3)} />
      {f.edney ? <Stat k="Edney Type IV" v={`${fmt(f.qEdney, 0)} W/cm²`} /> : null}
      {f.edney ? <Stat k="Isolator L/H" v={fmt(f.isolatorLH, 1)} /> : null}
      <Stat k="Eq-glide CL" v={fmt(f.clEqGlide, 3)} />
      <Stat k="Panel CL" v={fmt(f.clAvail, 3)} />
      {f.maxN > 0 ? <Stat k="Peak n (traj)" v={`${fmt(f.maxN, 2)} g`} /> : null}
      {f.twPeak > 0 ? <Stat k="Tw lump (traj)" v={`${fmt(f.twPeak, 0)} K`} /> : null}
      <p className="mt-3 text-xs leading-relaxed text-subtle">{f.notes.join(" ")}</p>
      <div className="mt-3 space-y-2">
        <Formula name="γ_vib" expr="cv/R = 5/2 + Σ xi (θv/T)² e^u/(e^u−1)²    γ = 1 + R/cv" note="O₂ 2270 K, N₂ 3390 K. Dissociation via Lighthill IDG on γ_real." />
        <Formula name="Da_vib" expr="Da = (L/V) / τ_v    τ_v from Millikan–White / Park 1990" note="γ_eff = 1.4 + (γ_eq−1.4)(1−e^{−Da}). Frozen if Da≪1." />
        <Formula name="Billig standoff" expr="Δ/Rn = 0.143 exp(3.24/M²)" note="Sphere. Enters Fay–Riddell due/ds." />
        <Formula name="Lees heat" expr="q/q_s = (p/p_s)^{1/2} (Rn/(Rn+s))^{1/2}" note="Laminar. Turbulent exponents 0.8 / 0.2. Mixed with Tauber." />
        <Formula name="χ̄" expr="χ̄ = M³ √C / √Re_x    p/p_inv = 1 + 0.31χ̄ + 0.05χ̄²" note="Hayes–Probstein weak interaction." />
        <Formula name="Kn" expr="Kn = λ/L    λ = kT/(√2 π d² p)" note="Schaaf–Chambre bridging w = Kn/(Kn+0.08)." />
        <Formula name="Waltrup–Billig" expr="L/H = √(θ/H) [50(p_r−1)+170(p_r−1)²] / (M²−1)" note="Isolator shock-train length." />
        <Formula name="Richardson CLα, Cnβ" expr="f' = [8(f₊ − f₋) − (f₊₊ − f₋₋)] / 12h" note="4th-order on α and β for 6DOF. Adaptive RK4 on the glide." />
      </div>
    </div>
  );
}

export function StabBlock({ study }: { study: StudyResult }) {
  const s = study.stab;
  return (
    <div>
      <Lead>Static derivatives at this point. A positive margin means the center of pressure is behind the CG.</Lead>
      <div className="hud-panel mb-3 grid grid-cols-2 gap-3 px-3 py-3">
        <Readout label="Static margin" value={fmt(s.staticMarginPct, 1)} unit="% L" tone={s.staticMargin > 0 ? "data" : "warn"} />
        <Readout label="Trim α" value={fmt(s.trimAlpha, 2)} unit="deg" size="sm" />
      </div>
      <Stat k="CLα ( /rad )" v={fmt(s.cla, 3)} />
      <Stat k="Cmα ( /rad )" v={fmt(s.cma, 3)} ok={s.cma < 0} />
      <Stat k="Cnβ ( /rad )" v={fmt(s.cnb, 3)} ok={s.cnb > 0} />
      <Stat k="Clβ ( /rad )" v={fmt(s.clb, 3)} />
      <Stat k="Long. stable" v={s.longitudinallyStable ? "yes" : "no"} ok={s.longitudinallyStable} />
      <Stat k="Dir. stable" v={s.directionallyStable ? "yes" : "no"} ok={s.directionallyStable} />
      <div className="mt-4 h-36">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={s.polar} margin={{ top: 4, right: 8, left: 0, bottom: 0 }}>
            <CartesianGrid stroke="rgba(255,255,255,0.08)" />
            <XAxis dataKey="a" tick={{ fill: "#ffd4da", fontSize: 10 }} />
            <YAxis tick={{ fill: "#ffd4da", fontSize: 10 }} />
            <Tooltip
              contentStyle={{ background: "#080003", border: "1px solid #ff1744", fontSize: 11, color: "#ffffff" }}
              labelFormatter={(v) => `α ${v}°`}
            />
            <Line type="monotone" dataKey="cl" stroke="#ff1744" dot={false} strokeWidth={1.5} name="CL" />
            <Line type="monotone" dataKey="cm" stroke="#ffffff" dot={false} strokeWidth={1.5} name="Cm" />
          </LineChart>
        </ResponsiveContainer>
      </div>
      <p className="mt-2 text-xs text-subtle">4th-order Richardson on α. Elevon on Geom tab for trim. CG at Flight x/L. Rotary derivatives live on the 6DOF tab.</p>
    </div>
  );
}

export function SixDofBlock({ study }: { study: StudyResult }) {
  const six = study.six;
  const d = six.derivs;
  const atm = study.atm;
  const data = six.samples.filter((_, i) => i % 2 === 0);
  if (six.skipped) {
    return <p className="text-xs leading-relaxed text-muted">{six.notes[0]}</p>;
  }
  return (
    <div>
      <Lead>Short-period motion at this condition. The trajectory tab is the long glide; this is whether the vehicle stays pointed.</Lead>
      <p className="mb-3 text-xs leading-relaxed text-muted">
        Flight at M {fmt(study.flightMach, 2)}, {fmt(atm.h / 1000, 1)} km, {fmt(atm.V, 0)} m/s, q∞{" "}
        {fmt(atm.q / 1000, 2)} kPa. Linear 6DOF from shock-expansion derivatives; RK4 from a +2° α pulse.
      </p>
      <p className="mb-2 hud-label">Static /rad</p>
      <Stat k="CLα" v={fmt(d.cla, 3)} />
      <Stat k="Cmα" v={fmt(d.cma, 3)} ok={d.cma < 0} />
      <Stat k="CDα" v={fmt(d.cda, 3)} />
      <Stat k="Cnβ" v={fmt(d.cnb, 3)} ok={d.cnb > 0} />
      <Stat k="Clβ" v={fmt(d.clb, 3)} />
      <Stat k="CYβ" v={fmt(d.cyb, 3)} />
      <Stat k="Static margin" v={`${fmt(d.staticMarginPct, 1)} % L`} ok={d.staticMargin > 0} />
      <Stat k="Trim α" v={`${fmt(d.trimAlpha, 2)}°`} />
      <p className="mt-3 mb-2 hud-label">Rotary (per rate hat)</p>
      <Stat k="Cmq" v={fmt(d.cmq, 3)} ok={d.cmq < 0} />
      <Stat k="CLq" v={fmt(d.clq, 3)} />
      <Stat k="Clp" v={fmt(d.clp, 3)} ok={d.clp < 0} />
      <Stat k="Cnr" v={fmt(d.cnr, 3)} ok={d.cnr < 0} />
      <Stat k="Clr" v={fmt(d.clr, 3)} />
      <Stat k="Cnp" v={fmt(d.cnp, 3)} />
      <p className="mt-3 mb-2 hud-label">Rigid-body modes (Etkin)</p>
      {six.modes.map((m) => (
        <div key={m.name} className="border-b border-border/80 py-2 last:border-0">
          <div className="flex items-baseline justify-between gap-2">
            <span className="text-xs text-fg">{m.name}</span>
            <span className={`font-mono text-[11px] ${m.stable ? "text-ok" : "text-warn"}`}>{m.stable ? "stable" : "unstable"}</span>
          </div>
          <p className="mt-0.5 font-mono text-[10px] text-subtle">
            ωn {fmt(m.wn, 3)} rad/s · ζ {fmt(m.zeta, 3)}
            {m.period > 0 && Number.isFinite(m.period) ? ` · T ${fmt(m.period, 2)} s` : ""}
            {m.tHalf > 0 && Number.isFinite(m.tHalf)
              ? ` · t½ ${fmt(m.tHalf, 2)} s`
              : m.tHalf < 0 && Number.isFinite(m.tHalf)
                ? ` · t2× ${fmt(-m.tHalf, 2)} s`
                : ""}
          </p>
          <p className="mt-1 text-[10px] leading-snug text-muted">{m.note}</p>
        </div>
      ))}
      <p className="mt-3 mb-2 hud-label">
        RK4 6DOF · {six.iterations} steps · dt {six.dt}s · α₀+2°
      </p>
      <Stat k="6DOF range" v={`${fmt(data.length ? data[data.length - 1].xKm : 0, 2)} km`} />
      <Stat k="6DOF altitude" v={`${fmt(data.length ? data[data.length - 1].hKm : 0, 2)} km`} />
      <Stat k="6DOF speed" v={`${fmt(data.length ? data[data.length - 1].V : 0, 0)} m/s`} />
      <Stat k="Point-mass range" v={`${fmt(study.traj.rangeKm, 1)} km`} />
      <p className="mt-3 mb-2 hud-label">Attitude</p>
      <div className="h-36">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data} margin={{ top: 4, right: 8, left: 0, bottom: 0 }}>
            <CartesianGrid stroke="rgba(255,23,68,0.16)" />
            <XAxis dataKey="t" type="number" domain={["auto", "auto"]} tickFormatter={(v) => `${Number(v).toFixed(1)}`} minTickGap={28} tick={{ fill: "#ffd4da", fontSize: 10 }} />
            <YAxis tick={{ fill: "#ffd4da", fontSize: 10 }} />
            <Tooltip
              contentStyle={{ background: "#080003", border: "1px solid #ff1744", fontSize: 11, color: "#ffffff" }}
              labelFormatter={(v) => `${Number(v).toFixed(2)} s`}
            />
            <Line type="monotone" dataKey="alphaDeg" stroke="#ff1744" dot={false} strokeWidth={1.5} name="α°" />
            <Line type="monotone" dataKey="qDeg" stroke="#ffffff" dot={false} strokeWidth={1.25} name="q °/s" />
            <Line type="monotone" dataKey="thetaDeg" stroke="#ff8fa0" dot={false} strokeWidth={1.1} name="θ°" />
          </LineChart>
        </ResponsiveContainer>
      </div>
      <p className="mt-3 mb-2 hud-label">6DOF path · h and γ</p>
      <div className="h-36">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data} margin={{ top: 4, right: 8, left: 0, bottom: 0 }}>
            <CartesianGrid stroke="rgba(255,23,68,0.16)" />
            <XAxis dataKey="xKm" type="number" domain={["auto", "auto"]} tickFormatter={(v) => `${Math.round(Number(v))}`} minTickGap={28} tick={{ fill: "#ffd4da", fontSize: 10 }} />
            <YAxis yAxisId="h" tick={{ fill: "#ffd4da", fontSize: 10 }} />
            <YAxis yAxisId="g" orientation="right" tick={{ fill: "#ff8fa0", fontSize: 10 }} />
            <Tooltip
              contentStyle={{ background: "#080003", border: "1px solid #ff1744", fontSize: 11, color: "#ffffff" }}
              labelFormatter={(v) => `${Number(v).toFixed(2)} km`}
            />
            <Line yAxisId="h" type="monotone" dataKey="hKm" stroke="#ff1744" dot={false} strokeWidth={1.5} name="h km" />
            <Line yAxisId="g" type="monotone" dataKey="gammaDeg" stroke="#ffffff" dot={false} strokeWidth={1.25} name="γ°" />
          </LineChart>
        </ResponsiveContainer>
      </div>
      <p className="mt-3 mb-2 hud-label">Point-mass entry</p>
      <div className="h-36">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={study.traj.samples.filter((_, i) => i % 2 === 0)} margin={{ top: 4, right: 8, left: 0, bottom: 0 }}>
            <CartesianGrid stroke="rgba(255,23,68,0.16)" />
            <XAxis dataKey="xKm" type="number" domain={["auto", "auto"]} tickFormatter={(v) => `${Math.round(Number(v))}`} minTickGap={28} tick={{ fill: "#ffd4da", fontSize: 10 }} />
            <YAxis yAxisId="h" tick={{ fill: "#ffd4da", fontSize: 10 }} />
            <YAxis yAxisId="g" orientation="right" tick={{ fill: "#ff8fa0", fontSize: 10 }} />
            <Tooltip
              contentStyle={{ background: "#080003", border: "1px solid #ff1744", fontSize: 11, color: "#ffffff" }}
              labelFormatter={(v) => `${Number(v).toFixed(0)} km`}
            />
            <Line yAxisId="h" type="monotone" dataKey="hKm" stroke="#ff1744" dot={false} strokeWidth={1.5} name="h km" />
            <Line yAxisId="g" type="monotone" dataKey="gammaDeg" stroke="#ffffff" dot={false} strokeWidth={1.25} name="γ°" />
          </LineChart>
        </ResponsiveContainer>
      </div>
      <div className="mt-3 space-y-2">
        <Formula
          name="Short period"
          expr="ω²_sp = Zα Mq / V − Mα     ζ = −(Zα/V + Mq) / (2 ω)"
          note="Etkin, Dynamics of Atmospheric Flight, ch. 6. Zα = −qS CLα / m."
        />
        <Formula
          name="Rotary incidence"
          expr="V_panel = V∞ î − ω × r_cg     q̂ = q L / 2V"
          note="Local velocity perturbation on every facet. Same strip Cp as the static polar."
        />
        <Formula
          name="RK4"
          expr="k₁=f(y), k₂=f(y+h k₁/2), k₃=f(y+h k₂/2), k₄=f(y+h k₃)    y ← y + h(k₁+2k₂+2k₃+k₄)/6"
          note="Classic 4th-order. Altitude and range are states of the same step, 800 × 0.02 s."
        />
        <Formula
          name="Euler pitch"
          expr="Iyy q̇ = q∞ S L (Cmα α + Cmq q̂)"
          note="Linear aero, Mirtich Iyy. C++ kernel (bowshock.cpp) integrates the same ODEs offline."
        />
      </div>
      <p className="mt-3 text-[11px] leading-relaxed text-subtle">{six.notes.join(" ")}</p>
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
      <Stat k="Isolator L/H" v={fmt(ic?.isolatorLH ?? 0, 1)} />
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
      <p className="mb-2 hud-label">Stations</p>
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
      <Lead>
        Published gas identities, independent of this shape. Whether the solid is closed is on the path above and on Output → CAD.
      </Lead>
      <Stat k="Kernel" v={`${pass}/${n} pass`} ok={pass === n} />
      <p className="mt-2 mb-3 text-xs leading-relaxed text-muted">
        Gas identities (Anderson, NACA 1135, US76, Kantrowitz), Taylor–Maccoll against an independent RK4, Prandtl–Meyer,
        skin-friction and heating correlations against independent implementations, an 8° wedge strip, a turning ramp
        against analytic shock-expansion, and a 2D HLLC Euler shock. The percent is |got − expected| / |expected|
        (checks with reference 0 report the absolute error against the bound). Full table: Data tab → validation.
      </p>
      {shown.map((c) => (
        <div key={c.id} className="border-b border-border/80 py-2 last:border-0">
          <div className="flex items-baseline justify-between gap-2">
            <span className="text-xs text-fg">{c.name}</span>
            <span className={`font-mono text-[11px] ${c.pass ? "text-ok" : "text-warn"}`}>{c.pass ? "PASS" : "FAIL"}</span>
          </div>
          <p className="mt-1 font-mono text-[10px] leading-snug text-subtle">{c.formula}</p>
          <p className="mt-0.5 font-mono text-[10px] text-muted">
            got {fmt(c.got, 4)} · expect {fmt(c.expected, 4)} · err {(c.relErr * 100).toFixed(2)}% (tol {c.tol < 0.01 ? c.tol.toExponential(0) : `${(c.tol * 100).toFixed(1)}%`}, {c.basis}) · {c.source}
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
      <Lead>3DOF glide from this energy. The plot under it is the same vehicle in 6DOF, not a second mission.</Lead>
      <Stat k="Range" v={withUnit(t.rangeKm, 1, "km")} />
      <Stat k="Eq-glide range" v={withUnit(t.rangeEqKm, 1, "km")} />
      <Stat k="Flat glide" v={withUnit(t.flatGlideKm, 1, "km")} />
      <div className="mt-3 h-36">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data} margin={{ top: 4, right: 8, left: 0, bottom: 0 }}>
            <CartesianGrid stroke="rgba(255,23,68,0.16)" />
            <XAxis dataKey="xKm" type="number" domain={["auto", "auto"]} tickFormatter={(v) => `${Math.round(Number(v))}`} minTickGap={28} tick={{ fill: "#ffd4da", fontSize: 10 }} />
            <YAxis yAxisId="h" tick={{ fill: "#ffd4da", fontSize: 10 }} />
            <YAxis yAxisId="g" orientation="right" tick={{ fill: "#ff8fa0", fontSize: 10 }} />
            <Tooltip
              contentStyle={{ background: "#080003", border: "1px solid #ff1744", fontSize: 11, color: "#ffffff" }}
              labelFormatter={(v) => `${Number(v).toFixed(0)} km`}
            />
            <Line yAxisId="h" type="monotone" dataKey="hKm" stroke="#ff1744" dot={false} strokeWidth={1.5} name="h km" />
            <Line yAxisId="g" type="monotone" dataKey="gammaDeg" stroke="#ffffff" dot={false} strokeWidth={1.25} name="γ°" />
          </LineChart>
        </ResponsiveContainer>
      </div>
      {!study.six.skipped && study.six.samples.length > 2 ? (
        <>
          <p className="mt-3 mb-2 hud-label">6DOF at this condition</p>
          <div className="h-36">
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={study.six.samples.filter((_, i) => i % 2 === 0)} margin={{ top: 4, right: 8, left: 0, bottom: 0 }}>
                <CartesianGrid stroke="rgba(255,23,68,0.16)" />
                <XAxis dataKey="t" type="number" domain={["auto", "auto"]} tickFormatter={(v) => `${Number(v).toFixed(1)}`} minTickGap={28} tick={{ fill: "#ffd4da", fontSize: 10 }} />
                <YAxis tick={{ fill: "#ffd4da", fontSize: 10 }} />
                <Tooltip
                  contentStyle={{ background: "#080003", border: "1px solid #ff1744", fontSize: 11, color: "#ffffff" }}
                  labelFormatter={(v) => `${Number(v).toFixed(2)} s`}
                />
                <Line type="monotone" dataKey="alphaDeg" stroke="#ff1744" dot={false} strokeWidth={1.5} name="α°" />
                <Line type="monotone" dataKey="qDeg" stroke="#ffffff" dot={false} strokeWidth={1.25} name="q °/s" />
                <Line type="monotone" dataKey="hKm" stroke="#ff8fa0" dot={false} strokeWidth={1.1} name="h km" />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </>
      ) : null}
      <div className="mt-3">
      <Stat k="Vacuum arc" v={t.vacuumRangeKm > 0 ? withUnit(t.vacuumRangeKm, 1, "km") : "—"} />
      <Stat k="Time" v={withUnit(t.timeS, 0, "s")} />
      <Stat k="Max q" v={withUnit(t.maxQ / 1000, 2, "kPa")} />
      <Stat k="Max-q alt" v={withUnit(t.maxQAltKm, 1, "km")} />
      <Stat k="Peak heat" v={withUnit(t.maxHeat, 2, "W/cm²")} />
      <Stat k="Peak-heat alt" v={withUnit(t.maxHeatAltKm, 1, "km")} />
      <Stat k="Peak n (3DOF)" v={withUnit(t.maxN, 2, "g")} />
      <Stat k="Allen–Eggers" v={t.allenG > 0 ? withUnit(t.allenG, 2, "g") : "—"} />
      <Stat
        k="Allen alt"
        v={t.allenG <= 0 ? "—" : t.allenUnderground ? "surface" : withUnit(t.allenAltKm, 1, "km")}
      />
      <Stat k="AE speed" v={t.allenV > 0 ? withUnit(t.allenV, 0, "m/s") : "—"} />
      <Stat k="AE heat" v={t.allenQ > 0 ? withUnit(t.allenQ, 2, "W/cm²") : "—"} />
      <Stat
        k="AE heat alt"
        v={t.allenQ <= 0 ? "—" : t.allenHeatAltKm <= 0 ? "surface" : withUnit(t.allenHeatAltKm, 1, "km")}
      />
      <Stat k="Heat load" v={withUnit(t.heatLoad, 1, "J/cm²")} />
      <Stat k="Tw lump" v={withUnit(t.twPeak, 0, "K")} />
      <Stat k="Final V" v={withUnit(t.finalV, 0, "m/s")} />
      <Stat k="Final h" v={withUnit(t.finalH, 1, "km")} />
      </div>
      {t.notes.map((n) => (
        <p key={n.slice(0, 24)} className="mt-2 text-xs leading-relaxed text-subtle">
          {n}
        </p>
      ))}
    </div>
  );
}
