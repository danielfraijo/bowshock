/**
 * Output formatters: classical USAF DATCOM for004-like text, Missile DATCOM for006-style
 * summary, CSV, JSON, and a 6DOF-ready aero table.
 *
 * Layout notes (honest):
 *  - Classical DATCOM for004.dat column order (ALPHA CD CL CM CN CA XCP CLA CMA CYB CNB CLB)
 *    is the widely published MATLAB Aerospace Toolbox / datcomimport convention; exact
 *    column widths / header banners of the 1976 FORTRAN printer were NOT verified against
 *    a real for004.dat in this work (see NOTES.md).
 *  - Missile DATCOM for006.dat total-configuration summary columns
 *    (ALPHA CN CM CA CY CLN CLL CNA CMA CYB CLNB CLLB XCP) match the published user's-manual
 *    description and sample for006 fragments (arktools/arkhangar). Exact page banners and
 *    partial-output blocks are omitted.
 */
import type { CaseResult, PointResult, RunResult } from "./types.ts";
import { RAD } from "./util.ts";

function f(n: number, w = 10, d = 4): string {
  if (!Number.isFinite(n)) return "NaN".padStart(w);
  const s = n.toFixed(d);
  return s.length >= w ? s.slice(0, w) : s.padStart(w);
}
function f6(n: number): string {
  return f(n, 10, 4);
}

function echoConfig(r: RunResult): string[] {
  const g = r.geometry;
  const lines: string[] = [];
  lines.push(`***** BOWSHOCK DATCOM SUBSTITUTE *****`);
  lines.push(`CASEID ${r.config.caseId ?? r.config.name}`);
  lines.push(`CONFIG ${r.config.kind.toUpperCase()}  LENGTH=${g.length.toFixed(4)} m  DMAX=${g.maxDiameter.toFixed(4)} m`);
  lines.push(
    `  SREF=${g.sRef.toFixed(6)} m2  LREF=${g.lRef.toFixed(4)} m  LATREF=${g.latRef.toFixed(4)} m  XCG=${g.xcg.toFixed(4)} m`,
  );
  lines.push(
    `  VOLUME=${g.volume.toFixed(6)} m3  PLANFORM=${g.planformArea.toFixed(6)} m2  WETTED=${g.wettedArea.toFixed(6)} m2  BASE=${g.baseArea.toFixed(6)} m2`,
  );
  lines.push(`  NOSE FINENESS=${g.noseFineness.toFixed(3)}  BODY FINENESS=${g.bodyFineness.toFixed(3)}`);
  for (const s of g.sets) {
    lines.push(
      `  SET ${s.name} (${s.role}) NPANEL=${s.nPanels}  Se=${s.exposedArea.toFixed(6)}  be=${s.exposedSpan.toFixed(4)}  AR=${s.aspectRatio.toFixed(3)}  lambda=${s.taper.toFixed(3)}  LLE=${s.sweepLE.toFixed(1)}  tau=${s.tauRoverS.toFixed(3)}`,
    );
  }
  if (r.warnings.length) {
    lines.push("WARNINGS:");
    for (const w of r.warnings) lines.push(`  * ${w}`);
  }
  lines.push("");
  return lines;
}

/** Classical USAF DATCOM for004.dat-like (one block per Mach). Derivatives in /deg by default. */
export function formatUSAF(r: RunResult, perRad = false): string {
  const lines = echoConfig(r);
  const k = perRad ? 1 : RAD;
  for (const c of r.cases) {
    lines.push(`***** MACH NUMBER = ${c.mach.toFixed(3)}   ALTITUDE = ${c.altKm.toFixed(2)} km *****`);
    lines.push(`  RE/L = ${c.reynoldsPerM.toExponential(3)} /m   Q = ${(c.qPa / 1000).toFixed(3)} kPa   T = ${c.tK.toFixed(1)} K`);
    lines.push(
      "   ALPHA        CD        CL        CM        CN        CA       XCP       CLA       CMA       CYB       CNB       CLB",
    );
    for (const p of c.points) {
      lines.push(
        `${f(p.alpha, 9, 2)}${f6(p.CD)}${f6(p.CL)}${f6(p.Cm)}${f6(p.CN)}${f6(p.CA)}${f6(p.xcp / r.geometry.lRef)}${f6(p.derivs.CLa / k)}${f6(p.derivs.Cma / k)}${f6(p.derivs.CYb / k)}${f6(p.derivs.Cnb / k)}${f6(p.derivs.Clb / k)}`,
      );
    }
    lines.push(`  METHOD: ${c.points[0]?.validity.method ?? "?"}`);
    for (const n of c.points[0]?.validity.notes ?? []) lines.push(`  NOTE: ${n}`);
    lines.push("");
  }
  return lines.join("\n");
}

/** Missile DATCOM for006.dat-like total-configuration summary. */
export function formatMissile(r: RunResult, perRad = false): string {
  const lines = echoConfig(r);
  const k = perRad ? 1 : RAD;
  for (const c of r.cases) {
    lines.push(`***** THE BOWSHOCK MISSILE DATCOM SUBSTITUTE *****`);
    lines.push(` MACH NO = ${c.mach.toFixed(3)}     ALTITUDE = ${c.altKm.toFixed(2)} km     REYNOLDS NO /L = ${c.reynoldsPerM.toExponential(3)}`);
    lines.push(` --------------- TOTAL CONFIGURATION AERODYNAMICS ----------------`);
    lines.push(
      "   ALPHA        CN        CM        CA        CY       CLN       CLL       CNA       CMA       CYB      CLNB      CLLB       XCP",
    );
    for (const p of c.points) {
      lines.push(
        `${f(p.alpha, 9, 2)}${f6(p.CN)}${f6(p.Cm)}${f6(p.CA)}${f6(p.CY)}${f6(p.Cn)}${f6(p.Cl)}${f6(p.derivs.CNa / k)}${f6(p.derivs.Cma / k)}${f6(p.derivs.CYb / k)}${f6(p.derivs.Cnb / k)}${f6(p.derivs.Clb / k)}${f6(p.xcp / r.geometry.lRef)}`,
      );
    }
    lines.push(` STATIC MARGIN (calibers, @ alpha=0-ish) = ${f(c.points.find((p) => Math.abs(p.alpha) < 1e-6)?.derivs.staticMargin ?? c.points[0]?.derivs.staticMargin ?? 0, 8, 3)}`);
    lines.push(` DYNAMIC: Clp=${f6(c.points[0]?.derivs.Clp ?? 0)}  Cmq=${f6(c.points[0]?.derivs.Cmq ?? 0)}  Cnr=${f6(c.points[0]?.derivs.Cnr ?? 0)}`);
    lines.push(` CONTROL: CNd=${f6(c.points[0]?.derivs.CNd / k)}  Cmd=${f6(c.points[0]?.derivs.Cmd / k)}  Cld=${f6(c.points[0]?.derivs.Cld / k)} (per ${perRad ? "rad" : "deg"})`);
    lines.push(` METHOD: ${c.points[0]?.validity.method ?? "?"}`);
    for (const n of c.points[0]?.validity.notes ?? []) lines.push(` NOTE: ${n}`);
    lines.push("");
  }
  return lines.join("\n");
}

export function formatCSV(r: RunResult, perRad = false): string {
  const k = perRad ? 1 : RAD;
  const head = [
    "mach",
    "alt_km",
    "alpha_deg",
    "beta_deg",
    "phi_deg",
    "CN",
    "CA",
    "Cm",
    "CY",
    "Cn",
    "Cl",
    "CL",
    "CD",
    "LD",
    "xcp_m",
    "xcp_calibers",
    "CNa",
    "Cma",
    "CYb",
    "Cnb",
    "Clb",
    "Clp",
    "Cmq",
    "Cnr",
    "CNd",
    "Cmd",
    "Cld",
    "static_margin",
    "confidence",
    "method",
  ];
  const rows = [head.join(",")];
  for (const c of r.cases) {
    for (const p of c.points) {
      rows.push(
        [
          c.mach,
          c.altKm,
          p.alpha,
          p.beta,
          p.phi,
          p.CN,
          p.CA,
          p.Cm,
          p.CY,
          p.Cn,
          p.Cl,
          p.CL,
          p.CD,
          p.LD,
          p.xcp,
          p.xcp / r.geometry.lRef,
          p.derivs.CNa / k,
          p.derivs.Cma / k,
          p.derivs.CYb / k,
          p.derivs.Cnb / k,
          p.derivs.Clb / k,
          p.derivs.Clp,
          p.derivs.Cmq,
          p.derivs.Cnr,
          p.derivs.CNd / k,
          p.derivs.Cmd / k,
          p.derivs.Cld / k,
          p.derivs.staticMargin,
          p.validity.confidence,
          JSON.stringify(p.validity.method),
        ].join(","),
      );
    }
  }
  return rows.join("\n");
}

export function formatJSON(r: RunResult): string {
  return JSON.stringify(r, null, 2);
}

/**
 * Shape expected by a linear 6DOF that consumes StabDerivs-like tables
 * (see src/lib/waverider/sixdof.ts StabDerivs). One entry per (mach, alpha).
 */
export interface SixDofAeroRow {
  mach: number;
  alphaDeg: number;
  betaDeg: number;
  cl: number;
  cd: number;
  cm: number;
  cy: number;
  cn: number;
  cll: number;
  cla: number;
  cda: number;
  cma: number;
  cyb: number;
  cnb: number;
  clb: number;
  clp: number;
  cmq: number;
  cnr: number;
  staticMargin: number;
  method: string;
  confidence: string;
}

export function toSixDofTable(r: RunResult): SixDofAeroRow[] {
  const out: SixDofAeroRow[] = [];
  for (const c of r.cases) {
    for (const p of c.points) {
      out.push({
        mach: c.mach,
        alphaDeg: p.alpha,
        betaDeg: p.beta,
        cl: p.CL,
        cd: p.CD,
        cm: p.Cm,
        cy: p.CY,
        cn: p.Cn,
        cll: p.Cl,
        cla: p.derivs.CLa,
        cda: 0,
        cma: p.derivs.Cma,
        cyb: p.derivs.CYb,
        cnb: p.derivs.Cnb,
        clb: p.derivs.Clb,
        clp: p.derivs.Clp,
        cmq: p.derivs.Cmq,
        cnr: p.derivs.Cnr,
        staticMargin: p.derivs.staticMargin,
        method: p.validity.method,
        confidence: p.validity.confidence,
      });
    }
  }
  return out;
}

export function formatSixDofJSON(r: RunResult): string {
  return JSON.stringify(
    {
      geometry: r.geometry,
      warnings: r.warnings,
      table: toSixDofTable(r),
    },
    null,
    2,
  );
}

export type OutputFormat = "usaf" | "missile" | "csv" | "json" | "sixdof";

export function format(r: RunResult, fmt: OutputFormat, perRad = false): string {
  switch (fmt) {
    case "usaf":
      return formatUSAF(r, perRad);
    case "missile":
      return formatMissile(r, perRad);
    case "csv":
      return formatCSV(r, perRad);
    case "json":
      return formatJSON(r);
    case "sixdof":
      return formatSixDofJSON(r);
  }
}

export function downloadName(fmt: OutputFormat): string {
  switch (fmt) {
    case "usaf":
      return "for004.dat";
    case "missile":
      return "for006.dat";
    case "csv":
      return "datcom.csv";
    case "json":
      return "datcom.json";
    case "sixdof":
      return "aero-6dof.json";
  }
}

export function summarizePoint(p: PointResult): string {
  return `a=${p.alpha.toFixed(1)} CN=${p.CN.toFixed(3)} CA=${p.CA.toFixed(3)} Cm=${p.Cm.toFixed(3)} CL=${p.CL.toFixed(3)} CD=${p.CD.toFixed(3)} L/D=${p.LD.toFixed(2)} SM=${p.derivs.staticMargin.toFixed(2)} [${p.validity.confidence}]`;
}

export function caseTable(c: CaseResult, lRef: number): { alpha: number; CN: number; CA: number; Cm: number; CL: number; CD: number; CY: number; Cl: number; Cn: number; xcp: number; CNa: number; Cma: number; SM: number }[] {
  return c.points.map((p) => ({
    alpha: p.alpha,
    CN: p.CN,
    CA: p.CA,
    Cm: p.Cm,
    CL: p.CL,
    CD: p.CD,
    CY: p.CY,
    Cl: p.Cl,
    Cn: p.Cn,
    xcp: p.xcp / lRef,
    CNa: p.derivs.CNa,
    Cma: p.derivs.Cma,
    SM: p.derivs.staticMargin,
  }));
}
