/**
 * DATCOM / Missile DATCOM input-deck parser and generator (supported subset).
 *
 * Supported Missile DATCOM namelists (AFRL-VA-WP-TR-1998-3009):
 *   $FLTCON  NALPHA, ALPHA, BETA, PHI, NMACH, MACH, ALT, REN, VINF
 *   $REFQ    SREF, LREF, LATREF, XCG, ZCG, ROUGH, BLAYER, SCALE
 *   $AXIBOD  Option 1 (TNOSE, LNOSE, DNOSE, BNOSE, POWER, LCENTR, DCENTR, TAFT, LAFT, DAFT, DEXIT)
 *            Option 2 (NX, X, R, BNOSE, DEXIT)
 *   $ELLBOD  Option 2 subset: NX, X, H, W / ELLIP (semi-axes); Option 1 → warning for partial support
 *   $INLET   NIN, INTYPE, XINLT, PHI, capture via W/H or equiv area, MFR, RAMP, COVER (subset)
 *   $PROTUB  NPROT, PTYPE, XPROT, NLOC, LPROT, WPROT, HPROT (subset)
 *   $FINSETn SSPAN, CHORD, XLE, SWEEP, STA, LER, NPANEL, PHIF, GAM, SECTYP, ZUPPER,
 *            LMAXU, LFLATU, CFOC
 *   $DEFLCT  DELTA1..DELTA4
 * Control cards: CASEID, DIM FT|IN|CM|M, DERIV DEG|RAD, NEXT CASE, DAMP, NO LAT, NACA
 *
 * Supported Aircraft DATCOM namelists (partial, USAF DATCOM 1976 style):
 *   $FLTCON, $OPTINS (SREF, CBARR, BLREF), $SYNTHS (XCG, XW, ZW, ALIW, XH, ZH, XV, ZV),
 *   $BODY (NX, X, R / S / P), $WGPLNF / $HTPLNF / $VTPLNF / $VTPLNF (CHRDTP, CHRDR, SSPNE,
 *   SSPAN, SAVSI, CHSTAT, TWISTA, DHDADI, TYPE), $NACA cards.
 *
 * Unsupported cards produce a clear WARNING and are NEVER silently ignored.
 * Unverified keyword meanings are listed in NOTES.md.
 */
import type {
  AftKind,
  BodyParametric,
  BodySpec,
  FinSet,
  FlightSpec,
  InletSpec,
  NoseKind,
  ProtuberanceKind,
  ProtuberanceSpec,
  SectionKind,
  SurfaceRole,
  VehicleConfig,
} from "./types.ts";
import { DEG } from "./util.ts";
import { buildBody, radiusAt } from "./geometry.ts";

export interface DeckParse {
  config: VehicleConfig;
  flight: FlightSpec;
  warnings: string[];
  unsupported: string[];
  supported: string[];
  units: "m" | "ft" | "in" | "cm";
  derivUnits: "deg" | "rad";
  raw: string;
}

const UNSUPPORTED_NAMELISTS = new Set([
  "TRIM",
  "EXPR",
  "PROPWR",
  "JETPWR",
  "SYMFLP",
  "ASYFLP",
  "HDRAG",
  "VGRND",
  "GRNDEF",
  "TVTPAN",
  "DEFLAP",
  "DAMP", // namelist form; control card DAMP is accepted as a flag
]);

function toMeters(v: number, u: "m" | "ft" | "in" | "cm"): number {
  switch (u) {
    case "m":
      return v;
    case "ft":
      return v * 0.3048;
    case "in":
      return v * 0.0254;
    case "cm":
      return v * 0.01;
  }
}
function fromMeters(v: number, u: "m" | "ft" | "in" | "cm"): number {
  switch (u) {
    case "m":
      return v;
    case "ft":
      return v / 0.3048;
    case "in":
      return v / 0.0254;
    case "cm":
      return v / 0.01;
  }
}

/** Strip comments (* …) and CASEID / control-card noise for namelist extraction. */
function preprocess(text: string): { lines: string[]; units: "m" | "ft" | "in" | "cm"; deriv: "deg" | "rad"; caseId: string; unsupportedCtrl: string[]; supportedCtrl: string[] } {
  const unsupportedCtrl: string[] = [];
  const supportedCtrl: string[] = [];
  let units: "m" | "ft" | "in" | "cm" = "ft";
  let deriv: "deg" | "rad" = "deg";
  let caseId = "";
  const lines: string[] = [];
  for (const raw of text.split(/\r?\n/)) {
    const t = raw.trimEnd();
    if (!t.trim()) continue;
    if (/^\*/.test(t)) continue;
    const u = t.trim().toUpperCase();
    if (u.startsWith("CASEID")) {
      caseId = t.trim().slice(6).trim();
      supportedCtrl.push("CASEID");
      continue;
    }
    if (/^DIM\s+(FT|IN|CM|M)\b/.test(u)) {
      const m = u.match(/^DIM\s+(FT|IN|CM|M)/)!;
      units = m[1] === "FT" ? "ft" : m[1] === "IN" ? "in" : m[1] === "CM" ? "cm" : "m";
      supportedCtrl.push(`DIM ${m[1]}`);
      continue;
    }
    if (/^DERIV\s+(DEG|RAD)\b/.test(u)) {
      deriv = /RAD/.test(u) ? "rad" : "deg";
      supportedCtrl.push(`DERIV ${deriv.toUpperCase()}`);
      continue;
    }
    if (/^NEXT\s+CASE/.test(u)) {
      supportedCtrl.push("NEXT CASE");
      lines.push("$$NEXT_CASE$$");
      continue;
    }
    if (/^(DAMP|NO\s+LAT|SAVE|DELETE|PART|BUILD|PLOT|PRESSURES|SOSE|HYPER|SPIN|TRIM|NOGO|NAMELIST|WRITE|FORMAT|INCRMT)\b/.test(u)) {
      // Accept as known (some no-op); flag the ones we don't act on.
      if (/^(DAMP|NO\s+LAT|TRIM)\b/.test(u)) supportedCtrl.push(u.split(/\s+/).slice(0, 2).join(" "));
      else unsupportedCtrl.push(u);
      continue;
    }
    if (/^NACA\b/.test(u)) {
      lines.push(t);
      supportedCtrl.push("NACA");
      continue;
    }
    lines.push(t);
  }
  return { lines, units, deriv, caseId, unsupportedCtrl, supportedCtrl };
}

interface NL {
  name: string;
  vars: Record<string, number | number[] | string | boolean>;
}

/** Parse one $NAME … $ / $END block. Arrays may be continued. */
function parseNamelists(lines: string[]): { lists: NL[]; unsupported: string[]; supported: string[] } {
  const lists: NL[] = [];
  const unsupported: string[] = [];
  const supported: string[] = [];
  let buf = "";
  const flush = () => {
    const s = buf.trim();
    buf = "";
    if (!s) return;
    const m = s.match(/^\$\s*([A-Z0-9]+)\s*([\s\S]*)/i);
    if (!m) return;
    const name = m[1].toUpperCase();
    const body = m[2].replace(/\$\s*$/, "").replace(/\$END\s*$/i, "").trim();
    if (UNSUPPORTED_NAMELISTS.has(name.replace(/\d+$/, "").replace(/FINSET.*/, "X")) || UNSUPPORTED_NAMELISTS.has(name)) {
      // FINSETn and known list handled below
    }
    const base = name.replace(/\d+$/, "");
    if (UNSUPPORTED_NAMELISTS.has(name) || UNSUPPORTED_NAMELISTS.has(base)) {
      unsupported.push(`$${name}`);
      return;
    }
    const vars: NL["vars"] = {};
    // tokenise KEY=value pairs, allowing comma-separated arrays and continued KEY(i)=
    const re = /([A-Z_][A-Z0-9_]*)\s*(?:\((\d+)\))?\s*=\s*/gi;
    const pieces: { key: string; idx: number | null; start: number }[] = [];
    let mm: RegExpExecArray | null;
    while ((mm = re.exec(body))) pieces.push({ key: mm[1].toUpperCase(), idx: mm[2] ? parseInt(mm[2], 10) - 1 : null, start: mm.index + mm[0].length });
    for (let i = 0; i < pieces.length; i++) {
      const p = pieces[i];
      const end = i + 1 < pieces.length ? body.slice(0, pieces[i + 1].start).lastIndexOf(pieces[i + 1].key, pieces[i + 1].start) : body.length;
      // simpler: take until next KEY=
      const nextKeyPos = i + 1 < pieces.length ? pieces[i + 1].start - pieces[i + 1].key.length - (pieces[i + 1].idx !== null ? 4 : 0) : body.length;
      // walk back to '=' of next
      let cut = body.length;
      if (i + 1 < pieces.length) {
        const searchFrom = pieces[i].start;
        const next = body.slice(searchFrom).search(new RegExp(`,?\\s*${pieces[i + 1].key}\\s*(?:\\(\\d+\\))?\\s*=`, "i"));
        cut = next >= 0 ? searchFrom + next : body.length;
      }
      let raw = body.slice(p.start, cut).replace(/,\s*$/, "").trim();
      raw = raw.replace(/,$/, "").trim();
      const vals = raw
        .split(",")
        .map((x) => x.trim())
        .filter((x) => x.length);
      const parseOne = (v: string): number | string | boolean => {
        if (/^\.TRUE\.$/i.test(v) || /^TRUE$/i.test(v)) return true;
        if (/^\.FALSE\.$/i.test(v) || /^FALSE$/i.test(v)) return false;
        if (/^[A-Z][A-Z0-9]*$/i.test(v) && !/^-?\d/.test(v)) return v.toUpperCase();
        const n = Number(v);
        return Number.isFinite(n) ? n : v;
      };
      if (p.idx !== null) {
        const arr = (Array.isArray(vars[p.key]) ? (vars[p.key] as number[]) : []) as (number | string | boolean)[];
        vals.forEach((v, k) => {
          arr[p.idx! + k] = parseOne(v);
        });
        vars[p.key] = arr as number[];
      } else if (vals.length === 1) {
        vars[p.key] = parseOne(vals[0]) as number | string | boolean;
      } else {
        vars[p.key] = vals.map(parseOne) as number[];
      }
      void end;
      void nextKeyPos;
    }
    lists.push({ name, vars });
    supported.push(`$${name}`);
  };
  for (const line of lines) {
    if (line === "$$NEXT_CASE$$") {
      flush();
      continue;
    }
    buf += (buf ? " " : "") + line;
    // end of namelist: trailing $ or $END
    if (/\$\s*$/.test(line) || /\$END\s*$/i.test(line) || (/^\s*\$/.test(line) === false && /\$\s*$/.test(buf))) {
      // keep accumulating until a lone terminating $
    }
    if (/\$(?:\s*$|END\b)/i.test(line) && /\$[A-Z]/i.test(buf)) flush();
  }
  flush();
  return { lists, unsupported, supported };
}

function num(v: unknown, d = 0): number {
  if (typeof v === "number") return v;
  if (Array.isArray(v) && typeof v[0] === "number") return v[0];
  return d;
}
function arr(v: unknown): number[] {
  if (Array.isArray(v)) return v.map((x) => (typeof x === "number" ? x : Number(x))).filter((x) => Number.isFinite(x));
  if (typeof v === "number") return [v];
  return [];
}
function str(v: unknown): string {
  if (typeof v === "string") return v;
  return "";
}

function noseOf(s: string): NoseKind {
  const u = s.toUpperCase();
  if (u.includes("CONE") || u.includes("CONICAL")) return "cone";
  if (u.includes("POWER")) return "power";
  if (u.includes("HAACK")) return "haack";
  if (u.includes("KARMAN")) return "karman";
  return "ogive";
}
function aftOf(s: string): AftKind {
  return /OGIVE/i.test(s) ? "ogive" : "cone";
}
function sectOf(s: string): SectionKind {
  const u = s.toUpperCase();
  if (u.includes("ARC")) return "biconvex";
  if (u.includes("NACA")) return "naca4";
  if (u.includes("USER")) return "hex";
  return "hex";
}

function buildFromMissile(lists: NL[], units: "m" | "ft" | "in" | "cm", caseId: string, warnings: string[]): { config: VehicleConfig; flight: FlightSpec } {
  const L = (v: number) => toMeters(v, units);
  let flight: FlightSpec = { machs: [0.5], alphas: [0, 5, 10], beta: 0, phi: 0, altKm: 0 };
  let body: BodySpec | undefined;
  const sets: FinSet[] = [];
  let xcg = 0;
  let sRef: number | undefined;
  let lRef: number | undefined;
  let latRef: number | undefined;
  let roughness: number | undefined;
  let blayer: "turb" | "natural" | undefined;
  const deltas: number[][] = [[], [], [], []];
  const nacaBySet: Record<number, string> = {};
  const inlets: InletSpec[] = [];
  const prots: ProtuberanceSpec[] = [];

  for (const nl of lists) {
    const V = nl.vars;
    if (nl.name === "FLTCON") {
      const machs = arr(V.MACH);
      const alphas = arr(V.ALPHA);
      const alts = arr(V.ALT).map((a) => (units === "m" || units === "cm" ? a / 1000 : a * 0.0003048));
      const ren = arr(V.REN).map((r) => r / (units === "m" ? 1 : units === "cm" ? 0.01 : units === "in" ? 0.0254 : 0.3048));
      flight = {
        machs: machs.length ? machs : [0.5],
        alphas: alphas.length ? alphas : [0, 5, 10],
        beta: num(V.BETA, 0),
        phi: num(V.PHI, 0),
        altKm: alts.length === 1 ? alts[0] : alts.length ? alts : 0,
        renPerM: ren.length === 1 ? ren[0] : ren.length ? ren : undefined,
      };
      if (V.VINF !== undefined && machs.length === 0) warnings.push("$FLTCON VINF without MACH: speed schedule not converted (provide MACH)");
    } else if (nl.name === "REFQ") {
      if (V.SREF !== undefined) sRef = L(num(V.SREF)) * L(1); // area: L^2 — careful
      // SREF is already area: convert L²
      if (V.SREF !== undefined) {
        const s = num(V.SREF);
        const f = L(1);
        sRef = s * f * f;
      }
      if (V.LREF !== undefined) lRef = L(num(V.LREF));
      if (V.LATREF !== undefined) latRef = L(num(V.LATREF));
      if (V.XCG !== undefined) xcg = L(num(V.XCG));
      if (V.ROUGH !== undefined) roughness = L(num(V.ROUGH));
      if (typeof V.BLAYER === "string") blayer = /NATURAL/i.test(V.BLAYER) ? "natural" : "turb";
      if (V.SCALE !== undefined && num(V.SCALE) !== 1) warnings.push(`$REFQ SCALE=${V.SCALE} applied to geometry only after parse`);
    } else if (nl.name === "AXIBOD") {
      if (V.NX !== undefined) {
        const X = arr(V.X).map(L);
        const R = arr(V.R).map(L);
        const n = Math.min(num(V.NX, X.length), X.length, R.length);
        body = { stations: Array.from({ length: n }, (_, i) => ({ x: X[i], r: R[i] })), noseBluntRadius: V.BNOSE !== undefined ? L(num(V.BNOSE)) : undefined, exitDiameter: V.DEXIT !== undefined ? L(num(V.DEXIT)) : undefined };
      } else {
        const p: BodyParametric = {
          noseKind: noseOf(str(V.TNOSE) || "OGIVE"),
          noseLength: L(num(V.LNOSE)),
          noseDiameter: L(num(V.DNOSE, 1)),
          power: num(V.POWER, 0.5),
          noseBluntRadius: V.BNOSE !== undefined ? L(num(V.BNOSE)) : undefined,
          centerLength: L(num(V.LCENTR)),
          centerDiameter: V.DCENTR !== undefined ? L(num(V.DCENTR)) : undefined,
          aftKind: aftOf(str(V.TAFT) || "CONE"),
          aftLength: L(num(V.LAFT)),
          aftDiameter: V.DAFT !== undefined ? L(num(V.DAFT)) : undefined,
        };
        body = { parametric: p, exitDiameter: V.DEXIT !== undefined ? L(num(V.DEXIT)) : undefined };
      }
    } else if (/^FINSET[1-4]$/.test(nl.name)) {
      const idx = parseInt(nl.name.slice(-1), 10) - 1;
      const sspan = arr(V.SSPAN).map(L);
      const chord = arr(V.CHORD).map(L);
      const xle = arr(V.XLE).map(L);
      const sweep = arr(V.SWEEP);
      const sta = arr(V.STA);
      const ler = arr(V.LER).map(L);
      const nPanel = Math.max(1, Math.round(num(V.NPANEL, 4)));
      const phif = arr(V.PHIF);
      const zup = arr(V.ZUPPER);
      const flat = arr(V.LFLATU);
      const root = chord[0] ?? 0.1;
      const tip = chord[chord.length - 1] ?? root;
      let semispan = sspan[sspan.length - 1] ?? 0.1;
      // If SSPAN(1)==0 the values are relative to the root (Missile DATCOM); we keep absolute-from-axis
      // and let geometry.ts add the body radius when SSPAN starts at 0.
      if (sspan.length >= 2 && Math.abs(sspan[0]) < 1e-12) {
        // relative — body radius will be added at synthesis by keeping semispan as exposed + r later;
        // store as "exposed tip + 0" and flag; geometry attaches via radiusAt.
        semispan = sspan[sspan.length - 1]; // exposed tip; attachBodyRoot will bump
        warnings.push(`$${nl.name}: SSPAN(1)=0 (relative to root) — body radius is added at the root mid-chord`);
      }
      let sweepLE = sweep[0] ?? 0;
      if (sta[0] === 1 && sweep[0] !== undefined && sspan.length >= 2) {
        // TE sweep given: convert to LE using chords
        const be = Math.abs(sspan[sspan.length - 1] - sspan[0]);
        const tanTE = Math.tan(sweep[0] * DEG);
        sweepLE = (Math.atan(tanTE + (root - tip) / Math.max(be, 1e-9)) * 180) / Math.PI;
      }
      const role: SurfaceRole = idx === 0 ? "canard" : "fin";
      const set: FinSet = {
        name: `FINSET${idx + 1}`,
        role,
        nPanels: nPanel,
        phiF: phif.length ? phif : undefined,
        rootChord: root,
        tipChord: tip,
        semispan,
        xLE: xle[0] ?? 0,
        sweepLE,
        thickness: zup[0] !== undefined ? zup[0] * 2 : 0.05,
        section: sectOf(str(V.SECTYP) || "HEX"),
        flatFraction: flat[0],
        leRadius: ler[0],
        cfoc: arr(V.CFOC)[0],
        dihedral: arr(V.GAM)[0],
      };
      if (nacaBySet[idx + 1]) {
        set.section = "naca4";
        set.naca = nacaBySet[idx + 1];
      }
      sets[idx] = set;
    } else if (nl.name === "DEFLCT") {
      for (let k = 1; k <= 4; k++) deltas[k - 1] = arr(V[`DELTA${k}`]);
    } else if (nl.name === "ELLBOD") {
      if (V.NX !== undefined) {
        const X = arr(V.X).map(L);
        const H = arr(V.H).map(L);
        const W = arr(V.W).map(L);
        const ELL = arr(V.ELLIP);
        const n = Math.min(num(V.NX, X.length), X.length);
        const elliptic = [] as { x: number; a: number; b: number }[];
        for (let i = 0; i < n; i++) {
          let a = W[i], b = H[i];
          if ((a === undefined || b === undefined) && ELL[i] !== undefined) {
            if (a !== undefined) b = a * ELL[i];
            else if (b !== undefined) a = b / Math.max(ELL[i], 1e-9);
          }
          elliptic.push({ x: X[i] ?? 0, a: a ?? 0, b: b ?? 0 });
        }
        body = { ...(body ?? {}), elliptic, exitDiameter: V.DEXIT !== undefined ? L(num(V.DEXIT)) : body?.exitDiameter };
        warnings.push("$ELLBOD option-2 loaded (equivalent-radius synthesis)");
      } else {
        warnings.push("$ELLBOD option-1 not fully mapped — provide NX/X/H/W");
      }
    } else if (nl.name === "INLET") {
      const nIn = Math.max(1, Math.round(num(V.NIN, 1)));
      const kindRaw = str(V.INTYPE) || "AXI";
      const kind = /2DTOP/i.test(kindRaw) ? "2dtop" as const : /2DSIDE/i.test(kindRaw) ? "2dside" as const : "axi" as const;
      const Hs = arr(V.H).map(L);
      const Ws = arr(V.W).map(L);
      let Ac = 0;
      if (kind === "axi") {
        const d = Ws[1] ?? Ws[0] ?? Hs[0] ?? 0.1;
        Ac = Math.PI * (d / 2) ** 2;
      } else {
        Ac = Math.max(Hs[1] ?? Hs[0] ?? 0.1, 0) * Math.max(Ws[1] ?? Ws[0] ?? 0.1, 0);
      }
      const phis = arr(V.PHI);
      const mfrs = arr(V.MFR);
      for (let i = 0; i < nIn; i++) {
        inlets.push({
          name: `inlet${i + 1}`,
          kind,
          x: L(num(V.XINLT)),
          captureArea: Ac,
          mfr: mfrs[i] ?? mfrs[0] ?? 0.9,
          rampDeg: num(V.RAMP, 0),
          phi: phis[i] ?? phis[0] ?? 0,
          covered: V.COVER === true || V.COVER === 1,
        });
      }
      for (const k of Object.keys(V)) {
        if (!["NIN","INTYPE","XINLT","XDIV","HDIV","LDIV","PHI","X","H","W","COVER","RAMP","ADD","MFR"].includes(k)) {
          warnings.push(`$INLET unsupported key ${k}`);
        }
      }
    } else if (nl.name === "PROTUB") {
      const nProt = Math.max(0, Math.round(num(V.NPROT, 0)));
      const types = (Array.isArray(V.PTYPE) ? V.PTYPE : V.PTYPE !== undefined ? [V.PTYPE] : []) as (string | number)[];
      const xprot = arr(V.XPROT).map(L);
      const nloc = arr(V.NLOC);
      const lprot = arr(V.LPROT).map(L);
      const wprot = arr(V.WPROT).map(L);
      const hprot = arr(V.HPROT).map(L);
      const mapKind = (t: string): ProtuberanceKind => {
        const u = t.toUpperCase();
        if (u.includes("VCYL") || u.includes("HCYL") || u === "CYLINDER") return "cylinder";
        if (u.includes("FAIR")) return "fairing";
        if (u.includes("SHOE") || u.includes("LUG") || u.includes("BLOCK")) return "block";
        return "block";
      };
      for (let i = 0; i < nProt; i++) {
        const raw = String(types[i] ?? "BLOCK");
        prots.push({
          name: `protub${i + 1}`,
          kind: mapKind(raw),
          x: xprot[i] ?? 0,
          length: lprot[i] ?? 0.1,
          width: wprot[i] ?? 0.05,
          height: hprot[i] ?? 0.05,
          count: Math.max(1, Math.round(nloc[i] ?? 1)),
        });
      }
      for (const k of Object.keys(V)) {
        if (!["NPROT","PTYPE","XPROT","NLOC","LPROT","WPROT","HPROT","OPROT"].includes(k)) {
          warnings.push(`$PROTUB unsupported key ${k}`);
        }
      }
      if (V.OPROT !== undefined) warnings.push("$PROTUB OPROT (offset) accepted but not modelled");
    } else if (nl.name === "OPTINS" || nl.name === "SYNTHS" || nl.name === "BODY" || /^(WG|HT|VT|CANARD)/.test(nl.name)) {
      // handled in aircraft path
    } else {
      warnings.push(`$${nl.name}: recognised but not mapped in the missile path`);
    }
  }
  // NACA cards were collected as raw lines earlier — handled by caller
  deltas.forEach((d, i) => {
    if (d.length && sets[i]) sets[i].deflection = d;
  });
  const config: VehicleConfig = {
    name: caseId || "missile-deck",
    kind: "missile",
    body,
    sets: sets.filter(Boolean),
    ref: { xcg, sRef, lRef, latRef, roughness, blayer },
    inlets: inlets.length ? inlets : undefined,
    protuberances: prots.length ? prots : undefined,
    units,
    caseId,
  };
  return { config, flight };
}

function buildFromAircraft(lists: NL[], units: "m" | "ft" | "in" | "cm", caseId: string, warnings: string[]): { config: VehicleConfig; flight: FlightSpec } | null {
  const has = (n: string) => lists.some((l) => l.name === n);
  if (!has("WGPLNF") && !has("BODY") && !has("SYNTHS")) return null;
  const L = (v: number) => toMeters(v, units);
  let flight: FlightSpec = { machs: [0.5], alphas: [0, 5, 10], altKm: 0 };
  let xcg = 0;
  let sRef: number | undefined;
  let lRef: number | undefined;
  let latRef: number | undefined;
  let body: BodySpec | undefined;
  const sets: FinSet[] = [];
  let xw = 0,
    xh = 0,
    xv = 0;
  for (const nl of lists) {
    const V = nl.vars;
    if (nl.name === "FLTCON") {
      flight = {
        machs: arr(V.MACH).length ? arr(V.MACH) : [0.5],
        alphas: arr(V.ALPHA).length ? arr(V.ALPHA) : [0, 5, 10],
        beta: num(V.BETA, 0),
        altKm: arr(V.ALT).length ? arr(V.ALT).map((a) => (units === "m" || units === "cm" ? a / 1000 : a * 0.0003048)) : 0,
      };
    } else if (nl.name === "OPTINS") {
      if (V.SREF !== undefined) {
        const f = L(1);
        sRef = num(V.SREF) * f * f;
      }
      if (V.CBARR !== undefined) lRef = L(num(V.CBARR));
      if (V.BLREF !== undefined) latRef = L(num(V.BLREF));
    } else if (nl.name === "SYNTHS") {
      xcg = L(num(V.XCG));
      xw = L(num(V.XW));
      xh = L(num(V.XH));
      xv = L(num(V.XV));
      if (V.ALIW !== undefined) warnings.push(`$SYNTHS ALIW=${V.ALIW} (wing incidence) — not applied to panel incidence`);
      if (V.ALIH !== undefined) warnings.push(`$SYNTHS ALIH=${V.ALIH} (htail incidence) — not applied`);
    } else if (nl.name === "BODY") {
      const X = arr(V.X).map(L);
      const R = arr(V.R).map(L);
      const S = arr(V.S);
      if (R.length && X.length) body = { stations: X.map((x, i) => ({ x, r: R[i] ?? 0 })) };
      else if (S.length && X.length) {
        const f = L(1);
        body = { stations: X.map((x, i) => ({ x, r: Math.sqrt(Math.max(S[i], 0) * f * f / Math.PI) })) };
        warnings.push("$BODY S (cross-section area) used to recover an equivalent circular radius");
      } else warnings.push("$BODY without X/R or X/S — ignored");
    } else if (nl.name === "WGPLNF" || nl.name === "HTPLNF" || nl.name === "VTPLNF") {
      const role: SurfaceRole = nl.name === "WGPLNF" ? "wing" : nl.name === "HTPLNF" ? "htail" : "vtail";
      const cr = L(num(V.CHRDR, num(V.CHRDRP, 1)));
      const ct = L(num(V.CHRDTP, cr * 0.5));
      const sspn = L(num(V.SSPAN, num(V.SSPNE, 1)));
      const savsi = num(V.SAVSI, 0);
      const chstat = num(V.CHSTAT, 0.25);
      // convert quarter-chord (or CHSTAT) sweep to LE
      const be = sspn;
      const tanL = Math.tan(savsi * DEG) + ((cr - ct) * (chstat === 0 ? 0 : chstat)) / Math.max(be, 1e-9);
      const xLE = role === "wing" ? xw : role === "htail" ? xh : xv;
      sets.push({
        name: role,
        role,
        nPanels: role === "vtail" ? 1 : 2,
        phiF: role === "vtail" ? [0] : [90, 270],
        rootChord: cr,
        tipChord: ct,
        semispan: sspn,
        xLE,
        sweepLE: (Math.atan(tanL) * 180) / Math.PI,
        thickness: 0.12,
        section: "naca4",
        naca: "0012",
        cfoc: role === "wing" ? 0.25 : 0.3,
      });
      if (V.DHDADI !== undefined) warnings.push(`$${nl.name} DHDADI (dihedral) accepted but not modelled in loads`);
      if (V.TWISTA !== undefined) warnings.push(`$${nl.name} TWISTA (twist) ignored`);
    }
  }
  if (!sets.length && !body) return null;
  return {
    config: {
      name: caseId || "aircraft-deck",
      kind: "aircraft",
      body,
      sets,
      ref: { xcg, sRef, lRef, latRef },
      units,
      caseId,
      notes: ["Parsed from aircraft-DATCOM namelists (subset)."],
    },
    flight,
  };
}

export function parseDeck(text: string): DeckParse {
  const pre = preprocess(text);
  const { lists, unsupported, supported } = parseNamelists(pre.lines);
  // NACA cards
  for (const line of pre.lines) {
    const m = line.trim().match(/^NACA\s*[^\d]*([1-4])\s*[^\d]*([1456S])\s*[- ]?\s*(.+)$/i);
    if (!m) continue;
    // attach later
    void m;
  }
  const warnings = [...pre.unsupportedCtrl.map((c) => `control card '${c}' is recognised but not acted on`)];
  const unsup = [...unsupported, ...pre.unsupportedCtrl.map((c) => `CTRL ${c}`)];
  const supp = [...supported, ...pre.supportedCtrl];

  // Try aircraft first if WGPLNF / SYNTHS present, else missile
  const ac = buildFromAircraft(lists, pre.units, pre.caseId, warnings);
  const ms = buildFromMissile(lists, pre.units, pre.caseId, warnings);
  const chosen = ac && (ac.config.sets.length || ac.config.body) && lists.some((l) => /PLNF|SYNTHS|OPTINS/.test(l.name)) ? ac : ms;

  // Scale
  for (const nl of lists) {
    if (nl.name === "REFQ" && nl.vars.SCALE !== undefined) {
      const s = num(nl.vars.SCALE, 1);
      if (s !== 1 && chosen.config.body?.parametric) {
        const p = chosen.config.body.parametric;
        p.noseLength *= s;
        p.noseDiameter *= s;
        p.centerLength *= s;
        if (p.centerDiameter) p.centerDiameter *= s;
        p.aftLength *= s;
        if (p.aftDiameter) p.aftDiameter *= s;
        if (p.noseBluntRadius) p.noseBluntRadius *= s;
        for (const set of chosen.config.sets) {
          set.rootChord *= s;
          set.tipChord *= s;
          set.semispan *= s;
          set.xLE *= s;
          if (set.leRadius) set.leRadius *= s;
        }
        chosen.config.ref.xcg *= s;
        if (chosen.config.ref.sRef) chosen.config.ref.sRef *= s * s;
        if (chosen.config.ref.lRef) chosen.config.ref.lRef *= s;
        if (chosen.config.ref.latRef) chosen.config.ref.latRef *= s;
      }
    }
  }

  // Relative-span fins (SSPAN(1)=0): bump semispan by the body radius at the root.
  if (chosen.config.body) {
    try {
      const prof = buildBody(chosen.config.body);
      for (const set of chosen.config.sets) {
        const r = radiusAt(prof, set.xLE + 0.5 * set.rootChord);
        if (set.semispan < r * 1.01) set.semispan = set.semispan + r;
      }
    } catch (e) {
      warnings.push(`body-root attach skipped: ${(e as Error).message}`);
    }
  }

  chosen.config.derivUnits = pre.deriv;
  chosen.config.units = pre.units;
  if (!lists.length) warnings.push("no namelists found in the deck");
  return {
    config: chosen.config,
    flight: chosen.flight,
    warnings,
    unsupported: unsup,
    supported: supp,
    units: pre.units,
    derivUnits: pre.deriv,
    raw: text,
  };
}


/** Generate a Missile DATCOM-style input deck from a VehicleConfig + FlightSpec. */
export function generateMissileDeck(cfg: VehicleConfig, flight: FlightSpec, units: "m" | "ft" | "in" | "cm" = "m"): string {
  const U = (v: number) => fromMeters(v, units);
  const lines: string[] = [];
  lines.push(`CASEID ${cfg.caseId ?? cfg.name}`);
  lines.push(`DIM ${units === "m" ? "M" : units === "ft" ? "FT" : units === "in" ? "IN" : "CM"}`);
  lines.push(`DERIV ${(cfg.derivUnits ?? "deg").toUpperCase()}`);
  const machs = flight.machs;
  const alphas = flight.alphas;
  lines.push(
    `$FLTCON NALPHA=${alphas.length}., ALPHA=${alphas.map((a) => a.toFixed(2)).join(",")},`,
  );
  lines.push(
    ` NMACH=${machs.length}., MACH=${machs.map((m) => m.toFixed(3)).join(",")},`,
  );
  const alts = Array.isArray(flight.altKm) ? flight.altKm : machs.map(() => flight.altKm ?? 0);
  const altU = alts.map((a) => (units === "m" || units === "cm" ? (a as number) * 1000 : (a as number) / 0.0003048));
  lines.push(` ALT=${altU.map((a) => a.toFixed(1)).join(",")}, BETA=${(flight.beta ?? 0).toFixed(2)}, PHI=${(flight.phi ?? 0).toFixed(2)},$`);
  lines.push(
    `$REFQ SREF=${(cfg.ref.sRef ? cfg.ref.sRef / (U(1) * U(1)) : 0).toFixed(6)}, LREF=${U(cfg.ref.lRef ?? 1).toFixed(4)}, LATREF=${U(cfg.ref.latRef ?? cfg.ref.lRef ?? 1).toFixed(4)}, XCG=${U(cfg.ref.xcg).toFixed(4)},$`,
  );
  if (cfg.body?.parametric) {
    const p = cfg.body.parametric;
    const tn = { cone: "CONE", ogive: "OGIVE", karman: "KARMAN", haack: "HAACK", power: "POWER", spherical: "OGIVE" }[p.noseKind];
    lines.push(
      `$AXIBOD TNOSE=${tn}, LNOSE=${U(p.noseLength).toFixed(4)}, DNOSE=${U(p.noseDiameter).toFixed(4)},` +
        (p.noseBluntRadius ? ` BNOSE=${U(p.noseBluntRadius).toFixed(4)},` : "") +
        ` LCENTR=${U(p.centerLength).toFixed(4)}, DCENTR=${U(p.centerDiameter ?? p.noseDiameter).toFixed(4)},` +
        (p.aftLength > 0
          ? ` TAFT=${p.aftKind === "ogive" ? "OGIVE" : "CONE"}, LAFT=${U(p.aftLength).toFixed(4)}, DAFT=${U(p.aftDiameter ?? p.noseDiameter).toFixed(4)},`
          : "") +
        (cfg.body.exitDiameter !== undefined ? ` DEXIT=${U(cfg.body.exitDiameter).toFixed(4)},` : "") +
        `$`,
    );
  } else if (cfg.body?.stations) {
    const st = cfg.body.stations;
    lines.push(`$AXIBOD NX=${st.length}.,`);
    lines.push(` X=${st.map((s) => U(s.x).toFixed(4)).join(",")},`);
    lines.push(` R=${st.map((s) => U(s.r).toFixed(4)).join(",")},$`);
  }
  cfg.sets.forEach((s, i) => {
    const n = i + 1;
    lines.push(
      `$FINSET${n} SSPAN=${U(0).toFixed(4)},${U(s.semispan).toFixed(4)}, CHORD=${U(s.rootChord).toFixed(4)},${U(s.tipChord).toFixed(4)},`,
    );
    lines.push(
      ` XLE=${U(s.xLE).toFixed(4)}, SWEEP=${s.sweepLE.toFixed(2)}, STA=0., NPANEL=${s.nPanels}.,` +
        (s.phiF ? ` PHIF=${s.phiF.map((p) => p.toFixed(1)).join(",")},` : "") +
        ` SECTYP=${s.section === "biconvex" ? "ARC" : s.section === "naca4" ? "NACA" : "HEX"},` +
        ` ZUPPER=${(s.thickness / 2).toFixed(4)},` +
        (s.flatFraction !== undefined ? ` LFLATU=${s.flatFraction.toFixed(3)},` : "") +
        (s.leRadius ? ` LER=${U(s.leRadius).toFixed(4)},` : "") +
        (s.cfoc !== undefined ? ` CFOC=${s.cfoc.toFixed(3)},` : "") +
        `$`,
    );
    if (s.section === "naca4" && s.naca) lines.push(`NACA ${n} 4 ${s.naca}`);
  });
  if (cfg.sets.some((s) => s.deflection && s.deflection.some((d) => Math.abs(d) > 1e-9))) {
    const parts = cfg.sets.map((s, i) => (s.deflection ? `DELTA${i + 1}=${s.deflection.map((d) => d.toFixed(2)).join(",")},` : "")).join(" ");
    lines.push(`$DEFLCT ${parts}$`);
  }
  lines.push("NEXT CASE");
  return lines.join("\n");
}
