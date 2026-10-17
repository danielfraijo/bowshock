/**
 * NaN / Inf / exception sweep across every family × method × Mach × alpha × altitude, plus geometry extremes.
 * Run: node --import ./scripts/ts-register.mjs scripts/robustness.ts [quick]
 */
import { atmosphere } from "../src/lib/waverider/atmosphere";
import { buildVehicle } from "../src/lib/waverider/generate";
import { panelAero } from "../src/lib/waverider/panel";
import { DEFAULT_PARAMS, type AeroMethod, type DesignParams, type WaveriderFamily } from "../src/lib/waverider/types";

const quick = process.argv[2] === "quick";
const families: WaveriderFamily[] = ["caret", "cone", "osculating", "viscopt", "inward", "elliptic", "busemann", "star", "wedgecone", "liftbody", "ramjet", "scramjet", "integrated"];
const methods: AeroMethod[] = ["newtonian", "tangent", "mixed", "cbaero", "euler"];
const scalars = ["cl", "cd", "cy", "cm", "cn", "cll", "ld", "ca", "cnA", "cdWave", "cdFric", "cdBase", "cpMax", "qStag", "qMax", "qMeanWind", "qRad", "twMax", "qFay", "qSG", "kn", "chiBar", "gammaEq", "pVisc", "xCp", "qMeanLower", "qMeanUpper"] as const;
const arrays = ["cp", "heat", "twEq", "stanton", "machE", "cf", "impact"] as const;

let n = 0;
let bad = 0;
let thrown = 0;
const seen = new Map<string, number>();
const note = (k: string, ctx: string) => {
  seen.set(k, (seen.get(k) ?? 0) + 1);
  if ((seen.get(k) ?? 0) <= Number(process.env.SHOW ?? 2)) console.log(`  ${k}: ${ctx}`);
};
function run(p: DesignParams, ctx: string, alpha = p.alphaDeg, beta = p.betaDeg) {
  n++;
  try {
    const b = buildVehicle(p);
    const a = panelAero(b.mesh, p, atmosphere(p.altKm, p.flightMach, p.gamma), alpha, beta, b.aero.planformArea, p.length);
    for (const k of scalars) {
      const v = (a as unknown as Record<string, number>)[k];
      if (!Number.isFinite(v)) {
        bad++;
        note(`non-finite ${k}`, ctx);
      }
    }
    for (const k of arrays) {
      const arr = (a as unknown as Record<string, Float32Array>)[k];
      let nb = 0;
      for (let i = 0; i < arr.length; i++) if (!Number.isFinite(arr[i])) nb++;
      if (nb) {
        bad++;
        note(`non-finite array ${k}`, `${ctx} (${nb}/${arr.length})`);
      }
    }
    if (a.force.some((x) => !Number.isFinite(x)) || a.moment.some((x) => !Number.isFinite(x))) {
      bad++;
      note("non-finite force/moment", ctx);
    }
    // silent-garbage screens: |Cp| bound by Newtonian-max-ish, |CD| sane
    let cpm = 0;
    for (let i = 0; i < a.cp.length; i++) cpm = Math.max(cpm, Math.abs(a.cp[i]));
    if (cpm > 2.2) note("|Cp| > 2.2", `${ctx} max ${cpm.toFixed(2)}`);
    if (Math.abs(a.cl) > 20 || Math.abs(a.cd) > 20) note("|CL|,|CD| > 20", `${ctx} CL ${a.cl.toFixed(2)} CD ${a.cd.toFixed(2)}`);
  } catch (e) {
    thrown++;
    note("exception", `${ctx}: ${(e as Error).message.slice(0, 100)}`);
  }
}

const base = (o: Partial<DesignParams>): DesignParams => ({ ...DEFAULT_PARAMS, nx: 24, ny: 14, lockFlight: false, ...o });
const Ms = quick ? [1.0, 3, 20] : [1.0, 1.05, 1.3, 3, 8, 20, 35];
const As = quick ? [-30, 0, 90] : [-30, -5, 0, 15, 60, 90];
const Hs = quick ? [0, 100] : [0, 30, 100, 200];
const t0 = performance.now();
for (const family of families)
  for (const aeroMethod of methods)
    for (const flightMach of Ms)
      for (const alphaDeg of As)
        for (const altKm of Hs) run(base({ family, aeroMethod, flightMach, alphaDeg, altKm }), `${family}/${aeroMethod} M${flightMach} α${alphaDeg} h${altKm}`);
console.log(`condition sweep: ${n} runs, ${bad} non-finite, ${thrown} exceptions, ${((performance.now() - t0) / 1000).toFixed(1)} s`);

const n0 = n;
const ext: [string, Partial<DesignParams>][] = [
  ["length 1e-3", { length: 1e-3, span: 5e-4, height: 2e-4, leRadius: 1e-6 }],
  ["length 1e3", { length: 1e3, span: 550, height: 200, leRadius: 5 }],
  ["leRadius 0", { leRadius: 0 }],
  ["leRadius huge", { leRadius: 1 }],
  ["leBlunt off", { leBlunt: false, leRadius: 0 }],
  ["nx,ny = 3", { nx: 3, ny: 3 }],
  ["nx,ny = 2", { nx: 2, ny: 2 }],
  ["nx,ny = 200,100", { nx: 200, ny: 100 }],
  ["shockDeg 1", { shockDeg: 1 }],
  ["shockDeg 45", { shockDeg: 45 }],
  ["shockDeg 70", { shockDeg: 70 }],
  ["span tiny", { span: 1e-3 }],
  ["span huge", { span: 100 }],
  ["height 0", { height: 0 }],
  ["twK 0", { twK: 0 }],
  ["twK 1e4", { twK: 1e4 }],
  ["gamma 1.1", { gamma: 1.1 }],
  ["gamma 1.67", { gamma: 1.67 }],
  ["cgFrac 0 / 1", { cgFrac: 0 }],
  ["cgFrac 1", { cgFrac: 1 }],
  ["beta 30", { betaDeg: 30 }],
  ["alt -1", { altKm: -1 }],
  ["alt 400", { altKm: 400 }],
  ["alt 1000", { altKm: 1000 }],
  ["teSweep 80", { teSweepDeg: 80 }],
  ["dihedral 60", { dihedralDeg: 60 }],
  ["camber 1", { camber: 1 }],
];
for (const family of families)
  for (const aeroMethod of methods)
    for (const [name, o] of ext) run(base({ family, aeroMethod, ...o }), `${family}/${aeroMethod} ${name}`, o.betaDeg !== undefined ? 3 : 3);
console.log(`geometry/edge sweep: ${n - n0} runs; total ${n} runs, ${bad} non-finite, ${thrown} exceptions, ${((performance.now() - t0) / 1000).toFixed(1)} s`);
console.log([...seen.entries()].map(([k, v]) => `${k}: ${v}`).join("\n") || "clean");
