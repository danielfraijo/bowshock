import { exactConeCp, exactRampAft, exactWedgePlate, buildBench, runBench, newtonianCone, type BenchCase } from "../src/lib/waverider/shapebench";
import { newtonianCpMax } from "../src/lib/waverider/math";
import type { AeroMethod } from "../src/lib/waverider/types";

const methods: AeroMethod[] = ["newtonian", "cbaero", "tangent", "mixed"];
const pct = (a: number, b: number) => (Number.isFinite(a) && Number.isFinite(b) && Math.abs(b) > 1e-9 ? (100 * (a - b)) / b : NaN);
const f = (x: number) => (Number.isFinite(x) ? x.toFixed(1).padStart(7) : "    n/a");
const withVisc = (cp: number, pv: number, M: number) => ((1 + cp * 0.7 * M * M) * pv - 1) / (0.7 * M * M);
const only = process.argv[2] ?? "all";
console.log("method order:", methods.join(" "));
if (only === "all" || only === "ramp") {
  for (const M of [4, 8, 12]) for (const [t1, t2] of [[5, 10], [8, 15], [10, 20], [15, 5], [10, 0], [12, 25]]) {
    const c: BenchCase = { shape: "ramp", M, thetaDeg: t1, theta2Deg: t2, alphaDeg: 0, altKm: 0 };
    const ex = exactRampAft(M, t1, t2);
    if (!Number.isFinite(ex)) { console.log(`ramp M${M} ${t1}->${t2} detached`); continue; }
    const body = buildBench(c);
    const r = methods.map((m) => runBench(c, m, body));
    console.log(`ramp M${String(M).padStart(2)} ${String(t1).padStart(2)}->${String(t2).padStart(2)} exact aft ${ex.toFixed(4)} | aft%: ${r.map((x) => f(pct(x.cpAft, withVisc(ex, x.aero.pVisc, M)))).join("")}`);
  }
}
if (only === "all" || only === "cone") {
  for (const M of [3, 5, 8, 12, 20]) for (const th of [5, 10, 15, 25]) {
    const c: BenchCase = { shape: "cone", M, thetaDeg: th, alphaDeg: 0, altKm: 0 };
    const ex = exactConeCp(M, th);
    const body = buildBench(c);
    const r = methods.map((m) => runBench(c, m, body));
    console.log(`cone M${String(M).padStart(2)} ${String(th).padStart(2)}° exact ${ex.toFixed(4)} | side% ${r.map((x) => f(pct(x.cpSide, withVisc(ex, x.aero.pVisc, M)))).join("")}`);
  }
}
if (only === "all" || only === "sphere") {
  for (const M of [3, 5, 8, 12, 20]) {
    const c: BenchCase = { shape: "sphere", M, thetaDeg: 0, alphaDeg: 0, altKm: 30 };
    const body = buildBench(c);
    const cpm = newtonianCpMax(M);
    const r = methods.map((m) => runBench(c, m, body));
    console.log(`sphere M${M} CD(S=πR²) newtonian ${(cpm / 2).toFixed(4)} | ${r.map((x) => x.aero.cd.toFixed(4)).join(" ")} | cdWave ${r.map((x) => x.aero.cdWave.toFixed(4)).join(" ")}`);
  }
}
