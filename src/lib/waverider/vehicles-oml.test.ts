/**
 * Class-vehicle outer mold lines: sphere-cone, blunted tangent ogive,
 * staged cylinders, and plate fins. Run:
 * node --import ./scripts/ts-register.mjs src/lib/waverider/vehicles-oml.test.ts
 */
import assert from "node:assert/strict";
import { atmosphere } from "./atmosphere";
import { topoMetrics } from "./geom-quality";
import { buildVehicle } from "./generate";
import { analyzeMesh } from "./mesh";
import { DEG } from "./math";
import { panelAero } from "./panel";
import { DEFAULT_PARAMS, type DesignParams, type WaveriderFamily } from "./types";
import { CLASS_SHAPE, revolutionRadius } from "./vehicles";

const p = (family: WaveriderFamily, patch: Partial<DesignParams> = {}): DesignParams => ({
  ...DEFAULT_PARAMS,
  ...(CLASS_SHAPE[family] ?? {}),
  family,
  aeroMethod: "newtonian",
  ...patch,
});

function medianRadius(built: ReturnType<typeof buildVehicle>, x: number, tol: number) {
  const pos = built.mesh.positions;
  const rs: number[] = [];
  for (let i = 0; i < pos.length; i += 3) {
    if (Math.abs(pos[i] - x) <= tol) rs.push(Math.hypot(pos[i + 1], pos[i + 2]));
  }
  if (!rs.length) return null;
  rs.sort((a, b) => a - b);
  return rs[Math.floor(rs.length / 2)];
}

function assertClosed(name: string, built: ReturnType<typeof buildVehicle>) {
  const q = analyzeMesh(built.mesh);
  const tm = topoMetrics(built.mesh);
  assert.equal(q.watertight, true, `${name} open ${q.openEdges}`);
  assert.equal(q.manifold, true, `${name} nonmanifold ${q.nonManifoldEdges}`);
  assert.ok(q.volume > 1e-8, `${name} volume ${q.volume}`);
  assert.ok(tm.minAngleDeg > 0.005, `${name} min angle ${tm.minAngleDeg.toFixed(3)}°`);
  const pos = built.mesh.positions;
  for (let i = 0; i < pos.length; i++) assert.ok(Number.isFinite(pos[i]), `${name} non-finite vertex`);
}

function kink(r: (x: number) => number, x: number, dx: number) {
  const sl = (r(x) - r(x - dx)) / dx;
  const sr = (r(x + dx) - r(x)) / dx;
  return Math.abs(sr - sl);
}

{
  const d = p("capsule");
  const r = revolutionRadius(d, 0);
  assert.ok(r === 0);
  const Rn = d.leRadius;
  const theta = d.coneDeg * DEG;
  const Rb = d.span / 2;
  const xt = Rn * (1 - Math.sin(theta));
  const rt = Rn * Math.cos(theta);
  const at = revolutionRadius(d, xt)!;
  assert.ok(Math.abs(at - rt) < 1e-6, `tangent radius ${at} vs ${rt}`);
  const dx = 1e-4;
  const slopeS = (revolutionRadius(d, xt)! - revolutionRadius(d, xt - dx)!) / dx;
  const slopeC = (revolutionRadius(d, xt + dx)! - revolutionRadius(d, xt)!) / dx;
  assert.ok(Math.abs(slopeS - slopeC) < 0.02, `sphere-cone kink ${slopeS} vs ${slopeC}, tan ${Math.tan(theta)}`);
  assert.ok(Math.abs(slopeC - Math.tan(theta)) < 0.02, `cone slope ${slopeC}`);
  let prev = 0;
  for (let i = 1; i <= 40; i++) {
    const ri = revolutionRadius(d, (d.length * i) / 40)!;
    assert.ok(ri + 1e-6 >= prev, `capsule radius fell ${prev} → ${ri}`);
    prev = ri;
  }
  const built = buildVehicle(d);
  assertClosed("capsule", built);
  const tol = d.length / Math.max(20, d.nx);
  const meshR = medianRadius(built, xt, tol);
  assert.ok(meshR != null && Math.abs(meshR - rt) / rt < 0.08, `mesh at tangency ${meshR}`);
  const rBase = revolutionRadius(d, d.length)!;
  let vol = 0;
  const n = 200;
  for (let i = 0; i < n; i++) {
    const x0 = (d.length * i) / n;
    const x1 = (d.length * (i + 1)) / n;
    const rm = revolutionRadius(d, 0.5 * (x0 + x1))!;
    vol += Math.PI * rm * rm * (x1 - x0);
  }
  const q = analyzeMesh(built.mesh);
  assert.ok(Math.abs(q.volume - vol) / vol < 0.04, `capsule volume ${q.volume} vs ${vol}`);
  assert.ok(Math.abs(rBase - Rb) / Rb < 0.02, `default capsule should reach the base, ${rBase} vs ${Rb}`);
  console.log(`capsule V err ${((100 * (q.volume - vol)) / vol).toFixed(2)}% base ${rBase.toFixed(3)} min∠ ${topoMetrics(built.mesh).minAngleDeg.toFixed(2)}°`);
}

{
  // Short length must truncate the cone, not flare past it.
  const d = p("capsule", { length: 1.4, span: 3.9, coneDeg: 25, leRadius: 0.6 });
  const Rb = d.span / 2;
  const rBase = revolutionRadius(d, d.length)!;
  assert.ok(rBase < Rb * 0.85, `truncated base ${rBase} should be under ${Rb}`);
  let maxOver = 0;
  const theta = d.coneDeg * DEG;
  const Rn = d.leRadius;
  const xt = Rn * (1 - Math.sin(theta));
  const rt = Rn * Math.cos(theta);
  for (let i = 0; i <= 30; i++) {
    const x = (d.length * i) / 30;
    const cone = x <= xt ? rt : rt + (x - xt) * Math.tan(theta);
    const ri = revolutionRadius(d, x)!;
    maxOver = Math.max(maxOver, ri - cone - 1e-6);
  }
  assert.ok(maxOver <= 1e-4, `flare above the cone ${maxOver}`);
  assertClosed("capsule-short", buildVehicle(d));
  console.log(`capsule truncated base ${rBase.toFixed(3)} < ${Rb.toFixed(3)}`);
}

{
  const d = p("missile", { leRadius: 0, leBlunt: false, finHeight: 0 });
  const L = d.length;
  const Rb = d.span / 2;
  const Ln = Math.min(0.6, Math.max(0.12, d.wedgeFrac)) * L;
  const rho = (Rb * Rb + Ln * Ln) / (2 * Rb);
  const sharp = (x: number) => Math.max(0, Math.sqrt(Math.max(0, rho * rho - (Ln - x) ** 2)) - (rho - Rb));
  assert.ok(Math.abs(revolutionRadius(d, 0)!) < 1e-8, "sharp ogive starts at a point");
  assert.ok(Math.abs(revolutionRadius(d, Ln)! - Rb) < 1e-6, "ogive meets the cylinder");
  const slope = (revolutionRadius(d, Ln)! - revolutionRadius(d, Ln - 1e-4)!) / 1e-4;
  assert.ok(Math.abs(slope) < 2e-3, `shoulder slope ${slope}`);
  for (let i = 0; i <= 20; i++) {
    const x = (Ln * i) / 20;
    assert.ok(Math.abs(revolutionRadius(d, x)! - sharp(x)) < 1e-6, `ogive ${x}`);
  }
  const built = buildVehicle({ ...d, nx: 48, ny: 24 });
  assertClosed("missile-sharp", built);
  console.log("sharp ogive matches the tangent-ogive identity");
}

{
  const d = p("missile", { finHeight: 0 });
  const r = (x: number) => revolutionRadius(d, x)!;
  assert.ok(r(0) < 1e-6, `blunt nose ${r(0)}`);
  const Ln = d.wedgeFrac * d.length;
  const dx = Ln / 400;
  let worst = 0;
  for (let x = dx * 4; x < Ln - dx * 4; x += dx * 2) worst = Math.max(worst, kink(r, x, dx));
  assert.ok(worst < 0.05, `ogive kink ${worst}`);
  assert.ok(Math.abs(r(Ln) - d.span / 2) < 1e-3, `cylinder ${r(Ln)}`);
  const rBase = r(d.length);
  assert.ok(Math.abs(rBase - (d.span / 2) * d.captureFrac) < 1e-3, `boat-tail ${rBase}`);
  let prev = 0;
  for (let i = 1; i <= 30; i++) {
    const x = (Ln * i) / 30;
    assert.ok(r(x) + 1e-6 >= prev, "ogive not monotone");
    prev = r(x);
  }
  assertClosed("missile-blunt", buildVehicle(d));
  console.log(`blunt ogive kink ${worst.toExponential(1)} boat-tail ${rBase.toFixed(3)}`);
}

{
  const d = p("missile", { finHeight: 0.85, fins: 4, nRamps: 1, elevonDeg: 0 });
  const built = buildVehicle(d);
  assertClosed("missile-fins", built);
  const pos = built.mesh.positions;
  let maxR = 0;
  for (let i = 0; i < pos.length; i += 3) maxR = Math.max(maxR, Math.hypot(pos[i + 1], pos[i + 2]));
  const Rb = d.span / 2;
  const h = d.finHeight * Rb;
  assert.ok(maxR > Rb + 0.55 * h, `fin did not stand off the body, maxR ${maxR} Rb ${Rb} h ${h}`);
  // Thickness: among vertices near the max radius, the spread normal to the fin should be small.
  const tips: { y: number; z: number }[] = [];
  for (let i = 0; i < pos.length; i += 3) {
    const rad = Math.hypot(pos[i + 1], pos[i + 2]);
    if (rad > Rb + 0.7 * h) tips.push({ y: pos[i + 1], z: pos[i + 2] });
  }
  assert.ok(tips.length > 4, "no fin tip vertices");
  let thick = 0;
  for (const t of tips) {
    const horiz = Math.abs(t.y) > Math.abs(t.z);
    thick = Math.max(thick, horiz ? Math.abs(t.z) : Math.abs(t.y));
  }
  assert.ok(thick < 0.35 * h, `fin is a lump, half-thickness ${thick} vs height ${h}`);
  const up = buildVehicle({ ...d, elevonDeg: 8 });
  const dn = buildVehicle({ ...d, elevonDeg: -8 });
  const teZ = (b: ReturnType<typeof buildVehicle>) => {
    let bestX = -1;
    let z = 0;
    const pp = b.mesh.positions;
    for (let i = 0; i < pp.length; i += 3) {
      if (Math.abs(pp[i + 1]) > Rb + 0.45 * h && pp[i] > bestX) {
        bestX = pp[i];
        z = pp[i + 2];
      }
    }
    return z;
  };
  assert.ok(teZ(up) < teZ(dn) - 1e-4, `positive elevon should drop the trailing edge ${teZ(up)} vs ${teZ(dn)}`);
  const leZ = (b: ReturnType<typeof buildVehicle>) => {
    let bestX = 1e9;
    let z = 0;
    const pp = b.mesh.positions;
    for (let i = 0; i < pp.length; i += 3) {
      if (Math.abs(pp[i + 1]) > Rb + 0.45 * h && pp[i] < bestX) {
        bestX = pp[i];
        z = pp[i + 2];
      }
    }
    return z;
  };
  assert.ok(Math.abs(leZ(up) - leZ(dn)) < 1e-3, `elevon must not swing the fin leading edge ${leZ(up)} vs ${leZ(dn)}`);
  const atm = atmosphere(d.altKm, d.mach, d.gamma);
  const S = Math.PI * Rb * Rb;
  const a0 = panelAero(built.mesh, d, atm, 0, 0, S, d.length, undefined, true);
  const aE = panelAero(up.mesh, { ...d, elevonDeg: 8 }, atm, 0, 0, S, d.length, undefined, true);
  assert.ok(Math.abs(a0.cl) < 0.03 && Math.abs(a0.cy) < 0.03, `fins not symmetric CL ${a0.cl} CY ${a0.cy}`);
  assert.ok(Math.abs(aE.cm - a0.cm) > 1e-4, `elevon produced no moment ${aE.cm} vs ${a0.cm}`);
  console.log(`fins maxR ${maxR.toFixed(3)} thick ${thick.toFixed(3)} ΔCm ${(aE.cm - a0.cm).toFixed(4)} min∠ ${topoMetrics(built.mesh).minAngleDeg.toFixed(2)}°`);
}

{
  const d = p("multistage", { nRamps: 3, finHeight: 0.4, fins: 4 });
  const r = (x: number) => revolutionRadius(d, x)!;
  assert.ok(r(0) < 1e-6, "stack nose is a point");
  // Cylindrical runs: radius constant over a stretch.
  const xs: number[] = [];
  for (let i = 0; i <= 80; i++) xs.push((d.length * i) / 80);
  let flats = 0;
  let run = 0;
  for (let i = 1; i < xs.length; i++) {
    if (Math.abs(r(xs[i]) - r(xs[i - 1])) < 1e-4 * d.span) run++;
    else {
      if (run >= 3) flats++;
      run = 0;
    }
  }
  if (run >= 3) flats++;
  assert.ok(flats >= 3, `expected a cylinder per stage, found ${flats}`);
  const built = buildVehicle(d);
  assertClosed("stack", built);
  const pos = built.mesh.positions;
  let maxR = 0;
  for (let i = 0; i < pos.length; i += 3) maxR = Math.max(maxR, Math.hypot(pos[i + 1], pos[i + 2]));
  assert.ok(maxR > d.span / 2 + 0.2 * d.finHeight * (d.span / 2), `stack fins missing ${maxR}`);
  console.log(`stack cylinders ${flats} maxR ${maxR.toFixed(3)}`);
}

{
  const edges: Partial<DesignParams>[] = [
    { family: "capsule", coneDeg: 5, length: 2, span: 4, leRadius: 1.2, nx: 16, ny: 12 },
    { family: "capsule", coneDeg: 55, length: 8, span: 1.2, leRadius: 0.01, nx: 64, ny: 40 },
    { family: "capsule", coneDeg: 20, length: 0.8, span: 3, leRadius: 0.9, nx: 20, ny: 14 },
    { family: "missile", span: 0.2, length: 12, finHeight: 1.4, fins: 8, nRamps: 2, elevonDeg: 12, wedgeFrac: 0.18, captureFrac: 0.4, leRadius: 0.02, nx: 36, ny: 16 },
    { family: "missile", span: 1.2, length: 2.2, finHeight: 0, fins: 2, wedgeFrac: 0.55, captureFrac: 1, leRadius: 0, leBlunt: false, nx: 18, ny: 12 },
    { family: "missile", finHeight: 0.9, fins: 3, elevonDeg: -12, nRamps: 1, ny: 10, nx: 16, leRadius: 0.04 },
    { family: "missile", finHeight: 0.2, fins: 6, nRamps: 2, captureFrac: 0.35, wedgeFrac: 0.12, span: 0.5, length: 4 },
    { family: "multistage", nRamps: 4, length: 22, span: 2.4, finHeight: 0.6, fins: 4, nx: 40, ny: 18 },
    { family: "multistage", nRamps: 2, finHeight: 0, length: 3, span: 0.3, nx: 16, ny: 12 },
    { family: "boostglide", planform: "power", planformPower: 2.2, nx: 28, ny: 16 },
    { family: "boostglide", planform: "rect", leBlunt: false, leRadius: 0, nx: 20, ny: 12 },
  ];
  for (const e of edges) {
    const d = p(e.family ?? "missile", e);
    const built = buildVehicle(d);
    assertClosed(`${d.family} edge`, built);
    if (d.family === "capsule" || d.family === "missile" || d.family === "multistage") {
      const body = revolutionRadius(d, d.length * 0.5);
      assert.ok(body != null && body >= 0, "profile");
    }
  }
  console.log(`edge cases ${edges.length} closed`);
}

{
  let s = 20260322;
  const rnd = () => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return s / 4294967296;
  };
  const families: WaveriderFamily[] = ["capsule", "missile", "multistage", "boostglide"];
  for (let k = 0; k < 24; k++) {
    const family = families[k % families.length];
    const d = p(family, {
      length: 0.6 + rnd() * 16,
      span: 0.2 + rnd() * 4,
      height: 0.08 + rnd() * 1.5,
      coneDeg: 6 + rnd() * 46,
      leRadius: rnd() * 0.4,
      leBlunt: rnd() > 0.2,
      wedgeFrac: 0.12 + rnd() * 0.45,
      captureFrac: 0.4 + rnd() * 0.6,
      finHeight: rnd() < 0.25 ? 0 : rnd() * 1.3,
      fins: 2 + Math.floor(rnd() * 7),
      nRamps: 1 + Math.floor(rnd() * 4),
      elevonDeg: (rnd() - 0.5) * 24,
      nx: 14 + Math.floor(rnd() * 40),
      ny: 10 + Math.floor(rnd() * 24),
      planform: (["delta", "power", "spatular", "rect", "double"] as const)[Math.floor(rnd() * 5)],
      planformPower: 0.5 + rnd() * 1.8,
    });
    const built = buildVehicle(d);
    assertClosed(`fuzz ${k} ${family}`, built);
  }
  console.log("fuzz 24 closed");
}

{
  // A long chord used to rotate the whole plate, so the leading edge ahead of
  // the hinge left the body. The flap must stay inside the fin envelope.
  const d = p("missile", { length: 12, span: 0.2, finHeight: 0.3, elevonDeg: 18, fins: 4, nRamps: 1 });
  const built = buildVehicle(d);
  assertClosed("slender elevon", built);
  const Rb = d.span / 2;
  const h = d.finHeight * Rb;
  const pos = built.mesh.positions;
  let maxR = 0;
  let leZ = 0;
  let leX = 1e9;
  let teZ = 0;
  let teX = -1;
  for (let i = 0; i < pos.length; i += 3) {
    const r = Math.hypot(pos[i + 1], pos[i + 2]);
    maxR = Math.max(maxR, r);
    if (Math.abs(pos[i + 1]) < Rb + 0.4 * h) continue;
    if (pos[i] < leX) {
      leX = pos[i];
      leZ = pos[i + 2];
    }
    if (pos[i] > teX) {
      teX = pos[i];
      teZ = pos[i + 2];
    }
  }
  assert.ok(Math.abs(leZ) < 0.04, `leading edge flung, z ${leZ} at x ${leX}`);
  assert.ok(teZ < -0.01, `trailing edge did not drop, z ${teZ}`);
  assert.ok(leX < teX - 0.2, `fin chord collapsed ${leX} → ${teX}`);
  let fwd = 0;
  for (let i = 0; i < pos.length; i += 3) {
    if (Math.abs(pos[i + 1]) < Rb + 0.4 * h) continue;
    if (pos[i] > (leX + teX) * 0.5) continue;
    fwd = Math.max(fwd, Math.hypot(pos[i + 1], pos[i + 2]));
  }
  assert.ok(fwd < Rb + h + 0.025, `fin ahead of the hinge left the body, r ${fwd}`);
  // Straight tip on a hard boat-tail: full-height stations share one radius.
  const boat = p("missile", { captureFrac: 0.45, finHeight: 0.7, fins: 4, elevonDeg: 0, nRamps: 1 });
  const bb = buildVehicle(boat);
  assertClosed("boat-tail fin", bb);
  const br = boat.span / 2;
  const bh = boat.finHeight * br;
  const tips: number[] = [];
  const bp = bb.mesh.positions;
  for (let i = 0; i < bp.length; i += 3) {
    const r = Math.hypot(bp[i + 1], bp[i + 2]);
    if (r > br + 0.82 * bh) tips.push(r);
  }
  assert.ok(tips.length > 8, "boat-tail fin has no straight tip");
  const mean = tips.reduce((s, v) => s + v, 0) / tips.length;
  const spread = Math.max(...tips) - Math.min(...tips);
  assert.ok(spread < 0.08 * bh, `fin tip kinked by the boat-tail, spread ${spread} mean ${mean}`);
  // Capsule flaps are real plates, and they are absent until the height is set.
  const bare = buildVehicle(p("capsule"));
  let bareR = 0;
  const bpos = bare.mesh.positions;
  for (let i = 0; i < bpos.length; i += 3) bareR = Math.max(bareR, Math.hypot(bpos[i + 1], bpos[i + 2]));
  const flapped = buildVehicle(p("capsule", { finHeight: 0.35, fins: 2, elevonDeg: 6 }));
  assertClosed("capsule flaps", flapped);
  let flapR = 0;
  const fp = flapped.mesh.positions;
  for (let i = 0; i < fp.length; i += 3) flapR = Math.max(flapR, Math.hypot(fp[i + 1], fp[i + 2]));
  assert.ok(Math.abs(bareR - p("capsule").span / 2) < 0.05, `bare capsule base ${bareR}`);
  assert.ok(flapR > bareR + 0.2 * 0.35 * bareR, `capsule flaps missing ${flapR} vs ${bareR}`);
  console.log(`slender maxR ${maxR.toFixed(3)} leZ ${leZ.toFixed(4)} teZ ${teZ.toFixed(3)} tip spread ${spread.toExponential(1)} flaps ${flapR.toFixed(3)}`);
}

console.log("vehicles-oml.test ok");
