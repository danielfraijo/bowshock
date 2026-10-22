/**
 * Control surfaces: geometry, symmetry, cache safety, and the aero-database axes.
 * Run: bun src/lib/waverider/controls.test.ts
 *  or: node --import ./scripts/ts-register.mjs src/lib/waverider/controls.test.ts
 */
import assert from "node:assert/strict";
import { buildVehicle } from "./generate";
import { DEFAULT_PARAMS, type BuiltVehicle, type DesignParams } from "./types";
import { PRESETS } from "./presets";
import { panelAero } from "./panel";
import { atmosphere } from "./atmosphere";
import { topoMetrics } from "./geom-quality";
import { controlChannels, listControlSurfaces, mixControls, withControls } from "./controls";
import {
  AERODB_POINT_CAP,
  aeroDbControlDerivativesToCsv,
  aeroDbToCsv,
  aeroDbToJson,
  aeroDbToSimulink,
  deflectVehicle,
  defaultAeroDbSpec,
  generateAeroDatabase,
  gridCount,
  type AeroDbSpec,
} from "./aerodb";

const t0 = performance.now();
const P = (o: Partial<DesignParams>): DesignParams => ({ ...DEFAULT_PARAMS, nx: 24, ny: 16, aeroMethod: "cbaero", ...o });
const preset = (id: string): DesignParams => {
  const p = PRESETS.find((x) => x.id === id);
  assert.ok(p, `preset ${id}`);
  return { ...p.params, aeroMethod: "cbaero" };
};

function fnv(buf: ArrayBufferView): string {
  const b = new Uint8Array(buf.buffer, buf.byteOffset, buf.byteLength);
  let h = 0x811c9dc5 >>> 0;
  for (let i = 0; i < b.length; i++) {
    h ^= b[i];
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h.toString(16).padStart(8, "0");
}
const meshHash = (b: BuiltVehicle) => fnv(b.mesh.positions) + fnv(b.mesh.indices) + fnv(b.mesh.surfaces);

function aero(b: BuiltVehicle, alpha = 4, beta = 0) {
  const p = b.params;
  const atm = atmosphere(p.altKm, p.mach, p.gamma);
  return panelAero(b.mesh, { ...p, lockFlight: false, flightMach: p.mach }, atm, alpha, beta, Math.max(b.aero.planformArea, 1e-6), p.length, undefined, true);
}

/* ---- 1. no per-surface input = main e586e26, bit for bit ---- */
{
  // FNV-1a of positions | indices | surfaces from origin/main e586e26 (before this branch).
  const golden: [string, Partial<DesignParams>, string][] = [
    ["caret", {}, "814926bc5a485e08ec94719b"],
    ["caret-elev6", { elevonDeg: 6 }, "8f549dedd231ae48ec94719b"],
    ["caret-half", { halfModel: true, elevonDeg: -4 }, "32d07a8cdddffe09b344a308"],
    ["caret-bottom", { lid: "bottom", elevonDeg: 3 }, "847c7851c331b980ec94719b"],
    ["osculating", { family: "osculating" }, "009249dbaa7975a4ec94719b"],
    ["cone", { family: "cone", elevonDeg: 5 }, "89830790a41508e5e9582e2d"],
    ["liftbody", { family: "liftbody", elevonDeg: 4 }, "8cde5ccb414b11077327e435"],
    ["viscopt", { family: "viscopt" }, "a1498de9c2292d8cec94719b"],
    ["elliptic", { family: "elliptic" }, "2317d290e6a5d7dcec94719b"],
    ["integrated", { family: "integrated", elevonDeg: 2 }, "76a559ee14d5f13cec94719b"],
    ["star", { family: "star" }, "2cf4013773c3e0294a7232f5"],
    ["ramjet", { family: "ramjet" }, "232f19f3a4cb5749f585fd31"],
    ["capsule", { family: "capsule", span: 3.9, length: 3.2, leRadius: 0.35, coneDeg: 20 }, "cbcd60518123da51b960f84c"],
    ["boostglide", { family: "boostglide", planform: "double", length: 6, span: 2.4, elevonDeg: 3 }, "99445ab6d861cab9b7d67095"],
    ["missile-nofin", { family: "missile", span: 0.45, length: 4.5, finHeight: 0 }, "6dae8e38574b92b5b960f84c"],
  ];
  for (const [name, o, h] of golden) {
    const p = { ...DEFAULT_PARAMS, nx: 24, ny: 16, ...o };
    assert.equal(meshHash(buildVehicle(p)), h, `${name}: geometry differs from main e586e26`);
    const zeros: Record<string, number> = {};
    for (const s of listControlSurfaces(p)) zeros[s.id] = 0;
    assert.equal(meshHash(buildVehicle({ ...p, controls: zeros })), h, `${name}: all-zero controls must be main's geometry`);
  }
  const b = buildVehicle(P({}));
  assert.equal(deflectVehicle(b, { de: 0, da: 0 }), b, "zero input returns the same vehicle");
  console.log(`bit-identical: ${golden.length} configurations match main e586e26 with no / all-zero controls`);
}

/* ---- 2. listing ---- */
{
  const ids = (p: DesignParams) => listControlSurfaces(p).map((s) => s.id);
  assert.deepEqual(ids(P({})), ["elevon_l", "elevon_r", "body_flap"]);
  assert.deepEqual(ids(P({ finHeight: 0.5 })), ["elevon_l", "elevon_r", "body_flap", "rudder_l", "rudder_r"]);
  assert.deepEqual(ids(P({ halfModel: true })), ["elevon_r", "body_flap"]);
  for (const f of ["capsule", "star", "ramjet", "scramjet"] as const) assert.deepEqual(ids(P({ family: f })), [], `${f} has no surfaces`);
  assert.deepEqual(ids(preset("boostglide-m15")), ["elevon_l", "elevon_r", "body_flap"]);
  const m = listControlSurfaces(buildVehicle(preset("missile-m6")));
  assert.deepEqual(
    m.map((s) => s.id),
    ["fin0_0", "fin0_1", "fin0_2", "fin0_3", "fin1_0", "fin1_1", "fin1_2", "fin1_3"],
  );
  assert.equal(m[0].pair, "fin0_2");
  assert.equal(m[1].pair, null, "the top fin mirrors onto itself");
  assert.deepEqual(m[0].axis, [0, 1, 0]);
  assert.ok(m.every((s) => s.limitsDeg[1] > 5 && s.limitsDeg[0] === -s.limitsDeg[1]), "fin limits");
  assert.deepEqual(controlChannels(preset("missile-m6")), ["de", "da", "dr", "fs1_da", "fs1_de", "fs1_dr"]);
  const el = listControlSurfaces(buildVehicle(P({})));
  assert.equal(el[0].pair, "elevon_r");
  assert.ok(el[0].hinge.a[2] !== 0, "hinge Z from the built OML");
  const mix = mixControls(el, { de: 3, da: 2, dbf: 50 });
  assert.deepEqual(mix.deflections, { elevon_l: 1, elevon_r: 5, body_flap: 20 });
  assert.deepEqual(mix.clamped, ["body_flap"]);
  console.log("listing: waverider / fins / half-model / boost-glide / missile ok");
}

/* ---- 3. watertight, manifold, consistent, no fold-over: every surface at min, 0 and max ---- */
{
  const configs: [string, DesignParams][] = [
    ["caret+fins", P({ finHeight: 0.6, ny: 20 })],
    ["caret-half", P({ halfModel: true })],
    ["caret-bottom", P({ lid: "bottom", finHeight: 0.4 })],
    ["osculating", P({ family: "osculating" })],
    ["cone", P({ family: "cone" })],
    ["liftbody", P({ family: "liftbody" })],
    ["integrated", P({ family: "integrated" })],
    ["viscopt", P({ family: "viscopt" })],
    ["boostglide", preset("boostglide-m15")],
    ["missile", preset("missile-m6")],
    ["missile-3fin", { ...preset("missile-m6"), fins: 3, nRamps: 1 }],
    ["stack", preset("stack-2")],
  ];
  let checks = 0;
  const retri: string[] = [];
  for (const [name, p] of configs) {
    const base = buildVehicle(p);
    const q0 = base.quality;
    assert.ok(q0.watertight && q0.manifold, `${name} base watertight`);
    const pos0 = base.mesh.positions.slice();
    const surfs = listControlSurfaces(base);
    const baseFolds = foldEdges(base);
    assert.ok(surfs.length > 0, `${name} has surfaces`);
    for (const s of surfs) {
      for (const d of [s.limitsDeg[0], s.limitsDeg[1]]) {
        const b = buildVehicle({ ...p, controls: { [s.id]: d } });
        const q = b.quality;
        const tm = topoMetrics(b.mesh);
        const tag = `${name} ${s.id}=${d}`;
        assert.ok(q.watertight, `${tag} watertight (open ${q.openEdges})`);
        assert.ok(q.manifold, `${tag} manifold (${q.nonManifoldEdges})`);
        assert.equal(tm.inconsistentEdges, 0, `${tag} winding`);
        assert.equal(tm.degenerateTris, 0, `${tag} degenerate`);
        assert.ok(tm.signedVolume > 0, `${tag} positive volume`);
        assert.ok(Math.abs(q.volume - q0.volume) / q0.volume < 0.05, `${tag} volume ${q.volume} vs ${q0.volume}`);
        assert.notEqual(b.mesh.positions, base.mesh.positions, `${tag} new positions array`);
        // fold-over: a reversed triangle shows up as a near-180° dihedral against its neighbour
        const folds = foldEdges(b);
        assert.ok(folds <= baseFolds, `${tag} fold-over: ${folds} edges > 170° (base ${baseFolds})`);
        let moved = 0;
        if (b.mesh.positions.length === pos0.length) {
          for (let i = 0; i < pos0.length; i++) moved = Math.max(moved, Math.abs(b.mesh.positions[i] - pos0[i]));
        } else moved = Infinity;
        assert.ok(moved > 1e-4 * p.length, `${tag} moved ${moved}`);
        if (b.mesh.indices.length !== base.mesh.indices.length) retri.push(`${tag} (${base.mesh.indices.length / 3}→${b.mesh.indices.length / 3} tris)`);
        checks++;
      }
    }
    assert.deepEqual(base.mesh.positions, pos0, `${name} base untouched`);
  }
  if (retri.length) console.log(`  TE tidy re-triangulated: ${retri.join("; ")}`);
  assert.ok(retri.length <= 4, "triangle count is kept almost everywhere");
  console.log(`watertight: ${checks} surface × limit builds watertight, manifold, consistent, positive volume, no fold-over`);
}

/**
 * Edges inside one surface patch (same surface id on both faces) whose faces meet at
 * more than 170° normal-to-normal: a folded (reversed) cell. The knife TE and tips are
 * upper|lower seams and do not count; thin fin crests reach ~157° and do not either.
 */
function foldEdges(b: BuiltVehicle): number {
  const p = b.mesh.positions;
  const ind = b.mesh.indices;
  const nt = ind.length / 3;
  const nrm = new Float64Array(nt * 3);
  for (let t = 0; t < nt; t++) {
    const a = ind[t * 3] * 3;
    const c = ind[t * 3 + 1] * 3;
    const d = ind[t * 3 + 2] * 3;
    const ux = p[c] - p[a], uy = p[c + 1] - p[a + 1], uz = p[c + 2] - p[a + 2];
    const vx = p[d] - p[a], vy = p[d + 1] - p[a + 1], vz = p[d + 2] - p[a + 2];
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const m = Math.hypot(nx, ny, nz) || 1;
    nrm[t * 3] = nx / m;
    nrm[t * 3 + 1] = ny / m;
    nrm[t * 3 + 2] = nz / m;
  }
  const first = new Map<string, number>();
  let n = 0;
  const lim = Math.cos((170 * Math.PI) / 180);
  for (let t = 0; t < nt; t++) {
    for (let k = 0; k < 3; k++) {
      const u = ind[t * 3 + k];
      const v = ind[t * 3 + ((k + 1) % 3)];
      const key = u < v ? `${u}:${v}` : `${v}:${u}`;
      const o = first.get(key);
      if (o === undefined) {
        first.set(key, t);
        continue;
      }
      if (b.mesh.surfaces[o] !== b.mesh.surfaces[t]) continue;
      const dot = nrm[o * 3] * nrm[t * 3] + nrm[o * 3 + 1] * nrm[t * 3 + 1] + nrm[o * 3 + 2] * nrm[t * 3 + 2];
      if (dot < lim) n++;
    }
  }
  return n;
}

/* ---- 4. aileron antisymmetry and channel signs on a waverider ---- */
{
  const base = buildVehicle(P({ finHeight: 0.5, ny: 28, nx: 28 }));
  const a0 = aero(base);
  assert.ok(Math.abs(a0.cll) < 1e-12 && Math.abs(a0.cy) < 1e-12, "symmetric base");
  const at = (input: Record<string, number>) => aero(deflectVehicle(base, input));
  const h = 4;
  const ap = at({ da: h });
  const am = at({ da: -h });
  const ep = at({ de: h });
  const em = at({ de: -h });
  const CLde = (ep.cl - em.cl) / (2 * h);
  const Cmde = (ep.cm - em.cm) / (2 * h);
  const clDa = (ap.cll - am.cll) / (2 * h);
  const CLda = (ap.cl - am.cl) / (2 * h);
  const Cmda = (ap.cm - am.cm) / (2 * h);
  // ±δa are mirror images. They match to the quad-diagonal discretization: the split of a
  // warped quad is not mirror-symmetric, so require 1e-3 of the first-order elevator terms.
  assert.ok(clDa > 1e-5, `Cll_da > 0 (right TE down raises the right wing): ${clDa}`);
  assert.ok(Math.abs(CLda) < 1e-3 * Math.abs(CLde), `CL_da ${CLda} ≈ 0 vs CL_de ${CLde}`);
  assert.ok(Math.abs(Cmda) < 1e-3 * Math.abs(Cmde), `Cm_da ${Cmda} ≈ 0 vs Cm_de ${Cmde}`);
  assert.ok(Math.abs(ap.cll + am.cll) < 1e-2 * Math.abs(ap.cll - am.cll), "Cll odd in δa");
  // the second-order (even) CL change of a ±4° aileron stays small next to the elevator's first-order one
  assert.ok(Math.abs(ap.cl - a0.cl) < 0.15 * Math.abs(CLde * h), `ΔCL(δa) ${ap.cl - a0.cl} small vs CL_de·h ${CLde * h}`);
  assert.ok(CLde > 0 && Cmde < 0, `CL_de ${CLde} > 0, Cm_de ${Cmde} < 0 (TE down)`);
  assert.ok(Math.abs(ep.cll) < 1e-2 * Math.abs(clDa * h) && Math.abs(ep.cn) < 1e-2 * Math.abs(clDa * h), `δe stays symmetric: Cll ${ep.cll}`);
  const bp = at({ dbf: h });
  const bm = at({ dbf: -h });
  assert.ok((bp.cm - bm.cm) / (2 * h) < 0, "Cm_dbf < 0");
  const rp = at({ dr: h });
  const rm = at({ dr: -h });
  const CYdr = (rp.cy - rm.cy) / (2 * h);
  const Cndr = (rp.cn - rm.cn) / (2 * h);
  assert.ok(CYdr > 0 && Cndr > 0, `TE-left rudder aft of the CG: CY_dr ${CYdr} > 0, Cn_dr ${Cndr} > 0 (main's +Z axis)`);
  assert.ok(Math.abs(rp.cl - rm.cl) < 1e-3 * Math.abs(CLde), "CL even in δr");
  // equal elevons + body flap at a common angle ≈ main's full-span elevon (same shear path)
  const full = buildVehicle({ ...base.params, elevonDeg: 5 });
  const part = buildVehicle({ ...base.params, controls: { elevon_l: 5, elevon_r: 5 } });
  const af = aero(full);
  const apart = aero(part);
  assert.ok(apart.cl > a0.cl && apart.cl < af.cl, "part-span elevons give part of the full-span lift");
  console.log(
    `waverider: Cll_da ${clDa.toExponential(3)} CL_da ${CLda.toExponential(1)} CL_de ${CLde.toExponential(3)} Cm_de ${Cmde.toExponential(3)} CY_dr ${CYdr.toExponential(3)} Cn_dr ${Cndr.toExponential(3)} /deg`,
  );
}

/* ---- 5. panel WeakMap: base → deflect → deflected never reads stale triangles ---- */
{
  const base = buildVehicle(P({}));
  const refs = { pos: base.mesh.positions, ind: base.mesh.indices, srf: base.mesh.surfaces };
  const copy = { pos: base.mesh.positions.slice(), ind: base.mesh.indices.slice(), srf: base.mesh.surfaces.slice() };
  const a0 = aero(base);
  for (const input of [{ de: 8 }, { da: 6 }, { elevon_l: -10 }, { dbf: 12 }]) {
    const d = deflectVehicle(base, input);
    assert.notEqual(d.mesh, base.mesh);
    assert.notEqual(d.mesh.positions, refs.pos, "deflected positions are a new array");
    assert.notEqual(d.mesh.indices, refs.ind, "deflected indices are a new array");
    assert.notEqual(d.mesh.surfaces, refs.srf, "deflected surfaces are a new array");
    const a1 = aero(d);
    assert.ok(Math.abs(a1.cl - a0.cl) + Math.abs(a1.cm - a0.cm) + Math.abs(a1.cll - a0.cll) > 1e-6, `${JSON.stringify(input)} changes the coefficients`);
    assert.equal(base.mesh.positions, refs.pos);
    assert.equal(base.mesh.indices, refs.ind);
    assert.equal(base.mesh.surfaces, refs.srf);
    assert.deepEqual(base.mesh.positions, copy.pos, "base positions unchanged by value");
    assert.deepEqual(base.mesh.indices, copy.ind);
    assert.deepEqual(base.mesh.surfaces, copy.srf);
    const again = aero(base);
    assert.equal(again.cl, a0.cl, "base coefficients unchanged after the deflected solve");
    assert.equal(again.cm, a0.cm);
  }
  console.log("weakmap: deflected meshes are new arrays, base unchanged by value and identity, no stale cache");
}

/* ---- 6. missile fin set: roll symmetry and mixing ---- */
{
  const base = buildVehicle(preset("missile-m6"));
  const z = aero(base, 0, 0);
  for (const k of ["cy", "cn", "cll", "cm"] as const) assert.ok(Math.abs(z[k]) < 1e-9, `4-fin missile at α = β = 0: ${k} = ${z[k]}`);
  const a4 = aero(base, 4, 0);
  // mirror symmetry holds to the quad-diagonal split (main's uniform ring gave CY −2.6e-2, Cll 2.4e-3)
  assert.ok(Math.abs(a4.cll) < 1e-5 && Math.abs(a4.cy) < 1e-4 && Math.abs(a4.cn) < 1e-5, `missile β = 0 lateral ≈ 0: CY ${a4.cy} Cll ${a4.cll} Cn ${a4.cn}`);
  const roll = aero(deflectVehicle(base, { da: 5 }), 0, 0);
  assert.ok(roll.cll > 1e-5, `δa rolls (+Cll): ${roll.cll}`);
  for (const k of ["cl", "cy", "cm", "cn"] as const) assert.ok(Math.abs(roll[k] - z[k]) < 1e-9, `pure roll at α = 0: Δ${k} ${roll[k] - z[k]}`);
  const rp = aero(deflectVehicle(base, { da: 5 }), 4, 0);
  const rm = aero(deflectVehicle(base, { da: -5 }), 4, 0);
  assert.ok(Math.abs(rp.cll + rm.cll - 2 * a4.cll) < 2e-2 * Math.abs(rp.cll - rm.cll), `Cll odd in δa at α = 4: ${rp.cll} ${rm.cll}`);
  assert.ok(Math.abs(rp.cl - rm.cl) < 1e-4, `CL even in δa at α = 4: ${rp.cl - rm.cl}`);
  const top = aero(deflectVehicle(base, { fin0_1: 5 }), 0, 0);
  const bot = aero(deflectVehicle(base, { fin0_3: 5 }), 0, 0);
  assert.ok(Math.abs(top.cll - bot.cll) < 1e-9 && top.cll > 0, "top and bottom fins roll the same way");
  assert.ok(Math.abs(top.cy + bot.cy) < 1e-9, "and push opposite sides");
  const e = aero(deflectVehicle(base, { de: 5 }), 4, 0);
  assert.ok(Math.abs(e.cll - a4.cll) < 1e-2 * roll.cll && Math.abs(e.cy - a4.cy) < 1e-4, `fin pitch is symmetric: ΔCll ${e.cll - a4.cll}`);
  assert.ok(e.cm < a4.cm, "TE-down tail fins pitch the nose down");
  const r = aero(deflectVehicle(base, { dr: 5 }), 0, 0);
  assert.ok(r.cy > 0 && r.cn > 0, `fin rudder TE left: CY ${r.cy} > 0, Cn ${r.cn} > 0`);
  // 3 fins: δe mixes cos θk, so pitch does not roll. At α = 0 every fin sees the same flow, so the
  // linear (central-difference) roll cancels; main's shear moved only the +Y fin and rolled.
  const three = buildVehicle({ ...preset("missile-m6"), fins: 3, nRamps: 1 });
  const dCll = (input: (s: number) => Record<string, number>) =>
    (aero(deflectVehicle(three, input(1)), 0, 0).cll - aero(deflectVehicle(three, input(-1)), 0, 0).cll) / 2;
  const leak = dCll((s) => ({ de: s }));
  const solo = dCll((s) => ({ fin0_0: s }));
  assert.ok(Math.abs(leak) < 0.02 * Math.abs(solo), `3-fin Cll_de ${leak} vs single fin ${solo}`);
  console.log(`missile: Cll(δa=5) ${roll.cll.toExponential(3)}, symmetric to 1e-9; rudder CY ${r.cy.toExponential(3)} Cn ${r.cn.toExponential(3)}`);
}

/* ---- 7. aero database: control axes, derivatives, cap, exports ---- */
{
  const built = buildVehicle(P({ length: 3, span: 1.6, height: 0.4, shockDeg: 14 }));
  const spec: AeroDbSpec = {
    ...defaultAeroDbSpec(built.params),
    machMin: 6,
    machMax: 8,
    machStep: 2,
    alphaMin: 0,
    alphaMax: 4,
    alphaStep: 4,
    betaMin: 0,
    betaMax: 0,
    deltaMin: 0,
    deltaMax: 0,
    method: "newtonian",
    dynamicDerivatives: false,
    controlAxes: [
      { channel: "de", min: -4, max: 4, step: 4 },
      { channel: "da", min: -4, max: 4, step: 4 },
    ],
    controlStepDeg: 4,
  };
  const g = gridCount(spec, built.params.mach, built.params.alphaDeg);
  assert.equal(g.points, g.axes.mach.length * g.axes.alpha_deg.length * 9);
  const db = await generateAeroDatabase(built, spec);
  assert.equal(db.points.length, g.points);
  assert.equal(db.axes.controls.length, 2);
  // flat index: ((((im*nA+ia)*nB+ib)*nD+id)*n_de + i_de)*n_da + i_da
  const nA = db.axes.alpha_deg.length;
  const idx = (im: number, ia: number, ide: number, ida: number) => (im * nA + ia) * 9 + ide * 3 + ida;
  const pt = db.points[idx(1, 1, 2, 0)];
  assert.deepEqual([pt.mach, pt.alpha_deg, pt.controls.de, pt.controls.da], [db.axes.mach[1], db.axes.alpha_deg[1], 4, -4]);
  // the interior table derivative equals the anchor ±4° central difference at the same point
  for (let im = 0; im < db.axes.mach.length; im++) {
    const ia = db.axes.alpha_deg.indexOf(0);
    const p = db.points[idx(im, ia, 1, 1)];
    for (const ch of ["de", "da"]) {
      const row = db.controlDerivatives.find((r) => r.channel === ch && r.mach === db.axes.mach[im]);
      assert.ok(row && p.controlDerivs[ch], `${ch} derivative present`);
      for (const k of ["cl", "cm", "cll", "cn", "cy"] as const) {
        assert.ok(Math.abs(p.controlDerivs[ch][k] - row[k]) < 1e-12, `table ${ch}.${k} ${p.controlDerivs[ch][k]} = anchor ${row[k]}`);
      }
    }
    assert.ok(Math.abs(p.controlDerivs.da.cl) < 1e-10 && p.controlDerivs.da.cll > 0, "Cll_da > 0, CL_da = 0");
    assert.ok(p.controlDerivs.de.cl > 0 && p.controlDerivs.de.cm < 0, "CL_de > 0, Cm_de < 0");
  }
  assert.ok(db.controlDerivatives.some((r) => r.channel === "dbf" && r.cm < 0), "anchor block covers unswept channels (dbf)");
  assert.equal(db.points[idx(0, 0, 0, 0)].controlDerivs.de, undefined, "axis edge has no two-sided stencil");
  const csv = aeroDbToCsv(db);
  const header = csv.split("\n").find((l) => l.startsWith("mach,"))!;
  assert.ok(header.startsWith("mach,alpha_deg,beta_deg,delta_deg,on_design,CL,"), "main's columns first");
  for (const c of ["de_deg", "da_deg", "Cll_da_per_deg", "Cm_de_per_deg", "CL_de_per_deg", "Cn_da_per_deg"]) assert.ok(header.split(",").includes(c), `csv ${c}`);
  assert.ok(csv.includes("# ctrl_deriv,") && csv.includes("# control: da:"), "csv anchor block and conventions");
  const rows = csv.split("\n").filter((l) => l && !l.startsWith("#"));
  assert.equal(rows.length, db.points.length + 1);
  assert.ok(rows.every((r) => r.split(",").length === header.split(",").length), "csv rectangular");
  const back = JSON.parse(aeroDbToJson(db));
  assert.equal(back.points.length, db.points.length);
  assert.equal(back.controlDerivatives.length, db.controlDerivatives.length);
  assert.equal(back.meta.units.controlDerivatives, "1/deg");
  const m = aeroDbToSimulink(db);
  assert.ok(m.includes("reshape([") && m.includes("n_da, n_de, nD, nB, nA, nM"), "simulink dims");
  assert.ok(m.includes("aero.ctrl.da.Cll = [") && m.includes("aero.Cll_da = reshape(") && m.includes("aero.ctrl_axes.de_deg"), "simulink control tables");
  const cd = aeroDbControlDerivativesToCsv(db);
  assert.ok(cd.split("\n")[0].startsWith("# cuspis control derivatives"));
  // cap and validation
  const big: AeroDbSpec = { ...spec, alphaMin: -10, alphaMax: 20, alphaStep: 1, controlAxes: [{ channel: "de", min: -10, max: 10, step: 2 }] };
  assert.ok(gridCount(big, 8, 0).points > AERODB_POINT_CAP);
  await assert.rejects(generateAeroDatabase(built, big), /cap is 480/);
  await assert.rejects(generateAeroDatabase(built, { ...spec, controlAxes: [{ channel: "dr", min: 0, max: 4, step: 4 }] }), /does not exist/);
  // no control axes: main's layout and columns
  const plain = await generateAeroDatabase(built, { ...spec, controlAxes: [], controlDerivatives: false });
  assert.equal(plain.points.length, db.axes.mach.length * nA);
  assert.ok(plain.meta.index.startsWith("flat i = ((im*nAlpha + ia)*nBeta + ib)*nDelta + id"));
  assert.ok(aeroDbToSimulink(plain).includes("nD, nB, nA, nM);"));
  console.log(`aerodb: ${db.points.length} points with δe × δa axes, ${db.controlDerivatives.length} anchor rows, exports ok`);
}

/* ---- 8. missile database: fin channels as axes ---- */
{
  const built = buildVehicle({ ...preset("missile-m6"), nx: 24, ny: 16 });
  const db = await generateAeroDatabase(built, {
    ...defaultAeroDbSpec(built.params),
    machMin: 6,
    machMax: 6,
    alphaMin: 0,
    alphaMax: 4,
    alphaStep: 4,
    method: "newtonian",
    dynamicDerivatives: false,
    controlAxes: [{ channel: "dr", min: -5, max: 5, step: 5 }],
  });
  const p = db.points.find((q) => q.alpha_deg === 0 && q.controls.dr === 0)!;
  assert.ok(p.controlDerivs.dr.cy > 0 && p.controlDerivs.dr.cn > 0, "missile CY_dr, Cn_dr > 0");
  assert.ok(db.controlDerivatives.some((r) => r.channel === "fs1_da"), "forward fin set channels");
  const w = withControls(built.params, { de: 2, dr: 1 });
  assert.deepEqual(w.controls, { fin0_0: 2, fin0_1: -1, fin0_2: -2, fin0_3: 1 });
  console.log("aerodb missile: dr axis and fin-set channels ok");
}

console.log(`controls.test ok (${((performance.now() - t0) / 1000).toFixed(1)} s)`);
