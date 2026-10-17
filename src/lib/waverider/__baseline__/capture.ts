/**
 * Capture legacy (pre-refinement) outputs so tests can prove that the
 * "medium" preset reproduces today's behaviour.
 *
 *   bun run src/lib/waverider/__baseline__/capture.ts <repoRoot> <out.json>
 *
 * Run it against the untouched base clone (b5fdccc) to produce the fixture.
 * Only imports from <repoRoot>; writes only to <out.json>.
 */
import { writeFileSync } from "node:fs";

const root = process.argv[2];
const out = process.argv[3];
if (!root || !out) throw new Error("usage: capture.ts <repoRoot> <out.json>");
const wr = `${root}/src/lib/waverider`;

const { eulerWedge } = await import(`${wr}/euler2d.ts`);
const { buildVehicle } = await import(`${wr}/generate.ts`);
const { analyzeMesh } = await import(`${wr}/mesh.ts`);
const { panelAero } = await import(`${wr}/panel.ts`);
const { atmosphere } = await import(`${wr}/atmosphere.ts`);
const { DEFAULT_PARAMS } = await import(`${wr}/types.ts`);

function fnv(arr: ArrayLike<number>): string {
  // FNV-1a over the IEEE bytes of every value: bitwise comparison.
  const f = new Float64Array(arr.length);
  for (let i = 0; i < arr.length; i++) f[i] = arr[i];
  const u = new Uint8Array(f.buffer);
  let h = 0x811c9dc5;
  for (let i = 0; i < u.length; i++) {
    h ^= u[i];
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h.toString(16).padStart(8, "0");
}

const wedge: Record<string, unknown> = {};
for (const [M, th] of [
  [2, 10],
  [8, 8],
  [8, 30],
  [5, 15],
  [8, 12],
] as const) {
  const r = eulerWedge(M, th, 1.4);
  wedge[`${M}|${th}`] = { p2p1: r.p2p1, exactP: r.exactP, relErr: r.relErr, cp: r.cp, exactCp: r.exactCp, residual: r.residual, iters: r.iters };
}

const vehicles: Record<string, unknown> = {};
for (const family of ["caret", "osculating", "cone", "liftbody", "wedgecone"]) {
  for (const alphaDeg of [0, 3]) {
    const params = { ...DEFAULT_PARAMS, family, alphaDeg };
    const built = buildVehicle(params);
    const q = analyzeMesh(built.mesh);
    const Mf = params.lockFlight ? params.mach : params.flightMach;
    const atm = atmosphere(params.altKm, Mf, params.gamma);
    const a = panelAero(built.mesh, params, atm, alphaDeg, 0, Math.max(built.aero.planformArea, 1e-6), params.length);
    vehicles[`${family}|${alphaDeg}`] = {
      triangles: q.triangles,
      vertices: q.vertices,
      watertight: q.watertight,
      manifold: q.manifold,
      posHash: fnv(built.mesh.positions),
      idxHash: fnv(Array.from(built.mesh.indices)),
      cl: a.cl,
      cd: a.cd,
      cm: a.cm,
      ca: a.ca,
      method: a.method,
      cpHash: fnv(Array.from(a.cp)),
      cpLen: a.cp.length,
    };
  }
}
writeFileSync(out, JSON.stringify({ capturedFrom: root, wedge, vehicles }, null, 2));
console.log("wrote", out, Object.keys(wedge).length, "wedge cases,", Object.keys(vehicles).length, "vehicle cases");
