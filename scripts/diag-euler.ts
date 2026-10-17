import { atmosphere } from "../src/lib/waverider/atmosphere";
import { buildVehicle } from "../src/lib/waverider/generate";
import { panelAero } from "../src/lib/waverider/panel";
import { DEFAULT_PARAMS, SURFACE_ID, type DesignParams, type WaveriderFamily } from "../src/lib/waverider/types";
import { DEG, betaFromThetaM, obliqueShock } from "../src/lib/waverider/math";

const fam = (process.argv[2] ?? "caret") as WaveriderFamily;
const alpha = Number(process.argv[3] ?? 0);
const p: DesignParams = { ...DEFAULT_PARAMS, family: fam, nx: 32, ny: 20, alphaDeg: alpha };
const b = buildVehicle(p);
const atm = atmosphere(p.altKm, p.flightMach, 1.4);
const S = b.aero.planformArea;
const res: Record<string, ReturnType<typeof panelAero>> = {};
for (const m of ["euler", "cbaero", "tangent", "mixed"] as const) res[m] = panelAero(b.mesh, { ...p, aeroMethod: m }, atm, alpha, 0, S, p.length);
for (const m of Object.keys(res)) console.log(m.padEnd(8), "CL", res[m].cl.toFixed(4), "CD", res[m].cd.toFixed(4), "L/D", res[m].ld.toFixed(3), "Cm", res[m].cm.toFixed(4));
const pos = b.mesh.positions, ind = b.mesh.indices;
const al = alpha * DEG; const v = [Math.cos(al), 0, Math.sin(al)];
const M = p.flightMach;
let rows: string[] = [];
for (let t = 0; t < ind.length / 3; t += 7) {
  if (b.mesh.surfaces[t] !== SURFACE_ID.lower) continue;
  const i0 = ind[t*3]*3, i1 = ind[t*3+1]*3, i2 = ind[t*3+2]*3;
  const u=[pos[i1]-pos[i0],pos[i1+1]-pos[i0+1],pos[i1+2]-pos[i0+2]], w=[pos[i2]-pos[i0],pos[i2+1]-pos[i0+1],pos[i2+2]-pos[i0+2]];
  const n=[u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]]; const m=Math.hypot(...n);
  const s=-(n[0]*v[0]+n[1]*v[1]+n[2]*v[2])/m; if (s<0.02) continue;
  const th=Math.asin(s); const be=betaFromThetaM(M,th,1.4);
  const ex=(obliqueShock(M,be,1.4).p2p1-1)/(0.5*1.4*M*M);
  const cx=(pos[i0]+pos[i1]+pos[i2])/3, cy=(pos[i0+1]+pos[i1+1]+pos[i2+1])/3;
  rows.push(`x=${cx.toFixed(2)} y=${cy.toFixed(2)} ny=${(n[1]/m).toFixed(3)} th=${(th/DEG).toFixed(2)} exact=${ex.toFixed(4)} euler=${res.euler.cp[t].toFixed(4)} cbaero=${res.cbaero.cp[t].toFixed(4)} mixed=${res.mixed.cp[t].toFixed(4)}`);
}
console.log(rows.slice(0, 40).join("\n"));
