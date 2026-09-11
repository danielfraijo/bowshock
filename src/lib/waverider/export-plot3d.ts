import type { SurfaceGrid } from "./types";

function fmt(n: number) {
  const v = Number.isFinite(n) ? n : 0;
  return v.toExponential(8);
}

/** Multi-block formatted Plot3D XYZ (nk = 1 surfaces). Pointwise File > Import > Plot3D. */
export function gridsToPlot3d(grids: SurfaceGrid[]): string {
  const blocks = grids.filter((g) => g.ni >= 2 && g.nj >= 2);
  const lines: string[] = [String(blocks.length)];
  for (const g of blocks) lines.push(`${g.ni} ${g.nj} 1`);
  for (const g of blocks) {
    const xs: string[] = [];
    const ys: string[] = [];
    const zs: string[] = [];
    for (let j = 0; j < g.nj; j++) {
      for (let i = 0; i < g.ni; i++) {
        const o = (i * g.nj + j) * 3;
        xs.push(fmt(g.xyz[o]));
        ys.push(fmt(g.xyz[o + 1]));
        zs.push(fmt(g.xyz[o + 2]));
      }
    }
    const wrap = (arr: string[]) => {
      for (let i = 0; i < arr.length; i += 4) lines.push(arr.slice(i, i + 4).join(" "));
    };
    wrap(xs);
    wrap(ys);
    wrap(zs);
  }
  return lines.join("\n") + "\n";
}

export function meshToObj(mesh: { positions: Float64Array; indices: Uint32Array; surfaces: Uint8Array }): string {
  const names = ["upper", "lower", "base", "leading", "symmetry", "inlet", "nozzle", "cowl"];
  const lines: string[] = ["# Bowshock waverider", "o waverider"];
  const nv = mesh.positions.length / 3;
  for (let i = 0; i < nv; i++) {
    lines.push(`v ${mesh.positions[i * 3]} ${mesh.positions[i * 3 + 1]} ${mesh.positions[i * 3 + 2]}`);
  }
  let last = -1;
  const nt = mesh.indices.length / 3;
  for (let t = 0; t < nt; t++) {
    const s = mesh.surfaces[t] ?? 0;
    if (s !== last) {
      lines.push(`g ${names[s] ?? `surf${s}`}`);
      last = s;
    }
    const a = mesh.indices[t * 3] + 1;
    const b = mesh.indices[t * 3 + 1] + 1;
    const c = mesh.indices[t * 3 + 2] + 1;
    lines.push(`f ${a} ${b} ${c}`);
  }
  return lines.join("\n") + "\n";
}
