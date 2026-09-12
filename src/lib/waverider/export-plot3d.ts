import type { SurfaceGrid, TriMesh, Vec3 } from "./types";
import { gridPoint, makeGrid } from "./mesh";

function fmt(n: number) {
  const v = Number.isFinite(n) ? n : 0;
  return v.toExponential(8);
}

function stripAt(g: SurfaceGrid, i: number): Vec3[] {
  const out: Vec3[] = [];
  for (let j = 0; j < g.nj; j++) out.push(gridPoint(g, i, j));
  return out;
}

function sideAt(g: SurfaceGrid, j: number): Vec3[] {
  const out: Vec3[] = [];
  for (let i = 0; i < g.ni; i++) out.push(gridPoint(g, i, j));
  return out;
}

function ruledGrid(name: string, a: Vec3[], b: Vec3[]): SurfaceGrid | null {
  const nj = Math.min(a.length, b.length);
  if (nj < 2) return null;
  let maxd = 0;
  for (let j = 0; j < nj; j++) {
    maxd = Math.max(maxd, Math.hypot(a[j][0] - b[j][0], a[j][1] - b[j][1], a[j][2] - b[j][2]));
  }
  if (maxd < 1e-9) return null;
  return makeGrid(name, 2, nj, (i, j) => (i === 0 ? a[j] : b[j]));
}

/** Named 2-D patches Pointwise can import as database surfaces. */
export function surfacePatches(grids: SurfaceGrid[]): SurfaceGrid[] {
  const out: SurfaceGrid[] = [];
  for (const g of grids) {
    if (g.ni >= 2 && g.nj >= 2) out.push(g);
  }
  const upper = grids.find((g) => g.name === "upper" || g.name === "cowl");
  const lower = grids.find((g) => g.name === "lower");
  if (upper && lower && upper.nj === lower.nj) {
    const te = ruledGrid("base_or_nozzle", stripAt(upper, upper.ni - 1), stripAt(lower, lower.ni - 1));
    if (te) out.push(te);
    const hasLead = grids.some((g) => g.name === "leading" || g.name === "cowl_lip");
    if (!hasLead) {
      const le = ruledGrid("inlet_or_le", stripAt(upper, 0), stripAt(lower, 0));
      if (le) out.push(le);
    }
    const jL = 0;
    const jR = upper.nj - 1;
    const sl = ruledGrid("side_l", sideAt(upper, jL), sideAt(lower, Math.min(jL, lower.nj - 1)));
    const sr = ruledGrid("side_r", sideAt(upper, jR), sideAt(lower, Math.min(jR, lower.nj - 1)));
    if (sl) out.push(sl);
    if (sr) out.push(sr);
  }
  return out;
}

/** Multi-block formatted Plot3D XYZ (3-D, nk=1). Pointwise: File > Import > Plot3D, 3-D, formatted, no IBLANK. */
export function gridsToPlot3d(grids: SurfaceGrid[]): string {
  const blocks = surfacePatches(grids).filter((g) => g.ni >= 2 && g.nj >= 2);
  const lines: string[] = [`${blocks.length}`];
  for (const g of blocks) lines.push(`${g.ni} ${g.nj} 1`);
  for (const g of blocks) {
    const xs: string[] = [];
    const ys: string[] = [];
    const zs: string[] = [];
    for (let k = 0; k < 1; k++) {
      for (let j = 0; j < g.nj; j++) {
        for (let i = 0; i < g.ni; i++) {
          const o = (i * g.nj + j) * 3;
          xs.push(fmt(g.xyz[o]));
          ys.push(fmt(g.xyz[o + 1]));
          zs.push(fmt(g.xyz[o + 2]));
        }
      }
    }
    const wrap = (arr: string[]) => {
      for (let i = 0; i < arr.length; i += 5) lines.push(arr.slice(i, i + 5).join(" "));
    };
    wrap(xs);
    wrap(ys);
    wrap(zs);
  }
  return lines.join("\n") + "\n";
}

export function meshToObj(mesh: { positions: Float64Array; indices: Uint32Array; surfaces: Uint8Array }): string {
  const names = ["upper", "lower", "base", "leading", "symmetry", "inlet", "nozzle", "cowl"];
  const lines: string[] = ["# Cuspis waverider", "o waverider"];
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

const SURF_NAMES = ["upper", "lower", "base", "leading", "symmetry", "inlet", "nozzle", "cowl"];

export function meshToAsciiStlRegions(mesh: TriMesh, name: string): string {
  const buckets = new Map<number, number[]>();
  const nt = mesh.indices.length / 3;
  for (let t = 0; t < nt; t++) {
    const s = mesh.surfaces[t] ?? 0;
    const arr = buckets.get(s) ?? [];
    arr.push(t);
    buckets.set(s, arr);
  }
  const lines: string[] = [];
  const get = (i: number): Vec3 => [mesh.positions[i * 3], mesh.positions[i * 3 + 1], mesh.positions[i * 3 + 2]];
  for (const [s, tris] of buckets) {
    const solid = `${name}_${SURF_NAMES[s] ?? `s${s}`}`;
    lines.push(`solid ${solid}`);
    for (const t of tris) {
      const a = get(mesh.indices[t * 3]);
      const b = get(mesh.indices[t * 3 + 1]);
      const c = get(mesh.indices[t * 3 + 2]);
      const abx = b[0] - a[0], aby = b[1] - a[1], abz = b[2] - a[2];
      const acx = c[0] - a[0], acy = c[1] - a[1], acz = c[2] - a[2];
      const nx = aby * acz - abz * acy;
      const ny = abz * acx - abx * acz;
      const nz = abx * acy - aby * acx;
      const mag = Math.hypot(nx, ny, nz) || 1;
      lines.push(`  facet normal ${nx / mag} ${ny / mag} ${nz / mag}`);
      lines.push("    outer loop");
      lines.push(`      vertex ${a[0]} ${a[1]} ${a[2]}`);
      lines.push(`      vertex ${b[0]} ${b[1]} ${b[2]}`);
      lines.push(`      vertex ${c[0]} ${c[1]} ${c[2]}`);
      lines.push("    endloop");
      lines.push("  endfacet");
    }
    lines.push(`endsolid ${solid}`);
  }
  return lines.join("\n") + "\n";
}

export function meshToVtk(
  mesh: TriMesh,
  cp?: Float32Array,
  heat?: Float32Array,
  tw?: Float32Array,
): string {
  const nv = mesh.positions.length / 3;
  const nt = mesh.indices.length / 3;
  const lines: string[] = [
    "# vtk DataFile Version 3.0",
    "Cuspis panel Cp / heat",
    "ASCII",
    "DATASET UNSTRUCTURED_GRID",
    `POINTS ${nv} double`,
  ];
  for (let i = 0; i < nv; i++) {
    lines.push(`${mesh.positions[i * 3]} ${mesh.positions[i * 3 + 1]} ${mesh.positions[i * 3 + 2]}`);
  }
  lines.push(`CELLS ${nt} ${nt * 4}`);
  for (let t = 0; t < nt; t++) {
    lines.push(`3 ${mesh.indices[t * 3]} ${mesh.indices[t * 3 + 1]} ${mesh.indices[t * 3 + 2]}`);
  }
  lines.push(`CELL_TYPES ${nt}`);
  for (let t = 0; t < nt; t++) lines.push("5");
  lines.push(`CELL_DATA ${nt}`);
  lines.push("SCALARS surface int 1");
  lines.push("LOOKUP_TABLE default");
  for (let t = 0; t < nt; t++) lines.push(String(mesh.surfaces[t] ?? 0));
  if (cp && cp.length >= nt) {
    lines.push("SCALARS Cp float 1");
    lines.push("LOOKUP_TABLE default");
    for (let t = 0; t < nt; t++) lines.push(fmt(cp[t]));
  }
  if (heat && heat.length >= nt) {
    lines.push("SCALARS q_W_cm2 float 1");
    lines.push("LOOKUP_TABLE default");
    for (let t = 0; t < nt; t++) lines.push(fmt(heat[t]));
  }
  if (tw && tw.length >= nt) {
    lines.push("SCALARS Tw_eq_K float 1");
    lines.push("LOOKUP_TABLE default");
    for (let t = 0; t < nt; t++) lines.push(fmt(tw[t]));
  }
  return lines.join("\n") + "\n";
}

export function pointwiseGlyph(name: string): string {
  return `# Pointwise Glyph 2 — Cuspis ${name}
# File > Glyph > Execute, or: pointwise -b ${name}.glf
package require PWI_Glyph 2

pw::Application reset
pw::Application setCAESolver {CGNS} 3

set here [file dirname [info script]]
set stl [file join $here "${name}.stl"]
set p3d [file join $here "${name}.x"]
set igs [file join $here "${name}.igs"]

# STL is the reliable path — triangles, not a control-point cloud.
if {[file exists $stl]} {
  if {[catch { pw::Database import -type STL $stl } err]} {
    puts "STL import note: $err"
  } else {
    puts "Imported watertight STL $stl"
  }
}

# IGES type-128 NURBS (Pointwise's most reliable CAD surface import).
if {[file exists $igs]} {
  if {[catch { pw::Database import -type IGES $igs } err]} {
    puts "IGES import note: $err"
  } else {
    puts "Imported IGES NURBS $igs"
  }
}

# Plot3D 3-D formatted, nk=1, IBLANK off — database surface patches, NOT XYZ scatter.
if {[file exists $p3d]} {
  if {[catch { pw::Database import -type PLOT3D $p3d } err]} {
    puts "Plot3D import note: $err — import ${name}.x as Plot3D 3-D formatted, IBLANK off."
  } else {
    puts "Imported Plot3D structured patches $p3d"
  }
}

puts "Frame: X stream, Y span, Z up. Nose / cowl lip at origin."
puts "Ramjet: blocks named inlet and nozzle are the capture and exhaust planes."
puts "Next: assemble domains from the database, T-Rex from walls, extrude volume."
`;
}
