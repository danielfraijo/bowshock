import type { TriMesh, Vec3 } from "./types";
import { vcross, vnorm, vsub } from "./math";

function get(mesh: TriMesh, i: number): Vec3 {
  return [mesh.positions[i * 3], mesh.positions[i * 3 + 1], mesh.positions[i * 3 + 2]];
}

export function meshToAsciiStl(mesh: TriMesh, name = "waverider"): string {
  const lines: string[] = [`solid ${name.replace(/\s+/g, "_")}`];
  const nt = mesh.indices.length / 3;
  for (let t = 0; t < nt; t++) {
    const a = get(mesh, mesh.indices[t * 3]);
    const b = get(mesh, mesh.indices[t * 3 + 1]);
    const c = get(mesh, mesh.indices[t * 3 + 2]);
    const n = vnorm(vcross(vsub(b, a), vsub(c, a)));
    lines.push(`  facet normal ${n[0]} ${n[1]} ${n[2]}`);
    lines.push("    outer loop");
    lines.push(`      vertex ${a[0]} ${a[1]} ${a[2]}`);
    lines.push(`      vertex ${b[0]} ${b[1]} ${b[2]}`);
    lines.push(`      vertex ${c[0]} ${c[1]} ${c[2]}`);
    lines.push("    endloop");
    lines.push("  endfacet");
  }
  lines.push(`endsolid ${name.replace(/\s+/g, "_")}`);
  return lines.join("\n");
}

export function meshToBinaryStl(mesh: TriMesh, name = "waverider"): ArrayBuffer {
  const nt = mesh.indices.length / 3;
  const buf = new ArrayBuffer(84 + nt * 50);
  const view = new DataView(buf);
  const header = `Bowshock ${name} watertight STL`.slice(0, 80);
  for (let i = 0; i < 80; i++) view.setUint8(i, i < header.length ? header.charCodeAt(i) : 0);
  view.setUint32(80, nt, true);
  let o = 84;
  for (let t = 0; t < nt; t++) {
    const a = get(mesh, mesh.indices[t * 3]);
    const b = get(mesh, mesh.indices[t * 3 + 1]);
    const c = get(mesh, mesh.indices[t * 3 + 2]);
    const n = vnorm(vcross(vsub(b, a), vsub(c, a)));
    view.setFloat32(o, n[0], true);
    view.setFloat32(o + 4, n[1], true);
    view.setFloat32(o + 8, n[2], true);
    o += 12;
    for (const p of [a, b, c]) {
      view.setFloat32(o, p[0], true);
      view.setFloat32(o + 4, p[1], true);
      view.setFloat32(o + 8, p[2], true);
      o += 12;
    }
    view.setUint16(o, 0, true);
    o += 2;
  }
  return buf;
}
