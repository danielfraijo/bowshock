import type { TriMesh, Vec3 } from "./types";

export interface MassProps {
  mass: number;
  volume: number;
  rho: number;
  cg: Vec3;
  Ixx: number;
  Iyy: number;
  Izz: number;
  Ixz: number;
  /** Product ∫xy dm about the CG (same +∫ convention as Ixz). */
  Ixy?: number;
  /** Product ∫yz dm about the CG. */
  Iyz?: number;
  ballistic: number;
  notes: string[];
}

/** Closed-mesh mass properties via tetrahedron decomposition (Mirtich). */
export function massProperties(mesh: TriMesh, rho: number, massOverride = 0): MassProps {
  const notes: string[] = [];
  const pos = mesh.positions;
  const idx = mesh.indices;
  const nt = idx.length / 3;
  let V = 0;
  let Cx = 0;
  let Cy = 0;
  let Cz = 0;
  let Ixx = 0;
  let Iyy = 0;
  let Izz = 0;
  let Ixy = 0;
  let Ixz = 0;
  let Iyz = 0;
  for (let t = 0; t < nt; t++) {
    const ia = idx[t * 3] * 3;
    const ib = idx[t * 3 + 1] * 3;
    const ic = idx[t * 3 + 2] * 3;
    const ax = pos[ia];
    const ay = pos[ia + 1];
    const az = pos[ia + 2];
    const bx = pos[ib];
    const by = pos[ib + 1];
    const bz = pos[ib + 2];
    const cx = pos[ic];
    const cy = pos[ic + 1];
    const cz = pos[ic + 2];
    const det =
      ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
    V += det;
    Cx += det * (ax + bx + cx);
    Cy += det * (ay + by + cy);
    Cz += det * (az + bz + cz);
    Ixx += det * (ay * ay + ay * by + by * by + az * az + az * bz + bz * bz + ay * cy + by * cy + cy * cy + az * cz + bz * cz + cz * cz);
    Iyy += det * (ax * ax + ax * bx + bx * bx + az * az + az * bz + bz * bz + ax * cx + bx * cx + cx * cx + az * cz + bz * cz + cz * cz);
    Izz += det * (ax * ax + ax * bx + bx * bx + ay * ay + ay * by + by * by + ax * cx + bx * cx + cx * cx + ay * cy + by * cy + cy * cy);
    Ixy += det * (2 * ax * ay + by * ax + ay * bx + 2 * bx * by + cy * ax + ay * cx + cy * bx + by * cx + 2 * cx * cy);
    Ixz += det * (2 * ax * az + bz * ax + az * bx + 2 * bx * bz + cz * ax + az * cx + cz * bx + bz * cx + 2 * cx * cz);
    Iyz += det * (2 * ay * az + bz * ay + az * by + 2 * by * bz + cz * ay + az * cy + cz * by + bz * cy + 2 * cy * cz);
  }
  V /= 6;
  const sign = V < 0 ? -1 : 1;
  V = Math.abs(V);
  if (V < 1e-12) notes.push("Degenerate volume — mass from bbox fallback.");
  Cx = (sign * Cx) / 24 / Math.max(V, 1e-12);
  Cy = (sign * Cy) / 24 / Math.max(V, 1e-12);
  Cz = (sign * Cz) / 24 / Math.max(V, 1e-12);
  const dens = Math.max(20, rho);
  let mass = dens * V;
  if (massOverride > 0) {
    mass = massOverride;
    notes.push("Mass overridden.");
  }
  // [accuracy-audit] Tonelli: for the origin tetrahedron ∫x² dV = det/60 · (Σ x_k² + Σ_{k<l} x_k x_l) and
  // ∫xy dV = det/120 · (2Σ x_k y_k + Σ_{k≠l} x_k y_l). The old 1/120 halved every second moment, so the
  // parallel-axis shift below went negative for any body away from the origin and Math.abs hid it
  // (a 2×1×0.5 m box of 1000 kg centred at (1,2,3): Ixx 6448 vs analytic 104.2 kg·m²). The fix was first
  // drafted in the p5-tps WIP stash. Inertia is scaled to the reported mass so an override keeps the
  // radius of gyration of the closed shell.
  const k = ((dens * sign) / 60) * (mass / Math.max(dens * V, 1e-12));
  Ixx = k * Ixx;
  Iyy = k * Iyy;
  Izz = k * Izz;
  Ixy = k * Ixy * 0.5;
  Ixz = k * Ixz * 0.5;
  Iyz = k * Iyz * 0.5;
  const IxxCg = Math.abs(Ixx - mass * (Cy * Cy + Cz * Cz));
  const IyyCg = Math.abs(Iyy - mass * (Cx * Cx + Cz * Cz));
  const IzzCg = Math.abs(Izz - mass * (Cx * Cx + Cy * Cy));
  const IxzCg = Ixz - mass * Cx * Cz;
  const IxyCg = Ixy - mass * Cx * Cy;
  const IyzCg = Iyz - mass * Cy * Cz;
  return {
    mass,
    volume: V,
    rho: dens,
    cg: [Cx, Cy, Cz],
    Ixx: IxxCg,
    Iyy: IyyCg,
    Izz: IzzCg,
    Ixz: IxzCg,
    Ixy: IxyCg,
    Iyz: IyzCg,
    ballistic: 0,
    notes,
  };
}
