/**
 * Inlet operability map for the air-breathing families.
 *
 * Each cell reruns the existing 1-D cycle (`ramjetCycle`) at a flight Mach
 * and equivalence ratio. "Started" is the Kantrowitz comparison already
 * computed in `internal.ts`: geometric contraction versus the Kantrowitz
 * self-start limit. It is a 1-D start screen, not a bleed/unstart CFD.
 */
import { atmosphere } from "./atmosphere";
import { ramjetCycle } from "./propulsion";
import type { DesignParams } from "./types";

export interface OperabilityRow {
  mach: number;
  phi: number;
  started: boolean;
  kantrowitz: number;
  contraction: number;
  thrust: number;
  isp: number;
  kind: string;
}

export function inletOperability(params: DesignParams, machs: number[], phis: number[]): OperabilityRow[] {
  const rows: OperabilityRow[] = [];
  for (const mach of machs) {
    for (const phi of phis) {
      const p: DesignParams = { ...params, lockFlight: false, flightMach: mach, phi };
      const atm = atmosphere(p.altKm, mach, p.gamma);
      const prop = ramjetCycle(p, atm);
      rows.push({
        mach,
        phi,
        started: prop?.internal?.started ?? false,
        kantrowitz: prop?.internal?.kantrowitz ?? 0,
        contraction: prop?.internal?.contraction ?? 0,
        thrust: prop?.thrust ?? 0,
        isp: prop?.isp ?? 0,
        kind: prop?.kind ?? "none",
      });
    }
  }
  return rows;
}
