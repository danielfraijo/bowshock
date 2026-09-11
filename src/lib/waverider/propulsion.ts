import type { DesignParams, FuelKind } from "./types";
import type { Atmosphere } from "./atmosphere";
import { internalCycle, type InternalResult } from "./internal";

export type { InternalResult, Station, ShockJump } from "./internal";

export interface PropResult {
  kind: "none" | "ramjet" | "scram-mode" | "scramjet" | "inlet";
  capture: number;
  mdot: number;
  mdotFuel: number;
  Tt0: number;
  pt0: number;
  M2: number;
  p2p0: number;
  Tt4: number;
  Te: number;
  Ue: number;
  Ue0: number;
  thrust: number;
  isp: number;
  tsfc: number;
  phi: number;
  fuel: FuelKind;
  erGeom: number;
  etaThermal: number;
  notes: string[];
  internal: InternalResult | null;
}

export function ramjetCycle(params: DesignParams, atm: Atmosphere): PropResult | null {
  const ic = internalCycle(params, atm);
  if (!ic) return null;
  return {
    kind: ic.kind === "scramjet" ? "scram-mode" : ic.kind,
    capture: ic.capture,
    mdot: ic.mdot,
    mdotFuel: ic.mdotFuel,
    Tt0: ic.Tt0,
    pt0: ic.pt0,
    M2: ic.M2,
    p2p0: ic.p2p0,
    Tt4: ic.Tt4,
    Te: ic.Te,
    Ue: ic.Ue,
    Ue0: ic.Ue0,
    thrust: ic.thrust,
    isp: ic.isp,
    tsfc: ic.tsfc,
    phi: ic.phi,
    fuel: ic.fuel,
    erGeom: ic.erGeom,
    etaThermal: ic.etaThermal,
    notes: ic.notes,
    internal: ic,
  };
}
