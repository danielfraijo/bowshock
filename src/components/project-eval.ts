// [p7-project] Metric evaluator for Projects → Compare.
import { evaluateDesign } from "@/lib/waverider/mdo";
import { safeBuild } from "@/lib/waverider/progressive-study";
import type { DesignParams } from "@/lib/waverider/types";
import type { VariantMetrics } from "@/lib/project/index";

/** Same cheap panel evaluation as Iterate → Compare (mdo.ts evaluateDesign); Sref/η from one extra build. */
export function evaluateVariant(p: DesignParams): VariantMetrics {
  const ev = evaluateDesign(p);
  if (ev.error) return { ...ev.metrics, sRef: 0, volEff: 0, error: ev.error };
  const b = safeBuild(p).built;
  const sRef = b.aero.planformArea;
  return { ...ev.metrics, sRef, volEff: sRef > 0 ? Math.pow(Math.max(ev.metrics.volume, 0), 2 / 3) / sRef : 0 };
}

