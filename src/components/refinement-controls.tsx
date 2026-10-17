/**
 * Mesh refinement selector for the Geom tab: coarse / medium / fine / custom.
 * Surface counts are the existing Streamwise / Spanwise sliders (params.nx / params.ny);
 * this adds LE clustering, the Euler volume grid, clustering strengths and a cost readout.
 * `medium` is the legacy behaviour (40x28 surface, 72x24x280 Euler).
 */
import { NumberField, Seg } from "@/components/fields";
import { Switch } from "@/components/ui/switch";
import {
  REFINEMENT_LIMITS,
  describeCost,
  estimateCost,
  presetSettings,
  refinementOf,
  surfaceWarnings,
  validateRefinement,
  type RefinementPreset,
  type RefinementSettings,
} from "@/lib/waverider/refinement";
import type { DesignParams } from "@/lib/waverider/types";

export function RefinementControls({
  params,
  patch,
  setDragging,
}: {
  params: DesignParams;
  patch: (p: Partial<DesignParams>) => void;
  setDragging: (v: boolean) => void;
}) {
  const cur = refinementOf(params);
  const preset: RefinementPreset = cur.preset;
  const L = REFINEMENT_LIMITS;
  const cost = estimateCost(cur);
  const warn = [...surfaceWarnings(cur)];
  if (params.aeroMethod !== "euler")
    warn.push(
      "Volume settings apply to the Euler method (Aero tab). Newtonian / tangent / CBAERO ignore them.",
    );
  const named = preset !== "custom" ? presetSettings(preset) : null;
  if (named && (named.surface.nx !== params.nx || named.surface.ny !== params.ny)) {
    warn.push(
      `Surface points (${params.nx} x ${params.ny}) differ from the ${preset} preset (${named.surface.nx} x ${named.surface.ny}) because the sliders above were moved.`,
    );
  }

  const pick = (id: RefinementPreset) => {
    if (id === "custom") {
      patch({ refinement: { ...cur, preset: "custom" } });
      return;
    }
    const s = presetSettings(id);
    patch({ nx: s.surface.nx, ny: s.surface.ny, refinement: s });
  };

  const setCustom = (
    volume: Partial<RefinementSettings["volume"]>,
    surface: Partial<RefinementSettings["surface"]> = {},
  ) => {
    const v = validateRefinement({
      preset: "custom",
      surface: { ...cur.surface, ...surface },
      volume: { ...cur.volume, ...volume },
    });
    patch({ refinement: v.settings });
  };
  const checked = validateRefinement({
    preset: "custom",
    surface: cur.surface,
    volume: cur.volume,
  });
  if (preset === "custom") warn.push(...checked.warnings);

  const drag = { onDragStart: () => setDragging(true), onDragEnd: () => setDragging(false) };
  return (
    <div className="space-y-3 rounded-sm border border-border/80 p-3">
      <div>
        <span className="mb-2 block text-xs font-medium text-muted">Mesh refinement</span>
        <Seg
          value={preset}
          onChange={pick}
          options={[
            { id: "coarse", label: "Coarse" },
            { id: "medium", label: "Medium" },
            { id: "fine", label: "Fine" },
            { id: "custom", label: "Custom" },
          ]}
        />
      </div>
      <p className="text-[11px] leading-snug text-subtle">{describeCost(cur)}</p>
      {!cost.withinCaps ? (
        <p className="text-[11px] text-warn">Over the size cap - values will be clamped.</p>
      ) : null}
      {preset === "custom" ? (
        <div className="space-y-3">
          <p className="text-[11px] text-subtle">
            Surface counts: Streamwise / Spanwise points above.
          </p>
          <NumberField
            label="LE / nose clustering"
            value={cur.surface.leClustering}
            min={L.surface.leClustering[0]}
            max={L.surface.leClustering[1]}
            step={0.05}
            digits={2}
            {...drag}
            onChange={(leClustering) => setCustom({}, { leClustering })}
          />
          <NumberField
            label="Euler cells streamwise"
            value={cur.volume.nStream}
            min={L.volume.nStream[0]}
            max={L.volume.nStream[1]}
            step={1}
            digits={0}
            {...drag}
            onChange={(nStream) => setCustom({ nStream: Math.round(nStream) })}
          />
          <NumberField
            label="Euler cells across shock layer"
            value={cur.volume.nCross}
            min={L.volume.nCross[0]}
            max={L.volume.nCross[1]}
            step={1}
            digits={0}
            {...drag}
            onChange={(nCross) => setCustom({ nCross: Math.round(nCross) })}
          />
          <NumberField
            label="Euler iterations"
            value={cur.volume.iters}
            min={L.volume.iters[0]}
            max={L.volume.iters[1]}
            step={10}
            digits={0}
            {...drag}
            onChange={(iters) => setCustom({ iters: Math.round(iters) })}
          />
          <NumberField
            label="Wall clustering"
            value={cur.volume.wallClustering}
            min={L.volume.wallClustering[0]}
            max={L.volume.wallClustering[1]}
            step={0.5}
            digits={1}
            {...drag}
            onChange={(wallClustering) => setCustom({ wallClustering })}
          />
          <NumberField
            label="Shock clustering"
            value={cur.volume.shockClustering}
            min={L.volume.shockClustering[0]}
            max={L.volume.shockClustering[1]}
            step={0.5}
            digits={1}
            {...drag}
            onChange={(shockClustering) => setCustom({ shockClustering })}
          />
          <div className="flex items-center justify-between gap-3 pt-1">
            <span className="text-xs font-medium text-muted">Adaptive re-cluster (2 solves)</span>
            <Switch
              checked={cur.volume.adaptive}
              onCheckedChange={(adaptive) => setCustom({ adaptive })}
            />
          </div>
        </div>
      ) : null}
      {warn.map((w) => (
        <p key={w} className="text-[11px] leading-snug text-warn">
          {w}
        </p>
      ))}
    </div>
  );
}
