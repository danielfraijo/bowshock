import * as React from "react";
import { cn } from "@/lib/utils";

/** Big numeric HUD readout: small-caps label, large tabular value, aligned unit. */
export function Readout({
  label,
  value,
  unit,
  size = "md",
  tone = "accent",
  className,
}: {
  label: string;
  value: React.ReactNode;
  unit?: string;
  size?: "sm" | "md";
  tone?: "accent" | "warn";
  className?: string;
}) {
  return (
    <div className={cn("hud-readout", className)}>
      <span className="hud-label">{label}</span>
      <span
        className={cn(
          "hud-readout-value",
          size === "sm" && "hud-readout-value--sm",
          tone === "warn" && "hud-readout-value--warn",
        )}
      >
        {value}
        {unit ? <span className="hud-readout-unit">{unit}</span> : null}
      </span>
    </div>
  );
}

/**
 * Decorative viewport chrome: corner brackets, edge tick rulers, scanline sweep and an
 * axis-triad slot. Purely visual (aria-hidden, pointer-events: none) — it never blocks the 3D view.
 */
export function ViewportChrome() {
  return (
    <div aria-hidden className="pointer-events-none absolute inset-0">
      <div className="hud-scan" />
      <div className="hud-frame" />
      <div className="hud-ticks-x" />
      <div className="hud-triad-slot" />
    </div>
  );
}
