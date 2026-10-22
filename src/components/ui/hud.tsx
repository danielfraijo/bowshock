import * as React from "react";
import { cn } from "@/lib/utils";

/** Big numeric HUD readout: small-caps label, large tabular value, aligned unit. */
export function Readout({
  label,
  value,
  unit,
  size = "md",
  tone = "data",
  className,
}: {
  label: string;
  value: React.ReactNode;
  unit?: string;
  size?: "sm" | "md";
  tone?: "data" | "accent" | "warn";
  className?: string;
}) {
  return (
    <div className={cn("hud-readout", className)}>
      <span className="hud-label">{label}</span>
      <span
        className={cn(
          "hud-readout-value",
          size === "sm" && "hud-readout-value--sm",
          tone === "accent" && "hud-readout-value--accent",
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
 * Viewport chrome: corner frame only. No glyph rain and no scan or tick
 * overlays — those sit on the vehicle and hide the shape.
 */
export function ViewportChrome() {
  return (
    <div aria-hidden className="pointer-events-none absolute inset-0">
      <div className="hud-frame" />
      <div className="hud-triad-slot" />
    </div>
  );
}
