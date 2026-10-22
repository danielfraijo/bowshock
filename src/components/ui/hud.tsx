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

const RAIN = [
  "0アイ1ウエAオカ7キクFケコ1サシ0セソ1タチ",
  "10シス0セソAタチ1ツテ7トナ0ニヌ1ネノAハヒ",
  "F0フヘ1ホマ0ミムAメモ1ヤユ0ヨラ7リルAレロ",
  "01ワヲ1ンア0イウAエオ1カキ0クケ7コサAシス",
  "7Aセソ0タチ1ツテAトナ0ニヌ1ネノ0ハヒAフヘ",
  "0Fホマ1ミム0メモAヤユ1ヨラ0リル7レロAワヲ",
  "1Aンア0イウ1エオAカキ0クケ1コサ0シス7セソ",
  "A0タチ1ツテ0トナAニヌ1ネノ0ハヒ1フヘ0ホマ",
  "01ミムAメモ0ヤユ1ヨラAリル0レロ1ワヲ0ンア",
  "7Fイウ0エオ1カキAクケ0コサ1シス0セソAタチ",
  "0Aツテ1トナ0ニヌAネノ1ハヒ0フヘ1ホマ0ミム",
  "10メモAヤユ0ヨラ1リルAレロ0ワヲ1ンア0イウ",
  "FAエオ0カキ1クケAコサ0シス1セソ0タチAツテ",
  "01トナAニヌ0ネノ1ハヒAフヘ0ホマ1ミム0メモ",
  "7Aヤユ0ヨラ1リルAレロ0ワヲ1ンア0イウ1エオ",
  "0FカキAクケ0コサ1シスAセソ0タチ1ツテ0トナ",
];

function MatrixRain() {
  return (
    <div className="hud-matrix">
      {RAIN.map((col, i) => (
        <span key={i}>
          <b>{col.slice(0, 1)}</b>
          {col.slice(1)}
          {col}
        </span>
      ))}
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
      <MatrixRain />
      <div className="hud-scan" />
      <div className="hud-frame" />
      <div className="hud-ticks-x" />
      <div className="hud-triad-slot" />
    </div>
  );
}
