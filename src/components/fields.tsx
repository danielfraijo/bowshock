import { Slider } from "@/components/ui/slider";

export function NumberField({
  label,
  value,
  onChange,
  min,
  max,
  step,
  unit,
  digits = 2,
}: {
  label: string;
  value: number;
  onChange: (v: number) => void;
  min: number;
  max: number;
  step: number;
  unit?: string;
  digits?: number;
}) {
  return (
    <label className="block">
      <span className="mb-1 flex items-baseline justify-between">
        <span className="text-xs font-medium tracking-wide text-muted">{label}</span>
        <span className="font-mono text-xs tabular-nums text-fg">
          {value.toFixed(digits)}
          {unit ? <span className="ml-1 text-subtle">{unit}</span> : null}
        </span>
      </span>
      <Slider min={min} max={max} step={step} value={[value]} onValueChange={(v) => onChange(v[0] ?? value)} />
    </label>
  );
}

export function Stat({ k, v, ok }: { k: string; v: string; ok?: boolean }) {
  return (
    <div className="flex items-baseline justify-between gap-3 border-b border-border/80 py-1.5 last:border-0">
      <span className="text-xs text-muted">{k}</span>
      <span className={`font-mono text-xs tabular-nums ${ok === false ? "text-warn" : ok === true ? "text-ok" : "text-fg"}`}>
        {v}
      </span>
    </div>
  );
}

export function Seg<T extends string>({
  value,
  onChange,
  options,
}: {
  value: T;
  onChange: (v: T) => void;
  options: { id: T; label: string }[];
}) {
  return (
    <div className="grid gap-1.5" style={{ gridTemplateColumns: `repeat(${options.length}, minmax(0, 1fr))` }}>
      {options.map((o) => (
        <button
          key={o.id}
          type="button"
          onClick={() => onChange(o.id)}
          className={`h-9 rounded-sm text-xs ${value === o.id ? "bg-accent text-accent-fg" : "bg-surface-2 text-muted"}`}
        >
          {o.label}
        </button>
      ))}
    </div>
  );
}

export function Formula({ name, expr, note }: { name: string; expr: string; note?: string }) {
  return (
    <div className="rounded-sm bg-surface-2 px-3 py-2">
      <p className="text-[10px] font-medium tracking-[0.14em] text-subtle uppercase">{name}</p>
      <p className="mt-1 font-mono text-[11px] leading-snug text-fg">{expr}</p>
      {note ? <p className="mt-1 text-[11px] leading-snug text-muted">{note}</p> : null}
    </div>
  );
}
