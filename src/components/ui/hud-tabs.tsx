import * as React from "react";

/**
 * Two-tier tab bar: primary = task group (Design / Analyze / Dynamics / Verify / Output),
 * secondary = the views inside the active group. All tabs of the active group are always
 * visible (no horizontal scrolling), the active group is derived from the active tab, and
 * the bar follows the WAI-ARIA tabs pattern (roving tabindex, arrow/Home/End keys).
 */
export type HudTabDef<T extends string = string> = { id: T; label: string };

const GROUPS: { id: string; label: string; hint: string; tabs: string[] }[] = [
  { id: "design", label: "Design", hint: "Geometry and flight condition", tabs: ["geom", "flight"] },
  { id: "analyze", label: "Analyze", hint: "Aero, heating, frontier, stability, internal flow", tabs: ["aero", "heat", "frontier", "stab", "cycle", "shocks"] },
  { id: "dynamics", label: "Dynamics", hint: "6-DOF and trajectory", tabs: ["sixdof", "traj"] },
  { id: "verify", label: "Verify", hint: "DATCOM and design checks", tabs: ["datcom", "checks"] },
  { id: "output", label: "Output", hint: "Data pack and CAD export", tabs: ["data", "cad"] },
];

export function HudTabs<T extends string>({
  tabs,
  value,
  onChange,
}: {
  tabs: HudTabDef<T>[];
  value: T;
  onChange: (id: T) => void;
}) {
  const present = new Set<string>(tabs.map((t) => t.id));
  const groups = GROUPS.map((g) => ({ ...g, items: tabs.filter((t) => g.tabs.includes(t.id)) })).filter((g) => g.items.length);
  const active = groups.find((g) => g.items.some((t) => t.id === value)) ?? groups[0];
  const lastInGroup = React.useRef<Record<string, T>>({});
  if (active && present.has(value)) lastInGroup.current[active.id] = value;

  const pick = (g: (typeof groups)[number]): T => {
    const last = lastInGroup.current[g.id];
    return last && g.items.some((t) => t.id === last) ? last : g.items[0]!.id;
  };
  const focusTab = (root: HTMLElement | null, id: string) => {
    root?.querySelector<HTMLElement>(`[data-tab-id="${id}"]`)?.focus();
  };
  const onKeys = (e: React.KeyboardEvent<HTMLDivElement>, ids: string[], activeId: string | undefined, select: (id: string) => void) => {
    const i = activeId ? ids.indexOf(activeId) : 0;
    let n = -1;
    if (e.key === "ArrowRight" || e.key === "ArrowDown") n = (i + 1) % ids.length;
    else if (e.key === "ArrowLeft" || e.key === "ArrowUp") n = (i - 1 + ids.length) % ids.length;
    else if (e.key === "Home") n = 0;
    else if (e.key === "End") n = ids.length - 1;
    if (n < 0) return;
    e.preventDefault();
    const root = e.currentTarget;
    select(ids[n]!);
    requestAnimationFrame(() => focusTab(root, ids[n]!));
  };

  return (
    <div className="hud-tabs shrink-0">
      <div
        className="hud-tab-primary"
        role="tablist"
        aria-label="Workspace section"
        onKeyDown={(e) =>
          onKeys(
            e,
            groups.map((g) => g.id),
            active?.id,
            (gid) => {
              const g = groups.find((x) => x.id === gid);
              if (g) onChange(pick(g));
            },
          )
        }
      >
        {groups.map((g) => (
          <button
            key={g.id}
            type="button"
            role="tab"
            data-tab-id={g.id}
            aria-selected={g === active}
            tabIndex={g === active ? 0 : -1}
            title={g.hint}
            onClick={() => onChange(pick(g))}
          >
            {g.label}
          </button>
        ))}
      </div>
      {active ? (
        <div
          className="hud-tab-secondary"
          role="tablist"
          aria-label={`${active.label} views`}
          onKeyDown={(e) =>
            onKeys(
              e,
              active.items.map((t) => t.id),
              value,
              (id) => onChange(id as T),
            )
          }
        >
          {active.items.map((t) => (
            <button
              key={t.id}
              type="button"
              role="tab"
              data-tab-id={t.id}
              aria-selected={t.id === value}
              tabIndex={t.id === value ? 0 : -1}
              onClick={() => onChange(t.id)}
            >
              {t.label}
            </button>
          ))}
        </div>
      ) : null}
    </div>
  );
}
