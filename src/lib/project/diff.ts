/** [p7-project] Param-level diff between two project states (design only; UI/view state is ignored). */
import { canonicalJson, designOf, isObj, type ProjectState } from "./schema.ts";

export interface ParamChange {
  /** Dotted path, e.g. "params.mach", "params.refinement.preset", "aeroDb.spec.alphaDeg". */
  path: string;
  kind: "added" | "removed" | "changed";
  from?: unknown;
  to?: unknown;
}

/** Arrays and aero-database results are compared as a whole (a 10^5-point table is one change, not 10^5). */
function walk(a: unknown, b: unknown, path: string, out: ParamChange[]) {
  if (canonicalJson(a) === canonicalJson(b)) return;
  const opaque = path === "aeroDb.database";
  if (!opaque && isObj(a) && isObj(b)) {
    const keys = [...new Set([...Object.keys(a), ...Object.keys(b)])].sort();
    for (const k of keys) {
      const p = path ? `${path}.${k}` : k;
      if (!(k in a) || a[k] === undefined) {
        if (b[k] !== undefined) out.push({ path: p, kind: "added", to: b[k] });
      } else if (!(k in b) || b[k] === undefined) out.push({ path: p, kind: "removed", from: a[k] });
      else walk(a[k], b[k], p, out);
    }
    return;
  }
  if (a === null || a === undefined) out.push({ path, kind: "added", to: b });
  else if (b === null || b === undefined) out.push({ path, kind: "removed", from: a });
  else out.push({ path, kind: "changed", from: a, to: b });
}

export function diffStates(a: ProjectState, b: ProjectState): ParamChange[] {
  const out: ParamChange[] = [];
  walk(designOf(a), designOf(b), "", out);
  return out;
}

function short(v: unknown): string {
  if (typeof v === "number") return Number.isInteger(v) ? String(v) : String(Number(v.toPrecision(4)));
  if (typeof v === "string") return v.length > 18 ? `${v.slice(0, 17)}…` : v;
  if (typeof v === "boolean") return v ? "on" : "off";
  if (v === null || v === undefined) return "—";
  return Array.isArray(v) ? `[${v.length}]` : "{…}";
}

export function describeChange(c: ParamChange): string {
  const name = c.path.replace(/^params\./, "");
  if (c.path === "aeroDb.database") return c.kind === "removed" ? "aero DB cleared" : "aero DB results";
  if (c.path === "aeroDb") return c.kind === "removed" ? "aero DB removed" : c.kind === "added" ? "aero DB added" : "aero DB";
  if (c.path === "aeroDb.builtFor") return "aero DB re-tagged";
  if (c.path.startsWith("aeroDb.spec.") && c.kind === "changed") return `DB ${c.path.slice(12)} ${short(c.from)}→${short(c.to)}`;
  if (c.kind === "added") return `+${name}=${short(c.to)}`;
  if (c.kind === "removed") return `−${name}`;
  return `${name} ${short(c.from)}→${short(c.to)}`;
}

export function summarizeDiff(changes: ParamChange[], max = 3): string {
  if (!changes.length) return "No design changes";
  const head = changes.slice(0, max).map(describeChange).join(", ");
  return changes.length > max ? `${head} (+${changes.length - max} more)` : head;
}
