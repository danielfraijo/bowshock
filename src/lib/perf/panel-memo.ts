/**
 * Memo for panelAero(). A full study calls panelAero ~25 times (derivative stencils, trim iterations,
 * the 6-point polar, rate pairs) and the euler method calls it again for its CBAERO base; consecutive
 * studies repeat many of those exact calls (the polar alphas never move when only alpha0 moves, the
 * stencil overlaps). The key is the exact content of every input, so a hit is bit-identical to a
 * recompute and any change in params / atmosphere / flow angles / reference sizes / rates / mesh misses.
 *
 * Mesh identity: meshes are immutable once built (every consumer treats them read-only), so a WeakMap
 * id per TriMesh object is a sound key. Returned PanelAero objects are shared: callers must not mutate
 * them (the in-repo callers copy before editing; see perf.test.ts).
 */
import { Hasher } from "./hash";
import { KERNEL_VERSION, kernelFlags } from "./euler-kernel";
import { LRU } from "./lru";
import { withoutCosmetic } from "./cosmetic";

const ids = new WeakMap<object, number>();
let nextId = 1;
function idOf(o: object): number {
  let v = ids.get(o);
  if (v === undefined) ids.set(o, (v = nextId++));
  return v;
}

export const panelCache = new LRU<unknown>(160, 64 * 1024 * 1024);
/** `verify`: on every hit, recompute and throw when the memoised value differs (debug / tests). */
export const panelMemoFlags: { enabled: boolean; verify: boolean } = { enabled: true, verify: false };

function bytesOf(r: unknown): number {
  let n = 2048;
  if (r && typeof r === "object") {
    for (const v of Object.values(r as Record<string, unknown>)) if (ArrayBuffer.isView(v)) n += v.byteLength;
  }
  return n;
}

export function memoPanel<T>(
  mesh: object,
  params: unknown,
  atm: unknown,
  alphaDeg: number,
  betaDeg: number,
  sRef: number,
  lRef: number,
  rates: unknown,
  compute: () => T,
): T {
  if (!panelMemoFlags.enabled) return compute();
  const g = kernelFlags.grid;
  const key = new Hasher()
    .int(idOf(mesh))
    .str(KERNEL_VERSION)
    .str(kernelFlags.reference ? "r" : "f")
    .str(kernelFlags.quality)
    .int(g ? g.ni * 10007 + g.nj * 101 + g.iters : 0)
    .num(alphaDeg)
    .num(betaDeg)
    .num(sRef)
    .num(lRef)
    .value(rates ?? null)
    .value(atm)
    .value(withoutCosmetic(params as object))
    .digest();
  const hit = panelCache.get(key);
  if (hit) {
    if (panelMemoFlags.verify) {
      const fresh = compute() as unknown as Record<string, unknown>;
      const old = hit as Record<string, unknown>;
      for (const k of Object.keys(fresh)) {
        const a = fresh[k];
        const b = old[k];
        if (typeof a === "number" && a !== b && !(Number.isNaN(a) && Number.isNaN(b as number))) throw new Error(`panel memo stale: ${k} ${String(b)} vs fresh ${a} (alpha ${alphaDeg}, beta ${betaDeg})`);
      }
    }
    return hit as T;
  }
  const out = compute();
  panelCache.set(key, out, bytesOf(out));
  return out;
}
