/** DesignParams fields that never influence any computed number (only labels / export units). */
export const COSMETIC_FIELDS = ["name", "unit"] as const;

export function withoutCosmetic<T extends object>(p: T): Record<string, unknown> {
  const o: Record<string, unknown> = {};
  for (const k of Object.keys(p)) if (!(COSMETIC_FIELDS as readonly string[]).includes(k)) o[k] = (p as Record<string, unknown>)[k];
  return o;
}
