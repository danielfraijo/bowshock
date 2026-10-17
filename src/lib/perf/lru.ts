/** Size-bounded LRU map. Bound by entry count and (optionally) an approximate byte budget. */
export class LRU<V> {
  private m = new Map<string, { v: V; bytes: number }>();
  private total = 0;
  hits = 0;
  misses = 0;
  constructor(
    readonly maxEntries: number,
    readonly maxBytes = Infinity,
  ) {}
  get size() {
    return this.m.size;
  }
  get bytes() {
    return this.total;
  }
  has(key: string) {
    return this.m.has(key);
  }
  get(key: string): V | undefined {
    const e = this.m.get(key);
    if (!e) {
      this.misses++;
      return undefined;
    }
    this.hits++;
    this.m.delete(key);
    this.m.set(key, e);
    return e.v;
  }
  peek(key: string): V | undefined {
    return this.m.get(key)?.v;
  }
  set(key: string, v: V, bytes = 0): V {
    const old = this.m.get(key);
    if (old) {
      this.total -= old.bytes;
      this.m.delete(key);
    }
    this.m.set(key, { v, bytes });
    this.total += bytes;
    while (this.m.size > this.maxEntries || (this.total > this.maxBytes && this.m.size > 1)) {
      const first = this.m.keys().next().value as string | undefined;
      if (first === undefined) break;
      this.total -= this.m.get(first)!.bytes;
      this.m.delete(first);
    }
    return v;
  }
  delete(key: string) {
    const e = this.m.get(key);
    if (e) this.total -= e.bytes;
    return this.m.delete(key);
  }
  clear() {
    this.m.clear();
    this.total = 0;
    this.hits = 0;
    this.misses = 0;
  }
  stats() {
    return { size: this.m.size, bytes: this.total, hits: this.hits, misses: this.misses };
  }
}
