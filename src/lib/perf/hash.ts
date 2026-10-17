/**
 * Stable, allocation-light content hashing used for cache keys.
 * `hashValue` canonicalises plain data (sorted object keys, exact number round-trip strings),
 * `hashFloats` hashes the raw bytes of a typed array. Both return a 53-bit-strength hex string
 * (two independent 32-bit lanes, cyrb53 style), which is ample for LRU sizes of a few thousand.
 */

function mix(h1: number, h2: number, ch: number): [number, number] {
  h1 = Math.imul(h1 ^ ch, 2654435761);
  h2 = Math.imul(h2 ^ ch, 1597334677);
  return [h1, h2];
}

function finish(h1: number, h2: number): string {
  h1 = Math.imul(h1 ^ (h1 >>> 16), 2246822507) ^ Math.imul(h2 ^ (h2 >>> 13), 3266489909);
  h2 = Math.imul(h2 ^ (h2 >>> 16), 2246822507) ^ Math.imul(h1 ^ (h1 >>> 13), 3266489909);
  return (h2 >>> 0).toString(16).padStart(8, "0") + (h1 >>> 0).toString(16).padStart(8, "0");
}

export class Hasher {
  private h1 = 0xdeadbeef;
  private h2 = 0x41c6ce57;
  private n = 0;
  int(v: number): this {
    [this.h1, this.h2] = mix(this.h1, this.h2, v | 0);
    this.n++;
    return this;
  }
  /** Exact: hashes the IEEE-754 bit pattern (so 0 and -0 differ, NaN is canonical). */
  num(v: number): this {
    if (v !== v) return this.int(0x7ff80000).int(0);
    scratch[0] = v;
    return this.int(scratch32[0]).int(scratch32[1]);
  }
  str(s: string): this {
    for (let i = 0; i < s.length; i++) this.int(s.charCodeAt(i));
    return this.int(s.length);
  }
  floats(a: ArrayLike<number> & { length: number }): this {
    for (let i = 0; i < a.length; i++) this.num(a[i]);
    return this.int(a.length);
  }
  /** Canonical walk over plain data: objects (sorted keys), arrays, numbers, strings, booleans, typed arrays. */
  value(v: unknown): this {
    if (v === null || v === undefined) return this.int(v === null ? 1 : 2);
    switch (typeof v) {
      case "number":
        return this.int(3).num(v);
      case "string":
        return this.int(4).str(v);
      case "boolean":
        return this.int(v ? 5 : 6);
      case "object": {
        if (ArrayBuffer.isView(v)) {
          const t = v as unknown as ArrayLike<number> & { length: number };
          return this.int(7).int((v as { BYTES_PER_ELEMENT?: number }).BYTES_PER_ELEMENT ?? 1).floats(t);
        }
        if (Array.isArray(v)) {
          this.int(8);
          for (const x of v) this.value(x);
          return this.int(v.length);
        }
        const o = v as Record<string, unknown>;
        const keys = Object.keys(o).sort();
        this.int(9);
        let used = 0;
        for (const k of keys) {
          if (o[k] === undefined) continue;
          this.str(k).value(o[k]);
          used++;
        }
        return this.int(used);
      }
      default:
        return this.int(10);
    }
  }
  digest(): string {
    return finish(this.h1, this.h2) + this.n.toString(36);
  }
}

const scratch = new Float64Array(1);
const scratch32 = new Uint32Array(scratch.buffer);

export function hashValue(v: unknown): string {
  return new Hasher().value(v).digest();
}

export function hashFloats(a: ArrayLike<number> & { length: number }): string {
  return new Hasher().floats(a).digest();
}
