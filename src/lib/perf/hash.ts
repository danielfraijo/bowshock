/**
 * Stable, allocation-light content hashing used for cache keys.
 * `hashValue` canonicalises plain data (sorted object keys, exact number round-trip strings),
 * `hashFloats` hashes the raw bytes of a typed array. Both return a 53-bit-strength hex string
 * (two independent 32-bit lanes), which is ample for LRU sizes of a few thousand.
 */

/*
 * Two independent 32-bit lanes (murmur3 body / xxhash32 round), each with a rotate so that differences in the HIGH
 * bits of an input word diffuse into the low bits. (A plain `h = imul(h ^ x, K)` chain - cyrb53's step - keeps a
 * difference confined to the bits it started in; for IEEE-754 words that made e.g. (alpha=-2, beta=0) and
 * (alpha=0, beta=2) collide, which corrupted the panel memo. Regression test: perf.test.ts "hash: no collisions".)
 */
const rotl = (x: number, r: number) => (x << r) | (x >>> (32 - r));

function mix(h1: number, h2: number, ch: number): [number, number] {
  // lane 1: murmur3_32 block
  let k = Math.imul(ch, 0xcc9e2d51);
  k = rotl(k, 15);
  k = Math.imul(k, 0x1b873593);
  h1 ^= k;
  h1 = rotl(h1, 13);
  h1 = (Math.imul(h1, 5) + 0xe6546b64) | 0;
  // lane 2: xxhash32 round
  h2 = (h2 + Math.imul(ch, 0x85ebca77)) | 0;
  h2 = rotl(h2, 13);
  h2 = Math.imul(h2, 0x9e3779b1);
  return [h1, h2];
}

function fmix(h: number): number {
  h ^= h >>> 16;
  h = Math.imul(h, 0x85ebca6b);
  h ^= h >>> 13;
  h = Math.imul(h, 0xc2b2ae35);
  h ^= h >>> 16;
  return h;
}

function finish(h1: number, h2: number): string {
  const a = fmix(h1 ^ Math.imul(h2, 0x27d4eb2f));
  const b = fmix(h2 ^ Math.imul(h1, 0x165667b1));
  return (b >>> 0).toString(16).padStart(8, "0") + (a >>> 0).toString(16).padStart(8, "0");
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
