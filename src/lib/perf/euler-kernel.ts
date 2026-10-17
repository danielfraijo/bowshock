/**
 * Rust/wasm Euler kernel front-end: loading (sync from the embedded bytes, or async/streaming from
 * a URL / file), buffer reuse (no per-call allocation after warm-up), and an exact-key LRU of
 * solved wall Cp distributions.
 *
 * The cache key is the exact bit pattern of (Mach, gamma, alpha, grid, wall x/z arrays), so a hit
 * is guaranteed to return the same Float64Array content a fresh solve would, and any change to the
 * geometry or flight condition misses. (The previous cache keyed on rounded Mach/alpha and on four
 * sample values of the wall, which could serve a stale distribution after a small geometry edit.)
 */
import { EULER_WASM_B64 } from "../waverider/euler-wasm";
import { hashFloats, Hasher } from "./hash";
import { LRU } from "./lru";

export interface EulerExports {
  euler_alloc(n: number): number;
  euler_solve(ptr: number, n: number, mach: number, gamma: number, alpha: number): number;
  euler_solve_fast?(ptr: number, n: number, mach: number, gamma: number, alpha: number): number;
  /** this crate's optimised fixed-iteration kernel on a selectable grid (bit-identical to euler_solve at 88x28x42) */
  euler_solve_grid?(ptr: number, n: number, mach: number, gamma: number, alpha: number, ni: number, nj: number, iters: number): number;
  /** mesh-refinement kernel: clustering, convergence stop, grid sequencing, CFL ramp (lib.rs) */
  euler_solve_ex?(
    ptr: number,
    n: number,
    mach: number,
    gamma: number,
    alpha: number,
    ni: number,
    nj: number,
    iters: number,
    wallAmp: number,
    shockAmp: number,
    shockWidth: number,
    tol: number,
    sequence: number,
    cfl: number,
  ): number;
  euler_wedge?(ptr: number, mach: number, thetaRad: number, gamma: number, yTop: number): void;
  memory: WebAssembly.Memory;
}

let exportsRef: EulerExports | null = null;
let scratchPtr = 0;
let scratchCap = 0;

export function wasmBytesFromBase64(b64: string = EULER_WASM_B64): Uint8Array<ArrayBuffer> {
  const bin = atob(b64);
  const bytes = new Uint8Array(new ArrayBuffer(bin.length));
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return bytes;
}

function adopt(instance: WebAssembly.Instance): EulerExports {
  exportsRef = instance.exports as unknown as EulerExports;
  scratchPtr = 0;
  scratchCap = 0;
  return exportsRef;
}

/** Synchronous instantiation from the embedded bytes (small module, ~ms). */
export function eulerSync(): EulerExports {
  if (exportsRef) return exportsRef;
  wasmSource = "embedded";
  return adopt(new WebAssembly.Instance(new WebAssembly.Module(wasmBytesFromBase64()), {}));
}

/**
 * Async instantiation. In a browser/worker pass the URL of `cuspis_euler.wasm` (streaming compile,
 * cacheable, off the main bundle); in Node pass a `file:` URL or omit to use the embedded bytes.
 */
export async function eulerAsync(url?: string | URL): Promise<EulerExports> {
  if (exportsRef) return exportsRef;
  if (url) {
    const href = typeof url === "string" ? url : url.href;
    if (typeof fetch === "function" && /^https?:|^\//.test(href)) {
      // 1) streaming compile (needs Content-Type: application/wasm)
      if (typeof WebAssembly.instantiateStreaming === "function") {
        try {
          const { instance } = await WebAssembly.instantiateStreaming(fetch(href), {});
          if (!exportsRef) wasmSource = "streaming";
          return exportsRef ?? adopt(instance);
        } catch {
          /* wrong MIME type or network error: try the buffered path */
        }
      }
      // 2) buffered compile
      try {
        const res = await fetch(href);
        if (res.ok) {
          const mod = await WebAssembly.compile(await res.arrayBuffer());
          const inst = await WebAssembly.instantiate(mod, {});
          if (!exportsRef) wasmSource = "fetch";
          return exportsRef ?? adopt(inst);
        }
      } catch {
        /* fall through to the embedded copy */
      }
    }
  }
  const mod = await WebAssembly.compile(wasmBytesFromBase64());
  const inst = await WebAssembly.instantiate(mod, {});
  if (!exportsRef) wasmSource = "embedded";
  return exportsRef ?? adopt(inst);
}

/** Test hook: forget the live instance so the next load path can be exercised. */
export function resetEulerForTest() {
  exportsRef = null;
  scratchPtr = 0;
  scratchCap = 0;
  wasmSource = "none";
}

/** How the live instance was created (diagnostics). */
export let wasmSource: "none" | "streaming" | "fetch" | "file" | "embedded" = "none";

export function eulerLoaded(): boolean {
  return exportsRef !== null;
}

function ensureScratch(k: EulerExports, bytes: number): number {
  if (scratchCap < bytes) {
    scratchPtr = k.euler_alloc(bytes);
    scratchCap = bytes;
  }
  return scratchPtr;
}

/**
 * Test / diagnostic switches. `reference` routes every solve through the original
 * (pre-optimisation) wasm entry point; `wedge` enables the wasm 2-D wedge march.
 */
export interface EulerGrid {
  ni: number;
  nj: number;
  iters: number;
}
/** Coarse grid used for drag previews (|dCp| <~ 2e-3 vs the default grid on the default caret, ~8x cheaper). */
export const PREVIEW_GRID: EulerGrid = { ni: 44, nj: 14, iters: 30 };

export const kernelFlags: { reference: boolean; wedge: boolean; grid?: EulerGrid } = { reference: false, wedge: true, grid: undefined };

/** Bumped when the kernel's numerics change; part of every persisted/cached key. */
export const KERNEL_VERSION = "cuspis-euler/fast-1";

/** Exact hash of a wall (x and z arrays). Cheap (~1 us for 60 points), so it is never memoised. */
export function wallKey(x: Float64Array, z: Float64Array): string {
  return hashFloats(x) + hashFloats(z);
}

export const eulerCache = new LRU<Float64Array>(512, 8 * 1024 * 1024);

export function eulerKey(x: Float64Array, z: Float64Array, mach: number, gamma: number, alpha: number, grid?: EulerGrid): string {
  const h = new Hasher().str(KERNEL_VERSION).str(kernelFlags.reference ? "ref" : "fast").num(mach).num(gamma).num(alpha);
  if (grid) h.int(grid.ni).int(grid.nj).int(grid.iters);
  return h.digest() + wallKey(x, z);
}

/** Raw solve with no caching. `reference` forces the original (pre-optimisation) kernel. */
export function solveWallRaw(
  x: Float64Array,
  z: Float64Array,
  mach: number,
  gamma: number,
  alpha: number,
  opts?: { reference?: boolean; grid?: EulerGrid },
): Float64Array {
  const k = eulerSync();
  const n = x.length;
  const ptr = ensureScratch(k, (n * 3 + 16) * 8);
  const view = new Float64Array(k.memory.buffer, ptr, n * 2);
  for (let i = 0; i < n; i++) {
    view[i * 2] = x[i];
    view[i * 2 + 1] = z[i];
  }
  if (opts?.grid && k.euler_solve_grid) {
    const g = opts.grid;
    k.euler_solve_grid(ptr, n, mach, gamma, alpha, g.ni, g.nj, g.iters);
  } else if (!opts?.reference && k.euler_solve_fast) k.euler_solve_fast(ptr, n, mach, gamma, alpha);
  else k.euler_solve(ptr, n, mach, gamma, alpha);
  return new Float64Array(new Float64Array(k.memory.buffer, ptr + n * 16, n));
}

/** Cached wall-Cp solve. The returned array is shared: treat it as read-only. */
export function solveWallCp(x: Float64Array, z: Float64Array, mach: number, gamma: number, alpha: number): Float64Array {
  const key = eulerKey(x, z, mach, gamma, alpha, kernelFlags.grid);
  const hit = eulerCache.get(key);
  if (hit) return hit;
  const out = solveWallRaw(x, z, mach, gamma, alpha, { reference: kernelFlags.reference, grid: kernelFlags.grid });
  return eulerCache.set(key, out, out.byteLength);
}

/** Insert a result computed elsewhere (a worker, a persisted store). */
export function primeWallCp(x: Float64Array, z: Float64Array, mach: number, gamma: number, alpha: number, cp: Float64Array, grid?: EulerGrid) {
  eulerCache.set(eulerKey(x, z, mach, gamma, alpha, grid), cp, cp.byteLength);
}

/** 2-D HLLC wedge march (euler2d.ts anchor) on the wasm kernel, or null when unavailable. */
export function wedgeMarch(mach: number, thetaRad: number, gamma: number, yTop: number): { p2: number; residual: number } | null {
  if (!kernelFlags.wedge) return null;
  let k: EulerExports;
  try {
    k = eulerSync();
  } catch {
    return null;
  }
  if (!k.euler_wedge) return null;
  const ptr = ensureScratch(k, 64 * 8);
  k.euler_wedge(ptr, mach, thetaRad, gamma, yTop);
  const out = new Float64Array(k.memory.buffer, ptr, 2);
  return { p2: out[0], residual: out[1] };
}

/** Parameters of the refinement kernel `euler_solve_ex` (see native/cuspis-euler/src/lib.rs). */
export interface ExParams {
  ni: number;
  nj: number;
  iters: number;
  wallAmp: number;
  shockAmp: number;
  shockWidth: number;
  /** > 0: convergence stop (iters is then the cap) */
  tol: number;
  /** coarser levels solved first (needs tol > 0) */
  sequence: number;
  /** <= 0: default */
  cfl: number;
  /** per node column shock position (ni + 1 values), read when shockAmp > 0 */
  shockS: Float64Array | null;
}

/** True when the loaded wasm exports the refinement kernel. */
export function hasSolveEx(): boolean {
  try {
    return typeof eulerSync().euler_solve_ex === "function";
  } catch {
    return false;
  }
}

/**
 * Refinement-kernel solve through the JS bridge. Returns null when the loaded wasm has no `euler_solve_ex`
 * or rejected the arguments (callers fall back to the TypeScript port). No caching here.
 */
export function solveWallEx(x: Float64Array, z: Float64Array, mach: number, gamma: number, alpha: number, p: ExParams): Float64Array | null {
  const k = eulerSync();
  if (!k.euler_solve_ex) return null;
  const n = x.length;
  const need = (n * 3 + p.ni + 1 + 16) * 8;
  const ptr = ensureScratch(k, need);
  const view = new Float64Array(k.memory.buffer, ptr, n * 3 + p.ni + 1);
  for (let i = 0; i < n; i++) {
    view[i * 2] = x[i];
    view[i * 2 + 1] = z[i];
  }
  for (let i = 0; i <= p.ni; i++) view[n * 3 + i] = p.shockS ? p.shockS[i] : 0.3;
  const res = k.euler_solve_ex(ptr, n, mach, gamma, alpha, p.ni, p.nj, p.iters, p.wallAmp, p.shockS ? p.shockAmp : 0, p.shockWidth, p.tol, p.sequence, p.cfl);
  if (!(res >= 0) || !Number.isFinite(res)) return null;
  return new Float64Array(new Float64Array(k.memory.buffer, ptr + n * 16, n));
}

function extraKey(x: Float64Array, z: Float64Array, mach: number, gamma: number, alpha: number, extra: string): string {
  return eulerKey(x, z, mach, gamma, alpha) + "|" + extra;
}

/** Cached wall-Cp for solvers that are not the default kernel (refinement presets): `extra` must identify every other input. */
export function cachedWallCp(x: Float64Array, z: Float64Array, mach: number, gamma: number, alpha: number, extra: string, compute: () => Float64Array): Float64Array {
  const key = extraKey(x, z, mach, gamma, alpha, extra);
  const hit = eulerCache.get(key);
  if (hit) return hit;
  const out = compute();
  return eulerCache.set(key, out, out.byteLength);
}

export function hasCachedWallCp(x: Float64Array, z: Float64Array, mach: number, gamma: number, alpha: number, extra: string): boolean {
  return eulerCache.has(extraKey(x, z, mach, gamma, alpha, extra));
}

export function storeCachedWallCp(x: Float64Array, z: Float64Array, mach: number, gamma: number, alpha: number, extra: string, cp: Float64Array) {
  eulerCache.set(extraKey(x, z, mach, gamma, alpha, extra), cp, cp.byteLength);
}

/** Node/CLI/desktop: instantiate the shipped `cuspis_euler.wasm` file bytes (instead of the embedded copy). */
export async function eulerFromBytes(bytes: ArrayBuffer | Uint8Array): Promise<EulerExports> {
  if (exportsRef) return exportsRef;
  const buf = bytes instanceof Uint8Array ? (bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer) : bytes;
  const mod = await WebAssembly.compile(buf);
  const inst = await WebAssembly.instantiate(mod, {});
  if (!exportsRef) wasmSource = "file";
  return exportsRef ?? adopt(inst);
}
