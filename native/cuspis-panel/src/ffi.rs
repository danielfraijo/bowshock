//! C ABI for the wasm build. Single-threaded: one Kernel per wasm instance (one per worker).
//!
//! Buffers are allocated with `cp_alloc` (8-byte aligned) and freed with `cp_free`.
//! Geometry layout (`cp_mesh_load`): `geom` = 7·nt f64, SoA [area | nx | ny | nz | cx | cy | cz] exactly as
//! panel.ts `panelGeom` computes them, `surf` = nt u16 (panelGeom's reclassified surface id), `a_wet` =
//! panelGeom.aWet. The raw mesh (positions 3·nv f64, indices 3·nt u32, surfaces nt u8, tags nt i32 or null)
//! is only used by the shock-expansion strip/face merge; pass nv = 0 to skip it.

use crate::{Config, Kernel, Method, PanelMesh, RawMesh, STRIDE_IN, STRIDE_OUT};
use std::cell::RefCell;

thread_local! {
    static KERNEL: RefCell<Kernel> = RefCell::new(Kernel::new());
}

#[no_mangle]
pub extern "C" fn cp_version() -> u32 {
    1
}

#[no_mangle]
pub extern "C" fn cp_stride_in() -> u32 {
    STRIDE_IN as u32
}

#[no_mangle]
pub extern "C" fn cp_stride_out() -> u32 {
    STRIDE_OUT as u32
}

/// 8-byte aligned buffer of at least `n_bytes`.
#[no_mangle]
pub extern "C" fn cp_alloc(n_bytes: u32) -> *mut u8 {
    let words = (n_bytes as usize).max(8).div_ceil(8);
    let mut v: Vec<u64> = vec![0; words];
    let p = v.as_mut_ptr() as *mut u8;
    std::mem::forget(v);
    p
}

/// # Safety
/// `ptr` must come from `cp_alloc(n_bytes)` with the same `n_bytes`.
#[no_mangle]
pub unsafe extern "C" fn cp_free(ptr: *mut u8, n_bytes: u32) {
    if ptr.is_null() {
        return;
    }
    let words = (n_bytes as usize).max(8).div_ceil(8);
    drop(Vec::from_raw_parts(ptr as *mut u64, words, words));
}

unsafe fn slice<'a, T>(p: *const T, n: usize) -> &'a [T] {
    if p.is_null() || n == 0 {
        &[]
    } else {
        std::slice::from_raw_parts(p, n)
    }
}

/// Load one mesh; returns a handle (>= 1), 0 on bad input.
///
/// # Safety
/// Pointers must reference buffers of the documented sizes in this module's memory.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn cp_mesh_load(
    nt: u32,
    geom: *const f64,
    surf: *const u16,
    a_wet: f64,
    nv: u32,
    positions: *const f64,
    indices: *const u32,
    surfaces: *const u8,
    tags: *const i32,
) -> u32 {
    let nt = nt as usize;
    if geom.is_null() || surf.is_null() {
        return 0;
    }
    let g = slice(geom, nt * 7);
    let raw = if nv > 0 && !positions.is_null() && !indices.is_null() {
        Some(RawMesh {
            positions: slice(positions, nv as usize * 3).to_vec(),
            indices: slice(indices, nt * 3).to_vec(),
            surfaces: slice(surfaces, nt).to_vec(),
            tags: if tags.is_null() { None } else { Some(slice(tags, nt).to_vec()) },
        })
    } else {
        None
    };
    let mut m = PanelMesh {
        area: g[0..nt].to_vec(),
        nx: g[nt..2 * nt].to_vec(),
        ny: g[2 * nt..3 * nt].to_vec(),
        nz: g[3 * nt..4 * nt].to_vec(),
        cx: g[4 * nt..5 * nt].to_vec(),
        cy: g[5 * nt..6 * nt].to_vec(),
        cz: g[6 * nt..7 * nt].to_vec(),
        surf: slice(surf, nt).to_vec(),
        a_wet,
        ..Default::default()
    };
    m.raw = raw;
    KERNEL.with(|k| k.borrow_mut().load(m))
}

#[no_mangle]
pub extern "C" fn cp_mesh_drop(h: u32) {
    KERNEL.with(|k| k.borrow_mut().drop_mesh(h));
}

#[no_mangle]
pub extern "C" fn cp_mesh_count() -> u32 {
    KERNEL.with(|k| k.borrow().mesh_count() as u32)
}

/// Evaluate `n` points: `inp` n·STRIDE_IN f64, `out` n·STRIDE_OUT f64. Returns 0 on success.
///
/// # Safety
/// `inp` / `out` must reference buffers of the documented sizes.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn cp_eval(
    h: u32,
    method: u32,
    gamma: f64,
    use_cone: u32,
    cg_x: f64,
    s_ref: f64,
    l_ref: f64,
    n: u32,
    inp: *const f64,
    out: *mut f64,
) -> i32 {
    let Some(method) = Method::from_u32(method) else { return -2 };
    if inp.is_null() || out.is_null() {
        return -3;
    }
    let n = n as usize;
    let cfg = Config { method, gamma, use_cone: use_cone != 0, cg_x, s_ref, l_ref };
    let i = slice(inp, n * STRIDE_IN);
    let o = std::slice::from_raw_parts_mut(out, n * STRIDE_OUT);
    KERNEL.with(|k| match k.borrow_mut().eval(h, &cfg, i, o) {
        Ok(()) => 0,
        Err(_) => -1,
    })
}

/// Drop the gas caches (fastBeta tables, ν⁻¹ table, cone nodes, shock-expansion tables).
#[no_mangle]
pub extern "C" fn cp_clear_caches() {
    KERNEL.with(|k| {
        let mut k = k.borrow_mut();
        k.gas.clear();
    });
}
