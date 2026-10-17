//! Optimised symmetry-plane HLLC solve. Same scheme, same Gauss-Seidel sweep order and the same
//! floating-point operation order as `solve_wall` in lib.rs (the reference), so the answer is
//! bit-identical there; the speed comes from
//!   * per-face area / unit normal / tangent computed once instead of 4 x iters x cells times,
//!   * per-cell primitive state (rho,u,v,p,a,E) computed once per update instead of once per face,
//!   * no bounds checks in the sweep (indices are derived from fixed grid dimensions),
//!   * residual evaluated on the last sweep only (it is the only one returned),
//!   * a reusable workspace, so a solve performs no heap allocation after the first call.
//! Grid dimensions and iteration count are runtime parameters (defaults match lib.rs).

use std::cell::RefCell;

pub const DEF_NI: usize = 88;
pub const DEF_NJ: usize = 28;
pub const DEF_ITERS: usize = 42;
const CFL: f64 = 0.5;
const WEDGE_CFL: f64 = 0.45;

#[derive(Clone, Copy, Default)]
struct Prim {
    r: f64,
    u: f64,
    v: f64,
    p: f64,
    a: f64,
    e: f64,
}

#[derive(Clone, Copy, Default)]
struct Face {
    area: f64,
    nx: f64,
    ny: f64,
    tx: f64,
    ty: f64,
    /// raw (unnormalised) vector, needed for the wall-pressure term
    ax: f64,
    ay: f64,
}

impl Face {
    #[inline]
    fn new(ax: f64, ay: f64) -> Face {
        let area = (ax * ax + ay * ay).sqrt();
        if area < 1e-14 {
            return Face { area, nx: 0.0, ny: 0.0, tx: 0.0, ty: 0.0, ax, ay };
        }
        let nx = ax / area;
        let ny = ay / area;
        Face { area, nx, ny, tx: -ny, ty: nx, ax, ay }
    }
}

#[derive(Default)]
struct Workspace {
    px: Vec<f64>,
    pz: Vec<f64>,
    vol: Vec<f64>,
    xc: Vec<f64>,
    /// per cell: 4 faces (W, E, S, N)
    faces: Vec<Face>,
    /// per cell: CFL * vol / per, max(vol, 1e-12)
    dtfac: Vec<f64>,
    volc: Vec<f64>,
    u: Vec<f64>,
    w: Vec<Prim>,
}

thread_local! {
    static WS: RefCell<Workspace> = RefCell::new(Workspace::default());
}

#[inline(always)]
fn prim_of(u: &[f64], gm1: f64, gamma: f64) -> Prim {
    let r = u[0].max(1e-8);
    let uu = u[1] / r;
    let vv = u[2] / r;
    let p = (gm1 * (u[3] - 0.5 * r * (uu * uu + vv * vv))).max(1e-8);
    Prim { r, u: uu, v: vv, p, a: (gamma * p / r).sqrt(), e: p / gm1 + 0.5 * r * (uu * uu + vv * vv) }
}

#[inline(always)]
fn flux4(r: f64, un: f64, ut: f64, p: f64, e: f64) -> [f64; 4] {
    [r * un, r * un * un + p, r * un * ut, (e + p) * un]
}

/// HLLC flux through `f` from state `l` (own cell) to state `rr`. Mirrors lib.rs::hllc.
#[inline(always)]
fn hllc<const DEN0: bool>(l: &Prim, rr: &Prim, f: &Face) -> [f64; 4] {
    if f.area < 1e-14 {
        return [0.0; 4];
    }
    let (nx, ny, tx, ty) = (f.nx, f.ny, f.tx, f.ty);
    let un_l = l.u * nx + l.v * ny;
    let ut_l = l.u * tx + l.v * ty;
    let un_r = rr.u * nx + rr.v * ny;
    let ut_r = rr.u * tx + rr.v * ty;
    let amax = l.a.max(rr.a);
    let sl = un_l.min(un_r) - amax;
    let sr = un_l.max(un_r) + amax;
    let den = l.r * (sl - un_l) - rr.r * (sr - un_r);
    // DEN0: the 2-D wedge anchor (euler2d.ts) tests den === 0, the strip solver tests |den| < 1e-14.
    let sm = if (DEN0 && den == 0.0) || (!DEN0 && den.abs() < 1e-14) {
        0.5 * (un_l + un_r)
    } else {
        (rr.p - l.p + l.r * un_l * (sl - un_l) - rr.r * un_r * (sr - un_r)) / den
    };
    let (fr, fun, fut, fe) = if sl >= 0.0 {
        let q = flux4(l.r, un_l, ut_l, l.p, l.e);
        (q[0], q[1], q[2], q[3])
    } else if sr <= 0.0 {
        let q = flux4(rr.r, un_r, ut_r, rr.p, rr.e);
        (q[0], q[1], q[2], q[3])
    } else {
        let (r, un, ut, p, e, s) = if sm >= 0.0 {
            (l.r, un_l, ut_l, l.p, l.e, sl)
        } else {
            (rr.r, un_r, ut_r, rr.p, rr.e, sr)
        };
        let coeff = (r * (s - un)) / (s - sm);
        let e_over = e / r + (sm - un) * (sm + p / (r * (s - un)));
        let q = flux4(r, un, ut, p, e);
        (
            q[0] + s * (coeff - r),
            q[1] + s * (coeff * sm - r * un),
            q[2] + s * (coeff * ut - r * ut),
            q[3] + s * (coeff * e_over - e),
        )
    };
    [
        fr * f.area,
        (fun * nx + fut * tx) * f.area,
        (fun * ny + fut * ty) * f.area,
        fe * f.area,
    ]
}

fn wall_z(x: f64, xw: &[f64], zw: &[f64]) -> f64 {
    if x <= xw[0] {
        return zw[0];
    }
    let last = xw.len() - 1;
    if x >= xw[last] {
        return zw[last];
    }
    let mut lo = 0usize;
    let mut hi = last;
    while hi - lo > 1 {
        let mid = (lo + hi) >> 1;
        if xw[mid] <= x {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let dx = xw[hi] - xw[lo];
    let t = if dx.abs() < 1e-14 { 0.0 } else { (x - xw[lo]) / dx };
    zw[lo] + (zw[hi] - zw[lo]) * t
}

/// Fill every workspace array that depends only on the node coordinates in `ws.px/pz`
/// (grid of `ni` x `nj` cells, `(ni+1) x (nj+1)` nodes).
fn build_metrics(ws: &mut Workspace, ni: usize, nj: usize, cfl: f64, vol_floor: f64) {
    let npx = ni + 1;
    let nc = ni * nj;
    ws.vol.clear();
    ws.vol.resize(nc, 0.0);
    ws.xc.clear();
    ws.xc.resize(nc, 0.0);
    ws.faces.clear();
    ws.faces.resize(nc * 4, Face::default());
    ws.dtfac.clear();
    ws.dtfac.resize(nc, 0.0);
    ws.volc.clear();
    ws.volc.resize(nc, 0.0);
    for j in 0..nj {
        for i in 0..ni {
            let sw = j * npx + i;
            let se = sw + 1;
            let nw = (j + 1) * npx + i;
            let ne = nw + 1;
            let id = j * ni + i;
            let poly_x = [ws.px[sw], ws.px[se], ws.px[ne], ws.px[nw]];
            let poly_z = [ws.pz[sw], ws.pz[se], ws.pz[ne], ws.pz[nw]];
            let mut a = 0.0;
            for k in 0..4 {
                let m = (k + 1) % 4;
                a += poly_x[k] * poly_z[m] - poly_x[m] * poly_z[k];
            }
            let vol = (a * 0.5).abs().max(vol_floor);
            ws.vol[id] = vol;
            ws.xc[id] = 0.25 * (poly_x[0] + poly_x[1] + poly_x[2] + poly_x[3]);
            let fw = Face::new(-(ws.pz[nw] - ws.pz[sw]), ws.px[nw] - ws.px[sw]);
            let fe = Face::new(ws.pz[ne] - ws.pz[se], -(ws.px[ne] - ws.px[se]));
            let fs = Face::new(ws.pz[se] - ws.pz[sw], -(ws.px[se] - ws.px[sw]));
            let fnn = Face::new(-(ws.pz[ne] - ws.pz[nw]), ws.px[ne] - ws.px[nw]);
            // reference: per = max of raw vector lengths, floor 1e-8 (lengths == Face.area here)
            let per = fw.area.max(fe.area).max(fs.area).max(fnn.area).max(1e-8);
            ws.dtfac[id] = cfl * (vol / per);
            ws.volc[id] = vol.max(1e-12);
            ws.faces[id * 4] = fw;
            ws.faces[id * 4 + 1] = fe;
            ws.faces[id * 4 + 2] = fs;
            ws.faces[id * 4 + 3] = fnn;
        }
    }
}

/// Uniform freestream initial state; returns the freestream primitive state exactly as the
/// reference passes it to the far-field faces (not re-derived through the conservative vector).
fn init_state(ws: &mut Workspace, nc: usize, gamma: f64, m: f64, alpha: f64) -> Prim {
    let rho_inf = 1.0;
    let p_inf = 1.0;
    let a_inf = (gamma * p_inf / rho_inf).sqrt();
    let u_inf = m * a_inf * alpha.cos();
    let v_inf = m * a_inf * alpha.sin();
    let e_inf = p_inf / (gamma - 1.0) + 0.5 * rho_inf * (u_inf * u_inf + v_inf * v_inf);
    ws.u.clear();
    ws.u.resize(nc * 4, 0.0);
    for c in 0..nc {
        ws.u[c * 4] = rho_inf;
        ws.u[c * 4 + 1] = rho_inf * u_inf;
        ws.u[c * 4 + 2] = rho_inf * v_inf;
        ws.u[c * 4 + 3] = e_inf;
    }
    let gm1 = gamma - 1.0;
    ws.w.clear();
    ws.w.resize(nc, Prim::default());
    for c in 0..nc {
        ws.w[c] = prim_of(&ws.u[c * 4..c * 4 + 4], gm1, gamma);
    }
    Prim {
        r: rho_inf,
        u: u_inf,
        v: v_inf,
        p: p_inf,
        a: (gamma * p_inf / rho_inf).sqrt(),
        e: p_inf / gm1 + 0.5 * rho_inf * (u_inf * u_inf + v_inf * v_inf),
    }
}

/// Gauss-Seidel sweeps in the exact order of the reference solvers. Returns the final residual.
fn sweep<const DEN0: bool>(ws: &mut Workspace, ni: usize, nj: usize, iters: usize, gamma: f64, fs_prim: &Prim) -> f64 {
    let nc = ni * nj;
    let gm1 = gamma - 1.0;
    let mut residual = 1.0;
    let u = &mut ws.u[..];
    let w = &mut ws.w[..];
    let faces = &ws.faces[..];
    let dtfac = &ws.dtfac[..];
    let volc = &ws.volc[..];
    for it in 0..iters {
        let last = it + 1 == iters;
        let mut res_sum = 0.0;
        for j in 0..nj {
            for i in 0..ni {
                let id = j * ni + i;
                // SAFETY: id < nc; neighbour reads are guarded by the i/j tests.
                unsafe {
                    let own = *w.get_unchecked(id);
                    let fcs = faces.get_unchecked(id * 4..id * 4 + 4);
                    let mut d = [0.0f64; 4];
                    let f = if i == 0 {
                        hllc::<DEN0>(&own, fs_prim, &fcs[0])
                    } else {
                        hllc::<DEN0>(&own, w.get_unchecked(id - 1), &fcs[0])
                    };
                    d[0] += f[0];
                    d[1] += f[1];
                    d[2] += f[2];
                    d[3] += f[3];
                    let f = if i + 1 == ni {
                        hllc::<DEN0>(&own, &own, &fcs[1])
                    } else {
                        hllc::<DEN0>(&own, w.get_unchecked(id + 1), &fcs[1])
                    };
                    d[0] += f[0];
                    d[1] += f[1];
                    d[2] += f[2];
                    d[3] += f[3];
                    let f = if j + 1 == nj {
                        hllc::<DEN0>(&own, fs_prim, &fcs[3])
                    } else {
                        hllc::<DEN0>(&own, w.get_unchecked(id + ni), &fcs[3])
                    };
                    d[0] += f[0];
                    d[1] += f[1];
                    d[2] += f[2];
                    d[3] += f[3];
                    if j == 0 {
                        d[1] += own.p * fcs[2].ax;
                        d[2] += own.p * fcs[2].ay;
                    } else {
                        let f = hllc::<DEN0>(&own, w.get_unchecked(id - ni), &fcs[2]);
                        d[0] += f[0];
                        d[1] += f[1];
                        d[2] += f[2];
                        d[3] += f[3];
                    }
                    let spec = (own.u * own.u + own.v * own.v).sqrt() + own.a;
                    let dt = *dtfac.get_unchecked(id) / spec.max(1e-6);
                    let scale = dt / *volc.get_unchecked(id);
                    let o = id * 4;
                    let nr = *u.get_unchecked(o) - d[0] * scale;
                    let nru = *u.get_unchecked(o + 1) - d[1] * scale;
                    let nrv = *u.get_unchecked(o + 2) - d[2] * scale;
                    let nre = *u.get_unchecked(o + 3) - d[3] * scale;
                    let rr = nr.max(1e-8);
                    let uu2 = nru / rr;
                    let vv2 = nrv / rr;
                    let pnew = gm1 * (nre - 0.5 * rr * (uu2 * uu2 + vv2 * vv2));
                    if pnew > 1e-6 && nr > 1e-8 {
                        *u.get_unchecked_mut(o) = nr;
                        *u.get_unchecked_mut(o + 1) = nru;
                        *u.get_unchecked_mut(o + 2) = nrv;
                        *u.get_unchecked_mut(o + 3) = nre;
                        *w.get_unchecked_mut(id) = prim_of(u.get_unchecked(o..o + 4), gm1, gamma);
                    }
                    if last {
                        let src = d[0] / *volc.get_unchecked(id);
                        res_sum += src * src;
                    }
                }
            }
        }
        if last {
            residual = (res_sum / nc as f64).sqrt();
        }
    }
    residual
}

/// Returns the residual; writes the wall Cp into `cp_out` (len n).
pub fn solve_wall_fast(
    xw: &[f64],
    zw: &[f64],
    mach: f64,
    gamma: f64,
    alpha: f64,
    ni: usize,
    nj: usize,
    iters: usize,
    cp_out: &mut [f64],
) -> f64 {
    WS.with(|cell| {
        let mut guard = cell.borrow_mut();
        solve_in(&mut guard, xw, zw, mach, gamma, alpha, ni, nj, iters, cp_out)
    })
}

#[allow(clippy::too_many_arguments)]
fn solve_in(
    ws: &mut Workspace,
    xw: &[f64],
    zw: &[f64],
    mach: f64,
    gamma: f64,
    alpha: f64,
    ni: usize,
    nj: usize,
    iters: usize,
    cp_out: &mut [f64],
) -> f64 {
    let n = xw.len();
    let x_nose = xw[0];
    let x_te = xw[n - 1];
    let length = (x_te - x_nose).abs().max(1e-4);
    let sign = if zw[n / 2] < -1e-8 { -1.0 } else { 1.0 };
    let xmin = x_nose - 0.12 * length;
    let xmax = x_te + 0.02 * length;
    let h_far = 0.55 * length;
    let npx = ni + 1;
    let npz = nj + 1;
    let nc = ni * nj;
    ws.px.clear();
    ws.px.resize(npx * npz, 0.0);
    ws.pz.clear();
    ws.pz.resize(npx * npz, 0.0);
    for j in 0..npz {
        let s = j as f64 / nj as f64;
        let eta = s.powf(1.55);
        for i in 0..npx {
            let xf = i as f64 / ni as f64;
            let x = xmin + (xmax - xmin) * xf;
            let zwall = if x <= x_nose { zw[0] } else { wall_z(x, xw, zw) };
            let id = j * npx + i;
            ws.px[id] = x;
            ws.pz[id] = zwall + sign * eta * h_far;
        }
    }
    build_metrics(ws, ni, nj, CFL, 1e-14);
    let m = mach.max(1.05);
    let fs_prim = init_state(ws, nc, gamma, m, alpha);
    let residual = sweep::<false>(ws, ni, nj, iters, gamma, &fs_prim);

    let q = 0.5 * gamma * m * m;
    for k in 0..n {
        let x = xw[k];
        let mut best = 0usize;
        let mut best_d = f64::MAX;
        for i in 0..ni {
            let d = (ws.xc[i] - x).abs();
            if d < best_d {
                best_d = d;
                best = i;
            }
        }
        cp_out[k] = (ws.w[best].p - 1.0) / q;
    }
    residual
}

/// 2-D wedge anchor (the `eulerWedge` check in euler2d.ts): unit-length ramp from x0 = 0.2,
/// freestream M along +x. `y_top` is the outer boundary height computed by the caller.
/// Returns the mean wall pressure ratio over the plateau; the residual is written to `*res`.
pub fn solve_wedge(m: f64, theta: f64, gamma: f64, y_top: f64, res: &mut f64) -> f64 {
    const NI: usize = 72;
    const NJ: usize = 24;
    const ITERS: usize = 280;
    const LX: f64 = 1.0;
    const X0: f64 = 0.2;
    WS.with(|cell| {
        let mut guard = cell.borrow_mut();
        let ws = &mut *guard;
        let npx = NI + 1;
        let npz = NJ + 1;
        ws.px.clear();
        ws.px.resize(npx * npz, 0.0);
        ws.pz.clear();
        ws.pz.resize(npx * npz, 0.0);
        let tan_t = theta.tan();
        for j in 0..npz {
            for i in 0..npx {
                let x = (i as f64 / NI as f64) * LX;
                let yw = if x <= X0 { 0.0 } else { (x - X0) * tan_t };
                let id = j * npx + i;
                ws.px[id] = x;
                ws.pz[id] = yw + (j as f64 / NJ as f64) * (y_top - yw);
            }
        }
        build_metrics(ws, NI, NJ, WEDGE_CFL, 0.0);
        let fs_prim = init_state(ws, NI * NJ, gamma, m, 0.0);
        *res = sweep::<true>(ws, NI, NJ, ITERS, gamma, &fs_prim);
        let mut p_sum = 0.0;
        let mut n_sum = 0usize;
        for i in 0..NI {
            let x = ((i as f64 + 0.5) / NI as f64) * LX;
            if x < X0 + 0.18 || x > LX * 0.82 {
                continue;
            }
            p_sum += ws.w[i].p;
            n_sum += 1;
        }
        if n_sum > 0 { p_sum / n_sum as f64 } else { 1.0 }
    })
}

/// Same memory contract as `euler_solve`: n (x, z) pairs in at `ptr`, n Cp values out at `ptr + 16 n`.
/// Uses the default grid / iteration count of the reference solver.
#[no_mangle]
pub extern "C" fn euler_solve_fast(ptr: *mut u8, n: i32, mach: f64, gamma: f64, alpha: f64) -> f64 {
    euler_solve_grid(ptr, n, mach, gamma, alpha, DEF_NI as i32, DEF_NJ as i32, DEF_ITERS as i32)
}

/// As `euler_solve_fast` with explicit grid size and sweep count (coarse previews, refinement).
#[no_mangle]
pub extern "C" fn euler_solve_grid(
    ptr: *mut u8,
    n: i32,
    mach: f64,
    gamma: f64,
    alpha: f64,
    ni: i32,
    nj: i32,
    iters: i32,
) -> f64 {
    if ptr.is_null() || n < 4 || ni < 4 || nj < 4 || iters < 1 {
        return 1.0;
    }
    let n = n as usize;
    // SAFETY: the caller allocated >= 24 n bytes with euler_alloc (input 16 n, output 8 n).
    let (input, out) = unsafe {
        (
            std::slice::from_raw_parts(ptr as *const f64, n * 2),
            std::slice::from_raw_parts_mut(ptr.add(n * 16) as *mut f64, n),
        )
    };
    let mut xw = Vec::with_capacity(n);
    let mut zw = Vec::with_capacity(n);
    for i in 0..n {
        xw.push(input[i * 2]);
        zw.push(input[i * 2 + 1]);
    }
    solve_wall_fast(&xw, &zw, mach, gamma, alpha, ni as usize, nj as usize, iters as usize, out)
}

/// Wedge anchor. `ptr` points at >= 16 bytes from euler_alloc: out[0] = mean wall p/p_inf, out[1] = residual.
#[no_mangle]
pub extern "C" fn euler_wedge(ptr: *mut u8, mach: f64, theta_rad: f64, gamma: f64, y_top: f64) {
    if ptr.is_null() {
        return;
    }
    let mut res = 1.0;
    let p2 = solve_wedge(mach, theta_rad, gamma, y_top, &mut res);
    // SAFETY: caller guarantees 16 writable bytes.
    unsafe {
        let out = std::slice::from_raw_parts_mut(ptr as *mut f64, 2);
        out[0] = p2;
        out[1] = res;
    }
}
