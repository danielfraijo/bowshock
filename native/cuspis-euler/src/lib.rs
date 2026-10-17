//! Symmetry-plane Euler. First-order HLLC, slip wall, local dt.
//! Input in wasm memory: n pairs (x, z) of the wall, nose at the first point.
//! Output: n Cp values written immediately after the input.
//!
//! [mesh-refinement] `euler_solve` is the original entry point and keeps its 88x28x42 grid.
//! `euler_solve_ex` takes the grid (cells, iterations) and wall / shock clustering from the
//! caller. The shipped euler-wasm.ts predates `euler_solve_ex`; until the wasm is rebuilt
//! (needs the wasm32-unknown-unknown target) the same algorithm runs in TypeScript
//! (src/lib/waverider/strip-euler.ts) and the two are cross-checked by the tests.

const NI: usize = 88;
const NJ: usize = 28;
const ITERS: usize = 42;
const CFL: f64 = 0.5;
/// Legacy wall stretching: eta = s^WALL_EXP.
const WALL_EXP: f64 = 1.55;

#[no_mangle]
pub extern "C" fn euler_alloc(n_bytes: i32) -> *mut u8 {
    let n = n_bytes.max(64) as usize;
    let mut buf = Vec::<u8>::with_capacity(n);
    buf.resize(n, 0);
    let ptr = buf.as_mut_ptr();
    std::mem::forget(buf);
    ptr
}

#[no_mangle]
pub extern "C" fn euler_solve(ptr: *mut u8, n: i32, mach: f64, gamma: f64, alpha: f64) -> f64 {
    if ptr.is_null() || n < 4 {
        return 1.0;
    }
    let n = n as usize;
    let input = unsafe { std::slice::from_raw_parts(ptr as *const f64, n * 2) };
    let mut xw = vec![0.0; n];
    let mut zw = vec![0.0; n];
    for i in 0..n {
        xw[i] = input[i * 2];
        zw[i] = input[i * 2 + 1];
    }
    let (cp, residual) = solve_wall(&xw, &zw, mach, gamma, alpha);
    let out = unsafe { std::slice::from_raw_parts_mut(ptr.add(n * 16) as *mut f64, n) };
    out.copy_from_slice(&cp);
    residual
}

/// Refined-grid entry point.
/// Buffer layout (f64): [0, 2n) wall (x, z) pairs, [2n, 3n) Cp out, [3n, 3n + ni + 1) shock position s per
/// node column (read only when `shock_amp > 0`). Allocate `(3n + ni + 1 + 16) * 8` bytes with `euler_alloc`.
/// Returns the final residual, or -1 when the arguments are rejected (nothing is written).
#[no_mangle]
pub extern "C" fn euler_solve_ex(
    ptr: *mut u8,
    n: i32,
    mach: f64,
    gamma: f64,
    alpha: f64,
    ni: i32,
    nj: i32,
    iters: i32,
    wall_amp: f64,
    shock_amp: f64,
    shock_width: f64,
) -> f64 {
    if ptr.is_null() || n < 4 || ni < 8 || nj < 6 || iters < 1 || ni > 1000 || nj > 400 || iters > 20000 {
        return -1.0;
    }
    let n = n as usize;
    let ni = ni as usize;
    let nj = nj as usize;
    let input = unsafe { std::slice::from_raw_parts(ptr as *const f64, n * 3 + ni + 1) };
    let mut xw = vec![0.0; n];
    let mut zw = vec![0.0; n];
    for i in 0..n {
        xw[i] = input[i * 2];
        zw[i] = input[i * 2 + 1];
    }
    let shock_s: Vec<f64> = input[n * 3..n * 3 + ni + 1].to_vec();
    let grid = Grid {
        ni,
        nj,
        iters: iters as usize,
        wall_amp: wall_amp.max(0.0),
        shock_amp: shock_amp.max(0.0),
        shock_width,
        shock_s: if shock_amp > 0.0 { Some(shock_s) } else { None },
    };
    let (cp, residual) = solve_wall_grid(&xw, &zw, mach, gamma, alpha, &grid);
    let out = unsafe { std::slice::from_raw_parts_mut(ptr.add(n * 16) as *mut f64, n) };
    out.copy_from_slice(&cp);
    residual
}

pub struct Grid {
    pub ni: usize,
    pub nj: usize,
    pub iters: usize,
    pub wall_amp: f64,
    pub shock_amp: f64,
    pub shock_width: f64,
    /// Per node column (ni + 1): predicted shock position in s. None = no shock clustering.
    pub shock_s: Option<Vec<f64>>,
}

/// Equidistribute m(s) = 1 + wall*exp(-s/lw) + shock*exp(-((s-c)/w)^2): n + 1 strictly increasing
/// coordinates from exactly 0 to exactly 1. Same algorithm as clusterCoordinates in refinement.ts.
pub fn cluster_coordinates(n: usize, wall: f64, shock: f64, pos: f64, width: f64) -> Vec<f64> {
    let mut out = vec![0.0; n + 1];
    if !(wall > 0.0) && !(shock > 0.0) {
        for j in 0..=n {
            out[j] = j as f64 / n as f64;
        }
        return out;
    }
    let samples = 1024usize.max(8 * n);
    let lw = 0.12;
    let c = pos.max(0.0).min(1.0);
    let w = width.max(1e-3);
    let monitor = |s: f64| {
        let d = (s - c) / w;
        1.0 + wall * (-s / lw).exp() + shock * (-d * d).exp()
    };
    let mut cum = vec![0.0; samples + 1];
    let mut prev = monitor(0.0);
    for k in 1..=samples {
        let m = monitor(k as f64 / samples as f64);
        cum[k] = cum[k - 1] + 0.5 * (prev + m) / samples as f64;
        prev = m;
    }
    let total = cum[samples];
    let mut k = 0usize;
    out[0] = 0.0;
    for j in 1..n {
        let target = total * j as f64 / n as f64;
        while k < samples - 1 && cum[k + 1] < target {
            k += 1;
        }
        let span = cum[k + 1] - cum[k];
        let t = if span > 0.0 { (target - cum[k]) / span } else { 0.0 };
        out[j] = (k as f64 + t) / samples as f64;
    }
    out[n] = 1.0;
    out
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

fn solve_wall(xw: &[f64], zw: &[f64], mach: f64, gamma: f64, alpha: f64) -> (Vec<f64>, f64) {
    let grid = Grid { ni: NI, nj: NJ, iters: ITERS, wall_amp: 0.0, shock_amp: 0.0, shock_width: 0.09, shock_s: None };
    solve_wall_grid(xw, zw, mach, gamma, alpha, &grid)
}

pub fn solve_wall_grid(xw: &[f64], zw: &[f64], mach: f64, gamma: f64, alpha: f64, grid: &Grid) -> (Vec<f64>, f64) {
    let gni = grid.ni;
    let gnj = grid.nj;
    let giters = grid.iters;
    let clustered = grid.wall_amp > 0.0 || grid.shock_amp > 0.0;
    let n = xw.len();
    let x_nose = xw[0];
    let x_te = xw[n - 1];
    let length = (x_te - x_nose).abs().max(1e-4);
    let sign = if zw[n / 2] < -1e-8 { -1.0 } else { 1.0 };
    let xmin = x_nose - 0.12 * length;
    let xmax = x_te + 0.02 * length;
    let h_far = 0.55 * length;
    let npx = gni + 1;
    let npz = gnj + 1;
    let mut px = vec![0.0; npx * npz];
    let mut pz = vec![0.0; npx * npz];
    let col_s: Vec<Vec<f64>> = if clustered {
        (0..npx)
            .map(|i| {
                let (amp, pos) = match &grid.shock_s {
                    Some(v) => (grid.shock_amp, v[i].max(0.04).min(0.92)),
                    None => (0.0, 0.3),
                };
                cluster_coordinates(gnj, grid.wall_amp, amp, pos, grid.shock_width)
            })
            .collect()
    } else {
        Vec::new()
    };
    for j in 0..npz {
        let s0 = j as f64 / gnj as f64;
        for i in 0..npx {
            let s = if clustered { col_s[i][j] } else { s0 };
            let eta = s.powf(WALL_EXP);
            let xf = i as f64 / gni as f64;
            let x = xmin + (xmax - xmin) * xf;
            let zwall = if x <= x_nose { zw[0] } else { wall_z(x, xw, zw) };
            let id = j * npx + i;
            px[id] = x;
            pz[id] = zwall + sign * eta * h_far;
        }
    }

    let nc = gni * gnj;
    let mut vol = vec![0.0; nc];
    let mut fw = vec![0.0; nc * 2];
    let mut fe = vec![0.0; nc * 2];
    let mut fs = vec![0.0; nc * 2];
    let mut fnor = vec![0.0; nc * 2];
    let mut xc = vec![0.0; nc];
    for j in 0..gnj {
        for i in 0..gni {
            let sw = j * npx + i;
            let se = sw + 1;
            let nw = (j + 1) * npx + i;
            let ne = nw + 1;
            let id = j * gni + i;
            let poly_x = [px[sw], px[se], px[ne], px[nw]];
            let poly_z = [pz[sw], pz[se], pz[ne], pz[nw]];
            let mut a = 0.0;
            for k in 0..4 {
                let n = (k + 1) % 4;
                a += poly_x[k] * poly_z[n] - poly_x[n] * poly_z[k];
            }
            vol[id] = (a * 0.5).abs().max(1e-14);
            xc[id] = 0.25 * (poly_x[0] + poly_x[1] + poly_x[2] + poly_x[3]);
            let ex = px[nw] - px[sw];
            let ez = pz[nw] - pz[sw];
            fw[id * 2] = -ez;
            fw[id * 2 + 1] = ex;
            let eex = px[ne] - px[se];
            let eez = pz[ne] - pz[se];
            fe[id * 2] = eez;
            fe[id * 2 + 1] = -eex;
            let sx = px[se] - px[sw];
            let sz = pz[se] - pz[sw];
            fs[id * 2] = sz;
            fs[id * 2 + 1] = -sx;
            let nx = px[ne] - px[nw];
            let nz = pz[ne] - pz[nw];
            fnor[id * 2] = -nz;
            fnor[id * 2 + 1] = nx;
        }
    }

    let rho_inf = 1.0;
    let p_inf = 1.0;
    let a_inf = (gamma * p_inf / rho_inf).sqrt();
    let m = mach.max(1.05);
    let u_inf = m * a_inf * alpha.cos();
    let v_inf = m * a_inf * alpha.sin();
    let e_inf = p_inf / (gamma - 1.0) + 0.5 * rho_inf * (u_inf * u_inf + v_inf * v_inf);
    let mut u = vec![0.0; nc * 4];
    for c in 0..nc {
        u[c * 4] = rho_inf;
        u[c * 4 + 1] = rho_inf * u_inf;
        u[c * 4 + 2] = rho_inf * v_inf;
        u[c * 4 + 3] = e_inf;
    }

    let mut residual = 1.0;
    let gm1 = gamma - 1.0;
    for _ in 0..giters {
        let mut res_sum = 0.0;
        for j in 0..gnj {
            for i in 0..gni {
                let id = j * gni + i;
                let o = id * 4;
                let r = u[o].max(1e-8);
                let uu = u[o + 1] / r;
                let vv = u[o + 2] / r;
                let pp = (gm1 * (u[o + 3] - 0.5 * r * (uu * uu + vv * vv))).max(1e-8);
                let mut d0 = 0.0;
                let mut d1 = 0.0;
                let mut d2 = 0.0;
                let mut d3 = 0.0;
                let take = |f: [f64; 4], d0: &mut f64, d1: &mut f64, d2: &mut f64, d3: &mut f64| {
                    *d0 += f[0];
                    *d1 += f[1];
                    *d2 += f[2];
                    *d3 += f[3];
                };
                if i == 0 {
                    let f = hllc(r, uu, vv, pp, rho_inf, u_inf, v_inf, p_inf, fw[id * 2], fw[id * 2 + 1], gamma);
                    take(f, &mut d0, &mut d1, &mut d2, &mut d3);
                } else {
                    let ol = ((j * gni + (i - 1)) * 4) as usize;
                    let (rr, ur, vr, pr) = cons_to_prim(&u, ol, gm1);
                    let f = hllc(r, uu, vv, pp, rr, ur, vr, pr, fw[id * 2], fw[id * 2 + 1], gamma);
                    take(f, &mut d0, &mut d1, &mut d2, &mut d3);
                }
                if i + 1 == gni {
                    let f = hllc(r, uu, vv, pp, r, uu, vv, pp, fe[id * 2], fe[id * 2 + 1], gamma);
                    take(f, &mut d0, &mut d1, &mut d2, &mut d3);
                } else {
                    let ol = ((j * gni + (i + 1)) * 4) as usize;
                    let (rr, ur, vr, pr) = cons_to_prim(&u, ol, gm1);
                    let f = hllc(r, uu, vv, pp, rr, ur, vr, pr, fe[id * 2], fe[id * 2 + 1], gamma);
                    take(f, &mut d0, &mut d1, &mut d2, &mut d3);
                }
                if j + 1 == gnj {
                    let f = hllc(r, uu, vv, pp, rho_inf, u_inf, v_inf, p_inf, fnor[id * 2], fnor[id * 2 + 1], gamma);
                    take(f, &mut d0, &mut d1, &mut d2, &mut d3);
                } else {
                    let ol = (((j + 1) * gni + i) * 4) as usize;
                    let (rr, ur, vr, pr) = cons_to_prim(&u, ol, gm1);
                    let f = hllc(r, uu, vv, pp, rr, ur, vr, pr, fnor[id * 2], fnor[id * 2 + 1], gamma);
                    take(f, &mut d0, &mut d1, &mut d2, &mut d3);
                }
                if j == 0 {
                    d1 += pp * fs[id * 2];
                    d2 += pp * fs[id * 2 + 1];
                } else {
                    let ol = (((j - 1) * gni + i) * 4) as usize;
                    let (rr, ur, vr, pr) = cons_to_prim(&u, ol, gm1);
                    let f = hllc(r, uu, vv, pp, rr, ur, vr, pr, fs[id * 2], fs[id * 2 + 1], gamma);
                    take(f, &mut d0, &mut d1, &mut d2, &mut d3);
                }
                let spec = (uu * uu + vv * vv).sqrt() + (gamma * pp / r).sqrt();
                let per = fw_len(&fw, id)
                    .max(fw_len(&fe, id))
                    .max(fw_len(&fs, id))
                    .max(fw_len(&fnor, id))
                    .max(1e-8);
                let dt = (CFL * (vol[id] / per)) / spec.max(1e-6);
                let scale = dt / vol[id].max(1e-12);
                let nr = u[o] - d0 * scale;
                let nru = u[o + 1] - d1 * scale;
                let nrv = u[o + 2] - d2 * scale;
                let nre = u[o + 3] - d3 * scale;
                let rr = nr.max(1e-8);
                let uu2 = nru / rr;
                let vv2 = nrv / rr;
                let pnew = gm1 * (nre - 0.5 * rr * (uu2 * uu2 + vv2 * vv2));
                if pnew > 1e-6 && nr > 1e-8 {
                    u[o] = nr;
                    u[o + 1] = nru;
                    u[o + 2] = nrv;
                    u[o + 3] = nre;
                }
                let src = d0 / vol[id].max(1e-12);
                res_sum += src * src;
            }
        }
        residual = (res_sum / nc as f64).sqrt();
    }

    let q = 0.5 * gamma * m * m;
    let mut cp = vec![0.0; n];
    for k in 0..n {
        let x = xw[k];
        let mut best = 0usize;
        let mut best_d = f64::MAX;
        for i in 0..gni {
            let d = (xc[i] - x).abs();
            if d < best_d {
                best_d = d;
                best = i;
            }
        }
        let (_r, _uu, _vv, p) = cons_to_prim(&u, best * 4, gm1);
        cp[k] = (p - 1.0) / q;
    }
    (cp, residual)
}

fn fw_len(a: &[f64], id: usize) -> f64 {
    (a[id * 2] * a[id * 2] + a[id * 2 + 1] * a[id * 2 + 1]).sqrt()
}

fn cons_to_prim(u: &[f64], o: usize, gm1: f64) -> (f64, f64, f64, f64) {
    let r = u[o].max(1e-8);
    let uu = u[o + 1] / r;
    let vv = u[o + 2] / r;
    let p = (gm1 * (u[o + 3] - 0.5 * r * (uu * uu + vv * vv))).max(1e-8);
    (r, uu, vv, p)
}

fn hllc(
    r_l: f64,
    u_l: f64,
    v_l: f64,
    p_l: f64,
    r_r: f64,
    u_r: f64,
    v_r: f64,
    p_r: f64,
    ax: f64,
    ay: f64,
    gamma: f64,
) -> [f64; 4] {
    let area = (ax * ax + ay * ay).sqrt();
    if area < 1e-14 {
        return [0.0; 4];
    }
    let nx = ax / area;
    let ny = ay / area;
    let tx = -ny;
    let ty = nx;
    let un_l = u_l * nx + v_l * ny;
    let ut_l = u_l * tx + v_l * ty;
    let un_r = u_r * nx + v_r * ny;
    let ut_r = u_r * tx + v_r * ty;
    let a_l = (gamma * p_l / r_l).sqrt();
    let a_r = (gamma * p_r / r_r).sqrt();
    let e_l = p_l / (gamma - 1.0) + 0.5 * r_l * (u_l * u_l + v_l * v_l);
    let e_r = p_r / (gamma - 1.0) + 0.5 * r_r * (u_r * u_r + v_r * v_r);
    let sl = un_l.min(un_r) - a_l.max(a_r);
    let sr = un_l.max(un_r) + a_l.max(a_r);
    let den = r_l * (sl - un_l) - r_r * (sr - un_r);
    let sm = if den.abs() < 1e-14 {
        0.5 * (un_l + un_r)
    } else {
        (p_r - p_l + r_l * un_l * (sl - un_l) - r_r * un_r * (sr - un_r)) / den
    };
    let (fr, fun, fut, fe) = if sl >= 0.0 {
        flux(r_l, un_l, ut_l, p_l, e_l)
    } else if sr <= 0.0 {
        flux(r_r, un_r, ut_r, p_r, e_r)
    } else {
        let left = sm >= 0.0;
        let (r, un, ut, p, e, s) = if left {
            (r_l, un_l, ut_l, p_l, e_l, sl)
        } else {
            (r_r, un_r, ut_r, p_r, e_r, sr)
        };
        let coeff = (r * (s - un)) / (s - sm);
        let e_over = e / r + (sm - un) * (sm + p / (r * (s - un)));
        let (f0, f1, f2, f3) = flux(r, un, ut, p, e);
        (
            f0 + s * (coeff - r),
            f1 + s * (coeff * sm - r * un),
            f2 + s * (coeff * ut - r * ut),
            f3 + s * (coeff * e_over - e),
        )
    };
    [
        fr * area,
        (fun * nx + fut * tx) * area,
        (fun * ny + fut * ty) * area,
        fe * area,
    ]
}

fn flux(r: f64, un: f64, ut: f64, p: f64, e: f64) -> (f64, f64, f64, f64) {
    (r * un, r * un * un + p, r * un * ut, (e + p) * un)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wedge(theta_deg: f64, n: usize, length: f64) -> (Vec<f64>, Vec<f64>) {
        let x: Vec<f64> = (0..n).map(|i| length * i as f64 / (n - 1) as f64).collect();
        let z: Vec<f64> = x.iter().map(|v| v * (theta_deg * std::f64::consts::PI / 180.0).tan()).collect();
        (x, z)
    }

    #[test]
    fn legacy_grid_is_unchanged() {
        // Golden values from the shipped wasm (euler-wasm.ts, 88x28x42) on an 8 deg wedge wall, M=8, alpha=0.
        let (x, z) = wedge(8.0, 60, 4.0);
        let (cp, res) = solve_wall(&x, &z, 8.0, 1.4, 0.0);
        assert!((cp[5] - 0.08404571308354909).abs() < 1e-9, "cp[5] = {}", cp[5]);
        assert!((cp[30] - 0.08738305035709486).abs() < 1e-9, "cp[30] = {}", cp[30]);
        assert!((cp[55] - 0.08738305035315007).abs() < 1e-9, "cp[55] = {}", cp[55]);
        assert!((res - 7.204826897313447).abs() < 1e-9, "res = {}", res);
    }

    #[test]
    fn clustering_is_monotone_exact_ends_and_off_is_uniform() {
        let u = cluster_coordinates(12, 0.0, 0.0, 0.5, 0.1);
        for j in 0..=12 {
            assert_eq!(u[j], j as f64 / 12.0);
        }
        let c = cluster_coordinates(40, 2.0, 6.0, 0.3, 0.09);
        assert_eq!(c[0], 0.0);
        assert_eq!(c[40], 1.0);
        for j in 1..=40 {
            assert!(c[j] > c[j - 1], "not strictly increasing at {}", j);
        }
        // Finer spacing at the shock (s = 0.3) than far from it.
        let near = (0..40).filter(|&j| c[j] < 0.3 && c[j + 1] >= 0.3).map(|j| c[j + 1] - c[j]).next().unwrap();
        assert!(near < 0.5 * (1.0 / 40.0), "shock spacing {} not clustered", near);
    }

    #[test]
    fn clustering_matches_typescript_reference() {
        // clusterCoordinates(10, { wall: 2, shock: 6, shockPos: 0.3, shockWidth: 0.09 }) from refinement.ts
        let want = [
            0.0, 0.0913287796753181, 0.19645591394704617, 0.24861662385687627, 0.2831088829975087, 0.3140255579974827,
            0.3483595736980215, 0.40178301370276004, 0.5627519821242225, 0.7805946023081516, 1.0,
        ];
        let got = cluster_coordinates(10, 2.0, 6.0, 0.3, 0.09);
        for j in 0..=10 {
            assert!((got[j] - want[j]).abs() < 1e-12, "j={} got {} want {}", j, got[j], want[j]);
        }
    }

    #[test]
    fn refined_grid_matches_typescript_port() {
        // strip-euler.ts solveStrip: 64x20x150, wall 2, shock 6, width 0.09, shockS_i = 0.1 + 0.4 i / 64.
        let (x, z) = wedge(8.0, 60, 4.0);
        let ni = 64usize;
        let shock_s: Vec<f64> = (0..=ni).map(|i| 0.1 + 0.4 * i as f64 / ni as f64).collect();
        let grid = Grid { ni, nj: 20, iters: 150, wall_amp: 2.0, shock_amp: 6.0, shock_width: 0.09, shock_s: Some(shock_s) };
        let (cp, res) = solve_wall_grid(&x, &z, 8.0, 1.4, 0.0, &grid);
        assert!((cp[5] - 0.07180176310553892).abs() < 1e-9, "cp[5] = {}", cp[5]);
        assert!((cp[30] - 0.06338922395989954).abs() < 1e-9, "cp[30] = {}", cp[30]);
        assert!((cp[55] - 0.06307605241013038).abs() < 1e-9, "cp[55] = {}", cp[55]);
        assert!((res - 0.3041611260342723).abs() < 1e-9, "res = {}", res);
    }

    #[test]
    fn clustered_wedge_is_close_to_exact_oblique_shock() {
        // M=8, 8 deg wedge: exact oblique-shock Cp = 0.06556. A converged shock-clustered grid must land within 2 %.
        let (x, z) = wedge(8.0, 80, 1.0);
        let (ni, nj) = (88usize, 28usize);
        let beta = 13.5625f64.to_radians();
        let theta = 8.0f64.to_radians();
        let h_far = 0.55;
        let xmin = -0.12;
        let xmax = 1.02;
        let shock_s: Vec<f64> = (0..=ni)
            .map(|i| {
                let xc = xmin + (xmax - xmin) * i as f64 / ni as f64;
                let h = xc.max(0.0) * (beta.tan() - theta.tan());
                (h / h_far).max(0.04).min(0.92).powf(1.0 / WALL_EXP)
            })
            .collect();
        let grid = Grid { ni, nj, iters: 1000, wall_amp: 0.0, shock_amp: 6.0, shock_width: 0.09, shock_s: Some(shock_s) };
        let (cp, _res) = solve_wall_grid(&x, &z, 8.0, 1.4, 0.0, &grid);
        let mid: Vec<f64> = (0..80).filter(|&i| x[i] > 0.3 && x[i] < 0.8).map(|i| cp[i]).collect();
        let mean = mid.iter().sum::<f64>() / mid.len() as f64;
        let exact = 0.06556490633109267; // oblique shock, M=8, theta=8 deg, gamma=1.4
        assert!((mean - exact).abs() / exact < 0.02, "mean Cp {} vs exact {}", mean, exact);
    }

    #[test]
    fn euler_solve_ex_rejects_bad_args_and_matches_grid_call() {
        let (x, z) = wedge(8.0, 40, 1.0);
        let n = x.len();
        let ni = 32usize;
        let bytes = (n * 3 + ni + 1 + 16) * 8;
        let ptr = euler_alloc(bytes as i32);
        let buf = unsafe { std::slice::from_raw_parts_mut(ptr as *mut f64, n * 3 + ni + 1) };
        for i in 0..n {
            buf[2 * i] = x[i];
            buf[2 * i + 1] = z[i];
        }
        for i in 0..=ni {
            buf[3 * n + i] = 0.25;
        }
        assert_eq!(euler_solve_ex(ptr, n as i32, 8.0, 1.4, 0.0, 4, 10, 50, 0.0, 0.0, 0.09), -1.0);
        assert_eq!(euler_solve_ex(ptr, n as i32, 8.0, 1.4, 0.0, 32, 10, 0, 0.0, 0.0, 0.09), -1.0);
        let res = euler_solve_ex(ptr, n as i32, 8.0, 1.4, 0.0, 32, 10, 60, 1.0, 4.0, 0.09);
        assert!(res >= 0.0 && res.is_finite());
        let out = unsafe { std::slice::from_raw_parts(ptr.add(n * 16) as *const f64, n) }.to_vec();
        let grid = Grid { ni: 32, nj: 10, iters: 60, wall_amp: 1.0, shock_amp: 4.0, shock_width: 0.09, shock_s: Some(vec![0.25; 33]) };
        let (cp, res2) = solve_wall_grid(&x, &z, 8.0, 1.4, 0.0, &grid);
        assert_eq!(res, res2);
        for i in 0..n {
            assert_eq!(out[i], cp[i]);
        }
    }
}
