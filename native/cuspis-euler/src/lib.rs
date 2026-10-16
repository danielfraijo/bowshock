//! Coarse symmetry-plane Euler. First-order HLLC, slip wall, local dt.
//! Input in wasm memory: n pairs (x, z) of the wall, nose at the first point.
//! Output: n Cp values written immediately after the input.

const NI: usize = 88;
const NJ: usize = 28;
const ITERS: usize = 42;
const CFL: f64 = 0.5;

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
    let n = xw.len();
    let x_nose = xw[0];
    let x_te = xw[n - 1];
    let length = (x_te - x_nose).abs().max(1e-4);
    let sign = if zw[n / 2] < -1e-8 { -1.0 } else { 1.0 };
    let xmin = x_nose - 0.12 * length;
    let xmax = x_te + 0.02 * length;
    let h_far = 0.55 * length;
    let npx = NI + 1;
    let npz = NJ + 1;
    let mut px = vec![0.0; npx * npz];
    let mut pz = vec![0.0; npx * npz];
    for j in 0..npz {
        let s = j as f64 / NJ as f64;
        let eta = s.powf(1.55);
        for i in 0..npx {
            let xf = i as f64 / NI as f64;
            let x = xmin + (xmax - xmin) * xf;
            let zwall = if x <= x_nose { zw[0] } else { wall_z(x, xw, zw) };
            let id = j * npx + i;
            px[id] = x;
            pz[id] = zwall + sign * eta * h_far;
        }
    }

    let nc = NI * NJ;
    let mut vol = vec![0.0; nc];
    let mut fw = vec![0.0; nc * 2];
    let mut fe = vec![0.0; nc * 2];
    let mut fs = vec![0.0; nc * 2];
    let mut fnor = vec![0.0; nc * 2];
    let mut xc = vec![0.0; nc];
    for j in 0..NJ {
        for i in 0..NI {
            let sw = j * npx + i;
            let se = sw + 1;
            let nw = (j + 1) * npx + i;
            let ne = nw + 1;
            let id = j * NI + i;
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
    for _ in 0..ITERS {
        let mut res_sum = 0.0;
        for j in 0..NJ {
            for i in 0..NI {
                let id = j * NI + i;
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
                    let ol = ((j * NI + (i - 1)) * 4) as usize;
                    let (rr, ur, vr, pr) = cons_to_prim(&u, ol, gm1);
                    let f = hllc(r, uu, vv, pp, rr, ur, vr, pr, fw[id * 2], fw[id * 2 + 1], gamma);
                    take(f, &mut d0, &mut d1, &mut d2, &mut d3);
                }
                if i + 1 == NI {
                    let f = hllc(r, uu, vv, pp, r, uu, vv, pp, fe[id * 2], fe[id * 2 + 1], gamma);
                    take(f, &mut d0, &mut d1, &mut d2, &mut d3);
                } else {
                    let ol = ((j * NI + (i + 1)) * 4) as usize;
                    let (rr, ur, vr, pr) = cons_to_prim(&u, ol, gm1);
                    let f = hllc(r, uu, vv, pp, rr, ur, vr, pr, fe[id * 2], fe[id * 2 + 1], gamma);
                    take(f, &mut d0, &mut d1, &mut d2, &mut d3);
                }
                if j + 1 == NJ {
                    let f = hllc(r, uu, vv, pp, rho_inf, u_inf, v_inf, p_inf, fnor[id * 2], fnor[id * 2 + 1], gamma);
                    take(f, &mut d0, &mut d1, &mut d2, &mut d3);
                } else {
                    let ol = (((j + 1) * NI + i) * 4) as usize;
                    let (rr, ur, vr, pr) = cons_to_prim(&u, ol, gm1);
                    let f = hllc(r, uu, vv, pp, rr, ur, vr, pr, fnor[id * 2], fnor[id * 2 + 1], gamma);
                    take(f, &mut d0, &mut d1, &mut d2, &mut d3);
                }
                if j == 0 {
                    d1 += pp * fs[id * 2];
                    d2 += pp * fs[id * 2 + 1];
                } else {
                    let ol = (((j - 1) * NI + i) * 4) as usize;
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
        for i in 0..NI {
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
