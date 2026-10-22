//! cargo test for cuspis-panel: gas-dynamics anchors, analytic panel checks, face merge, batch API, C ABI.
use cuspis_panel::gas::{self, GasCache, DEG};
use cuspis_panel::*;

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * (1.0 + b.abs())
}

// ---------------------------------------------------------------- gas anchors (NACA 1135 / Anderson)

#[test]
fn oblique_shock_m2_theta10() {
    let beta = gas::beta_from_theta_m(2.0, 10.0 * DEG, 1.4);
    assert!(close(beta / DEG, 39.31, 2e-3), "beta {}", beta / DEG);
    let sh = gas::oblique_shock(2.0, beta, 1.4);
    assert!(close(sh.p2p1, 1.7066, 1e-3), "p2p1 {}", sh.p2p1);
    assert!(close(sh.m2, 1.6405, 2e-3), "M2 {}", sh.m2);
    // fastBeta table agrees with the bisection away from detachment
    let mut gc = GasCache::new();
    let fb = gc.beta_tab(2.0, 1.4).beta(10.0 * DEG);
    assert!((fb - beta).abs() < 0.05 * DEG);
    // detached past theta_max (~22.97 deg at M 2)
    assert!(gc.beta_tab(2.0, 1.4).beta(24.0 * DEG).is_nan());
    let (tmax, _) = gas::max_theta(2.0, 1.4);
    assert!(close(tmax / DEG, 22.97, 3e-3), "theta max {}", tmax / DEG);
}

#[test]
fn prandtl_meyer_and_inverse() {
    assert!(close(gas::prandtl_meyer(2.0, 1.4) / DEG, 26.38, 1e-3));
    let mut gc = GasCache::new();
    for m in [1.2, 2.0, 3.5, 8.0, 20.0] {
        let nu = gas::prandtl_meyer(m, 1.4);
        assert!(close(gas::inv_prandtl_meyer(nu, 1.4), m, 1e-6), "inv {m}");
        assert!(close(gc.fast_inv_pm(nu, 1.4), m, 1e-6), "fast inv {m}");
    }
}

#[test]
fn newtonian_cp_max_and_cone() {
    // Cp_max → 1.839 as M → ∞ (γ = 1.4)
    assert!(close(gas::newtonian_cp_max(50.0, 1.4), 1.839, 2e-3));
    // 10° cone at M 2: shock ≈ 31.2°, Cp ≈ 0.1255 (NACA 1135 chart 5)
    let beta = gas::solve_cone_shock(2.0, 10.0 * DEG, 1.4);
    assert!(close(beta / DEG, 31.2, 5e-3), "cone beta {}", beta / DEG);
    let st = gas::cone_surface_state_exact(2.0, 10.0 * DEG, 1.4);
    assert!(close(st.cp, 0.1255, 2e-2), "cone cp {}", st.cp);
    let mut gc = GasCache::new();
    let cached = gc.cone_surface_state(2.0, 10.0 * DEG, 1.4);
    assert!(close(cached.cp, st.cp, 1e-6));
}

#[test]
fn hypot_matches_js_shape() {
    assert_eq!(gas::hypot3(3.0, 4.0, 12.0), 13.0);
    assert_eq!(gas::hypot3(0.0, 0.0, 0.0), 0.0);
    assert!(gas::hypot3(f64::NAN, 1.0, 0.0).is_nan());
    assert_eq!(gas::hypot3(f64::NAN, f64::INFINITY, 0.0), f64::INFINITY);
    assert!(gas::jmax(f64::NAN, 1.0).is_nan() && gas::jmin(1.0, f64::NAN).is_nan());
}

// ---------------------------------------------------------------- meshes

/// Flat rectangular plate in z = 0, chord c along x, span b, normal −z (windward at +α).
/// One tagged loft row (grid 0, j 0), `n` quads along x, two triangles each.
fn plate(c: f64, b: f64, n: usize, down: bool) -> RawMesh {
    let mut positions = Vec::new();
    for i in 0..=n {
        let x = c * i as f64 / n as f64;
        positions.extend_from_slice(&[x, -b / 2.0, 0.0, x, b / 2.0, 0.0]);
    }
    let mut indices = Vec::new();
    let mut tags = Vec::new();
    for i in 0..n {
        let a = (2 * i) as u32;
        let (p0, p1, p2, p3) = (a, a + 1, a + 2, a + 3);
        // (p0,p2,p1): normal = (x)×(y) = +z; flip for down
        if down {
            indices.extend_from_slice(&[p0, p1, p2, p1, p3, p2]);
        } else {
            indices.extend_from_slice(&[p0, p2, p1, p1, p2, p3]);
        }
        let tag = ((i as i32) << 12) | 0;
        tags.push(tag);
        tags.push(tag);
    }
    let nt = indices.len() / 3;
    RawMesh { positions, indices, surfaces: vec![if down { surf::LOWER as u8 } else { surf::UPPER as u8 }; nt], tags: Some(tags) }
}

fn point(mach: f64, alpha: f64) -> Point {
    Point { mach, alpha_deg: alpha, beta_deg: 0.0, has_rates: false, p: 0.0, q: 0.0, r: 0.0, v: 2000.0, qdyn: 1000.0, cf: 0.0, w_rare: 0.0, p_visc: 1.0 }
}

fn cfg(method: Method, s_ref: f64, l_ref: f64) -> Config {
    Config { method, gamma: 1.4, use_cone: false, cg_x: 0.0, s_ref, l_ref }
}

#[test]
fn newtonian_flat_plate_is_analytic() {
    let mut k = Kernel::new();
    let m = PanelMesh::from_raw(plate(2.0, 1.0, 4, true));
    assert!(m.surf.iter().all(|&s| s == surf::LOWER));
    let h = k.load(m);
    for alpha in [2.0f64, 5.0, 10.0, 20.0] {
        let c = k.eval_point(h, &cfg(Method::Newtonian, 2.0, 2.0), &point(10.0, alpha)).unwrap();
        let cpm = gas::newtonian_cp_max(10.0, 1.4);
        let a = alpha * DEG;
        let cn = cpm * a.sin().powi(2);
        assert!(close(c.cl, cn * a.cos(), 1e-12), "cl {} vs {}", c.cl, cn * a.cos());
        assert!(close(c.cd, cn * a.sin(), 1e-12));
        assert!(close(c.ld, 1.0 / a.tan(), 1e-12));
        // centre of pressure at mid-chord: Cm about the leading edge = −CN·(c/2)/Lref
        assert!(close(c.cm, -cn * 0.5, 1e-12), "cm {}", c.cm);
        assert!(c.cy.abs() < 1e-15 && c.cn.abs() < 1e-15 && c.cll.abs() < 1e-15);
    }
}

#[test]
fn tangent_wedge_flat_plate_is_oblique_shock() {
    let mut k = Kernel::new();
    let h = k.load(PanelMesh::from_raw(plate(1.0, 1.0, 3, true)));
    let (mach, alpha) = (6.0, 8.0);
    let c = k.eval_point(h, &cfg(Method::Tangent, 1.0, 1.0), &point(mach, alpha)).unwrap();
    let beta = gas::beta_from_theta_m(mach, alpha * DEG, 1.4);
    let cp = (gas::oblique_shock(mach, beta, 1.4).p2p1 - 1.0) / (0.5 * 1.4 * mach * mach);
    // CN from the windward face only (single-sided plate: no leeward panels)
    assert!(close(c.cn_a, cp, 5e-4), "cnA {} vs {}", c.cn_a, cp);
}

#[test]
fn shock_expansion_flat_plate_marches_one_shock() {
    let mut k = Kernel::new();
    let raw = plate(1.0, 1.0, 8, true);
    let faces = build_faces(&raw);
    assert_eq!(faces.strips.len(), 1);
    assert_eq!(faces.strips[0].len(), 8, "two triangles per quad merged");
    assert!(faces.loose.is_empty());
    let a0: f64 = faces.faces.iter().map(|f| f.area).sum();
    assert!(close(a0, 1.0, 1e-12));
    let h = k.load(PanelMesh::from_raw(raw));
    let (mach, alpha) = (8.0, 10.0);
    let c = k.eval_point(h, &cfg(Method::Mixed, 1.0, 1.0), &point(mach, alpha)).unwrap();
    let beta = gas::beta_from_theta_m(mach, alpha * DEG, 1.4);
    let cp = (gas::oblique_shock(mach, beta, 1.4).p2p1 - 1.0) / (0.5 * 1.4 * mach * mach);
    assert!(close(c.cn_a, cp, 2e-3), "shock-expansion Cp {} vs exact {}", c.cn_a, cp);
    // inviscid flat plate: L/D = cot α
    assert!(close(c.ld, 1.0 / (alpha * DEG).tan(), 1e-9));
}

#[test]
fn face_merge_is_area_weighted_and_sorted_by_i() {
    // two quads given out of i order, plus an untagged triangle
    let positions = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.1, 2.0, 0.0, 0.0, 2.0, 1.0, 0.0];
    // quad i=1: (1,4,3),(1,5,3)… keep simple: tag i=1 first, then tag i=0
    let indices = vec![1, 4, 3, 4, 5, 3, 0, 1, 2, 1, 3, 2, 0, 4, 5];
    let t1 = (1 << 12) | 7;
    let t0 = 7;
    let raw = RawMesh { positions, indices, surfaces: vec![1; 5], tags: Some(vec![t1, t1, t0, t0, -1]) };
    let f = build_faces(&raw);
    assert_eq!(f.strips.len(), 1);
    let s = &f.strips[0];
    assert_eq!(s.len(), 2);
    assert!(f.faces[s[0]].c[0] < f.faces[s[1]].c[0], "sorted by i");
    assert_eq!(f.loose.len(), 1);
    // area-weighted normal of the non-planar quad is unit length
    let n = f.faces[s[0]].n;
    assert!(close((n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt(), 1.0, 1e-14));
}

/// Closed, left/right symmetric double wedge (diamond airfoil extruded in span, capped).
fn diamond() -> RawMesh {
    let (c, b, t) = (1.0, 0.6, 0.08);
    let ys = [-b / 2.0, b / 2.0];
    let mut p = Vec::new();
    for &y in &ys {
        p.extend_from_slice(&[0.0, y, 0.0, 0.5 * c, y, t, c, y, 0.0, 0.5 * c, y, -t]);
    }
    // vertices: side L: 0 LE,1 top,2 TE,3 bottom ; side R: 4..7
    let quad = |a: u32, b2: u32, c2: u32, d: u32, out: &mut Vec<u32>| out.extend_from_slice(&[a, b2, c2, a, c2, d]);
    let mut ind = Vec::new();
    quad(0, 1, 5, 4, &mut ind); // upper front
    quad(1, 2, 6, 5, &mut ind); // upper aft
    quad(0, 4, 7, 3, &mut ind); // lower front
    quad(3, 7, 6, 2, &mut ind); // lower aft
    ind.extend_from_slice(&[0, 3, 2, 0, 2, 1]); // left cap
    ind.extend_from_slice(&[4, 5, 6, 4, 6, 7]); // right cap
    let nt = ind.len() / 3;
    let mut tags = vec![-1; nt];
    for (q, (i, j)) in [(0, 0), (1, 0), (0, 1), (1, 1)].iter().enumerate() {
        tags[2 * q] = (i << 12) | j;
        tags[2 * q + 1] = (i << 12) | j;
    }
    RawMesh { positions: p, indices: ind, surfaces: vec![0, 0, 0, 0, 1, 1, 1, 1, 4, 4, 4, 4], tags: Some(tags) }
}

#[test]
fn symmetric_body_has_no_lateral_forces_and_beta_is_antisymmetric() {
    let mut k = Kernel::new();
    let h = k.load(PanelMesh::from_raw(diamond()));
    for method in [Method::Newtonian, Method::Tangent, Method::Cbaero, Method::Mixed] {
        let cf = Config { cg_x: 0.5, ..cfg(method, 0.6, 1.0) };
        let c0 = k.eval_point(h, &cf, &point(6.0, 4.0)).unwrap();
        assert!(c0.cy.abs() < 1e-12 && c0.cn.abs() < 1e-12 && c0.cll.abs() < 1e-12, "{method:?} {c0:?}");
        let mut pp = point(6.0, 4.0);
        pp.beta_deg = 3.0;
        let cp = k.eval_point(h, &cf, &pp).unwrap();
        pp.beta_deg = -3.0;
        let cm = k.eval_point(h, &cf, &pp).unwrap();
        assert!(close(cp.cy, -cm.cy, 1e-9) && close(cp.cn, -cm.cn, 1e-9), "{method:?} antisym");
        assert!(close(cp.cl, cm.cl, 1e-9), "{method:?} CL even in beta");
    }
}

#[test]
fn pitch_rate_damps_and_batch_equals_single() {
    let mut k = Kernel::new();
    let h = k.load(PanelMesh::from_raw(diamond()));
    let cf = Config { cg_x: 0.5, ..cfg(Method::Newtonian, 0.6, 1.0) };
    let base = point(8.0, 6.0);
    let c0 = k.eval_point(h, &cf, &base).unwrap();
    let mut rq = base;
    rq.has_rates = true;
    rq.q = 40.0;
    let cq = k.eval_point(h, &cf, &rq).unwrap();
    assert!(cq.cm < c0.cm, "positive q gives a nose-down (damping) moment: {} vs {}", cq.cm, c0.cm);
    // batch of mixed points == point-by-point
    let pts = [base, rq, point(4.0, -2.0), point(12.0, 10.0)];
    let mut inp = Vec::new();
    for p in &pts {
        inp.extend_from_slice(&[p.mach, p.alpha_deg, p.beta_deg, if p.has_rates { 1.0 } else { 0.0 }, p.p, p.q, p.r, p.v, p.qdyn, p.cf, p.w_rare, p.p_visc]);
    }
    let mut out = vec![0.0; pts.len() * STRIDE_OUT];
    k.eval(h, &cf, &inp, &mut out).unwrap();
    for (i, p) in pts.iter().enumerate() {
        let c = k.eval_point(h, &cf, p).unwrap();
        assert_eq!(out[i * STRIDE_OUT], c.cl);
        assert_eq!(out[i * STRIDE_OUT + 3], c.cm);
    }
    assert!(k.eval(99, &cf, &inp, &mut out).is_err());
}

#[test]
fn c_abi_roundtrip() {
    let pm = PanelMesh::from_raw(diamond());
    let nt = pm.nt();
    let mut geom = Vec::new();
    for a in [&pm.area, &pm.nx, &pm.ny, &pm.nz, &pm.cx, &pm.cy, &pm.cz] {
        geom.extend_from_slice(a);
    }
    let raw = diamond();
    unsafe {
        let h = cp_mesh_load(nt as u32, geom.as_ptr(), pm.surf.as_ptr(), pm.a_wet, (raw.positions.len() / 3) as u32, raw.positions.as_ptr(), raw.indices.as_ptr(), raw.surfaces.as_ptr(), raw.tags.as_ref().unwrap().as_ptr());
        assert!(h >= 1);
        let p = point(6.0, 4.0);
        let inp = [p.mach, p.alpha_deg, 0.0, 0.0, 0.0, 0.0, 0.0, p.v, p.qdyn, 0.002, 0.0, 1.05];
        let mut out = [0.0; STRIDE_OUT];
        for method in 0..4 {
            assert_eq!(cp_eval(h, method, 1.4, 0, 0.5, 0.6, 1.0, 1, inp.as_ptr(), out.as_mut_ptr()), 0);
            assert!(out[0].is_finite() && out[1] > 0.0);
        }
        assert_eq!(cp_eval(h, 9, 1.4, 0, 0.5, 0.6, 1.0, 1, inp.as_ptr(), out.as_mut_ptr()), -2);
        cp_mesh_drop(h);
        assert_eq!(cp_eval(h, 0, 1.4, 0, 0.5, 0.6, 1.0, 1, inp.as_ptr(), out.as_mut_ptr()), -1);
        let b = cp_alloc(100);
        assert_eq!(b as usize % 8, 0);
        cp_free(b, 100);
    }
}
