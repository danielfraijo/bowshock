//! cuspis-panel: batch panel-pressure evaluator for the aero database.
//!
//! One mesh is loaded once, then many flight points (M, α, β, body rates) are evaluated in one call.
//! The per-panel loop is a 1:1 port of `panelAero` (src/lib/waverider/panel.ts, forces only) and of
//! `integrateShockExpansion` (src/lib/waverider/shockexp.ts, forces only), so coefficients agree with the
//! TypeScript path to round-off. Heating is not ported: the aero database never reads it.
//!
//! Per-point atmosphere / viscous scalars (q, V, body Cf, rarefaction weight, Hayes–Probstein p/p_inv)
//! are computed by the caller with the same TS functions panelAero uses and passed in per point, so the
//! atmosphere model is not duplicated here.
//!
//! Elevon deflection is a geometry rebuild (as in aerodb.ts): every deflection is its own loaded mesh.

pub mod gas;

use gas::{
    clamp, dahlem_buck_cp, hypot3, inv_prandtl_meyer, jmax, jmin, max_theta, newton_busemann_cp, newtonian_cp_max,
    oblique_p2p1, oblique_shock, p_over_p0, prandtl_meyer, vacuum_cp, beta_from_theta_m, BetaTab, GasCache, DEG, PI,
};
use std::collections::HashMap;
use std::rc::Rc;

/// SURFACE_ID in src/lib/waverider/types.ts
pub mod surf {
    pub const UPPER: u16 = 0;
    pub const LOWER: u16 = 1;
    pub const BASE: u16 = 2;
    pub const LEADING: u16 = 3;
    pub const SYMMETRY: u16 = 4;
    pub const INLET: u16 = 5;
    pub const NOZZLE: u16 = 6;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Method {
    Newtonian = 0,
    Tangent = 1,
    /// strip shock-expansion (shockexp.ts)
    Mixed = 2,
    Cbaero = 3,
}

impl Method {
    pub fn from_u32(v: u32) -> Option<Method> {
        match v {
            0 => Some(Method::Newtonian),
            1 => Some(Method::Tangent),
            2 => Some(Method::Mixed),
            3 => Some(Method::Cbaero),
            _ => None,
        }
    }
}

/// Per-call constants (same for every point of one mesh / one database).
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub method: Method,
    pub gamma: f64,
    /// CONE_FAMILIES.has(params.family)
    pub use_cone: bool,
    /// clamp(params.cgFrac, 0.2, 0.85) * params.length
    pub cg_x: f64,
    pub s_ref: f64,
    pub l_ref: f64,
}

/// One flight point. Layout of the flat input array (STRIDE_IN f64 per point).
pub const STRIDE_IN: usize = 12;
/// cl, cd, cy, cm, cn, cll, ca, ld, cnA, cdFric, cdBase, xCp
pub const STRIDE_OUT: usize = 12;

#[derive(Clone, Copy, Debug, Default)]
pub struct Point {
    /// flight Mach (before the 1.05 floor)
    pub mach: f64,
    pub alpha_deg: f64,
    pub beta_deg: f64,
    /// 0 = no rates (RateState undefined), 1 = rates present
    pub has_rates: bool,
    pub p: f64,
    pub q: f64,
    pub r: f64,
    /// atm.V
    pub v: f64,
    /// atm.q (dynamic pressure, Pa)
    pub qdyn: f64,
    /// bodyCf(atm.ReL * L, M, atm.T, Tw, γ)
    pub cf: f64,
    /// rarefactionWeight(knudsen(..)) (panel loop only)
    pub w_rare: f64,
    /// viscousInteractionPressure(viscousChi(..))
    pub p_visc: f64,
}

impl Point {
    pub fn from_slice(s: &[f64]) -> Point {
        Point {
            mach: s[0],
            alpha_deg: s[1],
            beta_deg: s[2],
            has_rates: s[3] != 0.0,
            p: s[4],
            q: s[5],
            r: s[6],
            v: s[7],
            qdyn: s[8],
            cf: s[9],
            w_rare: s[10],
            p_visc: s[11],
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Coeffs {
    pub cl: f64,
    pub cd: f64,
    pub cy: f64,
    pub cm: f64,
    pub cn: f64,
    pub cll: f64,
    pub ca: f64,
    pub ld: f64,
    pub cn_a: f64,
    pub cd_fric: f64,
    pub cd_base: f64,
    pub x_cp: f64,
}

impl Coeffs {
    pub fn write(&self, o: &mut [f64]) {
        o[0] = self.cl;
        o[1] = self.cd;
        o[2] = self.cy;
        o[3] = self.cm;
        o[4] = self.cn;
        o[5] = self.cll;
        o[6] = self.ca;
        o[7] = self.ld;
        o[8] = self.cn_a;
        o[9] = self.cd_fric;
        o[10] = self.cd_base;
        o[11] = self.x_cp;
    }
}

// ------------------------------------------------------------------ meshes

/// shockexp.ts `Face` (forces-only fields).
#[derive(Clone, Debug)]
pub struct Face {
    pub area: f64,
    pub n: [f64; 3],
    pub c: [f64; 3],
    pub surf: u16,
    grid: i32,
    i: i32,
    j: i32,
}

#[derive(Clone, Debug, Default)]
pub struct Faces {
    pub faces: Vec<Face>,
    /// indices into `faces`, each strip sorted by i
    pub strips: Vec<Vec<usize>>,
    pub loose: Vec<usize>,
}

/// Raw triangle mesh (TriMesh), needed only for the shock-expansion face merge.
#[derive(Clone, Debug, Default)]
pub struct RawMesh {
    pub positions: Vec<f64>,
    pub indices: Vec<u32>,
    pub surfaces: Vec<u8>,
    pub tags: Option<Vec<i32>>,
}

/// One loaded vehicle: panelGeom (panel.ts) arrays plus the raw mesh for shock-expansion.
#[derive(Clone, Debug, Default)]
pub struct PanelMesh {
    pub area: Vec<f64>,
    pub nx: Vec<f64>,
    pub ny: Vec<f64>,
    pub nz: Vec<f64>,
    pub cx: Vec<f64>,
    pub cy: Vec<f64>,
    pub cz: Vec<f64>,
    /// surface id after panelGeom's aft-facing base reclassification
    pub surf: Vec<u16>,
    pub a_wet: f64,
    pub raw: Option<RawMesh>,
    faces: Option<Rc<Faces>>,
}

#[inline]
fn norm3(a: [f64; 3]) -> [f64; 3] {
    let m = hypot3(a[0], a[1], a[2]);
    let m = if m == 0.0 || m.is_nan() { 1.0 } else { m };
    [a[0] / m, a[1] / m, a[2] / m]
}

impl PanelMesh {
    /// Same as panel.ts `panelGeom` (for tests and native callers; the wasm bridge passes TS's own arrays).
    pub fn from_raw(raw: RawMesh) -> PanelMesh {
        let nt = raw.indices.len() / 3;
        let mut m = PanelMesh {
            area: vec![0.0; nt],
            nx: vec![0.0; nt],
            ny: vec![0.0; nt],
            nz: vec![0.0; nt],
            cx: vec![0.0; nt],
            cy: vec![0.0; nt],
            cz: vec![0.0; nt],
            surf: vec![0; nt],
            a_wet: 0.0,
            raw: None,
            faces: None,
        };
        let pos = &raw.positions;
        for t in 0..nt {
            let ia = raw.indices[t * 3] as usize * 3;
            let ib = raw.indices[t * 3 + 1] as usize * 3;
            let ic = raw.indices[t * 3 + 2] as usize * 3;
            let (x1, y1, z1) = (pos[ia], pos[ia + 1], pos[ia + 2]);
            let (x2, y2, z2) = (pos[ib], pos[ib + 1], pos[ib + 2]);
            let (x3, y3, z3) = (pos[ic], pos[ic + 1], pos[ic + 2]);
            let (abx, aby, abz) = (x2 - x1, y2 - y1, z2 - z1);
            let (acx, acy, acz) = (x3 - x1, y3 - y1, z3 - z1);
            let crx = aby * acz - abz * acy;
            let cry = abz * acx - abx * acz;
            let crz = abx * acy - aby * acx;
            let mag = hypot3(crx, cry, crz);
            let mut s = *raw.surfaces.get(t).unwrap_or(&0) as u16;
            if mag < 1e-16 {
                m.surf[t] = s;
                continue;
            }
            let inv = 1.0 / mag;
            let nnx = crx * inv;
            m.nx[t] = nnx;
            m.ny[t] = cry * inv;
            m.nz[t] = crz * inv;
            m.area[t] = 0.5 * mag;
            m.cx[t] = (x1 + x2 + x3) / 3.0;
            m.cy[t] = (y1 + y2 + y3) / 3.0;
            m.cz[t] = (z1 + z2 + z3) / 3.0;
            if nnx > 0.72 && s != surf::LEADING && s != surf::INLET {
                s = surf::BASE;
            }
            m.surf[t] = s;
            if s != surf::BASE && s != surf::NOZZLE && s != surf::INLET && s != surf::SYMMETRY {
                m.a_wet += m.area[t];
            }
        }
        m.raw = Some(raw);
        m
    }

    pub fn nt(&self) -> usize {
        self.area.len()
    }

    fn faces(&mut self) -> Rc<Faces> {
        if let Some(f) = &self.faces {
            return f.clone();
        }
        let f = Rc::new(match &self.raw {
            Some(r) => build_faces(r),
            None => Faces::default(),
        });
        self.faces = Some(f.clone());
        f
    }
}

/// shockexp.ts `buildFaces`: merge the two triangles of each tagged loft quad (step-by-step
/// area-weighted normal and centroid, in triangle order), group by (grid, j) in first-seen order,
/// sort each strip by i. Untagged triangles are loose faces.
pub fn build_faces(m: &RawMesh) -> Faces {
    let nt = m.indices.len() / 3;
    let mut faces: Vec<Face> = Vec::new();
    let mut by_tag: HashMap<i32, usize> = HashMap::new();
    let mut tagged: Vec<usize> = Vec::new();
    let mut loose: Vec<usize> = Vec::new();
    let p = &m.positions;
    let get = |i: u32| -> [f64; 3] {
        let o = i as usize * 3;
        [p[o], p[o + 1], p[o + 2]]
    };
    for t in 0..nt {
        let a = get(m.indices[t * 3]);
        let b = get(m.indices[t * 3 + 1]);
        let c = get(m.indices[t * 3 + 2]);
        let abx = b[0] - a[0];
        let aby = b[1] - a[1];
        let abz = b[2] - a[2];
        let acx = c[0] - a[0];
        let acy = c[1] - a[1];
        let acz = c[2] - a[2];
        let nx = aby * acz - abz * acy;
        let ny = abz * acx - abx * acz;
        let nz = abx * acy - aby * acx;
        let mag = hypot3(nx, ny, nz);
        if mag < 1e-16 {
            continue;
        }
        let area = 0.5 * mag;
        let n = norm3([nx, ny, nz]);
        let ctr = [(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0, (a[2] + b[2] + c[2]) / 3.0];
        let s = *m.surfaces.get(t).unwrap_or(&0) as u16;
        let tag = match &m.tags {
            Some(tg) => *tg.get(t).unwrap_or(&-1),
            None => -1,
        };
        if tag < 0 {
            loose.push(faces.len());
            faces.push(Face { area, n, c: ctr, surf: s, grid: -1, i: 0, j: 0 });
            continue;
        }
        match by_tag.get(&tag) {
            None => {
                by_tag.insert(tag, faces.len());
                tagged.push(faces.len());
                faces.push(Face { area, n, c: ctr, surf: s, grid: tag >> 24, i: (tag >> 12) & 4095, j: tag & 4095 });
            }
            Some(&k) => {
                let prev = &mut faces[k];
                let w = prev.area + area;
                prev.n = norm3([
                    prev.n[0] * prev.area + n[0] * area,
                    prev.n[1] * prev.area + n[1] * area,
                    prev.n[2] * prev.area + n[2] * area,
                ]);
                prev.c = [
                    (prev.c[0] * prev.area + ctr[0] * area) / w,
                    (prev.c[1] * prev.area + ctr[1] * area) / w,
                    (prev.c[2] * prev.area + ctr[2] * area) / w,
                ];
                prev.area = w;
            }
        }
    }
    let mut group_of: HashMap<(i32, i32), usize> = HashMap::new();
    let mut strips: Vec<Vec<usize>> = Vec::new();
    for &k in &tagged {
        let key = (faces[k].grid, faces[k].j);
        match group_of.get(&key) {
            Some(&g) => strips[g].push(k),
            None => {
                group_of.insert(key, strips.len());
                strips.push(vec![k]);
            }
        }
    }
    for s in strips.iter_mut() {
        s.sort_by_key(|&k| faces[k].i); // stable, like Array.prototype.sort
    }
    Faces { faces, strips, loose }
}

// ------------------------------------------------------------------ shockexp gas tables

struct GasTables {
    gamma: f64,
    mach: Vec<f64>,
    theta: Vec<f64>,
    p2: Vec<f64>,
    g2: Vec<f64>,
    m2: Vec<f64>,
}

fn gas_tables_new(gamma: f64) -> GasTables {
    let n_m = 81usize;
    let n_t = 101usize;
    let mut mach = vec![0.0; n_m];
    let mut theta = vec![0.0; n_t];
    let mut p2 = vec![0.0; n_m * n_t];
    let mut g2 = vec![0.0; n_m * n_t];
    let mut m2 = vec![0.0; n_m * n_t];
    for (i, m) in mach.iter_mut().enumerate() {
        *m = 1.05 + (22.0 - 1.05) * (i as f64 / (n_m - 1) as f64);
    }
    for (j, t) in theta.iter_mut().enumerate() {
        *t = ((50.0 * PI) / 180.0) * (j as f64 / (n_t - 1) as f64);
    }
    for i in 0..n_m {
        for j in 0..n_t {
            let o = i * n_t + j;
            if theta[j] < 1e-6 {
                p2[o] = 1.0;
                g2[o] = (gamma * mach[i] * mach[i]) / (mach[i] * mach[i] - 1.0).sqrt();
                m2[o] = mach[i];
                continue;
            }
            let beta = beta_from_theta_m(mach[i], theta[j], gamma);
            if !beta.is_finite() {
                p2[o] = -1.0;
                m2[o] = -1.0;
                continue;
            }
            let sh = oblique_shock(mach[i], beta, gamma);
            p2[o] = sh.p2p1;
            g2[o] = (sh.p2p1 - 1.0) / theta[j];
            m2[o] = sh.m2;
        }
    }
    GasTables { gamma, mach, theta, p2, g2, m2 }
}

#[derive(Clone, Copy)]
struct Sh {
    p2p1: f64,
    m2: f64,
}

/// shockexp.ts `sample2`
fn sample2(tab: &GasTables, m: f64, th: f64) -> Option<Sh> {
    let mach = &tab.mach;
    let theta = &tab.theta;
    if th <= 1e-8 {
        return Some(Sh { p2p1: 1.0, m2: m });
    }
    if m < mach[0] || th > theta[theta.len() - 1] {
        return None;
    }
    if m > mach[mach.len() - 1] {
        let b = beta_from_theta_m(m, th, tab.gamma);
        if !b.is_finite() {
            return None;
        }
        let sh = oblique_shock(m, b, tab.gamma);
        return Some(Sh { p2p1: sh.p2p1, m2: jmax(sh.m2, 1.01) });
    }
    let mut i1 = 1usize;
    while i1 < mach.len() - 1 && mach[i1] < m {
        i1 += 1;
    }
    let i0 = i1 - 1;
    let mut j1 = 1usize;
    while j1 < theta.len() - 1 && theta[j1] < th {
        j1 += 1;
    }
    let j0 = j1 - 1;
    let or1 = |v: f64| if v == 0.0 || v.is_nan() { 1.0 } else { v };
    let tx = (m - mach[i0]) / or1(mach[i1] - mach[i0]);
    let ty = (th - theta[j0]) / or1(theta[j1] - theta[j0]);
    let nt = theta.len();
    let a = tab.p2[i0 * nt + j0];
    let b = tab.p2[i1 * nt + j0];
    let c = tab.p2[i0 * nt + j1];
    let d = tab.p2[i1 * nt + j1];
    if a < 0.0 || b < 0.0 || c < 0.0 || d < 0.0 {
        return None;
    }
    let ga = tab.g2[i0 * nt + j0];
    let gb = tab.g2[i1 * nt + j0];
    let gc = tab.g2[i0 * nt + j1];
    let gd = tab.g2[i1 * nt + j1];
    let gv = ga * (1.0 - tx) * (1.0 - ty) + gb * tx * (1.0 - ty) + gc * (1.0 - tx) * ty + gd * tx * ty;
    let p = 1.0 + gv * th;
    let a2 = tab.m2[i0 * nt + j0];
    let b2 = tab.m2[i1 * nt + j0];
    let c2 = tab.m2[i0 * nt + j1];
    let d2 = tab.m2[i1 * nt + j1];
    let mm = a2 * (1.0 - tx) * (1.0 - ty) + b2 * tx * (1.0 - ty) + c2 * (1.0 - tx) * ty + d2 * tx * ty;
    Some(Sh { p2p1: p, m2: jmax(mm, 1.01) })
}

// ------------------------------------------------------------------ kernel

#[inline]
fn flow_dir(alpha: f64, beta: f64) -> [f64; 3] {
    let ca = alpha.cos();
    let sa = alpha.sin();
    let cb = beta.cos();
    let sb = beta.sin();
    [ca * cb, sb, sa * cb]
}

/// panel.ts / shockexp.ts `rotateForce` → (lift, drag, side)
#[inline]
fn rotate_force(f: [f64; 3], alpha: f64, beta: f64) -> (f64, f64, f64) {
    let ca = alpha.cos();
    let sa = alpha.sin();
    let cb = beta.cos();
    let sb = beta.sin();
    let x1 = cb * f[0] + sb * f[1];
    let y1 = -sb * f[0] + cb * f[1];
    let drag = ca * x1 + sa * f[2];
    let lift = -sa * x1 + ca * f[2];
    (lift, drag, y1)
}

#[inline]
fn sq(x: f64) -> f64 {
    x.powi(2)
}

/// Per-point constants of the panel loop (pure functions of M and γ, hoisted out of the triangle loop).
struct PanelCtx {
    m: f64,
    gamma: f64,
    cp_max: f64,
    th_max: f64,
    /// 0.5 γ M²
    q: f64,
    vac: f64,
    nu_m: f64,
    nu_max: f64,
    iso1_pow: f64,
    beta_tab: Rc<BetaTab>,
}

/// Raw panel sums before nondimensionalisation.
#[derive(Default)]
struct Sums {
    fx: f64,
    fy: f64,
    fz: f64,
    mx: f64,
    my: f64,
    mz: f64,
    fx_base: f64,
    a_wet: f64,
}

/// Holds the process-wide gas caches (math.ts module caches) and the loaded meshes.
pub struct Kernel {
    pub gas: GasCache,
    tables: HashMap<i64, Rc<GasTables>>,
    meshes: Vec<Option<PanelMesh>>,
}

impl Default for Kernel {
    fn default() -> Self {
        Self::new()
    }
}

impl Kernel {
    pub fn new() -> Kernel {
        Kernel { gas: GasCache::new(), tables: HashMap::new(), meshes: Vec::new() }
    }

    /// Returns a 1-based handle.
    pub fn load(&mut self, m: PanelMesh) -> u32 {
        for (i, slot) in self.meshes.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(m);
                return i as u32 + 1;
            }
        }
        self.meshes.push(Some(m));
        self.meshes.len() as u32
    }

    pub fn drop_mesh(&mut self, h: u32) {
        if h >= 1 && (h as usize) <= self.meshes.len() {
            self.meshes[h as usize - 1] = None;
        }
    }

    pub fn mesh_count(&self) -> usize {
        self.meshes.iter().filter(|m| m.is_some()).count()
    }

    fn gas_tables(&mut self, gamma: f64) -> Rc<GasTables> {
        let key = (gamma * 100.0).round() as i64;
        if let Some(t) = self.tables.get(&key) {
            return t.clone();
        }
        let t = Rc::new(gas_tables_new(gamma));
        self.tables.insert(key, t.clone());
        t
    }

    /// Evaluate `n` points. `inp` is n*STRIDE_IN, `out` is n*STRIDE_OUT.
    pub fn eval(&mut self, h: u32, cfg: &Config, inp: &[f64], out: &mut [f64]) -> Result<(), &'static str> {
        if h < 1 || h as usize > self.meshes.len() || self.meshes[h as usize - 1].is_none() {
            return Err("bad mesh handle");
        }
        let n = inp.len() / STRIDE_IN;
        if out.len() < n * STRIDE_OUT {
            return Err("output too small");
        }
        // take the mesh out so the gas caches can be borrowed mutably alongside it
        let mut mesh = self.meshes[h as usize - 1].take().unwrap();
        let faces = if cfg.method == Method::Mixed { Some(mesh.faces()) } else { None };
        for k in 0..n {
            let pt = Point::from_slice(&inp[k * STRIDE_IN..(k + 1) * STRIDE_IN]);
            let c = match &faces {
                Some(f) => self.shock_expansion(f, cfg, &pt),
                None => self.panel(&mesh, cfg, &pt),
            };
            c.write(&mut out[k * STRIDE_OUT..(k + 1) * STRIDE_OUT]);
        }
        self.meshes[h as usize - 1] = Some(mesh);
        Ok(())
    }

    pub fn eval_point(&mut self, h: u32, cfg: &Config, pt: &Point) -> Result<Coeffs, &'static str> {
        let inp = [
            pt.mach,
            pt.alpha_deg,
            pt.beta_deg,
            if pt.has_rates { 1.0 } else { 0.0 },
            pt.p,
            pt.q,
            pt.r,
            pt.v,
            pt.qdyn,
            pt.cf,
            pt.w_rare,
            pt.p_visc,
        ];
        let mut out = [0.0; STRIDE_OUT];
        self.eval(h, cfg, &inp, &mut out)?;
        Ok(Coeffs {
            cl: out[0],
            cd: out[1],
            cy: out[2],
            cm: out[3],
            cn: out[4],
            cll: out[5],
            ca: out[6],
            ld: out[7],
            cn_a: out[8],
            cd_fric: out[9],
            cd_base: out[10],
            x_cp: out[11],
        })
    }

    /// panel.ts `windwardCp` → (cp, attached). `tangentWedgeCp` is inlined: it re-ran fastBeta(M, θ, γ), which
    /// is the β already in hand, and only reads p2/p1 from obliqueShock.
    fn windward_cp(&mut self, x: &PanelCtx, theta: f64, method: Method, use_cone: bool) -> (f64, bool) {
        let (m, gamma, cp_max) = (x.m, x.gamma, x.cp_max);
        let sin2 = sq(theta.sin());
        if method == Method::Newtonian {
            return (cp_max * sin2, true);
        }
        let beta = x.beta_tab.beta(theta);
        let attached = beta.is_finite();
        let tangent = |k: &mut Kernel| -> f64 {
            if use_cone {
                return k.gas.cone_surface_state(m, theta, gamma).cp;
            }
            if theta <= 1e-5 {
                return 0.0;
            }
            (oblique_p2p1(m, beta, gamma) - 1.0) / x.q
        };
        if method == Method::Cbaero {
            let db = dahlem_buck_cp(theta, cp_max);
            let nb = newton_busemann_cp(theta, cp_max, gamma);
            if !attached {
                return (0.65 * cp_max * sin2 + 0.35 * nb, false);
            }
            let exact = tangent(self);
            let we = cbaero_attached_weight(theta, x.th_max);
            let rest = (0.25 * db + 0.2 * nb) / 0.45;
            return (we * exact + (1.0 - we) * rest, true);
        }
        if !attached {
            return (cp_max * sin2, false);
        }
        if method == Method::Tangent || method == Method::Mixed {
            return (tangent(self), true);
        }
        (cp_max * sin2, attached)
    }

    /// panel.ts `leewardCp` with its per-point constants (ν(M), ν(20), freestream isentropic term) hoisted.
    fn leeward_cp(&mut self, x: &PanelCtx, theta: f64) -> f64 {
        let gamma = x.gamma;
        let nu2 = x.nu_m + theta;
        if nu2 >= x.nu_max * 0.98 {
            return x.vac;
        }
        let mm = self.gas.fast_inv_pm(nu2, gamma);
        let iso2 = 1.0 + 0.5 * (gamma - 1.0) * mm * mm;
        let p2 = iso2.powf(-gamma / (gamma - 1.0)) / x.iso1_pow;
        clamp((p2 - 1.0) / x.q, x.vac, 0.2)
    }

    /// Forces-only port of panel.ts `panelAero` (methods newtonian / tangent / cbaero).
    fn panel(&mut self, g: &PanelMesh, cfg: &Config, pt: &Point) -> Coeffs {
        let m = jmax(1.05, if pt.mach.is_finite() { pt.mach } else { 1.05 });
        let gam = cfg.gamma;
        let method = cfg.method;
        let alpha = pt.alpha_deg * DEG;
        let beta = pt.beta_deg * DEG;
        let vhat = flow_dir(alpha, beta);
        let cp_max = newtonian_cp_max(m, gam);
        let cg = [cfg.cg_x, 0.0, 0.0];
        let th_max = if method == Method::Cbaero { max_theta(m, gam).0 } else { 0.0 };
        let iso1 = 1.0 + 0.5 * (gam - 1.0) * m * m;
        let ctx = PanelCtx {
            m,
            gamma: gam,
            cp_max,
            th_max,
            q: 0.5 * gam * m * m,
            vac: vacuum_cp(m, gam),
            nu_m: prandtl_meyer(m, gam),
            nu_max: prandtl_meyer(20.0, gam),
            iso1_pow: iso1.powf(-gam / (gam - 1.0)),
            beta_tab: self.gas.beta_tab(m, gam),
        };
        let w_rare = pt.w_rare;
        let p_visc = pt.p_visc;
        let rates = pt.has_rates && pt.v > 1.0;
        let qd = 0.5 * gam * m * m;
        let mut s = Sums { a_wet: g.a_wet, ..Default::default() };
        for t in 0..g.nt() {
            let area = g.area[t];
            if area == 0.0 {
                continue;
            }
            let nnx = g.nx[t];
            let nny = g.ny[t];
            let nnz = g.nz[t];
            let rx = g.cx[t] - cg[0];
            let ry = g.cy[t];
            let rz = g.cz[t];
            let (mut vhx, mut vhy, mut vhz) = (vhat[0], vhat[1], vhat[2]);
            if rates {
                let vx = pt.v * vhat[0] - (pt.q * rz - pt.r * ry);
                let vy = pt.v * vhat[1] - (pt.r * rx - pt.p * rz);
                let vz = pt.v * vhat[2] - (pt.p * ry - pt.q * rx);
                let vm = hypot3(vx, vy, vz);
                let vm = if vm == 0.0 || vm.is_nan() { 1.0 } else { vm };
                vhx = vx / vm;
                vhy = vy / vm;
                vhz = vz / vm;
            }
            let ndv = nnx * vhx + nny * vhy + nnz * vhz;
            let sinth = clamp(-ndv, 0.0, 1.0);
            let sf = g.surf[t];
            let is_base = sf == surf::BASE || sf == surf::NOZZLE;
            let windward = ndv < 0.0 && !is_base;
            let mut apply_visc = false;
            let mut cpi;
            if is_base {
                cpi = -1.0 / (m * m);
            } else if !windward {
                let theta = clamp(ndv, 0.0, 1.0).asin();
                cpi = if method == Method::Newtonian { 0.0 } else { self.leeward_cp(&ctx, theta) };
            } else {
                let theta = sinth.asin();
                let (c, att) = self.windward_cp(&ctx, theta, method, cfg.use_cone);
                cpi = c;
                apply_visc = att && method != Method::Newtonian;
            }
            if windward && apply_visc {
                let visc_cp = ((1.0 + cpi * 0.5 * gam * m * m) * p_visc - 1.0) / (0.5 * gam * m * m);
                cpi = jmin(visc_cp, jmax(cpi, cp_max));
            }
            if w_rare > 1e-4 {
                let fm = 2.0 * sinth * sinth;
                cpi = (1.0 - w_rare) * cpi + w_rare * fm;
            }
            let dfx = -cpi * area * nnx;
            let dfy = -cpi * area * nny;
            let dfz = -cpi * area * nnz;
            s.fx += dfx;
            s.fy += dfy;
            s.fz += dfz;
            if is_base {
                s.fx_base += dfx;
            }
            s.mx += ry * dfz - rz * dfy;
            s.my += rz * dfx - rx * dfz;
            s.mz += rx * dfy - ry * dfx;
        }
        let _ = qd;
        finish(&s, cfg, pt, m, alpha, beta, cg)
    }

    /// Forces-only port of shockexp.ts `integrateShockExpansion`.
    fn shock_expansion(&mut self, faces: &Faces, cfg: &Config, pt: &Point) -> Coeffs {
        let m = jmax(1.05, pt.mach);
        let g = cfg.gamma;
        let method = cfg.method;
        let alpha = pt.alpha_deg * DEG;
        let beta = pt.beta_deg * DEG;
        let vhat = flow_dir(alpha, beta);
        let cp_max = newtonian_cp_max(m, g);
        let q_inf = 0.5 * g * m * m;
        let vac = vacuum_cp(m, g);
        let tab = self.gas_tables(g);
        let use_cone = cfg.use_cone;
        let cg = [cfg.cg_x, 0.0, 0.0];
        let p_visc = pt.p_visc;
        let rates = pt.has_rates && pt.v > 1.0;
        let mut s = Sums::default();
        let is_newt = method == Method::Newtonian;

        let local_velocity = |f: &Face| -> [f64; 3] {
            if !rates {
                return vhat;
            }
            let rx = f.c[0] - cg[0];
            let ry = f.c[1] - cg[1];
            let rz = f.c[2] - cg[2];
            let vx = pt.v * vhat[0] - (pt.q * rz - pt.r * ry);
            let vy = pt.v * vhat[1] - (pt.r * rx - pt.p * rz);
            let vz = pt.v * vhat[2] - (pt.p * ry - pt.q * rx);
            norm3([vx, vy, vz])
        };

        let accum = |s: &mut Sums, f: &Face, cpi_in: f64, v: [f64; 3], visc: bool| {
            let ndv0 = f.n[0] * v[0] + f.n[1] * v[1] + f.n[2] * v[2];
            let surf_base = f.surf == surf::BASE || f.surf == surf::NOZZLE;
            let cpi = if visc && !is_newt && ndv0 < -0.02 && !surf_base && q_inf > 0.0 {
                jmin(((1.0 + cpi_in * q_inf) * p_visc - 1.0) / q_inf, jmax(cpi_in, cp_max))
            } else {
                cpi_in
            };
            let dfx = -cpi * f.area * f.n[0];
            let dfy = -cpi * f.area * f.n[1];
            let dfz = -cpi * f.area * f.n[2];
            s.fx += dfx;
            s.fy += dfy;
            s.fz += dfz;
            if surf_base {
                s.fx_base += dfx;
            }
            let rx = f.c[0] - cg[0];
            let ry = f.c[1] - cg[1];
            let rz = f.c[2] - cg[2];
            s.mx += ry * dfz - rz * dfy;
            s.my += rz * dfx - rx * dfz;
            s.mz += rx * dfy - ry * dfx;
            if !surf_base && f.surf != surf::INLET && f.surf != surf::SYMMETRY {
                s.a_wet += f.area;
            }
        };

        // shockexp `impact` → (cp, attached)
        let impact = |k: &mut Kernel, inc: f64, v_m: f64| -> (f64, bool) {
            if inc <= 1e-5 {
                if is_newt || inc > -1e-5 {
                    return (0.0, true);
                }
                let nu = prandtl_meyer(v_m, g);
                let nu2 = nu + -inc;
                if nu2 >= prandtl_meyer(20.0, g) * 0.98 {
                    return (vac, true);
                }
                let mm = inv_prandtl_meyer(nu2, g);
                let p2 = p_over_p0(mm, g) / p_over_p0(v_m, g);
                return (clamp((p2 - 1.0) / q_inf, vac, 0.2), true);
            }
            if is_newt {
                return (cp_max * sq(inc.sin()), true);
            }
            if use_cone {
                if sample2(&tab, v_m, inc).is_none() {
                    return (cp_max * sq(inc.sin()), false);
                }
                let st = k.gas.cone_surface_state(m, inc, g);
                return (st.cp, true);
            }
            match sample2(&tab, v_m, inc) {
                None => (cp_max * sq(inc.sin()), false),
                Some(sh) => ((sh.p2p1 - 1.0) / (0.5 * g * v_m * v_m), true),
            }
        };

        let cos_smooth = (10.0 * DEG).cos();
        for strip in &faces.strips {
            let mut m_loc = m;
            let mut p_ratio = 1.0f64;
            let mut vcur: Option<[f64; 3]> = None;
            let mut virgin = true;
            let mut marching = method == Method::Mixed;
            for fi in 0..strip.len() {
                let face = &faces.faces[strip[fi]];
                let v = local_velocity(face);
                let vf = match (marching, vcur) {
                    (true, Some(c)) => c,
                    _ => v,
                };
                let inc = clamp(-(face.n[0] * vf[0] + face.n[1] * vf[1] + face.n[2] * vf[2]), -1.0, 1.0).asin();
                let cpi;
                let mut visc = true;
                if !marching {
                    let (c, att) = impact(self, inc, m);
                    cpi = c;
                    if !att {
                        visc = false;
                    }
                } else if face.surf == surf::BASE || face.surf == surf::NOZZLE {
                    cpi = -1.0 / (m * m);
                } else {
                    let dth = inc;
                    if dth > 0.15 * DEG {
                        let from_free = virgin;
                        virgin = false;
                        match sample2(&tab, m_loc, dth) {
                            None => {
                                cpi = cp_max * sq(jmax(inc, 0.0).sin());
                                marching = false;
                                visc = false;
                            }
                            Some(sh) => {
                                if use_cone && from_free {
                                    let st = self.gas.cone_surface_state(m, jmax(dth, 1e-4), g);
                                    p_ratio = st.p_ratio;
                                    m_loc = st.m;
                                    cpi = st.cp;
                                } else {
                                    p_ratio *= sh.p2p1;
                                    m_loc = sh.m2;
                                    cpi = jmin((p_ratio - 1.0) / q_inf, cp_max);
                                    if p_ratio > 1.0 + cp_max * q_inf {
                                        p_ratio = 1.0 + cp_max * q_inf;
                                    }
                                }
                            }
                        }
                    } else {
                        if dth.abs() > 1e-9 {
                            // shockexp `expand`
                            let m1 = jmax(m_loc, 1.001);
                            let nu2 = prandtl_meyer(m1, g) + -dth;
                            if nu2 > 0.0 {
                                let m2 = jmin(self.gas.fast_inv_pm(nu2, g), 40.0);
                                let ratio = p_over_p0(m2, g) / jmax(1e-12, p_over_p0(m1, g));
                                p_ratio *= ratio;
                                m_loc = m2;
                            }
                        }
                        cpi = clamp((p_ratio - 1.0) / q_inf, vac, 2.0);
                    }
                    if dth.abs() > 0.15 * DEG {
                        virgin = false;
                    }
                    // stripTangent
                    let tdir = strip_tangent(faces, strip, fi, face.n, cos_smooth);
                    vcur = Some(match tdir {
                        Some(t) => t,
                        None => {
                            let sn = -(vf[0] * face.n[0] + vf[1] * face.n[1] + vf[2] * face.n[2]);
                            norm3([vf[0] + sn * face.n[0], vf[1] + sn * face.n[1], vf[2] + sn * face.n[2]])
                        }
                    });
                }
                accum(&mut s, face, cpi, v, visc);
            }
        }
        for &k in &faces.loose {
            let face = &faces.faces[k];
            let v = local_velocity(face);
            if face.surf == surf::BASE || face.surf == surf::NOZZLE {
                accum(&mut s, face, -1.0 / (m * m), v, true);
                continue;
            }
            let inc = clamp(-(face.n[0] * v[0] + face.n[1] * v[1] + face.n[2] * v[2]), -1.0, 1.0).asin();
            let (c, att) = impact(self, inc, m);
            accum(&mut s, face, c, v, att);
        }
        finish(&s, cfg, pt, m, alpha, beta, cg)
    }
}

fn strip_tangent(faces: &Faces, strip: &[usize], k: usize, n: [f64; 3], cos_smooth: f64) -> Option<[f64; 3]> {
    let ka = k.saturating_sub(1);
    let kb = (k + 1).min(strip.len() - 1);
    for kk in [ka, kb] {
        let mm = faces.faces[strip[kk]].n;
        if n[0] * mm[0] + n[1] * mm[1] + n[2] * mm[2] < cos_smooth {
            return None;
        }
    }
    let a = faces.faces[strip[ka]].c;
    let b = faces.faces[strip[kb]].c;
    let mut dx = b[0] - a[0];
    let mut dy = b[1] - a[1];
    let mut dz = b[2] - a[2];
    let dn = dx * n[0] + dy * n[1] + dz * n[2];
    dx -= dn * n[0];
    dy -= dn * n[1];
    dz -= dn * n[2];
    let m = hypot3(dx, dy, dz);
    if !(m > 1e-12) {
        return None;
    }
    let sgn = if dx < 0.0 { -1.0 } else { 1.0 };
    Some([(sgn * dx) / m, (sgn * dy) / m, (sgn * dz) / m])
}

/// panel.ts `cbaeroAttachedWeight`
fn cbaero_attached_weight(theta: f64, th_max: f64) -> f64 {
    let x = if th_max > 1e-6 { theta / th_max } else { 1.0 };
    if x <= 0.6 {
        return 1.0;
    }
    if x >= 1.0 {
        return 0.55;
    }
    let u = (x - 0.6) / 0.4;
    0.55 + 0.45 * (0.5 * (1.0 + (PI * u).cos()))
}

/// Integration tail shared by both loops (panel.ts lines after the loop; identical in shockexp.ts).
fn finish(s: &Sums, cfg: &Config, pt: &Point, _m: f64, alpha: f64, beta: f64, cg: [f64; 3]) -> Coeffs {
    let q = if pt.qdyn == 0.0 || pt.qdyn.is_nan() { 1.0 } else { pt.qdyn };
    let sr = jmax(cfg.s_ref, 1e-8);
    let l = jmax(cfg.l_ref, 1e-8);
    let (lift, drag0, side) = rotate_force([s.fx * q, s.fy * q, s.fz * q], alpha, beta);
    let swet = if s.a_wet > 0.0 { s.a_wet } else { sr * 2.4 };
    let dfric = pt.cf * q * swet;
    let drag = drag0 + dfric;
    let cl = lift / (q * sr);
    let cd = drag / (q * sr);
    let cd_base = ((s.fx_base).abs() * q) / (q * sr);
    let cd_fric = dfric / (q * sr);
    Coeffs {
        cl,
        cd,
        cy: side / (q * sr),
        cm: s.my / (sr * l),
        cn: s.mz / (sr * l),
        cll: s.mx / (sr * l),
        ld: if cd > 1e-10 { cl / cd } else { 0.0 },
        ca: (s.fx * q) / (q * sr),
        cn_a: (s.fz * q) / (q * sr),
        cd_fric,
        cd_base,
        x_cp: if s.fz.abs() > 1e-14 { clamp(cg[0] - s.my / s.fz, 0.0, l) } else { cg[0] },
    }
}

// ------------------------------------------------------------------ C ABI (wasm)

mod ffi;
pub use ffi::*;
