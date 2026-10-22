//! Gas-dynamics helpers, ported 1:1 from src/lib/waverider/math.ts (same formulas, same tables, same
//! iteration counts) so the Rust batch evaluator agrees with the TS panel loop to round-off.
//! Every function names its TS counterpart.

use std::collections::HashMap;
use std::rc::Rc;

pub const PI: f64 = std::f64::consts::PI;
pub const DEG: f64 = PI / 180.0;

/// Math.max(a, Math.min(b, v)), NaN-propagating like JS.
#[inline]
pub fn clamp(v: f64, a: f64, b: f64) -> f64 {
    jmax(a, jmin(b, v))
}

/// JS Math.max (NaN-propagating).
#[inline]
pub fn jmax(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else if a > b {
        a
    } else {
        b
    }
}

/// JS Math.min (NaN-propagating).
#[inline]
pub fn jmin(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else if a < b {
        a
    } else {
        b
    }
}

/// Math.hypot(x, y, z) with V8's algorithm (scale by max, Kahan-compensated sum of squares).
#[inline]
pub fn hypot3(x: f64, y: f64, z: f64) -> f64 {
    let ax = x.abs();
    let ay = y.abs();
    let az = z.abs();
    if ax.is_infinite() || ay.is_infinite() || az.is_infinite() {
        return f64::INFINITY;
    }
    if ax.is_nan() || ay.is_nan() || az.is_nan() {
        return f64::NAN;
    }
    let mut m = ax;
    if ay > m {
        m = ay;
    }
    if az > m {
        m = az;
    }
    if m == 0.0 {
        return 0.0;
    }
    let mut sum = 0.0f64;
    let mut comp = 0.0f64;
    for v in [ax, ay, az] {
        let r = v / m;
        let s = r * r - comp;
        let p = sum + s;
        comp = (p - sum) - s;
        sum = p;
    }
    sum.sqrt() * m
}

/// thetaFromBetaM
pub fn theta_from_beta_m(m: f64, beta: f64, gamma: f64) -> f64 {
    let s = beta.sin();
    let c = beta.cos();
    if s.abs() < 1e-12 {
        return 0.0;
    }
    let m2 = m * m;
    let num = 2.0 * (c / s) * (m2 * s * s - 1.0);
    let den = m2 * (gamma + (2.0 * beta).cos()) + 2.0;
    (num / den).atan()
}

/// maxTheta → (theta, beta)
pub fn max_theta(m: f64, gamma: f64) -> (f64, f64) {
    let mu = clamp(1.0 / m, 0.0, 1.0).asin();
    let mut best_t = 0.0;
    let mut best_b = mu;
    let n = 80;
    for i in 1..n {
        let b = mu + ((PI / 2.0 - 1e-4 - mu) * i as f64) / n as f64;
        let t = theta_from_beta_m(m, b, gamma);
        if t > best_t {
            best_t = t;
            best_b = b;
        }
    }
    (best_t, best_b)
}

/// betaFromThetaM (weak shock, NaN if detached)
pub fn beta_from_theta_m(m: f64, theta: f64, gamma: f64) -> f64 {
    if theta <= 1e-10 {
        return clamp(1.0 / m, 0.0, 1.0).asin();
    }
    let (cap_t, cap_b) = max_theta(m, gamma);
    if theta >= cap_t * 0.999 {
        return f64::NAN;
    }
    let mu = clamp(1.0 / m, 0.0, 1.0).asin();
    let mut lo = mu + 1e-6;
    let mut hi = cap_b;
    for _ in 0..48 {
        let mid = 0.5 * (lo + hi);
        let t = theta_from_beta_m(m, mid, gamma);
        if t < theta {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// fastBeta table (math.ts `BetaTab`), built once per (M, γ).
pub struct BetaTab {
    m: f64,
    gamma: f64,
    cap_t: f64,
    cut: f64,
    b: Vec<f64>,
}

impl BetaTab {
    pub fn new(m: f64, gamma: f64) -> BetaTab {
        let (cap_t, cap_b) = max_theta(m, gamma);
        let n = 321usize;
        let cut = cap_t * 0.96;
        let mut b = vec![0.0; n];
        let mu = clamp(1.0 / m, 0.0, 1.0).asin();
        b[0] = mu;
        for (i, bi) in b.iter_mut().enumerate().skip(1) {
            let th = (cut * i as f64) / (n - 1) as f64;
            let mut lo = mu + 1e-8;
            let mut hi = cap_b;
            for _ in 0..40 {
                let mid = 0.5 * (lo + hi);
                if theta_from_beta_m(m, mid, gamma) < th {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            *bi = 0.5 * (lo + hi);
        }
        BetaTab { m, gamma, cap_t, cut, b }
    }

    /// fastBeta(M, theta, gamma)
    pub fn beta(&self, theta: f64) -> f64 {
        if theta <= 1e-10 {
            return clamp(1.0 / self.m, 0.0, 1.0).asin();
        }
        if theta >= self.cap_t * 0.999 {
            return f64::NAN;
        }
        if theta >= self.cut {
            return beta_from_theta_m(self.m, theta, self.gamma);
        }
        let n = self.b.len();
        let u = (theta / self.cut) * (n - 1) as f64;
        let i = jmin((n - 2) as f64, u.floor()) as usize;
        let f = u - i as f64;
        self.b[i] * (1.0 - f) + self.b[i + 1] * f
    }
}

pub struct Oblique {
    pub theta: f64,
    pub p2p1: f64,
    pub m2: f64,
}

/// obliqueShock (fields the force path needs)
pub fn oblique_shock(m: f64, beta: f64, gamma: f64) -> Oblique {
    let mn = m * beta.sin();
    let mn2 = mn * mn;
    let gp1 = gamma + 1.0;
    let gm1 = gamma - 1.0;
    let p2p1 = 1.0 + (2.0 * gamma) / gp1 * (mn2 - 1.0);
    let mn2sq = (mn2 + 2.0 / gm1) / ((2.0 * gamma) / gm1 * mn2 - 1.0);
    let theta = theta_from_beta_m(m, beta, gamma);
    let m2 = jmax(0.0, mn2sq).sqrt() / jmax(1e-6, beta - theta).sin();
    Oblique { theta, p2p1, m2 }
}

// ---------------------------------------------------------------- Taylor–Maccoll (math.ts)

pub struct TmTable {
    pub theta: Vec<f64>,
    pub vr: Vec<f64>,
    pub vt: Vec<f64>,
    pub cone: f64,
}

#[inline]
fn tm_deriv(vr: f64, vt: f64, th: f64, gamma: f64) -> f64 {
    let gm1h = (gamma - 1.0) / 2.0;
    let v2 = vr * vr + vt * vt;
    let a = gm1h * (1.0 - v2);
    let cot = th.cos() / jmax(1e-10, th.sin());
    let num = vt * vt * vr - a * (2.0 * vr + vt * cot);
    let den = a - vt * vt;
    if den == 0.0 {
        0.0
    } else {
        num / den
    }
}

/// taylorMaccoll(M, beta, gamma, stepScale)
pub fn taylor_maccoll(m: f64, beta: f64, gamma: f64, step_scale: f64) -> Option<TmTable> {
    if !(m > 1.0) || !(beta > 0.0) {
        return None;
    }
    let gm1 = gamma - 1.0;
    let v_inf = ((gm1 * 0.5 * m * m) / (1.0 + gm1 * 0.5 * m * m)).sqrt();
    let mn = m * beta.sin();
    let rho = ((gamma + 1.0) * mn * mn) / (gm1 * mn * mn + 2.0);
    let mut vr = v_inf * beta.cos();
    let mut vt = (-v_inf * beta.sin()) / rho;
    let mut th = beta;
    let mut theta = vec![th];
    let mut vr_a = vec![vr];
    let mut vt_a = vec![vt];
    let dth = -jmin(0.25 * DEG, beta / 80.0) * step_scale;
    let mut cone = 0.0;
    let max_steps = (4000.0 / step_scale).ceil() as i64;
    for _ in 0..max_steps {
        if th + dth <= 1e-3 {
            break;
        }
        let k1v = vt;
        let k1t = tm_deriv(vr, vt, th, gamma);
        let k2v = vt + 0.5 * dth * k1t;
        let k2t = tm_deriv(vr + 0.5 * dth * k1v, vt + 0.5 * dth * k1t, th + 0.5 * dth, gamma);
        let k3v = vt + 0.5 * dth * k2t;
        let k3t = tm_deriv(vr + 0.5 * dth * k2v, vt + 0.5 * dth * k2t, th + 0.5 * dth, gamma);
        let k4v = vt + dth * k3t;
        let k4t = tm_deriv(vr + dth * k3v, vt + dth * k3t, th + dth, gamma);
        vr += (dth / 6.0) * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
        vt += (dth / 6.0) * (k1t + 2.0 * k2t + 2.0 * k3t + k4t);
        th += dth;
        theta.push(th);
        vr_a.push(vr);
        vt_a.push(vt);
        if vt >= 0.0 {
            let n = vt_a.len();
            let t0 = theta[n - 2];
            let t1 = th;
            let h = t1 - t0;
            let v0 = vt_a[n - 2];
            let v1 = vt;
            let d0 = tm_deriv(vr_a[n - 2], v0, t0, gamma);
            let d1 = tm_deriv(vr, v1, t1, gamma);
            let herm = |u: f64| {
                let u2 = u * u;
                let u3 = u2 * u;
                (2.0 * u3 - 3.0 * u2 + 1.0) * v0 + (u3 - 2.0 * u2 + u) * h * d0 + (-2.0 * u3 + 3.0 * u2) * v1 + (u3 - u2) * h * d1
            };
            let mut lo = 0.0;
            let mut hi = 1.0;
            for _ in 0..50 {
                let mid = 0.5 * (lo + hi);
                if herm(mid) < 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let u_root = 0.5 * (lo + hi);
            cone = t0 + u_root * h;
            let r0 = vr_a[n - 2];
            let u2 = u_root * u_root;
            let u3 = u2 * u_root;
            let vr_c = (2.0 * u3 - 3.0 * u2 + 1.0) * r0 + (u3 - 2.0 * u2 + u_root) * h * v0 + (-2.0 * u3 + 3.0 * u2) * vr + (u3 - u2) * h * v1;
            theta[n - 1] = cone;
            vr_a[n - 1] = vr_c;
            vt_a[n - 1] = 0.0;
            break;
        }
    }
    if cone <= 0.0 || cone >= beta {
        return None;
    }
    if cone < 25.0 * -dth && step_scale > 2e-3 {
        return match taylor_maccoll(m, beta, gamma, step_scale * 0.1) {
            Some(t) => Some(t),
            None => Some(TmTable { theta, vr: vr_a, vt: vt_a, cone }),
        };
    }
    Some(TmTable { theta, vr: vr_a, vt: vt_a, cone })
}

fn tm_cone_angle(m: f64, beta: f64, gamma: f64) -> f64 {
    match taylor_maccoll(m, beta, gamma, 1.0) {
        Some(t) => t.cone,
        None => f64::NAN,
    }
}

/// solveConeShock
pub fn solve_cone_shock(m: f64, cone: f64, gamma: f64) -> f64 {
    let mu = clamp(1.0 / m, 0.0, 1.0).asin();
    let mut lo = jmax(mu * (1.0 + 1e-7), mu + 1e-7);
    let mut hi = 70.0 * DEG;
    let mut best = lo;
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        let c = tm_cone_angle(m, mid, gamma);
        if c.is_nan() {
            if mid < 0.5 * (mu + 70.0 * DEG) && cone > 1e-4 {
                lo = mid;
            } else {
                hi = mid;
            }
            continue;
        }
        best = mid;
        if c < cone {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    best
}

/// interpTable(table, th).vr
fn interp_vr(t: &TmTable, th: f64) -> f64 {
    let a = &t.theta;
    if th >= a[0] {
        return t.vr[0];
    }
    if th <= a[a.len() - 1] {
        return t.vr[a.len() - 1];
    }
    let mut lo = 0usize;
    let mut hi = a.len() - 1;
    while hi - lo > 1 {
        let m = (lo + hi) >> 1;
        if a[m] > th {
            lo = m;
        } else {
            hi = m;
        }
    }
    let u = (th - a[lo]) / (a[hi] - a[lo] + 1e-16);
    t.vr[lo] + (t.vr[hi] - t.vr[lo]) * u
}

#[derive(Clone, Copy, Debug)]
pub struct ConeState {
    pub cp: f64,
    pub p_ratio: f64,
    pub m: f64,
}

/// rayleighPitot
pub fn rayleigh_pitot(m: f64, g: f64) -> f64 {
    let m2 = m * m;
    let a = ((g + 1.0) * (g + 1.0) * m2) / (4.0 * g * m2 - 2.0 * (g - 1.0));
    let b = (1.0 - g + 2.0 * g * m2) / (g + 1.0);
    a.powf(g / (g - 1.0)) * b
}

/// newtonianCpMax
pub fn newtonian_cp_max(m: f64, gamma: f64) -> f64 {
    let q = 0.5 * gamma * m * m;
    (rayleigh_pitot(m, gamma) - 1.0) / q
}

/// vacuumCp
#[inline]
pub fn vacuum_cp(m: f64, gamma: f64) -> f64 {
    -2.0 / (gamma * m * m)
}

/// coneSurfaceStateExact
pub fn cone_surface_state_exact(m: f64, cone: f64, gamma: f64) -> ConeState {
    if cone <= 1e-9 {
        return ConeState { cp: 0.0, p_ratio: 1.0, m };
    }
    let q = 0.5 * gamma * m * m;
    let beta = solve_cone_shock(m, cone, gamma);
    let tm = match taylor_maccoll(m, beta, gamma, 1.0) {
        Some(t) => t,
        None => {
            let s = cone.sin();
            let cp = newtonian_cp_max(m, gamma) * (s * s);
            return ConeState { cp, p_ratio: 1.0 + cp * q, m: jmax(1.05, m * 0.6) };
        }
    };
    let sh = oblique_shock(m, beta, gamma);
    let v2 = tm.vr[0] * tm.vr[0] + tm.vt[0] * tm.vt[0];
    let t2t0 = jmax(1e-6, 1.0 - v2);
    let p02 = sh.p2p1 * t2t0.powf(-gamma / (gamma - 1.0));
    let vr = interp_vr(&tm, tm.cone);
    let tct0 = jmax(1e-6, 1.0 - vr * vr);
    let pc = p02 * tct0.powf(gamma / (gamma - 1.0));
    let cp = (pc - 1.0) / q;
    let msurf = vr / jmax(1e-12, 0.5 * (gamma - 1.0) * (1.0 - vr * vr)).sqrt();
    ConeState { cp, p_ratio: pc, m: jmax(1.01, msurf) }
}

/// prandtlMeyer
pub fn prandtl_meyer(m: f64, g: f64) -> f64 {
    if m <= 1.0 {
        return 0.0;
    }
    let k = ((g + 1.0) / (g - 1.0)).sqrt();
    k * (((g - 1.0) / (g + 1.0) * (m * m - 1.0)).sqrt()).atan() - (m * m - 1.0).sqrt().atan()
}

/// invPrandtlMeyer (finite-difference Newton, 24 iterations)
pub fn inv_prandtl_meyer(nu: f64, gamma: f64) -> f64 {
    if nu <= 0.0 {
        return 1.0;
    }
    let mut m = 1.5;
    for _ in 0..24 {
        let f = prandtl_meyer(m, gamma) - nu;
        let dm = 1e-4;
        let fp = (prandtl_meyer(m + dm, gamma) - prandtl_meyer(m, gamma)) / dm;
        let fp = if fp == 0.0 || fp.is_nan() { 1.0 } else { fp };
        m = jmax(1.0001, m - f / fp);
    }
    m
}

/// p/p0 = t^(−γ/(γ−1)) (shockexp `pOverP0`, panel `leewardCp` iso terms)
#[inline]
pub fn p_over_p0(m: f64, gamma: f64) -> f64 {
    let t = 1.0 + 0.5 * (gamma - 1.0) * m * m;
    t.powf(-gamma / (gamma - 1.0))
}

/// dahlemBuckCp
pub fn dahlem_buck_cp(theta: f64, cp_max: f64) -> f64 {
    let th = jmax(theta, 0.0);
    let s = th.sin();
    let tc = 22.5 * DEG;
    if th >= tc {
        return cp_max * s * s;
    }
    let n = 1.8 + 0.2 * (th / tc);
    let sc = tc.sin();
    cp_max * jmax(s, 1e-8).powf(n) * sc.powf(2.0 - n)
}

/// newtonBusemannCp
pub fn newton_busemann_cp(theta: f64, cp_max: f64, gamma: f64) -> f64 {
    if theta < 1e-5 {
        return 0.0;
    }
    let s = theta.sin();
    let c = theta.cos();
    let tcot = theta / theta.tan();
    let nb = s * s + ((3.0 + gamma) / (gamma + 1.0)) * (tcot - 1.0) * c * c;
    cp_max * clamp(nb, 0.0, 1.25)
}

struct NuTab {
    g: f64,
    nu: Vec<f64>,
    m0: f64,
    m1: f64,
}

pub const CONE_NODE: f64 = 0.1 * DEG;

/// `${M.toFixed(3)}` cache key (math.ts coneNode).
fn mach_key3(m: f64) -> i64 {
    (m * 1000.0).round() as i64
}

/// Process-wide gas caches, mirroring the module-level caches in math.ts
/// (fastBeta table, ν⁻¹ log table, 0.1° Taylor–Maccoll cone nodes).
pub struct GasCache {
    beta_tabs: HashMap<(u64, u64), Rc<BetaTab>>,
    nu_tab: Option<NuTab>,
    cone_nodes: HashMap<(i64, i64, u64), ConeState>,
}

impl GasCache {
    pub fn new() -> GasCache {
        GasCache { beta_tabs: HashMap::new(), nu_tab: None, cone_nodes: HashMap::new() }
    }

    pub fn beta_tab(&mut self, m: f64, gamma: f64) -> Rc<BetaTab> {
        let key = (m.to_bits(), gamma.to_bits());
        if let Some(t) = self.beta_tabs.get(&key) {
            return t.clone();
        }
        if self.beta_tabs.len() > 256 {
            self.beta_tabs.clear();
        }
        let t = Rc::new(BetaTab::new(m, gamma));
        self.beta_tabs.insert(key, t.clone());
        t
    }

    /// fastInvPrandtlMeyer
    pub fn fast_inv_pm(&mut self, nu: f64, gamma: f64) -> f64 {
        if nu <= 0.0 {
            return 1.0;
        }
        let rebuild = match &self.nu_tab {
            Some(t) => t.g != gamma,
            None => true,
        };
        if rebuild {
            let n = 257usize;
            let m0 = 1.0001f64;
            let m1 = 40.0f64;
            let nu_a: Vec<f64> = (0..n).map(|i| prandtl_meyer(m0 * (m1 / m0).powf(i as f64 / (n - 1) as f64), gamma)).collect();
            self.nu_tab = Some(NuTab { g: gamma, nu: nu_a, m0, m1 });
        }
        let tab = self.nu_tab.as_ref().unwrap();
        let n = tab.nu.len();
        let last = tab.nu[n - 1];
        if nu >= last {
            return inv_prandtl_meyer(nu, gamma);
        }
        let mut lo = 0usize;
        let mut hi = n - 1;
        while hi - lo > 1 {
            let mid = (lo + hi) >> 1;
            if tab.nu[mid] < nu {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let n0 = tab.nu[lo];
        let n1 = tab.nu[hi];
        let f = if n1 > n0 { (nu - n0) / (n1 - n0) } else { 0.0 };
        let log0 = tab.m0.ln();
        let log1 = tab.m1.ln();
        let a = log0 + ((log1 - log0) * lo as f64) / (n - 1) as f64;
        let b = log0 + ((log1 - log0) * hi as f64) / (n - 1) as f64;
        let seed = (a + (b - a) * f).exp();
        let gm1 = gamma - 1.0;
        let dnu = jmax(seed * seed - 1.0, 1e-12).sqrt() / (seed * (1.0 + 0.5 * gm1 * seed * seed));
        let polished = seed + (nu - prandtl_meyer(seed, gamma)) / dnu;
        if polished.is_finite() && polished >= 1.0 {
            polished
        } else {
            seed
        }
    }

    fn cone_node(&mut self, m: f64, k: i64, gamma: f64) -> ConeState {
        let key = (mach_key3(m), k, gamma.to_bits());
        if let Some(s) = self.cone_nodes.get(&key) {
            return *s;
        }
        let st = cone_surface_state_exact(m, k as f64 * CONE_NODE, gamma);
        if self.cone_nodes.len() > 20000 {
            self.cone_nodes.clear();
        }
        self.cone_nodes.insert(key, st);
        st
    }

    /// coneSurfaceState (0.1° node cache, Cp/θ² interpolated between nodes)
    pub fn cone_surface_state(&mut self, m: f64, cone: f64, gamma: f64) -> ConeState {
        if cone <= 1e-5 {
            return ConeState { cp: 0.0, p_ratio: 1.0, m };
        }
        let u = cone / CONE_NODE;
        let k0 = jmax(1.0, u.floor()) as i64;
        let f = u - k0 as f64;
        let a = self.cone_node(m, k0, gamma);
        if u < 1.0 {
            let s = (cone / CONE_NODE).powf(2.0);
            let cp = a.cp * s;
            return ConeState { cp, p_ratio: 1.0 + cp * 0.5 * gamma * m * m, m: a.m };
        }
        if f < 1e-9 {
            return a;
        }
        let b = self.cone_node(m, k0 + 1, gamma);
        let ta = (k0 as f64 * CONE_NODE).powf(2.0);
        let tb = ((k0 + 1) as f64 * CONE_NODE).powf(2.0);
        let ca = a.cp / ta;
        let cb = b.cp / tb;
        let cp = (ca + (cb - ca) * f) * cone * cone;
        let q = 0.5 * gamma * m * m;
        ConeState { cp, p_ratio: 1.0 + cp * q, m: a.m + (b.m - a.m) * f }
    }

    pub fn clear(&mut self) {
        self.beta_tabs.clear();
        self.cone_nodes.clear();
        self.nu_tab = None;
    }
}

impl Default for GasCache {
    fn default() -> Self {
        Self::new()
    }
}
