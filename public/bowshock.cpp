/* bowshock.cpp — C++ physics kernel for the Cuspis lab.
 *
 * Compile:  g++ -O2 -std=c++17 bowshock.cpp -o bowshock
 * Verify:   ./bowshock --check
 * 6DOF:     ./bowshock --6dof --mach 8 --alt 30 --alpha 2 \
 *             --CL 0.08 --CD 0.022 --CLa 1.8 --Cma -0.35 --Cmq -1.2 \
 *             --Cnb 0.25 --Clb -0.04 --Clp -0.35 --Cnr -0.2 \
 *             --mass 500 --S 4.4 --L 4 --b 2.2 --Iyy 180
 *
 * Same equations as the browser kernel (θ-β-M, Rankine–Hugoniot, US76,
 * Sutton–Graves, Etkin linear modes, RK4 6DOF). Run this offline so the UI
 * stays on the mixed-panel polar and this binary does the rigid-body work.
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double clampd(double v, double a, double b) {
  return v < a ? a : (v > b ? b : v);
}

static double deg = M_PI / 180.0;

/* ---------- gas dynamics ---------- */

static double beta_from_theta_M(double M, double theta, double g) {
  /* Anderson 3.3 / NACA 1135. Weak root via bisection on the θ-β-M residual. */
  if (theta <= 1e-9) return std::asin(clampd(1.0 / M, 0.0, 1.0));
  double mu = std::asin(clampd(1.0 / M, 0.0, 1.0));
  double lo = mu + 1e-6, hi = 70.0 * deg;
  auto f = [&](double b) {
    double sb = std::sin(b), cb = std::cos(b);
    double num = 2.0 * cb / sb * (M * M * sb * sb - 1.0);
    double den = M * M * (g + std::cos(2.0 * b)) + 2.0;
    return std::atan(num / den) - theta;
  };
  if (f(lo) * f(hi) > 0) return NAN;
  for (int i = 0; i < 60; i++) {
    double mid = 0.5 * (lo + hi);
    if (f(mid) > 0) hi = mid;
    else lo = mid;
  }
  return 0.5 * (lo + hi);
}

static void normal_shock(double M, double g, double& p2p1, double& M2) {
  double M2s = M * M;
  p2p1 = 1.0 + (2 * g) / (g + 1) * (M2s - 1);
  M2 = std::sqrt((M2s + 2 / (g - 1)) / ((2 * g) / (g - 1) * M2s - 1));
}

static double sutton_graves(double rho, double V, double Rn, double rec) {
  return 1.83e-8 * std::sqrt(rho / std::max(Rn, 1e-8)) * std::pow(V, 3.0) * rec;
}

static double billig_standoff(double M) {
  double m = std::max(1.05, M);
  return 0.143 * std::exp(3.24 / (m * m));
}

static double gamma_vib(double T) {
  auto contrib = [](double theta, double t) {
    double u = theta / std::max(t, 40.0);
    if (u > 20) return 0.0;
    double e = std::exp(u);
    double d = e - 1.0;
    return (u * u * e) / (d * d);
  };
  double cvR = 2.5 + 0.21 * contrib(2270, T) + 0.79 * contrib(3390, T);
  return 1.0 + 1.0 / std::max(cvR, 1.5);
}

static double millikan_white_tau(double T, double p) {
  double t = std::max(T, 400.0);
  double Tm = std::pow(t, -1.0 / 3.0);
  double tauN2 = std::exp(221.0 * (Tm - 0.029) - 18.42);
  double tauO2 = std::exp(129.0 * (Tm - 0.0295) - 18.42);
  double pAtm = std::max(p, 1e-6) / 101325.0;
  return (0.79 * tauN2 + 0.21 * tauO2) / pAtm;
}

static double waltrup_billig(double M, double p2p1, double thH = 0.01) {
  double d = std::max(p2p1, 1.0) - 1.0;
  double num = 50.0 * d + 170.0 * d * d;
  return std::sqrt(std::max(thH, 1e-4)) * num / std::max(M * M - 1.0, 0.25);
}

/* ---------- atmosphere (US76 tropo/strato) ---------- */

static void us76(double alt_km, double M, double g, double& T, double& p, double& rho, double& a, double& V, double& q) {
  double h = clampd(alt_km, 0, 84.9) * 1000.0;
  if (h < 11000) {
    T = 288.15 - 0.0065 * h;
    p = 101325.0 * std::pow(T / 288.15, 5.2561);
  } else if (h < 20000) {
    T = 216.65;
    p = 22632.1 * std::exp(-(h - 11000) * 9.80665 / (287.05 * T));
  } else if (h < 32000) {
    T = 216.65 + 0.001 * (h - 20000);
    p = 5474.89 * std::pow(T / 216.65, -9.80665 / (287.05 * 0.001));
  } else {
    T = 228.65 + 0.0028 * (h - 32000);
    p = 868.019 * std::pow(T / 228.65, -9.80665 / (287.05 * 0.0028));
  }
  rho = p / (287.05 * T);
  a = std::sqrt(g * 287.05 * T);
  V = M * a;
  q = 0.5 * rho * V * V;
}

/* ---------- linear 6DOF (Etkin) ---------- */

struct Coeffs {
  double mach = 8, alt = 30, alpha_deg = 2, gamma_deg = 0;
  double CL = 0.08, CD = 0.022, Cm = -0.01;
  double CLa = 1.8, CDa = 0.4, Cma = -0.35, Cmq = -1.2;
  double Cnb = 0.25, Clb = -0.04, CYb = -0.5;
  double Clp = -0.35, Cnr = -0.2, Clr = 0.08, Cnp = 0.02, CLq = 2.0;
  double mass = 500, S = 4.4, Lref = 4.0, bref = 2.2;
  double Ixx = 80, Iyy = 180, Izz = 220;
};

struct Mode {
  const char* name;
  double wn, zeta, period, t_half;
  bool stable;
};

static void modes_of(const Coeffs& c, double V, double qbar, std::vector<Mode>& out) {
  const double g0 = 9.80665;
  double Za = -(qbar * c.S * c.CLa) / c.mass;
  double Ma = (qbar * c.S * c.Lref * c.Cma) / c.Iyy;
  double Mq = (qbar * c.S * c.Lref * c.Cmq * (c.Lref / (2 * V))) / c.Iyy;
  double ZaV = Za / V;
  double wn_sp = std::sqrt(std::max(0.0, ZaV * Mq - Ma));
  double z_sp = wn_sp > 1e-8 ? -(ZaV + Mq) / (2 * wn_sp) : 0;
  double ld = std::max(0.4, c.CL / std::max(c.CD, 1e-6));
  double wn_ph = g0 * std::sqrt(2.0) / V;
  double z_ph = 1.0 / (std::sqrt(2.0) * ld);
  double Nb = (qbar * c.S * c.bref * c.Cnb) / c.Izz;
  double Nr = (qbar * c.S * c.bref * c.Cnr * (c.bref / (2 * V))) / c.Izz;
  double YbV = (qbar * c.S * c.CYb) / (c.mass * V);
  double wn_dr = std::sqrt(std::max(0.0, Nb));
  double z_dr = wn_dr > 1e-8 ? -(YbV + Nr) / (2 * wn_dr) : 0;
  double Lp = (qbar * c.S * c.bref * c.Clp * (c.bref / (2 * V))) / c.Ixx;
  double tau_r = Lp < -1e-6 ? -1.0 / Lp : 0;
  double lamS = (g0 / V) * ((c.Clb * c.Cnr - c.Cnb * c.Clr) / std::max(1e-6, std::abs(c.Clp))) * (c.Clp < 0 ? 1 : -1);

  auto push = [&](const char* n, double wn, double z, bool st, double th = 0) {
    Mode m;
    m.name = n;
    m.wn = wn;
    m.zeta = z;
    m.period = wn > 1e-8 ? 2 * M_PI / wn : 0;
    m.t_half = th != 0 ? th : (std::abs(z * wn) > 1e-8 ? std::log(2.0) / std::abs(z * wn) : 0);
    m.stable = st && wn >= 0 && z > -0.02;
    out.push_back(m);
  };
  push("Short period", wn_sp, z_sp, c.Cma < 0 && c.CLa > 0);
  push("Phugoid", wn_ph, z_ph, z_ph > 0);
  push("Dutch roll", wn_dr, z_dr, c.Cnb > 0);
  {
    Mode m{"Roll subsidence", std::abs(Lp), Lp < 0 ? 1.0 : -1.0, 0, tau_r, Lp < 0};
    out.push_back(m);
  }
  {
    Mode m{"Spiral", std::abs(lamS), lamS < 0 ? 1.0 : -1.0, 0,
           std::abs(lamS) > 1e-6 ? std::log(2.0) / std::abs(lamS) : 0, lamS <= 0};
    out.push_back(m);
  }
}

struct Sample {
  double t, alpha, q, beta, p, r, V, h, x;
};

static void rk4_6dof_clean(const Coeffs& c, double V0, double qbar, std::vector<Sample>& hist, int& iters) {
  hist.clear();
  const double dt = 0.02;
  const int n = 400;
  const double g0 = 9.80665, Re = 6371000.0;
  double y[7] = {(c.alpha_deg + 2) * deg, 0, 0, 0, 0, V0, c.gamma_deg * deg};
  double h = std::max(500.0, c.alt * 1000.0), x = 0;
  iters = n;

  auto f = [&](const double* s, double* d, double hh) {
    double al = s[0], qq = s[1], be = s[2], pp = s[3], rr = s[4], Vv = s[5], ga = s[6];
    double qh = qq * c.Lref / (2 * Vv);
    double ph = pp * c.bref / (2 * Vv);
    double rh = rr * c.bref / (2 * Vv);
    double da = al - c.alpha_deg * deg;
    double CL = c.CL + c.CLa * da + c.CLq * qh;
    double CD = std::max(0.002, c.CD + c.CDa * da);
    double CY = c.CYb * be;
    double Cl = c.Clb * be + c.Clp * ph + c.Clr * rh;
    double Cm = c.Cm + c.Cma * da + c.Cmq * qh;
    double Cn = c.Cnb * be + c.Cnp * ph + c.Cnr * rh;
    double qd = qbar * (Vv / V0) * (Vv / V0);
    double Lift = qd * c.S * CL;
    double Drag = qd * c.S * CD;
    double Y = qd * c.S * CY;
    double gLoc = g0 * std::pow(Re / (Re + hh), 2.0);
    d[0] = qq - Lift / (c.mass * Vv);
    d[1] = (qd * c.S * c.Lref * Cm) / c.Iyy;
    d[2] = pp * std::sin(al) - rr * std::cos(al) + Y / (c.mass * Vv);
    d[3] = (qd * c.S * c.bref * Cl) / c.Ixx;
    d[4] = (qd * c.S * c.bref * Cn) / c.Izz;
    d[5] = -Drag / c.mass - gLoc * std::sin(ga);
    d[6] = Vv > 20 ? Lift / (c.mass * Vv) + (Vv / (Re + hh) - gLoc / Vv) * std::cos(ga) : 0;
  };

  for (int k = 0; k <= n; k++) {
    if (k % 4 == 0)
      hist.push_back({k * dt, y[0] / deg, y[1] / deg, y[2] / deg, y[3] / deg, y[4] / deg, y[5], h / 1000.0, x / 1000.0});
    double k1[7], k2[7], k3[7], k4[7], w[7];
    f(y, k1, h);
    for (int i = 0; i < 7; i++) w[i] = y[i] + 0.5 * dt * k1[i];
    f(w, k2, h);
    for (int i = 0; i < 7; i++) w[i] = y[i] + 0.5 * dt * k2[i];
    f(w, k3, h);
    for (int i = 0; i < 7; i++) w[i] = y[i] + dt * k3[i];
    f(w, k4, h);
    for (int i = 0; i < 7; i++) y[i] += (dt / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    h += dt * y[5] * std::sin(y[6]);
    x += dt * y[5] * std::cos(y[6]) * Re / (Re + h);
    y[0] = clampd(y[0], -20 * deg, 25 * deg);
    y[2] = clampd(y[2], -20 * deg, 20 * deg);
    y[5] = clampd(y[5], 80, 12000);
    h = clampd(h, 0, 95000);
  }
}

static int check_kernel() {
  int fail = 0;
  double g = 1.4;
  double p2, M2;
  normal_shock(2, g, p2, M2);
  auto ok = [&](const char* name, double got, double exp, double tol) {
    if (std::abs(got - exp) > tol) {
      std::printf("FAIL  %s  got %.6g expected %.6g\n", name, got, exp);
      fail++;
    } else
      std::printf("PASS  %s\n", name);
  };
  ok("normal shock M=2 p2/p1 = 4.5", p2, 4.5, 1e-9);
  ok("normal shock M=2 M2 = 1/sqrt(3)", M2, 1.0 / std::sqrt(3.0), 1e-8);
  ok("isentropic T0/T M=5 = 6", 1 + 0.5 * (g - 1) * 25, 6, 1e-12);
  double beta = beta_from_theta_M(8, 8 * deg, g);
  ok("theta-beta-M M=8 theta=8 deg  beta~12.2-13.5", beta / deg, 12.8, 1.0);
  double qs = sutton_graves(1, 1000, 1, 1);
  ok("Sutton-Graves 1.83e-8 V^3 (18.3 W/cm^2)", qs, 18.3, 1e-6);
  double T, p, rho, a, V, q;
  us76(0, 8, g, T, p, rho, a, V, q);
  ok("US76 SL T=288.15", T, 288.15, 0.05);
  us76(30, 8, g, T, p, rho, a, V, q);
  ok("US76 30 km T~226.5", T, 226.65, 2.0);
  /* short-period identity: Cmα < 0 ⇒ ω real for a stable wedge */
  Coeffs c;
  c.Cma = -0.4;
  c.CLa = 2;
  c.Cmq = -2;
  std::vector<Mode> ms;
  us76(30, 8, g, T, p, rho, a, V, q);
  modes_of(c, V, q, ms);
  ok("short-period ωn > 0 (stable Cmα)", ms[0].wn > 0 ? 1 : 0, 1, 0.1);
  ok("Billig Δ/Rn M→∞ = 0.143", billig_standoff(1e6), 0.143, 1e-6);
  ok("γ_vib 288 K ≈ 1.4", gamma_vib(288.15), 1.4, 0.01);
  ok("Waltrup–Billig L/H M=3 pr=2.5", waltrup_billig(3, 2.5, 0.01), 5.719, 0.05);
  ok("Millikan–White τ 3000 K 1 atm", millikan_white_tau(3000, 101325), 6e-5, 3e-5);
  std::printf("%s  %d failed\n", fail ? "KERNEL FAIL" : "KERNEL PASS", fail);
  return fail;
}

static void print_6dof(const Coeffs& c) {
  double T, p, rho, a, V, q;
  us76(c.alt, c.mach, 1.4, T, p, rho, a, V, q);
  std::printf("6DOF  M=%.3f  h=%.1f km  V=%.1f m/s  q=%.0f Pa  α=%.2f deg\n", c.mach, c.alt, V, q, c.alpha_deg);
  std::printf("  mass %.1f kg  S %.3f m^2  L %.3f m  b %.3f m\n", c.mass, c.S, c.Lref, c.bref);
  std::printf("  Ixx %.2f  Iyy %.2f  Izz %.2f kg m^2\n", c.Ixx, c.Iyy, c.Izz);
  std::printf("static  CLα=%.4f  Cmα=%.4f  Cnβ=%.4f  Clβ=%.4f  CYβ=%.4f\n", c.CLa, c.Cma, c.Cnb, c.Clb, c.CYb);
  std::printf("rotary  Cmq=%.4f  Clp=%.4f  Cnr=%.4f  Clr=%.4f  Cnp=%.4f\n", c.Cmq, c.Clp, c.Cnr, c.Clr, c.Cnp);
  std::vector<Mode> ms;
  modes_of(c, V, q, ms);
  std::printf("modes (Etkin / Nelson)\n");
  for (const auto& m : ms) {
    std::printf("  %-18s  ωn=%8.4f rad/s  ζ=%7.4f  T=%8.3f s  t½=%8.3f s  %s\n", m.name, m.wn, m.zeta, m.period,
                m.t_half, m.stable ? "stable" : "UNSTABLE");
  }
  std::vector<Sample> hist;
  int iters = 0;
  rk4_6dof_clean(c, V, q, hist, iters);
  std::printf("RK4  %d steps  dt=0.02 s  α pulse +2 deg  n_samples=%zu\n", iters, hist.size());
  if (!hist.empty()) {
    const Sample& a0 = hist.front();
    const Sample& a1 = hist[std::min<size_t>(hist.size() - 1, 50)];
    const Sample& aN = hist.back();
    std::printf("  t=0     α=%7.3f  q=%8.3f  V=%.1f  h=%.2f km\n", a0.alpha, a0.q, a0.V, a0.h);
    std::printf("  t=%.2f  α=%7.3f  q=%8.3f  V=%.1f  h=%.2f km\n", a1.t, a1.alpha, a1.q, a1.V, a1.h);
    std::printf("  t=%.2f  α=%7.3f  q=%8.3f  V=%.1f  h=%.2f km  range=%.2f km\n", aN.t, aN.alpha, aN.q, aN.V, aN.h, aN.x);
  }
}

static void print_json(const Coeffs& c) {
  double T, p, rho, a, V, q;
  us76(c.alt, c.mach, 1.4, T, p, rho, a, V, q);
  std::vector<Mode> ms;
  modes_of(c, V, q, ms);
  std::vector<Sample> hist;
  int iters = 0;
  rk4_6dof_clean(c, V, q, hist, iters);
  std::printf("{\n  \"mach\": %.4f, \"altKm\": %.3f, \"V\": %.4f, \"q\": %.4f,\n", c.mach, c.alt, V, q);
  std::printf("  \"iterations\": %d, \"dt\": 0.02,\n", iters);
  std::printf("  \"modes\": [\n");
  for (size_t i = 0; i < ms.size(); i++) {
    std::printf("    {\"name\": \"%s\", \"wn\": %.6g, \"zeta\": %.6g, \"stable\": %s}%s\n", ms[i].name, ms[i].wn,
                ms[i].zeta, ms[i].stable ? "true" : "false", i + 1 < ms.size() ? "," : "");
  }
  std::printf("  ]\n}\n");
}

static double argd(int argc, char** argv, const char* key, double def) {
  for (int i = 1; i < argc - 1; i++)
    if (std::strcmp(argv[i], key) == 0) return std::atof(argv[i + 1]);
  return def;
}

static void usage() {
  std::printf("bowshock — hypersonic panel / 6DOF kernel\n");
  std::printf("  --check                 kernel identities (Anderson, NACA 1135, US76, SG)\n");
  std::printf("  --wedge --mach M --theta deg\n");
  std::printf("  --6dof  [coeff flags]   Etkin modes + RK4 6DOF\n");
  std::printf("  --json                  same as --6dof, JSON out\n");
  std::printf("  --atm --mach M --alt km\n");
  std::printf("  --heat --mach M --alt km --Rn m\n");
}

int main(int argc, char** argv) {
  if (argc < 2) {
    usage();
    return 0;
  }
  bool do_check = false, do_6 = false, do_json = false, do_atm = false, do_heat = false, do_wedge = false;
  for (int i = 1; i < argc; i++) {
    if (std::strcmp(argv[i], "--check") == 0) do_check = true;
    else if (std::strcmp(argv[i], "--6dof") == 0) do_6 = true;
    else if (std::strcmp(argv[i], "--json") == 0) do_json = true;
    else if (std::strcmp(argv[i], "--atm") == 0) do_atm = true;
    else if (std::strcmp(argv[i], "--heat") == 0) do_heat = true;
    else if (std::strcmp(argv[i], "--wedge") == 0) do_wedge = true;
    else if (std::strcmp(argv[i], "--help") == 0) {
      usage();
      return 0;
    }
  }
  if (do_check) return check_kernel();
  double M = argd(argc, argv, "--mach", 8);
  double alt = argd(argc, argv, "--alt", 30);
  double theta = argd(argc, argv, "--theta", 8);
  if (do_wedge) {
    double beta = beta_from_theta_M(M, theta * deg, 1.4);
    double cot = 1.0 / std::tan(theta * deg);
    std::printf("wedge M=%.3f  theta=%.3f  beta=%.3f deg  inviscid L/D = cot θ = %.3f\n", M, theta, beta / deg, cot);
    return 0;
  }
  if (do_atm) {
    double T, p, rho, a, V, q;
    us76(alt, M, 1.4, T, p, rho, a, V, q);
    std::printf("US76  h=%.1f km  M=%.2f  T=%.2f K  p=%.3f Pa  rho=%.5e  V=%.1f  q=%.1f\n", alt, M, T, p, rho, V, q);
    return 0;
  }
  if (do_heat) {
    double T, p, rho, a, V, q;
    us76(alt, M, 1.4, T, p, rho, a, V, q);
    double Rn = argd(argc, argv, "--Rn", 0.01);
    std::printf("Sutton-Graves qs = %.4f W/cm^2  (M=%.2f h=%.1f km Rn=%.4f m)\n", sutton_graves(rho, V, Rn, 0.5), M, alt,
                Rn);
    std::printf("Billig Δ/Rn = %.4f   γ_vib(T∞)=%.4f   τ_v=%.3e s\n", billig_standoff(M), gamma_vib(T),
                millikan_white_tau(T * (1 + 0.2 * M * M), p));
    return 0;
  }
  Coeffs c;
  c.mach = M;
  c.alt = alt;
  c.alpha_deg = argd(argc, argv, "--alpha", 2);
  c.CL = argd(argc, argv, "--CL", c.CL);
  c.CD = argd(argc, argv, "--CD", c.CD);
  c.Cm = argd(argc, argv, "--Cm", c.Cm);
  c.CLa = argd(argc, argv, "--CLa", c.CLa);
  c.CDa = argd(argc, argv, "--CDa", c.CDa);
  c.Cma = argd(argc, argv, "--Cma", c.Cma);
  c.Cmq = argd(argc, argv, "--Cmq", c.Cmq);
  c.Cnb = argd(argc, argv, "--Cnb", c.Cnb);
  c.Clb = argd(argc, argv, "--Clb", c.Clb);
  c.CYb = argd(argc, argv, "--CYb", c.CYb);
  c.Clp = argd(argc, argv, "--Clp", c.Clp);
  c.Cnr = argd(argc, argv, "--Cnr", c.Cnr);
  c.Clr = argd(argc, argv, "--Clr", c.Clr);
  c.Cnp = argd(argc, argv, "--Cnp", c.Cnp);
  c.CLq = argd(argc, argv, "--CLq", c.CLq);
  c.mass = argd(argc, argv, "--mass", c.mass);
  c.S = argd(argc, argv, "--S", c.S);
  c.Lref = argd(argc, argv, "--L", c.Lref);
  c.bref = argd(argc, argv, "--b", c.bref);
  c.Ixx = argd(argc, argv, "--Ixx", c.Ixx);
  c.Iyy = argd(argc, argv, "--Iyy", c.Iyy);
  c.Izz = argd(argc, argv, "--Izz", c.Izz);
  if (do_json) {
    print_json(c);
    return 0;
  }
  if (do_6) {
    print_6dof(c);
    return 0;
  }
  usage();
  return 0;
}
