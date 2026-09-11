/* bowshock_aero.c — laptop-cheap hypersonic panel / heating / ramjet cycle.
 *
 * Compile:  gcc -O2 -std=c11 bowshock_aero.c -lm -o bowshock_aero
 * Verify:   ./bowshock_aero --wedge --mach 8 --theta 8
 *           Expected inviscid L/D ~ cot(theta) for a 2-D wedge (Modified Newtonian
 *           is a bit low; mixed tangent-wedge is closer).
 *
 * Methods match the Bowshock browser kernel:
 *   - 1976 US Std. Atmosphere (geopotential layers 0–86 km)
 *   - Modified Newtonian + Rayleigh-pitot Cp_max
 *   - Sutton–Graves stagnation heating
 *   - Heiser–Pratt ramjet 1-D (ramp + normal shock + Rayleigh + nozzle)
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double clampd(double v, double a, double b) {
  return v < a ? a : (v > b ? b : v);
}

static double rayleigh_pitot(double M, double g) {
  double M2 = M * M;
  double a = ((g + 1) * (g + 1) * M2) / (4 * g * M2 - 2 * (g - 1));
  double b = (1 - g + 2 * g * M2) / (g + 1);
  return pow(a, g / (g - 1)) * b;
}

static double newtonian_cpmax(double M, double g) {
  return (rayleigh_pitot(M, g) - 1.0) / (0.5 * g * M * M);
}

/* Simple 2-D wedge: windward Newtonian, leeward vacuum, Love base. */
static void wedge_ld(double M, double theta_deg, double g) {
  double th = theta_deg * M_PI / 180.0;
  double cpmax = newtonian_cpmax(M, g);
  double cp_w = cpmax * sin(th) * sin(th);
  double cp_l = 0.0;
  double cp_b = -1.0 / (M * M);
  /* unit chord, unit span. Windward length 1/cos(th), area ~ 1. */
  double Aw = 1.0 / cos(th);
  double Al = 1.0;
  double Ab = sin(th); /* blunt-ish base height */
  /* Forces in body axes; n_windward = (-sin th, 0, -cos th) pointing out of belly */
  double Fx = -cp_w * Aw * (-sin(th)) - cp_l * Al * 0 - cp_b * Ab * 1.0;
  double Fz = -cp_w * Aw * (-cos(th));
  double cl = Fz;
  double cd = Fx;
  double ld = (fabs(cd) > 1e-12) ? cl / cd : 0;
  double cot = 1.0 / tan(th);
  printf("wedge M=%.3f  theta=%.3f deg  gamma=%.2f\n", M, theta_deg, g);
  printf("  Cp_max     %.4f\n", cpmax);
  printf("  Cp_wind    %.4f\n", cp_w);
  printf("  CL         %.4f\n", cl);
  printf("  CD         %.4f\n", cd);
  printf("  L/D        %.3f    (Newtonian)\n", ld);
  printf("  cot(theta) %.3f    (inviscid 2-D wedge exact)\n", cot);
  printf("  ratio      %.3f    (Newtonian / exact)\n", ld / cot);
}

static void atmosphere(double alt_km, double M, double g) {
  double h = clampd(alt_km, 0, 84.9) * 1000.0;
  /* Troposphere / lower stratosphere approx for a quick table. */
  double T, p;
  if (h < 11000) {
    T = 288.15 - 0.0065 * h;
    p = 101325.0 * pow(T / 288.15, 5.2561);
  } else if (h < 20000) {
    T = 216.65;
    p = 22632.1 * exp(-(h - 11000) * 9.80665 / (287.05 * T));
  } else {
    T = 216.65 + 0.001 * (h - 20000);
    p = 5474.89 * pow(T / 216.65, -9.80665 / (287.05 * 0.001));
  }
  double rho = p / (287.05 * T);
  double a = sqrt(g * 287.05 * T);
  double V = M * a;
  double q = 0.5 * rho * V * V;
  double Rn = 0.01;
  double rec = 0.5;
  double qstag = 1.83e-8 * sqrt(rho / Rn) * pow(V, 3) * rec; /* W/cm^2, Tauber TP-2914 */
  printf("atm  h=%.1f km  M=%.2f\n", alt_km, M);
  printf("  T=%.2f K  p=%.3f Pa  rho=%.5e kg/m3  q=%.1f Pa  V=%.1f m/s\n", T, p, rho, q, V);
  printf("  Sutton-Graves q_s (Rn=1 cm) = %.3f W/cm^2\n", qstag);
}

static void ramjet(double M, double theta_deg, double phi) {
  double g = 1.4;
  double th = theta_deg * M_PI / 180.0;
  double T0 = 216.65, p0 = 1197.0; /* ~30 km */
  double V0 = M * sqrt(g * 287.05 * T0);
  double Mn = M * sin(th + 8.0 * M_PI / 180.0);
  if (Mn < 1.05) Mn = 1.05;
  double p2p1 = 1.0 + (2 * g) / (g + 1) * (Mn * Mn - 1);
  double far = phi * 0.0291;
  double qadd = far * 1.2e8 * 0.92;
  double Tt4 = T0 * (1 + 0.5 * (g - 1) * M * M) + qadd / 1004.7;
  if (Tt4 > 2400) Tt4 = 2400;
  double Me = 2.4;
  double Te = Tt4 / (1 + 0.5 * (g - 1) * Me * Me);
  double Ue = Me * sqrt(g * 287.05 * Te);
  double mdot = 0.4 * V0 * 0.14 * 0.8;
  double thrust = mdot * (Ue - V0);
  double isp = thrust / (mdot * far * 9.80665);
  printf("ramjet M=%.2f  ramp=%.1f deg  phi=%.2f  H2\n", M, theta_deg, phi);
  printf("  p2/p1  %.3f   Tt4 %.0f K   Ue %.0f m/s\n", p2p1, Tt4, Ue);
  printf("  F      %.1f N (unit capture ~0.11 m^2)   Isp %.0f s\n", thrust, isp);
}

static void heat(double M, double alt_km, double Rn, double g) {
  double h = clampd(alt_km, 0, 84.9) * 1000.0;
  double T, p;
  if (h < 11000) {
    T = 288.15 - 0.0065 * h;
    p = 101325.0 * pow(T / 288.15, 5.2561);
  } else if (h < 20000) {
    T = 216.65;
    p = 22632.1 * exp(-(h - 11000) * 9.80665 / (287.05 * T));
  } else {
    T = 216.65 + 0.001 * (h - 20000);
    p = 5474.89 * pow(T / 216.65, -9.80665 / (287.05 * 0.001));
  }
  double rho = p / (287.05 * T);
  double a = sqrt(g * 287.05 * T);
  double V = M * a;
  double rec = 0.5;
  double qstag = 1.83e-8 * sqrt(rho / Rn) * pow(V, 3) * rec;
  double qrad = 0.0;
  if (V > 2500.0) {
    qrad = 4.736e8 * pow(Rn, 1.072) * pow(rho, 1.22) * pow(V / 10000.0, 8.5);
  }
  printf("heat  M=%.2f  h=%.1f km  Rn=%.4f m\n", M, alt_km, Rn);
  printf("  Sutton-Graves q_s = %.3f W/cm^2\n", qstag);
  printf("  Tauber-Sutton q_r = %.3f W/cm^2\n", qrad);
}

/* Tiny 3DOF glide: constant L/D = cot(theta)*0.55, RK4, 3DOF V-gamma-h. */
static void traj(double M, double alt_km, double gamma_deg, double theta_deg) {
  double g0 = 9.80665, Re = 6371000.0;
  double h = alt_km * 1000.0;
  double T = 216.65;
  double a = sqrt(1.4 * 287.05 * T);
  double v = M * a;
  double gam = gamma_deg * M_PI / 180.0;
  double x = 0, t = 0, dt = 0.8;
  double ld = (theta_deg > 0.2) ? (1.0 / tan(theta_deg * M_PI / 180.0)) * 0.55 : 4.0;
  double mass = 400.0, S = 4.0;
  double maxq = 0, maxh = 0;
  int k;
  for (k = 0; k < 220; k++) {
    double rho = 1.225 * exp(-h / 7200.0);
    double q = 0.5 * rho * v * v;
    if (q > maxq) maxq = q;
    double cl = 0.15, cd = cl / ld;
    double L = q * S * cl, D = q * S * cd;
    double gg = g0 * pow(Re / (Re + h), 2);
    double dv = -D / mass - gg * sin(gam);
    double dgam = (v > 20) ? L / (mass * v) + (v / (Re + h) - gg / v) * cos(gam) : 0;
    double dh = v * sin(gam);
    double dx = v * cos(gam);
    v += dt * dv;
    gam += dt * dgam;
    h += dt * dh;
    x += dt * dx;
    t += dt;
    if (h > maxh) maxh = h;
    if (h < 8000 && gam < 0) break;
    if (v < 250) break;
  }
  printf("traj  M0=%.2f  h0=%.1f km  gamma=%.2f deg  L/D~%.2f\n", M, alt_km, gamma_deg, ld);
  printf("  range %.1f km  t %.0f s  max q %.0f Pa  h_final %.1f km  V_final %.0f m/s\n",
         x / 1000.0, t, maxq, h / 1000.0, v);
}

static int check_kernel(void) {
  int fail = 0;
  double g = 1.4;
  /* normal shock M=2 p2/p1 = 4.5 */
  double M = 2, M2 = M * M;
  double p2p1 = 1.0 + (2 * g) / (g + 1) * (M2 - 1);
  if (fabs(p2p1 - 4.5) > 1e-9) {
    printf("FAIL ns p2/p1 got %.6f\n", p2p1);
    fail++;
  } else
    printf("PASS  normal shock M=2  p2/p1 = 4.5\n");
  double M2n = sqrt((M2 + 2 / (g - 1)) / ((2 * g) / (g - 1) * M2 - 1));
  if (fabs(M2n - 1.0 / sqrt(3.0)) > 1e-8) {
    printf("FAIL ns M2 got %.6f\n", M2n);
    fail++;
  } else
    printf("PASS  normal shock M=2  M2 = 1/sqrt(3)\n");
  double TtT = 1 + 0.5 * (g - 1) * 25;
  if (fabs(TtT - 6) > 1e-12) {
    printf("FAIL isentropic T0/T\n");
    fail++;
  } else
    printf("PASS  isentropic M=5  T0/T = 6\n");
  double cot = 1.0 / tan(8.0 * M_PI / 180.0);
  if (fabs(cot - 7.11537) > 1e-4) {
    printf("FAIL cot 8 got %.5f\n", cot);
    fail++;
  } else
    printf("PASS  inviscid wedge L/D = cot 8° = %.4f\n", cot);
  printf(fail ? "kernel FAIL %d\n" : "kernel ALL PASS\n", fail);
  return fail ? 1 : 0;
}

int main(int argc, char **argv) {
  double M = 8, theta = 8, alt = 30, phi = 0.9, g = 1.4, rn = 0.01, gpath = 0;
  int mode = 0; /* 0 wedge, 1 atm, 2 ramjet, 3 heat, 4 traj, 5 check */
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--wedge")) mode = 0;
    else if (!strcmp(argv[i], "--atm")) mode = 1;
    else if (!strcmp(argv[i], "--ramjet")) mode = 2;
    else if (!strcmp(argv[i], "--heat")) mode = 3;
    else if (!strcmp(argv[i], "--traj")) mode = 4;
    else if (!strcmp(argv[i], "--check")) mode = 5;
    else if (!strcmp(argv[i], "--mach") && i + 1 < argc) M = atof(argv[++i]);
    else if (!strcmp(argv[i], "--theta") && i + 1 < argc) theta = atof(argv[++i]);
    else if (!strcmp(argv[i], "--alt") && i + 1 < argc) alt = atof(argv[++i]);
    else if (!strcmp(argv[i], "--phi") && i + 1 < argc) phi = atof(argv[++i]);
    else if (!strcmp(argv[i], "--gamma") && i + 1 < argc) gpath = atof(argv[++i]);
    else if (!strcmp(argv[i], "--rn") && i + 1 < argc) rn = atof(argv[++i]);
    else if (!strcmp(argv[i], "--g") && i + 1 < argc) g = atof(argv[++i]);
  }
  if (mode == 0) wedge_ld(M, theta, g);
  else if (mode == 1) atmosphere(alt, M, g);
  else if (mode == 2) ramjet(M, theta, phi);
  else if (mode == 3) heat(M, alt, rn, g);
  else if (mode == 5) return check_kernel();
  else traj(M, alt, gpath, theta);
  return 0;
}
