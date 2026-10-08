/*
 * Bowshock kernel. Gasdynamics, a strip shock-expansion march, and the same
 * first-order HLLC wedge as src/lib/waverider/euler2d.ts.
 *
 *   gcc -O3 -std=c11 native/bowshock_kernel.c -lm -o bowshock_kernel
 *   ./bowshock_kernel --check
 *
 * This is a 2-D anchor. It is not a 3-D Euler solver and it does not replace
 * Cart3D on a vehicle. On a wedge, an Euler code has to recover the oblique
 * shock; --check scores this scheme against that shock.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const double PI = 3.14159265358979323846;
static const double DEG = 0.017453292519943295;

static double theta_from_beta(double M, double beta, double gamma) {
  double s = sin(beta);
  double c = cos(beta);
  double M2, num, den;
  if (fabs(s) < 1e-12) return 0.0;
  M2 = M * M;
  num = 2.0 * (c / s) * (M2 * s * s - 1.0);
  den = M2 * (gamma + cos(2.0 * beta)) + 2.0;
  return atan(num / den);
}

static void max_theta(double M, double gamma, double *theta, double *beta) {
  double mu = asin(fmax(0.0, fmin(1.0, 1.0 / M)));
  double bestT = 0.0;
  double bestB = mu;
  int i;
  for (i = 1; i < 80; i++) {
    double b = mu + ((PI / 2.0 - 1e-4 - mu) * i) / 80.0;
    double t = theta_from_beta(M, b, gamma);
    if (t > bestT) {
      bestT = t;
      bestB = b;
    }
  }
  *theta = bestT;
  *beta = bestB;
}

/* Weak-shock beta. NAN if the shock is detached. */
static double beta_from_theta(double M, double theta, double gamma) {
  double capT, capB, mu, lo, hi;
  int i;
  if (theta <= 1e-10) return asin(fmax(0.0, fmin(1.0, 1.0 / M)));
  max_theta(M, gamma, &capT, &capB);
  if (theta >= capT * 0.999) return NAN;
  mu = asin(fmax(0.0, fmin(1.0, 1.0 / M)));
  lo = mu + 1e-6;
  hi = capB;
  for (i = 0; i < 48; i++) {
    double mid = 0.5 * (lo + hi);
    if (theta_from_beta(M, mid, gamma) < theta) lo = mid;
    else hi = mid;
  }
  return 0.5 * (lo + hi);
}

typedef struct {
  double p2p1;
  double M2;
} Shock;

static Shock oblique(double M, double beta, double gamma) {
  double Mn = M * sin(beta);
  double Mn2 = Mn * Mn;
  double gp1 = gamma + 1.0;
  double gm1 = gamma - 1.0;
  double p2p1 = 1.0 + (2.0 * gamma) / gp1 * (Mn2 - 1.0);
  double Mn2sq = (Mn2 + 2.0 / gm1) / ((2.0 * gamma) / gm1 * Mn2 - 1.0);
  double theta = theta_from_beta(M, beta, gamma);
  Shock s;
  s.p2p1 = p2p1;
  s.M2 = sqrt(fmax(0.0, Mn2sq)) / sin(fmax(1e-6, beta - theta));
  return s;
}

static double prandtl(double M, double gamma) {
  double k, x;
  if (M <= 1.0) return 0.0;
  k = sqrt((gamma + 1.0) / (gamma - 1.0));
  x = sqrt(M * M - 1.0);
  return k * atan(sqrt((gamma - 1.0) / (gamma + 1.0)) * x) - atan(x);
}

static double inv_prandtl(double nu, double gamma) {
  double M = 1.5;
  int i;
  if (nu <= 0.0) return 1.0;
  for (i = 0; i < 24; i++) {
    double dM = 1e-4;
    double f = prandtl(M, gamma) - nu;
    double fp = (prandtl(M + dM, gamma) - prandtl(M, gamma)) / dM;
    M = fmax(1.0001, M - f / (fp == 0.0 ? 1.0 : fp));
  }
  return M;
}

/* p / p0 */
static double isen_p(double M, double gamma) {
  double t = 1.0 + 0.5 * (gamma - 1.0) * M * M;
  return pow(t, -gamma / (gamma - 1.0));
}

static double isen_Tt(double M, double gamma) {
  return 1.0 + 0.5 * (gamma - 1.0) * M * M;
}

static double normal_p(double M, double gamma) {
  return 1.0 + (2.0 * gamma) / (gamma + 1.0) * (M * M - 1.0);
}

/*
 * One shock at the first deflected panel, then Prandtl–Meyer when the
 * slope drops. theta[] is in radians. cp_out may be NULL.
 * Returns the last panel Cp.
 */
static double march_strip(double M, double gamma, const double *theta, int n) {
  double Mloc = M;
  double pRatio = 1.0;
  double flow = 0.0;
  double q = 0.5 * gamma * M * M;
  double cpi = 0.0;
  int i;
  for (i = 0; i < n; i++) {
    double inc = theta[i];
    double dth = inc - flow;
    if (dth > 0.15 * DEG) {
      double beta = beta_from_theta(Mloc, dth, gamma);
      Shock sh;
      if (!isfinite(beta)) return NAN;
      sh = oblique(Mloc, beta, gamma);
      pRatio *= sh.p2p1;
      Mloc = sh.M2;
      cpi = (pRatio - 1.0) / q;
    } else if (dth < -0.15 * DEG) {
      double M2 = inv_prandtl(prandtl(fmax(Mloc, 1.001), gamma) + (-dth), gamma);
      double ratio = isen_p(M2, gamma) / fmax(1e-12, isen_p(fmax(Mloc, 1.001), gamma));
      pRatio *= ratio;
      Mloc = M2;
      cpi = (pRatio - 1.0) / q;
    } else {
      cpi = (pRatio - 1.0) / q;
    }
    flow = inc;
  }
  return cpi;
}

static double analytic_aft(double M, double thetaNose, double thetaAft, double gamma) {
  double beta = beta_from_theta(M, thetaNose, gamma);
  Shock sh = oblique(M, beta, gamma);
  double M3 = inv_prandtl(prandtl(sh.M2, gamma) + (thetaNose - thetaAft), gamma);
  double p3p2 = isen_p(M3, gamma) / isen_p(sh.M2, gamma);
  double q = 0.5 * gamma * M * M;
  return (p3p2 * sh.p2p1 - 1.0) / q;
}

/* ---- 2-D HLLC wedge. Constants match euler2d.ts. ---- */

#define NI 72
#define NJ 24
#define ITERS 280
#define CFL 0.45

static double F4[4];

static void hllc(double rL, double uL, double vL, double pL, double rR, double uR, double vR, double pR, double Ax, double Ay, double gamma) {
  double area = hypot(Ax, Ay);
  double nx, ny, tx, ty, unL, utL, unR, utR, aL, aR, EL, ER, SL, SR, den, SM;
  double fr, fun, fut, fE, r, un, ut, p, E;
  if (area < 1e-14) {
    F4[0] = F4[1] = F4[2] = F4[3] = 0.0;
    return;
  }
  nx = Ax / area;
  ny = Ay / area;
  tx = -ny;
  ty = nx;
  unL = uL * nx + vL * ny;
  utL = uL * tx + vL * ty;
  unR = uR * nx + vR * ny;
  utR = uR * tx + vR * ty;
  aL = sqrt((gamma * pL) / rL);
  aR = sqrt((gamma * pR) / rR);
  EL = pL / (gamma - 1.0) + 0.5 * rL * (uL * uL + vL * vL);
  ER = pR / (gamma - 1.0) + 0.5 * rR * (uR * uR + vR * vR);
  SL = fmin(unL, unR) - fmax(aL, aR);
  SR = fmax(unL, unR) + fmax(aL, aR);
  den = rL * (SL - unL) - rR * (SR - unR);
  SM = den == 0.0 ? 0.5 * (unL + unR) : (pR - pL + rL * unL * (SL - unL) - rR * unR * (SR - unR)) / den;
  if (SL >= 0.0) {
    r = rL;
    un = unL;
    ut = utL;
    p = pL;
    E = EL;
    fr = r * un;
    fun = r * un * un + p;
    fut = r * un * ut;
    fE = (E + p) * un;
  } else if (SR <= 0.0) {
    r = rR;
    un = unR;
    ut = utR;
    p = pR;
    E = ER;
    fr = r * un;
    fun = r * un * un + p;
    fut = r * un * ut;
    fE = (E + p) * un;
  } else {
    int sideL = SM >= 0.0;
    double S, coeff, Eover, f0, f1, f2, f3;
    r = sideL ? rL : rR;
    un = sideL ? unL : unR;
    ut = sideL ? utL : utR;
    p = sideL ? pL : pR;
    E = sideL ? EL : ER;
    S = sideL ? SL : SR;
    coeff = (r * (S - un)) / (S - SM);
    Eover = E / r + (SM - un) * (SM + p / (r * (S - un)));
    f0 = r * un;
    f1 = r * un * un + p;
    f2 = r * un * ut;
    f3 = (E + p) * un;
    fr = f0 + S * (coeff - r);
    fun = f1 + S * (coeff * SM - r * un);
    fut = f2 + S * (coeff * ut - r * ut);
    fE = f3 + S * (coeff * Eover - E);
  }
  F4[0] = fr * area;
  F4[1] = (fun * nx + fut * tx) * area;
  F4[2] = (fun * ny + fut * ty) * area;
  F4[3] = fE * area;
}

typedef struct {
  double p2p1;
  double exactP;
  double relErr;
  double residual;
} EulerOut;

static EulerOut euler_wedge(double M, double thetaDeg, double gamma) {
  static double nodesX[(NI + 1) * (NJ + 1)];
  static double nodesY[(NI + 1) * (NJ + 1)];
  static double vol[NI * NJ];
  static double fW[NI * NJ * 2];
  static double fE[NI * NJ * 2];
  static double fS[NI * NJ * 2];
  static double fN[NI * NJ * 2];
  static double U[NI * NJ * 4];
  double theta = thetaDeg * DEG;
  double beta = beta_from_theta(M, theta, gamma);
  Shock exact = oblique(M, beta, gamma);
  double Lx = 1.0;
  double x0 = 0.2;
  double yTop = tan(beta) * (Lx - x0) * 1.45 + 0.08;
  double rhoInf = 1.0;
  double pInf = 1.0;
  double aInf = sqrt(gamma * pInf / rhoInf);
  double uInf = M * aInf;
  double gm1 = gamma - 1.0;
  double residual = 1.0;
  int i, j, iter, c;
  EulerOut out;
  out.exactP = exact.p2p1;
  out.p2p1 = 1.0;
  out.relErr = 1.0;
  out.residual = 1.0;
  if (!isfinite(beta)) return out;

  for (j = 0; j <= NJ; j++) {
    for (i = 0; i <= NI; i++) {
      double x = ((double)i / NI) * Lx;
      double yw = x <= x0 ? 0.0 : (x - x0) * tan(theta);
      double y = yw + ((double)j / NJ) * (yTop - yw);
      int id = j * (NI + 1) + i;
      nodesX[id] = x;
      nodesY[id] = y;
    }
  }
  for (j = 0; j < NJ; j++) {
    for (i = 0; i < NI; i++) {
      int sw = j * (NI + 1) + i;
      int se = sw + 1;
      int nw = sw + (NI + 1);
      int ne = nw + 1;
      double px[4] = {nodesX[sw], nodesX[se], nodesX[ne], nodesX[nw]};
      double py[4] = {nodesY[sw], nodesY[se], nodesY[ne], nodesY[nw]};
      double a = 0.0;
      int id = j * NI + i;
      int k;
      for (k = 0; k < 4; k++) {
        int n = (k + 1) % 4;
        a += px[k] * py[n] - px[n] * py[k];
      }
      vol[id] = fabs(a) * 0.5;
      fW[id * 2] = -(nodesY[nw] - nodesY[sw]);
      fW[id * 2 + 1] = nodesX[nw] - nodesX[sw];
      fE[id * 2] = nodesY[ne] - nodesY[se];
      fE[id * 2 + 1] = -(nodesX[ne] - nodesX[se]);
      fS[id * 2] = nodesY[se] - nodesY[sw];
      fS[id * 2 + 1] = -(nodesX[se] - nodesX[sw]);
      fN[id * 2] = -(nodesY[ne] - nodesY[nw]);
      fN[id * 2 + 1] = nodesX[ne] - nodesX[nw];
    }
  }
  for (c = 0; c < NI * NJ; c++) {
    U[c * 4] = rhoInf;
    U[c * 4 + 1] = rhoInf * uInf;
    U[c * 4 + 2] = 0.0;
    U[c * 4 + 3] = pInf / gm1 + 0.5 * rhoInf * uInf * uInf;
  }

  for (iter = 0; iter < ITERS; iter++) {
    double resSum = 0.0;
    for (j = 0; j < NJ; j++) {
      for (i = 0; i < NI; i++) {
        int id = j * NI + i;
        int o = id * 4;
        double r = U[o] > 1e-8 ? U[o] : 1e-8;
        double u = U[o + 1] / r;
        double v = U[o + 2] / r;
        double p = gm1 * (U[o + 3] - 0.5 * r * (u * u + v * v));
        double pp = p > 1e-8 ? p : 1e-8;
        double d0 = 0.0, d1 = 0.0, d2 = 0.0, d3 = 0.0;
        double nr, nu, nv, nE, rr, uu, vv, pNew, spec, per, dt, scale;
        if (i == 0) {
          hllc(r, u, v, pp, rhoInf, uInf, 0.0, pInf, fW[id * 2], fW[id * 2 + 1], gamma);
        } else {
          int oo = (id - 1) * 4;
          double rr2 = U[oo] > 1e-8 ? U[oo] : 1e-8;
          double uu2 = U[oo + 1] / rr2;
          double vv2 = U[oo + 2] / rr2;
          double pp2 = gm1 * (U[oo + 3] - 0.5 * rr2 * (uu2 * uu2 + vv2 * vv2));
          if (pp2 < 1e-8) pp2 = 1e-8;
          hllc(r, u, v, pp, rr2, uu2, vv2, pp2, fW[id * 2], fW[id * 2 + 1], gamma);
        }
        d0 += F4[0];
        d1 += F4[1];
        d2 += F4[2];
        d3 += F4[3];
        if (i == NI - 1) {
          hllc(r, u, v, pp, r, u, v, pp, fE[id * 2], fE[id * 2 + 1], gamma);
        } else {
          int oo = (id + 1) * 4;
          double rr2 = U[oo] > 1e-8 ? U[oo] : 1e-8;
          double uu2 = U[oo + 1] / rr2;
          double vv2 = U[oo + 2] / rr2;
          double pp2 = gm1 * (U[oo + 3] - 0.5 * rr2 * (uu2 * uu2 + vv2 * vv2));
          if (pp2 < 1e-8) pp2 = 1e-8;
          hllc(r, u, v, pp, rr2, uu2, vv2, pp2, fE[id * 2], fE[id * 2 + 1], gamma);
        }
        d0 += F4[0];
        d1 += F4[1];
        d2 += F4[2];
        d3 += F4[3];
        if (j == NJ - 1) {
          hllc(r, u, v, pp, rhoInf, uInf, 0.0, pInf, fN[id * 2], fN[id * 2 + 1], gamma);
        } else {
          int oo = (id + NI) * 4;
          double rr2 = U[oo] > 1e-8 ? U[oo] : 1e-8;
          double uu2 = U[oo + 1] / rr2;
          double vv2 = U[oo + 2] / rr2;
          double pp2 = gm1 * (U[oo + 3] - 0.5 * rr2 * (uu2 * uu2 + vv2 * vv2));
          if (pp2 < 1e-8) pp2 = 1e-8;
          hllc(r, u, v, pp, rr2, uu2, vv2, pp2, fN[id * 2], fN[id * 2 + 1], gamma);
        }
        d0 += F4[0];
        d1 += F4[1];
        d2 += F4[2];
        d3 += F4[3];
        if (j == 0) {
          d1 += pp * fS[id * 2];
          d2 += pp * fS[id * 2 + 1];
        } else {
          int oo = (id - NI) * 4;
          double rr2 = U[oo] > 1e-8 ? U[oo] : 1e-8;
          double uu2 = U[oo + 1] / rr2;
          double vv2 = U[oo + 2] / rr2;
          double pp2 = gm1 * (U[oo + 3] - 0.5 * rr2 * (uu2 * uu2 + vv2 * vv2));
          if (pp2 < 1e-8) pp2 = 1e-8;
          hllc(r, u, v, pp, rr2, uu2, vv2, pp2, fS[id * 2], fS[id * 2 + 1], gamma);
          d0 += F4[0];
          d1 += F4[1];
          d2 += F4[2];
          d3 += F4[3];
        }
        spec = hypot(u, v) + sqrt((gamma * pp) / r);
        per = fmax(hypot(fW[id * 2], fW[id * 2 + 1]), hypot(fE[id * 2], fE[id * 2 + 1]));
        per = fmax(per, hypot(fS[id * 2], fS[id * 2 + 1]));
        per = fmax(per, hypot(fN[id * 2], fN[id * 2 + 1]));
        per = fmax(per, 1e-8);
        dt = (CFL * (vol[id] / per)) / fmax(spec, 1e-6);
        scale = dt / fmax(vol[id], 1e-12);
        nr = U[o] - d0 * scale;
        nu = U[o + 1] - d1 * scale;
        nv = U[o + 2] - d2 * scale;
        nE = U[o + 3] - d3 * scale;
        rr = nr > 1e-8 ? nr : 1e-8;
        uu = nu / rr;
        vv = nv / rr;
        pNew = gm1 * (nE - 0.5 * rr * (uu * uu + vv * vv));
        if (pNew > 1e-6 && nr > 1e-8) {
          U[o] = nr;
          U[o + 1] = nu;
          U[o + 2] = nv;
          U[o + 3] = nE;
        }
        resSum += (d0 / fmax(vol[id], 1e-12)) * (d0 / fmax(vol[id], 1e-12));
      }
    }
    residual = sqrt(resSum / (NI * NJ));
  }

  {
    double pSum = 0.0;
    int nSum = 0;
    for (i = 0; i < NI; i++) {
      double x = ((i + 0.5) / NI) * Lx;
      int o;
      double r, u, v, p;
      if (x < x0 + 0.18 || x > Lx * 0.82) continue;
      o = i * 4;
      r = U[o] > 1e-8 ? U[o] : 1e-8;
      u = U[o + 1] / r;
      v = U[o + 2] / r;
      p = gm1 * (U[o + 3] - 0.5 * r * (u * u + v * v));
      if (p < 1e-8) p = 1e-8;
      pSum += p;
      nSum++;
    }
    out.p2p1 = nSum > 0 ? pSum / nSum : 1.0;
    out.exactP = exact.p2p1;
    out.relErr = fabs(out.p2p1 - out.exactP) / fmax(out.exactP, 1e-9);
    out.residual = residual;
  }
  return out;
}

static int g_fail = 0;

static void expect_abs(const char *name, double got, double expected, double tol) {
  double err = fabs(got - expected);
  int ok = err <= tol;
  if (!ok) g_fail++;
  printf("%s  %-28s got %.6f expect %.6f err %.3g\n", ok ? "PASS" : "FAIL", name, got, expected, err);
}

static void expect_rel(const char *name, double got, double expected, double tol) {
  double err = fabs(got - expected) / fmax(fabs(expected), 1e-12);
  int ok = err <= tol;
  if (!ok) g_fail++;
  printf("%s  %-28s got %.6f expect %.6f rel %.3g\n", ok ? "PASS" : "FAIL", name, got, expected, err);
}

static int cmd_check(void) {
  double beta, cp8, aft, exactAft;
  double th[16];
  EulerOut eu;
  int i;
  Shock sh40;
  g_fail = 0;
  expect_abs("normal M=2 p2/p1", normal_p(2.0, 1.4), 4.5, 1e-12);
  expect_abs("isentropic M=5 T0/T", isen_Tt(5.0, 1.4), 6.0, 1e-12);
  beta = beta_from_theta(2.0, 10.0 * DEG, 1.4);
  expect_abs("beta M=2 theta=10 deg", beta / DEG, 39.32, 0.2);
  sh40 = oblique(2.0, 40.0 * DEG, 1.4);
  expect_abs("oblique M=2 beta=40 p2/p1", sh40.p2p1, 1.0 + (2.8 / 2.4) * (pow(2.0 * sin(40.0 * DEG), 2.0) - 1.0), 1e-9);
  {
    double b8 = beta_from_theta(8.0, 8.0 * DEG, 1.4);
    Shock s8 = oblique(8.0, b8, 1.4);
    double q = 0.5 * 1.4 * 64.0;
    cp8 = (s8.p2p1 - 1.0) / q;
  }
  expect_abs("wedge Cp M=8 theta=8", cp8, 0.0656, 5e-4);
  for (i = 0; i < 8; i++) th[i] = 30.0 * DEG;
  for (i = 8; i < 16; i++) th[i] = 0.0;
  aft = march_strip(8.0, 1.4, th, 16);
  exactAft = analytic_aft(8.0, 30.0 * DEG, 0.0, 1.4);
  expect_abs("strip aft Cp vs analytic", aft, exactAft, 2e-3);
  expect_abs("strip aft Cp M=8 30to0", aft, 0.0265, 1e-3);
  eu = euler_wedge(2.0, 10.0, 1.4);
  printf("      euler M=2 theta=10     p2/p1 %.4f exact %.4f rel %.2f%% residual %.3e\n", eu.p2p1, eu.exactP, eu.relErr * 100.0, eu.residual);
  expect_rel("euler p2/p1 vs oblique", eu.p2p1, eu.exactP, 0.08);
  if (g_fail) {
    printf("FAIL  %d checks\n", g_fail);
    return 1;
  }
  printf("PASS\n");
  return 0;
}

int main(int argc, char **argv) {
  int i;
  int check = 0;
  for (i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--check") == 0) check = 1;
  }
  if (check || argc < 2) return cmd_check();
  fprintf(stderr, "usage: bowshock_kernel --check\n");
  return 2;
}
