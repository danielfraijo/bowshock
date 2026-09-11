#!/usr/bin/env python3
"""Bowshock — watertight waverider CAD for CFD.

Zero third-party dependencies. Python 3.9+.

Exports STL (ASCII + binary), STEP AP214 (faceted B-rep), IGES (NURBS 128),
Plot3D XYZ, and OBJ. Coordinate frame: X streamwise (nose at origin), Y span,
Z up. Lengths in metres internally; --unit scales the files.

Examples
--------
  python waverider_cad.py --type caret --mach 8 --stl wr.stl --step wr.step
  python waverider_cad.py --config design.json --out kit/
  python waverider_cad.py --sweep-mach 5,6,8,10 --type osculating --out family/
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
from dataclasses import asdict, dataclass
from typing import List, Sequence, Tuple

Vec3 = Tuple[float, float, float]
DEG = math.pi / 180.0


@dataclass
class Design:
    family: str = "caret"
    mach: float = 8.0
    gamma: float = 1.4
    length: float = 4.0
    span: float = 2.2
    height: float = 0.55
    shock_deg: float = 16.0
    cone_deg: float = 8.0
    super_n: float = 2.2
    planform: str = "delta"
    planform_power: float = 1.15
    fins: int = 4
    wedge_frac: float = 0.42
    capture_frac: float = 0.55
    nx: int = 48
    ny: int = 36
    le_radius: float = 0.0
    half_model: bool = False
    unit: str = "m"
    name: str = "waverider"
    lid: str = "top"
    dihedral_deg: float = 0.0
    camber: float = 0.0
    te_sweep_deg: float = 0.0
    elevon_deg: float = 0.0
    fin_height: float = 0.0
    n_ramps: int = 2
    cowl_side: str = "belly"
    inlet_height: float = 0.14
    cowl_frac: float = 0.38
    combustor_frac: float = 0.22
    nozzle_er: float = 4.0
    ramp_deg: float = 10.0
    flow_through: bool = False


def unit_scale(unit: str) -> float:
    if unit == "mm":
        return 1000.0
    if unit == "in":
        return 1.0 / 0.0254
    return 1.0


def clamp(v: float, a: float, b: float) -> float:
    return a if v < a else b if v > b else v


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def vsub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vcross(a: Vec3, b: Vec3) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def vdot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vlen(a: Vec3) -> float:
    return math.hypot(a[0], a[1], a[2])


def vnorm(a: Vec3) -> Vec3:
    n = vlen(a)
    return (0.0, 0.0, 0.0) if n < 1e-15 else (a[0] / n, a[1] / n, a[2] / n)


def theta_from_beta_m(M: float, beta: float, gamma: float = 1.4) -> float:
    s, c = math.sin(beta), math.cos(beta)
    if abs(s) < 1e-12:
        return 0.0
    M2 = M * M
    num = 2.0 * (c / s) * (M2 * s * s - 1.0)
    den = M2 * (gamma + math.cos(2.0 * beta)) + 2.0
    return math.atan(num / den)


def max_theta(M: float, gamma: float = 1.4) -> Tuple[float, float]:
    mu = math.asin(clamp(1.0 / M, 0.0, 1.0))
    best_t, best_b = 0.0, mu
    for i in range(1, 80):
        b = mu + (math.pi / 2.0 - 1e-4 - mu) * i / 80.0
        t = theta_from_beta_m(M, b, gamma)
        if t > best_t:
            best_t, best_b = t, b
    return best_t, best_b


def beta_from_theta_m(M: float, theta: float, gamma: float = 1.4) -> float:
    if theta <= 1e-10:
        return math.asin(clamp(1.0 / M, 0.0, 1.0))
    cap_t, cap_b = max_theta(M, gamma)
    if theta >= cap_t * 0.999:
        return float("nan")
    mu = math.asin(clamp(1.0 / M, 0.0, 1.0))
    lo, hi = mu + 1e-6, cap_b
    for _ in range(48):
        mid = 0.5 * (lo + hi)
        if theta_from_beta_m(M, mid, gamma) < theta:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def resolve_shock(d: Design) -> Tuple[float, float, bool]:
    theta = math.atan(d.height / max(d.length, 1e-9))
    cap_t, cap_b = max_theta(d.mach, d.gamma)
    attached = theta < cap_t * 0.98
    if not attached:
        theta = cap_t * 0.96
    beta = beta_from_theta_m(d.mach, theta, d.gamma)
    if not math.isfinite(beta):
        beta = cap_b
        attached = False
    user = d.shock_deg * DEG
    mu = math.asin(clamp(1.0 / d.mach, 0.0, 1.0))
    if d.family in ("caret", "star") and mu + 0.2 * DEG < user < 70 * DEG:
        t_user = theta_from_beta_m(d.mach, user, d.gamma)
        if t_user > 0:
            return t_user, user, True
    return theta, beta, attached


class Mesh:
    def __init__(self, length: float = 1.0) -> None:
        self._q = 1e10 / max(length, 1e-6)
        self._key = {}
        self.pos: List[Vec3] = []
        self.idx: List[Tuple[int, int, int]] = []
        self.surf: List[int] = []
        self.skipped = 0

    def vert(self, x: float, y: float, z: float) -> int:
        key = (round(x * self._q), round(y * self._q), round(z * self._q))
        i = self._key.get(key)
        if i is not None:
            return i
        i = len(self.pos)
        self._key[key] = i
        self.pos.append((x, y, z))
        return i

    def tri(self, a: int, b: int, c: int, surface: int = 0) -> None:
        if a == b or b == c or c == a:
            self.skipped += 1
            return
        pa, pb, pc = self.pos[a], self.pos[b], self.pos[c]
        n = vcross(vsub(pb, pa), vsub(pc, pa))
        if vlen(n) < 1e-16:
            self.skipped += 1
            return
        self.idx.append((a, b, c))
        self.surf.append(surface)

    def quad(self, a: int, b: int, c: int, d: int, surface: int = 0) -> None:
        self.tri(a, b, c, surface)
        self.tri(a, c, d, surface)

    def fan(self, loop: Sequence[int], surface: int = 2, reverse: bool = False) -> None:
        pts = list(reversed(loop)) if reverse else list(loop)
        if len(pts) < 3:
            return
        c = pts[0]
        for i in range(1, len(pts) - 1):
            self.tri(c, pts[i], pts[i + 1], surface)


@dataclass
class Grid:
    name: str
    ni: int
    nj: int
    xyz: List[Vec3]

    def at(self, i: int, j: int) -> Vec3:
        return self.xyz[i * self.nj + j]


def make_grid(name: str, ni: int, nj: int, sample) -> Grid:
    xyz = []
    for i in range(ni):
        for j in range(nj):
            xyz.append(sample(i, j))
    return Grid(name, ni, nj, xyz)


def cosine_space(i: int, n: int) -> float:
    if n <= 1:
        return 0.0
    return 0.5 * (1.0 - math.cos(math.pi * i / (n - 1)))


def cluster(i: int, n: int, power: float = 1.35) -> float:
    if n <= 1:
        return 0.0
    u = i / (n - 1)
    return u if power <= 1.001 else 1.0 - (1.0 - u) ** power


def half_span_list(ny: int, s: float, half: bool) -> List[float]:
    n = max(4 if half else 5, math.ceil(ny / 2) if half else ny)
    ys = []
    for j in range(n):
        t = cosine_space(j, n)
        ys.append(s * t if half else -s + 2.0 * s * t)
    return ys


def x_leading(y: float, L: float, s: float, planform: str, p: float, spatular: float) -> float:
    yn = clamp(abs(y) / max(s, 1e-12), 0.0, 1.0)
    if planform == "rect":
        return 0.0
    if planform == "spatular":
        nose = clamp(spatular, 0.04, 0.45)
        blunt = 1.0 - math.sqrt(max(0.0, 1.0 - yn * yn))
        return min(L * lerp(nose * blunt, yn, 0.35), L * 0.985)
    if planform == "double":
        kink = 0.42
        if yn < kink:
            x = L * 0.1 * (yn / kink)
        else:
            x = L * (0.1 + 0.9 * ((yn - kink) / (1.0 - kink)) ** 1.05)
        return min(x, L * 0.985)
    pow_ = 1.0 if planform == "delta" else clamp(p, 0.6, 2.4)
    return min(L * (yn ** pow_), L * 0.985)


def super_z(y: float, s: float, h: float, n: float) -> float:
    yn = min(clamp(abs(y) / max(s, 1e-12), 0.0, 1.0), 0.96)
    return -h * (1.0 - yn ** n) ** (1.0 / n)


def stitch(mesh: Mesh, g: Grid, surface: int, flip: bool = False) -> None:
    for i in range(g.ni - 1):
        for j in range(g.nj - 1):
            a = mesh.vert(*g.at(i, j))
            b = mesh.vert(*g.at(i + 1, j))
            c = mesh.vert(*g.at(i + 1, j + 1))
            d = mesh.vert(*g.at(i, j + 1))
            if flip:
                mesh.quad(a, d, c, b, surface)
            else:
                mesh.quad(a, b, c, d, surface)


def close_vehicle(mesh: Mesh, upper: Grid, lower: Grid, half: bool, flow_through: bool = False) -> None:
    stitch(mesh, upper, 0, False)
    stitch(mesh, lower, 1, True)
    if not flow_through:
        te_u = [mesh.vert(*upper.at(upper.ni - 1, j)) for j in range(upper.nj)]
        te_l = [mesh.vert(*lower.at(lower.ni - 1, j)) for j in range(lower.nj)]
        for j in range(upper.nj - 1):
            mesh.quad(te_u[j], te_l[j], te_l[j + 1], te_u[j + 1], 2)
    js = [upper.nj - 1] if half else [0, upper.nj - 1]
    for j in js:
        for i in range(upper.ni - 1):
            u0 = mesh.vert(*upper.at(i, j))
            u1 = mesh.vert(*upper.at(i + 1, j))
            l0 = mesh.vert(*lower.at(i, j))
            l1 = mesh.vert(*lower.at(i + 1, j))
            if j == 0:
                mesh.quad(u0, l0, l1, u1, 3)
            else:
                mesh.quad(u0, u1, l1, l0, 3)
    if half:
        su = [mesh.vert(*upper.at(i, 0)) for i in range(upper.ni)]
        sl = [mesh.vert(*lower.at(i, 0)) for i in range(lower.ni)]
        n = min(len(su), len(sl))
        for i in range(n - 1):
            mesh.quad(su[i], su[i + 1], sl[i + 1], sl[i], 4)
    if not flow_through:
        le_u = [mesh.vert(*upper.at(0, j)) for j in range(upper.nj)]
        le_l = [mesh.vert(*lower.at(0, j)) for j in range(lower.nj)]
        for j in range(upper.nj - 1):
            mesh.quad(le_u[j], le_u[j + 1], le_l[j + 1], le_l[j], 5)


def analyze(mesh: Mesh) -> dict:
    edges = {}
    volume = 0.0
    area = 0.0
    open_e = non = 0
    for a, b, c in mesh.idx:
        pa, pb, pc = mesh.pos[a], mesh.pos[b], mesh.pos[c]
        n = vcross(vsub(pb, pa), vsub(pc, pa))
        area += 0.5 * vlen(n)
        volume += vdot(pa, n) / 6.0
        for u, v in ((a, b), (b, c), (c, a)):
            key = (u, v) if u < v else (v, u)
            edges[key] = edges.get(key, 0) + 1
    for n in edges.values():
        if n == 1:
            open_e += 1
        elif n != 2:
            non += 1
    return {
        "vertices": len(mesh.pos),
        "triangles": len(mesh.idx),
        "volume": abs(volume),
        "area": area,
        "watertight": open_e == 0 and len(mesh.idx) > 0,
        "manifold": open_e == 0 and non == 0 and len(mesh.idx) > 0,
        "open_edges": open_e,
        "nonmanifold": non,
    }


def _loft(d: Design, z_base, planform: str, power: float, spat: float, z_exp: float = 1.0) -> Tuple[Grid, Grid]:
    L, s = d.length, d.span / 2.0
    ys = half_span_list(d.ny, s, d.half_model)
    nx, nj = max(8, d.nx), len(ys)
    dih = math.tan(d.dihedral_deg * DEG)
    cam = d.camber * d.height
    te_tan = math.tan(d.te_sweep_deg * DEG)
    z_pow = max(0.6, z_exp)

    def xle(y: float) -> float:
        return x_leading(y, L, s, planform, power, spat)

    def xt(y: float, xl: float) -> float:
        return max(L - abs(y) * te_tan, xl + 0.08 * (L - xl))

    def up(i, j):
        y = ys[j]
        xl = xle(y)
        xe = xt(y, xl)
        x = lerp(xl, xe, cluster(i, nx))
        xi = (x - xl) / max(xe - xl, 1e-12)
        z_lid = 4.0 * cam * xi * (1.0 - xi)
        return (x, y, z_lid + abs(y) * dih)

    def lo(i, j):
        y = ys[j]
        xl = xle(y)
        xe = xt(y, xl)
        x = lerp(xl, xe, cluster(i, nx))
        frac = (x - xl) / max(xe - xl, 1e-12)
        z = z_base(y) * (frac ** z_pow)
        z_lid = 4.0 * cam * frac * (1.0 - frac)
        return (x, y, z + z_lid + abs(y) * dih)

    return make_grid("upper", nx, nj, up), make_grid("lower", nx, nj, lo)


def build_caret(d: Design) -> Tuple[Grid, Grid]:
    theta, _b, _ = resolve_shock(d)
    h = d.length * math.tan(theta)
    s = d.span / 2.0
    return _loft(d, lambda y: -h * (1.0 - min(clamp(abs(y) / max(s, 1e-12), 0.0, 1.0), 0.96)), "delta", 1.0, 0.0)


def build_osculating(d: Design) -> Tuple[Grid, Grid]:
    s = d.span / 2.0
    n = clamp(d.super_n, 1.0, 8.0)
    return _loft(d, lambda y: super_z(y, s, d.height, n), d.planform, d.planform_power, d.capture_frac)


def build_cone(d: Design) -> Tuple[Grid, Grid]:
    s = d.span / 2.0
    return _loft(d, lambda y: super_z(y, s, d.height, 2.15), "spatular", 1.0, d.capture_frac)


def build_wedgecone(d: Design) -> Tuple[Grid, Grid]:
    s = d.span / 2.0
    h, yw = d.height, clamp(d.wedge_frac, 0.08, 0.9) * s
    n = clamp(d.super_n, 1.2, 6.0)

    def z_base(y: float) -> float:
        ay = abs(y)
        if ay <= yw:
            return -h
        t = min((ay - yw) / max(s - yw, 1e-12), 0.96)
        return -h * (1.0 - t ** n) ** (1.0 / n)

    return _loft(d, z_base, d.planform, d.planform_power, 0.2)


def build_viscopt(d: Design) -> Tuple[Grid, Grid]:
    s = d.span / 2.0
    n = clamp(d.super_n, 1.05, 5.0)
    return _loft(d, lambda y: super_z(y, s, d.height, n), d.planform if d.planform != "rect" else "power", d.planform_power, d.capture_frac)


def build_inward(d: Design) -> Tuple[Grid, Grid]:
    s = d.span / 2.0
    h = d.height
    wall = clamp(d.wedge_frac, 0.35, 0.88)

    def z_base(y: float) -> float:
        yn = clamp(abs(y) / max(s, 1e-12), 0.0, 0.97)
        if yn <= wall:
            return -h
        t = (yn - wall) / max(1.0 - wall, 1e-9)
        return -h * (1.0 - t * t)

    return _loft(d, z_base, d.planform, d.planform_power, d.capture_frac)


def build_elliptic(d: Design) -> Tuple[Grid, Grid]:
    s, h = d.span / 2.0, d.height
    pf = "delta" if d.planform == "rect" else d.planform
    return _loft(
        d,
        lambda y: -h * math.sqrt(max(0.0, 1.0 - clamp(abs(y) / max(s, 1e-12), 0.0, 0.999) ** 2)),
        pf,
        d.planform_power,
        d.capture_frac,
        1.0,
    )


def build_busemann(d: Design) -> Tuple[Grid, Grid]:
    s, h = d.span / 2.0, d.height

    def z_base(y: float) -> float:
        yn = clamp(abs(y) / max(s, 1e-12), 0.0, 0.97)
        circ = math.sqrt(max(0.0, 1.0 - yn * yn))
        return -h * (0.28 + 0.72 * circ)

    return _loft(d, z_base, "spatular", 1.0, clamp(d.capture_frac, 0.08, 0.35), 0.72)


def build_liftbody(d: Design) -> Tuple[Grid, Grid]:
    L, s, h = d.length, d.span / 2.0, d.height
    nx = max(8, d.nx)
    ys = half_span_list(d.ny, 1.0, d.half_model)
    nj = len(ys)
    pow_ = 1.0 if d.planform == "delta" else clamp(d.planform_power, 0.55, 1.6)

    def up(i, j):
        t = cluster(i, nx)
        x = t * L
        a = s * max(t, 0.04) ** (0.85 if pow_ == 1 else pow_ * 0.7)
        b = 0.5 * h * max(t, 0.06) ** 0.55
        y = ys[j] * a
        yn = clamp(abs(ys[j]), 0.0, 1.0)
        return (x, y, b * math.sqrt(max(0.0, 1.0 - yn * yn)))

    def lo(i, j):
        t = cluster(i, nx)
        x = t * L
        a = s * max(t, 0.04) ** (0.85 if pow_ == 1 else pow_ * 0.7)
        b = 0.55 * h * max(t, 0.06) ** 0.55
        y = ys[j] * a
        yn = clamp(abs(ys[j]), 0.0, 1.0)
        return (x, y, -b * math.sqrt(max(0.0, 1.0 - yn * yn)))

    return make_grid("upper", nx, nj, up), make_grid("lower", nx, nj, lo)


def build_ramjet(d: Design) -> Tuple[Grid, Grid]:
    L, s = d.length, d.span / 2.0
    h_in = max(0.04, d.inlet_height)
    h_max = max(h_in * 1.2, d.height)
    th = d.ramp_deg * DEG
    x1 = L * 0.2
    x2 = L * clamp(d.cowl_frac, 0.28, 0.55)
    x3 = min(L * 0.92, x2 + L * clamp(d.combustor_frac, 0.12, 0.4))
    er = max(1.4, d.nozzle_er)
    z_iso = -x1 * math.tan(th) - 1e-4
    z_exit = z_iso * er

    def z_bot(x: float) -> float:
        if x <= x1:
            return -x * math.tan(th)
        if x <= x3:
            return z_iso
        u = (x - x3) / max(L - x3, 1e-9)
        return lerp(z_iso, min(z_exit, -h_max), u)

    def z_top(x: float) -> float:
        cowl = 0.012 * h_max
        if x < x2:
            return cowl + h_in * 0.15 * (1.0 - x / max(x2, 1e-9))
        if x < x3:
            return cowl
        u = (x - x3) / max(L - x3, 1e-9)
        return lerp(cowl, cowl + 0.25 * h_in * (er - 1.0), u)

    ys = half_span_list(d.ny, s, d.half_model)
    nx, nj = max(10, d.nx), len(ys)
    upper = make_grid("upper", nx, nj, lambda i, j: ((i / (nx - 1)) * L, ys[j], z_top((i / (nx - 1)) * L)))
    lower = make_grid("lower", nx, nj, lambda i, j: ((i / (nx - 1)) * L, ys[j], z_bot((i / (nx - 1)) * L)))
    return upper, lower


def build_scramjet(d: Design) -> Tuple[Grid, Grid]:
    L, s = d.length, d.span / 2.0
    h_in = max(0.04, d.inlet_height)
    h_max = max(h_in * 1.2, d.height)
    th = d.ramp_deg * DEG
    n_r = int(clamp(d.n_ramps, 1, 3))
    x_ramp = L * (0.1 + 0.07 * n_r)
    x2 = L * clamp(d.cowl_frac, 0.28, 0.58)
    x3 = min(L * 0.88, x2 + L * clamp(d.combustor_frac, 0.1, 0.4))
    er = max(1.4, d.nozzle_er)

    def z_ramp(x: float) -> float:
        dx = x_ramp / n_r
        z = 0.0
        x0 = 0.0
        for k in range(n_r):
            x1 = dx * (k + 1)
            slope = math.tan(th * (k + 1) / n_r)
            if x <= x1 + 1e-12:
                return z - (x - x0) * slope
            z -= (x1 - x0) * slope
            x0 = x1
        return z

    z_iso = z_ramp(x_ramp) - 1e-4
    z_comb = z_iso * 1.08
    z_exit = z_iso * er

    def z_bot(x: float) -> float:
        if x <= x_ramp:
            return z_ramp(x)
        if x <= x2:
            return z_iso
        if x <= x3:
            u = (x - x2) / max(x3 - x2, 1e-9)
            return lerp(z_iso, z_comb, u)
        u = (x - x3) / max(L - x3, 1e-9)
        return lerp(z_comb, min(z_exit, -h_max), u)

    def z_top(x: float) -> float:
        cowl = 0.012 * h_max
        if x < x2:
            return cowl + h_in * 0.15 * (1.0 - x / max(x2, 1e-9))
        if x < x3:
            return cowl
        u = (x - x3) / max(L - x3, 1e-9)
        return lerp(cowl, cowl + 0.25 * h_in * (er - 1.0), u)

    ys = half_span_list(d.ny, s, d.half_model)
    nx, nj = max(10, d.nx), len(ys)
    upper = make_grid("upper", nx, nj, lambda i, j: ((i / (nx - 1)) * L, ys[j], z_top((i / (nx - 1)) * L)))
    lower = make_grid("lower", nx, nj, lambda i, j: ((i / (nx - 1)) * L, ys[j], z_bot((i / (nx - 1)) * L)))
    return upper, lower


def build_integrated(d: Design) -> Tuple[Grid, Grid]:
    upper, lower = build_osculating(d)
    L, s = d.length, d.span / 2.0
    yw = clamp(d.wedge_frac, 0.12, 0.55) * s
    x_c = L * clamp(d.cowl_frac, 0.28, 0.7)
    h_in = max(0.03, d.inlet_height)
    for i in range(lower.ni):
        for j in range(lower.nj):
            x, y, z = lower.at(i, j)
            if abs(y) > yw or x < x_c:
                continue
            u = (x - x_c) / max(0.08 * L, 1e-9)
            s_u = 1.0 if u >= 1.0 else u * u * (3.0 - 2.0 * u)
            yf = 1.0 - (abs(y) / yw) ** 2
            sign = 1.0 if d.cowl_side == "dorsal" else -1.0
            if d.cowl_side == "dorsal":
                ux, uy, uz = upper.at(i, j)
                upper.xyz[i * upper.nj + j] = (ux, uy, uz + h_in * s_u * yf)
            else:
                lower.xyz[i * lower.nj + j] = (x, y, z - h_in * s_u * yf)
    return upper, lower


def flip_z_grid(g: Grid) -> None:
    g.xyz = [(p[0], p[1], -p[2]) for p in g.xyz]


def flip_z_mesh(mesh: Mesh) -> None:
    mesh.pos = [(p[0], p[1], -p[2]) for p in mesh.pos]


def build_star_mesh(d: Design) -> Tuple[Mesh, List[Grid]]:
    L = d.length
    theta, _b, _ = resolve_shock(d)
    h = L * math.tan(theta)
    r_out = d.span / 2.0
    r_in = max(0.08 * r_out, r_out - h)
    fins = int(clamp(d.fins, 3, 8))
    nx = max(8, d.nx)
    n_per = max(4, round(d.ny / fins))
    mesh = Mesh(L)
    grids: List[Grid] = []

    def ang(k: int, inner: bool) -> float:
        a0 = (2.0 * math.pi * k) / fins - math.pi / 2.0
        return a0 + math.pi / fins if inner else a0

    def pt(x: float, k: int, inner: bool) -> Vec3:
        r = (x / L) * (r_in if inner else r_out)
        a = ang(k, inner)
        return (x, r * math.cos(a), r * math.sin(a))

    for f in range(fins):
        k0, k1 = f, (f + 1) % fins

        def ga(i, j, k0=k0):
            x = (i / (nx - 1)) * L
            t = j / (n_per - 1)
            a, c = pt(x, k0, False), pt(x, k0, True)
            return (x, lerp(a[1], c[1], t), lerp(a[2], c[2], t))

        def gb(i, j, k0=k0, k1=k1):
            x = (i / (nx - 1)) * L
            t = j / (n_per - 1)
            c, a = pt(x, k0, True), pt(x, k1, False)
            return (x, lerp(c[1], a[1], t), lerp(c[2], a[2], t))

        g1 = make_grid(f"fin{f}-a", nx, n_per, ga)
        g2 = make_grid(f"fin{f}-b", nx, n_per, gb)
        stitch(mesh, g1, 0, False)
        stitch(mesh, g2, 1, False)
        grids.extend((g1, g2))

    outline = []
    for f in range(fins):
        gA = grids[2 * f]
        gB = grids[2 * f + 1]
        for j in range(gA.nj):
            outline.append(mesh.vert(*gA.at(gA.ni - 1, j)))
        for j in range(1, gB.nj):
            outline.append(mesh.vert(*gB.at(gB.ni - 1, j)))
    core = mesh.vert(L, 0.0, 0.0)
    n = len(outline)
    for i in range(n):
        mesh.tri(core, outline[i], outline[(i + 1) % n], 2)
    return mesh, grids


def build(d: Design) -> Tuple[Mesh, List[Grid], dict]:
    if d.family == "star":
        mesh, grids = build_star_mesh(d)
        if d.lid == "bottom":
            flip_z_mesh(mesh)
            for g in grids:
                flip_z_grid(g)
    else:
        if d.family == "caret":
            upper, lower = build_caret(d)
        elif d.family == "cone":
            upper, lower = build_cone(d)
        elif d.family == "wedgecone":
            upper, lower = build_wedgecone(d)
        elif d.family == "viscopt":
            upper, lower = build_viscopt(d)
        elif d.family == "inward":
            upper, lower = build_inward(d)
        elif d.family == "elliptic":
            upper, lower = build_elliptic(d)
        elif d.family == "busemann":
            upper, lower = build_busemann(d)
        elif d.family == "liftbody":
            upper, lower = build_liftbody(d)
        elif d.family == "ramjet":
            upper, lower = build_ramjet(d)
        elif d.family == "scramjet":
            upper, lower = build_scramjet(d)
        elif d.family == "integrated":
            upper, lower = build_integrated(d)
        else:
            upper, lower = build_osculating(d)
        if d.lid == "bottom":
            flip_z_grid(upper)
            flip_z_grid(lower)
        if abs(d.elevon_deg) > 1e-3:
            hinge = 0.82 * d.length
            k = math.tan(d.elevon_deg * DEG)
            for g in (upper, lower):
                xyz = []
                for p in g.xyz:
                    z = p[2] - (p[0] - hinge) * k if p[0] > hinge else p[2]
                    xyz.append((p[0], p[1], z))
                g.xyz = xyz
        mesh = Mesh(d.length)
        duct = bool(d.flow_through) and d.family in ("ramjet", "scramjet")
        close_vehicle(mesh, upper, lower, d.half_model, duct)
        grids = [upper, lower]
    snap_nose(mesh, grids)
    return mesh, grids, analyze(mesh)


def snap_nose(mesh: Mesh, grids: List[Grid]) -> None:
    """Put the vehicle tip at the origin (X stream, Y span, Z up)."""
    if not mesh.pos:
        return
    xmin = min(p[0] for p in mesh.pos)
    z_tip = 0.0
    best_y = 1e9
    for p in mesh.pos:
        if p[0] - xmin > 1e-8:
            continue
        if abs(p[1]) < best_y:
            best_y = abs(p[1])
            z_tip = p[2]
    mesh.pos = [(p[0] - xmin, p[1], p[2] - z_tip) for p in mesh.pos]
    for g in grids:
        g.xyz = [(p[0] - xmin, p[1], p[2] - z_tip) for p in g.xyz]


def scale_mesh(mesh: Mesh, s: float) -> None:
    if s == 1.0:
        return
    mesh.pos = [(p[0] * s, p[1] * s, p[2] * s) for p in mesh.pos]


def scale_grids(grids: List[Grid], s: float) -> None:
    if s == 1.0:
        return
    for g in grids:
        g.xyz = [(p[0] * s, p[1] * s, p[2] * s) for p in g.xyz]


def write_stl_ascii(path: str, mesh: Mesh, name: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"solid {name}\n")
        for a, b, c in mesh.idx:
            pa, pb, pc = mesh.pos[a], mesh.pos[b], mesh.pos[c]
            n = vnorm(vcross(vsub(pb, pa), vsub(pc, pa)))
            f.write(f"  facet normal {n[0]} {n[1]} {n[2]}\n    outer loop\n")
            for p in (pa, pb, pc):
                f.write(f"      vertex {p[0]} {p[1]} {p[2]}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def write_stl_binary(path: str, mesh: Mesh, name: str) -> None:
    header = f"Bowshock {name} watertight STL".encode("ascii", "replace")[:80]
    header = header + b"\x00" * (80 - len(header))
    with open(path, "wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(mesh.idx)))
        for a, b, c in mesh.idx:
            pa, pb, pc = mesh.pos[a], mesh.pos[b], mesh.pos[c]
            n = vnorm(vcross(vsub(pb, pa), vsub(pc, pa)))
            f.write(struct.pack("<12fH", *n, *pa, *pb, *pc, 0))


def _s(n: float) -> str:
    if not math.isfinite(n) or abs(n) < 1e-15:
        return "0."
    t = f"{n:.14g}"
    return t if ("." in t or "e" in t or "E" in t) else t + "."


def write_step(path: str, mesh: Mesh, name: str, unit: str) -> None:
    lines = []
    n = [0]

    def add(entity: str) -> int:
        n[0] += 1
        lines.append(f"#{n[0]} = {entity};")
        return n[0]

    hdr = (
        "ISO-10303-21;\nHEADER;\n"
        "FILE_DESCRIPTION(('Bowshock watertight waverider'),'2;1');\n"
        f"FILE_NAME('{name}.step','2026-01-01T00:00:00',('Bowshock'),('Bowshock'),"
        "'Bowshock CAD','Bowshock','');\n"
        "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n"
        "ENDSEC;\nDATA;\n"
    )
    app = add("APPLICATION_CONTEXT('automotive design')")
    add(f"APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2010,#{app})")
    pctx = add(f"PRODUCT_CONTEXT('',#{app},'mechanical')")
    dctx = add(f"PRODUCT_DEFINITION_CONTEXT('',#{app},'design')")
    prod = add(f"PRODUCT('waverider','Waverider','inverse-design waverider',(#{pctx}))")
    pdf = add(f"PRODUCT_DEFINITION_FORMATION('','',#{prod})")
    pd = add(f"PRODUCT_DEFINITION('design','',#{pdf},#{dctx})")
    pds = add(f"PRODUCT_DEFINITION_SHAPE('','',#{pd})")
    if unit == "mm":
        lu = add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))")
    else:
        lu = add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.))")
    ang = add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))")
    sol = add("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())")
    um = add("LENGTH_MEASURE(1.E-8)")
    unc = add(f"UNCERTAINTY_MEASURE_WITH_UNIT(#{um},#{lu},'distance_accuracy_value','closure')")
    ctx = add(
        f"(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc})) "
        f"GLOBAL_UNIT_ASSIGNED_CONTEXT((#{lu},#{ang},#{sol})) REPRESENTATION_CONTEXT('3D',' '))"
    )
    pts = [add(f"CARTESIAN_POINT('',({_s(p[0])},{_s(p[1])},{_s(p[2])}))") for p in mesh.pos]
    faces = []
    for a, b, c in mesh.idx:
        pa, pb, pc = mesh.pos[a], mesh.pos[b], mesh.pos[c]
        nrm = vnorm(vcross(vsub(pb, pa), vsub(pc, pa)))
        if nrm == (0.0, 0.0, 0.0):
            continue
        ref = vnorm(vsub(pb, pa))
        if ref == (0.0, 0.0, 0.0):
            ref = (1.0, 0.0, 0.0)
        loop = add(f"POLY_LOOP('',(#{pts[a]},#{pts[b]},#{pts[c]}))")
        bnd = add(f"FACE_OUTER_BOUND('',#{loop},.T.)")
        dn = add(f"DIRECTION('',({_s(nrm[0])},{_s(nrm[1])},{_s(nrm[2])}))")
        dr = add(f"DIRECTION('',({_s(ref[0])},{_s(ref[1])},{_s(ref[2])}))")
        ax = add(f"AXIS2_PLACEMENT_3D('',#{pts[a]},#{dn},#{dr})")
        pl = add(f"PLANE('',#{ax})")
        faces.append(add(f"ADVANCED_FACE('',(#{bnd}),#{pl},.T.)"))
    shell = add("CLOSED_SHELL('',(" + ",".join(f"#{i}" for i in faces) + "))")
    solid = add(f"MANIFOLD_SOLID_BREP('Waverider',#{shell})")
    repr_ = add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{solid}),#{ctx})")
    add(f"SHAPE_DEFINITION_REPRESENTATION(#{pds},#{repr_})")
    with open(path, "w", encoding="ascii", errors="replace") as f:
        f.write(hdr)
        f.write("\n".join(lines))
        f.write("\nENDSEC;\nEND-ISO-10303-21;\n")


def write_plot3d(path: str, grids: List[Grid]) -> None:
    blocks = [g for g in grids if g.ni >= 2 and g.nj >= 2]
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"{len(blocks)}\n")
        for g in blocks:
            f.write(f"{g.ni} {g.nj} 1\n")
        for g in blocks:
            xs, ys, zs = [], [], []
            for j in range(g.nj):
                for i in range(g.ni):
                    p = g.at(i, j)
                    xs.append(p[0]); ys.append(p[1]); zs.append(p[2])

            def dump(arr):
                for k in range(0, len(arr), 4):
                    f.write(" ".join(f"{v:.8e}" for v in arr[k:k + 4]) + "\n")

            dump(xs); dump(ys); dump(zs)


def write_obj(path: str, mesh: Mesh) -> None:
    names = ["upper", "lower", "base", "leading", "symmetry"]
    with open(path, "w", encoding="utf-8") as f:
        f.write("# Bowshock waverider\no waverider\n")
        for p in mesh.pos:
            f.write(f"v {p[0]} {p[1]} {p[2]}\n")
        last = -1
        for t, (a, b, c) in enumerate(mesh.idx):
            s = mesh.surf[t] if t < len(mesh.surf) else 0
            if s != last:
                f.write(f"g {names[s] if s < len(names) else 'surf'}\n")
                last = s
            f.write(f"f {a + 1} {b + 1} {c + 1}\n")


def write_iges(path: str, grids: List[Grid], name: str) -> None:
    def pad(s: str) -> str:
        s = s[:80] if len(s) > 80 else s
        return s + " " * (80 - len(s))

    def knots(n_poles: int, deg: int) -> List[float]:
        p = min(deg, n_poles - 1)
        k = [0.0] * (p + 1)
        internal = n_poles - p - 1
        for i in range(1, internal + 1):
            k.append(i / (internal + 1))
        k.extend([1.0] * (p + 1))
        return k

    lines = [pad("Bowshock IGES,".ljust(72) + "S" + "1".rjust(7))]
    gsec = [
        "1H,,1H;,8HWAVERIDE,7HIGES5.3,",
        "8HBowshock,8HBowshock,32,38,6,308,15,",
        "8HBowshock,1.,2,1HM,32768,0.,15H20260101.000000,",
        "1.E-6,1000.,7HUnknown,7HUnknown,11,0,0;",
    ]
    for i, g in enumerate(gsec):
        lines.append(pad(g.ljust(72) + "G" + str(i + 1).rjust(7)))
    p_lines: List[str] = []
    d_lines: List[str] = []
    pcur, de = 1, 1
    for g in grids[:8]:
        cap_u, cap_v = 16, 14
        du = max(1, (g.ni - 1) // min(cap_u, max(g.ni - 1, 1)))
        dv = max(1, (g.nj - 1) // min(cap_v, max(g.nj - 1, 1)))
        ii = list(range(0, g.ni, du))
        jj = list(range(0, g.nj, dv))
        if ii[-1] != g.ni - 1:
            ii.append(g.ni - 1)
        if jj[-1] != g.nj - 1:
            jj.append(g.nj - 1)
        poles = [[g.at(i, j) for j in jj] for i in ii]
        k1, k2 = len(poles) - 1, len(poles[0]) - 1
        m1, m2 = min(3, k1), min(3, k2)
        vals: List[str] = [str(k1), str(k2), str(m1), str(m2), "0", "0", "0", "0", "0"]
        for kv in knots(k1 + 1, m1) + knots(k2 + 1, m2):
            vals.append(f"{kv:.6f}")
        vals.extend(["1"] * ((k1 + 1) * (k2 + 1)))
        for row in poles:
            for p in row:
                vals.extend((f"{p[0]:.8e}", f"{p[1]:.8e}", f"{p[2]:.8e}"))
        vals.extend(["0", "1", "0", "1"])
        payload = "128," + ",".join(vals) + ";"
        chunks = [payload[i:i + 64] for i in range(0, len(payload), 64)]
        for k, ch in enumerate(chunks):
            p_lines.append(pad(ch.ljust(64) + str(de).rjust(8) + "P" + str(pcur + k).rjust(7)))
        dir_a = f"{128:8d}{pcur:8d}{0:8d}{0:8d}{0:8d}{0:8d}{0:8d}{0:8d}"
        dir_b = f"{128:8d}{0:8d}{len(chunks):8d}{0:8d}{0:8d}        {0:8d}"
        d_lines.append(pad(dir_a[:72] + "D" + str(de).rjust(7)))
        d_lines.append(pad(dir_b[:72] + "D" + str(de + 1).rjust(7)))
        pcur += len(chunks)
        de += 2
    lines.extend(d_lines)
    lines.extend(p_lines)
    t = f"S{1:7d}G{len(gsec):7d}D{len(d_lines):7d}P{len(p_lines):7d}"
    lines.append(pad(t.ljust(72) + "T" + "1".rjust(7)))
    with open(path, "w", encoding="ascii", errors="replace") as f:
        f.write("\n".join(lines) + "\n")


def export_all(d: Design, prefix: str, out_dir: str = ".") -> dict:
    os.makedirs(out_dir, exist_ok=True)
    mesh, grids, q = build(d)
    sc = unit_scale(d.unit)
    scale_mesh(mesh, sc)
    scale_grids(grids, sc)
    base = os.path.join(out_dir, prefix)
    write_stl_binary(base + ".stl", mesh, d.name)
    write_stl_ascii(base + "_ascii.stl", mesh, d.name)
    write_step(base + ".step", mesh, d.name, d.unit)
    write_iges(base + ".igs", grids, d.name)
    write_plot3d(base + ".xyz", grids)
    write_obj(base + ".obj", mesh)
    with open(base + "_quality.json", "w", encoding="utf-8") as f:
        json.dump({"design": asdict(d), "quality": q}, f, indent=2)
    print(f"{prefix}: {q['triangles']} tris, watertight={q['watertight']}, vol={q['volume']:.6g}")
    return q


def design_from_args(ns: argparse.Namespace) -> Design:
    d = Design()
    if ns.config:
        with open(ns.config, encoding="utf-8") as f:
            data = json.load(f)
        mapping = {
            "shockDeg": "shock_deg", "coneDeg": "cone_deg", "superN": "super_n",
            "planformPower": "planform_power", "wedgeFrac": "wedge_frac",
            "captureFrac": "capture_frac", "leRadius": "le_radius", "halfModel": "half_model",
            "dihedralDeg": "dihedral_deg", "teSweepDeg": "te_sweep_deg", "elevonDeg": "elevon_deg",
            "finHeight": "fin_height", "nRamps": "n_ramps", "cowlSide": "cowl_side",
            "inletHeight": "inlet_height", "cowlFrac": "cowl_frac", "combustorFrac": "combustor_frac",
            "nozzleER": "nozzle_er", "rampDeg": "ramp_deg", "flowThrough": "flow_through",
        }
        norm = {mapping.get(k, k): v for k, v in data.items()}
        for k, v in norm.items():
            if hasattr(d, k):
                setattr(d, k, v)
    for k in ("family", "mach", "gamma", "length", "span", "height", "unit", "name", "planform", "lid"):
        v = getattr(ns, k, None)
        if v is not None:
            setattr(d, k, v)
    if ns.shock is not None:
        d.shock_deg = ns.shock
    if ns.cone is not None:
        d.cone_deg = ns.cone
    if ns.nx is not None:
        d.nx = ns.nx
    if ns.ny is not None:
        d.ny = ns.ny
    if ns.fins is not None:
        d.fins = ns.fins
    if ns.half:
        d.half_model = True
    if ns.le_radius is not None:
        d.le_radius = ns.le_radius
    return d


def main(argv: List[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Watertight waverider CAD (STL/STEP/IGES/Plot3D)")
    p.add_argument(
        "--type",
        dest="family",
        choices=[
            "caret", "cone", "osculating", "viscopt", "inward", "elliptic", "busemann",
            "star", "wedgecone", "liftbody", "ramjet", "scramjet", "integrated",
        ],
    )
    p.add_argument("--mach", type=float)
    p.add_argument("--gamma", type=float)
    p.add_argument("--length", type=float)
    p.add_argument("--span", type=float)
    p.add_argument("--height", type=float)
    p.add_argument("--shock", type=float, help="shock angle, degrees")
    p.add_argument("--cone", type=float, help="cone half-angle, degrees")
    p.add_argument("--planform", choices=["delta", "power", "spatular", "rect", "double"])
    p.add_argument("--nx", type=int)
    p.add_argument("--ny", type=int)
    p.add_argument("--fins", type=int)
    p.add_argument("--le-radius", type=float, dest="le_radius")
    p.add_argument("--half", action="store_true")
    p.add_argument("--lid", choices=["top", "bottom"])
    p.add_argument("--unit", choices=["m", "mm", "in"])
    p.add_argument("--name", type=str)
    p.add_argument("--config", type=str)
    p.add_argument("--out", type=str, default=".")
    p.add_argument("--stl", type=str)
    p.add_argument("--step", type=str)
    p.add_argument("--iges", type=str)
    p.add_argument("--plot3d", type=str)
    p.add_argument("--obj", type=str)
    p.add_argument("--all", action="store_true")
    p.add_argument("--sweep-mach", type=str)
    ns = p.parse_args(argv)

    if ns.sweep_mach:
        d0 = design_from_args(ns)
        for tok in ns.sweep_mach.split(","):
            d = Design(**asdict(d0))
            d.mach = float(tok)
            d.name = f"{d0.name}_M{tok.strip()}"
            export_all(d, d.name, ns.out)
        return 0

    d = design_from_args(ns)
    mesh, grids, q = build(d)
    sc = unit_scale(d.unit)
    scale_mesh(mesh, sc)
    scale_grids(grids, sc)
    wrote = False
    if ns.stl:
        write_stl_binary(ns.stl, mesh, d.name)
        wrote = True
    if ns.step:
        write_step(ns.step, mesh, d.name, d.unit)
        wrote = True
    if ns.iges:
        write_iges(ns.iges, grids, d.name)
        wrote = True
    if ns.plot3d:
        write_plot3d(ns.plot3d, grids)
        wrote = True
    if ns.obj:
        write_obj(ns.obj, mesh)
        wrote = True
    if ns.all or not wrote:
        export_all(d, d.name, ns.out)
    else:
        print(f"{d.name}: {q['triangles']} tris, watertight={q['watertight']}, open={q['open_edges']}")
    return 0 if q["watertight"] else 2


if __name__ == "__main__":
    sys.exit(main())
