#!/usr/bin/env python3
"""Cuspis — watertight waverider CAD for CFD.

Zero third-party dependencies. Python 3.9+.

Exports STL (ASCII + binary), STEP AP214 (NURBS surfaces + faceted B-rep),
IGES (NURBS 128), Plot3D `.x` (3-D formatted, nk=1), and OBJ. Coordinate frame:
X streamwise (most-forward USED point of the solid at origin), Y span, Z up. Lengths in metres internally;
--unit scales the files. Pointwise: import the binary STL or Plot3D `.x`
(3-D formatted, IBLANK off) — do not import STEP as XYZ points. Blunt LE
is on by default (--le-radius).

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
    le_radius: float = 0.02
    le_blunt: bool = True
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
        self._q = 1e8 / max(length, 1e-6)
        self._area_min = 1e-18 * max(length, 1e-6) ** 2
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
        if vlen(n) < self._area_min:
            self.skipped += 1
            return
        self.idx.append((a, b, c))
        self.surf.append(surface)

    def quad(self, a: int, b: int, c: int, d: int, surface: int = 0) -> None:
        if a == b == c == d:
            self.skipped += 1
            return
        pa, pb, pc, pd = self.pos[a], self.pos[b], self.pos[c], self.pos[d]
        ac = (pa[0] - pc[0]) ** 2 + (pa[1] - pc[1]) ** 2 + (pa[2] - pc[2]) ** 2
        bd = (pb[0] - pd[0]) ** 2 + (pb[1] - pd[1]) ** 2 + (pb[2] - pd[2]) ** 2
        if ac <= bd:
            self.tri(a, b, c, surface)
            self.tri(a, c, d, surface)
        else:
            self.tri(a, b, d, surface)
            self.tri(b, c, d, surface)

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
    if half:
        n = max(6, math.ceil(ny / 2))
        y_max = s * (1.0 - 0.4 / n)
        ys = [y_max * cosine_space(j, n) for j in range(n)]
        ys[0] = 0.0
        ys[-1] = y_max
        return ys
    n = max(7, ny)
    if n % 2 == 0:
        n += 1
    y_max = s * (1.0 - 0.4 / n)
    ys = [-y_max + 2.0 * y_max * cosine_space(j, n) for j in range(n)]
    ys[0] = -y_max
    ys[-1] = y_max
    ys[(n - 1) // 2] = 0.0
    return ys


def x_leading(y: float, L: float, s: float, planform: str, p: float, spatular: float, min_chord: float = 0.0) -> float:
    yn = clamp(abs(y) / max(s, 1e-12), 0.0, 1.0)
    if planform == "rect":
        x = 0.0
    elif planform == "spatular":
        nose = clamp(spatular, 0.04, 0.45)
        blunt = 1.0 - math.sqrt(max(0.0, 1.0 - yn * yn))
        x = L * lerp(nose * blunt, yn, 0.35)
    elif planform == "double":
        kink = 0.42
        if yn < kink:
            x = L * 0.1 * (yn / kink)
        else:
            x = L * (0.1 + 0.9 * ((yn - kink) / (1.0 - kink)) ** 1.05)
    else:
        pow_ = 1.0 if planform == "delta" else clamp(p, 0.6, 2.4)
        x = L * (yn ** pow_)
    cap = L
    return min(max(x, 0.0), cap)


def x_trailing(y: float, L: float, te_tan: float, xl: float) -> float:
    xt = L - abs(y) * te_tan
    return xt if xt > xl else xl


def super_z(y: float, s: float, h: float, n: float) -> float:
    yn = clamp(abs(y) / max(s, 1e-12), 0.0, 1.0)
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


def close_vehicle(
    mesh: Mesh, upper: Grid, lower: Grid, half: bool, flow_through: bool = False, leading: Grid | None = None
) -> None:
    stitch(mesh, upper, 0, False)
    stitch(mesh, lower, 1, True)
    if leading is not None and leading.ni >= 2 and leading.nj >= 2:
        stitch(mesh, leading, 3, False)
        if leading.ni >= 3:
            if half:
                mesh.fan([mesh.vert(*leading.at(k, 0)) for k in range(leading.ni)], 4)
                mesh.fan([mesh.vert(*leading.at(k, leading.nj - 1)) for k in range(leading.ni)], 3)
            else:
                mesh.fan([mesh.vert(*leading.at(k, 0)) for k in range(leading.ni)], 3)
                mesh.fan([mesh.vert(*leading.at(k, leading.nj - 1)) for k in range(leading.ni)], 3)
    if not flow_through:
        te_u = [mesh.vert(*upper.at(upper.ni - 1, j)) for j in range(upper.nj)]
        te_l = [mesh.vert(*lower.at(lower.ni - 1, j)) for j in range(lower.nj)]
        for j in range(upper.nj - 1):
            if te_u[j] == te_l[j] and te_u[j + 1] == te_l[j + 1]:
                continue
            mesh.quad(te_u[j], te_l[j], te_l[j + 1], te_u[j + 1], 2)
    js = [upper.nj - 1] if half else [0, upper.nj - 1]
    for j in js:
        for i in range(upper.ni - 1):
            u0 = mesh.vert(*upper.at(i, j))
            u1 = mesh.vert(*upper.at(i + 1, j))
            l0 = mesh.vert(*lower.at(i, j))
            l1 = mesh.vert(*lower.at(i + 1, j))
            if u0 == l0 and u1 == l1:
                continue
            if j == 0:
                mesh.quad(u0, l0, l1, u1, 3)
            else:
                mesh.quad(u0, u1, l1, l0, 3)
    if half:
        su = [mesh.vert(*upper.at(i, 0)) for i in range(upper.ni)]
        sl = [mesh.vert(*lower.at(i, 0)) for i in range(lower.ni)]
        n = min(len(su), len(sl))
        for i in range(n - 1):
            if su[i] == sl[i] and su[i + 1] == sl[i + 1]:
                continue
            mesh.quad(su[i], su[i + 1], sl[i + 1], sl[i], 4)
    if not flow_through and leading is None:
        le_u = [mesh.vert(*upper.at(0, j)) for j in range(upper.nj)]
        le_l = [mesh.vert(*lower.at(0, j)) for j in range(lower.nj)]
        for j in range(upper.nj - 1):
            if le_u[j] == le_l[j] and le_u[j + 1] == le_l[j + 1]:
                continue
            mesh.quad(le_u[j], le_u[j + 1], le_l[j + 1], le_l[j], 5)


def _set(g: Grid, i: int, j: int, p: Vec3) -> None:
    g.xyz[i * g.nj + j] = p


def _vsub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vadd(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vscale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _vdot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vlen(a: Vec3) -> float:
    return math.hypot(a[0], a[1], a[2])


def _vnorm(a: Vec3) -> Vec3:
    n = _vlen(a)
    return a if n < 1e-15 else (a[0] / n, a[1] / n, a[2] / n)


def _slerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    d = max(-1.0, min(1.0, _vdot(a, b)))
    om = math.acos(d)
    if om < 1e-8:
        return _vnorm((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t))
    s = math.sin(om)
    wa = math.sin((1.0 - t) * om) / s
    wb = math.sin(t * om) / s
    return _vnorm((a[0] * wa + b[0] * wb, a[1] * wa + b[1] * wb, a[2] * wa + b[2] * wb))


def _row(g: Grid, j: int) -> List[Vec3]:
    return [g.at(i, j) for i in range(g.ni)]


def _write_row(g: Grid, j: int, pts: Sequence[Vec3]) -> None:
    for i, p in enumerate(pts[: g.ni]):
        _set(g, i, j, p)


def _chord_len(pts: Sequence[Vec3]) -> float:
    return sum(_vlen(_vsub(pts[i], pts[i - 1])) for i in range(1, len(pts)))


def _tangent_at(pts: Sequence[Vec3]) -> Vec3:
    for i in range(len(pts) - 1):
        t = _vsub(pts[i + 1], pts[i])
        if _vlen(t) > 1e-12:
            return _vnorm(t)
    return (1.0, 0.0, 0.0)


def _point_at_length(pts: Sequence[Vec3], s_want: float) -> Tuple[Vec3, int]:
    acc = 0.0
    for i in range(len(pts) - 1):
        d = _vlen(_vsub(pts[i + 1], pts[i]))
        if acc + d >= s_want or i == len(pts) - 2:
            u = 0.0 if d <= 1e-15 else clamp((s_want - acc) / d, 0.0, 1.0)
            p = (
                pts[i][0] + (pts[i + 1][0] - pts[i][0]) * u,
                pts[i][1] + (pts[i + 1][1] - pts[i][1]) * u,
                pts[i][2] + (pts[i + 1][2] - pts[i][2]) * u,
            )
            return p, i + 1
        acc += d
    return pts[-1], len(pts) - 1


def _resample(pts: Sequence[Vec3], n: int, power: float = 1.45) -> List[Vec3]:
    if n <= 1:
        return [pts[0]]
    acc = [0.0]
    for i in range(1, len(pts)):
        acc.append(acc[-1] + _vlen(_vsub(pts[i], pts[i - 1])))
    total = acc[-1]
    if total < 1e-15:
        return [pts[0]] * n
    out: List[Vec3] = []
    for i in range(n):
        u = i / (n - 1)
        s = (u ** power) * total
        k = 0
        while k < len(acc) - 2 and acc[k + 1] < s:
            k += 1
        span = acc[k + 1] - acc[k]
        t = 0.0 if span <= 1e-15 else (s - acc[k]) / span
        t = clamp(t, 0.0, 1.0)
        out.append(
            (
                pts[k][0] + (pts[k + 1][0] - pts[k][0]) * t,
                pts[k][1] + (pts[k + 1][1] - pts[k][1]) * t,
                pts[k][2] + (pts[k + 1][2] - pts[k][2]) * t,
            )
        )
    out[0] = pts[0]
    out[-1] = pts[-1]
    return out


def _planar(p: Vec3, y: float) -> Vec3:
    return (p[0], y, p[2])


def _xz_norm(p: Vec3) -> Vec3:
    n = math.hypot(p[0], p[2])
    return (1.0, 0.0, 0.0) if n < 1e-15 else (p[0] / n, 0.0, p[2] / n)


def apply_leading_fillet(upper: Grid, lower: Grid, radius: float, length: float) -> Grid | None:
    """Circular LE fillet in each constant-y station (G1). Never chops the planform."""
    if radius <= 1e-9:
        return None
    L = max(length, 1e-6)
    nj = min(upper.nj, lower.nj)
    n_arc = 13
    alpha_min = math.radians(3.5)
    stations = []
    for j in range(nj):
        U, Lo = _row(upper, j), _row(lower, j)
        y = U[0][1]
        p = _planar(
            ((U[0][0] + Lo[0][0]) * 0.5, y, (U[0][2] + Lo[0][2]) * 0.5),
            y,
        )
        opening = _vlen(_vsub(U[0], Lo[0]))
        t_u, t_l = _xz_norm(_tangent_at(U)), _xz_norm(_tangent_at(Lo))
        du = max(-0.999, min(0.999, _vdot(t_u, t_l)))
        alpha = math.acos(du)
        if alpha < alpha_min:
            bis = _xz_norm(_vadd(t_u, t_l)) if _vlen(_vadd(t_u, t_l)) > 0.2 else (1.0, 0.0, 0.0)
            thick = _xz_norm((-bis[2], 0.0, bis[0]))
            if _vlen(thick) < 0.2:
                thick = (0.0, 0.0, 1.0)
            h = alpha_min / 2.0
            t_u = _xz_norm(_vadd(_vscale(bis, math.cos(h)), _vscale(thick, math.sin(h))))
            t_l = _xz_norm(_vadd(_vscale(bis, math.cos(h)), _vscale(thick, -math.sin(h))))
            alpha = alpha_min
        half = 0.5 * alpha
        chord = max(_chord_len(U), _chord_len(Lo), 1e-9)
        r = min(radius, 0.08 * L, 0.28 * chord * math.tan(half))
        t_probe = min(0.16 * chord, max(4.0 * radius, 1e-6))
        tu_p, _ = _point_at_length(U, t_probe)
        tl_p, _ = _point_at_length(Lo, t_probe)
        t_loc = _vlen(_vsub(tu_p, tl_p))
        r = min(r, 0.42 * t_loc, 0.12 * chord)
        if r < 2e-5 * L and chord > 4e-4 * L:
            r = min(2e-5 * L, 0.18 * chord)
        ok = False
        arc = [p] * n_arc
        tu = tl = p
        u_row = list(U)
        l_row = list(Lo)
        if opening > 1.6 * max(radius, 1e-9):
            stations.append((False, arc, tu, tl, u_row, l_row, y))
            continue
        if r > 1e-12:
            s_len = r / math.tan(half)
            s_cut = min(s_len, 2.4 * r, 0.10 * chord)
            r = s_cut * math.tan(half)
            d = r / math.sin(half)
            bis = _xz_norm(_vadd(t_u, t_l))
            if _vlen(bis) >= 0.2:
                c = _planar(_vadd(p, _vscale(bis, d)), y)
                tu, uk = _point_at_length(U, s_cut)
                tl, lk = _point_at_length(Lo, s_cut)
                tu, tl = _planar(tu, y), _planar(tl, y)
                if _vlen(_vsub(tu, tl)) < 4e-4 * L:
                    tu = _planar(_vadd(p, _vscale(t_u, s_cut)), y)
                    tl = _planar(_vadd(p, _vscale(t_l, s_cut)), y)
                f = _planar(_vsub(c, _vscale(bis, r)), y)
                a_tl, a_tu, a_f = _xz_norm(_vsub(tl, c)), _xz_norm(_vsub(tu, c)), _xz_norm(_vsub(f, c))
                if _vlen(a_tl) >= 0.5 and _vlen(a_tu) >= 0.5:
                    arc = []
                    for k in range(n_arc):
                        t = k / (n_arc - 1)
                        direction = _slerp(a_tl, a_f, t * 2) if t <= 0.5 else _slerp(a_f, a_tu, t * 2 - 1)
                        arc.append(_planar(_vadd(c, _vscale(_xz_norm(direction), r)), y))
                    arc[0], arc[-1] = tl, tu
                    u_row = _resample([tu] + list(U[max(uk, 1) :]), upper.ni)
                    l_row = _resample([tl] + list(Lo[max(lk, 1) :]), lower.ni)
                    u_row[0], l_row[0] = tu, tl
                    ok = True
        stations.append((ok, arc, tu, tl, u_row, l_row, y))
    if sum(1 for st in stations if st[0]) < 3:
        return None
    for j, st in enumerate(stations):
        if st[0]:
            continue
        lo = next((i for i in range(j - 1, -1, -1) if stations[i][0]), None)
        hi = next((i for i in range(j + 1, nj) if stations[i][0]), None)
        y = st[6]
        keep_u, keep_l = st[4], st[5]
        if lo is not None and hi is not None:
            t = (j - lo) / max(hi - lo, 1)

            def mix(a: Vec3, b: Vec3) -> Vec3:
                return _planar((a[0] + (b[0] - a[0]) * t, y, a[2] + (b[2] - a[2]) * t), y)

            arc = [mix(stations[lo][1][k], stations[hi][1][k]) for k in range(n_arc)]
            tu = mix(stations[lo][2], stations[hi][2])
            tl = mix(stations[lo][3], stations[hi][3])
            if keep_u:
                keep_u = list(keep_u)
                keep_u[0] = tu
            if keep_l:
                keep_l = list(keep_l)
                keep_l[0] = tl
            stations[j] = (True, arc, tu, tl, keep_u, keep_l, y)
        elif lo is not None or hi is not None:
            src = stations[lo] if lo is not None else stations[hi]
            tu = _planar(src[2], y)
            tl = _planar(src[3], y)
            if keep_u:
                keep_u = list(keep_u)
                keep_u[0] = tu
            if keep_l:
                keep_l = list(keep_l)
                keep_l[0] = tl
            stations[j] = (
                True,
                [_planar(p, y) for p in src[1]],
                tu,
                tl,
                keep_u,
                keep_l,
                y,
            )
    lead = make_grid("leading", n_arc, nj, lambda i, j: stations[j][1][i])
    for j, (_ok, arc, tu, tl, u_row, l_row, yj) in enumerate(stations):
        tu, tl = _planar(tu, yj), _planar(tl, yj)
        arc = [_planar(p, yj) for p in arc]
        if u_row:
            u_row = list(u_row)
            u_row[0] = tu
            _write_row(upper, j, u_row)
        if l_row:
            l_row = list(l_row)
            l_row[0] = tl
            _write_row(lower, j, l_row)
        _set(upper, 0, j, tu)
        _set(lower, 0, j, tl)
        for k, p in enumerate(arc):
            _set(lead, k, j, p)
        _set(lead, 0, j, tl)
        _set(lead, n_arc - 1, j, tu)
    return lead


def fillet_tip_cap(lead: Grid, j: int, name: str) -> Grid | None:
    n = lead.ni
    if n < 3 or j < 0 or j >= lead.nj:
        return None
    tl, tu = lead.at(0, j), lead.at(n - 1, j)
    bow = 0.0
    for k in range(n):
        t = 0.0 if n <= 1 else k / (n - 1)
        chord = (tl[0] + (tu[0] - tl[0]) * t, tl[1] + (tu[1] - tl[1]) * t, tl[2] + (tu[2] - tl[2]) * t)
        bow = max(bow, _vlen(_vsub(lead.at(k, j), chord)))
    if bow < 1e-10:
        return None
    return make_grid(
        name,
        2,
        n,
        lambda i, k: lead.at(k, j)
        if i == 0
        else (
            tl[0] + (tu[0] - tl[0]) * (k / (n - 1)),
            tl[1] + (tu[1] - tl[1]) * (k / (n - 1)),
            tl[2] + (tu[2] - tl[2]) * (k / (n - 1)),
        ),
    )


def sharpen_tips(upper: Grid, lower: Grid, leading: Grid | None, half: bool) -> None:
    ni, nj = min(upper.ni, lower.ni), min(upper.nj, lower.nj)
    if nj < 3:
        return
    js = [nj - 1] if half else [0, nj - 1]
    for j in js:
        for i in range(ni):
            u, lo = upper.at(i, j), lower.at(i, j)
            m = ((u[0] + lo[0]) * 0.5, (u[1] + lo[1]) * 0.5, (u[2] + lo[2]) * 0.5)
            _set(upper, i, j, m)
            _set(lower, i, j, m)
        if leading is not None:
            p = upper.at(0, j)
            for k in range(leading.ni):
                _set(leading, k, j, p)


def zipper_sharp_edges(upper: Grid, lower: Grid, length: float) -> None:
    L = max(length, 1e-6)
    eps = 1e-7 * L
    ni = min(upper.ni, lower.ni)
    nj = min(upper.nj, lower.nj)

    def dist(a: Vec3, b: Vec3) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])

    def mid(a: Vec3, b: Vec3) -> Vec3:
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)

    for j in range(nj):
        u_le, l_le = upper.at(0, j), lower.at(0, j)
        if dist(u_le, l_le) <= eps:
            m = mid(u_le, l_le)
            _set(upper, 0, j, m)
            _set(lower, 0, j, m)
        u_te = upper.at(ni - 1, j)
        if dist(upper.at(0, j), u_te) <= 10 * eps:
            p = upper.at(0, j)
            for i in range(ni):
                _set(upper, i, j, p)
                _set(lower, i, j, p)
            continue
        for i in range(ni):
            u, l = upper.at(i, j), lower.at(i, j)
            if dist(u, l) <= eps:
                m = mid(u, l)
                _set(upper, i, j, m)
                _set(lower, i, j, m)


def apply_fins(lid: Grid, d: Design, z_sign: float) -> None:
    h = (d.fin_height or 0.0) * max(d.height, 0.05)
    if h < 1e-4:
        return
    L = d.length
    y_off = d.span * 0.36
    y_fins = [y_off] if d.half_model else [-y_off, y_off]
    x0 = L * 0.68
    half_t = max(d.span * 0.018, 0.01)
    xyz = []
    for i in range(lid.ni):
        for j in range(lid.nj):
            p = lid.at(i, j)
            x, y, z = p
            if x >= x0:
                dy = min(abs(y - yc) for yc in y_fins)
                if dy < half_t:
                    along = (x - x0) / max(L - x0, 1e-9)
                    across = 1.0 - (dy / half_t) ** 2
                    z += z_sign * h * along * across
            xyz.append((x, y, z))
    lid.xyz = xyz


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
        return x_leading(y, L, s, planform, power, spat, 0.0)

    def xt(y: float, xl: float) -> float:
        return x_trailing(y, L, te_tan, xl)

    def sample(i, j, is_lower: bool):
        y = ys[j]
        xl = xle(y)
        xe = xt(y, xl)
        chord = xe - xl
        z_off = abs(y) * dih
        if chord <= 1e-12 * L:
            return (xl, y, z_off)
        frac = cluster(i, nx)
        x = lerp(xl, xe, frac)
        z_lid = 4.0 * cam * frac * (1.0 - frac)
        z = z_base(y) * (frac ** z_pow) if is_lower else 0.0
        return (x, y, z + z_lid + z_off)

    return make_grid("upper", nx, nj, lambda i, j: sample(i, j, False)), make_grid(
        "lower", nx, nj, lambda i, j: sample(i, j, True)
    )


def build_caret(d: Design) -> Tuple[Grid, Grid]:
    theta, _b, _ = resolve_shock(d)
    h = d.length * math.tan(theta)
    s = d.span / 2.0
    return _loft(d, lambda y: -h * (1.0 - clamp(abs(y) / max(s, 1e-12), 0.0, 1.0)), "delta", 1.0, 0.0)


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
        t = clamp((ay - yw) / max(s - yw, 1e-12), 0.0, 1.0)
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
        yn = clamp(abs(y) / max(s, 1e-12), 0.0, 1.0)
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
        yn = clamp(abs(y) / max(s, 1e-12), 0.0, 1.0)
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


def build_duct(d: Design, scram: bool) -> Tuple[Grid, Grid]:
    """2-D ramjet/scramjet OML: rectangular capture at x=0, nozzle at x=L.

    Floor ramps rise (internal compression) → isolator → SERN drop.
    Cowl is constant capture height then expands. Both surfaces span 0→L so
    the inlet and nozzle are real rectangular faces, not collapsed lips.
    """
    L, s = d.length, d.span / 2.0
    h_in = max(0.05, d.inlet_height)
    th = d.ramp_deg * DEG
    n_r = int(clamp(round(d.n_ramps or 2), 1, 3))
    x_ramp = L * clamp(d.cowl_frac, 0.22, 0.58)
    x_comb = min(L * (0.82 if scram else 0.86), x_ramp + L * clamp(d.combustor_frac, 0.12, 0.38))
    er = max(1.4, d.nozzle_er)
    throat_t = clamp(0.52 * h_in if scram else 0.36 * h_in, 0.24 * h_in, 0.72 * h_in)
    rise = clamp(h_in - throat_t, 0.08 * h_in, 0.78 * h_in)
    throat = h_in - rise
    h_exit = throat * math.sqrt(er) if scram else throat * er
    expand = max(0.0, h_exit - throat)
    cowl_up = 0.35 * expand
    floor_drop = 0.65 * expand
    dx = x_ramp / n_r
    weights = [math.tan(th * (k + 1) / n_r) for k in range(n_r)]
    wsum = sum(weights) or 1.0

    def z_floor(x: float) -> float:
        if x <= 0:
            return 0.0
        if x <= x_ramp:
            z = 0.0
            x0 = 0.0
            for k in range(n_r):
                x1 = dx * (k + 1)
                dz = rise * (weights[k] / wsum)
                if x <= x1 + 1e-12:
                    return z + (x - x0) / max(x1 - x0, 1e-12) * dz
                z += dz
                x0 = x1
            return rise
        if x <= x_comb:
            return rise
        u = (x - x_comb) / max(L - x_comb, 1e-9)
        return rise - floor_drop * u * u

    def z_cowl(x: float) -> float:
        if x <= x_comb:
            return h_in
        u = (x - x_comb) / max(L - x_comb, 1e-9)
        return h_in + cowl_up * u

    ys = half_span_list(d.ny, s, d.half_model)
    nx, nj = max(16, d.nx), len(ys)
    n1 = max(6, round(nx * (x_ramp / L)))
    n2 = max(4, round(nx * ((x_comb - x_ramp) / L)))
    n3 = max(6, nx - n1 - n2)
    xs = [(x_ramp * i) / max(n1 - 1, 1) for i in range(n1)]
    xs += [x_ramp + ((x_comb - x_ramp) * i) / n2 for i in range(1, n2 + 1)]
    xs += [x_comb + ((L - x_comb) * i) / n3 for i in range(1, n3 + 1)]
    xs[0] = 0.0
    xs[-1] = L
    lower = make_grid("lower", len(xs), nj, lambda i, j: (xs[i], ys[j], z_floor(xs[i])))
    upper = make_grid("cowl", len(xs), nj, lambda i, j: (xs[i], ys[j], z_cowl(xs[i])))
    return upper, lower


def build_ramjet(d: Design) -> Tuple[Grid, Grid]:
    return build_duct(d, False)


def build_scramjet(d: Design) -> Tuple[Grid, Grid]:
    return build_duct(d, True)


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
        is_duct = d.family in ("ramjet", "scramjet")
        leading = None
        r_le = d.le_radius if getattr(d, "le_blunt", True) else 0.0
        tips: List[Grid] = []
        if (not is_duct) and r_le > 1e-9:
            leading = apply_leading_fillet(upper, lower, r_le, d.length)
        if not is_duct:
            if d.family != "liftbody":
                sharpen_tips(upper, lower, leading, d.half_model)
            if leading is not None:
                for j, name in ((0, "tip_l"), (leading.nj - 1, "tip_r")):
                    cap = fillet_tip_cap(leading, j, name)
                    if cap is not None:
                        tips.append(cap)
        if (not is_duct) and abs(d.elevon_deg) > 1e-3:
            hinge = 0.82 * d.length
            k = math.tan(d.elevon_deg * DEG)
            for g in (upper, lower):
                xyz = []
                for p in g.xyz:
                    z = p[2] - (p[0] - hinge) * k if p[0] > hinge else p[2]
                    xyz.append((p[0], p[1], z))
                g.xyz = xyz
        if not is_duct:
            zipper_sharp_edges(upper, lower, d.length)
            apply_fins(upper, d, -1.0 if d.lid == "bottom" else 1.0)
            zipper_sharp_edges(upper, lower, d.length)
        grids = [upper, lower]
        if leading is not None:
            grids.append(leading)
        grids.extend(tips)
        if is_duct:
            nj = min(upper.nj, lower.nj)
            inlet = make_grid("inlet", 2, nj, lambda i, j: lower.at(0, j) if i == 0 else upper.at(0, j))
            nozzle = make_grid(
                "nozzle",
                2,
                nj,
                lambda i, j: lower.at(lower.ni - 1, j) if i == 0 else upper.at(upper.ni - 1, j),
            )
            grids.extend([inlet, nozzle])
        project_half(grids, d.half_model)
        lock_frame(None, grids, d.half_model)
        mesh = Mesh(d.length)
        close_vehicle(mesh, upper, lower, d.half_model, is_duct and d.flow_through, leading)
    compact_mesh(mesh)
    lock_frame(mesh, grids, d.half_model)
    return mesh, grids, analyze(mesh)


def compact_mesh(mesh: Mesh) -> None:
    """Drop unused builder verts so STL/OBJ origin is the solid, not the loft."""
    if not mesh.idx:
        return
    used = [False] * len(mesh.pos)
    for a, b, c in mesh.idx:
        used[a] = used[b] = used[c] = True
    remap = [-1] * len(mesh.pos)
    new_pos: List[Vec3] = []
    for i, p in enumerate(mesh.pos):
        if not used[i]:
            continue
        remap[i] = len(new_pos)
        new_pos.append(p)
    if len(new_pos) == len(mesh.pos):
        return
    mesh.pos = new_pos
    mesh.idx = [(remap[a], remap[b], remap[c]) for a, b, c in mesh.idx]
    mesh._key = {}


def project_half(grids: List[Grid], half: bool) -> None:
    if not half:
        for g in grids:
            for j in range(g.nj):
                y_mean = sum(g.at(i, j)[1] for i in range(g.ni)) / max(g.ni, 1)
                if abs(y_mean) > 1e-9:
                    continue
                for i in range(g.ni):
                    p = g.at(i, j)
                    _set(g, i, j, (p[0], 0.0, p[2]))
        return
    for g in grids:
        for i in range(g.ni):
            for j in range(g.nj):
                p = g.at(i, j)
                y = 0.0 if j == 0 or abs(p[1]) < 1e-6 else max(p[1], 0.0)
                _set(g, i, j, (p[0], y, p[2]))


def lock_frame(mesh: Mesh | None, grids: List[Grid], half: bool) -> None:
    """Most-forward USED solid point at (0,0,0). Half-model never shifts Y."""
    pts: List[Vec3] = []
    if mesh is not None and mesh.idx:
        for a, b, c in mesh.idx:
            pts.append(mesh.pos[a])
            pts.append(mesh.pos[b])
            pts.append(mesh.pos[c])
    else:
        for g in grids:
            pts.extend(g.xyz)
    finite = [p for p in pts if math.isfinite(p[0]) and math.isfinite(p[1]) and math.isfinite(p[2])]
    if not finite:
        return
    xmin = min(p[0] for p in finite)
    band = 1e-3
    y_n = 0.0
    z_n = 0.0
    best_ay = float("inf")
    for x, y, z in finite:
        if x > xmin + band:
            continue
        ay = abs(y)
        if ay < best_ay:
            best_ay, y_n, z_n = ay, y, z
    dy = 0.0 if half else y_n

    def sh(p: Vec3) -> Vec3:
        return (p[0] - xmin, p[1] - dy, p[2] - z_n)

    if mesh is not None:
        mesh.pos = [sh(p) for p in mesh.pos]
    for g in grids:
        g.xyz = [sh(p) for p in g.xyz]
    project_half(grids, half)
    if half and mesh is not None:
        mesh.pos = [(p[0], 0.0 if abs(p[1]) < 1e-6 else p[1], p[2]) for p in mesh.pos]


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
    header = f"Cuspis {name} watertight STL".encode("ascii", "replace")[:80]
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


def _spline_knots(n_poles: int, degree: int) -> Tuple[List[float], List[int]]:
    p = min(degree, max(1, n_poles - 1))
    knots = [0.0]
    mult = [p + 1]
    internal = n_poles - p - 1
    for i in range(1, internal + 1):
        knots.append(i / (internal + 1))
        mult.append(1)
    knots.append(1.0)
    mult.append(p + 1)
    return knots, mult


def _downsample_grid(g: Grid, cap_u: int = 16, cap_v: int = 16) -> List[List[Vec3]]:
    keep_u = g.ni <= 4 or g.name in ("leading", "cowl_lip", "tip_l", "tip_r") or g.name.startswith("side")
    nu = g.ni if keep_u else min(g.ni, cap_u)
    nv = min(g.nj, cap_v)
    poles: List[List[Vec3]] = []
    for iu in range(nu):
        i = g.ni - 1 if iu == nu - 1 else round(iu * (g.ni - 1) / max(nu - 1, 1))
        row = []
        for jv in range(nv):
            j = g.nj - 1 if jv == nv - 1 else round(jv * (g.nj - 1) / max(nv - 1, 1))
            row.append(g.at(i, j))
        poles.append(row)
    return poles


def write_nurbs_step(path: str, grids: List[Grid], name: str, unit: str) -> None:
    """Degree-3 NURBS STEP — surfaces, not a CARTESIAN_POINT cloud."""
    usable = [g for g in grids if g.ni >= 2 and g.nj >= 2]
    lines: List[str] = []
    n = [0]

    def add(entity: str) -> int:
        n[0] += 1
        lines.append(f"#{n[0]} = {entity};")
        return n[0]

    hdr = (
        "ISO-10303-21;\nHEADER;\n"
        "FILE_DESCRIPTION(('Cuspis NURBS waverider blunt LE'),'2;1');\n"
        f"FILE_NAME('{name}.step','2026-01-01T00:00:00',('Cuspis'),('Cuspis Roma'),"
        "'Cuspis CAD','Cuspis','');\n"
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

    def emit_surface(poles: List[List[Vec3]], degree: int = 3) -> int:
        ni, nj = len(poles), len(poles[0])
        p1, p2 = min(degree, ni - 1), min(degree, nj - 1)
        ids = []
        for row in poles:
            ids.append([add(f"CARTESIAN_POINT('',({_s(p[0])},{_s(p[1])},{_s(p[2])}))") for p in row])
        ku, mu = _spline_knots(ni, p1)
        kv, mv = _spline_knots(nj, p2)
        grid = ",".join("(" + ",".join(f"#{i}" for i in row) + ")" for row in ids)
        return add(
            f"B_SPLINE_SURFACE_WITH_KNOTS('',{p1},{p2},({grid}),.UNSPECIFIED.,.F.,.F.,.F.,"
            f"({','.join(str(m) for m in mu)}),({','.join(str(m) for m in mv)}),"
            f"({','.join(_s(k) for k in ku)}),({','.join(_s(k) for k in kv)}),.UNSPECIFIED.)"
        )

    def emit_curve(poles: List[Vec3], degree: int = 3) -> int:
        p = min(degree, len(poles) - 1)
        ids = [add(f"CARTESIAN_POINT('',({_s(pt[0])},{_s(pt[1])},{_s(pt[2])}))") for pt in poles]
        kts, mul = _spline_knots(len(poles), p)
        return add(
            f"B_SPLINE_CURVE_WITH_KNOTS('',{p},({','.join(f'#{i}' for i in ids)}),.UNSPECIFIED.,.F.,.F.,"
            f"({','.join(str(m) for m in mul)}),({','.join(_s(k) for k in kts)}),.UNSPECIFIED.)"
        )

    def emit_face(poles: List[List[Vec3]], fname: str) -> int:
        ni, nj = len(poles), len(poles[0])
        surf = emit_surface(poles, 3)
        corners = [poles[0][0], poles[ni - 1][0], poles[ni - 1][nj - 1], poles[0][nj - 1]]
        vxs = []
        for c in corners:
            pt = add(f"CARTESIAN_POINT('',({_s(c[0])},{_s(c[1])},{_s(c[2])}))")
            vxs.append(add(f"VERTEX_POINT('',#{pt})"))
        c0 = emit_curve([row[0] for row in poles], 1)
        c1 = emit_curve(poles[ni - 1], 1)
        c2 = emit_curve(list(reversed([row[nj - 1] for row in poles])), 1)
        c3 = emit_curve(list(reversed(poles[0])), 1)
        e0 = add(f"EDGE_CURVE('',#{vxs[0]},#{vxs[1]},#{c0},.T.)")
        e1 = add(f"EDGE_CURVE('',#{vxs[1]},#{vxs[2]},#{c1},.T.)")
        e2 = add(f"EDGE_CURVE('',#{vxs[2]},#{vxs[3]},#{c2},.T.)")
        e3 = add(f"EDGE_CURVE('',#{vxs[3]},#{vxs[0]},#{c3},.T.)")
        o0 = add(f"ORIENTED_EDGE('',*,*,#{e0},.T.)")
        o1 = add(f"ORIENTED_EDGE('',*,*,#{e1},.T.)")
        o2 = add(f"ORIENTED_EDGE('',*,*,#{e2},.T.)")
        o3 = add(f"ORIENTED_EDGE('',*,*,#{e3},.T.)")
        loop = add(f"EDGE_LOOP('',(#{o0},#{o1},#{o2},#{o3}))")
        bound = add(f"FACE_OUTER_BOUND('',#{loop},.T.)")
        return add(f"ADVANCED_FACE('{fname}',(#{bound}),#{surf},.T.)")

    faces = [emit_face(_downsample_grid(g), g.name or "surface") for g in usable]
    upper = next((g for g in usable if g.name in ("upper", "cowl")), usable[0] if usable else None)
    lower = next((g for g in usable if g.name == "lower"), usable[-1] if usable else None)
    if upper and lower and upper is not lower:
        u_p, l_p = _downsample_grid(upper), _downsample_grid(lower)
        nj = min(len(u_p[0]), len(l_p[0]))
        mid = nj // 2
        te_u, te_l = u_p[-1][:nj], l_p[-1][:nj]
        le_u, le_l = u_p[0][:nj], l_p[0][:nj]
        te_d = math.hypot(te_u[mid][0] - te_l[mid][0], te_u[mid][1] - te_l[mid][1], te_u[mid][2] - te_l[mid][2])
        le_d = math.hypot(le_u[mid][0] - le_l[mid][0], le_u[mid][1] - le_l[mid][1], le_u[mid][2] - le_l[mid][2])
        if te_d > 1e-7:
            faces.append(emit_face([te_u, te_l], "nozzle_or_base"))
        has_lead = any(g.name in ("leading", "cowl_lip") for g in usable)
        if le_d > 1e-7 and not has_lead:
            faces.append(emit_face([le_u, le_l], "inlet"))
    if not faces:
        write_step(path, Mesh(1.0), name, unit)
        return
    shell = add("CLOSED_SHELL('',(" + ",".join(f"#{i}" for i in faces) + "))")
    model = add(f"MANIFOLD_SOLID_BREP('Cuspis',#{shell})")
    repr_ = add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{model}),#{ctx})")
    add(f"SHAPE_DEFINITION_REPRESENTATION(#{pds},#{repr_})")
    with open(path, "w", encoding="ascii", errors="replace") as f:
        f.write(hdr)
        f.write("\n".join(lines))
        f.write("\nENDSEC;\nEND-ISO-10303-21;\n")


def write_step(path: str, mesh: Mesh, name: str, unit: str) -> None:
    lines = []
    n = [0]

    def add(entity: str) -> int:
        n[0] += 1
        lines.append(f"#{n[0]} = {entity};")
        return n[0]

    hdr = (
        "ISO-10303-21;\nHEADER;\n"
        "FILE_DESCRIPTION(('Cuspis watertight waverider'),'2;1');\n"
        f"FILE_NAME('{name}.step','2026-01-01T00:00:00',('Cuspis'),('Cuspis Roma'),"
        "'Cuspis CAD','Cuspis','');\n"
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
        f.write("# Cuspis waverider\no waverider\n")
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

    lines = [pad("Cuspis IGES,".ljust(72) + "S" + "1".rjust(7))]
    gsec = [
        "1H,,1H;,8HWAVERIDE,7HIGES5.3,",
        "6HCuspis,6HCuspis,32,38,6,308,15,",
        "6HCuspis,1.,5,1HM,32768,0.,15H20260101.000000,",
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
    write_nurbs_step(base + ".step", grids, d.name, d.unit)
    write_nurbs_step(base + "_nurbs.step", grids, d.name, d.unit)
    write_step(base + "_faceted.step", mesh, d.name, d.unit)
    write_iges(base + ".igs", grids, d.name)
    write_plot3d(base + ".x", grids)
    write_obj(base + ".obj", mesh)
    with open(base + "_POINTWISE.txt", "w", encoding="utf-8") as f:
        f.write(
            "POINTWISE\n"
            "=========\n"
            "Do not import STEP as XYZ points (cyan cloud).\n"
            f"1. File > Import > STL     {prefix}.stl     (recommended)\n"
            f"2. File > Import > IGES    {prefix}.igs\n"
            f"3. File > Import > Plot3D  {prefix}.x   3-D formatted, IBLANK off\n"
            f"4. NURBS STEP              {prefix}.step / {prefix}_nurbs.step  (sewn CLOSED_SHELL)\n"
            "Frame: X stream, Y span, Z up. Nose / cowl lip at origin.\n"
            "Rounded LE is ON by default (--le-radius). Toggle off with --le-radius 0.\n"
        )
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
            "captureFrac": "capture_frac", "leRadius": "le_radius", "leBlunt": "le_blunt", "halfModel": "half_model",
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
    if d.family in ("ramjet", "scramjet"):
        d.flow_through = True
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
        write_nurbs_step(ns.step, grids, d.name, d.unit)
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
