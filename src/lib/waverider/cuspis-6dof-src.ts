/** Headless 6DOF that consumes a Cuspis aero-database JSON. Kept as a string so the browser kit can ship it. */
export const CUSPIS_6DOF_PY = `#!/usr/bin/env python3
"""Cuspis headless trajectory.

Reads a cuspis-aerodb-1 JSON (the Iterate tab export) and integrates a
boost-glide, PID-on-alpha, point-plus-short-period model. It is the same
idea as the in-browser closed loop, not a bit-exact port: the browser also
rebuilds the mesh for each deflection, while this script only interpolates
the table you already exported.

    python cuspis_6dof.py aero.json --seconds 60 --out traj.csv

Angles in the file are degrees. CL, CD, Cm are interpolated in Mach, alpha,
and delta at beta = 0. Rate damping is the Cmq column when it is filled.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys


def bracket(axis, x):
    if x <= axis[0]:
        return 0, 0, 0.0
    last = len(axis) - 1
    if x >= axis[last]:
        return last, last, 0.0
    i = 0
    while i < last - 1 and axis[i + 1] < x:
        i += 1
    span = axis[i + 1] - axis[i] or 1.0
    return i, i + 1, (x - axis[i]) / span


def lerp(a, b, u):
    return a + (b - a) * u


class Table:
    def __init__(self, doc):
        ax = doc["axes"]
        self.mach = list(ax["mach"])
        self.alpha = list(ax["alpha_deg"])
        self.delta = list(ax["delta_deg"])
        self.beta = list(ax["beta_deg"])
        self.meta = doc["meta"]
        nM, nA, nB, nD = len(self.mach), len(self.alpha), len(self.beta), len(self.delta)
        self.cl = [0.0] * (nM * nA * nB * nD)
        self.cd = [0.0] * (nM * nA * nB * nD)
        self.cm = [0.0] * (nM * nA * nB * nD)
        self.cmq = {}
        for p in doc["points"]:
            im = self.mach.index(p["mach"])
            ia = self.alpha.index(p["alpha_deg"])
            ib = self.beta.index(p["beta_deg"])
            id_ = self.delta.index(p["delta_deg"])
            i = ((im * nA + ia) * nB + ib) * nD + id_
            self.cl[i] = p["cl"]
            self.cd[i] = p["cd"]
            self.cm[i] = p["cm"]
            if p.get("cmq"):
                self.cmq[p["mach"]] = p["cmq"]
        self.nA, self.nB, self.nD = nA, nB, nD
        self.ib0 = self.beta.index(min(self.beta, key=lambda b: abs(b)))

    def at(self, mach, alpha, delta):
        im0, im1, um = bracket(self.mach, mach)
        ia0, ia1, ua = bracket(self.alpha, alpha)
        id0, id1, ud = bracket(self.delta, delta)

        def grab(arr, im, ia, id_):
            i = ((im * self.nA + ia) * self.nB + self.ib0) * self.nD + id_
            return arr[i]

        def row(arr, im, ia):
            return lerp(grab(arr, im, ia, id0), grab(arr, im, ia, id1), ud)

        def face(arr, im):
            return lerp(row(arr, im, ia0), row(arr, im, ia1), ua)

        def field(arr):
            return lerp(face(arr, im0), face(arr, im1), um)

        return field(self.cl), max(1e-4, field(self.cd)), field(self.cm)


def isa_rho(h_m):
    # Exponential screen, scale height 7200 m. The browser uses US76.
    return 1.225 * math.exp(-max(h_m, 0.0) / 7200.0)


def integrate(doc, seconds, dt, kp, ki, kd, tau, dmax):
    tab = Table(doc)
    meta = tab.meta
    S = float(meta["sRef_m2"])
    Lref = float(meta["lRef_m"])
    mass = 500.0
    Iyy = max(1.0, mass * Lref * Lref / 12.0)
    g0 = 9.80665
    Re = 6371000.0
    M = float(meta["designMach"])
    h = float(meta["altitude_km"]) * 1000.0
    a = 300.0
    V = M * a
    alpha = 4.0
    q = 0.0
    gamma = -1.0 * math.pi / 180.0
    theta = alpha * math.pi / 180.0 + gamma
    x = 0.0
    delta = 0.0
    integ = 0.0
    eprev = 0.0
    t = 0.0
    rows = []
    n = int(seconds / dt)
    for k in range(n + 1):
        rho = isa_rho(h)
        qdyn = 0.5 * rho * V * V
        # alpha of best L/D at this Mach, delta 0
        best, best_ld = tab.alpha[0], -1e9
        for al in tab.alpha:
            cl, cd, _cm = tab.at(V / a, al, 0.0)
            ld = cl / cd
            if ld > best_ld:
                best, best_ld = al, ld
        err = best - alpha
        integ = max(-30.0, min(30.0, integ + err * dt))
        dedt = (err - eprev) / dt
        eprev = err
        cmd = max(-dmax, min(dmax, kp * err + ki * integ + kd * dedt))
        ddelta = (cmd - delta) / max(tau, 1e-3) * dt
        ddelta = max(-40.0 * dt, min(40.0 * dt, ddelta))
        delta = max(-dmax, min(dmax, delta + ddelta))
        cl, cd, cm = tab.at(V / a, alpha, delta)
        mach_key = min(tab.mach, key=lambda m: abs(m - V / a))
        cmq = tab.cmq.get(mach_key, -0.3)
        qhat = q * Lref / (2.0 * max(V, 1.0))
        cm = cm + cmq * qhat
        Lift = qdyn * S * cl
        Drag = qdyn * S * cd
        g = g0 * (Re / (Re + h)) ** 2
        if k % max(1, int(0.25 / dt)) == 0:
            rows.append({
                "t_s": t, "alpha_deg": alpha, "delta_deg": delta, "alpha_cmd_deg": best,
                "V_m_s": V, "h_km": h / 1000.0, "x_km": x / 1000.0,
                "q_kPa": qdyn / 1000.0, "gamma_deg": gamma * 180.0 / math.pi,
            })
        a_dot = q - Lift / (mass * max(V, 1.0))
        q_dot = (qdyn * S * Lref * cm) / Iyy
        v_dot = -Drag / mass - g * math.sin(gamma)
        g_dot = Lift / (mass * max(V, 1.0)) - g * math.cos(gamma) / max(V, 1.0)
        alpha = max(-8.0, min(20.0, alpha + a_dot * dt * 180.0 / math.pi))
        q += q_dot * dt
        gamma += g_dot * dt
        V = max(0.0, V + v_dot * dt)
        h += V * math.sin(gamma) * dt
        x += V * math.cos(gamma) * dt
        t += dt
        if h < 0 or V < 200:
            break
    return rows


def main(argv):
    ap = argparse.ArgumentParser(description="Cuspis headless boost-glide on an aero JSON table")
    ap.add_argument("aero_json")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--dt", type=float, default=0.05)
    ap.add_argument("--kp", type=float, default=1.4)
    ap.add_argument("--ki", type=float, default=0.15)
    ap.add_argument("--kd", type=float, default=0.08)
    ap.add_argument("--tau", type=float, default=0.08)
    ap.add_argument("--delta-max", type=float, default=12.0)
    ap.add_argument("--out", default="-")
    args = ap.parse_args(argv)
    with open(args.aero_json, "r", encoding="utf-8") as f:
        doc = json.load(f)
    if doc.get("meta", {}).get("format") != "cuspis-aerodb-1" and doc.get("format") != "cuspis-aerodb-1":
        # meta.format is the canonical key
        fmt = doc.get("meta", {}).get("format")
        if fmt not in (None, "cuspis-aerodb-1"):
            print("warning: unexpected format %s" % fmt, file=sys.stderr)
    rows = integrate(doc, args.seconds, args.dt, args.kp, args.ki, args.kd, args.tau, args.delta_max)
    fields = ["t_s", "alpha_deg", "alpha_cmd_deg", "delta_deg", "V_m_s", "h_km", "x_km", "q_kPa", "gamma_deg"]
    out = sys.stdout if args.out == "-" else open(args.out, "w", encoding="utf-8", newline="")
    try:
        w = csv.DictWriter(out, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: "%.5f" % r[k] for k in fields})
    finally:
        if out is not sys.stdout:
            out.close()
    print("wrote %d samples" % len(rows), file=sys.stderr)


if __name__ == "__main__":
    main(sys.argv[1:])
`;
