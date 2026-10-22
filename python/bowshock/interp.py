"""Multilinear interpolation on a rectilinear grid with analytic gradients (numpy only)."""

from __future__ import annotations

from typing import Dict, Mapping, Sequence, Tuple

import numpy as np


class GridInterpolator:
    """Multilinear interpolant of one or more tables sharing rectilinear axes.

    ``extrapolate="clip"`` clamps queries to the axis range (gradient 0 outside along that axis);
    ``"linear"`` extends the edge cells; ``"raise"`` raises ValueError. Axes of length 1 are constant.
    """

    def __init__(self, axes: Sequence[np.ndarray], values: Mapping[str, np.ndarray], names: Sequence[str] = (), extrapolate: str = "clip"):
        self.axes = tuple(np.asarray(a, dtype=float) for a in axes)
        for a in self.axes:
            if a.ndim != 1 or len(a) == 0 or (len(a) > 1 and np.any(np.diff(a) <= 0)):
                raise ValueError("axes must be 1-D, non-empty, strictly increasing")
        shape = tuple(len(a) for a in self.axes)
        self.values: Dict[str, np.ndarray] = {}
        for k, v in values.items():
            v = np.asarray(v, dtype=float)
            if v.shape != shape:
                raise ValueError(f"{k}: shape {v.shape} != axes {shape}")
            self.values[k] = v
        if extrapolate not in ("clip", "linear", "raise"):
            raise ValueError("extrapolate must be clip, linear or raise")
        self.extrapolate = extrapolate
        self.names = tuple(names) or tuple(f"x{i}" for i in range(len(self.axes)))

    @property
    def coefs(self) -> Tuple[str, ...]:
        return tuple(self.values)

    def _locate(self, d: int, x: np.ndarray):
        a = self.axes[d]
        if len(a) == 1:
            z = np.zeros_like(x, dtype=int)
            return z, z, np.zeros_like(x), np.zeros_like(x)
        if self.extrapolate == "raise" and (np.any(x < a[0] - 1e-12) or np.any(x > a[-1] + 1e-12)):
            raise ValueError(f"{self.names[d]} outside [{a[0]}, {a[-1]}]")
        xc = np.clip(x, a[0], a[-1]) if self.extrapolate == "clip" else x
        i = np.clip(np.searchsorted(a, xc, side="right") - 1, 0, len(a) - 2)
        h = a[i + 1] - a[i]
        t = (xc - a[i]) / h
        dt = 1.0 / h
        if self.extrapolate == "clip":
            dt = np.where((x < a[0]) | (x > a[-1]), 0.0, dt)
        return i, i + 1, t, dt

    def evaluate(self, *x, grad: bool = False):
        """Values (and d/dx_k when ``grad``) for broadcastable query arrays, one per axis."""
        if len(x) != len(self.axes):
            raise ValueError(f"expected {len(self.axes)} coordinates {self.names}")
        xs = np.broadcast_arrays(*[np.asarray(v, dtype=float) for v in x])
        loc = [self._locate(d, xs[d]) for d in range(len(self.axes))]
        nd = len(self.axes)
        out = {k: np.zeros(xs[0].shape) for k in self.values}
        dout = {k: [np.zeros(xs[0].shape) for _ in range(nd)] for k in self.values} if grad else None
        for corner in range(1 << nd):
            idx = []
            w = np.ones(xs[0].shape)
            dw = [np.ones(xs[0].shape) for _ in range(nd)] if grad else None
            for d in range(nd):
                lo, hi, t, dt = loc[d]
                up = (corner >> d) & 1
                idx.append(hi if up else lo)
                wd = t if up else 1 - t
                w = w * wd
                if grad:
                    for e in range(nd):
                        dw[e] = dw[e] * ((dt if up else -dt) if e == d else wd)
            for k, v in self.values.items():
                vv = v[tuple(idx)]
                out[k] += w * vv
                if grad:
                    for e in range(nd):
                        dout[k][e] += dw[e] * vv
        return (out, dout) if grad else out

    def __call__(self, *x, **kw):
        if kw:
            x = tuple(kw[n] for n in self.names)
        res = self.evaluate(*x)
        return {k: (float(v) if np.ndim(v) == 0 else v) for k, v in res.items()}
