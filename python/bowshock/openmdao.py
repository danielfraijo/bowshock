"""OpenMDAO-friendly wrappers. OpenMDAO itself is optional: imported only inside openmdao_component()."""

from __future__ import annotations

from typing import Callable, Dict, Sequence, Tuple

import numpy as np

INPUTS = ("mach", "alpha", "beta", "delta")  # [-], deg, deg, deg — the aerodb.ts axes (delta = elevonDeg)


def aero_function(db, coefs: Sequence[str] = ("CL", "CD", "Cm"), extrapolate: str = "clip") -> Callable:
    """Return ``f(mach, alpha_deg, beta_deg=0, delta_deg=0, grad=False)`` → dict of arrays.

    With ``grad=True`` also returns ``{coef: (d/dmach, d/dalpha_deg, d/dbeta_deg, d/ddelta_deg)}`` (per degree for
    the angles — the interpolant's own derivative, not the table's per-radian ``*_per_rad`` columns). Pure numpy.
    """
    interp = db.interpolator(coefs, extrapolate=extrapolate)

    def f(mach, alpha, beta=0.0, delta=0.0, grad: bool = False):
        if grad:
            vals, d = interp.evaluate(mach, alpha, beta, delta, grad=True)
            return vals, {k: tuple(v) for k, v in d.items()}
        return interp.evaluate(mach, alpha, beta, delta)

    f.interpolator = interp  # type: ignore[attr-defined]
    return f


def openmdao_component(db, coefs: Sequence[str] = ("CL", "CD", "Cm"), vec_size: int = 1, extrapolate: str = "clip"):
    """An ``openmdao.api.ExplicitComponent`` interpolating the database.

    Inputs ``mach`` [-], ``alpha``, ``beta``, ``delta`` [deg] (shape ``vec_size``); one output per coefficient.
    Partials are the analytic derivatives of the multilinear interpolant (diagonal in the vectorised index).
    Raises ImportError with an install hint when OpenMDAO is not available.
    """
    try:
        import openmdao.api as om
    except ImportError as e:  # pragma: no cover - exercised when openmdao is absent
        raise ImportError("openmdao_component needs OpenMDAO: pip install 'bowshock[openmdao]'") from e

    f = aero_function(db, coefs, extrapolate=extrapolate)
    names: Tuple[str, ...] = tuple(coefs)
    m0 = float(db.axes["mach"][0])

    class BowshockAeroComp(om.ExplicitComponent):
        def setup(self):
            n = vec_size
            self.add_input("mach", val=np.full(n, m0))
            for a in INPUTS[1:]:
                self.add_input(a, val=np.zeros(n), units="deg")
            for c in names:
                self.add_output(c, val=np.zeros(n))
            ar = np.arange(n)
            for c in names:
                for i in INPUTS:
                    self.declare_partials(c, i, rows=ar, cols=ar)

        def compute(self, inputs, outputs):
            vals = f(*(inputs[i] for i in INPUTS))
            for c in names:
                outputs[c] = vals[c]

        def compute_partials(self, inputs, partials):
            _, d = f(*(inputs[i] for i in INPUTS), grad=True)
            for c in names:
                for i, g in zip(INPUTS, d[c]):
                    partials[c, i] = g

    return BowshockAeroComp()


def describe(db) -> Dict[str, object]:
    """Small summary useful in notebooks."""
    p = db.points
    i = int(np.nanargmax(p["LD"].to_numpy(float)))
    return {"points": len(p), "method": db.meta.get("method"), "maxLD": float(p["LD"].iloc[i]), "at": {a: float(p[a].iloc[i]) for a in ("mach", "alpha_deg", "beta_deg", "delta_deg")}}
