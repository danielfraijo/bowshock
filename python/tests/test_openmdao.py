import numpy as np
import pytest

from bowshock import aero_function, describe, openmdao_component


def test_aero_function_gradient_matches_finite_difference(db):
    f = aero_function(db, ("CL", "CD", "Cm"))
    x = [np.array([6.7]), np.array([1.3]), np.array([0.7]), np.array([-1.1])]
    vals, d = f(*x, grad=True)
    for k in range(4):
        h = 1e-4
        xp = [v.copy() for v in x]
        xm = [v.copy() for v in x]
        xp[k] += h
        xm[k] -= h
        fd = (f(*xp)["CL"] - f(*xm)["CL"]) / (2 * h)
        np.testing.assert_allclose(d["CL"][k], fd, rtol=1e-6, atol=1e-9)
    assert describe(db)["points"] == len(db.points)


def test_openmdao_component_partials(db):
    om = pytest.importorskip("openmdao.api")
    prob = om.Problem()
    prob.model.add_subsystem("aero", openmdao_component(db, ("CL", "CD"), vec_size=3), promotes=["*"])
    prob.setup(force_alloc_complex=False)
    prob.set_val("mach", [6.3, 7.1, 7.9])
    prob.set_val("alpha", [-1.0, 1.5, 3.2], units="deg")
    prob.set_val("beta", [0.4, -0.6, 1.1], units="deg")
    prob.set_val("delta", [-2.5, 0.5, 3.0], units="deg")
    prob.run_model()
    ref = aero_function(db, ("CL", "CD"))(prob.get_val("mach"), prob.get_val("alpha"), prob.get_val("beta"), prob.get_val("delta"))
    np.testing.assert_allclose(prob.get_val("CL"), ref["CL"])
    data = prob.check_partials(out_stream=None, method="fd", step=1e-6)
    for comp in data.values():
        for key, v in comp.items():
            # OpenMDAO >= 3.3x returns rel/abs error tuples under various keys; compare J_fwd with J_fd directly
            np.testing.assert_allclose(v["J_fwd"], v["J_fd"], rtol=1e-4, atol=1e-6, err_msg=str(key))
