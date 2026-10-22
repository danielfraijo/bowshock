import numpy as np
import pytest

from bowshock import GridInterpolator


def _lin_table():
    m = np.array([2.0, 4.0, 8.0])
    a = np.array([-5.0, 0.0, 5.0, 10.0])
    b = np.array([0.0, 3.0])
    M, A, B = np.meshgrid(m, a, b, indexing="ij")
    return (m, a, b), 0.5 + 0.1 * M - 0.03 * A + 0.2 * B + 0.01 * M * A  # multilinear → reproduced exactly


def test_multilinear_exact_and_gradient():
    axes, v = _lin_table()
    gi = GridInterpolator(axes, {"CL": v}, names=("mach", "alpha", "beta"))
    x = (np.array([3.1, 7.9]), np.array([-1.2, 6.3]), np.array([1.0, 2.5]))
    ref = 0.5 + 0.1 * x[0] - 0.03 * x[1] + 0.2 * x[2] + 0.01 * x[0] * x[1]
    vals, d = gi.evaluate(*x, grad=True)
    np.testing.assert_allclose(vals["CL"], ref, rtol=1e-12)
    np.testing.assert_allclose(d["CL"][0], 0.1 + 0.01 * x[1], rtol=1e-12)
    np.testing.assert_allclose(d["CL"][1], -0.03 + 0.01 * x[0], rtol=1e-12)
    np.testing.assert_allclose(d["CL"][2], 0.2, rtol=1e-12)
    assert gi(mach=4.0, alpha=0.0, beta=0.0)["CL"] == pytest.approx(0.5 + 0.4)


def test_extrapolation_modes():
    axes, v = _lin_table()
    clip = GridInterpolator(axes, {"CL": v}, extrapolate="clip")
    lin = GridInterpolator(axes, {"CL": v}, extrapolate="linear")
    edge = clip.evaluate(8.0, 10.0, 3.0)["CL"]
    assert clip.evaluate(20.0, 10.0, 3.0)["CL"] == pytest.approx(edge)
    _, d = clip.evaluate(20.0, 10.0, 3.0, grad=True)
    assert d["CL"][0] == 0.0
    assert lin.evaluate(9.0, 10.0, 3.0)["CL"] == pytest.approx(0.5 + 0.9 - 0.3 + 0.6 + 0.9)
    with pytest.raises(ValueError):
        GridInterpolator(axes, {"CL": v}, extrapolate="raise").evaluate(1.0, 0.0, 0.0)


def test_validation_errors():
    axes, v = _lin_table()
    with pytest.raises(ValueError):
        GridInterpolator(axes, {"CL": v[:, :2]})
    with pytest.raises(ValueError):
        GridInterpolator((np.array([2.0, 1.0]),), {"x": np.zeros(2)})
    single = GridInterpolator((np.array([5.0]), np.array([0.0, 1.0])), {"x": np.array([[1.0, 3.0]])})
    assert single.evaluate(7.0, 0.5)["x"] == pytest.approx(2.0)
