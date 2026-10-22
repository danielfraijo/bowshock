import json
import math

import numpy as np
import pytest

from bowshock import AeroDbError, load_database, load_manifest, load_table, load_trajectory, read_csv, read_json, run_aerodb
from bowshock.io import COEFS, CSV_COLUMNS

from conftest import GEOM, SMALL, needs_node


def test_json_layout_matches_main(db, db_dir):
    assert db.meta["format"] == "cuspis-aerodb-1"
    assert db.meta["units"]["derivatives"] == "1/rad"
    assert db.meta["generatedAt"] == "1970-01-01T00:00:00.000Z"  # --reproducible
    assert list(db.points.columns) == list(CSV_COLUMNS)
    nm, na, nb, nd = db.shape
    assert len(db.points) == nm * na * nb * nd
    assert 8.0 in db.axes["mach"] and 0.0 in db.axes["beta_deg"] and 0.0 in db.axes["delta_deg"]
    # main's flat index: i = ((im*nAlpha + ia)*nBeta + ib)*nDelta + id (delta fastest)
    raw = json.loads((db_dir / "aerodb.json").read_text())
    im, ia, ib, idd = 1, 2, 0, 2
    i = ((im * na + ia) * nb + ib) * nd + idd
    p = raw["points"][i]
    assert (p["mach"], p["alpha_deg"], p["beta_deg"], p["delta_deg"]) == (db.axes["mach"][im], db.axes["alpha_deg"][ia], db.axes["beta_deg"][ib], db.axes["delta_deg"][idd])
    assert db.grid("CL")[im, ia, ib, idd] == pytest.approx(p["cl"], abs=0)
    assert len(db.dynamics) == nm


def test_csv_agrees_with_json(db, db_dir):
    c = read_csv(db_dir / "aerodb.csv")
    assert c.meta["format"] == "cuspis-aerodb-1"
    assert c.meta["sref_m2"] == pytest.approx(db.meta["sRef_m2"])
    assert c.meta["lref_m"] == pytest.approx(db.meta["lRef_m"]) and c.meta["bref_m"] == pytest.approx(db.meta["bRef_m"])
    assert c.meta["cg_x_m"] == pytest.approx(db.meta["cg_m"][0])
    assert c.meta["mass_kg"] == pytest.approx(db.meta["mass_kg"]) and c.meta["gamma"] == pytest.approx(db.meta["gamma"])
    assert c.meta["units"] == {"angles": "deg", "derivatives": "per radian"}
    assert len(c.meta["assumptions"]) == len(db.meta["assumptions"])
    for a in ("mach", "alpha_deg", "beta_deg", "delta_deg"):
        np.testing.assert_allclose(c.axes[a], db.axes[a])
    for col in COEFS:
        x, y = db.points[col].to_numpy(float), c.points[col].to_numpy(float)
        assert np.array_equal(np.isnan(x), np.isnan(y)), col
        ok = ~np.isnan(x)
        # toFixed(6) → 5e-7 absolute; toExponential(6) → 5e-7 relative
        np.testing.assert_allclose(y[ok], x[ok], rtol=1e-6, atol=6e-7, err_msg=col)
    assert len(c.dynamics) == len(db.dynamics)
    np.testing.assert_allclose(c.dynamics["Cmq"].to_numpy(), db.dynamics.sort_values("mach")["Cmq"].to_numpy(), atol=6e-7)


def test_static_derivatives_are_per_radian_differences(db):
    cl = db.grid("CL")
    cla = db.grid("CLa_per_rad")
    a = np.radians(db.axes["alpha_deg"])
    j = 1  # interior alpha: central difference
    np.testing.assert_allclose(cla[:, j], (cl[:, j + 1] - cl[:, j - 1]) / (a[j + 1] - a[j - 1]), rtol=1e-9, atol=1e-12)
    # axis ends: one-sided (aerodb.ts stencil() since 55d643f)
    np.testing.assert_allclose(cla[:, 0], (cl[:, 1] - cl[:, 0]) / (a[1] - a[0]), rtol=1e-9, atol=1e-12)
    np.testing.assert_allclose(cla[:, -1], (cl[:, -1] - cl[:, -2]) / (a[-1] - a[-2]), rtol=1e-9, atol=1e-12)
    cm, cmd = db.grid("Cm"), db.grid("Cmd_per_rad")
    d = np.radians(db.axes["delta_deg"])
    np.testing.assert_allclose(cmd[..., 1], (cm[..., 2] - cm[..., 0]) / (d[2] - d[0]), rtol=1e-9, atol=1e-12)


def test_interpolator_hits_breakpoints_and_refuses_rate_columns(db):
    f = db.interpolator(("CL", "CD", "Cm"))
    g = db.grid("CD")
    v = f(mach=db.axes["mach"][1], alpha_deg=db.axes["alpha_deg"][2], beta_deg=db.axes["beta_deg"][1], delta_deg=db.axes["delta_deg"][0])
    assert v["CD"] == pytest.approx(g[1, 2, 1, 0], rel=1e-12)
    with pytest.raises(ValueError, match="dynamic_interpolator"):
        db.interpolator(("Cmq",))
    d = db.dynamic_interpolator(("Cmq", "Clp", "Cnr"))
    m = float(db.axes["mach"].mean())
    assert math.isfinite(d(mach=m)["Cmq"])
    with pytest.raises(ValueError, match="euler"):
        db.interpolator(("euler_CL",))  # not computed → empty CSV cells / JSON null


def test_manifest_hashes_and_resolved_config(db_dir):
    man = load_manifest(db_dir)
    assert man["mode"] == "aerodb" and man["configHash"].startswith("c-")
    names = {f["name"] for f in man["files"]}
    assert {"aerodb.csv", "aerodb.json", "cuspis_aero_db.m", "config.resolved.json"} <= names
    res = json.loads((db_dir / "config.resolved.json").read_text())
    assert res["aerodb"]["method"] == "newtonian"


def test_load_database_accepts_files_and_rejects_wrong_format(db_dir, tmp_path):
    assert len(load_database(db_dir / "aerodb.csv").points) == len(read_json(db_dir / "aerodb.json").points)
    bad = tmp_path / "x.json"
    bad.write_text(json.dumps({"meta": {"format": "other"}}))
    with pytest.raises(ValueError, match="cuspis-aerodb-1"):
        load_database(bad)
    with pytest.raises(FileNotFoundError):
        load_database(tmp_path)


@needs_node
def test_bad_config_raises_with_error_paths(tmp_path, repo):
    cfg = {**SMALL, "aerodb": {**SMALL["aerodb"], "machMin": 0.5, "method": "cfd"}}
    with pytest.raises(AeroDbError) as ei:
        run_aerodb(cfg, tmp_path / "o", repo=repo, stream_progress=False)
    assert ei.value.returncode == 2
    assert "aerodb.machMin" in ei.value.stderr and "aerodb.method" in ei.value.stderr
    assert not (tmp_path / "o").exists()


@needs_node
def test_batch_modes_load(tmp_path, repo):
    fly = run_aerodb({**GEOM, "mode": "fly", "fly": {"durationS": 3}}, tmp_path / "fly", repo=repo, quiet=True, stream_progress=False)
    t = load_trajectory(fly)
    assert list(t.columns[:3]) == ["t_s", "alpha_deg", "alpha_cmd_deg"] and len(t) > 5
    assert any("boost-glide" in n for n in t.attrs["notes"])
    sw = run_aerodb({**GEOM, "mode": "sweep", "sweep": {"key": "alphaDeg", "values": [0, 2, 4]}}, tmp_path / "sw", repo=repo, quiet=True, stream_progress=False)
    s = load_table(sw / "sweep.csv")
    assert list(s["alphaDeg"]) == [0, 2, 4] and s["ld"].notna().all()
