import os

os.environ.setdefault("OPENMDAO_REPORTS", "0")  # no reports/ dirs from check_partials

import shutil
from pathlib import Path

import pytest

from bowshock import find_repo_root, load_database, run_aerodb

# Small grid in main's AeroDbSpec terms (aerodb.ts); design Mach 8 / design alpha are forced into the axes.
SMALL = {
    "name": "pytest-caret",
    "geometry": {"preset": "caret-m8", "params": {"nx": 14, "ny": 10}},
    "aerodb": {
        "machMin": 6, "machMax": 8, "machStep": 2,
        "alphaMin": -2, "alphaMax": 4, "alphaStep": 2,
        "betaMin": -2, "betaMax": 2, "betaStep": 2,
        "deltaMin": -4, "deltaMax": 4, "deltaStep": 4,
        "method": "newtonian", "dynamicDerivatives": True, "altKm": 30,
    },
}
GEOM = {"geometry": SMALL["geometry"]}

needs_node = pytest.mark.skipif(shutil.which("npm") is None, reason="npm/node not on PATH")


@pytest.fixture(scope="session")
def repo() -> Path:
    return find_repo_root()


@pytest.fixture(scope="session")
def db_dir(tmp_path_factory, repo):
    if shutil.which("npm") is None:
        pytest.skip("npm/node not on PATH")
    out = tmp_path_factory.mktemp("aerodb") / "out"
    return run_aerodb(SMALL, out, repo=repo, reproducible=True, quiet=True, stream_progress=False)


@pytest.fixture(scope="session")
def db(db_dir):
    return load_database(db_dir)
