"""Bowshock headless tools: run ``npm run aerodb``, load its outputs, interpolate (OpenMDAO optional)."""

from .cli import AeroDbError, find_repo_root, run_aerodb
from .interp import GridInterpolator
from .io import AeroDatabase, load_database, load_manifest, load_table, load_trajectory, read_csv, read_json, read_meta_csv
from .openmdao import aero_function, describe, openmdao_component

__all__ = [
    "AeroDbError",
    "AeroDatabase",
    "GridInterpolator",
    "aero_function",
    "describe",
    "find_repo_root",
    "load_database",
    "load_manifest",
    "load_table",
    "load_trajectory",
    "openmdao_component",
    "read_csv",
    "read_json",
    "read_meta_csv",
    "run_aerodb",
]
__version__ = "0.1.0"
