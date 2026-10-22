"""Loaders for the files written by ``npm run aerodb`` (main's aerodb.ts / guidance.ts / mdo.ts formats).

* ``aerodb.json``  — ``aeroDbToJson``: ``{meta, axes, points[], dynamics[], summary, slices}``, full precision. Preferred.
* ``aerodb.csv``   — ``aeroDbToCsv``: ``#`` header lines (meta) then one row per point; 6 significant decimals
  (``toFixed(6)``, exponent form when |v| >= 1e4 or < 1e-4); empty cell = not computed (e.g. ``euler_*``).
* ``trajectory.csv`` (``samplesToCsv``), ``pareto.csv``/``history.csv`` (mdo), ``sweep.csv``, ``sensitivity.csv``.

Point columns use the CSV names (``CL``, ``Cma_per_rad`` …) whichever file was read. Static derivatives are per
radian: central inside an axis, one-sided at its ends (main 55d643f). The per-point rate columns (``CLq``, ``Cmq``,
``Clp`` …) are zero except at one anchor point per Mach; use :attr:`AeroDatabase.dynamics` /
:meth:`AeroDatabase.dynamic_interpolator` for them.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
import pandas as pd

from .interp import GridInterpolator

PathLike = Union[str, os.PathLike]

FORMAT = "cuspis-aerodb-1"
AXES = ("mach", "alpha_deg", "beta_deg", "delta_deg")

#: aerodb.ts AeroDbPoint field -> aeroDbToCsv column (same order as the CSV header)
POINT_FIELDS: Tuple[Tuple[str, str], ...] = (
    ("mach", "mach"), ("alpha_deg", "alpha_deg"), ("beta_deg", "beta_deg"), ("delta_deg", "delta_deg"),
    ("onDesign", "on_design"),
    ("cl", "CL"), ("cd", "CD"), ("cy", "CY"), ("cm", "Cm"), ("cn", "Cn"), ("cll", "Cll"), ("ca", "CA"), ("ld", "LD"),
    ("cla", "CLa_per_rad"), ("cda", "CDa_per_rad"), ("cma", "Cma_per_rad"),
    ("cyb", "CYb_per_rad"), ("cnb", "Cnb_per_rad"), ("clb", "Clb_per_rad"),
    ("cld", "CLd_per_rad"), ("cmd", "Cmd_per_rad"),
    ("clq", "CLq"), ("cmq", "Cmq"), ("clp", "Clp"), ("cnp", "Cnp"), ("cyp", "Cyp"), ("cnr", "Cnr"), ("clr", "Clr"), ("cyr", "Cyr"),
    ("euler_cl", "euler_CL"), ("euler_cd", "euler_CD"), ("euler_cm", "euler_Cm"),
)
CSV_COLUMNS = tuple(c for _, c in POINT_FIELDS)
COEFS = CSV_COLUMNS[5:]
_F2C = dict(POINT_FIELDS)
DYNAMIC_FIELDS = ("clq", "cmq", "clp", "cnp", "cyp", "cnr", "clr", "cyr")

#: '# key: value' CSV header lines that are numbers
_NUMERIC_META = {"design_mach", "altitude_km", "sref_m2", "lref_m", "bref_m", "mass_kg", "gamma", "cg_x_m", "cg_y_m", "cg_z_m"}


def _num(v):
    return np.nan if v is None else v


@dataclass
class AeroDatabase:
    """One aero database. ``points`` has one row per grid point in main's flat order (delta fastest)."""

    meta: Dict[str, object]
    axes: Dict[str, np.ndarray]
    points: pd.DataFrame
    dynamics: pd.DataFrame = field(default_factory=pd.DataFrame)
    summary: Dict[str, object] = field(default_factory=dict)
    slices: Dict[str, object] = field(default_factory=dict)
    source: Optional[Path] = None

    @property
    def shape(self) -> Tuple[int, ...]:
        return tuple(len(self.axes[a]) for a in AXES)

    def grid(self, coef: str) -> np.ndarray:
        """``coef`` as an array of shape (n_mach, n_alpha, n_beta, n_delta); NaN where a point is missing."""
        if coef not in self.points:
            raise KeyError(f"{coef!r} not a column; known: {', '.join(COEFS)}")
        out = np.full(self.shape, np.nan)
        idx = tuple(_axis_index(self.axes[a], self.points[a].to_numpy(float), a) for a in AXES)
        out[idx] = self.points[coef].to_numpy(float)
        return out

    def interpolator(self, coefs: Sequence[str] = ("CL", "CD", "Cm"), extrapolate: str = "clip") -> GridInterpolator:
        """4-D multilinear interpolant over (mach, alpha_deg, beta_deg, delta_deg); length-1 axes are constant."""
        for c in coefs:
            if c in ("CLq", "Cmq", "Clp", "Cnp", "Cyp", "Cnr", "Clr", "Cyr"):
                raise ValueError(f"{c} is only filled at one anchor point per Mach; use dynamic_interpolator()")
        vals = {c: self.grid(c) for c in coefs}
        for c, v in vals.items():
            if np.isnan(v).any():
                raise ValueError(f"{c} has missing/non-finite grid values (e.g. euler_* outside the Euler slice)")
        return GridInterpolator([self.axes[a] for a in AXES], vals, names=AXES, extrapolate=extrapolate)

    def dynamic_interpolator(self, coefs: Sequence[str] = ("Cmq", "Clp", "Cnr"), extrapolate: str = "clip") -> GridInterpolator:
        """1-D interpolant over Mach of the rate derivatives (per radian of q̂ = qL/2V, p̂ = pb/2V, r̂ = rb/2V)."""
        d = self.dynamics.sort_values("mach")
        return GridInterpolator([d["mach"].to_numpy(float)], {c: d[c].to_numpy(float) for c in coefs}, names=("mach",), extrapolate=extrapolate)


def _axis_index(axis: np.ndarray, x: np.ndarray, name: str) -> np.ndarray:
    i = np.clip(np.searchsorted(axis, x), 0, len(axis) - 1)
    j = np.clip(i - 1, 0, len(axis) - 1)
    pick = np.where(np.abs(axis[j] - x) < np.abs(axis[i] - x), j, i)
    if np.any(np.abs(axis[pick] - x) > 1e-6 * max(1.0, float(np.max(np.abs(axis))))):
        raise ValueError(f"{name}: point values not on the axis breakpoints")
    return pick


def _points_frame(rows: List[dict]) -> pd.DataFrame:
    cols = {c: [] for c in CSV_COLUMNS}
    for r in rows:
        for f, c in POINT_FIELDS:
            cols[c].append(_num(r.get(f)))
    df = pd.DataFrame(cols)
    df["on_design"] = df["on_design"].astype(bool)
    for c in CSV_COLUMNS:
        if c != "on_design":
            df[c] = df[c].astype(float)
    return df


def read_json(path: PathLike) -> AeroDatabase:
    """Read ``aerodb.json`` (``aeroDbToJson``)."""
    p = Path(path)
    raw = json.loads(p.read_text(encoding="utf-8"))
    meta = raw.get("meta", {})
    if meta.get("format") != FORMAT:
        raise ValueError(f"{p}: meta.format {meta.get('format')!r} != {FORMAT!r}")
    axes = {a: np.asarray(raw["axes"][a], dtype=float) for a in AXES}
    dyn = pd.DataFrame([{**{k: _num(r.get(k)) for k in ("mach", "alpha_deg", "delta_deg")}, **{_F2C[k]: _num(r.get(k)) for k in DYNAMIC_FIELDS}} for r in raw.get("dynamics", [])])
    return AeroDatabase(meta, axes, _points_frame(raw.get("points", [])), dyn.astype(float) if len(dyn) else dyn, raw.get("summary", {}), raw.get("slices", {}), p)


def read_meta_csv(path: PathLike) -> Dict[str, object]:
    """Parse the ``#`` header block of ``aerodb.csv`` into a dict (numbers converted, assumptions split on ' | ')."""
    meta: Dict[str, object] = {}
    free: List[str] = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            if not line.startswith("#"):
                break
            s = line[1:].strip()
            if s.startswith("cuspis aerodynamic database"):
                meta["format"] = s.split()[-1]
            elif s.startswith("angles:"):
                parts = s.split()
                meta["units"] = {"angles": parts[1], "derivatives": " ".join(parts[3:])}
            elif ": " in s and s.split(": ", 1)[0].replace("_", "").isalnum() and " " not in s.split(": ", 1)[0]:
                k, v = s.split(": ", 1)
                meta[k] = float(v) if k in _NUMERIC_META else v
            else:
                free.append(s)
    if free:
        meta["on_design_note"] = free[0] if free[0].startswith("on_design") else None
        rest = [f for f in free if not f.startswith("on_design")]
        if rest:
            meta["assumptions"] = rest[-1].split(" | ")
    return meta


def read_csv(path: PathLike) -> AeroDatabase:
    """Read ``aerodb.csv`` (``aeroDbToCsv``). Values carry 6 decimals; prefer :func:`read_json` when available."""
    p = Path(path)
    meta = read_meta_csv(p)
    if meta.get("format") != FORMAT:
        raise ValueError(f"{p}: not a {FORMAT} CSV")
    nskip = 0
    with open(p, encoding="utf-8") as fh:
        for line in fh:
            if not line.startswith("#"):
                break
            nskip += 1
    df = pd.read_csv(p, skiprows=nskip)
    missing = [c for c in CSV_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"{p}: missing columns {missing}")
    df = df[list(CSV_COLUMNS)].copy()
    df["on_design"] = df["on_design"].astype(int).astype(bool)
    axes = {a: np.unique(df[a].to_numpy(float)) for a in AXES}
    # rate derivatives are written on every row but only filled at the per-Mach anchor (nonzero Cmq)
    anchor = df[(df[["CLq", "Cmq", "Clp", "Cnr"]].abs().sum(axis=1) > 0)]
    dyn = anchor[["mach", "alpha_deg", "delta_deg", "CLq", "Cmq", "Clp", "Cnp", "Cyp", "Cnr", "Clr", "Cyr"]].reset_index(drop=True)
    return AeroDatabase(meta, axes, df.reset_index(drop=True), dyn, {}, {}, p)


def load_database(path: PathLike) -> AeroDatabase:
    """Load a CLI output directory (``aerodb.json`` preferred, else ``aerodb.csv``) or a single file."""
    p = Path(path)
    if p.is_dir():
        for name, fn in (("aerodb.json", read_json), ("aerodb.csv", read_csv)):
            if (p / name).is_file():
                return fn(p / name)
        raise FileNotFoundError(f"{p}: no aerodb.json or aerodb.csv")
    return read_json(p) if p.suffix.lower() == ".json" else read_csv(p)


def load_manifest(out_dir: PathLike) -> Dict[str, object]:
    return json.loads((Path(out_dir) / "manifest.json").read_text(encoding="utf-8"))


def _read_commented_csv(path: PathLike) -> Tuple[pd.DataFrame, List[str]]:
    notes: List[str] = []
    n = 0
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            if not line.startswith("#"):
                break
            notes.append(line[1:].strip())
            n += 1
    return pd.read_csv(path, skiprows=n), notes


def load_trajectory(path: PathLike) -> pd.DataFrame:
    """``trajectory.csv`` from ``--mode fly`` (guidance.ts ``samplesToCsv``). Header notes go to ``df.attrs['notes']``."""
    p = Path(path)
    if p.is_dir():
        p = p / "trajectory.csv"
    df, notes = _read_commented_csv(p)
    df.attrs["notes"] = notes
    return df


def load_table(path: PathLike) -> pd.DataFrame:
    """Any plain CSV from the CLI: ``pareto.csv``, ``history.csv``, ``sweep.csv``, ``sensitivity.csv``."""
    df, notes = _read_commented_csv(path)
    df.attrs["notes"] = notes
    return df
