"""CSV files with a header of named columns (static checks, gravity-compensation logs)."""

from __future__ import annotations

from pathlib import Path

import numpy as np


def read_named_csv(path: str | Path, delimiter: str = ",") -> tuple[list[str], np.ndarray]:
    """Read a numeric CSV whose first line is a header of column names."""
    with Path(path).open("r", encoding="utf-8") as handle:
        header = [name.strip() for name in handle.readline().strip().lstrip("#").split(delimiter)]
    data = np.loadtxt(path, delimiter=delimiter, skiprows=1, ndmin=2)
    if data.shape[1] != len(header):
        raise ValueError(f"{path}: header has {len(header)} names but rows have {data.shape[1]} values")
    return header, data


def named_columns(header: list[str], prefix: str, dof: int | None = None) -> list[int] | None:
    """Indices of ``prefix1..prefixN`` in ``header``, or None if ``prefix1`` is absent."""
    if f"{prefix}1" not in header:
        return None
    if dof is None:
        dof = 0
        while f"{prefix}{dof + 1}" in header:
            dof += 1
    missing = [f"{prefix}{i}" for i in range(1, dof + 1) if f"{prefix}{i}" not in header]
    if missing:
        raise ValueError(f"CSV header is missing {', '.join(missing)}")
    return [header.index(f"{prefix}{i}") for i in range(1, dof + 1)]


def save_motion_csv(path: str | Path, time: np.ndarray | None = None, **arrays: np.ndarray) -> Path:
    """Write ``time, q1.., dq1.., ...`` with a named header (keyword order is kept)."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    names, columns = [], []
    if time is not None:
        names.append("time")
        columns.append(np.asarray(time, dtype=float).reshape(-1, 1))
    for prefix, values in arrays.items():
        values = np.asarray(values, dtype=float)
        names.extend(f"{prefix}{i}" for i in range(1, values.shape[1] + 1))
        columns.append(values)
    np.savetxt(destination, np.hstack(columns), delimiter=",", header=",".join(names), comments="", fmt="%.10g")
    return destination
