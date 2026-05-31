import re

import numpy as np
import pandas as pd


def burn_rate(fuel, p_k, path="assets/fuels.csv"):
    """Return (u, rho*u) for a fuel at chamber pressure p_k.

    The CSV's `u-p_k` column stores a law of the form `a*p_k^(n)`,
    so u(p_k) = a * p_k**n.
    """
    row = pd.read_csv(path).set_index("fuel").loc[fuel]

    m = re.match(r"([\d.]+)\*p_k\^\(([\d.]+)\)", row["u-p_k"])
    if m is None:
        raise ValueError(f"unrecognized burn-rate law for {fuel!r}: {row['u-p_k']!r}")
    a, n = map(float, m.groups())
    u = a * p_k**n  # mm/s
    return u, float(row["rho"]) * u / 1000  # rho [kg/m³] * u [mm/s] → kg/(m²·s)


def alpha_dv(rho_u, l_z, path="assets/chart-4-26-alpha.csv"):
    """Return alpha_dv by interpolating chart 4-26.

    Performs bilinear interpolation: first within each l_z curve over rho_u,
    then across curves for the given l_z.
    """
    raw = pd.read_csv(path, header=1)
    l_z_values = (
        pd.read_csv(path, header=None, nrows=1)
        .iloc[0, ::2]
        .dropna()
        .astype(int)
        .tolist()
    )

    per_curve = {}
    for i, lz in enumerate(l_z_values):
        col = i * 2
        x = raw.iloc[:, col].dropna().astype(float).to_numpy()
        y = raw.iloc[:, col + 1].dropna().astype(float).to_numpy()
        order = np.argsort(x)
        per_curve[lz] = (x[order], y[order])

    alpha_at_lz = {lz: float(np.interp(rho_u, *per_curve[lz])) for lz in l_z_values}
    lz_arr = np.array(sorted(alpha_at_lz))
    alpha_arr = np.array([alpha_at_lz[lz] for lz in lz_arr])
    return float(np.interp(l_z, lz_arr, alpha_arr))


def l_coefficient(rho_u, p_idx, path="assets/chart-4-27-l.csv"):
    """Return lambda_p for a given rho*u value by interpolating chart 4-27.

    p_idx: 1, 2, or 3 — selects the lambda_p curve.
    """
    raw = pd.read_csv(path, header=1)  # skip lambda_p row, use X/Y row
    col = (p_idx - 1) * 2  # each curve occupies two columns
    x, y = (
        raw.iloc[:, col].dropna().astype(float),
        raw.iloc[:, col + 1].dropna().astype(float),
    )
    return float(np.interp(rho_u, x, y))
