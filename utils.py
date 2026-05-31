import re

import numpy as np
import pandas as pd

G0 = 9.80665  # m/s²


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


def fuel_props(fuel, path="assets/fuels.csv"):
    """Return a dict of fuel properties from the CSV."""
    return pd.read_csv(path).set_index("fuel").loc[fuel].to_dict()


def specific_thrust_corrected(P_ud_st, al_pct):
    """Return corrected standard specific thrust (s).

    No aluminum: fixed 4% efficiency loss → 0.96 * P_ud_st.
    With aluminum: two-phase (Al₂O₃) correction per formula (3.1).
    """
    if al_pct == 0:
        return 0.96 * float(P_ud_st)
    a = al_pct
    correction = (4.3 + 0.17 * a + 0.009 * a**2) * 1e-2
    return P_ud_st * (1 - correction)


def specific_thrust_design(P_ud_pr, p_k, p_a):
    """Specific thrust at actual chamber/exit pressures (bar), result in seconds."""
    return P_ud_pr + 19.4 + 0.76 * p_k - 0.003 * p_k**2 - 70 * p_a + 25 * p_a**2


def combustion_temp(T_st, p_k):
    """Actual combustion temperature (K) at chamber pressure p_k (bar)."""
    return T_st + 1.12 * (p_k - 40)


def specific_thrust_vacuum(P_ud_r, R, T, k, p_a, p_k):
    """Vacuum specific thrust (s) from design-condition specific thrust."""
    return P_ud_r + (R * T) / (G0**2 * P_ud_r) * (p_a / p_k) ** ((k - 1) / k)


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
