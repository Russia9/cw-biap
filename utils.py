import re

import numpy as np
import pandas as pd

G0 = 9.80665  # m/s²


def _norm_label(s: str) -> str:
    """Normalize a chart label: '2.0' → '2', 'lambda_p1' stays as-is."""
    try:
        f = float(s)
        return str(int(f)) if f == int(f) else s
    except ValueError:
        return s


def load_chart(path: str) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Load a multi-curve digitized chart CSV.

    Format: row 0 = curve labels (one per column-pair), row 1 = X/Y headers,
    remaining rows = data. Returns {label: (x_sorted, y)} for each curve.
    """
    raw_labels = (
        pd.read_csv(path, header=None, nrows=1)
        .iloc[0, ::2]
        .dropna()
        .astype(str)
        .tolist()
    )
    labels = [_norm_label(s) for s in raw_labels]
    data = pd.read_csv(path, header=1)
    curves: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for i, label in enumerate(labels):
        col = i * 2
        x = data.iloc[:, col].dropna().astype(float).to_numpy()
        y = data.iloc[:, col + 1].dropna().astype(float).to_numpy()
        order = np.argsort(x)
        curves[label] = (x[order], y[order])
    return curves


def interp_chart(path: str, key: str, x: float) -> float:
    """1D interpolation at x within the named curve in a chart CSV."""
    return float(np.interp(x, *load_chart(path)[key]))


def interp_chart_2d(path: str, x: float, y: float) -> float:
    """Bilinear interpolation: x within each curve, then y across numeric curve labels."""
    curves = load_chart(path)
    keyed = sorted((float(k), k) for k in curves)
    label_vals = np.array([kv for kv, _ in keyed])
    interped = np.array([float(np.interp(x, *curves[k])) for _, k in keyed])
    return float(np.interp(y, label_vals, interped))


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
    return interp_chart_2d(path, rho_u, l_z)


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
    return interp_chart(path, f"lambda_p{p_idx}", rho_u)


def k0_from_k(k, path="assets/table-k-k0.csv"):
    """Return K0 by linear interpolation from table 3.10."""
    df = pd.read_csv(path)
    return float(np.interp(k, df["k"].to_numpy(), df["K0"].to_numpy()))


def load_materials(path="assets/materials.csv"):
    """Return {id: value} of material properties from the materials table CSV."""
    df = pd.read_csv(path)
    return dict(zip(df["id"], df["value"].astype(float)))


def load_trajectory(path="assets/table-2.1.csv"):
    """Return the burnout-trajectory reference table (2.1) as a DataFrame.

    Columns: L (km), h_k (km), l_k (km), theta_k (deg), V_k (m/s),
    Lv (km per m/s) — full flight range mapped to active-segment parameters.
    """
    return pd.read_csv(path)
