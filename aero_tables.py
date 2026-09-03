"""Render the CFD coefficient sweep as Typst tables on stdout.

`openfoam/results/averages.csv` holds one row per (part, Ma, alpha) case. This
script reshapes it into one table per (part, coefficient): angle of attack down
the rows, Mach number across the columns, values to 3 decimals.

Only the three coefficients the trajectory simulator reads are emitted — Cd, Cl
and CmPitch (see `traj/aero.go`); Cs, CmRoll and CmYaw are symmetry residue of
order 1e-5 and would print as zeros.

The grid shape follows the data: each part carries its own Mach and alpha
points, so the column and row counts are derived per part, not fixed.

    uv run python aero_tables.py
    uv run python aero_tables.py --part all --coeff Cd
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

import typst

HERE = Path(__file__).resolve().parent
DEFAULT_CSV = HERE / "openfoam" / "results" / "averages.csv"

# CSV part key -> the caption phrase naming it, in flight order (matching
# `aeroParts` in traj/aero.go).
PARTS = {
    "all": "для полной ракеты",
    "stage2up": "для второй ступени",
    "stage3up": "для третьей ступени",
    "head": "для головной части",
}

# CSV column -> the Typst symbol the report uses for it.
COEFFS = {"Cd": "$C_(x a)$", "Cl": "$C_(y a)$", "CmPitch": "$m_(z a)$"}


def cell(value):
    """Format one coefficient to 3 decimals, without a signed zero."""
    text = f"{value:.3f}"
    return "$0.000$" if text == "-0.000" else f"${text}$"


def grid(part_df, column):
    """Reshape one part's rows into (alphas, machs, cells) for `column`.

    Cells with no CFD case are `typst.DASH`: the sweep grid is rectangular per
    part today, but traj/aero.go treats it as ragged and still growing.
    """
    alphas = sorted(part_df["alpha"].unique())
    machs = sorted(part_df["Ma"].unique())
    values = part_df.set_index(["alpha", "Ma"])[f"{column}_mean"].to_dict()
    cells = [
        [cell(values[a, m]) if (a, m) in values else typst.DASH for m in machs]
        for a in alphas
    ]
    return alphas, machs, cells


def emit(df, parts, coeffs):
    """Print one marked-up figure per (part, coefficient) present in the data."""
    for part in parts:
        part_df = df[df["part"] == part]
        if part_df.empty:
            continue
        for column in coeffs:
            typst.comment(f"{part}: {COEFFS[column]}")
            caption = f"Значения коэффициента {COEFFS[column]} {PARTS[part]}"
            typst.coeff_table(*grid(part_df, column), caption)
            print()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="CFD averages.csv")
    ap.add_argument(
        "--part", choices=list(PARTS), help="only this part (default: all four)"
    )
    ap.add_argument(
        "--coeff", choices=list(COEFFS), help="only this coefficient (default: all)"
    )
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    emit(
        df,
        [args.part] if args.part else list(PARTS),
        [args.coeff] if args.coeff else list(COEFFS),
    )


if __name__ == "__main__":
    main()
