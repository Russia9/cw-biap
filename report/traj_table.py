"""Render the flown trajectory as a Typst table on stdout.

`optimizer/out/best.csv` is the simulator's row-per-step dump in SI units, with
angles in rad and the stage index this table drops. Rows are thinned to a fixed
time grid (50 s), and the characteristic instants — every stage separation and
the impact — are added on top; those do not land on the grid, so their `t` is
quoted to 4 decimals instead of 2.

`cmd/main` writes each staging instant twice and takes several half-steps at
H = 0; the duplicate times are collapsed, keeping the last row of each. At a
separation both copies are identical and already carry the *next* stage, so a
separation row shows the mass after the drop.

    uv run python report/traj_table.py
    uv run python report/traj_table.py --csv optimizer/out/best.csv --step 50
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
DEFAULT_CSV = HERE.parent / "optimizer" / "out" / "best.csv"

DEG = 180 / np.pi

# (CSV column, factor into the header's unit, decimals) — in table order, after t.
COLUMNS = (
    ("m", 1.0, 0),
    ("x", 1e-3, 2),
    ("y", 1e-3, 2),
    ("Vx", 1.0, 2),
    ("Vy", 1.0, 2),
    ("V", 1.0, 2),
    ("r", 1e-3, 2),
    ("h", 1e-3, 2),
    ("q", 1e-3, 2),
    ("Ma", 1.0, 2),
    ("pitch", DEG, 2),
    ("flightAngle", DEG, 2),
    ("attack", DEG, 2),
)

HEADER = (
    '$t, "c"$, $m, "кг"$, $x, "км"$, $y, "км"$, $V_x, "м"/"с"$, $V_y, "м"/"с"$, '
    '$V, "м"/"с"$, $r, "км"$, $h, "км"$, $q, "кПа"$, $M$, $theta.alt, "град."$, '
    '$theta, "град."$, $alpha, "град."$'
)


def cell(value, digits):
    """Format one value to `digits` decimals, without a signed zero."""
    text = f"{value:.{digits}f}"
    if float(text) == 0:
        text = text.lstrip("-")
    return f"${text}$"


def load(path):
    """Read the trajectory, collapsing the times written more than once."""
    df = pd.read_csv(path)
    return df.drop_duplicates(subset="t", keep="last").reset_index(drop=True)


def pick(df, step):
    """Return `(index, is_characteristic)` pairs, in time order.

    Characteristic rows are the stage separations and the impact; the rest are
    the rows closest to each multiple of `step`.
    """
    char = set(df.index[df.stage.diff() > 0])
    char.add(df.index[-1])

    grid = np.arange(0, df.t.iloc[-1], step)
    nearest = {int(np.abs(df.t - t).idxmin()) for t in grid}

    return [(i, i in char) for i in sorted(char | nearest)]


def emit(df, rows):
    """Print the picked rows as a Typst table."""
    print("#figure(")
    print("  table(")
    print(f"    columns: {len(COLUMNS) + 1},")
    print(f"    table.header({HEADER}),")
    print()
    for i, is_char in rows:
        row = df.loc[i]
        cells = [cell(row.t, 4 if is_char else 2)]
        cells += [cell(row[name] * scale, digits) for name, scale, digits in COLUMNS]
        print("    " + ", ".join(cells) + ",")
    print("  ),")
    print(")")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="simulator output")
    ap.add_argument("--step", type=float, default=50.0, help="grid step [s]")
    args = ap.parse_args()

    df = load(args.csv)
    emit(df, pick(df, args.step))


if __name__ == "__main__":
    main()
