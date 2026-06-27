#!/usr/bin/env python3
"""Plot and summarize the per-case coefficient histories from `convergence.py`.

`convergence.py` writes one tidy CSV per case under `results/`
(`<part>_<regime>_Ma<Ma>_a<alpha>.csv`, columns Time + every coefficient). This
script consumes those and, for each case:

  * draws one PNG with two stacked panels -> `results/plots/<case>.png`
      top:    Cd, Cl, Cs   vs Time (iteration)
      bottom: CmPitch, CmRoll, CmYaw vs Time
    The averaging window (last --window iterations) is shaded so the plot shows
    exactly what the summary averaged over.
  * contributes one row to `results/averages.csv`: part, regime, Ma, alpha,
    n_iters, then mean+std of Cd, Cl, Cs, CmPitch, CmRoll, CmYaw over the window.

Re-runnable as cases arrive: it rebuilds the plots and averages.csv from whatever
result CSVs currently exist (idempotent overwrite). A short or unreadable CSV is
warned and skipped, never fatal.

    uv run python openfoam/plot_coeffs.py
    uv run python openfoam/plot_coeffs.py --window 25
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import NamedTuple

import matplotlib

matplotlib.use("Agg")  # headless: just write PNGs
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

HERE = Path(__file__).resolve().parent  # openfoam/

# Filename -> case metadata. Doubles as the file filter: the output averages.csv
# and the stray `..._export.csv` duplicate fail the `$` anchor and are skipped.
CASE_RE = re.compile(
    r"^(?P<part>\w+?)_(?P<regime>subsonic|supersonic)_Ma(?P<Ma>[\d.]+)_a(?P<alpha>\d+)\.csv$"
)

FORCE_COEFFS = ["Cd", "Cl", "Cs"]
MOMENT_COEFFS = ["CmPitch", "CmRoll", "CmYaw"]
SUMMARY_COEFFS = FORCE_COEFFS + MOMENT_COEFFS


class Case(NamedTuple):
    path: Path
    name: str  # filename stem, e.g. all_supersonic_Ma8_a10
    part: str
    regime: str
    Ma: str
    alpha: str


def discover_cases(results: Path) -> list[Case]:
    """Result CSVs whose name matches the <part>_<regime>_Ma_a convention."""
    cases: list[Case] = []
    for path in sorted(results.glob("*.csv")):
        m = CASE_RE.match(path.name)
        if not m:
            continue
        cases.append(Case(path, path.stem, m["part"], m["regime"], m["Ma"], m["alpha"]))
    # stable, diff-friendly order: part, regime, Ma (numeric), alpha (numeric)
    cases.sort(key=lambda c: (c.part, c.regime, float(c.Ma), int(c.alpha)))
    return cases


def summarize(df: pd.DataFrame, window: int) -> dict[str, float]:
    """Mean+std of each summary coefficient over the last `window` rows."""
    tail = df.tail(window)
    out: dict[str, float] = {}
    for col in SUMMARY_COEFFS:
        out[f"{col}_mean"] = float(tail[col].mean())
        out[f"{col}_std"] = float(tail[col].std())
    return out


def plot_case(df: pd.DataFrame, name: str, window: int, dst: Path) -> None:
    """Two stacked panels (forces, moments) vs Time, window shaded, -> dst.

    Each panel's y-axis is cropped to the converged window's data range padded by
    100% on each side, so the startup transient runs off-screen and the settled
    region fills the plot.
    """
    t = df["Time"]
    n = min(window, len(df))
    win = df.tail(n)
    win_start = t.iloc[-n]  # left edge of the averaged span

    fig, (ax_f, ax_m) = plt.subplots(2, 1, sharex=True, figsize=(9, 7))
    for ax, cols, ylabel in (
        (ax_f, FORCE_COEFFS, "force coefficient"),
        (ax_m, MOMENT_COEFFS, "moment coefficient"),
    ):
        for col in cols:
            ax.plot(t, df[col], label=col, linewidth=1.2)
        ax.axvspan(win_start, t.iloc[-1], color="0.85", zorder=0,
                   label=f"avg window ({n} it.)")
        lo, hi = float(win[cols].to_numpy().min()), float(win[cols].to_numpy().max())
        span = hi - lo
        if span > 0:  # pad the window range by 100% each side (skip if flat)
            ax.set_ylim(lo - span, hi + span)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize="small")

    ax_m.set_xlabel("Time (iteration)")
    ax_f.set_title(name)
    fig.tight_layout()
    fig.savefig(dst, dpi=120)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--results", type=Path, default=HERE / "results",
                    help="dir with the per-case CSVs (default openfoam/results/)")
    ap.add_argument("--plots", type=Path, default=None,
                    help="output dir for PNGs (default <results>/plots/)")
    ap.add_argument("--summary", type=Path, default=None,
                    help="averages CSV path (default <results>/averages.csv)")
    ap.add_argument("--window", type=int, default=50,
                    help="iterations to average over, from the end (default 50)")
    args = ap.parse_args()

    plots_dir = args.plots or args.results / "plots"
    summary_path = args.summary or args.results / "averages.csv"

    cases = discover_cases(args.results)
    if not cases:
        sys.exit(f"no case CSVs found in {args.results}")

    plots_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    skipped: list[str] = []
    for c in cases:
        try:
            df = pd.read_csv(c.path)
            if df.empty or not set(SUMMARY_COEFFS + ["Time"]).issubset(df.columns):
                raise ValueError("missing coefficient columns or no rows")
        except Exception as e:  # noqa: BLE001 - any unreadable CSV is skip-not-fatal
            skipped.append(f"{c.path.name}: {e}")
            continue

        plot_case(df, c.name, args.window, plots_dir / f"{c.name}.png")
        rows.append({
            "part": c.part, "regime": c.regime, "Ma": c.Ma, "alpha": c.alpha,
            "n_iters": min(args.window, len(df)),
            **summarize(df, args.window),
        })
        print(f"  {c.name}.png  ({len(df)} iterations)")

    if rows:
        pd.DataFrame(rows).to_csv(summary_path, index=False)

    print(f"\n{len(rows)} case(s): PNGs -> {plots_dir}, averages -> {summary_path}", end="")
    if skipped:
        print(f", {len(skipped)} skipped:")
        for s in skipped:
            print(f"  ! {s}", file=sys.stderr)
    else:
        print()


if __name__ == "__main__":
    main()
