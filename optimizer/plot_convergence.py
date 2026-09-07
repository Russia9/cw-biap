"""Plot the CMA-ES objective against the iteration count.

pycma prints one progress line per logged iteration, with a header line repeated
every so often:

    Iterat #Fevals   function value  axis ratio  sigma  min&max std  t[m:s]
        1     24 2.660618437019829e+07 1.0e+00 4.74e-02  2e-02  5e-01 0:01.2

This reads those lines back — from a saved log or straight off a pipe — and
draws f(iteration). The axis is logarithmic because the search spans some
sixteen decades: it starts inside the W_CON penalty band, where a §4.4 limit is
still violated, and ends on range miss alone.

    uv run python optimizer/main.py | tee optimizer/out/search.log
    uv run python optimizer/plot_convergence.py optimizer/out/search.log
    uv run python optimizer/main.py | uv run python optimizer/plot_convergence.py
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless: just write PNGs
import matplotlib.pyplot as plt

from main import W_CON

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"

# iteration, evaluations, objective — the rest of the line is not plotted.
ROW = re.compile(r"^\s*(\d+)\s+(\d+)\s+(-?\d+\.?\d*(?:[eE][-+]?\d+)?)\s")


def parse(lines):
    """Pull (iteration, f) out of pycma's progress lines, skipping every other
    line: the repeated header, the termination notes, anything main.py printed."""
    out = []
    for line in lines:
        m = ROW.match(line)
        if m:
            out.append((int(m[1]), float(m[3])))
    return out


def plot(history, dst, penalty_band):
    it = [i for i, _ in history]
    f = [v for _, v in history]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(it, f, color="C0", lw=1.4, label="f")
    if penalty_band:
        # One-way: above W_CON a limit is certainly violated, since the range term
        # alone would need a miss of 3e5 km to get there. Below it says nothing —
        # a small excess costs W_CON times its square.
        ax.axhline(
            W_CON,
            color="C3",
            ls=":",
            lw=1.2,
            label="выше - нарушаются ограничения",
        )
    ax.plot(it[-1], f[-1], "o", color="C2", ms=5, zorder=3, label=f"f = {f[-1]:.2e}")
    ax.set_yscale("log")
    ax.set_xlabel("итерация")
    ax.set_ylabel("значение целевой функции")
    ax.grid(True, which="major", alpha=0.3)
    ax.grid(True, which="minor", alpha=0.12)
    ax.legend(loc="best", fontsize="small")
    fig.tight_layout()
    fig.savefig(dst, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "log", nargs="?", type=Path, help="saved pycma output (default: stdin)"
    )
    ap.add_argument("--out", type=Path, default=OUT / "plots" / "convergence.png")
    args = ap.parse_args()

    lines = args.log.read_text().splitlines() if args.log else sys.stdin.readlines()
    history = parse(lines)
    if not history:
        sys.exit("no pycma progress lines found")

    positive = [(i, v) for i, v in history if v > 0]
    if len(positive) < len(history):
        print(
            f"! dropped {len(history) - len(positive)} non-positive f", file=sys.stderr
        )
    if not positive:
        sys.exit("every f is non-positive, nothing to draw on a log axis")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    plot(positive, args.out, max(v for _, v in positive) > W_CON)
    print(
        f"{len(positive)} iterations, f: {positive[0][1]:.3e} -> {positive[-1][1]:.3e}"
    )
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
