"""Render the optimized pitch program as a Typst table on stdout.

`optimizer/out/best.json` carries the program as a flat list of hermite arcs,
each holding only its own end: `t_end`, `theta_deg` and `k`, the exit slope in
deg/s. The start of an arc is the end of the one before it — which is exactly
how the report states it — so the table lists ends only, and row 0 holds the
program start (`t_start`, `theta_deg_start`, and the zero entry slope
`pitch.go` hands the first arc).

`k` is read as $dot(theta.alt)$, which holds for hermite arcs alone; a cos arc
carries an exponent there and is rejected.

    uv run python report/pitch_table.py
    uv run python report/pitch_table.py --json optimizer/out/best.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import typst

HERE = Path(__file__).resolve().parent
DEFAULT_JSON = HERE.parent / "optimizer" / "out" / "best.json"

HEADERS = (
    "Участок",
    '$t_"end"$, с',
    '$theta.alt_"end"$, град',
    '$dot(theta.alt)_"end"$, град/с',
)


def cell(value, digits):
    """Format one value to `digits` decimals, without a signed zero."""
    text = f"{value:.{digits}f}"
    if float(text) == 0:
        text = text.lstrip("-")
    return f"${text}$"


def rows(pitch):
    """Build the table rows: the program start, then one per arc."""
    out = [(0, pitch["t_start"], pitch["theta_deg_start"], 0.0)]
    for i, seg in enumerate(pitch["segments"], start=1):
        out.append((i, seg["t_end"], seg["theta_deg"], seg["k"]))
    return out


def emit(pitch):
    """Print the program as a captioned Typst figure."""
    print("#figure(")
    print("  table(")
    print(f"    columns: {len(HEADERS)},")
    print("    table.header(" + ", ".join(f"[{h}]" for h in HEADERS) + "),")
    for i, t, theta, k in rows(pitch):
        cells = [f"${i}$", cell(t, 2), cell(theta, 2), cell(k, 3)]
        print("    " + ", ".join(cells) + ",")
    print("  ),")
    print("  caption: [Программа изменения угла тангажа]")
    print(")")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", type=Path, default=DEFAULT_JSON, help="optimizer output")
    args = ap.parse_args()

    pitch = json.loads(args.json.read_text())["pitch"]
    shapes = {seg["shape"] for seg in pitch["segments"]}
    if shapes != {"hermite"}:
        sys.exit(f"{args.json}: k is a slope only on hermite arcs, got {shapes}")

    typst.comment("Программа тангажа")
    print()
    emit(pitch)


if __name__ == "__main__":
    main()
