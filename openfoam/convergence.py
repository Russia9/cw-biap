#!/usr/bin/env python3
"""Extract per-iteration force-coefficient histories from the sweep cases.

`sweep.py` records only each case's *final* Cd/Cl/Cm. To check that a case
actually converged (coefficients leveled off rather than still drifting), we
need the full iteration history OpenFOAM writes to

    <case>/postProcessing/forceCoeffs/<time>/coefficient.dat

This walks every generated case under --base, parses that file (whitespace
columns under a '#'-comment label line), and writes one tidy CSV per case:

    <out>/<part>_<regime>_Ma<Ma>_a<alpha>.csv   (Time + every coefficient column)

    uv run python openfoam/convergence.py
    uv run python openfoam/convergence.py --only-part all
    uv run python openfoam/convergence.py --base openfoam --out openfoam/results

Disk-driven: it parses whatever produced output, independent of
sweep_state.json. A case with no/empty forceCoeffs output is warned and skipped,
never fatal.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent  # openfoam/
PARTS = ["all", "stage2up", "stage3up", "head"]  # mirrors sweep.py
REGIMES = {"subsonic", "supersonic"}


class ParseError(Exception):
    """A coefficient.dat that can't be turned into rows (skip this case)."""


def parse_coeff_file(path: Path) -> tuple[list[str], list[list[str]]]:
    """(labels, rows) from one coefficient.dat.

    Labels are taken from the '#' comment line whose tokens include 'Cd'
    (e.g. 'Time Cd Cd(f) ... Cs(r)'); rows are the whitespace-split data lines.
    Each row is aligned to the label count so a short/long line can't desync
    the columns.
    """
    labels: list[str] = []
    rows: list[list[str]] = []
    for line in path.read_text(errors="replace").splitlines():
        s = line.strip()
        if not s:
            continue
        if s.startswith("#"):
            toks = s.lstrip("#").split()
            if "Cd" in toks:  # the column-label comment line
                labels = toks
            continue
        rows.append(s.split())
    if not labels:
        raise ParseError(f"no column-label line (with 'Cd') in {path}")
    if not rows:
        raise ParseError(f"no data rows in {path}")
    width = max(len(labels), max(len(r) for r in rows))
    labels = labels + [f"col{i}" for i in range(len(labels), width)]
    rows = [r + [""] * (width - len(r)) for r in rows]
    return labels, rows


def read_case_history(force_dir: Path) -> tuple[list[str], list[list[str]]]:
    """Merge all forceCoeffs/<time>/coefficient.dat into one Time-sorted history.

    A solver restart writes a new <time> dir; reading them in ascending order
    and keying rows by the Time column lets a later run overwrite an earlier
    run's overlapping iterations, yielding a single monotonic history.
    """
    time_dirs = sorted(
        (d for d in force_dir.iterdir() if d.is_dir() and (d / "coefficient.dat").is_file()),
        key=lambda d: float(d.name),
    )
    if not time_dirs:
        raise ParseError(f"no <time>/coefficient.dat under {force_dir}")
    labels: list[str] = []
    by_time: dict[float, list[str]] = {}
    for d in time_dirs:
        lbls, rows = parse_coeff_file(d / "coefficient.dat")
        if len(lbls) >= len(labels):  # keep the widest label set seen
            labels = lbls
        for r in rows:
            try:
                t = float(r[0])
            except (ValueError, IndexError):
                continue
            by_time[t] = r
    if not by_time:
        raise ParseError(f"no parseable data rows under {force_dir}")
    ordered = [by_time[t] for t in sorted(by_time)]
    return labels, ordered


def discover_cases(base: Path, pattern: str, only: set[str] | None) -> list[Path]:
    """forceCoeffs dirs under base matching the part/regime/Ma..._a... layout."""
    found: list[Path] = []
    for force_dir in sorted(base.glob(pattern)):
        if not force_dir.is_dir():
            continue
        case = force_dir.parent.parent  # .../<name>/postProcessing/forceCoeffs
        part, regime = case.parent.parent.name, case.parent.name
        if part not in PARTS or regime not in REGIMES:
            continue
        if only and part not in only:
            continue
        found.append(force_dir)
    return found


def out_name(force_dir: Path) -> str:
    case = force_dir.parent.parent
    part, regime, name = case.parent.parent.name, case.parent.name, case.name
    return f"{part}_{regime}_{name}.csv"


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--base", type=Path, default=HERE, help="root holding the case dirs (default openfoam/)")
    ap.add_argument("--out", type=Path, default=HERE / "results", help="output dir for CSVs (default openfoam/results/)")
    ap.add_argument("--only-part", action="append", choices=PARTS, help="restrict to part(s); repeatable")
    ap.add_argument("--glob", default="*/*/Ma*_a*/postProcessing/forceCoeffs", help="case discovery pattern under --base")
    args = ap.parse_args()

    only = set(args.only_part) if args.only_part else None
    force_dirs = discover_cases(args.base, args.glob, only)
    if not force_dirs:
        sys.exit(f"no forceCoeffs output found under {args.base} (pattern: {args.glob})")

    args.out.mkdir(parents=True, exist_ok=True)
    written = 0
    skipped: list[str] = []
    for force_dir in force_dirs:
        name = out_name(force_dir)
        try:
            labels, rows = read_case_history(force_dir)
        except ParseError as e:
            skipped.append(f"{name}: {e}")
            continue
        dst = args.out / name
        with dst.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(labels)
            w.writerows(rows)
        print(f"  {name}  ({len(rows)} iterations)")
        written += 1

    print(f"\n{written} CSV(s) -> {args.out}", end="")
    if skipped:
        print(f", {len(skipped)} skipped:")
        for s in skipped:
            print(f"  ! {s}", file=sys.stderr)
    else:
        print()


if __name__ == "__main__":
    main()
