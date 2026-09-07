"""Draw the flown trajectory from `optimizer/out/best.csv` as six report figures.

`trajectory/cmd/main` writes one row per integration step; this script cuts that
into the plots the report needs, one PNG per figure under `out/plots/`:

    pitch.png       ϑ(t) and θ(t) on the active leg
    program.png     ϑ(t) alone, with the arcs of the pitch program marked
    alpha.png       α(t) on the active leg, against the §4.4 limits
    mass.png        m(t) on the active leg
    altitude.png    h(t) over the whole flight
    velocity.png    V(t) over the whole flight
    trajectory.png  y(x) with the Earth drawn

The **active leg** is the powered part of the flight: the rows whose stage is
still a powered one, the same cut `metrics()` in main.py scores. The limits and
the atmosphere constants are imported from main.py rather than restated, so a
change to `LIMITS` moves the lines on alpha.png with it.

Every figure marks the staging instants.

    uv run python optimizer/plot_traj.py
    uv run python optimizer/plot_traj.py --csv optimizer/out/best.csv
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless: just write PNGs
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from main import H_ATM, LIMITS, Q_SEP, RZ

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"

R2D = 180 / np.pi
ROMAN = ("I", "II", "III")
M_SUB = 1.1  # Ma below which the subsonic alpha limit applies, as in metrics()


def separations(df):
    """One row per staging instant.

    `cmd/main` writes each instant twice and resolves the stage from the right,
    so the second row already carries the new stage — the same row `metrics()`
    in main.py picks.
    """
    return df[df.stage.diff() > 0]


def mark(ax, sep):
    """Draw a dashed line at every staging instant, labelled by the stage left
    behind. Labels are stepped down the axes so they stay readable on the
    whole-flight plots, where the three events nearly coincide, and flip to the
    left of their line once it runs close to the right edge."""
    lo, hi = ax.get_xlim()
    for n, t in enumerate(sep.t):
        ax.axvline(t, color="0.45", linestyle="--", linewidth=0.9, zorder=0)
        name = ROMAN[n] if n < len(ROMAN) else str(n + 1)
        right = t > lo + 0.85 * (hi - lo)
        ax.annotate(
            f"отд. {name}",
            xy=(t, 1.0 - 0.07 * n),
            xycoords=("data", "axes fraction"),
            xytext=(-3 if right else 3, -3),
            textcoords="offset points",
            ha="right" if right else "left",
            va="top",
            fontsize=8,
            color="0.35",
        )


def figure(xlabel, ylabel):
    """A single-panel figure with the axes already labelled."""
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    return fig, ax


def save(fig, dst):
    fig.tight_layout()
    fig.savefig(dst, dpi=150)
    plt.close(fig)
    print(f"  {dst.name}")


def moving(active):
    """Rows where the flight angle exists. At t = 0 the vehicle is at rest and
    `atan2(0, 0)` reads back as θ = 0, which the writer turns into α = 90°; the
    q > 1 Pa mask in `metrics()` drops the same row."""
    return active[active.V > 0]


def alpha_limit(df):
    """The |α| cap in force at each row, as `metrics()` applies it: the subsonic
    limit up to Ma 1.1, the supersonic one above it, and none once the row is out
    of the air — below 1 Pa of head, or past the atmosphere boundary."""
    lim = np.where(df.Ma > M_SUB, LIMITS["alpha_sup"], LIMITS["alpha_sub"])
    inside = (df.q > 1) & ((df.Ma <= M_SUB) | (df.h <= H_ATM))
    return np.where(inside, lim, np.nan)


def plot_pitch(active, sep, dst):
    active = moving(active)
    fig, ax = figure("t, с", "угол, град")
    ax.plot(active.t, active.pitch * R2D, color="C0", lw=1.4, label="ϑ — тангаж")
    ax.plot(
        active.t,
        active.flightAngle * R2D,
        color="C1",
        lw=1.4,
        label="θ — наклон траектории",
    )
    mark(ax, sep)
    ax.legend(loc="best", fontsize="small")
    save(fig, dst)


def plot_program(active, sep, pitch, dst):
    """The programmed pitch on its own, with the arcs it is built from marked.

    The curve is the `pitch` column of the flown CSV: on a controlled stage
    `cmd/main` writes the program itself, so the cubics need no re-evaluation
    here. The joints come from the config, where every arc carries its own end
    and the one before it supplies the start.
    """
    knots = [(pitch["t_start"], pitch["theta_deg_start"])] + [
        (s["t_end"], s["theta_deg"]) for s in pitch["segments"]
    ]
    fig, ax = figure("t, с", "ϑ, град")
    ax.plot(active.t, active.pitch * R2D, color="C0", lw=1.4, label="ϑ — программа")
    for t, _ in knots:
        ax.axvline(t, color="0.7", lw=0.7, zorder=0)
    ax.plot(
        *zip(*knots),
        ls="none",
        marker="o",
        color="C1",
        ms=4,
        zorder=3,
        label=f"границы участков ({len(pitch['segments'])} шт.)",
    )
    mark(ax, sep)
    ax.legend(loc="best", fontsize="small")
    save(fig, dst)


def plot_alpha(active, sep, dst):
    active = moving(active)
    lim = alpha_limit(active)
    fig, ax = figure("t, с", "α, град")
    ax.plot(active.t, active.attack * R2D, color="C0", lw=1.4, label="α")
    ax.plot(active.t, lim, color="C3", ls=":", lw=1.2, label="предел |α|")
    ax.plot(active.t, -lim, color="C3", ls=":", lw=1.2)
    # a separation is only held to alpha_sep while it is still an aerodynamic one
    aero = sep[(sep.h <= H_ATM) & (sep.q >= Q_SEP)]
    if len(aero):
        ax.plot(
            aero.t,
            aero.attack * R2D,
            "o",
            color="C3",
            ms=5,
            label=f"отделение, |α| ≤ {LIMITS['alpha_sep']:g}°",
        )
    mark(ax, sep)
    ax.legend(loc="best", fontsize="small")
    save(fig, dst)


def plot_mass(active, sep, dst):
    fig, ax = figure("t, с", "m, кг")
    ax.plot(active.t, active.m, color="C0", lw=1.4)
    mark(ax, sep)
    save(fig, dst)


def plot_altitude(df, sep, dst):
    fig, ax = figure("t, с", "h, км")
    ax.plot(df.t, df.h / 1000, color="C0", lw=1.4, label="h")
    ax.axhline(
        LIMITS["apogee"],
        color="C3",
        ls=":",
        lw=1.2,
        label=f"предел h ≤ {LIMITS['apogee']:g} км",
    )
    mark(ax, sep)
    ax.legend(loc="best", fontsize="small")
    save(fig, dst)


def plot_velocity(df, sep, dst):
    fig, ax = figure("t, с", "V, м/с")
    ax.plot(df.t, df.V, color="C0", lw=1.4)
    mark(ax, sep)
    save(fig, dst)


def plot_trajectory(df, sep, dst):
    """y(x) with the Earth drawn.

    The state is planar with the launch point at the origin and the Earth's
    centre at (0, −Rz), so the globe is that circle and the flight path needs no
    transform to sit on it.
    """
    x, y = df.x / 1000, df.y / 1000
    rz = RZ / 1000
    a = np.linspace(0, 2 * np.pi, 721)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.fill(
        rz * np.sin(a), rz * np.cos(a) - rz, color="#dbe7f3", zorder=0, label="Земля"
    )
    ax.plot(rz * np.sin(a), rz * np.cos(a) - rz, color="0.5", lw=0.8, zorder=1)
    ax.plot(x, y, color="C0", lw=1.5, zorder=2, label="траектория")
    ax.plot(x.iloc[0], y.iloc[0], "^", color="C2", ms=8, zorder=3, label="старт")
    ax.plot(x.iloc[-1], y.iloc[-1], "v", color="C3", ms=8, zorder=3, label="падение")
    ax.plot(
        sep.x / 1000,
        sep.y / 1000,
        "o",
        color="C1",
        ms=5,
        zorder=3,
        label="отделение ступеней",
    )
    for n, (sx, sy) in enumerate(zip(sep.x / 1000, sep.y / 1000)):
        name = ROMAN[n] if n < len(ROMAN) else str(n + 1)
        ax.annotate(
            name,
            (sx, sy),
            xytext=(7, 5),
            textcoords="offset points",
            fontsize=8,
            color="C1",
        )

    pad = 0.05 * max(x.max() - x.min(), y.max() - y.min())
    ax.set_xlim(x.min() - pad, x.max() + pad)
    ax.set_ylim(y.min() - pad, y.max() + pad)
    ax.set_aspect("equal")
    ax.set_xlabel("x, км")
    ax.set_ylabel("y, км")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize="small")
    save(fig, dst)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--csv", type=Path, default=OUT / "best.csv", help="flown trajectory"
    )
    ap.add_argument("--json", type=Path, default=OUT / "best.json", help="its config")
    ap.add_argument("--plots", type=Path, default=OUT / "plots", help="output dir")
    args = ap.parse_args()

    for path in (args.csv, args.json):
        if not path.exists():
            sys.exit(f"{path} not found, run: uv run python optimizer/main.py")

    df = pd.read_csv(args.csv)
    config = json.loads(args.json.read_text())
    powered = [i for i, st in enumerate(config["stages"]) if st["powered"]]
    active = df[df.stage.isin(powered)]
    sep = separations(df)

    args.plots.mkdir(parents=True, exist_ok=True)
    plot_pitch(active, sep, args.plots / "pitch.png")
    plot_program(active, sep, config["pitch"], args.plots / "program.png")
    plot_alpha(active, sep, args.plots / "alpha.png")
    plot_mass(active, sep, args.plots / "mass.png")
    plot_altitude(df, sep, args.plots / "altitude.png")
    plot_velocity(df, sep, args.plots / "velocity.png")
    plot_trajectory(df, sep, args.plots / "trajectory.png")
    print(f"\n7 figures -> {args.plots}")


if __name__ == "__main__":
    main()
