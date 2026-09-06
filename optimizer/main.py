"""CMA-ES search of the pitch program for trajectory/cmd/main.

Stages come from optimizer/input/rocket.json (written by report/main.py), the starting
pitch block from seed.json (any config with a pitch block, e.g. a previous
best.json). The program is tuned so the range hits --target with the
§4.4 limits as penalties. The result goes to optimizer/out/best.json, which the
simulator can fly as is.
"""

import argparse
import json
import math
import os
import random
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from itertools import pairwise
from pathlib import Path

import cma
import numpy as np
import pandas as pd

HERE = Path(__file__).parent
ROOT = HERE.parent

SCALE_L = 100.0  # km of range miss that costs 1
W_CON = 1e7  # a limit violation must outweigh any range miss
CON_MARGIN = 1e-3  # converge just inside the limits, not on them
F_FAIL = 1e15  # a run that did not finish, worse than any real penalty
W_MONO = 10.0  # pitch should decrease arc to arc
RZ = 6371000.0  # trajectory/constants.go
H_ATM = 94000.0
Q_SEP = 1000.0  # Pa; below this a separation is not an aerodynamic event
# deg, deg, deg, deg/s, kPa, km
LIMITS = {
    "alpha_sub": 1.5,
    "alpha_sup": 10.0,
    "alpha_sep": 1.5,
    "pitch_rate": 3.0,
    "q_max": 120.0,
    "apogee": 1800.0,
}

# vector: [theta_deg x N | w x N-1 | k x N | t_start]; the arcs tile
# [t_start, t_powered] in proportion to w, the last arc's weight pinned at 1
BOUNDS = {"theta": (5, 89), "w": (0.1, 10), "k": (1, 8), "t_start": (5, 40)}
STDS = {"theta": 10, "w": 0.5, "k": 2, "t_start": 5}


def per_block(n, table):
    return (
        [table["theta"]] * n
        + [table["w"]] * (n - 1)
        + [table["k"]] * n
        + [table["t_start"]]
    )


def x0_from_seed(seed):
    segs = seed["segments"]
    dt = np.diff([seed["t_start"]] + [s["t_end"] for s in segs])
    return (
        [s["theta_deg"] for s in segs]
        + list(dt[:-1] / dt[-1])  # durations enter as ratios to the last arc
        + [s["k"] for s in segs]
        + [seed["t_start"]]
    )


def resample(segs, t_start, t_powered, m):
    """Put the program on m evenly spaced arcs, reading theta and k off the old ones."""
    t = [s["t_end"] for s in segs]
    return [
        {
            "t_end": float(tt),
            "shape": "cos",
            "k": float(np.interp(tt, t, [s["k"] for s in segs])),
            "theta_deg": float(np.interp(tt, t, [s["theta_deg"] for s in segs])),
        }
        for tt in np.linspace(t_start, t_powered, m + 1)[1:]
    ]


def pitch_from_x(x, theta_start, t_powered):
    n = len(x) // 3
    theta, w, k, t_start = x[:n], x[n : 2 * n - 1], x[2 * n - 1 : 3 * n - 1], x[-1]
    # the program steers nothing once the last stage burns out, so the arcs tile the
    # powered window instead of drifting past it. The last weight is pinned at 1
    # because only the ratios matter and a free scale is a flat direction CMA-ES
    # would random-walk into the bounds.
    w = np.append(w, 1.0)
    t_end = t_start + (t_powered - t_start) * np.cumsum(w) / np.sum(w)
    return {
        "theta_deg_start": theta_start,
        "t_start": float(t_start),
        "segments": [
            {"t_end": float(t), "shape": "cos", "k": float(kk), "theta_deg": float(th)}
            for t, kk, th in zip(t_end, k, theta)
        ],
    }


def run_sim(binary, aero, config, out_dir, csv=None):
    with tempfile.TemporaryDirectory(dir=out_dir) as tmp:
        cfg = Path(tmp) / "rocket.json"
        cfg.write_text(json.dumps(config))
        csv = csv or Path(tmp) / "traj.csv"
        # the integrator only stops at H = 0, so an orbital candidate never returns
        subprocess.run(
            [binary, cfg, aero, csv], check=True, capture_output=True, timeout=60
        )
        return pd.read_csv(csv)


def metrics(df, powered):
    last = df.iloc[-1]
    pw = df[df.stage.isin(powered)]
    air = pw[pw.q > 1]  # at t = 0 the flight angle is undefined
    alpha = air.attack.abs() * 180 / math.pi
    # one row per staging event. Take it from df, not pw: cmd/main resolves the stage
    # from the right, so the row at a separation already carries the new stage.
    sep = df[df.stage.diff() > 0]
    sep = sep[(sep.h <= H_ATM) & (sep.q >= Q_SEP)]
    # a staging instant is emitted twice, so drop the zero-length gaps rather than
    # relying on the writer to deduplicate them
    dt = np.diff(pw.t)
    ok = dt > 1e-9
    rate = np.abs(np.diff(pw.pitch))[ok] / dt[ok] * 180 / math.pi
    return {
        "range_km": RZ * math.atan2(last.x, RZ + last.y) / 1000,
        "apogee": df.h.max() / 1000,
        "q_max": pw.q.max() / 1000,
        "alpha_sub": alpha.where(air.Ma <= 1.1, 0).max(),
        "alpha_sup": alpha.where((air.Ma > 1.1) & (air.h <= H_ATM), 0).max(),
        "alpha_sep": (sep.attack.abs() * 180 / math.pi).max() if len(sep) else 0.0,
        "pitch_rate": rate.max(),
    }


def penalty(m):
    f = 0.0
    for key, lim in LIMITS.items():
        f += max(0.0, (m[key] - lim * (1 - CON_MARGIN)) / lim) ** 2
    return W_CON * f


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--target", type=float, default=12000, help="range to hit [km]")
    ap.add_argument("--rocket", type=Path, default=HERE / "input/rocket.json")
    ap.add_argument("--aero", type=Path, default=ROOT / "openfoam/out/averages.csv")
    ap.add_argument(
        "--seed",
        type=Path,
        default=HERE / "seed.json",
        help="config whose pitch block to start from",
    )
    ap.add_argument(
        "--arcs", type=int, help="resample the seed program onto this many arcs"
    )
    ap.add_argument("--out", type=Path, default=HERE / "out")
    ap.add_argument("--maxiter", type=int, default=200)
    ap.add_argument(
        "--popsize", type=int, help="CMA-ES population (default 4 + 3 ln n)"
    )
    ap.add_argument("--sigma0", type=float, default=1.0)
    ap.add_argument("--rng-seed", type=int, default=random.randint(0, 10000000))
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    args = ap.parse_args()

    if args.arcs is not None and args.arcs < 2:
        sys.exit("--arcs needs at least 2")

    if not args.rocket.exists():
        sys.exit(
            f"{args.rocket} not found, run: uv run python report/main.py --write-traj-config"
        )
    stages = json.loads(args.rocket.read_text())["stages"]
    powered = [i for i, s in enumerate(stages) if s["powered"]]
    # through the last powered stage, so an intermediate coast still counts
    t_powered = sum(s["burn_time"] for s in stages[: powered[-1] + 1])
    seed = json.loads(args.seed.read_text())["pitch"]
    theta_start = seed["theta_deg_start"]
    segs = seed["segments"]
    # Arcs starting after burnout steer nothing; keep the one straddling it, the
    # parametrization pulls its end back. Counting arcs that START before burnout
    # is what makes a re-seed lossless -- a program this script wrote ends exactly
    # at t_powered, and a `t_end < t_powered` test would shed one arc every run.
    n = min(sum(s["t_end"] < t_powered for s in segs) + 1, len(segs))
    if n < 2 and not args.arcs:
        sys.exit(f"seed: {n} arc(s) before burnout at {t_powered:g} s, pass --arcs")
    if args.arcs:
        seed["segments"] = resample(segs[:n], seed["t_start"], t_powered, args.arcs)
        n = args.arcs
    else:
        seed["segments"] = segs[:n]
    x0 = x0_from_seed(seed)

    args.out.mkdir(parents=True, exist_ok=True)
    binary = args.out.resolve() / "sim"
    subprocess.run(
        ["go", "build", "-o", binary, "./cmd/main"], cwd=ROOT / "trajectory", check=True
    )

    def evaluate(x):
        config = {"stages": stages, "pitch": pitch_from_x(x, theta_start, t_powered)}
        try:
            m = metrics(run_sim(binary, args.aero, config, args.out), powered)
        except (
            subprocess.CalledProcessError,
            subprocess.TimeoutExpired,
            pd.errors.EmptyDataError,
        ):
            return F_FAIL
        if not all(np.isfinite(list(m.values()))):
            return F_FAIL
        return ((m["range_km"] - args.target) / SCALE_L) ** 2 + penalty(m)

    def objective(x):
        theta = x[:n]
        mono = sum(max(0.0, b - a) ** 2 for a, b in pairwise(theta))
        return evaluate(x) + W_MONO * mono

    try:
        run_sim(binary, args.aero, {"stages": stages, "pitch": seed}, args.out)
    except subprocess.CalledProcessError as e:
        sys.exit(f"seed program does not fly:\n{e.stderr.decode()}")

    opts = {
        "bounds": list(zip(*per_block(n, BOUNDS))),
        "CMA_stds": per_block(n, STDS),
        "maxiter": args.maxiter,
        "seed": args.rng_seed,
        "verb_log": 0,
    }
    if args.popsize:
        opts["popsize"] = args.popsize
    es = cma.CMAEvolutionStrategy(x0, args.sigma0, opts)
    with ThreadPoolExecutor(args.jobs) as pool:
        try:
            while not es.stop():
                X = es.ask()
                es.tell(X, list(pool.map(objective, X)))
                es.disp()
        except KeyboardInterrupt:
            pool.shutdown(cancel_futures=True)
            print("\ninterrupted, keeping the best so far")

    x = es.result.xbest
    if x is None:
        sys.exit("nothing evaluated")
    pitch = pitch_from_x(x, theta_start, t_powered)
    config = {"stages": stages, "pitch": pitch}
    m = metrics(
        run_sim(binary, args.aero, config, args.out, csv=args.out / "best.csv"), powered
    )
    (args.out / "best.json").write_text(
        json.dumps({**config, "target_km": args.target, "metrics": m}, indent=2)
    )

    print(f"\nrange {m['range_km']:.1f} km (target {args.target:.0f})")
    for key, lim in LIMITS.items():
        print(f"{key:11s} {m[key]:8.2f} / {lim:g}")
    print(f"wrote {args.out / 'best.json'}")


if __name__ == "__main__":
    main()
