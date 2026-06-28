"""CMA-ES optimizer for the configured trajectory pitch program.

Runs the Go simulator (traj/main) as a black box and tunes the pitch program so
the simulated surface range hits a target (default 12000 km) while respecting the
§4.4 constraints, which enter the objective as penalty terms.

The base rocket (stage masses/thrust and the per-stage arc *shapes*) is read from
rocket.json; the optimizer varies all configured pitch-arc terminal angles, the
vertical-hold t_в [s], t_end for non-final arcs, and all per-arc shape exponents.
It writes a temporary config per evaluation. Switch an arc's "shape" in
rocket.json to optimize that law for the arc.

Usage:
    uv run python traj/optimize.py [--target 12000] [--maxiter 150]
"""

import argparse
import copy
import json
import os
import subprocess
import tempfile
from pathlib import Path

import cma

HERE = Path(__file__).parent
BIN = HERE / "out" / "traj-sim"
BASE_CONFIG = HERE / "rocket.json"

# Objective weights. SCALE_L sets the range-error scale (km); a miss of SCALE_L
# costs 1. W_CON makes any constraint violation dominate once the range is close.
SCALE_L = 100.0
W_CON = 1000.0
W_MONO = 10.0
W_TIME = 1000.0
DEFAULT_K_EXP = 3.0
DEFAULT_K_COS = 1.1

with open(BASE_CONFIG) as _f:
    _BASE = json.load(_f)


def build() -> None:
    subprocess.run(["go", "build", "-o", str(BIN), "./main"], cwd=HERE, check=True)


def arc_shape(arc: dict) -> str:
    return arc.get("shape") or "exp"


def arc_k(arc: dict) -> float:
    if "k" in arc:
        return arc["k"]
    return DEFAULT_K_COS if arc_shape(arc) == "cos" else DEFAULT_K_EXP


def arc_specs() -> list[dict]:
    """Return flattened arc metadata in the same order used by config parsing."""
    specs = []
    stage_start = 0.0
    for si, st in enumerate(_BASE["stages"]):
        stage_end = stage_start + st["burn_time"]
        for ai, _arc in enumerate(st["pitch"]):
            specs.append(
                {
                    "stage": si,
                    "arc": ai,
                    "stage_start": stage_start,
                    "stage_end": stage_end,
                    "is_final": ai == len(st["pitch"]) - 1,
                }
            )
        stage_start = stage_end
    return specs


def split_specs() -> list[dict]:
    return [sp for sp in arc_specs() if not sp["is_final"]]


def vector_dim() -> int:
    n_arcs = len(arc_specs())
    return n_arcs + 1 + len(split_specs()) + n_arcs


def default_x0() -> list[float]:
    specs = arc_specs()
    splits = split_specs()
    angles = []
    split_times = []
    ks = []
    for sp in specs:
        arc = _BASE["stages"][sp["stage"]]["pitch"][sp["arc"]]
        angles.append(arc["theta_deg"])
        ks.append(arc_k(arc))
        if not sp["is_final"]:
            split_times.append(arc.get("t_end", (sp["stage_start"] + sp["stage_end"]) / 2))
    if len(split_times) != len(splits):
        raise RuntimeError("internal optimizer layout mismatch")
    return [*angles, _BASE["t_vertical"], *split_times, *ks]


def unpack_x(x):
    specs = arc_specs()
    splits = split_specs()
    n_arcs = len(specs)
    n_splits = len(splits)
    if len(x) != n_arcs + 1 + n_splits + n_arcs:
        raise ValueError(
            f"expected {n_arcs + 1 + n_splits + n_arcs} optimizer values, got {len(x)}"
        )
    angles = list(x[:n_arcs])
    t_vertical = float(x[n_arcs])
    split_times = list(x[n_arcs + 1 : n_arcs + 1 + n_splits])
    ks = list(x[n_arcs + 1 + n_splits :])
    return specs, splits, angles, t_vertical, split_times, ks


def config_from_x(x) -> dict:
    """Map the CMA-ES vector onto a rocket config (arc shapes kept from base).
    """
    specs, splits, angles, t_vertical, split_times, ks = unpack_x(x)
    c = copy.deepcopy(_BASE)
    c["t_vertical"] = t_vertical
    for sp, theta, k in zip(specs, angles, ks):
        c["stages"][sp["stage"]]["pitch"][sp["arc"]].update(
            {"theta_deg": theta, "k": k}
        )
    for sp, t_end in zip(splits, split_times):
        c["stages"][sp["stage"]]["pitch"][sp["arc"]]["t_end"] = t_end
    return c


def k_lower_bounds() -> list[float]:
    lows = []
    for sp in arc_specs():
        arc = _BASE["stages"][sp["stage"]]["pitch"][sp["arc"]]
        lows.append(1.0 if arc_shape(arc) == "cos" else -8.0)
    return lows


def bounds() -> list[list[float]]:
    specs = arc_specs()
    splits = split_specs()
    n_arcs = len(specs)
    angle_low = [5.0] * n_arcs
    angle_high = [89.0] * n_arcs
    split_low = [sp["stage_start"] + 1.0 for sp in splits]
    split_high = [sp["stage_end"] - 0.1 for sp in splits]
    return [
        [*angle_low, 5.0, *split_low, *k_lower_bounds()],
        [*angle_high, 40.0, *split_high, *([8.0] * n_arcs)],
    ]


def cma_stds() -> list[float]:
    n_arcs = len(arc_specs())
    return [*([10.0] * n_arcs), 8.0, *([8.0] * len(split_specs())), *([3.0] * n_arcs)]


def end_times_from_x(x) -> list[float]:
    specs, splits, _angles, _t_vertical, split_times, _ks = unpack_x(x)
    split_by_ref = {
        (sp["stage"], sp["arc"]): t_end for sp, t_end in zip(splits, split_times)
    }
    return [
        sp["stage_end"] if sp["is_final"] else split_by_ref[(sp["stage"], sp["arc"])]
        for sp in specs
    ]


def program_penalty(x) -> float:
    specs, _splits, angles, t_vertical, _split_times, _ks = unpack_x(x)
    ends = end_times_from_x(x)
    f = 0.0

    prev = t_vertical
    for end in ends:
        f += W_TIME * max(0.0, (prev + 0.1 - end) / 10.0) ** 2
        prev = end

    # Keep the program physical: expect pitch terminal angles to decrease with
    # time across the flattened powered program.
    f += W_MONO * sum(
        max(0.0, angles[i + 1] - angles[i]) ** 2 for i in range(len(angles) - 1)
    )
    return f


def format_params(x) -> str:
    specs, splits, angles, t_vertical, split_times, ks = unpack_x(x)
    split_by_ref = {
        (sp["stage"], sp["arc"]): t_end for sp, t_end in zip(splits, split_times)
    }
    lines = [f"t_в={t_vertical:.3f} s"]
    for sp, theta, k in zip(specs, angles, ks):
        shape = arc_shape(_BASE["stages"][sp["stage"]]["pitch"][sp["arc"]])
        t_end = ""
        if not sp["is_final"]:
            t_end = f", t_end={split_by_ref[(sp['stage'], sp['arc'])]:.3f} s"
        lines.append(
            f"s{sp['stage'] + 1}a{sp['arc'] + 1}: ϑ={theta:.3f} deg, "
            f"shape={shape}, k={k:.3f}{t_end}"
        )
    return "\n  ".join(lines)


def run_sim(x, h, metrics=True, out=None):
    cfg = config_from_x(x)
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, dir=HERE) as f:
        json.dump(cfg, f)
        cfg_path = f.name
    try:
        cmd = [str(BIN), f"-config={cfg_path}", f"-h={h}"]
        if metrics:
            cmd.append("-metrics")
        if out is not None:
            cmd.append(f"-out={out}")
        return subprocess.run(cmd, cwd=HERE, capture_output=True, text=True, check=True)
    finally:
        os.unlink(cfg_path)


def metrics(x, h) -> dict:
    res = run_sim(x, h, metrics=True)
    return json.loads(res.stdout.strip().splitlines()[-1])


def objective(x, target_km, h) -> float:
    try:
        f_prog = program_penalty(x)
    except ValueError:
        return 1e9
    try:
        m = metrics(x, h)
    except (subprocess.CalledProcessError, json.JSONDecodeError, IndexError):
        return 1e9 + f_prog  # infeasible / failed run

    f = f_prog + ((m["impact_range_km"] - target_km) / SCALE_L) ** 2

    def pen(val, lim):
        return max(0.0, (val - lim) / lim) ** 2

    f += W_CON * pen(m["max_alpha_sub_deg"], m["lim_eps1"])
    f += W_CON * pen(m["max_alpha_sup_deg"], m["lim_eps2"])
    f += W_CON * pen(m["max_pitch_rate_dps"], m["lim_theta_dot"])
    f += W_CON * pen(m["max_q_pa"], m["lim_qmax"])
    return f


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--target", type=float, default=12000.0, help="target surface range [km]"
    )
    ap.add_argument("--maxiter", type=int, default=150, help="max CMA-ES iterations")
    ap.add_argument(
        "--sigma0", type=float, default=1.0, help="global CMA-ES step multiplier"
    )
    ap.add_argument(
        "--h-opt", type=float, default=0.5, help="integration step during search [s]"
    )
    ap.add_argument(
        "--h-final",
        type=float,
        default=0.1,
        help="integration step for the final run [s]",
    )
    ap.add_argument(
        "--x0",
        type=float,
        nargs="*",
        default=None,
        help=(
            "initial vector: all arc terminal angles [deg], t_в [s], t_end for "
            "non-final arcs [s], then all arc k values; defaults from rocket.json"
        ),
    )
    args = ap.parse_args()

    build()

    x0 = args.x0 if args.x0 is not None else default_x0()
    expected = vector_dim()
    if len(x0) != expected:
        raise SystemExit(f"--x0 must contain {expected} values for this config, got {len(x0)}")

    # Per-dimension steps: angles [deg], t_в [s], k [-]. sigma0 scales all.
    opts = {
        "bounds": bounds(),
        "CMA_stds": cma_stds(),
        "maxiter": args.maxiter,
        "verb_disp": 10,
    }
    es = cma.CMAEvolutionStrategy(x0, args.sigma0, opts)
    es.optimize(lambda x: objective(x, args.target, args.h_opt))

    best = es.result.xbest
    print(f"\nbest params:\n  {format_params(best)}")
    m = metrics(best, args.h_final)
    print(
        f"impact range     : {m['impact_range_km']:.1f} km (target {args.target:.0f})"
    )
    print(
        "constraints      : "
        f"|α|sub={m['max_alpha_sub_deg']:.2f}/{m['lim_eps1']:.0f} "
        f"|α|sup={m['max_alpha_sup_deg']:.2f}/{m['lim_eps2']:.0f} "
        f"ϑ̇={m['max_pitch_rate_dps']:.2f}/{m['lim_theta_dot']:.0f} "
        f"q={m['max_q_pa'] / 1000:.1f}/{m['lim_qmax'] / 1000:.0f} kPa"
    )

    # Persist the best config and print a reproducible run command.
    best_path = HERE / "out" / "best.json"
    best_path.parent.mkdir(exist_ok=True)
    with open(best_path, "w") as f:
        json.dump(config_from_x(best), f, indent=2)
    print(f"\nwrote best config -> {best_path}")
    print(
        f"run command:\n  ./out/traj-sim -config=out/best.json -h={args.h_final} -out=out/traj.csv"
    )

    # Final fine-step run: writes out/traj.csv and prints the full diagnostics.
    print("\n=== final run (fine step) ===")
    res = run_sim(best, args.h_final, metrics=False, out="out/traj.csv")
    print(res.stdout, end="")


if __name__ == "__main__":
    main()
