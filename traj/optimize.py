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
# costs 1. W_CON makes any constraint violation dominate the range term.
#
# W_CON has to stay far above (miss/SCALE_L)^2. When the target is out of the
# rocket's reach the range term never approaches zero — a 4000 km miss alone
# costs 1.6e3 — and its gradient is roughly constant, so a moderate W_CON lets
# the optimizer buy range by violating the α and q limits. Those are hard §4.4
# design limits, so the weight is set high enough that any violation outweighs
# the whole range term.
#
# A one-sided penalty puts the optimum exactly on the constraint, so it lands
# a hair either side of it. CON_MARGIN shrinks the limits during the search by
# 0.1 % so the converged solution is strictly inside the reported limits.
SCALE_L = 100.0
W_CON = 1.0e7
CON_MARGIN = 1.0e-3
W_MONO = 10.0
W_TIME = 1000.0
# Must mirror defaultKExp/defaultKCos in traj/pitch.go — the Go parser applies
# these same defaults to arcs that leave "k" unset.
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


# The arc layout is fixed by _BASE for the whole run, so resolve it once:
# unpack_x() reads it on every objective evaluation.
SPECS = arc_specs()
SPLITS = [sp for sp in SPECS if not sp["is_final"]]


def vector_dim() -> int:
    """Length of the CMA-ES vector: angles, t_в, non-final t_end values, ks."""
    return 2 * len(SPECS) + 1 + len(SPLITS)


def base_arc(sp: dict) -> dict:
    return _BASE["stages"][sp["stage"]]["pitch"][sp["arc"]]


def default_x0() -> list[float]:
    specs, splits = SPECS, SPLITS
    angles = []
    split_times = []
    ks = []
    for sp in specs:
        arc = base_arc(sp)
        angles.append(arc["theta_deg"])
        ks.append(arc_k(arc))
        if not sp["is_final"]:
            split_times.append(
                arc.get("t_end", (sp["stage_start"] + sp["stage_end"]) / 2)
            )
    if len(split_times) != len(splits):
        raise RuntimeError("internal optimizer layout mismatch")
    return [*angles, _BASE["t_vertical"], *split_times, *ks]


def unpack_x(x):
    specs, splits = SPECS, SPLITS
    n_arcs = len(specs)
    n_splits = len(splits)
    if len(x) != vector_dim():
        raise ValueError(f"expected {vector_dim()} optimizer values, got {len(x)}")
    angles = list(x[:n_arcs])
    t_vertical = float(x[n_arcs])
    split_times = list(x[n_arcs + 1 : n_arcs + 1 + n_splits])
    ks = list(x[n_arcs + 1 + n_splits :])
    return specs, splits, angles, t_vertical, split_times, ks


def config_from_x(x) -> dict:
    """Map the CMA-ES vector onto a rocket config (arc shapes kept from base)."""
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
    return [1.0 if arc_shape(base_arc(sp)) == "cos" else -8.0 for sp in SPECS]


def bounds() -> list[list[float]]:
    n_arcs = len(SPECS)
    split_low = [sp["stage_start"] + 1.0 for sp in SPLITS]
    split_high = [sp["stage_end"] - 0.1 for sp in SPLITS]
    return [
        [*([5.0] * n_arcs), 5.0, *split_low, *k_lower_bounds()],
        [*([89.0] * n_arcs), 40.0, *split_high, *([8.0] * n_arcs)],
    ]


def cma_stds() -> list[float]:
    n_arcs = len(SPECS)
    return [*([10.0] * n_arcs), 8.0, *([8.0] * len(SPLITS)), *([3.0] * n_arcs)]


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
    _specs, _splits, angles, t_vertical, _split_times, _ks = unpack_x(x)
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
        shape = arc_shape(base_arc(sp))
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
    f_prog = program_penalty(x)
    try:
        m = metrics(x, h)
    except (subprocess.CalledProcessError, json.JSONDecodeError, IndexError):
        return 1e9 + f_prog  # infeasible / failed run

    f = f_prog + ((m["impact_range_km"] - target_km) / SCALE_L) ** 2

    def pen(val, lim):
        return max(0.0, (val - lim * (1.0 - CON_MARGIN)) / lim) ** 2

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
    ap.add_argument("--seed", type=int, default=20260805, help="CMA-ES RNG seed")
    # Search and verification must use the same step: at h=0.5 the α peaks read
    # ~0.1 deg low, which is enough to make a solution that looks feasible
    # during the search violate the limits at h=0.1.
    ap.add_argument(
        "--h-opt", type=float, default=0.1, help="integration step during search [s]"
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
        "seed": args.seed,
    }
    es = cma.CMAEvolutionStrategy(x0, args.sigma0, opts)
    es.optimize(lambda x: objective(x, args.target, args.h_opt))

    best = es.result.xbest
    print(f"\nbest params:\n  {format_params(best)}")

    # Persist the best config before anything that can fail, so a broken final
    # run never discards the whole search.
    best_path = HERE / "out" / "best.json"
    best_path.parent.mkdir(exist_ok=True)
    with open(best_path, "w") as f:
        json.dump(config_from_x(best), f, indent=2)
    print(f"wrote best config -> {best_path}")
    print(
        f"run command:\n  ./out/traj-sim -config=out/best.json -h={args.h_final} -out=out/traj.csv"
    )

    try:
        m = metrics(best, args.h_final)
    except (subprocess.CalledProcessError, json.JSONDecodeError, ValueError) as exc:
        print(f"\nfinal metrics run failed: {exc}")
        return
    miss = m["impact_range_km"] - args.target
    print(
        f"\nimpact range     : {m['impact_range_km']:.1f} km "
        f"(target {args.target:.0f}, miss {miss:+.1f})"
    )
    print(
        "constraints      : "
        f"|α|sub={m['max_alpha_sub_deg']:.2f}/{m['lim_eps1']:.2f} "
        f"|α|sup={m['max_alpha_sup_deg']:.2f}/{m['lim_eps2']:.2f} "
        f"ϑ̇={m['max_pitch_rate_dps']:.2f}/{m['lim_theta_dot']:.2f} "
        f"q={m['max_q_pa'] / 1000:.1f}/{m['lim_qmax'] / 1000:.0f} kPa"
    )

    # Final fine-step run: writes out/traj.csv and prints the full diagnostics.
    print("\n=== final run (fine step) ===")
    res = run_sim(best, args.h_final, metrics=False, out="out/traj.csv")
    print(res.stdout, end="")


if __name__ == "__main__":
    main()
