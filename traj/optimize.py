"""CMA-ES optimizer for the three-stage trajectory pitch program.

Runs the Go simulator (traj/main) as a black box and tunes the pitch program so
the simulated surface range hits a target (default 12000 km) while respecting the
§4.4 constraints, which enter the objective as penalty terms. Stage 1 is split
into two sub-arcs at t_split, giving the search a dedicated handle on the subsonic
α limit. Free parameters: terminal angles ϑ_k11/ϑ_k12/ϑ_k2/ϑ_k3 [deg], the
vertical-hold t_в and split time t_split [s], and the per-arc shape rates
k_exp11/12/2/3. The integrator itself stays in Go.

The base rocket (stage masses/thrust and the per-stage arc *shapes*) is read from
rocket.json; the optimizer only varies the continuous parameters above, writing a
temporary config per evaluation. Switch an arc's "shape" to "cos" in rocket.json
to optimize a cosine program for that stage.

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

with open(BASE_CONFIG) as _f:
    _BASE = json.load(_f)


def build() -> None:
    subprocess.run(["go", "build", "-o", str(BIN), "./main"], cwd=HERE, check=True)


def config_from_x(x) -> dict:
    """Map the CMA-ES vector onto a rocket config (arc shapes kept from base).

    x = [ϑ_k11, ϑ_k12, ϑ_k2, ϑ_k3, t_в, t_split, k_exp11, k_exp12, k_exp2, k_exp3]
    """
    c = copy.deepcopy(_BASE)
    c["t_vertical"] = x[4]
    s1, s2, s3 = c["stages"][0]["pitch"], c["stages"][1]["pitch"], c["stages"][2]["pitch"]
    s1[0].update({"theta_deg": x[0], "t_end": x[5], "k": x[6]})  # stage-1 split sub-arc
    s1[1].update({"theta_deg": x[1], "k": x[7]})                 # stage-1 to burnout
    s2[0].update({"theta_deg": x[2], "k": x[8]})                 # stage-2
    s3[0].update({"theta_deg": x[3], "k": x[9]})                 # stage-3
    return c


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
        m = metrics(x, h)
    except (subprocess.CalledProcessError, json.JSONDecodeError, IndexError):
        return 1e9  # infeasible / failed run

    f = ((m["impact_range_km"] - target_km) / SCALE_L) ** 2

    def pen(val, lim):
        return max(0.0, (val - lim) / lim) ** 2

    f += W_CON * pen(m["max_alpha_sub_deg"], m["lim_eps1"])
    f += W_CON * pen(m["max_alpha_sup_deg"], m["lim_eps2"])
    f += W_CON * pen(m["max_pitch_rate_dps"], m["lim_theta_dot"])
    f += W_CON * pen(m["max_q_pa"], m["lim_qmax"])

    # Keep the program physical: expect ϑ_k11 ≥ ϑ_k12 ≥ ϑ_k2 ≥ ϑ_k3, and the
    # stage-1 split after the vertical hold (t_в < t_split).
    f += W_MONO * (
        max(0.0, x[1] - x[0]) ** 2
        + max(0.0, x[2] - x[1]) ** 2
        + max(0.0, x[3] - x[2]) ** 2
        + max(0.0, x[4] - x[5]) ** 2
    )
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
        nargs=10,
        default=[80.0, 72.0, 46.0, 28.0, 10.0, 25.0, 3.0, 3.0, 3.0, 3.0],
        help="initial ϑ_k11/k12/k2/k3 [deg], t_в/t_split [s], k_exp11/12/2/3",
    )
    args = ap.parse_args()

    build()

    # Per-dimension steps: angles [deg], t_в/t_split [s], k_exp [-]. sigma0 scales all.
    opts = {
        "bounds": [
            [5.0, 5.0, 5.0, 5.0, 5.0, 10.0, -8.0, -8.0, -8.0, -8.0],
            [89.0, 89.0, 89.0, 89.0, 40.0, 60.0, 8.0, 8.0, 8.0, 8.0],
        ],
        "CMA_stds": [10.0, 10.0, 10.0, 10.0, 8.0, 10.0, 3.0, 3.0, 3.0, 3.0],
        "maxiter": args.maxiter,
        "verb_disp": 10,
    }
    es = cma.CMAEvolutionStrategy(args.x0, args.sigma0, opts)
    es.optimize(lambda x: objective(x, args.target, args.h_opt))

    best = es.result.xbest
    print(
        f"\nbest params: ϑ_k11={best[0]:.3f} ϑ_k12={best[1]:.3f} ϑ_k2={best[2]:.3f} ϑ_k3={best[3]:.3f} deg, "
        f"t_в={best[4]:.3f} t_split={best[5]:.3f} s, k_exp={best[6]:.2f}/{best[7]:.2f}/{best[8]:.2f}/{best[9]:.2f}"
    )
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
