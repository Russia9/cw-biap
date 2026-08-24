"""CMA-ES optimizer for the configured trajectory pitch program.

Runs the Go simulator (traj/main) as a black box and tunes the pitch program so
the simulated surface range hits a target (default 12000 km) while respecting the
§4.4 constraints, which enter the objective as penalty terms.

The base rocket (stage masses/thrust, the per-stage arc *shapes* and each
stage's steering frame) is read from rocket.json; the optimizer varies all
configured pitch-arc terminal values, the vertical-hold t_в [s], t_end for
non-final arcs, all per-arc shape exponents, and any frame-change entry angles.
It writes a temporary config per evaluation. Switch an arc's "shape" in
rocket.json to optimize that law for the arc, or a stage's "steering" to switch
that stage between ϑ-framed and α-framed arcs.

Usage:
    uv run python traj/optimize.py [--target 12000] [--maxiter 150]
"""

import argparse
import copy
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import cma

HERE = Path(__file__).parent
BIN = HERE / "out" / "traj-sim"


def _base_config_path() -> Path:
    """Resolve --config before argparse runs.

    The arc layout (SPECS/SPLITS/ENTRIES below) is derived at import time from
    the base config, so the path has to be known before main() parses argv.
    """
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == "--config" and i + 1 < len(argv):
            return Path(argv[i + 1])
        if a.startswith("--config="):
            return Path(a.split("=", 1)[1])
    return HERE / "rocket.json"


BASE_CONFIG = _base_config_path()

# CFD coefficient table passed to every simulator call, set from --aero. Empty
# means the simulator runs with zero aerodynamics, which is the default.
AERO = ""

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
#
# F_FAIL scores a run the simulator could not complete. CMA-ES ranks the
# population rather than reading the values, so this only has to sort below
# every real evaluation — and a real one is not bounded: W_CON times a large
# relative excess reaches 1e10 easily. A sentinel inside that range would make
# the search prefer configs that fail outright over configs that merely violate
# a limit, which is how a plain 1e9 stalled the α runs at their starting point.
SCALE_L = 100.0
W_CON = 1.0e7
F_FAIL = 1.0e15
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


def stage_frame(stage: dict) -> str:
    """Steering frame of a stage: "theta" (arcs are ϑ) or "alpha" (arcs are α)."""
    return stage.get("steering") or "theta"


def angle_key(frame: str) -> str:
    return "alpha_deg" if frame == "alpha" else "theta_deg"


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
                    "frame": stage_frame(st),
                }
            )
        stage_start = stage_end
    return specs


# The arc layout is fixed by _BASE for the whole run, so resolve it once:
# unpack_x() reads it on every objective evaluation.
SPECS = arc_specs()
SPLITS = [sp for sp in SPECS if not sp["is_final"]]


def base_arc(sp: dict) -> dict:
    return _BASE["stages"][sp["stage"]]["pitch"][sp["arc"]]


# Arcs carrying an explicit entry pitch, which a stage needs when it switches
# steering frame. Their continuity with the preceding arc is not structural, so
# the objective penalises the resulting ϑ jump via max_pitch_rate_num_dps.
ENTRIES = [sp for sp in SPECS if "theta0_deg" in base_arc(sp)]


def vector_dim() -> int:
    """Angles, t_в, non-final t_end values, ks, then frame-change entry angles."""
    return 2 * len(SPECS) + 1 + len(SPLITS) + len(ENTRIES)


def default_x0() -> list[float]:
    specs, splits = SPECS, SPLITS
    angles = []
    split_times = []
    ks = []
    for sp in specs:
        arc = base_arc(sp)
        angles.append(arc[angle_key(sp["frame"])])
        ks.append(arc_k(arc))
        if not sp["is_final"]:
            split_times.append(
                arc.get("t_end", (sp["stage_start"] + sp["stage_end"]) / 2)
            )
    if len(split_times) != len(splits):
        raise RuntimeError("internal optimizer layout mismatch")
    entries = [base_arc(sp)["theta0_deg"] for sp in ENTRIES]
    return [*angles, _BASE["t_vertical"], *split_times, *ks, *entries]


def unpack_x(x):
    specs, splits = SPECS, SPLITS
    n_arcs = len(specs)
    n_splits = len(splits)
    if len(x) != vector_dim():
        raise ValueError(f"expected {vector_dim()} optimizer values, got {len(x)}")
    angles = list(x[:n_arcs])
    t_vertical = float(x[n_arcs])
    split_times = list(x[n_arcs + 1 : n_arcs + 1 + n_splits])
    ks = list(x[n_arcs + 1 + n_splits : 2 * n_arcs + 1 + n_splits])
    entries = list(x[2 * n_arcs + 1 + n_splits :])
    return specs, splits, angles, t_vertical, split_times, ks, entries


def config_from_x(x) -> dict:
    """Map the CMA-ES vector onto a rocket config (arc shapes kept from base)."""
    specs, splits, angles, t_vertical, split_times, ks, entries = unpack_x(x)
    c = copy.deepcopy(_BASE)
    c["t_vertical"] = t_vertical
    for sp, angle, k in zip(specs, angles, ks):
        c["stages"][sp["stage"]]["pitch"][sp["arc"]].update(
            {angle_key(sp["frame"]): angle, "k": k}
        )
    for sp, t_end in zip(splits, split_times):
        c["stages"][sp["stage"]]["pitch"][sp["arc"]]["t_end"] = t_end
    for sp, entry in zip(ENTRIES, entries):
        c["stages"][sp["stage"]]["pitch"][sp["arc"]]["theta0_deg"] = entry
    return c


def k_lower_bounds() -> list[float]:
    return [1.0 if arc_shape(base_arc(sp)) == "cos" else -8.0 for sp in SPECS]


# Angle bounds per steering frame. ϑ arcs sweep the whole powered turn; α arcs
# only ever need to reach a little past the §4.4 supersonic limit of 10 deg, and
# a small positive α must stay reachable for the pitch-over.
ANGLE_BOUNDS = {"theta": (5.0, 89.0), "alpha": (-12.0, 3.0)}


def angle_bounds() -> tuple[list[float], list[float]]:
    lo, hi = zip(*(ANGLE_BOUNDS[sp["frame"]] for sp in SPECS))
    return list(lo), list(hi)


def bounds() -> list[list[float]]:
    n_arcs = len(SPECS)
    n_entries = len(ENTRIES)
    angle_low, angle_high = angle_bounds()
    split_low = [sp["stage_start"] + 1.0 for sp in SPLITS]
    split_high = [sp["stage_end"] - 0.1 for sp in SPLITS]
    return [
        [*angle_low, 5.0, *split_low, *k_lower_bounds(), *([0.0] * n_entries)],
        [*angle_high, 40.0, *split_high, *([8.0] * n_arcs), *([89.0] * n_entries)],
    ]


def cma_stds() -> list[float]:
    n_arcs = len(SPECS)
    return [
        *([10.0] * n_arcs),
        8.0,
        *([8.0] * len(SPLITS)),
        *([3.0] * n_arcs),
        *([8.0] * len(ENTRIES)),
    ]


def end_times_from_x(x) -> list[float]:
    specs, splits, _angles, _t_vertical, split_times, _ks, _entries = unpack_x(x)
    split_by_ref = {
        (sp["stage"], sp["arc"]): t_end for sp, t_end in zip(splits, split_times)
    }
    return [
        sp["stage_end"] if sp["is_final"] else split_by_ref[(sp["stage"], sp["arc"])]
        for sp in specs
    ]


def program_penalty(x) -> float:
    specs, _splits, angles, t_vertical, _split_times, _ks, _entries = unpack_x(x)
    ends = end_times_from_x(x)
    f = 0.0

    prev = t_vertical
    for end in ends:
        f += W_TIME * max(0.0, (prev + 0.1 - end) / 10.0) ** 2
        prev = end

    # Keep the program physical: expect pitch terminal angles to decrease with
    # time across the flattened powered program. α arcs are exempt — α is a
    # deviation from the flight path, not a monotone attitude, and it has to be
    # free to relax back toward zero as the atmosphere thins.
    f += W_MONO * sum(
        max(0.0, angles[i + 1] - angles[i]) ** 2
        for i in range(len(angles) - 1)
        if specs[i]["frame"] == "theta" and specs[i + 1]["frame"] == "theta"
    )
    return f


def format_params(x) -> str:
    specs, splits, angles, t_vertical, split_times, ks, entries = unpack_x(x)
    split_by_ref = {
        (sp["stage"], sp["arc"]): t_end for sp, t_end in zip(splits, split_times)
    }
    entry_by_ref = {(sp["stage"], sp["arc"]): e for sp, e in zip(ENTRIES, entries)}
    lines = [f"t_в={t_vertical:.3f} s"]
    for sp, angle, k in zip(specs, angles, ks):
        ref = (sp["stage"], sp["arc"])
        sym = "α" if sp["frame"] == "alpha" else "ϑ"
        t_end = "" if sp["is_final"] else f", t_end={split_by_ref[ref]:.3f} s"
        entry = f", ϑ0={entry_by_ref[ref]:.3f} deg" if ref in entry_by_ref else ""
        lines.append(
            f"s{sp['stage'] + 1}a{sp['arc'] + 1}: {sym}={angle:.3f} deg, "
            f"shape={arc_shape(base_arc(sp))}, k={k:.3f}{t_end}{entry}"
        )
    return "\n  ".join(lines)


def run_sim(x, h, metrics=True, out=None):
    cfg = config_from_x(x)
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, dir=HERE) as f:
        json.dump(cfg, f)
        cfg_path = f.name
    try:
        cmd = [str(BIN), f"-config={cfg_path}", f"-h={h}"]
        if AERO:
            cmd.append(f"-aero={AERO}")
        if metrics:
            cmd.append("-metrics")
        if out is not None:
            cmd.append(f"-out={out}")
        # A normal evaluation takes ~40 ms. The timeout is a backstop against a
        # pathological config the simulator cannot resolve; objective() scores
        # the resulting TimeoutExpired as infeasible.
        return subprocess.run(
            cmd, cwd=HERE, capture_output=True, text=True, check=True, timeout=60
        )
    finally:
        os.unlink(cfg_path)


def metrics(x, h) -> dict:
    res = run_sim(x, h, metrics=True)
    return json.loads(res.stdout.strip().splitlines()[-1])


def objective(x, target_km, h) -> float:
    f_prog = program_penalty(x)
    try:
        m = metrics(x, h)
    except (
        subprocess.CalledProcessError,
        subprocess.TimeoutExpired,
        json.JSONDecodeError,
        IndexError,
    ):
        return F_FAIL + f_prog  # infeasible / failed run

    f = f_prog + ((m["impact_range_km"] - target_km) / SCALE_L) ** 2

    def pen(val, lim):
        return max(0.0, (val - lim * (1.0 - CON_MARGIN)) / lim) ** 2

    f += W_CON * pen(m["max_alpha_sub_deg"], m["lim_eps1"])
    f += W_CON * pen(m["max_alpha_sup_deg"], m["lim_eps2"])
    f += W_CON * pen(m["max_pitch_rate_dps"], m["lim_theta_dot"])
    # The analytic rate is sampled on the integration grid and aliases peaks
    # between samples; the finite-difference rate across rows catches those, and
    # a ϑ discontinuity at a steering-frame change shows up here as a huge value.
    f += W_CON * pen(m["max_pitch_rate_num_dps"], m["lim_theta_dot"])
    f += W_CON * pen(m["max_q_pa"], m["lim_qmax"])
    # Max trajectory ordinate. Guarded: pen() divides by the limit, so a config
    # that does not set h_max would raise instead of skipping the term.
    if m.get("lim_h_max_km", 0) > 0:
        f += W_CON * pen(m["apogee_h_km"], m["lim_h_max_km"])
    return f


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--target", type=float, default=12000.0, help="target surface range [km]"
    )
    ap.add_argument(
        "--config",
        default=str(HERE / "rocket.json"),
        help="base rocket config: fixes the arc layout, shapes and steering frames",
    )
    ap.add_argument(
        "--out-best",
        default="out/best.json",
        help="where to write the winning config, relative to traj/",
    )
    ap.add_argument("--maxiter", type=int, default=150, help="max CMA-ES iterations")
    ap.add_argument(
        "--sigma0", type=float, default=1.0, help="global CMA-ES step multiplier"
    )
    ap.add_argument("--seed", type=int, default=20260805, help="CMA-ES RNG seed")
    ap.add_argument(
        "--aero",
        default="",
        help="path to a CFD averages.csv (default: zero aerodynamics)",
    )
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
    if Path(args.config).resolve() != BASE_CONFIG.resolve():
        raise SystemExit("internal: --config disagrees with the imported base config")

    global AERO
    if args.aero:
        if not Path(args.aero).exists():
            raise SystemExit(f"aero table not found: {args.aero}")
        # run_sim executes with cwd=HERE, so resolve before handing it over.
        AERO = str(Path(args.aero).resolve())
    print(f"aero: {AERO or 'ZERO (no table)'}")

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
        # Keyed to --out-best so concurrent runs do not share log files.
        "verb_filenameprefix": f"outcmaes/{Path(args.out_best).stem}-",
    }
    es = cma.CMAEvolutionStrategy(x0, args.sigma0, opts)
    es.optimize(lambda x: objective(x, args.target, args.h_opt))

    best = es.result.xbest
    print(f"\nbest params:\n  {format_params(best)}")

    # Persist the best config before anything that can fail, so a broken final
    # run never discards the whole search.
    best_path = HERE / args.out_best
    best_path.parent.mkdir(parents=True, exist_ok=True)
    with open(best_path, "w") as f:
        json.dump(config_from_x(best), f, indent=2)
    print(f"wrote best config -> {best_path}")
    print(
        f"run command:\n  ./out/traj-sim -config={args.out_best} "
        f"-h={args.h_final} -out=out/traj.csv"
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
        f"ϑ̇={m['max_pitch_rate_dps']:.2f}~{m['max_pitch_rate_num_dps']:.2f}"
        f"/{m['lim_theta_dot']:.2f} "
        f"q={m['max_q_pa'] / 1000:.1f}/{m['lim_qmax'] / 1000:.0f} kPa "
        f"H={m['apogee_h_km']:.0f}/{m.get('lim_h_max_km', 0):.0f} km"
    )

    # Final fine-step run: writes out/traj.csv and prints the full diagnostics.
    print("\n=== final run (fine step) ===")
    res = run_sim(best, args.h_final, metrics=False, out="out/traj.csv")
    print(res.stdout, end="")


if __name__ == "__main__":
    main()
