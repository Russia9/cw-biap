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
import pickle
import subprocess
import tempfile
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cma

HERE = Path(__file__).parent

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

# The CMA-ES state is checkpointed every this many iterations (matching the
# verb_disp progress cadence), so an interrupted search resumes via --resume
# instead of being discarded.
CHECKPOINT_EVERY = 10

# Angle bounds per steering frame. ϑ arcs sweep the whole powered turn; α arcs
# only ever need to reach a little past the §4.4 supersonic limit of 10 deg, and
# a small positive α must stay reachable for the pitch-over.
ANGLE_BOUNDS = {"theta": (5.0, 89.0), "alpha": (-12.0, 3.0)}

Config = dict[str, Any]


def arc_shape(arc: Config) -> str:
    return arc.get("shape") or "exp"


def stage_frame(stage: Config) -> str:
    """Steering frame of a stage: "theta" (arcs are ϑ) or "alpha" (arcs are α)."""
    return stage.get("steering") or "theta"


def angle_key(frame: str) -> str:
    return "alpha_deg" if frame == "alpha" else "theta_deg"


def arc_k(arc: Config) -> float:
    if "k" in arc:
        return arc["k"]
    return DEFAULT_K_COS if arc_shape(arc) == "cos" else DEFAULT_K_EXP


@dataclass(frozen=True)
class ArcSpec:
    """Position and steering context of one pitch arc in the flattened program."""

    stage: int
    arc: int
    stage_start: float
    stage_end: float
    is_final: bool
    frame: str

    @property
    def ref(self) -> tuple[int, int]:
        return (self.stage, self.arc)


@dataclass(frozen=True)
class Params:
    """One optimizer vector, unpacked into its named blocks."""

    angles: list[float]
    t_vertical: float
    split_times: list[float]
    ks: list[float]
    entries: list[float]


@dataclass(frozen=True)
class Layout:
    """The optimization problem's fixed structure, derived from one base config.

    The base config fixes the arc layout, shapes and steering frames for the
    whole run; the vector layout is [angles | t_в | non-final t_end | ks |
    frame-change entry angles].
    """

    base: Config
    specs: list[ArcSpec]
    splits: list[ArcSpec]
    entries: list[ArcSpec]

    @classmethod
    def from_config(cls, base: Config) -> "Layout":
        specs = []
        stage_start = 0.0
        for si, st in enumerate(base["stages"]):
            stage_end = stage_start + st["burn_time"]
            for ai in range(len(st["pitch"])):
                specs.append(
                    ArcSpec(
                        stage=si,
                        arc=ai,
                        stage_start=stage_start,
                        stage_end=stage_end,
                        is_final=ai == len(st["pitch"]) - 1,
                        frame=stage_frame(st),
                    )
                )
            stage_start = stage_end
        splits = [sp for sp in specs if not sp.is_final]
        # Arcs carrying an explicit entry pitch, which a stage needs when it
        # switches steering frame. Their continuity with the preceding arc is
        # not structural, so the objective penalises the resulting ϑ jump via
        # max_pitch_rate_num_dps.
        entries = [sp for sp in specs if "theta0_deg" in cls._arc(base, sp)]
        return cls(base=base, specs=specs, splits=splits, entries=entries)

    @staticmethod
    def _arc(base: Config, sp: ArcSpec) -> Config:
        return base["stages"][sp.stage]["pitch"][sp.arc]

    def base_arc(self, sp: ArcSpec) -> Config:
        return self._arc(self.base, sp)

    def vector_dim(self) -> int:
        """Angles, t_в, non-final t_end values, ks, then frame-change entry angles."""
        return 2 * len(self.specs) + 1 + len(self.splits) + len(self.entries)

    def default_x0(self) -> list[float]:
        angles = []
        split_times = []
        ks = []
        for sp in self.specs:
            arc = self.base_arc(sp)
            key = angle_key(sp.frame)
            if key not in arc:
                # The typical cause: a stage's "steering" was flipped without
                # renaming its arcs' angle fields to match the new frame.
                raise SystemExit(
                    f"stage {sp.stage + 1} arc {sp.arc + 1}: missing {key!r} "
                    f"(the stage is {sp.frame!r}-steered; rename the arc's "
                    "angle field to match)"
                )
            angles.append(arc[key])
            ks.append(arc_k(arc))
            if not sp.is_final:
                split_times.append(
                    arc.get("t_end", (sp.stage_start + sp.stage_end) / 2)
                )
        if len(split_times) != len(self.splits):
            raise RuntimeError("internal optimizer layout mismatch")
        entries = [self.base_arc(sp)["theta0_deg"] for sp in self.entries]
        return [*angles, self.base["t_vertical"], *split_times, *ks, *entries]

    def unpack(self, x: Sequence[float]) -> Params:
        n_arcs = len(self.specs)
        n_splits = len(self.splits)
        if len(x) != self.vector_dim():
            raise ValueError(
                f"expected {self.vector_dim()} optimizer values, got {len(x)}"
            )
        return Params(
            angles=[float(v) for v in x[:n_arcs]],
            t_vertical=float(x[n_arcs]),
            split_times=[float(v) for v in x[n_arcs + 1 : n_arcs + 1 + n_splits]],
            ks=[float(v) for v in x[n_arcs + 1 + n_splits : 2 * n_arcs + 1 + n_splits]],
            entries=[float(v) for v in x[2 * n_arcs + 1 + n_splits :]],
        )

    def config_from_x(self, x: Sequence[float]) -> Config:
        """Map the CMA-ES vector onto a rocket config (arc shapes kept from base)."""
        p = self.unpack(x)
        c = copy.deepcopy(self.base)
        c["t_vertical"] = p.t_vertical
        for sp, angle, k in zip(self.specs, p.angles, p.ks):
            arc = self._arc(c, sp)
            arc.update({angle_key(sp.frame): angle, "k": k})
            # Drop a stale opposite-frame angle (left behind by a steering
            # flip in the base config): the simulator rejects an arc carrying
            # both, which would silently fail every evaluation.
            arc.pop(
                "theta_deg" if sp.frame == "alpha" else "alpha_deg", None
            )
        for sp, t_end in zip(self.splits, p.split_times):
            self._arc(c, sp)["t_end"] = t_end
        for sp, entry in zip(self.entries, p.entries):
            self._arc(c, sp)["theta0_deg"] = entry
        return c

    def bounds(self) -> tuple[list[float], list[float]]:
        angle_low, angle_high = zip(
            *(ANGLE_BOUNDS[sp.frame] for sp in self.specs), strict=True
        )
        k_low = [
            1.0 if arc_shape(self.base_arc(sp)) == "cos" else -8.0
            for sp in self.specs
        ]
        split_low = [sp.stage_start + 1.0 for sp in self.splits]
        split_high = [sp.stage_end - 0.1 for sp in self.splits]
        n_arcs = len(self.specs)
        # An entry angle lives in its arc's own frame (configfile.go reads
        # theta0_deg as the raw entry value), so α-framed entries get the α
        # bounds — 0..89° would be nonsense for a deviation angle.
        entry_low = [
            ANGLE_BOUNDS["alpha"][0] if sp.frame == "alpha" else 0.0
            for sp in self.entries
        ]
        entry_high = [
            ANGLE_BOUNDS["alpha"][1] if sp.frame == "alpha" else 89.0
            for sp in self.entries
        ]
        return (
            [*angle_low, 5.0, *split_low, *k_low, *entry_low],
            [*angle_high, 40.0, *split_high, *([8.0] * n_arcs), *entry_high],
        )

    def cma_stds(self) -> list[float]:
        n_arcs = len(self.specs)
        return [
            *([10.0] * n_arcs),
            8.0,
            *([8.0] * len(self.splits)),
            *([3.0] * n_arcs),
            *([8.0] * len(self.entries)),
        ]

    def _split_by_ref(self, p: Params) -> dict[tuple[int, int], float]:
        return {sp.ref: t_end for sp, t_end in zip(self.splits, p.split_times)}

    def end_times(self, p: Params) -> list[float]:
        split_by_ref = self._split_by_ref(p)
        return [
            sp.stage_end if sp.is_final else split_by_ref[sp.ref] for sp in self.specs
        ]

    def program_penalty(self, x: Sequence[float]) -> float:
        p = self.unpack(x)
        ends = self.end_times(p)
        f = 0.0

        prev = p.t_vertical
        for end in ends:
            f += W_TIME * max(0.0, (prev + 0.1 - end) / 10.0) ** 2
            prev = end

        # Keep the program physical: expect pitch terminal angles to decrease
        # with time across the flattened powered program. α arcs are exempt — α
        # is a deviation from the flight path, not a monotone attitude, and it
        # has to be free to relax back toward zero as the atmosphere thins.
        f += W_MONO * sum(
            max(0.0, p.angles[i + 1] - p.angles[i]) ** 2
            for i in range(len(p.angles) - 1)
            if self.specs[i].frame == "theta" and self.specs[i + 1].frame == "theta"
        )
        return f

    def format_params(self, x: Sequence[float]) -> str:
        p = self.unpack(x)
        split_by_ref = self._split_by_ref(p)
        entry_by_ref = {sp.ref: e for sp, e in zip(self.entries, p.entries)}
        lines = [f"t_в={p.t_vertical:.3f} s"]
        for sp, angle, k in zip(self.specs, p.angles, p.ks):
            sym = "α" if sp.frame == "alpha" else "ϑ"
            t_end = "" if sp.is_final else f", t_end={split_by_ref[sp.ref]:.3f} s"
            entry = (
                f", ϑ0={entry_by_ref[sp.ref]:.3f} deg" if sp.ref in entry_by_ref else ""
            )
            lines.append(
                f"s{sp.stage + 1}a{sp.arc + 1}: {sym}={angle:.3f} deg, "
                f"shape={arc_shape(self.base_arc(sp))}, k={k:.3f}{t_end}{entry}"
            )
        return "\n  ".join(lines)


class SimError(Exception):
    """A simulator evaluation that could not produce metrics.

    Wraps every failure mode of one run — a non-zero exit, a timeout, or
    unparseable output — and carries the simulator's stderr for reporting.
    """

    def __init__(self, message: str, stderr: str = "") -> None:
        super().__init__(message)
        self.stderr = stderr.strip()


@dataclass(frozen=True)
class SimRunner:
    """Runs the Go simulator binary against generated config files."""

    binary: Path
    cwd: Path
    aero: str  # resolved averages.csv path, or "" for zero aerodynamics
    timeout: float = 60.0

    def build(self) -> None:
        subprocess.run(
            ["go", "build", "-o", str(self.binary), "./main"],
            cwd=self.cwd,
            check=True,
        )

    def run(
        self,
        cfg: Config,
        h: float,
        metrics: bool = True,
        out: str | None = None,
    ) -> "subprocess.CompletedProcess[str]":
        with tempfile.NamedTemporaryFile(
            "w", suffix=".json", delete=False, dir=self.cwd
        ) as f:
            json.dump(cfg, f)
            cfg_path = Path(f.name)
        try:
            cmd = [str(self.binary), f"-config={cfg_path}", f"-h={h}"]
            if self.aero:
                cmd.append(f"-aero={self.aero}")
            if metrics:
                cmd.append("-metrics")
            if out is not None:
                cmd.append(f"-out={out}")
            # A normal evaluation takes ~40 ms. The timeout is a backstop
            # against a pathological config the simulator cannot resolve;
            # objective() scores the resulting SimError as infeasible.
            try:
                return subprocess.run(
                    cmd,
                    cwd=self.cwd,
                    capture_output=True,
                    text=True,
                    check=True,
                    timeout=self.timeout,
                )
            except subprocess.CalledProcessError as exc:
                raise SimError(str(exc), exc.stderr or "") from exc
            except subprocess.TimeoutExpired as exc:
                stderr = exc.stderr
                if isinstance(stderr, bytes):
                    stderr = stderr.decode(errors="replace")
                raise SimError(str(exc), stderr or "") from exc
        finally:
            cfg_path.unlink()

    def metrics(self, cfg: Config, h: float) -> dict[str, float]:
        res = self.run(cfg, h, metrics=True)
        try:
            return json.loads(res.stdout.strip().splitlines()[-1])
        except (json.JSONDecodeError, IndexError) as exc:
            raise SimError(f"unparseable metrics output: {exc}", res.stderr) from exc


def objective(
    layout: Layout,
    runner: SimRunner,
    x: Sequence[float],
    target_km: float,
    h: float,
) -> float:
    f_prog = layout.program_penalty(x)
    try:
        m = runner.metrics(layout.config_from_x(x), h)
    except SimError:
        return F_FAIL + f_prog  # infeasible / failed run

    f = f_prog + ((m["impact_range_km"] - target_km) / SCALE_L) ** 2

    def pen(val: float, lim: float) -> float:
        # A limit of zero means "not configured" (the Go side emits 0 for an
        # absent h_max, and hand-written configs may drop others): no penalty
        # term, rather than a division by zero mid-search.
        if lim <= 0:
            return 0.0
        return max(0.0, (val - lim * (1.0 - CON_MARGIN)) / lim) ** 2

    f += W_CON * pen(m["max_alpha_sub_deg"], m["lim_eps1"])
    f += W_CON * pen(m["max_alpha_sup_deg"], m["lim_eps2"])
    f += W_CON * pen(m["max_pitch_rate_dps"], m["lim_theta_dot"])
    # The analytic rate is sampled on the integration grid and aliases peaks
    # between samples; the finite-difference rate across rows catches those, and
    # a ϑ discontinuity at a steering-frame change shows up here as a huge value.
    f += W_CON * pen(m["max_pitch_rate_num_dps"], m["lim_theta_dot"])
    f += W_CON * pen(m["max_q_pa"], m["lim_qmax"])
    f += W_CON * pen(m["apogee_h_km"], m.get("lim_h_max_km", 0.0))
    return f


def parse_args() -> argparse.Namespace:
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
            "non-final arcs [s], all arc k values, then any frame-change entry "
            "angles [deg]; defaults from rocket.json"
        ),
    )
    ap.add_argument(
        "--resume",
        default="",
        help=(
            "path to a CMA-ES checkpoint (out/<stem>-cma.pkl) to continue an "
            "interrupted or finished search; pass a larger --maxiter to extend. "
            "--x0/--sigma0/--seed are ignored, the checkpoint carries them"
        ),
    )
    return ap.parse_args()


def main() -> None:
    args = parse_args()

    with open(args.config) as f:
        layout = Layout.from_config(json.load(f))

    aero = ""
    if args.aero:
        if not Path(args.aero).exists():
            raise SystemExit(f"aero table not found: {args.aero}")
        # The simulator executes with cwd=traj/, so resolve before handing over.
        aero = str(Path(args.aero).resolve())
    print(f"aero: {aero or 'ZERO (no table)'}")

    # A per-run binary: concurrent seed runs must not race one `go build -o`
    # target while clobbering each other's executable. Removed on exit.
    stem = Path(args.out_best).stem
    runner = SimRunner(
        binary=HERE / "out" / f"traj-sim-{stem}-{os.getpid()}", cwd=HERE, aero=aero
    )
    runner.build()
    try:
        run_search(args, layout, runner)
    finally:
        runner.binary.unlink(missing_ok=True)


def run_search(args: argparse.Namespace, layout: Layout, runner: SimRunner) -> None:
    expected = layout.vector_dim()
    ckpt_path = HERE / "out" / f"{Path(args.out_best).stem}-cma.pkl"

    if args.resume:
        # pickle is the only serialization cma offers for a live strategy
        # (es.pickle_dumps); the checkpoint is a local file this same tool
        # wrote, not untrusted input.
        with open(args.resume, "rb") as f:
            es = pickle.load(f)
        if es.N != expected:
            raise SystemExit(
                f"checkpoint dimension {es.N} does not match this config's "
                f"{expected} — it was made for a different arc layout"
            )
        es.opts.set({"maxiter": args.maxiter})
        print(f"resumed CMA-ES state from {args.resume} at iteration {es.countiter}")
    else:
        x0 = args.x0 if args.x0 is not None else layout.default_x0()
        if len(x0) != expected:
            raise SystemExit(
                f"--x0 must contain {expected} values for this config, got {len(x0)}"
            )

        # A base config the simulator rejects outright would score every
        # candidate F_FAIL and present the search with a flat, hopeless
        # landscape; surface the actual error before spending hours on it.
        try:
            runner.metrics(layout.config_from_x(x0), args.h_opt)
        except SimError as exc:
            msg = f"the starting configuration does not simulate: {exc}"
            if exc.stderr:
                msg += f"\nsimulator stderr:\n{exc.stderr}"
            raise SystemExit(msg) from exc

        # The cma logger resolves its prefix against the process cwd while the
        # simulator runs with cwd=traj/; anchor it so the logs land in
        # traj/outcmaes/ regardless of the launch directory.
        outcmaes = HERE / "outcmaes"
        outcmaes.mkdir(exist_ok=True)

        # Per-dimension steps: angles [deg], t_в [s], t_end [s], k [-], entries
        # [deg]. sigma0 scales all.
        opts = {
            "bounds": list(layout.bounds()),
            "CMA_stds": layout.cma_stds(),
            "maxiter": args.maxiter,
            "verb_disp": 10,
            "seed": args.seed,
            # Keyed to --out-best so concurrent runs do not share log files.
            "verb_filenameprefix": f"{outcmaes}/{Path(args.out_best).stem}-",
        }
        es = cma.CMAEvolutionStrategy(x0, args.sigma0, opts)

    def checkpoint(es_: Any) -> None:
        """Atomically persist the strategy so Ctrl-C cannot discard the search."""
        tmp = ckpt_path.with_suffix(".pkl.tmp")
        tmp.write_bytes(es_.pickle_dumps())
        tmp.replace(ckpt_path)

    def on_iteration(es_: Any) -> None:
        if es_.countiter % CHECKPOINT_EVERY == 0:
            checkpoint(es_)

    ckpt_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"checkpoint every {CHECKPOINT_EVERY} iterations -> {ckpt_path}")
    es.optimize(
        lambda x: objective(layout, runner, x, args.target, args.h_opt),
        callback=on_iteration,
    )
    checkpoint(es)  # the finished state stays resumable with a larger --maxiter

    best = es.result.xbest
    if best is None:
        raise SystemExit("search produced no evaluated candidate")
    print(f"\nbest params:\n  {layout.format_params(best)}")

    # Persist the best config before the verification runs below, so a broken
    # final run never discards the whole search.
    best_path = HERE / args.out_best
    best_path.parent.mkdir(parents=True, exist_ok=True)
    with open(best_path, "w") as f:
        json.dump(layout.config_from_x(best), f, indent=2)
    print(f"wrote best config -> {best_path}")
    print(
        f"run command:\n  go run ./main -config={args.out_best} "
        f"-h={args.h_final} -out=out/traj.csv"
    )

    try:
        m = runner.metrics(layout.config_from_x(best), args.h_final)
    except SimError as exc:
        print(f"\nfinal metrics run failed: {exc}")
        if exc.stderr:
            print(f"simulator stderr:\n{exc.stderr}")
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
    try:
        res = runner.run(
            layout.config_from_x(best), args.h_final, metrics=False, out="out/traj.csv"
        )
    except SimError as exc:
        print(f"final run failed: {exc}")
        if exc.stderr:
            print(f"simulator stderr:\n{exc.stderr}")
        return
    print(res.stdout, end="")


if __name__ == "__main__":
    main()
