#!/usr/bin/env python3
"""Robust, resumable sweep runner for the coursework rocket aero study.

Drives the existing tooling (gen_case.py + the case Allrun/Allrun.pre scripts)
over the full (part, Ma, alpha) matrix in a deliberate *priority order* so the
most important results land first, and so a single failing case never sinks the
whole multi-hour run.

    uv run python openfoam/sweep.py --dry-run     # print the ordered queue only
    uv run python openfoam/sweep.py               # full run (needs OpenFOAM sourced)
    uv run python openfoam/sweep.py --retry-failed

What it does per case, in queue order:
  1. generate  -> gen_case.py --part .. --regime .. --Ma .. --alpha .. --out DIR
  2. mesh      -> ./Allrun.pre  (once per (part, Ma) group; later alphas reuse it)
  3. solve     -> ./Allrun      (decomposePar -> solver -> reconstructPar)
  4. extract   -> last Cd/Cl/Cm from postProcessing/forceCoeffs/*/coefficient.dat

Per-case status is tracked in a JSON manifest written atomically after every
step, so the sweep resumes where it left off (completed cases are skipped) and
failures are isolated rows, not an aborted run. The mesh depends only on
(part, Ma) -- not alpha -- so alphas in a group share one snappyHexMesh run
(README: "alpha-only sweeps reuse the same mesh").

Regime is chosen from the Mach number (Ma < --subsonic-max -> rhoSimpleFoam,
else hisa); everything else is forwarded to gen_case.py unchanged.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as _dt
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

# --- locations -------------------------------------------------------------
HERE = Path(__file__).resolve().parent  # openfoam/
ROOT = HERE.parent  # repo root (has the Makefile)
GEN_CASE = HERE / "gen_case.py"

PART_STL = {  # Makefile target per part (mirrors gen_case.PART_STL)
    "all": "rocket.stl",
    "stage2up": "stage2up.stl",
    "stage3up": "stage3up.stl",
    "head": "head.stl",
}

# --- sweep matrix ----------------------------------------------------------
# Part order is the priority order across parts. For each part, Ma and alpha are
# listed in priority order; the second item of each pair flags a *preferential*
# value (the parenthesised ones in the study request). A case is preferential
# iff BOTH its Ma and its alpha are preferential.
PARTS = ["all", "stage2up", "stage3up", "head"]
SWEEP: dict[str, dict[str, list[tuple[float, int]]]] = {
    "all": {
        "Ma": [(0.4, 1), (0.7, 0), (0.9, 1), (1.2, 1), (1.4, 0), (1.7, 1), (4, 0), (8, 1)],
        "alpha": [(0, 1), (3, 1), (5, 0), (10, 1), (15, 0)],
    },
    "stage2up": {
        "Ma": [(6, 1), (9, 0), (12, 1), (16, 0)],
        "alpha": [(0, 1), (3, 1), (5, 0), (10, 1)],
    },
    "stage3up": {
        "Ma": [(15, 1), (18, 0), (21, 1), (24, 0), (27, 1)],
        "alpha": [(0, 1), (3, 1), (5, 0), (10, 1)],
    },
    "head": {
        "Ma": [(18, 0), (20, 1), (22, 0), (24, 1), (26, 1)],
        "alpha": [(0, 1)],
    },
}


def fmt_num(x: float) -> str:
    """Compact, filesystem-safe number (0.4 -> '0.4', 4.0 -> '4', 10 -> '10')."""
    return f"{x:g}"


@dataclasses.dataclass
class Case:
    part: str
    regime: str
    ma: float
    alpha: float
    preferential: bool
    sort_key: tuple[int, int, int, int]

    @property
    def name(self) -> str:
        return f"Ma{fmt_num(self.ma)}_a{fmt_num(self.alpha)}"

    @property
    def cid(self) -> str:
        return f"{self.part}/{self.regime}/{self.name}"

    @property
    def group(self) -> tuple[str, str]:
        """Cases sharing a mesh: same part and Mach (alpha doesn't change the mesh)."""
        return (self.part, fmt_num(self.ma))

    def out_dir(self, base: Path) -> Path:
        return base / self.part / self.regime / self.name


def build_queue(subsonic_max: float) -> list[Case]:
    """Whole matrix, sorted into priority order.

    sort_key = (preferential-tier, part, Ma-order, alpha-order):
      * tier 0 = (Ma preferential AND alpha preferential) -> every part's
        preferential cases come before ANY non-preferential case.
      * then part order (all -> stage2up -> stage3up -> head),
      * then Ma in listed order, then alpha in listed order.
    """
    cases: list[Case] = []
    for pi, part in enumerate(PARTS):
        ma_list = SWEEP[part]["Ma"]
        alpha_list = SWEEP[part]["alpha"]
        for mi, (ma, ma_pref) in enumerate(ma_list):
            regime = "subsonic" if ma < subsonic_max else "supersonic"
            for ai, (alpha, alpha_pref) in enumerate(alpha_list):
                pref = bool(ma_pref and alpha_pref)
                key = (0 if pref else 1, pi, mi, ai)
                cases.append(Case(part, regime, float(ma), float(alpha), pref, key))
    cases.sort(key=lambda c: c.sort_key)
    return cases


# --- manifest / results ----------------------------------------------------
def load_state(path: Path) -> dict[str, dict]:
    if path.is_file():
        return json.loads(path.read_text())
    return {}


def save_state(path: Path, state: dict[str, dict]) -> None:
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(state, indent=2, sort_keys=True))
    os.replace(tmp, path)  # atomic: a crash mid-write can't corrupt the manifest


def set_status(state: dict, path: Path, case: Case, base: Path, **fields) -> None:
    rec = state.setdefault(case.cid, {})
    rec.update(
        part=case.part,
        regime=case.regime,
        Ma=case.ma,
        alpha=case.alpha,
        preferential=case.preferential,
        dir=str(case.out_dir(base)),
    )
    rec.update(fields)
    rec["updated"] = _dt.datetime.now().isoformat(timespec="seconds")
    save_state(path, state)


def write_results(state: dict, queue: list[Case], path: Path) -> None:
    cols = [
        "part", "regime", "Ma", "alpha", "preferential",
        "status", "Cd", "Cl", "Cm", "case_dir", "error",
    ]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for c in queue:
            rec = state.get(c.cid, {})
            w.writerow({
                "part": c.part, "regime": c.regime, "Ma": c.ma, "alpha": c.alpha,
                "preferential": int(c.preferential),
                "status": rec.get("status", "pending"),
                "Cd": rec.get("Cd", ""), "Cl": rec.get("Cl", ""), "Cm": rec.get("Cm", ""),
                "case_dir": rec.get("dir", ""), "error": rec.get("error", ""),
            })


# --- shell helpers ---------------------------------------------------------
class StepError(Exception):
    def __init__(self, step: str, reason: str, tail: str = ""):
        super().__init__(f"{step}: {reason}")
        self.step = step
        self.reason = reason
        self.tail = tail


def _tail(path: Path, n: int = 40) -> str:
    try:
        return "".join(path.read_text(errors="replace").splitlines(keepends=True)[-n:])
    except OSError:
        return ""


def run_step(step: str, cmd: list[str], cwd: Path, log_path: Path, timeout: float) -> None:
    """Run a command, streaming combined output to log_path; raise StepError on failure."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w") as log:
        try:
            subprocess.run(
                cmd, cwd=cwd, stdout=log, stderr=subprocess.STDOUT,
                timeout=timeout, check=True,
            )
        except subprocess.TimeoutExpired:
            raise StepError(step, f"timed out after {timeout:.0f}s", _tail(log_path))
        except subprocess.CalledProcessError as e:
            raise StepError(step, f"exited {e.returncode}", _tail(log_path))
        except FileNotFoundError as e:
            raise StepError(step, f"command not found: {e}", "")


def has_mesh(case_dir: Path) -> bool:
    return (case_dir / "constant" / "polyMesh" / "points").is_file()


def copy_mesh(src: Path, dst: Path) -> None:
    """Reuse a sibling's mesh: copy constant/polyMesh into dst."""
    dst_poly = dst / "constant" / "polyMesh"
    if dst_poly.exists():
        shutil.rmtree(dst_poly)
    shutil.copytree(src / "constant" / "polyMesh", dst_poly)


def extract_coeffs(case_dir: Path) -> tuple[float, float, float]:
    """Last (Cd, Cl, Cm[pitch]) from postProcessing/forceCoeffs/*/coefficient.dat."""
    pp = case_dir / "postProcessing" / "forceCoeffs"
    times = [d for d in pp.iterdir() if d.is_dir()] if pp.is_dir() else []
    if not times:
        raise StepError("extract", f"no forceCoeffs output under {pp}")
    latest = max(times, key=lambda d: float(d.name))
    dat = latest / "coefficient.dat"
    header: list[str] = []
    last: list[str] = []
    for line in dat.read_text().splitlines():
        s = line.strip()
        if not s:
            continue
        if s.startswith("#"):
            toks = s.lstrip("#").split()
            if "Cd" in toks:  # the column-label comment line
                header = toks
            continue
        last = s.split()
    if not last:
        raise StepError("extract", f"no data rows in {dat}")

    def col(name: str, fallback_idx: int) -> float:
        # header labels align to data columns including the leading Time column
        if header and name in header:
            idx = header.index(name)
            if idx < len(last):
                return float(last[idx])
        return float(last[fallback_idx])

    cd = col("Cd", 1)
    cl = col("Cl", 3)
    cm = float("nan")
    for nm, fb in (("CmPitch", 5), ("Cm", 5)):
        try:
            cm = col(nm, fb)
            break
        except (ValueError, IndexError):
            continue
    return cd, cl, cm


# --- preflight -------------------------------------------------------------
def preflight(queue: list[Case]) -> None:
    """Fail fast (before hours of work) if the environment can't run the sweep."""
    if not os.environ.get("WM_PROJECT_DIR"):
        sys.exit("error: WM_PROJECT_DIR is not set -- source your OpenFOAM environment first")
    need = ["blockMesh", "snappyHexMesh", "surfaceFeatureExtract", "decomposePar"]
    regimes = {c.regime for c in queue}
    if "subsonic" in regimes:
        need.append("rhoSimpleFoam")
    if "supersonic" in regimes:
        need.append("hisa")
    missing = [b for b in need if shutil.which(b) is None]
    if missing:
        sys.exit(f"error: OpenFOAM binaries not on PATH: {', '.join(missing)}")

    # Build any STLs that gen_case will need (it is invoked with --no-stl below).
    parts = {c.part for c in queue} | {"all"}  # 'all' is always the coefficient reference
    targets = sorted({PART_STL[p] for p in parts})
    missing_stls = [t for t in targets if not (ROOT / t).is_file()]
    if missing_stls:
        if shutil.which("make") is None:
            sys.exit(f"error: STLs missing {missing_stls} and 'make' not on PATH to build them")
        run = subprocess.run(["make", *missing_stls, "SCALE=1"], cwd=ROOT)
        if run.returncode != 0:
            sys.exit("error: failed to build STLs (make returned non-zero)")


# --- per-case pipeline -----------------------------------------------------
def discover_meshes(queue: list[Case], state: dict, base: Path) -> dict[tuple[str, str], Path]:
    """Map each (part, Ma) group to an already-meshed, completed case dir on disk.

    Only 'done' cases qualify -- they will not be reprocessed (and thus not
    wiped), so reusing their mesh is safe across a resume.
    """
    src: dict[tuple[str, str], Path] = {}
    for c in queue:
        rec = state.get(c.cid)
        d = c.out_dir(base)
        if rec and rec.get("status") == "done" and has_mesh(d):
            src.setdefault(c.group, d)
    return src


def run_case(
    case: Case, base: Path, args, state: dict, group_src: dict[tuple[str, str], Path]
) -> None:
    out = case.out_dir(base)
    if out.exists():  # start clean so generate + Allrun never trip over stale logs
        shutil.rmtree(out)
    out.mkdir(parents=True)

    # 1. generate -----------------------------------------------------------
    gen_cmd = [
        sys.executable, str(GEN_CASE),
        "--part", case.part, "--regime", case.regime,
        "--Ma", fmt_num(case.ma), "--alpha", fmt_num(case.alpha),
        "--np", str(args.np), "--out", str(out), "--no-stl",
    ]
    run_step("generate", gen_cmd, ROOT, out / "log.gen", args.gen_timeout)
    set_status(state, args.state, case, base, status="generated", error="")

    # 2. mesh (once per (part, Ma) group; reuse for the other alphas) -------
    src = None if args.no_mesh_reuse else group_src.get(case.group)
    if src is not None and has_mesh(src):
        copy_mesh(src, out)
    else:
        run_step("mesh", ["./Allrun.pre"], out, out / "log.Allrun.pre", args.mesh_timeout)
        group_src[case.group] = out
    set_status(state, args.state, case, base, status="meshed")

    # 3. solve --------------------------------------------------------------
    run_step("solve", ["./Allrun"], out, out / "log.Allrun", args.solve_timeout)

    # 4. extract coefficients ----------------------------------------------
    cd, cl, cm = extract_coeffs(out)
    set_status(state, args.state, case, base, status="done", Cd=cd, Cl=cl, Cm=cm, error="")


# --- reporting -------------------------------------------------------------
def print_queue(queue: list[Case]) -> None:
    for i, c in enumerate(queue, 1):
        flag = "*" if c.preferential else " "
        print(f"{i:3d}. {flag} {c.regime:10s} {c.cid}")
    n_pref = sum(c.preferential for c in queue)
    n_sub = sum(c.regime == "subsonic" for c in queue)
    groups = {c.group for c in queue}
    print(
        f"\n{len(queue)} cases  |  preferential {n_pref}, rest {len(queue) - n_pref}"
        f"  |  subsonic {n_sub}, supersonic {len(queue) - n_sub}"
        f"  |  {len(groups)} mesh groups (snappyHexMesh runs)"
    )
    by_part = {p: sum(c.part == p for c in queue) for p in PARTS}
    print("  per part: " + ", ".join(f"{p}={n}" for p, n in by_part.items()))


def log(msg: str, log_path: Path | None) -> None:
    stamp = _dt.datetime.now().isoformat(timespec="seconds")
    line = f"[{stamp}] {msg}"
    print(line, flush=True)
    if log_path is not None:
        with log_path.open("a") as f:
            f.write(line + "\n")


# --- main ------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--dry-run", action="store_true", help="print the ordered queue and exit")
    ap.add_argument("--base", type=Path, default=HERE, help="root for case dirs (default openfoam/)")
    ap.add_argument("--state", type=Path, default=HERE / "sweep_state.json", help="manifest path")
    ap.add_argument("--results-csv", type=Path, default=HERE / "sweep_results.csv")
    ap.add_argument("--log", type=Path, default=HERE / "sweep.log")
    ap.add_argument("--np", type=int, default=12, help="MPI subdomains per case (forwarded to gen_case)")
    ap.add_argument("--subsonic-max", type=float, default=1.0, help="Ma < this -> subsonic regime")
    ap.add_argument("--only-part", action="append", choices=PARTS, help="restrict to part(s); repeatable")
    ap.add_argument("--max-cases", type=int, default=None, help="process at most N pending cases")
    ap.add_argument("--retry-failed", action="store_true", help="re-queue cases marked failed")
    ap.add_argument("--no-mesh-reuse", action="store_true", help="mesh every case (no alpha sharing)")
    ap.add_argument("--gen-timeout", type=float, default=600.0)
    ap.add_argument("--mesh-timeout", type=float, default=7200.0)
    ap.add_argument("--solve-timeout", type=float, default=21600.0)
    args = ap.parse_args()

    queue = build_queue(args.subsonic_max)
    if args.only_part:
        queue = [c for c in queue if c.part in set(args.only_part)]

    if args.dry_run:
        print_queue(queue)
        return

    state = load_state(args.state)

    # resume: drop completed (and failed, unless retrying) cases
    todo: list[Case] = []
    skipped_done = skipped_failed = 0
    for c in queue:
        st = state.get(c.cid, {}).get("status")
        if st == "done":
            skipped_done += 1
            continue
        if st == "failed" and not args.retry_failed:
            skipped_failed += 1
            continue
        todo.append(c)
    if args.max_cases is not None:
        todo = todo[: args.max_cases]

    preflight(queue)
    group_src = discover_meshes(queue, state, args.base)

    log(
        f"sweep start: {len(todo)} to run, {skipped_done} done, "
        f"{skipped_failed} failed-skipped (of {len(queue)} total)",
        args.log,
    )
    n_ok = n_fail = 0
    for i, c in enumerate(todo, 1):
        log(f"[{i}/{len(todo)}] {c.cid}  (regime={c.regime}) ...", args.log)
        try:
            run_case(c, args.base, args, state, group_src)
            rec = state[c.cid]
            log(f"    done  Cd={rec['Cd']:.5g} Cl={rec['Cl']:.5g} Cm={rec['Cm']:.5g}", args.log)
            n_ok += 1
        except StepError as e:
            set_status(
                state, args.state, c, args.base,
                status="failed", error=f"{e.step}: {e.reason}",
            )
            (c.out_dir(args.base) / "sweep.error").write_text(
                f"{e.step}: {e.reason}\n\n--- last output ---\n{e.tail}\n"
            )
            log(f"    FAILED at {e.step}: {e.reason} -- continuing", args.log)
            n_fail += 1
        except Exception as e:  # never let one case kill the sweep
            set_status(state, args.state, c, args.base, status="failed", error=repr(e))
            log(f"    FAILED (unexpected): {e!r} -- continuing", args.log)
            n_fail += 1
        write_results(state, queue, args.results_csv)

    log(f"sweep end: {n_ok} ok, {n_fail} failed this run. results -> {args.results_csv}", args.log)


if __name__ == "__main__":
    main()
