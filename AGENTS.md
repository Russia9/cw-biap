# Repository Guidelines

## Project Structure & Module Organization

This repository contains BIAP coursework for a three-stage solid-fuel rocket: sizing calculations that emit Typst snippets for the report, and a trajectory simulator that checks the resulting design.

Python calculation layer: `main.py` is the primary script and emits Typst math/table snippets; `preliminary.py` emits the fuel-selection tables. Keep reusable physics, interpolation and CSV loading in `utils.py`; keep Typst formatting helpers in `typst.py`. Raw textbook data lives in `assets/*.csv`.

Trajectory layer under `traj/`: Go package `traj` (`sim.go` integration, `model.go` equations of motion, `pitch.go` pitch program, `aero.go` coefficient tables, `config.go`/`configfile.go` configuration, `output.go` CSV and diagnostics), CLI in `traj/main/`, atmosphere in `traj/atmosphere/`. `traj/rocket.json` is the vehicle config, `traj/optimize.py` is the CMA-ES pitch-program driver and `traj/plot_trajectory.py` renders charts.

`main.py` owns the physical fields of `traj/rocket.json`. Do not hand-edit masses, burn times, specific impulses or motor diameters there — change `main.py` and run `uv run python main.py --write-traj-config`. The pitch arcs, `t_vertical` and `limits` are optimizer output and must be preserved.

## Build, Test, and Development Commands

- `uv run python main.py` prints the main Typst calculation output; add `--write-traj-config` to resync the simulator config.
- `uv run python preliminary.py` prints the preliminary burn-rate and geometry tables.
- `cd traj && go build ./... && go test ./...` builds and tests the simulator.
- `cd traj && go run ./main -config=rocket.json` writes `out/traj.csv` and prints the §4.4 diagnostics.
- `cd traj && uv run python optimize.py` tunes the pitch program and writes `out/best.json`.
- `uv run pyright` type-checks every Python file (pyright, `standard` mode; configured in `pyproject.toml`).

Aerodynamics are currently zeroed: the CFD table was removed, and the simulator runs drag-free unless `-aero=<averages.csv>` is supplied. See README.md.

## Coding Style & Naming Conventions

Use Python 3.11 with 4-space indentation and clear type hints for shared helpers. Keep `utils.py` stateless and computation-focused. In `main.py`, preserve the existing split: `calc_*` functions return data and `emit_*` functions print Typst. Use uppercase names for module constants, `snake_case` for functions and variables, and preserve Cyrillic labels in emitted Typst where they are part of the report notation. Go code follows standard `gofmt`.

## Testing Guidelines

`traj/pitch_test.go` covers the pitch program; run it with `go test ./...`. There is no Python test suite; `uv run pyright` is the static check that stands in for one. For changes to calculations, verify by running `uv run python main.py` and diffing the stdout against the previous output — it should change only where you intended. Sanity-check simulator results against expected ranges (burnout velocity, apogee, constraint margins) before reporting success.

## Commit & Pull Request Guidelines

Use conventional commits (`feat:`, `fix:`, `refactor:`, `docs:`, `chore:`) with short imperative summaries, scoped to a single logical change. PRs should describe the calculation or trajectory workflow affected and list the verification commands run.

## Agent-Specific Instructions

When using shell commands in this workspace, prefix them with `rtk` as required by the local agent configuration. Do not commit regenerable output: the compiled simulator binary, `traj/out/traj.csv` and `outcmaes/` are gitignored.
