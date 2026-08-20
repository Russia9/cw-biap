# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

Coursework for BIAP (ballistic/thrust design of a solid-fuel three-stage rocket). Computations produce Typst-formatted math equations and table rows for a report — output is printed to stdout and pasted into the document. A Go trajectory simulator under `traj/` checks the resulting design against the §4.4 constructive-ballistic limits.

## Running scripts

```bash
uv run python main.py        # thrust/specific-impulse calculations → Typst math blocks
uv run python preliminary.py # burn-rate and l_z/alpha_dv preliminary tables
uv run python main.py --write-traj-config  # resync traj/rocket.json with main.py
uv run pyright               # type-check all Python (standard mode)

cd traj
go build ./... && go test ./...            # build and test the simulator
go run ./main -config=rocket.json -aero=../openfoam/results/averages.csv  # → out/traj.csv + §4.4 diagnostics
uv run python optimize.py --aero ../openfoam/results/averages.csv  # CMA-ES pitch → out/best.json
uv run python plot_trajectory.py           # charts from out/traj.csv
```

## Architecture

Four layers on the Python side:

1. **`assets/*.csv`** — raw data tables digitized from textbook charts:
   - `fuels.csv` — fuel properties (ρ, R, k, T, P_ud, burn-rate law, Al%)
   - `materials.csv` (`load_materials`), `table-2.1.csv` (`load_trajectory`, burnout-trajectory reference)
   - `chart-4-26-alpha.csv`, `chart-4-27-l.csv` — digitized nomogram curves for bilinear interpolation
   - `chart-3-5-*.csv`, `chart-3-6-*.csv`, `table-k-k0.csv` — additional reference tables

2. **`utils.py`** — pure physics functions + CSV-backed interpolation. All functions are stateless; they accept SI/practical units and return floats. Chart lookups use `np.interp` with bilinear interpolation across curves.

3. **`typst.py`** — pure presentation/rendering layer, kept separate from physics: `eq()` (wrap a math body), `fmt()`, `section()`, `param_row()`/`param_table()` (emit Typst `table(...)` rows with a `[Параметр]` column plus per-stage columns).

4. **`main.py` / `preliminary.py`** — calculation scripts. Each defines a `STAGES` list / constants at the top and a `main()` that prints Typst snippets. In `main.py`, keep the split: `calc_*` functions are pure (return `Thrust`/`Weight`/`Subrockets` NamedTuples), and `emit_*` functions do the printing — don't mix computation into the emit functions.

## Trajectory layer (`traj/`)

Go package `traj` (planar spherical-Earth RK4, GOST 4401-81 atmosphere, programmed pitch angle) with the CLI in `traj/main/`, plus `optimize.py` (CMA-ES over the pitch program) and `plot_trajectory.py`.

`main.py` is the single source of truth for the physical fields of `traj/rocket.json` — `payload_mass` and each stage's `m0`, `m_fuel`, `burn_time`, `isp_sl`, `isp_vac`, `dm`. Never hand-edit those; change `main.py` and run `--write-traj-config`. A bare `main.py` run warns on stderr when they drift. The `t_vertical`, `pitch` and `limits` fields are optimizer output and are preserved by the writer.

**Aerodynamics come from `openfoam/results/averages.csv`** and must be passed explicitly: `-aero=<averages.csv>` for the simulator, `--aero=<averages.csv>` for `optimize.py`. Without the flag `traj.ZeroAero()` supplies an empty table and drag, lift and pitch moment are all zero — the force path in `model.go` is unchanged, it just multiplies by zero. The pitch program in `rocket.json` is optimized *with* the table, so a run without `-aero` will not reproduce the reported range.

Aerodynamics are not a pure penalty here: lift is what turns the vehicle inside the §4.4 |α| limits. Restoring the table moved burnout from 35.8° to 28.5° and the range from 8118 to 8529 km, because the flatter climb saves more gravity loss than the drag costs.

`Aref`/`Lref` in `config.go` (RrefAll = 0.795 m, Lref = 22.393 m) are the CFD run's reference geometry, not the current design's — d_м1 is now 1.53 m, so the table over-predicts both drag and lift by roughly 8 % until the CFD is re-run.

## Output format

Scripts emit Typst source, not plain text. Inline strings use Typst math syntax with Cyrillic labels (e.g. `"уд"`, `"ст"`). The `eq()` helper (in `typst.py`) wraps a body in a `#math.equation(numbering: none, block: true, $ … $)` call.

## CSV chart format

Multi-curve charts store each curve as a pair of columns (X, Y). Row 0 holds curve labels; row 1 holds `X, Y` headers. Use `pd.read_csv(path, header=1)` for data and a separate `pd.read_csv(path, header=None, nrows=1)` pass to read labels — see `alpha_dv()` in `utils.py` for the pattern.

## Python environment

Use `uv` (see global CLAUDE.md). Python 3.11, dependencies: `numpy`, `pandas`, `pandas-stubs`, `cma` and `matplotlib` (the latter two for `traj/`). Go 1.26 for the simulator.
