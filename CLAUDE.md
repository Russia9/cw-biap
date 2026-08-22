# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

Coursework for BIAP (ballistic/thrust design of a solid-fuel three-stage rocket). Four artifacts in dependency order:

1. Python sizing scripts print Typst math blocks and table rows on **stdout** — nothing is written to a document.
2. Those snippets are pasted into the report by hand; `archive.typ` is the in-repo snapshot of it.
3. `rocket.scad` turns the same dimensions into STLs, which `openfoam/` meshes and sweeps into the aerodynamic coefficient table.
4. The Go simulator under `traj/` flies the resulting design against that table and checks it against the §4.4 constructive-ballistic limits.

Design point at HEAD: m₀ = 29 724 kg, full range 12 418 km, burnout V = 7362 m/s, H = 181.5 km, θ = 20.2°.

## Running scripts

```bash
uv run python main.py        # thrust/weights/geometry → Typst math blocks + tables
uv run python preliminary.py # burn-rate and l_z/alpha_dv preliminary tables
uv run python main.py --write-traj-config  # resync traj/rocket.json with main.py
uv run pyright               # type-check all Python (standard mode)

cd traj
go build ./... && go test ./...              # build and test the simulator
go test -run TestCosinePitchSegmentsChainContinuously ./...  # single test
go run ./main -config=rocket.json -aero=../openfoam/results/averages.csv  # → out/traj.csv + §4.4 diagnostics
go run ./main -config=rocket.json -aero=... -metrics   # one JSON line, what the optimizer reads
uv run python optimize.py --aero ../openfoam/results/averages.csv  # CMA-ES pitch → out/best.json
uv run python plot_trajectory.py [out/traj.csv]        # charts from a trajectory CSV

cd ..                                        # repo root
make stls                                    # rocket.scad -> the four CFD STLs
make png                                     # preview render
uv run python main.py --write-scad-params    # regenerate rocket-params.scad
uv run python openfoam/gen_case.py --part all --regime supersonic --Ma 4 --alpha 0
uv run python openfoam/sweep.py --dry-run    # print the 81-case queue, run nothing
```

`uv run ruff check .` is clean on the live code; the only violations are in the archived `preliminary-29700/scripts/`.

## Architecture

Four layers on the Python side:

1. **`assets/*.csv`** — raw data tables digitized from textbook charts:
   - `fuels.csv` — fuel properties (ρ, R, k, T, P_ud, burn-rate law, Al%)
   - `materials.csv` (`load_materials`), `table-2.1.csv` (`load_trajectory`, burnout-trajectory reference)
   - `chart-4-26-alpha.csv`, `chart-4-27-l.csv` — digitized nomogram curves for bilinear interpolation
   - `chart-3-5-*.csv`, `chart-3-6-*.csv`, `table-k-k0.csv` — additional reference tables

2. **`utils.py`** — pure physics functions + CSV-backed interpolation. All functions are stateless; they accept SI/practical units and return floats. Chart lookups use `np.interp` with bilinear interpolation across curves.

3. **`typst.py`** — pure presentation/rendering layer, kept separate from physics: `emit()` (wrap a math body), `fmt()`, `section()`, `stage_header()`, `param_row()`/`param_rows()`/`param_table()` (emit Typst `table(...)` rows with a `[Параметр]` column plus per-stage columns).

4. **`main.py` / `preliminary.py`** — calculation scripts. Each defines a `STAGES` list / constants at the top and a `main()` that prints Typst snippets. In `main.py`, keep the split: `calc_*` functions are pure (return `Thrust`/`Weight`/`Subrockets` NamedTuples), and `emit_*` functions do the printing — don't mix computation into the emit functions.

`main.py`'s module-level constants are the design knobs, and each one carries a comment recording why it holds its value (see the `K_V = 1.198` block on the loss-factor band, and the `STAGES` block on λ_з). Preserve that reasoning when changing a value.

## The report (`archive.typ`)

`archive.typ` is a tracked snapshot of the Typst report with the script output already pasted in. Two things to know:

- **It does not compile standalone**: it references `minuteman-1.png`, which is not in the repo, and expects fonts that may not be installed.
- **It lags `main.py`**: e.g. its motor-geometry table still carries d_з1 = 1.535 m and L_1 = 7.65 m against the current 1.545 m / 7.71 m. After changing a calculation, re-paste the affected block. `section()` prints a `// ===== <title> =====` marker at each section boundary in stdout to make the blocks easy to cut; those markers are not present in `archive.typ`.

## Trajectory layer (`traj/`)

Go package `traj` — planar spherical-Earth RK4 (`sim.go` integration, `model.go` equations of motion, `pitch.go` programmed pitch, `aero.go` coefficient tables, `config.go`/`configfile.go` config, `output.go` CSV + diagnostics), GOST 4401-81 atmosphere in `atmosphere/`, CLI in `main/`, plus `optimize.py` (CMA-ES) and `plot_trajectory.py`.

The model is 3-DOF with a **programmed** pitch angle: `AeroForces` returns a pitch moment `Mz`, but `activeAccel` discards it (`model.go:81`) — `Mz` only reaches the CSV and diagnostics. Consequently `Lref` in `config.go` scales nothing that affects the trajectory.

### `main.py` owns the physical fields of `rocket.json`

`payload_mass` and each stage's `m0`, `m_fuel`, `burn_time`, `isp_sl`, `isp_vac`, `dm` come from `main.py`. Never hand-edit those; change `main.py` and run `--write-traj-config`. A bare `main.py` run warns on stderr when they drift. The `t_vertical`, `pitch` and `limits` fields are optimizer output and are preserved by the writer.

### Aerodynamics must be passed explicitly

The CFD table is `openfoam/results/averages.csv` (part keys `all`, `stage2up`, `stage3up`, `head`, with fallbacks in `aero.go`), and it must be passed: `-aero=<averages.csv>` for the simulator, `--aero=<averages.csv>` for `optimize.py`. Without the flag `traj.ZeroAero()` supplies an empty table, so drag, lift and pitch moment are all zero — the force path in `model.go` is unchanged, it just multiplies by zero.

The pitch program in `rocket.json` was optimized *with* the table, so a drag-free run does not reproduce the reported result: the same config reports 12 747 km instead of 12 418 km and **violates two §4.4 limits** (max |α| 1.64° vs 1.50° subsonic, 16.0° vs 10.0° supersonic). Lift is what turns the vehicle inside those |α| limits, so removing the table is not a conservative simplification.

### The design sits on the §4.4 boundary

The pitch program was optimized flat against the constraints, so there is no robustness margin except on q (76 kPa of 120 kPa). Any change to masses, impulses, the aero table or the reference area makes it infeasible rather than merely suboptimal. Re-run `optimize.py` and check the diagnostics before reporting a result.

**Currently both |α| limits are exceeded**: 1.5013° against 1.50 subsonic and 10.036° against 10.00 supersonic, at 12 427 km. This surfaced when `RrefAll` was corrected from 0.795 to the CFD's actual 0.79 — the old value inflated `Aref` by 1.25 %, and the extra lift was holding α inside the limits. The violation is therefore pre-existing and was masked by the wrong reference area, not introduced by it. Fixing it means re-optimizing the pitch program (warm-started from `rocket.json`, `--maxiter` ≥ 1500, several seeds).

### Optimizer loop is not automatic

`optimize.py` builds `out/traj-sim`, searches, and writes the winner to **`out/best.json`** — it never writes `rocket.json`. Promoting a result means copying `t_vertical` and the per-stage `pitch` arrays from `out/best.json` into `traj/rocket.json` yourself. Settings that mattered on this landscape: `--maxiter` ≥ 1500 (the 150 default is severely under-converged), best-of-N over several seeds, and keeping `--h-opt` equal to `--h-final` at 0.1 s — at h = 0.5 the α peaks read ~0.1° low, enough to make an infeasible solution look feasible during the search.

### Reference geometry in `config.go`

`RrefAll = 0.79` and `Lref = 18.243` are the bounding box of `rocket.stl`, measured with `openfoam/gen_case.py`'s `stl_bbox()` — the same function that writes `Aref`/`lRef` into each CFD case, so the simulator and the coefficients share one reference by construction. Refresh both after any change to `main.py`'s d_(м i)/L_i or to `rocket.scad`; the recipe is in the comment there.

`Lref` exceeds the 18.24 m stack height by the 3 mm `eps` overhang `rocket.scad` uses to fuse stacked sections into one solid. It scales only `Mz`, which `activeAccel` discards, so its exact value is cosmetic. `RrefAll` is not: it scales every aerodynamic force.

## Geometry and CFD (`rocket.scad`, `openfoam/`)

The aerodynamic chain is `main.py` → `rocket-params.scad` → `rocket.scad` → STL → `gen_case.py` → snappyHexMesh → `averages.csv` → `traj/aero.go`.

`rocket.scad` is the **outer mold line only** — the surface the flow sees. No motor internals, no bores, no charge cavities, and no nozzles (nozzle bells broke snappyHexMesh and were dropped in `a540884`). Every section is a solid of revolution unioned with an `eps` overlap, so each STL exports as one genus-0 manifold shell.

`main.py` owns the dimensions via `rocket-params.scad` (`--write-scad-params`, same contract as `--write-traj-config`: stderr-only warnings, clean stdout). `rocket.scad` holds the shape logic plus the structural constants `main.py` does not compute — the interstage, adapter, nav-module and warhead dimensions.

**Stage 3's outer diameter is stage 2's, not its motor diameter.** Condition (3.43) requires d_м ≥ d_a(1+√2) to seat four nozzles; stage 3 fails it (0.83 < 1.07), so per `archive.typ:636` it keeps its narrow motor and gains an external shell at d_м2. `rocket.scad` encodes this as `d_ext = [d_m[0], d_m[1], d_m[1]]`. Stages 2 and 3 therefore mate flush — there is no 2/3 interstage. This is why the CFD meshed stage 3 at 1.17 m: it was never a mistake, and the existing `stage3up`/`head` coefficients are valid to ~1 %.

The `Makefile` maps parts to STLs the way `gen_case.py` expects: `all → rocket.stl`, `stage2up`, `stage3up`, `head`. In the STL pattern rule `$(SCAD)` must stay the **first** prerequisite — the recipe passes `$<` to OpenSCAD, and putting `$(PARAMS)` first would render the parameter file instead.

Case generation needs only `make` and `openscad`; meshing and solving need OpenFOAM v2512 with HiSA. `sweep.py --dry-run` writes nothing. **Never run `plot_coeffs.py` without a complete sweep** — it rebuilds `averages.csv`, the only surviving CFD result, and a partial set of inputs silently truncates it.

## Output format

Scripts emit Typst source, not plain text. Inline strings use Typst math syntax with Cyrillic labels (e.g. `"уд"`, `"ст"`). The `emit()` helper (in `typst.py`) wraps a body in a `#math.equation(numbering: none, block: true, $ … $)` call. Only substituted calculations and tables are emitted — the symbolic formulas live in the report document.

## CSV chart format

Multi-curve charts store each curve as a pair of columns (X, Y). Row 0 holds curve labels; row 1 holds `X, Y` headers. Use `pd.read_csv(path, header=1)` for data and a separate `pd.read_csv(path, header=None, nrows=1)` pass to read labels — see `alpha_dv()` in `utils.py` for the pattern.

## `preliminary-29700/`

A tracked record of the mass-reduction study that produced the current design (29 724 kg / 12 418 km, down from 39 523 kg). Its `main.py` deltas and 14-arc pitch program are **already applied at HEAD**, so treat it as history plus a list of open items (λ_2/λ_3 below their bands, the stage-3 nozzle condition (3.43) failing, the zero-margin §4.4 limits). Its own README calls the folder untracked; that is no longer accurate. It is excluded from `pyright` and is the only source of `ruff` violations.

## Python environment

Use `uv` (see global CLAUDE.md). Python 3.11, dependencies: `numpy`, `pandas`, `pandas-stubs`, `cma` and `matplotlib` (the latter two for `traj/`). Go 1.26 for the simulator.
