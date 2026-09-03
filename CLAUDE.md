# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

Coursework for BIAP (ballistic/thrust design of a solid-fuel three-stage rocket). Four artifacts in dependency order:

1. Python sizing scripts print Typst math blocks and table rows on **stdout** — nothing is written to a document.
2. Those snippets are pasted into the report by hand; `archive.typ` is the in-repo snapshot of it.
3. `rocket.scad` turns the same dimensions into STLs, which `openfoam/` meshes and sweeps into the aerodynamic coefficient table.
4. The Go simulator under `traj/` flies the resulting design against that table and checks it against the §4.4 constructive-ballistic limits.

Design point at HEAD: m₀ = 29 724 kg, full range 12 749 km, burnout V = 7388 m/s, H = 169.8 km, θ = 17.5°.
The pitch program is a genuine open-loop ϑ_пр(t): 24 C¹ Hermite arcs, all three
stages `"steering": "theta"`, so α is an output of the trajectory rather than a
commanded quantity. See the 2026-09 note in `main.py`'s K_V block for why the
earlier α-framed parameterisation was needed and what replaced it.

## Running scripts

```bash
uv run python main.py        # thrust/weights/geometry → Typst math blocks + tables
uv run python preliminary.py # burn-rate and l_z/alpha_dv preliminary tables
uv run python main.py --write-traj-config  # resync traj/rocket.json with main.py
uv run python aero_tables.py  # CFD coefficients → Typst tables (α rows × M columns)
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

Go package `traj` — planar spherical-Earth RK4 (`sim.go` integration driver, `model.go` equations of motion, `pitch.go` programmed pitch, `aero.go` coefficient tables, `constants.go`/`rocket.go` constants and config types, `configfile.go` JSON parsing, `row.go` trajectory rows, `diagnostics.go` §4.4 measurement, `output.go` CSV/metrics/diagnostics printing), GOST 4401-81 atmosphere in `atmosphere/`, CLI in `main/`, plus `optimize.py` (CMA-ES) and `plot_trajectory.py`. Simulator behavior is pinned by golden tests (`sim_test.go`, `atmosphere_test.go`): a refactor must keep them byte-exact, and a deliberate physics change updates the pins in the same commit with the delta recorded.

The model is 3-DOF with a **programmed** pitch angle: `AeroForces` returns a pitch moment `Mz`, but `activeAccel` discards it — `Mz` only reaches the CSV and diagnostics. Consequently `Lref` in `constants.go` scales nothing that affects the trajectory.

### `main.py` owns the physical fields of `rocket.json`

`payload_mass` and each stage's `m0`, `m_fuel`, `burn_time`, `isp_sl`, `isp_vac`, `dm` come from `main.py`. Never hand-edit those; change `main.py` and run `--write-traj-config`. A bare `main.py` run warns on stderr when they drift. The `t_vertical`, `pitch` and `limits` fields are optimizer output and are preserved by the writer.

### Aerodynamics must be passed explicitly

The CFD table is `openfoam/results/averages.csv` (part keys `all`, `stage2up`, `stage3up`, `head`, with fallbacks in `aero.go`), and it must be passed: `-aero=<averages.csv>` for the simulator, `--aero=<averages.csv>` for `optimize.py`. Without the flag `traj.ZeroAero()` supplies an empty table, so drag, lift and pitch moment are all zero — the force path in `model.go` is unchanged, it just multiplies by zero.

The pitch program in `rocket.json` was optimized *with* the table, so a drag-free run does not reproduce the reported result: the same config reports 13 010 km instead of 12 749 km and **violates two §4.4 limits** (max |α| 1.83° vs 1.50° subsonic, 17.07° vs 10.00° supersonic). Lift is what turns the vehicle inside those |α| limits, so removing the table is not a conservative simplification — and now that the program is ϑ-framed, α is an output, so a drag-free run changes it directly.

### The design sits on the §4.4 boundary

The pitch program was optimized flat against the constraints, so there is no robustness margin except on q (76 kPa of 120 kPa). Any change to masses, impulses, the aero table or the reference area makes it infeasible rather than merely suboptimal. Re-run `optimize.py` and check the diagnostics before reporting a result.

**Every §4.4 check currently reads OK** at 12 749 km, but four of them sit within 0.2 % of their limit: |α| subsonic 1.4985/1.50, |α| supersonic 9.9887/10.00, and both pitch rates 2.9968/3.00. Only q (34 % margin) and the apogee (6 %) have real room. The |α| limits are now *derived* rather than commanded — the program steers ϑ — which makes them the sharpest test that a change has not broken anything. Two further constraints bind:

- **|α| ≤ 1.5° at any separation inside the atmosphere** (`eps_sep`), gated on q > `QSepMin` = 1 kPa rather than on altitude — separation loads are a dynamic-pressure phenomenon, and an H ≤ 94 km gate would sit within metres of the 2/3 separation and flip on sub-second timing changes. In practice only the 1/2 separation qualifies (q ≈ 15 kPa); the 2/3 one is exempt at q ≈ 0. Measured at 1.4973/1.50.
- **§4.1's 94 km ascending crossing must fall in the stage-2 burn.** Penalised on `CrossUpMargin` (seconds inside that window, negative outside) rather than on the stage number, which is a step with no gradient to follow. The optimum converges onto `CROSS_MARGIN_S` = 0.5 s by construction, keeping it clear of the staging discontinuity.

Any change to masses, impulses, the aero table or the reference area makes this infeasible rather than merely suboptimal. Re-optimize (warm-started from `rocket.json`, `--maxiter` ≥ 3000, several seeds at σ0 = 0.02…0.10) and read the full diagnostics before reporting a result.

### Optimizer loop is not automatic

`optimize.py` builds a per-run simulator binary under `out/` (removed on exit; concurrent seed runs are safe), searches, and writes the winner to **`out/best.json`** — it never writes `rocket.json`. The CMA-ES state is checkpointed to `out/<stem>-cma.pkl` every 10 iterations; `--resume <pkl>` continues an interrupted or finished search (pass a larger `--maxiter` to extend). Promoting a result means copying `t_vertical` and the per-stage `pitch` arrays from `out/best.json` into `traj/rocket.json` yourself (`out/` is untracked — regenerable output). Settings that mattered on this landscape: `--maxiter` ≥ 1500 (the 150 default is severely under-converged), best-of-N over several seeds, and keeping `--h-opt` equal to `--h-final` at 0.1 s — at h = 0.5 the α peaks read ~0.1° low, enough to make an infeasible solution look feasible during the search.

### Reference geometry in `constants.go`

`RrefAll = 0.785` and `Lref = 16.393` are the bounding box of `rocket.stl`, measured with `openfoam/gen_case.py`'s `stl_bbox()` — the same function that writes `Aref`/`lRef` into each CFD case, so the simulator and the coefficients share one reference by construction. Refresh both after any change to `main.py`'s d_(м i)/L_i or to `rocket.scad`; the recipe is in the comment there.

`Lref` exceeds the 16.39 m stack height by the 3 mm `eps` overhang `rocket.scad` uses to fuse stacked sections into one solid. It scales only `Mz`, which `activeAccel` discards, so its exact value is cosmetic. `RrefAll` is not: it scales every aerodynamic force — which is why the (3.44) shortening was safe to land without a re-sweep: L_i reaches no diameter, so `RrefAll` and therefore `Aref` did not move.

## Geometry and CFD (`rocket.scad`, `openfoam/`)

The aerodynamic chain is `main.py` → `rocket-params.scad` → `rocket.scad` → STL → `gen_case.py` → snappyHexMesh → `averages.csv` → `traj/aero.go`.

`rocket.scad` is the **outer mold line only** — the surface the flow sees. No motor internals, no bores, no charge cavities, and no nozzles (nozzle bells broke snappyHexMesh and were dropped in `a540884`). Every section is a solid of revolution unioned with an `eps` overlap, so each STL exports as one genus-0 manifold shell.

`main.py` owns the dimensions via `rocket-params.scad` (`--write-scad-params`, same contract as `--write-traj-config`: stderr-only warnings, clean stdout). `rocket.scad` holds the shape logic plus the structural constants `main.py` does not compute — the interstage, adapter, nav-module and warhead dimensions.

**Every stage now flies at its own motor diameter.** The seating condition

    d_м >= d_a (1 + sqrt(2)) + 2 l_a sin(δ_с)

is satisfied on the motor itself for all three stages (margins 378 / 0.6 / 15 mm at the prototype's δ_с = 8° / 6° / 4°), so the external shell that used to wrap a too-narrow stage 3 is retired and `rocket.scad` sets `d_ext = d_m`. `main.py`'s `STAGES` note explains how stages 2 and 3 buy that fit with their λ_з/ρ_т·u pair. The swing term is the bell alone, `l_a sin δ_с` about the throat — `l_дк` does not enter, which is what leaves stage 2 its 0.6 mm at 6°.

The `Makefile` maps parts to STLs the way `gen_case.py` expects: `all → rocket.stl`, `stage2up`, `stage3up`, `head`. In the STL pattern rule `$(SCAD)` must stay the **first** prerequisite — the recipe passes `$<` to OpenSCAD, and putting `$(PARAMS)` first would render the parameter file instead.

**`openfoam/results/averages.csv` is stale as of the (3.44) change.** L_i dropped 1.44 m in total (7.70/3.95/2.18 → 7.14/3.51/1.75) and the STLs were regenerated, but the coefficients were solved on the old ~18 m mold line and have not been re-swept. This is safe to carry, not free: `RrefAll` is unchanged, so `Aref` still matches what `gen_case.py` non-dimensionalised by and the retained C_x/C_y remain dimensionally consistent — the trajectory is bit-identical, verified by diffing `-metrics`. What is unbounded is fidelity. Less wetted area means the real C_x is lower, so range is pessimistic; but afterbody length also shifts C_y, and the |α| margins sit at 0.1 % of the §4.4 limits, so "conservative on range" does **not** imply "conservative on the α constraints". Only a re-sweep closes that.

Case generation needs only `make` and `openscad`; meshing and solving need OpenFOAM v2512 with HiSA. `sweep.py --dry-run` writes nothing. **Never run `plot_coeffs.py` without a complete sweep** — it rebuilds `averages.csv`, the only surviving CFD result, and a partial set of inputs silently truncates it.

## Output format

Scripts emit Typst source, not plain text. Inline strings use Typst math syntax with Cyrillic labels (e.g. `"уд"`, `"ст"`). The `emit()` helper (in `typst.py`) wraps a body in a `#math.equation(numbering: none, block: true, $ … $)` call. Only substituted calculations and tables are emitted — the symbolic formulas live in the report document.

## CSV chart format

Multi-curve charts store each curve as a pair of columns (X, Y). Row 0 holds curve labels; row 1 holds `X, Y` headers. Use `pd.read_csv(path, header=1)` for data and a separate `pd.read_csv(path, header=None, nrows=1)` pass to read labels — see `alpha_dv()` in `utils.py` for the pattern.

## `preliminary-29700/`

A tracked record of the mass-reduction study that produced the current design (29 724 kg / 12 418 km, down from 39 523 kg). Its `main.py` deltas and 14-arc pitch program are **already applied at HEAD**, so treat it as history plus a list of open items (λ_2/λ_3 below their bands, the stage-3 nozzle condition (3.43) failing, the zero-margin §4.4 limits). Its own README calls the folder untracked; that is no longer accurate. It is excluded from `pyright` and is the only source of `ruff` violations.

## Python environment

Use `uv` (see global CLAUDE.md). Python 3.11, dependencies: `numpy`, `pandas`, `pandas-stubs`, `cma` and `matplotlib` (the latter two for `traj/`). Go 1.26 for the simulator.
