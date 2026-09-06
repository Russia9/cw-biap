# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

Coursework for BIAP (ballistic/thrust design of a solid-fuel three-stage rocket).

**Five modules, each writing only into its own directory.** Data flows one way:

```
report/ ──> openscad/ ──> openfoam/ ──> trajectory/
   │                          │              ▲
   └────> optimizer/input/ ───┴──────────────┘
```

1. `report/` — Python sizing scripts print Typst math blocks and table rows on
   **stdout**; nothing is written to a document. Pasted into the report by hand;
   `report/archive.typ` is the in-repo snapshot of it.
2. `openscad/` — `rocket.scad` turns the same dimensions into STLs.
3. `openfoam/` — meshes and sweeps them into the aerodynamic coefficient table.
4. `trajectory/` — the Go simulator flies the design against that table.
5. `optimizer/` — `main.py` searches the pitch program with CMA-ES against the simulator.

Every generated artifact is gitignored by its own module's `.gitignore`
(`openscad/out/`, `openfoam/input/` + `out/`, `optimizer/input/` + `out/`,
`trajectory/out/`); the root `.gitignore` carries only repo-wide rules.
**A fresh clone therefore has no STLs, no CFD table and no simulator config** —
run `make stls && make openfoam-input`, `report/main.py --write-traj-config`, and
either re-run the sweep or obtain `openfoam/out/averages.csv` out of band.

Design point at HEAD (`uv run python report/main.py`): m₀₁ = 29 708 kg, stage masses
29 708 / 8 546 / 2 419 kg, burn times 66.2 / 40.9 / 35.1 s, motor diameters
1.57 / 1.25 / 1.09 m, stage lengths 7.14 / 3.51 / 1.75 m. The prototype launch
mass the assignment fixes is 29 724 kg; `K_V = 1.187` is the knob tuned to hit it.

**The trajectory layer is mid-rewrite.** The earlier simulator (per-stage
Hermite pitch arcs, §4.4 diagnostics, CSV rows, `-metrics`, a CMA-ES driver) was
replaced at `90701cf` by a smaller integrator; `optimizer/main.py` is a new,
smaller CMA-ES driver written against it. Range,
apogee and §4.4 numbers quoted in `main.py`'s comments and in `archive.typ` come
from the **old** simulator and cannot currently be reproduced — see
[Trajectory layer](#trajectory-layer-trajectory) and [Known drift](#known-drift).

## Running scripts

Every path below is anchored to the script file, so these work from anywhere;
the repo root is the conventional place to stand.

```bash
uv run python report/main.py        # thrust/weights/geometry → Typst math blocks + tables
uv run python report/preliminary.py # burn-rate and l_z/alpha_dv preliminary tables
uv run python report/aero_tables.py # CFD coefficients → Typst tables (α rows × M columns)
uv run python report/main.py --write-scad-params  # → openscad/rocket-params.scad
uv run python report/main.py --write-traj-config  # → optimizer/input/rocket.json
uv run pyright               # type-checks report/ — clean
uv run ruff check .          # clean

make stls                    # → openscad/out/{all,stage2up,stage3up,head}.stl
make png                     # → openscad/out/rocket.png
make openfoam-input          # stage those meshes into openfoam/input/
uv run python openfoam/gen_case.py --part all --regime supersonic --Ma 4 --alpha 0
uv run python openfoam/sweep.py --dry-run    # print the 96-case queue, run nothing

cd trajectory
go build ./... && go test ./...                      # build and test
go test -run TestGOSTAtmosphere ./atmosphere/        # a single test
go run ./cmd/main ../optimizer/input/rocket.json ../openfoam/out/averages.csv out/traj.csv
go run ./cmd/plot ../optimizer/input/rocket.json     # → out/pitch.png

cd ..
uv run python optimizer/main.py --maxiter 3   # smoke-test the pitch search → optimizer/out/best.json
```

Both Go binaries take their config path as a positional argument and create
`trajectory/out/` themselves.

## Architecture

Four layers inside `report/`:

1. **`report/assets/*.csv`** — raw data tables digitized from textbook charts:
   - `fuels.csv` — fuel properties (ρ, R, k, T, P_ud, burn-rate law, Al%)
   - `materials.csv` (`load_materials`), `table-2.1.csv` (`load_trajectory`, burnout-trajectory reference)
   - `chart-4-26-alpha.csv`, `chart-4-27-l.csv` — digitized nomogram curves for bilinear interpolation
   - `chart-3-5-*.csv`, `chart-3-6-*.csv`, `table-k-k0.csv` — additional reference tables

2. **`report/utils.py`** — pure physics functions + CSV-backed interpolation. All functions are stateless; they accept SI/practical units and return floats. Chart lookups use `np.interp` with bilinear interpolation across curves. Every
   default is anchored to `ASSETS = Path(__file__).parent / "assets"`, so the
   scripts do not depend on the working directory.

3. **`report/typst.py`** — pure presentation/rendering layer, kept separate from physics: `emit()` (wrap a math body), `fmt()`, `section()`, `stage_header()`, `param_row()`/`param_rows()`/`param_table()`/`coeff_table()`.

4. **`report/main.py` / `preliminary.py` / `aero_tables.py`** — calculation scripts. Each defines a `STAGES` list / constants at the top and a `main()` that prints Typst snippets. In `main.py`, keep the split: `calc_*` functions are pure (return `Thrust`/`Weight`/`Subrockets` NamedTuples), and `emit_*` functions do the printing — don't mix computation into the emit functions.

`main.py`'s module-level constants are the design knobs, and each one carries a comment recording why it holds its value (see the `K_V = 1.187` block on the loss-factor band, and the `STAGES` block on the l̄_з/ρ_т·u pair that buys the nozzle fit). Preserve that reasoning when changing a value.

## The report (`report/archive.typ`)

`report/archive.typ` is a tracked snapshot of the Typst report with the script output already pasted in. Three things to know:

- **It does not compile standalone**: it references `minuteman-1.png`, which is not in the repo, and expects fonts that may not be installed.
- **It lags `main.py` in places.** 111 of the 129 `#math.equation` blocks `main.py` prints appear in it verbatim; the 18 that don't are the nozzle/length block — it still carries L = 7.70 / 3.95 / 2.18 m against the current 7.14 / 3.51 / 1.75, and δ_с1 = 6° against 8°. To find drift, diff the emitted blocks against the file:

  ```bash
  # prints every emitted block that is not in archive.typ verbatim (18 today)
  cd report && uv run python main.py 2>/dev/null \
    | grep '^#math.equation' | grep -vxFf archive.typ
  ```

- `section()` prints a `// ===== <title> =====` marker at each section boundary in stdout to make the blocks easy to cut; those markers are not present in `archive.typ`.

## Trajectory layer (`trajectory/`)

Go module **`traj`** (the import paths are `traj`, `traj/aero`, `traj/atmosphere` — the module name did not follow the `traj` → `trajectory` directory rename in `7dc0abd`). Go 1.27.

- `model.go` — equations of motion and `InitModel`, which returns a `na.FuncSystem` of four derivatives over the state `[Vx, Vy, x, y]`.
- `pitch.go` — the programmed pitch ϑ(t): a flat list of arcs, each moving ϑ from the previous arc's `theta_deg` to its own by `t_end`. Two shapes: `"cos"`, where `k` is an exponent and every arc has zero slope at both ends; and `"hermite"`, a cubic where `k` is the arc's **exit slope in deg/s** and the entry slope is read from the previous arc via `exitSlope`, so joints are C¹. A cos arc placed after a hermite one kinks, since cos always enters flat.
- `rocket.go` — config types and `LoadRocketJSON`, which also validates the program (no zero-length arcs, ordered `t_end`, `k >= 1` so joints are not discontinuous, and no powered-but-uncontrolled stage).
- `constants.go` — Earth/atmosphere constants and the aerodynamic reference.
- `aero/aero.go` — loads `averages.csv` into bilinear interpolants per part, mirroring each row to −α (Cd even, Cl and CmPitch odd).
- `atmosphere/` — GOST 4401-81, valid to `HBoundary` = 94 km, vacuum above.
- `cmd/main` — integrate the trajectory and write it as CSV
  (`t,stage,m,x,y,Vx,Vy,V,r,h,q,Ma,pitch,flightAngle,attack`; angles in rad,
  `stage` 0-based, each staging instant written twice, `attack` = 0 on an
  uncontrolled stage); `cmd/plot` — plot the pitch program alone.

RK4, bilinear interpolation and the `Point3D`/`FuncSystem` types come from the
external `github.com/Russia9/numerical-analysis`; plotting from `gonum.org/v1/plot`.

### What the model does and does not do

3-DOF, planar, spherical Earth, no rotation. Thrust is applied along the
programmed pitch, drag along −V and lift normal to it. `Aref` scales both.

- **`Lref` is dead code**: declared, never read (there is no pitch moment in the
  model). `RrefAll = 0.785` *is* load-bearing — it scales every aerodynamic
  force. Both are the `all.stl` bounding box; refresh after any change to
  `report/main.py`'s d_(м i)/L_i or to `openscad/rocket.scad`:

  ```bash
  make stls && uv run python -c "import sys; sys.path.insert(0,'openfoam');
  from pathlib import Path; from gen_case import stl_bbox
  print(stl_bbox(Path('openscad/out/all.stl')))"   # -> (16.393, 0.785)
  ```

- **`CmPitch` is loaded, interpolated, and never consumed.** There is no pitch
  moment in `accel()` and no rotational DOF.
- **α is derived, not integrated**: `alphaDeg = ϑ_пр(t) − θ(t)` for a controlled
  stage, and is forced to zero for an uncontrolled one. Nothing bounds it.
- **The 4th "stage"** is the payload coast: `powered: false`,
  `controlled: false`, `burn_time: 3000` as a carry-on horizon. `stage()`
  carries the last stage past its burn time regardless.
- **The aero table is a required positional argument**, not an optional flag —
  `go run ./cmd/main <rocket.json> <averages.csv> <out.csv>`. There is no drag-free mode.
- Integration stops at H = 0 (`h < 0` requests a half-step), fixed h = 0.1 s.
- **Past the last pitch arc the final angle is HELD.** Evaluating the arc beyond
  its own `t_end` would put the normalised time above 1, and `cos(pi*x^k)` then
  swings the vehicle for the whole coast. A config with no `pitch` block is
  rejected by `LoadRocketJSON` rather than flown at ϑ = 0.

### The config is generated, and split in ownership

`optimizer/input/rocket.json` is the only config; `trajectory/rocket.json` is
gone. Ownership is split down the middle and the split is the point:

- **`report/main.py` owns the whole `stages` array** — all four entries,
  including `part`/`powered`/`controlled` and the payload coast. Never hand-edit
  it; change `main.py` and rerun `--write-traj-config`. A bare run warns on
  stderr, listing every drifted field.
- **The `pitch` block is optimizer OUTPUT.** `main.py` never writes it and
  preserves any block already in the file verbatim, so a re-sync cannot destroy
  a search result. A file created from scratch has no pitch and will not load —
  that is deliberate: you cannot fly without a program.

The file is gitignored, so it does not exist in a fresh clone.

## Optimizer (`optimizer/main.py`)

One script, run as `uv run python optimizer/main.py`. It reads the stages from
`optimizer/input/rocket.json`, the starting pitch block from the tracked
`optimizer/seed.json` — a config in the simulator's form, only its `pitch` key
is read, so a `best.json` serves as a seed too; the arc count **inside the powered
window** fixes the problem size — builds
`trajectory/cmd/main` once into `optimizer/out/sim`, and runs pycma over the
vector `[theta_deg × N | w × N-1 | k × N | t_start]`. Every arc it emits is
`"hermite"`, so `k` is an exit slope in deg/s boxed at ±3 by the rate limit.
**Seeds must already be hermite** — there is no cos conversion, a cos seed exits with
a message, and the pre-hermite `best.json` files are no longer usable as seeds.

**The program is pinned to the powered window.** ϑ(t) steers nothing once the last
stage burns out — `model.go` reads `Pitch()` only inside the `st.Powered` thrust
branch and the `st.Controlled` α branch, and the payload coast is neither — so an
arc spent there is a dead search dimension. The `w` block is therefore weights, not
durations: arcs tile `[t_start, t_powered]` in proportion to `w`, with the last
weight pinned at 1 (only ratios matter, and a free scale would be a flat direction).
`t_powered` is the cumulative burn time through the last powered stage, 142.2 s at
HEAD. Ordering and uniqueness hold by construction because `w > 0`.

On load the seed is trimmed to the arcs that *start* before burnout, plus the one
straddling it. Counting starts rather than ends is what makes a re-seed lossless:
a program this script wrote already ends at `t_powered`, and an end-based test would
shed one arc per run. `--arcs M` resamples the trimmed program onto M evenly spaced
arcs, reading θ and k off the old breakpoints — that is how a `best.json` from before
this parametrization (5 in-window arcs of 20) is restored to full resolution. Each candidate is flown in a temp dir under
`optimizer/out/` (subprocess, 60 s timeout — the integrator only stops at H = 0,
so an orbital candidate would otherwise never return) and scored from the CSV:

- objective = `((range − target)/100 km)²` + `1e7 · Σ max(0, (v − lim·(1 − 1e-3))/lim)²`
  over the six §4.4 limits, + `10 · Σ max(0, ϑ_{i+1} − ϑ_i)²`; a run that fails,
  times out or yields NaN scores `1e15`.
- the limits are `LIMITS` at the top of the script (deg, deg, deg, deg/s, kPa, km);
  α is taken over powered rows with q > 1 Pa (the t = 0 row has attack = 90°
  because the flight angle is undefined at V = 0), q over powered rows only,
  ϑ̇ by finite difference after masking the zero-length gaps the doubled staging
  rows produce.
- **`alpha_sep` is the separation limit**, |α| ≤ 1.5° at a staging event that is
  still an aerodynamic one — `h ≤ H_ATM` **and** `q ≥ Q_SEP` (1 kPa). Rows are picked
  by `df.stage.diff() > 0` off the full frame, not `pw`: `cmd/main` resolves the stage
  from the right, so the row at a separation already carries the new stage and the
  final one is absent from `pw` altogether. Only the stage 1→2 separation qualifies
  at HEAD (66.2 s, 40.6 km, 6.35 kPa). An all-vacuum staging set scores 0.0, not NaN,
  which would otherwise fail every candidate through the `isfinite` guard.

Output is `optimizer/out/best.json` — stages + pitch, plus `target_km` and
`metrics` keys the Go loader ignores, so the simulator flies the file directly —
and `best.csv`. The search never writes `optimizer/input/rocket.json`; Ctrl-C
keeps the best candidate seen so far.

### Tests

Only `atmosphere/` has tests (`TestGOSTAtmosphere` conformance against the
printed GOST tables, `TestAtmosphereCharacterization`). The golden trajectory
tests that pinned the old simulator went with it. `go vet ./...` is clean.

## Geometry and CFD (`rocket.scad`, `openfoam/`)

The aerodynamic chain is `report/main.py` → `openscad/rocket-params.scad` → `openscad/rocket.scad` → `openscad/out/*.stl` → `make openfoam-input` → `openfoam/input/*.stl` → `gen_case.py` → snappyHexMesh → `openfoam/out/averages.csv` → `trajectory/aero/aero.go`.

`openscad/rocket.scad` is the **outer mold line only** — the surface the flow sees. No motor internals, no bores, no charge cavities, and no nozzles (nozzle bells broke snappyHexMesh and were dropped in `a540884`). Every section is a solid of revolution unioned with an `eps` = 3 mm overlap, so each STL exports as one genus-0 manifold shell.

`report/main.py` owns the dimensions via `openscad/rocket-params.scad` (`--write-scad-params`: stderr-only warnings, clean stdout). It is generated but stays **tracked**, so a fresh clone opens in OpenSCAD without running the generator. `rocket.scad` holds the shape logic plus the structural constants `main.py` does not compute — the interstage, adapter, nav-module and warhead dimensions.

**Every stage flies at its own motor diameter.** The seating condition

    d_м >= d_a (1 + sqrt(2)) + 2 l_a sin(δ_с)

is satisfied on the motor itself for all three stages (margins 378 / 0.6 / 15 mm at δ_с = 8° / 6° / 4°), so the external shell that used to wrap a too-narrow stage 3 is retired and `rocket.scad` sets `d_ext = d_m`. `report/main.py`'s `STAGES` note explains how stages 2 and 3 buy that fit with their l̄_з/ρ_т·u pair. The swing term is the bell alone, `l_a sin δ_с` about the throat — `l_дк` does not enter, which is what leaves stage 2 its 0.6 mm at 6°.

**One reference for every part.** `gen_case.py` meshes each part against *its
own* bounding box but non-dimensionalises **all** parts by `part="all"`
(`L_all`, `R_all` at `gen_case.py:210`). That is why `trajectory` can carry a
single `Aref` and apply it to the `stage2up`/`stage3up`/`head` coefficients
without rescaling — the invariant is deliberate, not an accident.

**The two modules hand off through `openfoam/input/`, and only there.** `make stls` renders `openscad/out/<part>.stl` (the stem of the target *is* the `PART` selector), and `make openfoam-input` copies them across. `gen_case.py` and `sweep.py` never build geometry — they exit with an error naming `make openfoam-input` if a part is missing. That is what stops a long sweep from silently re-rendering against a changed `rocket-params.scad` halfway through. In the STL pattern rule `$(SCAD)` must stay the **first** prerequisite — the recipe passes `$<` to OpenSCAD, and putting `$(PARAMS)` first would render the parameter file instead.

`sweep.py --dry-run` writes nothing and currently queues 96 cases (33 preferential, 63 rest; 25 snappyHexMesh mesh groups). `averages.csv` holds 101 rows — `all` 55, `stage3up` 20, `stage2up` 16, `head` 10 — so the shipped table is not exactly the current queue.

**`openfoam/out/averages.csv` was solved on the pre-shortening mold line.** L_i dropped 1.44 m in total (7.70/3.95/2.18 → 7.14/3.51/1.75) and the STLs were regenerated, but the coefficients have not been re-swept. This is safe to carry, not free: `RrefAll` is unchanged, so `Aref` still matches what `gen_case.py` non-dimensionalised by and the retained C_x/C_y remain dimensionally consistent. What is unbounded is fidelity — less wetted area means the real C_x is lower (range pessimistic), but afterbody length also shifts C_y, so "conservative on range" does **not** imply "conservative on α". Only a re-sweep closes that.

Case generation needs only `make` and `openscad`; meshing and solving need OpenFOAM v2512 with HiSA. **Never run `plot_coeffs.py` without a complete sweep** — it rebuilds `averages.csv`, the only surviving CFD result, and a partial set of inputs silently truncates it. Since `openfoam/out/` is gitignored, that file is now unbacked by git: treat it as precious, and copy it somewhere safe before any operation that could rewrite it.

## Known drift

- **`report/main.py`'s `K_V` comment block** refers to `traj/pitch.go`'s
  `FrameAlpha` and `ShapeHermite`. Both are gone; the block is a design-history
  record, and its range figures (12 749 km, k_V = 1.176) are not reproducible
  against the current simulator. Keep the reasoning, but do not cite the numbers
  as current results.
- **`report/archive.typ`** lags the scripts on the nozzle/length block (18 of
  129 emitted equations) — see [The report](#the-report-reportarchivetyp).
- **`openfoam/out/averages.csv`** predates the (3.44) length change and is no
  longer tracked by git. See the CFD section above.

## Output format

Scripts emit Typst source, not plain text. Inline strings use Typst math syntax with Cyrillic labels (e.g. `"уд"`, `"ст"`). The `emit()` helper (in `typst.py`) wraps a body in a `#math.equation(numbering: none, block: true, $ … $)` call. Only substituted calculations and tables are emitted — the symbolic formulas live in the report document.

## CSV chart format

Multi-curve charts store each curve as a pair of columns (X, Y). Row 0 holds curve labels; row 1 holds `X, Y` headers. Use `pd.read_csv(path, header=1)` for data and a separate `pd.read_csv(path, header=None, nrows=1)` pass to read labels — see `alpha_dv()` in `utils.py` for the pattern.

## Python environment

Use `uv` (see global CLAUDE.md). Python 3.11, dependencies: `numpy`, `pandas`, `pandas-stubs`, `matplotlib` (used by `openfoam/plot_coeffs.py`) and `cma` (`optimizer/main.py`). Go 1.27 for the simulator.

`report/`, `openfoam/` and `optimizer/` are **script directories, not packages**: each script is run directly, so its own directory is `sys.path[0]` and the intra-module imports (`from utils import …`, `import manifest`) resolve without any packaging. `[tool.ruff] src` in `pyproject.toml` tells isort the same thing — drop it and those imports get filed as third-party.
