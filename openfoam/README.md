# OpenFOAM external-aero cases for the coursework rocket

Parametric CFD of `rocket.scad` flight configurations: drag/lift/moment
coefficients vs Mach and angle of attack. One generator stamps a complete,
meshable case from regime templates + the part geometry.

## Layout

```
openfoam/
  gen_case.py            generator (stdlib only; run with uv)
  templates/
    common/              shared: thermo, turbulence, mesh dicts, run scripts
    subsonic/            rhoSimpleFoam stack (0/ BCs, schemes, solution)
    supersonic/          hisa stack (0/ BCs, schemes, solution)
  <part>/<regime>/       generated cases (e.g. all/supersonic)
  hisa_example/          upstream HiSA reference (read-only; do not run)
  mesh_example/          upstream mesh reference (read-only)
```

`OpenFOAM v2512` with the **HiSA** module. Solvers are not assumed on PATH here;
source your OpenFOAM environment (or enter your container) before meshing/running.

## Generating a case

```bash
uv run python openfoam/gen_case.py --part all --regime supersonic --Ma 4   --alpha 0
uv run python openfoam/gen_case.py --part all --regime subsonic   --Ma 0.7 --alpha 0
```

Options: `--part {all,stage2up,stage3up,head}`, `--regime {subsonic,supersonic}`,
`--Ma`, `--alpha` (deg), `--p`/`--T` (default sea-level ISA 101325 Pa / 288.15 K),
`--yplus` (default 325), `--layers` (override auto layer count; auto gives
Ma 4 about 12 and never below 6),
`--expansion` (1.2), `--np` (6), `--out`, `--no-stl` (reuse existing STLs).

The generator builds the part STL (`make <part>.stl`), reads its bounding box,
and **adapts the mesh to the geometry** — domain (`-5L … 11L`, farfield `5L`),
refinement cylinders and the boundary layer all scale with the part, so the cell
count stays ~constant across parts. `subsonic → rhoSimpleFoam`,
`supersonic → hisa`.

### Target flows for the study

| part      | regime     | design Ma | solver        |
|-----------|------------|-----------|---------------|
| all       | subsonic   | 0.7       | rhoSimpleFoam |
| all       | supersonic | 4         | hisa          |
| stage2up  | supersonic | 8         | hisa          |
| stage3up  | supersonic | 12        | hisa          |
| head      | supersonic | 20        | hisa          |
| head      | subsonic   | 0.7       | rhoSimpleFoam |

## Meshing and running (in the OpenFOAM environment)

```bash
cd openfoam/all/supersonic
./Allrun.pre      # blockMesh, surfaceFeatureExtract, parallel snappyHexMesh,
                  #   reconstruct to serial, checkMesh
./Allrun          # decomposePar -> mpirun -np <np> <solver> -parallel -> reconstructPar
```

Outputs: `postProcessing/forceCoeffs/.../coefficient.dat` (Cd, Cl, Cm),
`postProcessing/yPlus/...`, plus `Ma`, `Cp`, and (supersonic) `schlieren` fields.

## y+ workflow

Layers target **y+ = 300–350**. `firstLayerThickness` is a flat-plate estimate at
the case Mach (mid-body Schlichting Cf), so verify after meshing:

```bash
postProcess -func yPlus -latestTime    # or read the yPlus log from a short run
```

If y+ drifts, rerun the generator with an adjusted `--yplus` (it scales the first
layer linearly) or an explicit `--layers` override, then `./Allrun.pre` again.
**alpha-only sweeps reuse the same mesh** (geometry unchanged — only `0/U`
differs); a new Mach changes the layer sizing, so regenerate + remesh for a new
representative Mach.

## Sweep study

`sweep.py` runs the whole `(part, Ma, alpha)` matrix unattended, in a priority
order, picking the regime per Mach (`Ma < 1 → subsonic`, else supersonic):

```bash
uv run python openfoam/sweep.py --dry-run    # print the 81-case ordered queue, no run
uv run python openfoam/sweep.py              # full run (OpenFOAM env must be sourced)
uv run python openfoam/sweep.py --retry-failed
```

**Order.** All *preferential* cases first — across every part (`all → stage2up →
stage3up → head`) — then the rest, by part then Ma then alpha. A case is
preferential when both its Ma and its alpha are flagged in the matrix at the top
of `sweep.py`.

**Robust + resumable.** Per-case status lives in `sweep_state.json` (written
atomically); a rerun skips completed cases and a failing case is logged and
skipped, not fatal. Mesh depends only on `(part, Ma)`, so the alphas of one
group share a single `snappyHexMesh` run (22 meshes for 81 cases). Coefficients
are aggregated into `sweep_results.csv`; per-case OpenFOAM logs and a failed
case's `sweep.error` stay in the case dir. A pre-flight check (OpenFOAM on PATH,
STLs buildable) fails fast before committing hours. Useful flags: `--np`,
`--only-part`, `--max-cases`, `--mesh-timeout`/`--solve-timeout`,
`--no-mesh-reuse`, `--subsonic-max`.

## Convergence CSVs

`sweep_results.csv` keeps only each case's *final* coefficients. To check a case
actually converged (Cd/Cl/Cm flattened out, not still drifting), `convergence.py`
reads the per-iteration history OpenFOAM writes to
`<case>/postProcessing/forceCoeffs/<time>/coefficient.dat` and emits one tidy CSV
per case into `results/` — `Time` plus every coefficient column, one row per
iteration, ready to plot:

```bash
uv run python openfoam/convergence.py                 # all cases -> openfoam/results/*.csv
uv run python openfoam/convergence.py --only-part all
```

It's disk-driven (parses whatever produced output, no OpenFOAM env needed), merges
restart time-dirs into one monotonic history, and warns-and-skips empty cases.

## Plots & averages

`plot_coeffs.py` consumes those `results/*.csv` and, per case, writes one PNG with
two stacked panels (`Cd/Cl/Cs` on top, `CmPitch/CmRoll/CmYaw` below) vs iteration,
with the averaging window shaded. It also writes one `results/averages.csv` row per
case — `part, regime, Ma, alpha, n_iters` plus the **mean and std** of each of the
six coefficients over the last `--window` iterations:

```bash
uv run python openfoam/plot_coeffs.py              # PNGs -> results/plots/, averages.csv
uv run python openfoam/plot_coeffs.py --window 25  # average over the last 25 iterations
```

Re-runnable as cases arrive: it rebuilds the plots and `averages.csv` from whatever
result CSVs currently exist, and warns-and-skips short/unreadable ones.

## Coefficient reference

`forceCoeffs` uses a **single fixed reference taken from part=all** — frontal
area `Aref = π·R_all²` (≈ 1.96 m²) and `lRef = L_all` (≈ 18.7 m), moment center at
the nose tip — so coefficients are directly comparable across parts and Mach.
`dragDir`/`liftDir` follow alpha; `pitchAxis` is y.

## Caveat (high Mach)

The gas model is calorically-perfect air + Sutherland + kOmegaSST. For the
head/upper-stage cases at Ma 8–20 this ignores real-gas / vibrational effects and
is a trend approximation only — adequate for the coursework comparison, not for
absolute hypersonic accuracy.
