# cw-biap

Coursework for BIAP: ballistic and thrust design of a three-stage solid-fuel
rocket, plus a trajectory simulator used to check the design against the
constructive-ballistic limits of §4.4.

The repository has two layers.

## 1. Report calculations (Python)

Sizing calculations that emit [Typst](https://typst.app) math blocks and table
rows on stdout, to be pasted into the report document.

```bash
uv run python preliminary.py   # burn-rate and l_з/α_дв fuel-selection tables
uv run python main.py          # thrust, weights, masses and geometry
```

`main.py` also owns the physical inputs of the trajectory simulator. A bare run
warns on stderr if `traj/rocket.json` has drifted out of sync; to update it:

```bash
uv run python main.py --write-traj-config
```

That rewrites only the masses, burn times, specific impulses and motor
diameters. The pitch program and limits in that file belong to the optimizer and
are preserved. `main.py --write-scad-params` does the same for the CAD geometry
in `rocket-params.scad`.

## 2. Trajectory simulator (Go) and optimizer (Python)

A planar, spherical-Earth RK4 integrator with a GOST 4401-81 atmosphere and a
programmed pitch angle, plus a CMA-ES driver that tunes the pitch program.

```bash
cd traj
go run ./main -config=rocket.json          # simulate, write out/traj.csv + diagnostics
go run ./main -config=rocket.json -metrics # one JSON line, for the optimizer
uv run python optimize.py                  # tune the pitch program -> out/best.json
uv run python plot_trajectory.py           # charts from out/traj.csv
```

### Aerodynamics must be passed explicitly

The coefficient table lives in `openfoam/results/averages.csv` and is **not**
loaded by default. Without `-aero` the simulator runs with no drag, lift or
pitch moment, which does not reproduce the reported result — the pitch program
was optimized with the table, and a drag-free run violates the §4.4 angle-of-attack
limits. Always pass it:

```bash
go run ./main -config=rocket.json -aero=../openfoam/results/averages.csv
uv run python optimize.py --aero ../openfoam/results/averages.csv
```

## 3. Geometry (OpenSCAD) and CFD (OpenFOAM)

`rocket.scad` is the outer mold line — the surface the flow sees, with no motor
internals or nozzles. Its dimensions come from `rocket-params.scad`, which
`main.py` generates, so the drawing cannot drift from the report.

```bash
make stls        # rocket.stl, stage2up.stl, stage3up.stl, head.stl
make png         # preview render
```

`openfoam/` turns those STLs into meshable OpenFOAM cases and sweeps them over
Mach and angle of attack. Case generation needs only `make` and `openscad`;
meshing and solving need OpenFOAM v2512 with the HiSA module. See
`openfoam/README.md`.

```bash
uv run python openfoam/gen_case.py --part all --regime supersonic --Ma 4 --alpha 0
uv run python openfoam/sweep.py --dry-run
```
