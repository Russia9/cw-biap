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
are preserved.

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

### Aerodynamics are currently zeroed

The CFD data set was removed from the repository, and the simulator runs with
**no drag, lift or pitch moment** unless a coefficient table is supplied with
`-aero=<path to averages.csv>`. Range figures are therefore optimistic by the
drag loss (roughly 175 m/s of burnout velocity on the active leg). Dynamic
pressure, Mach and angle of attack are still computed from the atmosphere and
the kinematics, so the §4.4 constraints remain meaningful.
