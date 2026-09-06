# cw-biap

Coursework for BIAP: ballistic and thrust design of a three-stage solid-fuel
rocket, plus a trajectory simulator used to check the design.

The repository is five modules, each owning one artifact and writing only into
its own directory. Data flows one way:

```
report/ ──> openscad/ ──> openfoam/ ──> trajectory/
   │                          │              ▲
   └────> optimizer/input/ ───┴──────────────┘
```

| module        | owns                                        | generated (gitignored)     |
|---------------|---------------------------------------------|----------------------------|
| `report/`     | sizing calculations, Typst output, `archive.typ` | –                     |
| `openscad/`   | the outer mold line                         | `out/` (STLs, renders)     |
| `openfoam/`   | the CFD sweep                               | `input/`, `out/`           |
| `optimizer/`  | the CMA-ES pitch search                     | `input/`, `out/`           |
| `trajectory/` | the Go simulator                            | `out/` (plots)             |

## 1. `report/` — sizing calculations (Python)

Emit [Typst](https://typst.app) math blocks and table rows on stdout, to be
pasted into the report document. `archive.typ` is the in-repo snapshot of it.

```bash
uv run python report/preliminary.py   # burn-rate and l_з/α_дв fuel-selection tables
uv run python report/main.py          # thrust, weights, masses and geometry
uv run python report/aero_tables.py   # CFD coefficients as α × Mach tables
```

`main.py` also owns the physical inputs of the other modules. A bare run warns
on stderr if either generated file has drifted; to update them:

```bash
uv run python report/main.py --write-traj-config   # -> optimizer/input/rocket.json
uv run python report/main.py --write-scad-params   # -> openscad/rocket-params.scad
```

`--write-traj-config` writes only the `stages` array. The pitch program is
optimizer **output**, not a sizing input, so it is never written here — an
existing `pitch` block is preserved verbatim.

## 2. `openscad/` — geometry

`rocket.scad` is the outer mold line: the surface the flow sees, with no motor
internals and no nozzles. Its dimensions come from `rocket-params.scad`, which
`report/main.py` generates, so the drawing cannot drift from the report.

```bash
make stls              # -> openscad/out/{all,stage2up,stage3up,head}.stl
make png               # -> openscad/out/rocket.png
make openfoam-input    # stage the meshes into openfoam/input/
```

## 3. `openfoam/` — CFD

Turns the staged STLs into meshable OpenFOAM cases and sweeps them over Mach and
angle of attack. **It never builds geometry** — `make openfoam-input` puts it in
`openfoam/input/`, and everything generated lands in `openfoam/out/`.

Case generation needs only `make` and `openscad`; meshing and solving need
OpenFOAM v2512 with the HiSA module. See `openfoam/README.md`.

```bash
uv run python openfoam/gen_case.py --part all --regime supersonic --Ma 4 --alpha 0
uv run python openfoam/sweep.py --dry-run
```

The aggregated coefficient table is `openfoam/out/averages.csv`. It is
gitignored like the rest of `out/`, so a fresh clone must re-run the sweep (or
be handed the file) before the simulator can fly with aerodynamics.

## 4. `trajectory/` — the simulator (Go)

A planar, spherical-Earth RK4 integrator with a GOST 4401-81 atmosphere and a
programmed pitch angle. Both binaries take their paths as positional arguments
and create `out/` themselves.

```bash
cd trajectory
go build ./... && go test ./...
go run ./cmd/main ../optimizer/input/rocket.json ../openfoam/out/averages.csv out/traj.csv
go run ./cmd/plot ../optimizer/input/rocket.json    # the pitch program alone
```

The coefficient table is **required**, not optional: there is no drag-free mode.

## 5. `optimizer/` — pitch search

`main.py` runs CMA-ES (pycma) over the pitch block — `theta_deg` and exit slope `k`
of every arc, their relative durations, and `t_start` — starting from the pitch block
in `optimizer/seed.json` (a config in the simulator's form; only its `pitch` key is
read, and every arc must be `"hermite"`). It takes the
stages from `optimizer/input/rocket.json`, flies each candidate with
`trajectory/cmd/main`, and scores the squared miss from `--target` (km) plus
penalties on the §4.4 limits: |α| ≤ 1.5° subsonic, ≤ 10° supersonic, ≤ 1.5° at a
stage separation still inside the atmosphere (below 94 km and at 1 kPa or more),
|ϑ̇| ≤ 3 °/s, q ≤ 120 kPa on the powered leg, apogee ≤ 1800 km.

The arcs tile the powered window and cannot leave it. Past burnout the programmed
angle drives neither thrust nor angle of attack, so an arc out there is a search
dimension that cannot change the trajectory.

```bash
uv run python optimizer/main.py --target 12000 --maxiter 500 --jobs 8
uv run python optimizer/main.py --arcs 24            # coarser or finer program
```

The winner goes to `optimizer/out/best.json` — a full config the simulator flies
as is — with the flown trajectory beside it in `best.csv`. Pass that file back
as `--seed optimizer/out/best.json` to continue the search at the same resolution,
or add `--arcs` to change it. The search never writes `optimizer/input/rocket.json`.

## Environment

Python 3.11 via [`uv`](https://docs.astral.sh/uv/); Go 1.27 for the simulator.

```bash
uv run ruff check .   # lint
uv run pyright        # type-check report/
```
