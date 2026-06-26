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
`--yplus` (default 325), `--layers` (6), `--expansion` (1.2), `--np` (6),
`--out`, `--no-stl` (reuse existing STLs).

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
./Allrun.pre      # blockMesh -> surfaceFeatureExtract -> snappyHexMesh -> checkMesh
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
layer linearly) or `--layers`, then `./Allrun.pre` again. **alpha-only sweeps
reuse the same mesh** (geometry unchanged — only `0/U` differs); a new Mach
changes the layer sizing, so regenerate + remesh for a new representative Mach.

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
