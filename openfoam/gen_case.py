#!/usr/bin/env python3
"""Generate an OpenFOAM external-aero case for the coursework rocket.

One generator stamps out a runnable case for a given (part, regime, Ma, alpha):

    uv run python openfoam/gen_case.py --part all --regime supersonic --Ma 4 --alpha 0
    uv run python openfoam/gen_case.py --part all --regime subsonic   --Ma 0.7

What it does
------------
* Builds the part STL via the project Makefile and reads its bounding box, so the
  mesh domain / refinement / boundary-layer adapt to the geometry (cell count
  stays ~constant across parts) without duplicating any rocket.scad dimensions.
* Computes the freestream state (sea-level ISA by default), the turbulence inlet
  values, and the wall-adjacent cell height that lands y+ in 300-350 at the case
  Mach, plus enough layers to keep the prism stack from thinning out at high Ma
  (flat-plate estimate; refine with the yPlus function object and rerun).
* Writes constant/freestreamProperties (flow + a single force-coefficient
  reference taken from part="all" so Cd/Cl/Cm compare across all cases), renders
  the geometry-dependent mesh dicts, and copies the rest of the regime template.

Regimes: subsonic -> rhoSimpleFoam, supersonic -> hisa.
The gas model (perfect air, gamma=1.4, R=287, Sutherland) matches
templates/common/constant/thermophysicalProperties.
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import struct
import subprocess
import sys
from pathlib import Path

# --- locations -------------------------------------------------------------
HERE = Path(__file__).resolve().parent  # openfoam/
ROOT = HERE.parent  # repo root (has the Makefile)
TEMPLATES = HERE / "templates"

PART_STL = {  # Makefile target per part
    "all": "rocket.stl",
    "stage2up": "stage2up.stl",
    "stage3up": "stage3up.stl",
    "head": "head.stl",
}
SOLVER = {"subsonic": "rhoSimpleFoam", "supersonic": "hisa"}

# --- gas model (must match thermophysicalProperties) -----------------------
GAMMA = 1.4
RGAS = 287.0
SUTHERLAND_AS = 1.458e-6
SUTHERLAND_TS = 110.4
TURB_INTENSITY = 0.01  # freestream turbulence intensity (1%)
MU_RATIO = 234.0  # eddy-viscosity ratio mu_t/mu
MIN_LAYER_COUNT = 6  # lowest auto layer count for thick low-Ma cells
REFERENCE_LAYER_COUNT = 12  # auto layers keep Ma~4 at about this count
LAYER_REFERENCE_MACH = 4.0  # auto layers preserve this stack thickness


# --- helpers ---------------------------------------------------------------
def fmt(x: float) -> str:
    """Format a float for an OpenFOAM dictionary (no trailing noise, no -0)."""
    if x == 0.0:
        return "0"
    return f"{x:.6g}"


def stl_bbox(path: Path) -> tuple[float, float]:
    """Return (length along x, max radius sqrt(y^2+z^2)) of an STL.

    Handles binary and ASCII STL; OpenSCAD writes binary by default.
    The model is laid on +x with the nose tip at x~0, so length = xmax-xmin.
    """
    data = path.read_bytes()
    verts: list[tuple[float, float, float]] = []

    binary = False
    if len(data) >= 84:
        ntri = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + ntri * 50:
            binary = True

    if binary:
        ntri = struct.unpack_from("<I", data, 80)[0]
        off = 84
        for i in range(ntri):
            base = off + i * 50 + 12  # skip the 12-byte facet normal
            for v in range(3):
                verts.append(struct.unpack_from("<3f", data, base + v * 12))
    else:
        for line in data.decode("ascii", "replace").splitlines():
            s = line.strip()
            if s.startswith("vertex"):
                _, x, y, z = s.split()[:4]
                verts.append((float(x), float(y), float(z)))

    if not verts:
        sys.exit(f"error: no vertices parsed from {path}")

    xs = [v[0] for v in verts]
    rad = max(math.hypot(v[1], v[2]) for v in verts)
    return max(xs) - min(xs), rad


def ensure_stl(part: str, skip: bool) -> Path:
    """Build (unless --no-stl) and return the path to the part STL."""
    target = PART_STL[part]
    path = ROOT / target
    if not skip:
        subprocess.run(["make", target, "SCALE=1"], cwd=ROOT, check=True)
    if not path.is_file():
        sys.exit(f"error: {path} not found (run without --no-stl to build it)")
    return path


def render(src: Path, dst: Path, tokens: dict[str, str]) -> None:
    text = src.read_text()
    for key, val in tokens.items():
        text = text.replace(f"@@{key}@@", val)
    dst.write_text(text)


def copy(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src, dst)


def wall_layer_estimate(
    U: float, rho: float, mu: float, L: float, yplus: float
) -> tuple[float, float]:
    """Return (first_layer_thickness, Re_mid) from the flat-plate y+ estimate."""
    Re = rho * U * (L / 2.0) / mu
    Cf = (2.0 * math.log10(Re) - 0.65) ** -2.3  # Schlichting
    utau = U * math.sqrt(Cf / 2.0)
    y_centroid = yplus * mu / (rho * utau)
    return 2.0 * y_centroid, Re  # centroid at y_centroid


def layer_stack_thickness(first_layer: float, n_layers: int, expansion: float) -> float:
    """Return total prism-stack thickness for OpenFOAM first-layer sizing."""
    if n_layers <= 0:
        return 0.0
    if math.isclose(expansion, 1.0):
        return first_layer * n_layers
    return first_layer * (expansion**n_layers - 1.0) / (expansion - 1.0)


def auto_layer_count(first_layer: float, target_stack: float, expansion: float) -> int:
    """Preserve target_stack within a bounded, first-layer-driven layer count."""
    if layer_stack_thickness(first_layer, MIN_LAYER_COUNT, expansion) >= target_stack:
        return MIN_LAYER_COUNT
    if math.isclose(expansion, 1.0):
        return max(MIN_LAYER_COUNT, math.ceil(target_stack / first_layer))
    return max(
        MIN_LAYER_COUNT,
        math.ceil(
            math.log1p((expansion - 1.0) * target_stack / first_layer)
            / math.log(expansion)
        ),
    )


# --- main ------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--part", choices=PART_STL, required=True)
    ap.add_argument("--regime", choices=SOLVER, required=True)
    ap.add_argument("--Ma", type=float, required=True, help="freestream Mach number")
    ap.add_argument("--alpha", type=float, default=0.0, help="angle of attack [deg]")
    ap.add_argument("--p", type=float, default=101325.0, help="static pressure [Pa]")
    ap.add_argument("--T", type=float, default=288.15, help="static temperature [K]")
    ap.add_argument("--yplus", type=float, default=325.0, help="target wall y+")
    ap.add_argument(
        "--layers",
        type=int,
        default=None,
        help=(
            "boundary-layer cell count override (default: auto, Ma 4 about 12, min 6)"
        ),
    )
    ap.add_argument(
        "--expansion", type=float, default=1.2, help="layer expansion ratio"
    )
    ap.add_argument("--np", type=int, default=12, help="MPI subdomains")
    ap.add_argument(
        "--out",
        type=Path,
        default=None,
        help="output case dir (default openfoam/<part>/<regime>)",
    )
    ap.add_argument("--no-stl", action="store_true", help="reuse existing STLs")
    args = ap.parse_args()
    if args.layers is not None and args.layers < 1:
        sys.exit("error: --layers must be >= 1")
    if args.expansion < 1.0:
        sys.exit("error: --expansion must be >= 1")

    out = args.out or (HERE / args.part / args.regime)
    solver = SOLVER[args.regime]

    # 1. geometry from STL bounding boxes -----------------------------------
    part_stl = ensure_stl(args.part, args.no_stl)
    all_stl = part_stl if args.part == "all" else ensure_stl("all", args.no_stl)
    L, R = stl_bbox(part_stl)  # this part: drives the mesh
    L_all, R_all = stl_bbox(all_stl)  # part="all": fixed coefficient reference

    # 2. freestream state ----------------------------------------------------
    a = math.sqrt(GAMMA * RGAS * args.T)
    U = args.Ma * a
    arad = math.radians(args.alpha)
    Ux, Uz = U * math.cos(arad), U * math.sin(arad)
    rho = args.p / (RGAS * args.T)
    mu = SUTHERLAND_AS * args.T**1.5 / (args.T + SUTHERLAND_TS)
    nu = mu / rho
    q = 0.5 * rho * U * U
    kInf = 1.5 * (TURB_INTENSITY * U) ** 2
    omegaInf = kInf / (MU_RATIO * nu)

    # 3. y+ first-layer height + auto layer count ---------------------------
    first_layer, Re = wall_layer_estimate(U, rho, mu, L, args.yplus)
    ref_U = LAYER_REFERENCE_MACH * a
    ref_first_layer, _ = wall_layer_estimate(ref_U, rho, mu, L, args.yplus)
    target_stack = layer_stack_thickness(
        ref_first_layer, REFERENCE_LAYER_COUNT, args.expansion
    )
    if args.layers is None:
        n_layers = auto_layer_count(first_layer, target_stack, args.expansion)
        layer_mode = "auto"
    else:
        n_layers = args.layers
        layer_mode = "manual"
    stack_thickness = layer_stack_thickness(first_layer, n_layers, args.expansion)

    # 4. domain + refinement, scaled from the part bbox ----------------------
    tok = {
        "xIn": fmt(-5.0 * L),
        "xOut": fmt(11.0 * L),
        "Rfar": fmt(5.0 * L),
        "c": fmt(5.0 * L / math.sqrt(2.0)),
        "a": fmt(5.1 * R),
        "bcX1": fmt(-0.437 * L),
        "bcX2": fmt(3.0 * L),
        "bcR": fmt(7.6 * R),
        "nbX1": fmt(-0.164 * L),
        "nbX2": fmt(1.64 * L),
        "nbR": fmt(3.16 * R),
        "cbX1": fmt(-0.0547 * L),
        "cbX2": fmt(1.148 * L),
        "cbR": fmt(1.65 * R),
        "locX": fmt(-2.7 * L),
        "locYZ": fmt(1.5 * R),
        "firstLayer": fmt(first_layer),
        "nLayers": str(n_layers),
        "expansion": fmt(args.expansion),
        "np": str(args.np),
        "solver": solver,
    }

    # 5. fixed force/moment-coefficient reference (from part="all") ----------
    Aref = math.pi * R_all * R_all
    dragDir = (math.cos(arad), 0.0, math.sin(arad))
    liftDir = (-math.sin(arad), 0.0, math.cos(arad))

    # --- write the case ----------------------------------------------------
    (out / "0").mkdir(parents=True, exist_ok=True)
    (out / "system").mkdir(parents=True, exist_ok=True)
    (out / "constant" / "triSurface").mkdir(parents=True, exist_ok=True)

    common = TEMPLATES / "common"
    regime = TEMPLATES / args.regime

    # shared static dicts
    copy(common / "case.foam", out / "case.foam")
    copy(
        common / "constant/thermophysicalProperties",
        out / "constant/thermophysicalProperties",
    )
    copy(
        common / "constant/turbulenceProperties", out / "constant/turbulenceProperties"
    )
    copy(
        common / "system/surfaceFeatureExtractDict",
        out / "system/surfaceFeatureExtractDict",
    )

    # rendered, geometry-dependent dicts + run scripts
    render(common / "system/blockMeshDict", out / "system/blockMeshDict", tok)
    render(common / "system/snappyHexMeshDict", out / "system/snappyHexMeshDict", tok)
    render(common / "system/decomposeParDict", out / "system/decomposeParDict", tok)
    for s in ("Allrun.pre", "Allrun", "Allclean"):
        render(common / s, out / s, tok)
        os.chmod(out / s, 0o755)

    # regime-specific 0/ and solver dicts
    for f in sorted((regime / "0").iterdir()):
        copy(f, out / "0" / f.name)
    for f in sorted((regime / "system").iterdir()):
        copy(f, out / "system" / f.name)

    # generated flow + coefficient reference
    write_freestream(
        out / "constant/freestreamProperties",
        args,
        U,
        Ux,
        Uz,
        rho,
        q,
        kInf,
        omegaInf,
        Aref,
        L_all,
        dragDir,
        liftDir,
    )

    # the surface itself
    copy(part_stl, out / "constant/triSurface/model.stl")

    # --- summary -----------------------------------------------------------
    print(f"\nGenerated {args.regime} case ({solver}) -> {out}")
    print(
        f"  part={args.part}  Ma={args.Ma}  alpha={args.alpha} deg  "
        f"p={fmt(args.p)} Pa  T={fmt(args.T)} K"
    )
    print(
        f"  geometry: L={fmt(L)} m  R_body={fmt(R)} m   "
        f"(reference part=all: L={fmt(L_all)} m  Aref={fmt(Aref)} m^2)"
    )
    print(
        f"  flow:     a={fmt(a)}  |U|={fmt(U)} m/s  rho={fmt(rho)}  "
        f"mu={fmt(mu)}  Re_mid={Re:.3e}"
    )
    print(f"  turb:     kInf={fmt(kInf)}  omegaInf={fmt(omegaInf)}")
    print(
        f"  layers:   firstLayerThickness={first_layer:.3e} m  "
        f"nLayers={n_layers} ({layer_mode})  expansion={fmt(args.expansion)}  "
        f"stack={stack_thickness:.3e} m  target y+={fmt(args.yplus)}"
    )
    print(
        f"  decompose: {args.np} subdomains\n"
        f"  next: cd {out} && ./Allrun.pre   # mesh, then check yPlus and rerun if y+ drifts"
    )


def write_freestream(
    path, args, U, Ux, Uz, rho, q, kInf, omegaInf, Aref, lRef, dragDir, liftDir
) -> None:
    def vec(t):
        return f"({fmt(t[0])} {fmt(t[1])} {fmt(t[2])})"

    path.write_text(f"""\
/*--------------------------------*- C++ -*----------------------------------*\\
| GENERATED by gen_case.py -- edit Ma/alpha/p/T via the script, not here.      |
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      freestreamProperties;
}}
// part={args.part} regime={args.regime}  Ma={args.Ma}  alpha={args.alpha} deg
// static state (sea-level ISA unless overridden): p={fmt(args.p)} Pa, T={fmt(args.T)} K
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

pInf        {fmt(args.p)};
TInf        {fmt(args.T)};
RGas        {fmt(RGAS)};

UInfMag     {fmt(U)};
UInf        ({fmt(Ux)} 0 {fmt(Uz)});
magUInf     {fmt(U)};

rhoInf      {fmt(rho)};
qInf        {fmt(q)};
kInf        {fmt(kInf)};
omegaInf    {fmt(omegaInf)};

// force/moment-coefficient references -- FIXED from part="all" so Cd/Cl/Cm are
// comparable across every part and Mach number (frontal area + body length).
Aref        {fmt(Aref)};
lRef        {fmt(lRef)};
CofR        (0 0 0);
dragDir     {vec(dragDir)};
liftDir     {vec(liftDir)};
pitchAxis   (0 1 0);


// ************************************************************************* //
""")


if __name__ == "__main__":
    main()
