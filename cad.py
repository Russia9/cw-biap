"""cad.py — STEP CAD model of the 3-stage rocket (BIAP coursework).

Builds a solid B-rep model with CadQuery (OpenCASCADE) and writes a named STEP
assembly for import into a downstream CAD to draft engineering drawings.

Simplified, for-now layout:
  * Each motor (РДТТ) is a solid rounded box — a cylinder Ø d_m with both ends
    filleted at radius l_дн (flat faces + rounded corners). No internals yet.
  * Nozzles are external convergent-divergent bells: the inlet (d_вх) is sunk
    into the flat aft face of the engine; the throat (d_кр) and exit (d_а) hang
    below. fit_nozzles() scales the report nozzle (keeping its proportions —
    distinct inlet/exit diameters and convergent/divergent lengths) so the four
    inlets ring-pack onto the flat aft disk (radius R - l_дн).
  * Stage 3 = external Ø1.16 m casing with the Ø0.83 m engine centered inside,
    tied to the casing by N_S3_FRAMES ring frames (шпангоуты); stages 1 & 2 use
    the engine box as the airframe.
  * Stages connected by an interstage flare (d_m1->d_m2) + skirt that meet each
    stage at full diameter; then navigation block, warhead and nose.

Dimensions mirror the report tables produced by ``uv run python main.py`` — they
are NOT recomputed here. Re-sync the STAGES block if main.py's STAGES change.
Everything is built in millimetres (STEP exports in mm).

Run:  uv run python cad.py [output.step]   (default: rocket.step)
"""

from __future__ import annotations

import math
import sys
from typing import NamedTuple

import cadquery as cq

# ---------------------------------------------------------------------------
# Dimensions (mm) — mirrored from main.py geometry table (3.28-3.42).
# d_m engine diameter; l_z charge length; l_dn dome/bottom depth; d_k channel /
# nozzle inlet; d_kr throat; d_a exit; l_dk convergent length; l_a divergent.
# (d_z, d_v, l_v, delta_* are kept for reference; unused while engines are empty.)
# ---------------------------------------------------------------------------


class Stage(NamedTuple):
    """Per-stage motor geometry, all in millimetres."""

    d_m: float      # motor-case (engine) outer diameter
    l_z: float      # charge length -> straight body length
    l_dn: float     # dome / bottom depth = corner fillet radius
    d_z: float      # charge (grain) outer diameter
    d_k: float      # channel bore / nozzle inlet diameter
    d_kr: float     # nozzle throat diameter
    d_a: float      # nozzle exit diameter
    l_dk: float     # nozzle convergent (subcritical) length
    l_a: float      # nozzle divergent (supercritical) length
    d_v: float      # igniter diameter
    l_v: float      # igniter length
    delta_k: float  # case wall thickness
    delta_tz: float  # heat-protection liner thickness


STAGES = [
    Stage(1560, 6500, 469, 1535, 461, 166, 447, 405, 386, 312, 156, 2.34, 11.31),
    Stage(1160, 3250, 347, 1137, 341, 135, 463, 284, 451, 232, 116, 1.22, 9.16),
    Stage(830, 1840, 250, 815, 245, 86, 445, 218, 494, 167, 83, 0.88, 8.45),
]

N_NOZZLES = 4  # nozzles per stage (ring layout)

# Stage-3 external casing (engine sits centered inside it) + mounting frames.
S3_CASING_OD = 1160.0  # mates flush with stage 2
CASING_WALL = 8.0
N_S3_FRAMES = 6        # ring frames tying the engine to the casing
FRAME_THK = 40.0       # axial thickness of each frame

# Structural / payload geometry — reused from rocket.scad; edit freely (mm).
NOZZLE_WALL = 10.0       # nozzle wall thickness
NOZZLE_EMBED = 30.0      # how far the nozzle inlet sinks into the engine
STRUCT_WALL = 10.0       # interstage / adapter wall
BELL_CLEAR = 40.0        # interstage clearance below a protruding nozzle
D_NAV = 700.0            # navigation block diameter
L_NAV = 180.0            # navigation block length
H_S3_NAV = 750.0         # adapter stage-3 casing -> nav (1160 -> 700)
D_HEAD = 540.0           # warhead body diameter
L_HEAD = 650.0           # warhead body length
H_NAV_HEAD = 500.0       # adapter nav -> warhead (700 -> 540)
H_NOSE = 510.0           # warhead nose cone height

# Colours (carried into STEP AP242). Mirrors the rocket.scad palette.
C_STAGE = [cq.Color(0.70, 0.70, 0.72),
           cq.Color(0.76, 0.76, 0.78),
           cq.Color(0.82, 0.82, 0.84)]
C_STRUCT = cq.Color(0.45, 0.45, 0.48)
C_NAV = cq.Color(0.30, 0.50, 0.80)
C_PAYLOAD = cq.Color(0.80, 0.30, 0.30)

Part = tuple[str, cq.Shape, cq.Color]


# ---------------------------------------------------------------------------
# Primitive builders (pure: return a cq.Shape in local coordinates)
# ---------------------------------------------------------------------------


def tube(od: float, idia: float, h: float) -> cq.Shape:
    """Annular cylinder (z = 0..h). idia = 0 gives a solid cylinder."""
    wp = cq.Workplane("XY").circle(od / 2)
    if idia > 0:
        wp = wp.circle(idia / 2)
    return wp.extrude(h).val()


def rrect(r: float, h: float, fillet: float) -> cq.Shape:
    """Rounded-box solid of revolution: a cylinder (radius r, height h) with both
    rim edges filleted at `fillet` — flat faces, rounded corners. z = 0..h."""
    return cq.Workplane("XY").circle(r).extrude(h).edges("%CIRCLE").fillet(fillet).val()


def cone_shell(d_lo: float, d_hi: float, h: float, wall: float) -> cq.Shape:
    """Hollow truncated cone (frustum) shell, z = 0..h (cylinder if d_lo==d_hi)."""
    outer = (
        cq.Workplane("XY").circle(d_lo / 2)
        .workplane(offset=h).circle(d_hi / 2).loft(ruled=True)
    )
    inner = (
        cq.Workplane("XY").circle(d_lo / 2 - wall)
        .workplane(offset=h).circle(d_hi / 2 - wall).loft(ruled=True)
    )
    return outer.cut(inner).val()


def _throat_land(stage: Stage) -> float:
    return max(10.0, 0.1 * stage.d_kr)


def _nozzle_loft(stage: Stage, extra: float) -> cq.Workplane:
    """External C-D contour hanging below z = 0: inlet d_k at the top (sunk into
    the engine aft face), convergent l_dk to the throat d_kr (+ short land),
    divergent l_a to the exit d_a — distinct inlet/exit and lengths."""
    r_in, r_kr, r_a = stage.d_k / 2, stage.d_kr / 2, stage.d_a / 2
    return (
        cq.Workplane("XY").circle(r_in + extra)                       # inlet (z = 0)
        .workplane(offset=-stage.l_dk).circle(r_kr + extra)           # throat
        .workplane(offset=-_throat_land(stage)).circle(r_kr + extra)  # throat land
        .workplane(offset=-stage.l_a).circle(r_a + extra)             # exit
        .loft(ruled=True)
    )


def nozzle(stage: Stage) -> cq.Shape:
    """One hollow external convergent-divergent nozzle (gas path shown in section)."""
    return _nozzle_loft(stage, NOZZLE_WALL).cut(_nozzle_loft(stage, 0)).val()


def nozzle_drop(stage: Stage) -> float:
    """Axial length the nozzle reaches below the flat aft face (past the embed)."""
    return stage.l_dk + _throat_land(stage) + stage.l_a - NOZZLE_EMBED


def fit_nozzles(stage: Stage) -> Stage:
    """Scale the report nozzle (preserving its proportions) so the four inlets
    ring-pack onto the flat aft disk (radius R - l_dn) of the rounded box."""
    factor = 1 / math.sin(math.pi / N_NOZZLES) + 1     # 1 + sqrt2 for N = 4
    flat = stage.d_m / 2 - stage.l_dn                  # flat aft disk radius
    widest = max(stage.d_k, stage.d_a)
    s = 2 * (flat / factor - NOZZLE_WALL) / widest     # uniform scale to fit
    return stage._replace(d_k=stage.d_k * s, d_kr=stage.d_kr * s, d_a=stage.d_a * s,
                          l_dk=stage.l_dk * s, l_a=stage.l_a * s)


# ---------------------------------------------------------------------------
# Engine (motor) — solid rounded box, nozzles sunk into the flat aft face
# ---------------------------------------------------------------------------


def build_engine(prefix: str, stage: Stage) -> tuple[list[Part], float, float]:
    """Build one (empty) motor in local coords (flat aft face at z = 0).

    Returns (parts, cyl_top, height): cyl_top is the top of the straight body,
    height the flat forward face. Nozzle inlets are sunk NOZZLE_EMBED into the box.
    """
    r = stage.d_m / 2
    cyl_top = stage.l_dn + stage.l_z
    h = 2 * stage.l_dn + stage.l_z
    color = C_STAGE[int(prefix[5]) - 1]

    parts: list[Part] = [(f"{prefix}_case", rrect(r, h, stage.l_dn), color)]

    half = max(stage.d_k, stage.d_a) / 2 + NOZZLE_WALL
    r_bc = half / math.sin(math.pi / N_NOZZLES)
    for j in range(N_NOZZLES):
        ang = 2 * math.pi * j / N_NOZZLES
        loc = cq.Location(
            cq.Vector(r_bc * math.cos(ang), r_bc * math.sin(ang), NOZZLE_EMBED))
        parts.append((f"{prefix}_nozzle_{j + 1}", nozzle(stage).located(loc), C_STRUCT))
    return parts, cyl_top, h


# ---------------------------------------------------------------------------
# Full rocket assembly
# ---------------------------------------------------------------------------


def build_rocket() -> cq.Assembly:
    """Assemble the rocket: stage-1 flat aft face at z = 0, nose at +Z."""
    asm = cq.Assembly(name="rocket")

    def add(name: str, shape: cq.Shape, color: cq.Color, z: float) -> None:
        asm.add(shape, name=name, loc=cq.Location(cq.Vector(0, 0, z)), color=color)

    def place(parts: list[Part], z: float) -> None:
        for name, shape, color in parts:
            add(name, shape, color, z)

    s1, s2, s3 = (fit_nozzles(s) for s in STAGES)

    # --- Stage 1 (engine = airframe), flat aft face at z = 0 ---
    p1, cyl1, h1 = build_engine("stage1", s1)
    place(p1, 0.0)

    # --- Interstage 1-2: flare (d_m1 -> d_m2) at stage-1 shoulder, then skirt ---
    add("interstage12_flare", cone_shell(s1.d_m, s2.d_m, s1.l_dn, STRUCT_WALL),
        C_STRUCT, cyl1)
    skirt_z0 = cyl1 + s1.l_dn
    z_aft2 = h1 + nozzle_drop(s2) + BELL_CLEAR
    cyl_bottom2 = z_aft2 + s2.l_dn
    add("interstage12_skirt", tube(s2.d_m, s2.d_m - 2 * STRUCT_WALL, cyl_bottom2 - skirt_z0),
        C_STRUCT, skirt_z0)

    # --- Stage 2 (engine = airframe) ---
    p2, cyl2, h2 = build_engine("stage2", s2)
    place(p2, z_aft2)
    cyl_top2 = z_aft2 + cyl2
    fwd_pole2 = z_aft2 + h2

    # --- Stage 3: external casing with the engine + ring frames inside ---
    z_aft3 = fwd_pole2 + nozzle_drop(s3) + BELL_CLEAR
    p3, _, h3 = build_engine("stage3_engine", s3)
    place(p3, z_aft3)
    fwd_pole3 = z_aft3 + h3
    add("stage3_casing",
        tube(S3_CASING_OD, S3_CASING_OD - 2 * CASING_WALL, fwd_pole3 - cyl_top2),
        C_STAGE[2], cyl_top2)
    for k in range(N_S3_FRAMES):
        zf = z_aft3 + s3.l_dn + s3.l_z * (k + 0.5) / N_S3_FRAMES
        add(f"stage3_frame_{k + 1}",
            tube(S3_CASING_OD - 2 * CASING_WALL, s3.d_m, FRAME_THK),
            C_STRUCT, zf - FRAME_THK / 2)

    # --- Adapter -> navigation block -> adapter -> warhead -> nose ---
    z = fwd_pole3
    add("adapter_s3_nav", cone_shell(S3_CASING_OD, D_NAV, H_S3_NAV, STRUCT_WALL),
        C_STRUCT, z)
    z += H_S3_NAV
    add("nav_block", tube(D_NAV, D_NAV - 2 * STRUCT_WALL, L_NAV), C_NAV, z)
    z += L_NAV
    add("adapter_nav_head", cone_shell(D_NAV, D_HEAD, H_NAV_HEAD, STRUCT_WALL),
        C_STRUCT, z)
    z += H_NAV_HEAD
    add("warhead_body", tube(D_HEAD, D_HEAD - 2 * STRUCT_WALL, L_HEAD), C_PAYLOAD, z)
    z += L_HEAD
    add("nose_cone", cq.Solid.makeCone(D_HEAD / 2, 0.0, H_NOSE), C_PAYLOAD, z)

    return asm


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "rocket.step"
    asm = build_rocket()
    n_parts = len(asm.children)
    asm.export(out, "STEP", unit="MM")

    # Round-trip the written file as ground truth for the summary / validity.
    solids = cq.importers.importStep(out).solids().vals()
    bb = cq.Compound.makeCompound(solids).BoundingBox()
    print(f"Wrote {out}")
    print(f"  named parts : {n_parts}")
    print(f"  solids      : {len(solids)}")
    print(f"  overall len : {bb.zlen / 1000:.2f} m (z {bb.zmin / 1000:.2f}"
          f" .. {bb.zmax / 1000:.2f} m)")
    print(f"  max diameter: {max(bb.xlen, bb.ylen) / 1000:.2f} m")


if __name__ == "__main__":
    main()
