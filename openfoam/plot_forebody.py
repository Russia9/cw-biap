"""Compare the CFD forebody drag of the verification body with reference data.

The solver-verification case is an ogive-cylinder — a tangent circular-arc nose
of l_нч = 1.54 D on a cylindrical afterbody of l_ц = 7 D — for which the
textbook gives C_x0(M) directly. Only the FOREBODY is comparable: the base of a
cylinder has the same area as the midsection, so its drag enters C_x0 at 0.08 to
0.29 here and swamps everything else. It is removed by integrating Cp over the
base disc alone (see base_drag.sh) and subtracting.

The plotted curve is the CFD pressure drag with the base removed, plus a
friction term:

    C_x0 = (C_(d,давл) - C_(d,дон)) + C_(d,тр)

Every term but the friction is measured. The friction term is the CFD's own
value below M = 1, where the solver reproduces the Schlichting flat plate to
~1%, and the Schlichting value above M = 1, where the pipeline applies low-Re
wall functions on a mesh built for y+ = 325 and loses ~95% of the skin friction.
The supersonic half of the curve is therefore an estimate of what the case would
give once that defect is fixed, not a result.

    uv run python openfoam/plot_forebody.py
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")  # headless: just write PNGs
import matplotlib.pyplot as plt
import pandas as pd

HERE = Path(__file__).resolve().parent  # openfoam/
OUT = HERE / "out"  # everything this module generates
SUBSONIC_MAX = 1.0  # regime split, mirrors sweep.py --subsonic-max
def plot(df, dst):
    ref = df[df.Cx0_chart.notna()]

    fig, ax = plt.subplots(figsize=(8, 4.5))

    ax.plot(ref.Ma, ref.Cx0_chart, "o--", color="k", lw=1.4, ms=5,
            label=r"справочные данные, $\lambda_{нч}$ = 1,54")
    ax.plot(df.Ma, df.fore_fix, "s-", color="C0", lw=1.4, ms=5,
            label=r"OpenFOAM")

    # the regime switch: below it rhoSimpleFoam, above it hisa, and the skin
    # friction defect lives entirely on the hisa side
    ax.axvline(1.0, color="C3", ls=":", lw=1.2,
               label="граница rhoSimpleFoam / hisa")

    ax.set_xlabel(r"число Маха $M_\infty$")
    ax.set_ylabel(r"коэффициент $C_{x0}$ (без донного сопротивления)")
    ax.set_xlim(0.3, 4.2)
    ax.set_ylim(0, None)
    ax.grid(True, which="major", alpha=0.3)
    ax.grid(True, which="minor", alpha=0.12)
    ax.minorticks_on()
    ax.legend(loc="best", fontsize="small")
    fig.tight_layout()
    fig.savefig(dst, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--csv", type=Path, default=OUT / "verification.csv",
                    help="sweep summary from analyse.py (default openfoam/out/)")
    ap.add_argument("--out", type=Path, default=OUT / "plots" / "forebody.png",
                    help="output PNG (default openfoam/out/plots/)")
    args = ap.parse_args()

    df = pd.read_csv(args.csv).sort_values("Ma")
    # C_x0 = (CFD pressure drag minus the base) + flat-plate friction.
    # Identity: Cd_fore - Cd_visc == Cd_wave, so this is Cd_wave + Cd_visc_th.
    # Friction term: the CFD's own where it is trustworthy, the correlation only
    # where it is not. Below M = 1 the solver reproduces flat-plate friction to
    # ~1%, so its value is used as measured; above M = 1 the low-Re wall
    # functions lose ~95% of it, so Schlichting is substituted.
    friction = np.where(df.Ma < SUBSONIC_MAX, df.Cd_visc, df.Cd_visc_th)
    df["fore_fix"] = df.Cd_wave + friction

    args.out.parent.mkdir(parents=True, exist_ok=True)
    plot(df, args.out)

    ref = df[df.Cx0_chart.notna()].copy()
    ref["delta"] = 100 * (ref.fore_fix - ref.Cx0_chart) / ref.Cx0_chart
    for _, r in ref.iterrows():
        print(f"  M = {r.Ma:<4g} CFD {r.fore_fix:.4f}  спр. {r.Cx0_chart:.2f}  "
              f"откл. {r.delta:+6.1f}%")
    worst = ref.loc[ref.delta.abs().idxmax()]
    sup = ref[ref.Ma >= 1.1]
    sworst = sup.loc[sup.delta.abs().idxmax()]
    print(f"  макс. отклонение          : {worst.delta:+.1f}% (M = {worst.Ma:g})")
    print(f"  макс. на сверхзвуке       : {sworst.delta:+.1f}% (M = {sworst.Ma:g})")
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
