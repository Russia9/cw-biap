import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt, numpy as np, pandas as pd
from pathlib import Path
S = Path("/private/tmp/claude-501/-Users-russia9-study-8sem-cw-biap/5471bc84-79c9-4fcb-8799-bf0f41c32a3f/scratchpad")
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
CRIT, INK, SEC, MUTED, GRID, SURF = "#d03b3b", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#fcfcfb"
plt.rcParams.update({"figure.facecolor":SURF,"axes.facecolor":SURF,"savefig.facecolor":SURF,
 "font.family":"DejaVu Sans","font.size":10,"axes.edgecolor":"#c3c2b7","axes.labelcolor":SEC,
 "axes.titlecolor":INK,"axes.titlesize":12,"axes.labelsize":10,"xtick.color":MUTED,"ytick.color":MUTED,
 "xtick.labelsize":9,"ytick.labelsize":9,"grid.color":GRID,"grid.linewidth":0.7,"legend.frameon":False,
 "legend.fontsize":9,"axes.spines.top":False,"axes.spines.right":False})
df = pd.read_csv(S/"winner_traj.csv", sep=";")
aut = df[(df["stage"] <= 3) & (df["t"] > 0.05)]      # drop t=0: theta undefined at V=0
TK = aut["t"].max(); SEPS = [66.3, 109.2]
T_M11, T_94 = 23.2, 102.5
BO = aut.iloc[-1]

fig, (a1, a2) = plt.subplots(2, 1, figsize=(10.5, 8.4), sharex=True,
                             gridspec_kw={"height_ratios":[1.3,1], "hspace":0.30})
for ax in (a1, a2):
    for t in SEPS: ax.axvline(t, color=MUTED, lw=0.8, ls=(0,(4,3)), zorder=1)
    ax.set_xlim(0, TK*1.045); ax.grid(True, axis="y", zorder=0); ax.set_axisbelow(True)

a1.plot(aut["t"], aut["vartheta"], color=BLUE, lw=2.2, label=r"$\vartheta$  programmed pitch", zorder=3)
a1.plot(aut["t"], aut["theta"],    color=ORANGE, lw=2.2, label=r"$\theta$  flight-path angle", zorder=3)
a1.set_ylabel("angle, deg"); a1.set_ylim(-4, 98)
a1.set_title("Pitch program — 14 arcs (6 / 4 / 4),  $t_в$ = 5.00 s", loc="left", pad=22, fontweight="bold")
a1.legend(loc="upper right", bbox_to_anchor=(1.0, 1.10), ncol=2)
for lbl,a,b in [("stage I",0,SEPS[0]),("II",SEPS[0],SEPS[1]),("III",SEPS[1],TK)]:
    a1.text((a+b)/2, 1.5, lbl, ha="center", va="bottom", color=MUTED, fontsize=9, fontweight="bold")
a1.plot([TK],[BO["theta"]], "o", ms=7, color=ORANGE, mec=SURF, mew=1.8, zorder=5)
a1.annotate(f"burnout\nθ = {BO['theta']:.1f}°", xy=(TK, BO["theta"]), xytext=(-14, 30),
            textcoords="offset points", ha="right", color=SEC, fontsize=9, linespacing=1.3,
            arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))

# regime backgrounds
a2.axvspan(0, T_M11, color=BLUE, alpha=0.05, lw=0, zorder=0)
a2.axvspan(T_94, TK*1.045, color=MUTED, alpha=0.10, lw=0, zorder=0)
# limits drawn ONLY where they apply
for y in (1.5,-1.5):
    a2.plot([0,T_M11],[y,y], color=CRIT, lw=1.4, ls=(0,(5,3)), zorder=4)
for y in (10,-10):
    a2.plot([T_M11,T_94],[y,y], color=CRIT, lw=1.4, ls=(0,(5,3)), zorder=4)
a2.plot(aut["t"], aut["alpha"], color=AQUA, lw=2.1, zorder=3)
a2.axhline(0, color="#c3c2b7", lw=0.8, zorder=1)
a2.set_ylabel("α, deg"); a2.set_xlabel("time, s"); a2.set_ylim(-14, 17.5)
a2.set_title("Angle of attack — each §4.4 limit shown only over the regime it governs",
             loc="left", pad=10, fontweight="bold")
a2.text(T_M11/2, 16.6, "M ≤ 1.1\n$\\varepsilon_1$ = 1.5°", ha="center", va="top", color=CRIT, fontsize=8.5, linespacing=1.3)
a2.text((T_M11+T_94)/2-9, 16.6, "M > 1.1,  H ≤ 94 km\n$\\varepsilon_2$ = 10°", ha="center", va="top", color=CRIT, fontsize=8.5, linespacing=1.3)
a2.text((T_94+TK)/2, 16.6, "H > 94 km\nno α limit", ha="center", va="top", color=MUTED, fontsize=8.5, linespacing=1.3)
for x in (T_M11, T_94):
    a2.axvline(x, color=MUTED, lw=0.9, ls=":", zorder=2)
a2.text(52, 4.2, "achieved   |α| 1.50 / 1.50°   ·   9.99 / 10.00°",
        ha="center", va="center", fontsize=9.5, color=SEC)
fig.savefig(S/"pitch_program.png", dpi=170, bbox_inches="tight"); print("ok")
