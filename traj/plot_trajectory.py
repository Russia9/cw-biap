"""Plot the Go trajectory output (traj/out/traj.csv).

Usage:
    uv run python traj/plot_trajectory.py [path/to/traj.csv]
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

csv = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent / "out" / "traj.csv"
df = pd.read_csv(csv, sep=";")
outdir = csv.parent

# stage-transition times (first row of each new stage)
seps = df.loc[df["stage"].diff().fillna(0) != 0, "t"].tolist()


def marks(ax):
    for t in seps:
        ax.axvline(t, color="0.7", lw=0.8, ls="--")


fig, ax = plt.subplots(2, 3, figsize=(15, 8))

ax[0, 0].plot(df["t"], df["H"] / 1000)
ax[0, 0].set(xlabel="t, s", ylabel="H, km", title="Altitude")
marks(ax[0, 0])

ax[0, 1].plot(df["t"], df["V"])
ax[0, 1].set(xlabel="t, s", ylabel="V, m/s", title="Velocity")
marks(ax[0, 1])

ax[0, 2].plot(df["t"], df["vartheta"], label="ϑ (pitch)")
ax[0, 2].plot(df["t"], df["theta"], label="θ (flight angle)")
ax[0, 2].plot(df["t"], df["alpha"], label="α")
ax[0, 2].set(xlabel="t, s", ylabel="deg", title="Angles")
ax[0, 2].legend()
marks(ax[0, 2])

ax[1, 0].plot(df["t"], df["q"] / 1000)
ax[1, 0].set(xlabel="t, s", ylabel="q, kPa", title="Dynamic pressure")
marks(ax[1, 0])

ax[1, 1].plot(df["t"], df["Mach"])
ax[1, 1].set(xlabel="t, s", ylabel="Mach", title="Mach")
marks(ax[1, 1])

ax[1, 2].plot(df["x"] / 1000, df["H"] / 1000)
ax[1, 2].set(xlabel="x, km", ylabel="H, km", title="Trajectory (downrange vs altitude)")

fig.tight_layout()
fig.savefig(outdir / "trajectory.png", dpi=150)
print(f"saved {outdir / 'trajectory.png'}")
