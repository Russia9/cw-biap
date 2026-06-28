"""Plot the Go trajectory output (traj/out/traj.csv).

Usage:
    uv run python traj/plot_trajectory.py [path/to/traj.csv]
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

RZ = 6371000.0  # Earth radius [m] (mirrors traj/config.go)

csv = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent / "out" / "traj.csv"
df = pd.read_csv(csv, sep=";")
outdir = csv.parent

# stage-transition times (first row of each new stage)
seps = df.loc[df["stage"].diff().fillna(0) != 0, "t"].tolist()

# Powered (active, АУТ) vs passive (ПУТ) segments. The active leg is stages 1-3;
# the passive leg is the payload coast + re-entry (stage 4).
t_aut = df.loc[df["m"] <= df["m"].min() + 1.0, "t"].iloc[0]
aut = df[df["stage"] <= 3]
put = df[df["stage"] == 4]


def marks(ax):
    for t in seps:
        ax.axvline(t, color="0.7", lw=0.8, ls="--")


fig, ax = plt.subplots(4, 4, figsize=(20, 16))

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

# Downrange distance along the surface, L(t).
L = RZ * np.arctan2(df["x"], RZ + df["y"]) / 1000
ax[0, 3].plot(df["t"], L)
ax[0, 3].set(xlabel="t, s", ylabel="L, km", title="Downrange L")
marks(ax[0, 3])

ax[1, 0].plot(df["t"], df["q"] / 1000)
ax[1, 0].set(xlabel="t, s", ylabel="q, kPa", title="Dynamic pressure")
marks(ax[1, 0])

ax[1, 1].plot(df["t"], df["Mach"])
ax[1, 1].set(xlabel="t, s", ylabel="Mach", title="Mach")
marks(ax[1, 1])

# Trajectory in the launch-centred Cartesian frame, y(x), with the curved
# Earth surface (a circle of radius RZ centred at (0, -RZ)) drawn beneath it.
ax[1, 2].plot(df["x"] / 1000, df["y"] / 1000, label="trajectory")
phi_max = 1.05 * np.arctan2(df["x"], RZ + df["y"]).max()
phi = np.linspace(0, phi_max, 400)
ax[1, 2].plot(RZ * np.sin(phi) / 1000, (RZ * np.cos(phi) - RZ) / 1000, color="0.5", lw=1, label="Earth surface")
ax[1, 2].set(xlabel="x, km", ylabel="y, km", title="Trajectory (Earth frame)")
ax[1, 2].set_aspect("equal")
ax[1, 2].legend()

# Mass over the powered (active) segment only.
ax[1, 3].plot(aut["t"], aut["m"] / 1000)
ax[1, 3].set(xlabel="t, s", ylabel="m, t", title="Mass (АУТ)")
marks(ax[1, 3])
ax[1, 3].set_xlim(0, t_aut)

# Third row: angles, Mach and altitude over the powered (active) segment only.
ax[2, 0].plot(aut["t"], aut["vartheta"], label="ϑ (pitch)")
ax[2, 0].plot(aut["t"], aut["theta"], label="θ (flight angle)")
ax[2, 0].plot(aut["t"], aut["alpha"], label="α")
ax[2, 0].set(xlabel="t, s", ylabel="deg", title="Angles (АУТ)")
ax[2, 0].legend()
marks(ax[2, 0])
ax[2, 0].set_xlim(0, t_aut)

ax[2, 1].plot(aut["t"], aut["Mach"])
ax[2, 1].set(xlabel="t, s", ylabel="Mach", title="Mach (АУТ)")
marks(ax[2, 1])
ax[2, 1].set_xlim(0, t_aut)

ax[2, 2].plot(aut["t"], aut["H"] / 1000)
ax[2, 2].set(xlabel="t, s", ylabel="H, km", title="Altitude (АУТ)")
marks(ax[2, 2])
ax[2, 2].set_xlim(0, t_aut)

ax[2, 3].axis("off")

# Fourth row: aerodynamic forces (drag X, lift Y) on the active and passive legs.
ax[3, 0].plot(aut["t"], aut["X"] / 1000, label="X (drag)")
ax[3, 0].plot(aut["t"], aut["Y"] / 1000, label="Y (lift)")
ax[3, 0].set(xlabel="t, s", ylabel="force, kN", title="Aero forces (АУТ)")
ax[3, 0].legend()
marks(ax[3, 0])
ax[3, 0].set_xlim(0, t_aut)

ax[3, 1].plot(put["t"], put["X"] / 1000, label="X (drag)")
ax[3, 1].plot(put["t"], put["Y"] / 1000, label="Y (lift)")
ax[3, 1].set(xlabel="t, s", ylabel="force, kN", title="Aero forces (ПУТ)")
ax[3, 1].legend()

ax[3, 2].axis("off")
ax[3, 3].axis("off")

fig.tight_layout()
fig.savefig(outdir / "trajectory.png", dpi=150)
print(f"saved {outdir / 'trajectory.png'}")
