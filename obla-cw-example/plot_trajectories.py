import os
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# --- Visual style (only) -------------------------------------------------------
# Use a modern, readable theme; fall back gracefully if style name differs.
for _style in ("seaborn-v0_8-whitegrid", "seaborn-whitegrid", "ggplot"):
    try:
        plt.style.use(_style)
        break
    except OSError:
        pass

# Typography + clean, high-contrast axes (scientific plotting)
plt.rcParams["font.family"] = "STIX Two Text"
plt.rcParams["mathtext.fontset"] = "stix"
plt.rcParams["font.size"] = 10

plt.rcParams["figure.facecolor"] = "white"
plt.rcParams["axes.facecolor"] = "white"
plt.rcParams["axes.edgecolor"] = "#222222"
plt.rcParams["axes.linewidth"] = 0.9
plt.rcParams["axes.labelcolor"] = "#222222"

plt.rcParams["grid.linewidth"] = 0.6
plt.rcParams["grid.alpha"] = 0.35
plt.rcParams["grid.color"] = "#9aa0a6"

plt.rcParams["lines.linewidth"] = 1.6
plt.rcParams["lines.solid_capstyle"] = "round"

plt.rcParams["xtick.major.width"] = 0.9
plt.rcParams["ytick.major.width"] = 0.9
plt.rcParams["xtick.minor.width"] = 0.6
plt.rcParams["ytick.minor.width"] = 0.6
plt.rcParams["xtick.color"] = "#222222"
plt.rcParams["ytick.color"] = "#222222"
plt.rcParams["xtick.direction"] = "in"
plt.rcParams["ytick.direction"] = "in"
plt.rcParams["xtick.top"] = True
plt.rcParams["ytick.right"] = True

# A nicer default color cycle (works even if there are many trajectories)
plt.rcParams["axes.prop_cycle"] = mpl.cycler(color=plt.get_cmap("tab10").colors)
# -----------------------------------------------------------------------------

# Configuration
R_M = 1738 * 1000  # Moon radius in meters

X_MIN = -20000  # meters
X_MAX = 2000000  # meters
Y_MIN = -400000  # meters
Y_MAX = 500000  # meters

# Directories
out_dir = Path("out")
subdirs = sorted([d for d in out_dir.iterdir() if d.is_dir()])

# Create figure (academic paper format - approximately 3.5 inches for single column)
fig, ax = plt.subplots(figsize=(7, 7))

# Draw Moon surface
# Moon center is at (0, -R_M) since (0, 0) is on the surface
moon_center_x = 0
moon_center_y = -R_M

theta = np.linspace(0, 2 * np.pi, 1000)
moon_x = moon_center_x + R_M * np.cos(theta)
moon_y = moon_center_y + R_M * np.sin(theta)
ax.fill(moon_x, moon_y, color="#b0b0b0", alpha=0.35, label="Луна")
ax.plot(moon_x, moon_y, color="#111111", linewidth=1.1)

# Greyscale styles for trajectories
trajectory_styles = [
    {"color": "black", "linestyle": "-"},  # Trajectory 1: black solid
    {"color": "black", "linestyle": "--"},  # Trajectory 2: black dashed
    {"color": "grey", "linestyle": "-"},  # Trajectory 3: grey solid
    {"color": "grey", "linestyle": "--"},  # Trajectory 4: grey dashed
]

# Process each trajectory
for idx, subdir in enumerate(subdirs):
    # Parse directory name: TRAJNUMBER_P_Htarget
    dir_name = subdir.name
    parts = dir_name.split("_")
    if len(parts) != 3:
        continue
    traj_num = parts[0]
    h_target = float(parts[2]) * 1000  # Convert km to meters

    # Read trajectory data
    traj_file = subdir / "traj.csv"
    df = pd.read_csv(traj_file, sep=";")

    # Extract data and convert to starting coordinate system
    x = df["x, м"].values
    y = (
        df["y, м"].values - R_M
    )  # Convert to starting coordinates where (0,0) is on surface
    m = df["m, кг"].values

    # Calculate dm (mass difference between consecutive points)
    dm = np.diff(m)

    # Find transition points where dm changes (exclude first and last)
    transition_indices = []
    for i in range(1, len(dm) - 1):
        # Transition from non-zero to zero (engine off)
        if dm[i - 1] != 0 and dm[i] == 0:
            transition_indices.append(i)
        # Transition from zero to non-zero (engine on)
        elif dm[i - 1] == 0 and dm[i] != 0:
            transition_indices.append(i)

    style = trajectory_styles[idx % len(trajectory_styles)]
    color = style["color"]
    base_linestyle = style["linestyle"]

    # Plot entire trajectory with same linestyle
    ax.plot(
        x,
        y,
        color=color,
        linewidth=1.8,
        linestyle=base_linestyle,
    )

    # Add legend label for trajectory (empty plot for legend only)
    ax.plot(
        [],
        [],
        color=color,
        linewidth=1.8,
        linestyle=base_linestyle,
        label=f"Траектория {traj_num}",
    )

    # Plot transition points as colored dots
    if transition_indices:
        transition_x = x[transition_indices]
        transition_y = y[transition_indices]
        ax.plot(
            transition_x,
            transition_y,
            "o",
            color=color,
            markersize=3.2,
            markeredgecolor="white",
            markeredgewidth=0.6,
            zorder=5,
        )

    # Draw desired orbit as dotted circle
    orbit_radius = R_M + h_target
    orbit_x = moon_center_x + orbit_radius * np.cos(theta)
    orbit_y = moon_center_y + orbit_radius * np.sin(theta)
    # Only add legend label for first orbit
    orbit_label = "Целевые орбиты" if idx == 3 else None
    ax.plot(
        orbit_x,
        orbit_y,
        color="lightgrey",
        linestyle=":",
        linewidth=1.2,
        label=orbit_label,
    )

# Set plot limits
ax.set_xlim(X_MIN, X_MAX)
ax.set_ylim(Y_MIN, Y_MAX)

# Labels and formatting
ax.set_xlabel("$x$, м", fontsize=11)
ax.set_ylabel("$y$, м", fontsize=11)
ax.grid(True)
ax.legend(
    loc="best",
    fontsize=9,
    frameon=True,
    fancybox=True,
    shadow=False,
    edgecolor="black",
    framealpha=0.9,
)
ax.set_aspect("equal")
ax.minorticks_on()
ax.tick_params(labelsize=9)

plt.tight_layout(pad=0.3)
plt.savefig("out/charts/trajectories.png", dpi=600, bbox_inches="tight")
plt.show()
