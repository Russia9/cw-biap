import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Set academic style (visual styling only — computation/logic unchanged)
try:
    # Matplotlib >= 3.6
    plt.style.use("seaborn-v0_8-whitegrid")
except Exception:
    try:
        plt.style.use("ggplot")
    except Exception:
        pass

plt.rcParams.update(
    {
        # Typography
        "font.family": "serif",
        "font.serif": ["STIX Two Text", "STIXGeneral", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 10,
        # Axes / grid
        "axes.linewidth": 0.9,
        "axes.labelsize": 11,
        "axes.facecolor": "white",
        "figure.facecolor": "white",
        "grid.linewidth": 0.6,
        "grid.alpha": 0.35,
        # Lines
        "lines.linewidth": 1.6,
        "lines.solid_capstyle": "round",
        "lines.dash_capstyle": "round",
        # Ticks
        "xtick.major.width": 0.8,
        "ytick.major.width": 0.8,
        "xtick.minor.width": 0.6,
        "ytick.minor.width": 0.6,
        "xtick.direction": "in",
        "ytick.direction": "in",
        "xtick.top": True,
        "ytick.right": True,
        # Legend
        "legend.frameon": True,
        "legend.fancybox": True,
        "legend.framealpha": 0.95,
    }
)
# Configuration
out_dir = Path("out")
charts_dir = out_dir / "charts"
charts_dir.mkdir(exist_ok=True)

subdirs = sorted([d for d in out_dir.iterdir() if d.is_dir() and d.name != "charts"])

# Colors and line styles for trajectories (greyscale)
# Format: (color, linestyle)
trajectory_styles = [
    ("#000000", "-"),  # 1: black solid
    ("#000000", "--"),  # 2: black dashed
    ("#808080", "-"),  # 3: grey solid
    ("#808080", "--"),  # 4: grey dashed
]

# Parameters to plot: (column_name, label, filename, skip_coast)
parameters = [
    ("Theta, град.", r"$\theta$, град", "Theta", False),
    ("Theta_с, град.", r"$\theta_c$, град", "Theta_c", False),
    ("theta, град.", r"$\vartheta$, град", "theta_sm", True),
    ("alpha, град.", r"$\alpha$, град", "alpha", True),
    ("V, м/с", r"$V$, м/с", "V", False),
    ("h, м", r"$h$, м", "h", False),
]

# Create a chart for each parameter
for param_col, param_label, param_file, skip_coast in parameters:
    fig, ax = plt.subplots(figsize=(8, 4))

    # Process each trajectory
    for idx, subdir in enumerate(subdirs):
        # Parse directory name
        dir_name = subdir.name
        parts = dir_name.split("_")
        traj_num = parts[0]

        # Read trajectory data
        traj_file = subdir / "traj.csv"
        df = pd.read_csv(traj_file, sep=";")

        # Extract data
        t = df["t, с"].values
        param_values = df[param_col].values
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

        color, base_linestyle = trajectory_styles[idx % len(trajectory_styles)]

        # Identify continuous segments where dm is either all 0 or all non-zero
        segments = []
        current_segment_start = 0
        current_is_coast = dm[0] == 0

        for i in range(1, len(dm)):
            is_coast = dm[i] == 0
            if is_coast != current_is_coast:
                # Segment boundary - save current segment
                segments.append((current_segment_start, i, current_is_coast))
                current_segment_start = i
                current_is_coast = is_coast

        # Add the last segment
        segments.append((current_segment_start, len(dm), current_is_coast))

        # Plot each segment
        for seg_start, seg_end, is_coast in segments:
            # Skip coast segments for angle parameters
            if skip_coast and is_coast:
                continue
            # Use base linestyle (trajectory linestyle is already set)
            linestyle = base_linestyle
            ax.plot(
                t[seg_start : seg_end + 1],
                param_values[seg_start : seg_end + 1],
                color=color,
                linewidth=1.6,
                linestyle=linestyle,
            )

        # Add legend label for trajectory (empty plot for legend only)
        ax.plot(
            [],
            [],
            color=color,
            linewidth=1.6,
            linestyle=base_linestyle,
            label=f"Траектория {traj_num}",
        )

        # Plot transition points as colored dots
        if transition_indices:
            transition_t = t[transition_indices]
            transition_param = param_values[transition_indices]
            ax.plot(
                transition_t,
                transition_param,
                "o",
                color=color,
                markersize=4.0,
                markeredgecolor="white",
                markeredgewidth=0.6,
                zorder=6,
            )

    # Labels and formatting
    ax.set_xlabel(r"$t$, с", fontsize=11)
    ax.set_ylabel(param_label, fontsize=11)
    ax.set_axisbelow(True)
    ax.minorticks_on()
    ax.grid(True)
    ax.legend(
        loc="best",
        fontsize=9,
        frameon=True,
        fancybox=True,
        edgecolor="0.3",
        framealpha=0.95,
    )
    ax.tick_params(labelsize=9)

    plt.tight_layout(pad=0.3)
    plt.savefig(charts_dir / f"{param_file}.png", dpi=600, bbox_inches="tight")
    plt.close()

print(f"Charts saved to {charts_dir}")
