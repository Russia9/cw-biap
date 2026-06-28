import libs.python_plots.table_processing as mptl
from matplotlib.patches import Circle
import matplotlib.pyplot as plt

t = mptl.read_column_from_csv_file("trajectory.csv", 0, True, ",")
x = mptl.read_column_from_csv_file("trajectory.csv", 1, True, ",")
y = mptl.read_column_from_csv_file("trajectory.csv", 2, True, ",")
v = mptl.read_column_from_csv_file("trajectory.csv", 5, True, ",")
pitch = mptl.read_column_from_csv_file("trajectory.csv", 6, True, ",")
f_angle = mptl.read_column_from_csv_file("trajectory.csv", 7, True, ",")
aoa = mptl.read_column_from_csv_file("trajectory.csv", 8, True, ",")
m = mptl.read_column_from_csv_file("trajectory.csv", 9, True, ",")
Mach = mptl.read_column_from_csv_file("trajectory.csv", 10, True, ",")
altitude = mptl.read_column_from_csv_file("trajectory.csv", 11, True, ",")
l_range = mptl.read_column_from_csv_file("trajectory.csv", 12, True, ",")
x_aero = mptl.read_column_from_csv_file("trajectory.csv", 14, True, ",")
y_aero = mptl.read_column_from_csv_file("trajectory.csv", 15, True, ",")
mz_aero = mptl.read_column_from_csv_file("trajectory.csv", 16, True, ",")

omega_z = mptl.read_column_from_csv_file("trajectory.csv", 17, True, ",")
stagenr = mptl.read_column_from_csv_file("trajectory.csv", 18, True, ",")

active_time = 51.6+50.5

#mptl.make_plot(x, y, "x, m", "y, m", "plots/xy.png", [])
# fig, ax = plt.subplots(layout="constrained")
# ax.add_patch(Circle((0, -6371), 6371, fill = False, color = 'gray'))
# ax.plot(x, y)
# ax.set_xlim(-100, 2250)
# ax.set_ylim(-450, 1800)
# ax.grid()
# ax.set_xlabel(r"$x_c, км$", loc="right")
# ax.set_ylabel(r"$y_c, км$", loc="top", rotation=0)
# plt.show()
# plt.close()

mptl.make_plot(t, altitude, "t, с", "H, км", "plots/H.png", [], [0, t[-1]], [0, 1600])
mptl.make_plot(t, l_range, "t, с", "L, м", "plots/L.png", [])
mptl.make_plot(t, pitch, "t, с", r"$\vartheta, \degree$", "plots/pitch.png", [], [0, active_time-0.01], [50, 100])
mptl.make_plot(t, f_angle, "t, с", r"$\theta, \degree$", "plots/f_angle.png", [])
mptl.make_plot(t, v, "t, с", "V, м/с", "plots/v.png", [], [t[0], t[-1]], [0, 5200])
mptl.make_plot(t, x_aero, "t, с", "X", "plots/xaero.png", [])
mptl.make_plot(t, y_aero, "t, с", "Y", "plots/yaero.png", [])
mptl.make_plot(t, mz_aero, "t, с", r"$M_z$", "plots/mzaero.png", [])
mptl.make_plot(t, omega_z, "t, с", r"$\omega_z \degree/с$", "plots/omegaz.png", [])

mptl.make_plot(t, m, "t, с", "m, кг", "plots/m.png", [], [t[0], active_time+10], [0, 13000])
mptl.make_plot(t, aoa, "t, с", r"$\alpha, \degree$", "plots/aoa.png", [], [0, t[-1]], [-10, 0.1])
mptl.make_plot(t, Mach, "t, с", "M", "plots/Mach.png", [])
mptl.make_plot(t, stagenr, "t, с", r"stage_N", "plots/staging.png", [])

# def plot_coeff(name: str, aoa_grid: list, x_label: str, y_label: str):
#     c = []
#     for i in range(0, len(aoa_grid)):
#         c.append(read_column_from_csv_file(name, i, False, ","))

#     m = []
#     leg = []
#     for i in range(0, len(aoa_grid)):
#         m.append(c[0])
#         leg.append(str(aoa_grid[i]))
#     make_plot(m, c[1:-1], x_label, y_label, "", leg)


# aoa_grid_1st_stage = [0.0, 3.0, 5.0, 7.0, 10.0]

# aoa_grid_2nd_stage = [0.0, 3.0, 5.0, 7.0, 10.0]

# aoa_grid_3rd_stage = [0.0, 3.0, 7.0, 11.0, 15.0]
# plot_coeff("Cx1.csv", aoa_grid_1st_stage, "M", "Cx")
# plot_coeff("Cy1.csv", aoa_grid_1st_stage, "M", "Cy")

# plot_coeff("Cx2.csv", aoa_grid_2nd_stage, "M", "Cx")
# plot_coeff("Cy2.csv", aoa_grid_2nd_stage, "M", "Cy")

# plot_coeff("Cx3.csv", aoa_grid_3rd_stage, "M", "Cx")
# plot_coeff("Cy3.csv", aoa_grid_3rd_stage, "M", "Cy")
# plot_coeff("mz3.csv", aoa_grid_3rd_stage, "M", "mz")

# name = "Cx.csv"

# m = read_column_from_csv_file(name, 0, False, ",")
# Cx0 = read_column_from_csv_file(name, 1, False, ",")
# Cx3 = read_column_from_csv_file(name, 2, False, ",")
# Cx5 = read_column_from_csv_file(name, 3, False, ",")
# Cx7 = read_column_from_csv_file(name, 4, False, ",")
# Cx10 = read_column_from_csv_file(name, 5, False, ",")

# make_plot(
#     [m, m, m, m, m],
#     [Cx0, Cx3, Cx5, Cx7, Cx10],
#     "M",
#     "Cx",
#     ["aoa = 0", "3", "5", "7", "10"],
# )
