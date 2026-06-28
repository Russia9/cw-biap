import csv
import math
import matplotlib.pyplot as plt
import matplotlib as mpl

# Contains functions to manage basic plotting and table processing needs.


# Returns a column from a given file
def read_column_from_csv_file(
    filename: str, column_nr: int, do_skip_first_line: bool, delimiter_: str = ";"
):
    ans = []
    with open(filename, mode="r") as file:
        csvFile = csv.reader(file, delimiter=delimiter_)
        if do_skip_first_line:
            next(csvFile)
        for lines in csvFile:
            ans.append(float(lines[column_nr]))
    return ans


# Writes multiple columns to a csv file
def write_to_csv_file(
    filename: str, header_field_names: list, data_to_write: list, delimiter_: str = ";"
):
    with open(filename, mode="w") as file:
        csvwriter = csv.writer(file, delimiter=delimiter_)
        csvwriter.writerow(header_field_names)
        # This evil black magic fuckery transposes all the lists and writes everything in a single go
        # I was initially going to do this manually, but we have a list of lists of floats, and that is essentially a matrix
        # Which is why we transpose it rather than try to manually write everything into a row and write that row
        res = [list(row) for row in zip(*data_to_write)]
        csvwriter.writerows(res)


# Makes a plot or multiple plots depending on arguments
# If two lists are given, plots them together
# If two lists of lists are given, makes figures and plots them all in a single axis and adds legend below
def make_plot(
    x: list,
    y: list,
    x_axis_label: str,
    y_axis_label: str,
    img_name: str = "",
    legend: list = [],
    xlimits: list = [],
    ylimits: list = [],
):
    fig, ax = plt.subplots(layout="constrained")
    if isinstance(y[0], float):
        ax.plot(x, y)
        # fig.subplots_adjust(top=0.9, right=0.95, left = 0.12)

    elif isinstance(y[0], list):
        for i in range(0, len(y)):
            ax.plot(x[i], y[i], label=legend[i])
            ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.15), ncols=2)
            # fig.subplots_adjust(top=0.9, right=0.95, left = 0.12, bottom=0.25)

    ax.set_xlabel(x_axis_label, loc="right")
    ax.set_ylabel(y_axis_label, loc="top", rotation=0)
    # ax.yaxis.set_label_coords(-0.15, 0.95)

    ax.grid()
    if xlimits != []:
        ax.set_xlim(xlimits[0], xlimits[1])
    if ylimits != []:
        ax.set_ylim(ylimits[0], ylimits[1])
    if img_name != "":
        plt.savefig(img_name)
    else:
        plt.show()
    plt.close()