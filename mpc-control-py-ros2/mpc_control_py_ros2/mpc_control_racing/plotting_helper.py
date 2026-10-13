from typing import List, Tuple, Optional

import numpy as np
import matplotlib.pyplot as plt

def plot_bar_plot_helper(plot_data: List[Tuple[float, str]], labels, ylabel, title: str = '', width: float=0.8, figure_filename: Optional[str]=None, figsize=None):

    fig, ax = plt.subplots(figsize=figsize)
    num_entries = len(labels)
    bottom = np.zeros(num_entries)

    for timing, label in plot_data:
        if not np.all(timing == 0):
            plt.bar(labels, timing, width, label=label, bottom=bottom)
            bottom += timing

    ax.set_title(title)
    plt.xticks(rotation=10)
    plt.grid(axis="y")
    plt.ylabel(ylabel)
    plt.tight_layout()
    ax.legend()

    if figure_filename is not None:
        plt.savefig(figure_filename)
        print(f"Saved figure to {figure_filename}")
    plt.show()


def plot_acados_timings(timings: dict, timings_unit: str = "ms"):
    if timings_unit == "ms":
        factor = 1e3
    elif timings_unit == "s":
        factor = 1e6
    elif timings_unit == "us":
        factor = 1
    else:
        raise ValueError(f"Timings unit {timings_unit} not supported")

    for k in timings:
        timings[k] = np.mean(timings[k], axis=0)

    time_tot = factor * np.array(timings["time_tot"])
    time_reg = factor * np.array(timings["time_reg"])
    time_sim = factor * np.array(timings["time_sim"])
    time_lin = factor * np.array(timings["time_lin"])
    time_qp_solver_call = factor * np.array(timings["time_qp_solver_call"])
    time_qp = factor * np.array(timings["time_qp"])
    time_glob = factor * np.array(timings["time_glob"])
    time_qp_remaining = time_qp - time_qp_solver_call

    time_remaining = time_tot - time_qp - time_reg - time_lin - time_glob
    time_remaining_lin = time_lin - time_sim

    bar_data = [(time_sim, "integrator"),
                (time_remaining_lin, "remaining linearization"),
                (time_qp_solver_call, "QP solver"),
                (time_qp_remaining, "QP preprocessing"),
                (time_reg, "regularization"),
                (time_glob, "globalization"),
                (time_remaining, "remaining"),
                ]
    labels = ['default']
    ylabel = f'time in {timings_unit}'
    plot_bar_plot_helper(bar_data, labels, ylabel)