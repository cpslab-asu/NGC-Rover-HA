from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from matplotlib import pyplot as plt
from matplotlib import patches as patches
from staliro import Trace

from controller.automaton import State


@dataclass()
class Plot:
    trajectory: Trace[list[float]]
    magnet: tuple[float, float] | None
    color: Literal["r", "g", "b", "k"] = "k"


@dataclass()
class StatePlot:
    trajectory: Trace[State]


def plot(*plots: Plot, states: StatePlot | None = None):
    plt.rc("font", size=18)

    if states:
        _, axs = plt.subplots(1, 2)
        axs[1].set_title("Discrete State Trajectory")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("State")
        axs[1].plot(
            list(states.trajectory.times),
            [int(state) for state in states.trajectory.states]
        )

        ax = axs[0]
    else:
        _, ax = plt.subplots(1, 2)

    ax.set_title("Trajectory")
    # ax.set_xlim(left=0, right=16)
    ax.set_ylim(bottom=-2, top=10)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.add_patch(patches.Rectangle((0, 0), 8, 8, linewidth=1, edgecolor="r", fill=False))
    magnets = [plot.magnet for plot in plots if plot.magnet is not None]

    if magnets:
        ax.scatter(
            [magnet[0] for magnet in magnets],
            [magnet[1] for magnet in magnets],
            s=None,
            c="b",
        )

    for plot in plots:
        # ax.add_patch(patches.Circle(plot.magnet, 0.1, linewidth=1, edgecolor="b"))

        times = list(plot.trajectory.times)
        ax.plot(
            [plot.trajectory[time][0] for time in times],
            [plot.trajectory[time][1] for time in times],
            plot.color,
        )

    plt.show(block=True)
