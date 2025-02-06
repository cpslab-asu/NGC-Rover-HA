from __future__ import annotations

import contextlib
import dataclasses as dc
import itertools
import logging
import pathlib
import typing

import attrs
import click
import docker
import gzcm.gazebo
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy.random as rand
import staliro
import zmq

from controller.attacks import FixedSpeed, Magnet, SpeedController, GaussianMagnet
from controller.messages import Result, Start

if typing.TYPE_CHECKING:
    from collections.abc import Generator
    from docker.models.containers import Container

GZ_PORT: typing.Final[int] = 7722
GZ_IMAGE: typing.Final[str] = "ghcr.io/cpslab-asu/ngc-rover-ha/gazebo:harmonic"
GZ_BASE: typing.Final[pathlib.Path] = pathlib.Path("resources/worlds/default.sdf")
GZ_WORLD: typing.Final[pathlib.Path] = pathlib.Path("/tmp/generated.sdf")


@contextlib.contextmanager
def controller_socket(container: Container, port: int) -> Generator[zmq.Socket, None, None]:
    logger = logging.getLogger("test.containers")
    logger.addHandler(logging.NullHandler())

    port_key = f"{port}/tcp"

    while not container.ports.get(port_key):
        container.reload()

    mappings = container.ports[port_key]
    port = int(mappings[0]["HostPort"])

    logger.info(f"Discovered port {port} from controller container")

    with zmq.Context() as ctx:
        with ctx.socket(zmq.REQ) as sock:
            with sock.connect(f"tcp://127.0.0.1:{port}"):
                try:
                    yield sock
                finally:
                    pass


class SimulationError(Exception):
    pass


class AbnormalExitError(SimulationError):
    def __init__(self, exit_code: int):
        super().__init__(f"Automaton docker container exited abnormally (exit code {exit_code}). Please check container logs.")


@contextlib.contextmanager
def controller_container(
    client: docker.DockerClient,
    frequency: int,
    port: int,
    *,
    verbose: bool = False
) -> Generator[Container, None, None]:
    logger = logging.getLogger("test.containers")
    logger.addHandler(logging.NullHandler())
    logger.info(f"Creating controller container with published port {port}")

    command = f"controller --port {port} --world {GZ_WORLD.stem} --frequency {frequency}"

    if verbose:
        command = f"{command} --verbose"

    controller = client.containers.run(
        image="ghcr.io/cpslab-asu/ngc-rover-ha/controller:latest",
        command=command,
        detach=True,
        ports={f"{port}/tcp": None},
    )

    while controller.status != "running":
        controller.reload()

    logger.info("Container controller running.")

    try:
        yield controller
    except KeyboardInterrupt as e:
        controller.kill()
        raise e
    finally:
        result = controller.wait(timeout=1.0)
        exit_code = result["StatusCode"]

        if exit_code != 0 and exit_code != 137:  # 137 is the exit code for a process killed by SIGKILL
            raise AbnormalExitError(exit_code)


@attrs.define()
class Simulation:
    socket: zmq.Socket = attrs.field()

    def run(self, magnet: Magnet | None, speed: SpeedController | None) -> Result:
        self.socket.send_pyobj(
            Start(
                commands=itertools.repeat(None),
                magnet=magnet,
                speed=speed,
            )
        )
        
        msg = self.socket.recv_pyobj()

        if isinstance(msg, Exception):
            raise msg

        if isinstance(msg, Result):
            return msg

        raise TypeError(f"Unexpected type received {type(msg)}")


@contextlib.contextmanager
def start(frequency: int, *, verbose: bool = False) -> Generator[Simulation, None, None]:
    logger = logging.getLogger("test.simulation")
    logger.addHandler(logging.NullHandler())

    client = docker.from_env()
    gz = gzcm.gazebo.Gazebo()

    with controller_container(client, frequency, GZ_PORT, verbose=verbose) as controller:
        with gzcm.gazebo.gazebo(gz, controller, image=GZ_IMAGE, base=GZ_BASE, world=GZ_WORLD):
            with controller_socket(controller, GZ_PORT) as sock:
                logger.info("Simulation stack ready.")

                try:
                    yield Simulation(sock)
                finally:
                    pass


def run(frequency: int, magnet: Magnet | None, speed: SpeedController | None, verbose: bool = False) -> Result:
    with start(frequency, verbose=verbose) as sim:
        result = sim.run(magnet, speed)

    return result


@dc.dataclass()
class Plot:
    trajectory: staliro.Trace[list[float]]
    magnet: tuple[float, float] | None
    color: typing.Literal["r", "g", "b", "k"] = "k"


def plot(*plots: Plot):
    _, ax = plt.subplots()
    ax.set_title("Trajectory")
    # ax.set_xlim(left=0, right=16)
    ax.set_ylim(bottom=-2, top=10)
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


@click.command()
@click.option("-f", "--frequency", type=int, default=2)
@click.option("-s", "--speed", type=float, default=5.0)
@click.option("-m", "--magnet", type=float, nargs=2, default=None)
def simulation(
    speed: float,
    frequency: int,
    magnet: tuple[float, float] | None,
):
    if magnet:
        rng = rand.default_rng()
        magnet_ = GaussianMagnet(x=magnet[0], y=magnet[1], rng=rng)
    else:
        magnet_ = None

    result = run(
        frequency=frequency,
        magnet=magnet_,
        speed=FixedSpeed(speed),
        verbose=False,
    )
    p = Plot(
        magnet=magnet,
        trajectory=staliro.Trace({
            step.time: [step.position[0], step.position[1]] for step in result.history
        }),
    )

    plot(p)


if __name__ == "__main__":
    simulation()
