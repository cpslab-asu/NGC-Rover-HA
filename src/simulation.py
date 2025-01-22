from __future__ import annotations

import contextlib
import itertools
import logging
import pathlib
import typing

import attrs
import click
import docker
import gzcm.gazebo
import matplotlib.pyplot as plt
import zmq

from controller.attacks import FixedSpeed, Magnet, SpeedController
from controller.messages import Result, Start

if typing.TYPE_CHECKING:
    from collections.abc import Generator
    from docker.models.containers import Container

GZ_PORT: typing.Final[int] = 7722
GZ_IMAGE: typing.Final[str] = "ghcr.io/cpslab-asu/gzcm/px4/gazebo:harmonic"
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


@click.command()
@click.option("-s", "--speed", type=float, default=5.0)
@click.option("-f", "--frequency", type=int, default=2)
def simulation(speed: float, frequency: int):
    result = run(
        frequency=frequency,
        magnet=None,
        speed=FixedSpeed(speed),
        verbose=False,
    )

    times = [step.time for step in result.history]
    fig, axs = plt.subplots(2, 2)

    axs[0, 0].plot(times, [step.position[0] for step in result.history])
    axs[0, 0].set_xlabel("Time")
    axs[0, 0].set_ylabel("X position")

    axs[0, 1].plot(times, [step.position[1] for step in result.history])
    axs[0, 1].set_xlabel("Time")
    axs[0, 1].set_ylabel("Y position")

    axs[1, 0].plot(times, [step.heading for step in result.history])
    axs[1, 0].set_xlabel("Time")
    axs[1, 0].set_ylabel("Heading")

    axs[1, 1].plot(times, [step.roll for step in result.history])
    axs[1, 1].set_xlabel("Time")
    axs[1, 1].set_ylabel("Roll")

    plt.show(block=True)


if __name__ == "__main__":
    simulation()
