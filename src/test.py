from __future__ import annotations

import collections.abc
import contextlib
import dataclasses as dc
import itertools
import pathlib
import tempfile
import typing

import click
import docker
import gzcm
import gzcm.gazebo
import staliro
import staliro.optimizers
import staliro.specifications.rtamt
import zmq

import attack
import automaton

if typing.TYPE_CHECKING:
    from collections.abc import Generator

GZ_IMAGE: typing.Final[str] = "ghcr.io/cpslab-asu/gzcm/px4/gazebo:harmonic"
GZ_WORLD: typing.Final[str] = "generated"


@contextlib.contextmanager
def temp_path() -> Generator[pathlib.Path, None, None]:
    with tempfile.TemporaryDirectory() as tempdir:
        try:
            yield pathlib.Path(tempdir) / "transport.sock"
        finally:
            pass


@contextlib.contextmanager
def create_socket(path: pathlib.Path) -> Generator[zmq.Socket, None, None]:
    with zmq.Context() as ctx:
        with ctx.socket(zmq.REQ) as sock:
            with sock.connect(str(path)):
                try:
                    yield sock
                finally:
                    pass


@contextlib.contextmanager
def rover_container(client: docker.DockerClient, world: str, sock_path: pathlib.Path):
    sock_dir = sock_path.parent
    container = client.containers.run(
        image="ghcr.io/cpslab-asu/ngc-rover-ha/rover:latest",
        command=f"publisher --world {world}",
        detach=True,
        volumes={
            "/var/run/rover": {"path": str(sock_dir), "mode": "rw"},
        },
    )

    try:
        yield container
    finally:
        exit_code_str: str | None = container.attrs["Status"].get("ExitCode")
        exit_code: int | None = int(exit_code_str) if exit_code_str else None

        if container.status == "exited" and exit_code != 0:
            raise RuntimeError("Automaton docker container exited abnormally. Please check container logs.")
        elif container.status != "exited":
            container.kill()
            container.wait()

        container.remove()


@dc.dataclass()
class Step:
    time: float
    position: tuple[float, float, float]
    heading: float
    roll: float
    state: automaton.State


@dc.dataclass()
class Result:
    history: list[Step]


@dc.dataclass()
class Start:
    commands: collections.abc.Iterable[automaton.Command | None]
    magnet: attack.Magnet | None


def run_simulation() -> Result:
    client = docker.from_env()
    gz = gzcm.Gazebo()

    with temp_path() as sock_path:
        with create_socket(sock_path) as sock:
            with rover_container(client, GZ_WORLD, sock_path) as rover:
                with gzcm.gazebo.gazebo(gz, rover, image=GZ_IMAGE, client=client, world=pathlib.Path(f"/tmp/{GZ_WORLD}.sdf")):
                    sock.send_pyobj(Start(commands=itertools.repeat(None), magnet=None))
                    msg = sock.recv_pyobj()

                    if isinstance(msg, Exception):
                        raise msg

                    if not isinstance(msg, Result):
                        raise TypeError(f"Unexpected type received {type(msg)}")

                    return msg


@click.group()
def test():
    pass


@test.command()
def cpv1():
    @staliro.models.model()
    def model(sample: staliro.Sample) -> staliro.Trace[dict[str, float]]:
        sim_result = run_simulation()
        trace = {
            step.time: {
                "x": step.position[0],
                "y": step.position[1],
                "z": step.position[2],
                "theta": step.heading,
                "omega": step.roll,
            }
            for step in sim_result.history
        }

        return staliro.Trace(trace)


    spec = staliro.specifications.rtamt.parse_dense("always (x > 0)")  # Use reference trajectory to assert error never exceeds given bound
    opt = staliro.optimizers.UniformRandom() # TODO: replace with SOAR
    opts = staliro.TestOptions(
        runs=1,
        iterations=100,
        signals={},
    )
    test_result = staliro.test(model, spec, opt, opts)

    # TODO: do something with the result


@test.command()
def cpv2():
    pass


@test.command()
def cpv3():
    pass


if __name__ == "__main__":
    test()
