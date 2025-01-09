from __future__ import annotations

import contextlib
import dataclasses as dc
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

import publisher

if typing.TYPE_CHECKING:
    from collections.abc import Generator


@contextlib.contextmanager
def temp_path() -> Generator[pathlib.Path, None, None]:
    with tempfile.TemporaryDirectory() as tempdir:
        try:
            yield pathlib.Path(tempdir) / "socket"
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
def rover_container(client: docker.DockerClient, sock_path: pathlib.Path):
    sock_dir = sock_path.parent
    container = client.containers.run(
        image="",
        detach=True,
        volumes={"/var/run/receiver": {"path": str(sock_dir), "mode": "rw"}},
    )

    try:
        yield container
    finally:
        if container.status != "exited":
            container.kill()
            container.wait()

        container.remove()


def run_simulation() -> publisher.Result:
    client = docker.from_env()
    gz = gzcm.Gazebo()

    with temp_path() as sock_path:
        with create_socket(sock_path) as sock:
            with rover_container(client, sock_path) as rover:
                with gzcm.gazebo.gazebo(gz, rover, client=client):
                    sock.send_pyobj(None)
                    result = sock.recv_pyobj()

                    if not isinstance(result, publisher.Result):
                        raise TypeError(f"Unexpected type received {type(result)}")

                    return result


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


    spec = staliro.specifications.rtamt.parse_dense("always (x > 0)")
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
