from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass

import apscheduler.schedulers.blocking as sched
import click
import zmq

import automaton
import rover


def _parse_cmds(msg: object) -> Iterator[automaton.Command]:
    ...


@dataclass()
class Step:
    time: float
    position: tuple[float, float, float]
    heading: float
    roll: float
    state: automaton.State


@dataclass()
class Result:
    history: list[Step]


@dataclass()
class Start:
    commands: list[automaton.Command]
    magnet: rover.Magnet | None


class PublisherError(Exception):
    pass


@click.command()
@click.option("-w", "--world", default="default")
@click.option("-f", "--frequency", type=int, default=1)
def main(world: str, frequency: int):
    with zmq.Context() as ctx:
        with ctx.socket(zmq.REP) as sock:
            msg = sock.recv_pyobj()

            if not isinstance(msg, Start):
                sock.send_pyobj(PublisherError(f"Unexpected start message type {type(msg)}"))

            cmds = _parse_cmds(msg)
            vehicle = rover.spawn(world, magnet=msg.magnet)
            controller = automaton.Automaton(vehicle)
            scheduler = sched.BlockingScheduler()
            history = []

            def update():
                history.append(
                    Step(
                        time=controller.vehicle.clock,
                        position=controller.vehicle.position,
                        heading=controller.vehicle.heading,
                        roll=controller.vehicle.roll,
                        state=controller.state,
                    )
                )

                if controller.state.is_terminal():
                    sock.send_pyobj(Result(history))
                    scheduler.shutdown()
                else:
                    controller.step(next(cmds))

            scheduler.add_job(update, "interval", seconds=1/frequency)

            try:
                scheduler.start()
            except SystemExit:
                pass


if __name__ == "__main__":
    main()
