from __future__ import annotations

from itertools import repeat
from pathlib import Path
from pprint import pprint
from logging import DEBUG, NullHandler, basicConfig, getLogger

import apscheduler.schedulers.blocking as sched
import click
import zmq

import rover
import controller.messages as msgs
import controller.automaton as ha


class PublisherError(Exception):
    pass


def run(world: str, frequency: int, msg: msgs.Start) -> list[msgs.Step]:
    logger = getLogger("publisher")
    logger.addHandler(NullHandler())

    cmds = iter(msg.commands)
    vehicle = rover.spawn(world, magnet=msg.magnet)
    controller = ha.Automaton(vehicle)
    scheduler = sched.BlockingScheduler()
    history: list[msgs.Step] = []

    def update():
        logger.debug("Running controller update")
        history.append(
            msgs.Step(
                time=vehicle.clock,
                position=vehicle.position,
                heading=vehicle.heading,
                roll=vehicle.roll,
                state=controller.state,
            )
        )

        if controller.state.is_terminal():
            logger.debug("Found terminal state. Shutting down scheduler.")
            scheduler.shutdown()
        else:
            last_state = controller.state
            controller.step(next(cmds))
            logger.debug(f"Stepping controller. Last state: {last_state} -> Current state: {controller.state}")

    logger.debug("Creating controller scheduler job")
    scheduler.add_job(update, "interval", seconds=1/frequency)

    logger.debug("Starting scheduler")
    scheduler.start()

    return history


@click.command()
@click.option("-w", "--world", default="default")
@click.option("-f", "--frequency", type=int, default=1)
@click.option("-s", "--socket", "socket_path", type=click.Path(exists=True, dir_okay=False, writable=True, path_type=Path), default=None)
@click.option("-v", "--verbose", is_flag=True)
def controller(world: str, frequency: int, socket_path: Path | None, verbose: bool):
    logger = getLogger("publisher")
    logger.addHandler(NullHandler())

    if verbose:
        basicConfig(level=DEBUG)

    if socket_path is None:
        logger.debug("No socket provided, starting controller using defaults.")

        msg = msgs.Start(commands=repeat(None), magnet=None)
        history = run(world, frequency, msg)

        pprint(history)
    else:

        with zmq.Context() as ctx:
            with ctx.socket(zmq.REP) as sock:
                with sock.connect(str(socket_path)):
                    logger.debug("Listening for start message.")
                    msg = sock.recv_pyobj()

                    if not isinstance(msg, msgs.Start):
                        sock.send_pyobj(PublisherError(f"Unexpected start message type {type(msg)}"))

                    logger.debug("Start message received. Running simulation.")
                    history = run(world, frequency, msg)
                    sock.send_pyobj(msgs.Result(history))


if __name__ == "__main__":
    controller()
