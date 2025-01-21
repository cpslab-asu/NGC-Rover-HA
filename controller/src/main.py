from __future__ import annotations

from itertools import repeat
from pathlib import Path
from pprint import pprint
from time import time_ns
from logging import DEBUG, NullHandler, basicConfig, getLogger

import apscheduler.schedulers.blocking as sched
import click
import zmq

import rover
import controller.messages as msgs
import controller.attacks as atk
import controller.automaton as ha


class PublisherError(Exception):
    pass


def run(world: str, frequency: int, msg: msgs.Start) -> list[msgs.Step]:
    logger = getLogger("publisher")
    logger.addHandler(NullHandler())

    step_size: float = 1.0/frequency
    cmds = iter(msg.commands)
    scheduler = sched.BlockingScheduler()
    history: list[msgs.Step] = []

    speed_ctl = msg.speed or atk.FixedSpeed(1.0)
    magnet = msg.magnet or atk.StationaryMagnet(0.0)
    vehicle = rover.spawn(world, magnet=magnet)
    controller = ha.Automaton(vehicle, step_size)

    vehicle.wait()
    tstart = vehicle.clock

    def update():
        tsim = vehicle.clock - tstart
        logger.debug("Running controller update.")
        history.append(
            msgs.Step(
                time=tsim,
                position=vehicle.position,
                heading=vehicle.heading,
                roll=vehicle.roll,
                state=controller.state,
            )
        )

        speed = abs(speed_ctl.speed(tsim))
        vehicle.velocity = speed * controller.velocity
        vehicle.omega = speed * controller.omega

        if controller.state.is_terminal():
            logger.debug("Found terminal state. Shutting down scheduler.")
            scheduler.shutdown()
        else:
            controller.step(next(cmds))

    logger.debug("Creating controller scheduler job")
    scheduler.add_job(update, "interval", seconds=step_size)

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
