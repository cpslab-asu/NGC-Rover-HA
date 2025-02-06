from __future__ import annotations

from itertools import repeat
from pprint import pprint
from logging import DEBUG, INFO, WARNING, NullHandler, basicConfig, getLogger

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
    logger = getLogger("controller")
    logger.addHandler(NullHandler())

    step_size: float = 1.0/frequency
    logger.info(f"Step size: {step_size}")

    magnet = msg.magnet or atk.StationaryMagnet(0.0)
    logger.info(f"Magnet: {magnet}")

    speed_ctl = msg.speed or atk.FixedSpeed(5.0)
    logger.info(f"Speed: {speed_ctl}")

    vehicle = rover.ngc(world, magnet=magnet)
    controller = ha.Automaton(vehicle, step_size)
    scheduler = sched.BlockingScheduler()
    history: list[msgs.Step] = []
    cmds = iter(msg.commands)

    vehicle.wait()
    tstart = vehicle.clock

    def update():
        tsim = vehicle.clock - tstart
        logger.debug("Running controller step.")
        history.append(
            msgs.Step(
                time=tsim,
                position=vehicle.position,
                heading=vehicle.heading,
                roll=vehicle.roll,
                state=controller.state,
            )
        )

        action = controller.action
        speed = speed_ctl.speed(tsim)

        if action is ha.Action.STOP:
            vehicle.velocity = 0.0
        else:
            vehicle.velocity = speed

        if action is ha.Action.TURN:
            vehicle.steering_angle = 0.5
        else:
            vehicle.steering_angle = 0.0

        if controller.state.is_terminal():
            logger.info("Found terminal state. Shutting down scheduler.")
            scheduler.remove_all_jobs()
            scheduler.shutdown(wait=False)
        else:
            controller.step(next(cmds))

    logger.debug("Creating controller scheduler job")
    scheduler.add_job(update, "interval", seconds=step_size, id="control_loop")

    logger.debug("Starting scheduler")
    scheduler.start()

    return history


@click.command()
@click.option("-w", "--world", default="default")
@click.option("-f", "--frequency", type=int, default=1)
@click.option("-p", "--port", type=int, default=5556)
@click.option("-v", "--verbose", is_flag=True)
@click.option("-s", "--start", is_flag=True)
def controller(world: str, frequency: int, port: int | None, verbose: bool, start: bool):
    if verbose:
        basicConfig(level=DEBUG)
    else:
        basicConfig(level=INFO)
        getLogger("apscheduler").setLevel(WARNING)

    logger = getLogger("publisher")
    logger.addHandler(NullHandler())
    logger.info("Rover hybrid automaton controller version 0.1.0")

    if start:
        logger.info("No port specified, starting controller using defaults.")
        msg = msgs.Start(commands=repeat(None), magnet=None)
        history = run(world, frequency, msg)

        pprint(history)
    else:
        with zmq.Context() as ctx:
            with ctx.socket(zmq.REP) as sock:
                with sock.bind(f"tcp://*:{port}"):
                    logger.info(f"Listening for start message on port {port}")
                    msg = sock.recv_pyobj()

                    if not isinstance(msg, msgs.Start):
                        sock.send_pyobj(PublisherError(f"Unexpected start message type {type(msg)}"))

                    logger.info("Start message received. Running simulation.")
                    history = run(world, frequency, msg)
                    sock.send_pyobj(msgs.Result(history))


if __name__ == "__main__":
    controller()
