from __future__ import annotations

from pathlib import Path

import apscheduler.schedulers.blocking as sched
import click
import zmq

import automaton
import rover
import test


class PublisherError(Exception):
    pass


@click.command()
@click.option("-w", "--world", default="default")
@click.option("-f", "--frequency", type=int, default=1)
@click.option("-s", "--socket", "socket_path", type=click.Path(exists=True, dir_okay=False, writable=True, path_type=Path), default=Path("/var/run/rover/transport.sock"))
def main(world: str, frequency: int, socket_path: Path):
    with zmq.Context() as ctx:
        with ctx.socket(zmq.REP) as sock:
            with sock.connect(str(socket_path)):
                msg = sock.recv_pyobj()

                if not isinstance(msg, test.Start):
                    sock.send_pyobj(PublisherError(f"Unexpected start message type {type(msg)}"))

                cmds = iter(msg.commands)
                vehicle = rover.spawn(world, magnet=msg.magnet)
                controller = automaton.Automaton(vehicle)
                scheduler = sched.BlockingScheduler()
                history: list[test.Step] = []

                def update():
                    history.append(
                        test.Step(
                            time=vehicle.clock,
                            position=vehicle.position,
                            heading=vehicle.heading,
                            roll=vehicle.roll,
                            state=controller.state,
                        )
                    )

                    if controller.state.is_terminal():
                        sock.send_pyobj(test.Result(history))
                        scheduler.shutdown()
                    else:
                        controller.step(next(cmds))

                scheduler.add_job(update, "interval", seconds=1/frequency)
                scheduler.start()


if __name__ == "__main__":
    main()
