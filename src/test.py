from __future__ import annotations

import itertools
import logging
import pathlib
import sys
import typing

import click
import gzcm
import numpy.random as rand

import staliro
import staliro.optimizers
import staliro.specifications.rtamt

from controller.messages import Start, Result
from controller.attacks import FixedSpeed, GaussianMagnet, SpeedController, Magnet
from plots import Plot, plot

PORT: typing.Final[int] = 5556
GZ_IMAGE: typing.Final[str] = "ghcr.io/cpslab-asu/ngc-rover-ha/gazebo:harmonic"
GZ_BASE: typing.Final[pathlib.Path] = pathlib.Path("resources/worlds/default.sdf")
GZ_WORLD: typing.Final[pathlib.Path] = pathlib.Path("/tmp/generated.sdf")


def firmware(*, verbose: bool):
    prefix = "controller"

    if verbose:
        prefix = f"{prefix} --verbose"

    @gzcm.manage(
        firmware_image="ghcr.io/cpslab-asu/ngc-rover-ha/controller:latest",
        gazebo_image="ghcr.io/cpslab-asu/ngc-rover-ha/gazebo:harmonic",
        command=f"{prefix} serve --port {PORT}",
        port=PORT,
        rtype=Result,
    )
    def inner(world: str, magnet: Magnet | None, speed: SpeedController | None, freq: int) -> Start:
        return Start(world, freq, magnet, speed, commands=itertools.repeat(None))

    return inner


@click.group()
@click.option("-v", "--verbose", is_flag=True)
@click.pass_context
def test(ctx: click.Context, verbose: bool):
    if verbose:
        logging.basicConfig(level=logging.INFO)

    ctx.ensure_object(dict)
    ctx.obj["verbose"] = verbose


@test.command()
@click.pass_context
def cpv1(ctx: click.Context):
    firmware_ = firmware(verbose=ctx.obj["verbose"])

    @staliro.models.model()
    def model(sample: staliro.Sample) -> staliro.Trace[list[float]]:
        gazebo = gzcm.Gazebo()
        speed = FixedSpeed(sample.static["speed"])
        result = firmware_.run(gazebo, freq=1, magnet=None, speed=speed)
        trace = {
            step.time: [
                step.position[0],
                step.position[1],
                step.position[2],
                step.heading,
                step.roll,
            ]
            for step in result
        }

        return staliro.Trace(trace)

    spec = staliro.specifications.rtamt.parse_dense("always (x >= 0)", { "x": 0, "y": 1, "z": 2, "theta": 3, "omega": 4})
    opt = staliro.optimizers.UniformRandom() # TODO: replace with SOAR
    opts = staliro.TestOptions(
        runs=1,
        iterations=10,
        static_inputs={
            "speed": (2, 50),
        },
        signals={},
    )
    runs = staliro.test(model, spec, opt, opts)
    run = runs[0]  # We know there is only a single run, so just extract it
    eval = run.evaluations[0]  # Extract the first sample generated by the optimizer

    print(eval)


@test.command()
@click.pass_context
def cpv2(ctx: click.Context):
    gazebo = gzcm.Gazebo()
    firmware_ = firmware(verbose=ctx.obj["verbose"])

    @staliro.models.model()
    def model(sample: staliro.Sample) -> staliro.Result[staliro.Trace[list[float]], int]:
        speed = FixedSpeed(sample.static["speed"])
        seed = rand.randint(0, sys.maxsize - 1)
        magnet=GaussianMagnet(sample.static["x"], sample.static["y"], rng=rand.default_rng(seed))
        result = firmware_.run(gazebo, freq=1, magnet=magnet, speed=speed)
        trace = {
            step.time: [
                step.position[0],
                step.position[1],
                step.position[2],
                step.heading,
                step.roll,
            ]
            for step in result.history
        }

        return staliro.Result(staliro.Trace(trace), seed)

    req ="always (x >= 0 and x <= 8.0 and y >= 0 and y <= 8.0)"
    spec = staliro.specifications.rtamt.parse_dense(req, {"x": 0, "y": 1})
    opt = staliro.optimizers.DualAnnealing()
    opts = staliro.TestOptions(
        runs=1,
        iterations=5,
        static_inputs={
            "x": (0, 8),
            "y": (0, 8),
        },
    )
    runs = staliro.test(model, spec, opt, opts)
    run = runs[0]  # We know there is only a single run, so just extract it
    worst = min(run.evaluations, key=lambda e: e.cost)  # Extract the first sample generated by the optimizer
    worst_plot = Plot(worst.extra.trace, (worst.sample.static["x"], worst.sample.static["y"]), color="r")
    plots = [
        Plot(e.extra.trace, (e.sample.static["x"], e.sample.static["y"]))
        for e in run.evaluations
    ]
    ground_truth = firmware_.run(gazebo, freq=5, magnet=None, speed=FixedSpeed(5.0))
    ground_truth_plot = Plot(
        magnet=None,
        trajectory=staliro.Trace({
            step.time: [step.position[0], step.position[1]] for step in ground_truth
        }),
        color="g",
    )

    plot(*plots, worst_plot, ground_truth_plot)


@test.command()
def cpv3():
    pass


@test.command()
@click.pass_context
@click.option("-f", "--frequency", "freq", type=int, default=2)
@click.option("-s", "--speed", type=float, default=5.0)
@click.option("-m", "--magnet", type=float, nargs=2, default=None)
def simulation(ctx: click.Context, speed: float, freq: int, magnet: tuple[float, float] | None):
    if magnet:
        rng = rand.default_rng()
        magnet_ = GaussianMagnet(x=magnet[0], y=magnet[1], rng=rng)
    else:
        magnet_ = None

    gazebo = gzcm.Gazebo()
    firmware_ = firmware(verbose=ctx.obj["verbose"])
    result = firmware_.run(gazebo, freq=freq, magnet=magnet_, speed=FixedSpeed(speed))
    p = Plot(
        magnet=magnet,
        trajectory=staliro.Trace({
            step.time: [step.position[0], step.position[1]] for step in result.history
        }),
    )

    plot(p)


if __name__ == "__main__":
    test()
