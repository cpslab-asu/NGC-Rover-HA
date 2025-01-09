from __future__ import annotations

import abc
import dataclasses as dc
import math
import typing

import rover

if typing.TYPE_CHECKING:
    Position: typing.TypeAlias = tuple[float, float, float]
    Command = typing.Literal[55, 66]


@dc.dataclass(frozen=True, slots=True)
class Flags:
    """State of the internal flags of the system."""

    autodrive: bool = dc.field(default=False)
    update_compass: bool = dc.field(default=False)
    update_gps: bool = dc.field(default=False)
    check_position: bool = dc.field(default=True)
    move: bool = dc.field(default=False)


@dc.dataclass(frozen=True, slots=True)
class Model:
    """Wrapper class to avoid setting rover properties unintentionally in states."""

    rover: rover.Rover

    @property
    def position(self) -> Position:
        return self.rover.position

    @property
    def heading(self) -> float:
        return self.rover.heading


@dc.dataclass(frozen=True, slots=True)
class State(abc.ABC):
    """An abstract system state representing a behavior of the system."""

    model: Model
    flags: Flags
    
    @abc.abstractmethod
    def next(self, cmd: Command | None) -> State:
        """Advance the system to the next state."""

        ...

    @property
    def velocity(self) -> float:
        """The linear velocity of the vehicle for the given state.

        This property should be overridden by state implementations in which the system is intended
        to be moving.
        """

        return 0.0

    @property
    def omega(self) -> float:
        """The angular velocity of the vehicle for the given state.

        This property should be overridden by state implementations in which the system is intended
        to be turning.
        """

        return 0.0

    def is_terminal(self) -> bool:
        return False


@dc.dataclass(frozen=True, slots=True)
class S1(State):
    time: int

    def __post_init__(self):
        assert self.flags.check_position
        assert not self.flags.autodrive
        assert not self.flags.update_compass
        assert not self.flags.update_gps
        assert not self.flags.move
    
    def next(self, cmd: Command | None) -> State:
        if self.time >= 5:
            return S2(
                model=self.model,
                flags=dc.replace(self.flags, autodrive=True),
                initial_position=self.model.position,
            )

        return S1(self.model, self.flags, time=self.time + 1)


def euclidean_distance(p1: Position, p2: Position) -> float:
    return math.sqrt(
        math.pow(p2[0] - p1[0], 2) + math.pow(p2[1] - p1[1], 2) + math.pow(p2[2] - p1[2], 2)
    )


@dc.dataclass(frozen=True, slots=True)
class S2(State):
    initial_position: tuple[float, float, float] = dc.field()

    def __post_init__(self):
        assert self.flags.check_position
        assert self.flags.autodrive
        assert not self.flags.update_compass
        assert not self.flags.update_gps
        assert not self.flags.move

    @property
    def velocity(self) -> float:
        return 1.0

    def next(self, cmd: Command | None) -> State:
        if cmd == 66:
            return S6(self.model, flags=dc.replace(self.flags, autodrive=False, check_position=False))

        position = self.model.position
        distance = euclidean_distance(position, self.initial_position)

        if distance >= 7:
            return S3(
                model=self.model,
                flags=dc.replace(self.flags, check_position=False, update_compass=True),
                initial_heading=self.model.heading,
            )
        
        return S2(self.model, self.flags, self.initial_position)


@dc.dataclass(frozen=True, slots=True)
class S3(State):
    initial_heading: float = dc.field()
    
    def __post_init__(self):
        assert self.flags.autodrive
        assert self.flags.update_compass
        assert not self.flags.check_position
        assert not self.flags.update_gps
        assert not self.flags.move

    @property
    def omega(self) -> float:
        return 1.0

    def next(self, cmd: Command | None) -> State:
        if cmd == 66:
            return S8(self.model, flags=dc.replace(self.flags, autodrive=False, check_position=False))

        heading = self.model.heading
        degrees = math.fabs(heading - self.initial_heading)

        if degrees >= 70:
            return S4(
                flags=dc.replace(self.flags, update_compass=False, update_gps=True),
                model=self.model,
            )
        
        return S3(self.model, self.flags, self.initial_heading)


@dc.dataclass(frozen=True, slots=True)
class S4(State):
    def __post_init__(self):
        assert self.flags.autodrive
        assert self.flags.update_gps
        assert not self.flags.update_compass
        assert not self.flags.check_position
        assert not self.flags.move

    @property
    def omega(self) -> float:
        return 1.0
    
    def next(self, cmd: Command | None) -> State:
        return S5(
            model=self.model,
            flags=dc.replace(self.flags, update_gps=False, move=True),
            initial_position=self.model.position,
        )


@dc.dataclass(frozen=True, slots=True)
class S5(State):
    initial_position: Position
    
    def __post_init__(self):
        assert self.flags.autodrive
        assert self.flags.move
        assert not self.flags.update_gps
        assert not self.flags.update_compass
        assert not self.flags.check_position

    @property
    def velocity(self) -> float:
        return 1.0
    
    def next(self, cmd: Command | None) -> State:
        if cmd == 66:
            return S7(self.model, flags=dc.replace(self.flags, autodrive=False, check_position=False))

        position = self.model.position
        distance = euclidean_distance(self.initial_position, position)

        if distance >= 7:
            return S6(flags=dc.replace(self.flags, autodrive=False, move=False), model=self.model)

        return S5(self.model, self.flags, self.initial_position)


@dc.dataclass(frozen=True, slots=True)
class S6(State):
    def __post_init__(self):
        assert not self.flags.autodrive
        assert not self.flags.move
        assert not self.flags.update_gps
        assert not self.flags.update_compass
        assert not self.flags.check_position

    def is_terminal(self) -> bool:
        return True

    def next(self, cmd: Command | None) -> State:
         return S6(self.model, self.flags)


@dc.dataclass(frozen=True, slots=True)
class S7(State):
    def __post_init__(self):
        assert self.flags.move
        assert not self.flags.autodrive
        assert not self.flags.update_gps
        assert not self.flags.update_compass
        assert not self.flags.check_position

    @property
    def velocity(self) -> float:
        return 1.0

    def next(self, cmd: Command | None) -> State:
        if cmd == 55:
            return S9(self.model, self.flags)
        
        return S6(self.model, flags=dc.replace(self.flags, move=False))


@dc.dataclass(frozen=True, slots=True)
class S8(State):
    def __post_init__(self):
        assert self.flags.update_compass
        assert not self.flags.autodrive
        assert not self.flags.check_position
        assert not self.flags.update_gps
        assert not self.flags.move

    @property
    def omega(self) -> float:
        return 1.0

    def next(self, cmd: Command | None) -> State:
        return S7(self.model, flags=dc.replace(self.flags, move=True, update_compass=False))


@dc.dataclass(frozen=True, slots=True)
class S9(State):
    def __post_init__(self):
        assert not self.flags.move
        assert not self.flags.update_compass
        assert not self.flags.autodrive
        assert not self.flags.check_position
        assert not self.flags.update_gps

    def is_terminal(self) -> bool:
        return True

    def next(self, cmd: Command | None) -> State:
        return S9(self.model, self.flags)


class Automaton:
    def __init__(self, vehicle: rover.Rover):
        self.vehicle = vehicle
        self.state: State = S1(flags=Flags(), model=Model(self.vehicle), time=0)
        self.history: list[State] = []

    def step(self, cmd: Command | None):
        self.history.append(self.state)
        self.state = self.state.next(cmd)
        self.vehicle.velocity = self.state.velocity
        self.vehicle.omega = self.state.omega
