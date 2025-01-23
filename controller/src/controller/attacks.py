from __future__ import annotations

from typing import Protocol
from .automaton import Model
import numpy as np
from scipy.spatial.transform import Rotation

class Magnet(Protocol):
    def offset(self, time: float) -> float:
        ...


class StationaryMagnet(Magnet):
    def __init__(self, magnitude: float):
        self.magnitude = magnitude

    def offset(self, time: float) -> float:
        return self.magnitude
    
class EMFMagnet(Magnet):
    def __init__(self, magnitude: float, position: tuple[float,float,float]):
        self.pos = position
        self.magnitude = magnitude

    def offset(self, time: float, model: Model) -> float:
        r = model.position - self.pos
        distance = np.linalg.norm(r)
        unit_r = r/distance
        b_scalar = self.magnitude/(4.0*np.pi*np.power(distance,3.0)) 
        b_vector = unit_r*np.dot(np.array([0.0,0.0,1.0]),unit_r)*3.0 - np.array([0.0,0.0,1.0])
        b = b_scalar * b_vector
        b += model.worldField # np.array([0.224902,0.0,0.428])
        b = Rotation.from_euler(0.0,0.0,np.deg2rad(model.heading)).inv().apply(b)
        newHeading = -np.rad2deg(np.arctan2(b[1],b[0]))
        return (newHeading - model.heading)

class SpeedController:
    def speed(self, time: float) -> float:
        ...


class FixedSpeed(SpeedController):
    def __init__(self, magnitude: float):
        self.magnitude = magnitude

    def speed(self, time: float) -> float:
        return self.magnitude
