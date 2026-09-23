#!/usr/bin/env python3

from typing import List, Tuple

from .base import SteeringMode
from .wheel_config import WHEEL_XY, WHEEL_ORDER, WHEEL_RADIUS

class DifferentialSteering(SteeringMode):
    name = "differential"

    def compute(self, v: float, wz: float) -> Tuple[List[float], List[float]]:
        steer_angles, wheel_angle_vels = [], []

        for wheel_name in WHEEL_ORDER:
            _, y = WHEEL_XY[wheel_name]
            speed = v - wz*y
            steer_angles.append(0.0)
            wheel_angle_vels.append(speed/WHEEL_RADIUS)

        return steer_angles, wheel_angle_vels