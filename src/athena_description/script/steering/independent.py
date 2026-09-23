#!/usr/bin/env python3

import math
from typing import List, Tuple

from .base import SteeringMode
from .wheel_config import WHEEL_XY, WHEEL_ORDER, WHEEL_RADIUS, MAX_STEER_ANGLE

class IndependentSteering(SteeringMode):
    name = "independent"

    def compute(self, v: float, wz: float) -> Tuple[List[float], List[float]]:
        steer_angles, wheel_ang_vels = [], []
        for wheel_name in WHEEL_ORDER:
            x, y = WHEEL_XY[wheel_name]
            vx = v - wz * y
            vy = wz * x
            angle = math.atan2(vy, vx)
            speed = math.hypot(vx, vy)
            if abs(angle) > math.pi / 2:
                angle = math.atan2(-vy, -vx)
                speed = -speed
            angle = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, angle))
            steer_angles.append(angle)
            wheel_ang_vels.append(speed / WHEEL_RADIUS)
        return steer_angles, wheel_ang_vels
