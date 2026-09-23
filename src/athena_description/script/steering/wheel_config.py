#!usr/bin/env python3

WHEEL_XY = {
    "front_left":  ( 0.39291,  0.39181),
    "front_right": ( 0.39291, -0.39181),
    "rear_left":   (-0.34458,  0.37898),
    "rear_right":  (-0.34458, -0.37898),
}


WHEEL_ORDER = ["front_left", "front_right", "rear_left", "rear_right"]

STEER_JOINTS = ["steer_front_left", "steer_front_right", "steer_rear_left", "steer_rear_right"]

WHEEL_JOINTS = [
    "wheel_front_left_spin", "wheel_front_right_spin",
    "wheel_rear_left_spin", "wheel_rear_right_spin",
]


WHEEL_RADIUS = 0.115
MAX_STEER_ANGLE = 1.57

REAR_AXLE_X = (WHEEL_XY["rear_left"][0] + WHEEL_XY["rear_right"][0]) / 2.0
FRONT_AXLE_X = (WHEEL_XY["front_left"][0] + WHEEL_XY["front_right"][0]) / 2.0
WHEELBASE = abs(FRONT_AXLE_X - REAR_AXLE_X)
 
FRONT_TRACK = abs(WHEEL_XY["front_left"][1] - WHEEL_XY["front_right"][1])
REAR_TRACK = abs(WHEEL_XY["rear_left"][1] - WHEEL_XY["rear_right"][1])
