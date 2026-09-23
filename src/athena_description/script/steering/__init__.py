#!/usr/bin/env python3

from .base import SteeringMode
from .ackermann import AckermannSteering
from .differential import DifferentialSteering
from .independent import IndependentSteering

STEERING_MODES = {
    "ackermann": AckermannSteering,
    "differential": DifferentialSteering,
    "independent": IndependentSteering,
}

def make_steering_mode(name: str) -> SteeringMode:
    try:
        return STEERING_MODES[name]()
    except KeyError:
        valid = ", ".join(sorted(STEERING_MODES))
        raise ValueError(f"Unknown steering mode '{name}'. Valid options: {valid}")


__all__ = [
    "SteeringMode",
    "AckermannSteering",
    "DifferentialSteering",
    "IndependentSteering",
    "STEERING_MODES",
    "make_steering_mode",
]
