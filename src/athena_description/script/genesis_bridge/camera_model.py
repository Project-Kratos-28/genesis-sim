from dataclasses import dataclass, field
from typing import List
 
import numpy as np
 
from . import config
 
 
@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    K: List[float] = field(repr=False)
    P_left: List[float] = field(repr=False)
    P_right: List[float] = field(repr=False)
 
    @classmethod
    def from_fov(cls, width=config.WIDTH, height=config.HEIGHT,
                 fov_deg=config.FOV_DEG, baseline=config.BASELINE):
        f = (width / 2.0) / np.tan(np.deg2rad(fov_deg) / 2.0)
        cx, cy = width / 2.0, height / 2.0
        K = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        P_left = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        P_right = [f, 0.0, cx, -f * baseline, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return cls(width, height, f, f, cx, cy, K, P_left, P_right)
