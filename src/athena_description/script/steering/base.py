#!/usr/vin/env python3

from abc import ABC, abstractmethod
from typing import List, Tuple

class SteeringMode(ABC):
    name: str = "base"

    @abstractmethod
    def compute(self, v: float, wz: float) -> Tuple[List[float], List[float]]:

        
        raise NotImplemented