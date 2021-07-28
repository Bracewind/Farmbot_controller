from abc import ABC
from typing import List


class MoveControllerInterface(ABC):
    def move_to_point_at_given_speed(
        self, x: float, y: float, z: float, vx: float, vy: float, vz: float
    ):
        pass

    def move_home(self, axis_to_move_home: List[bool] = [True, True, True]):
        pass