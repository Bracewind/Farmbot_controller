from abc import ABC
from typing import List

from farmbot_controller.interface_move_controller import (
    MoveControllerInterface,
)


class StubMoveController(MoveControllerInterface):
    def __init__(self):
        self.current_position = []
        self.home = [True, True, True]

    def move_to_point_at_given_speed(
        self, x: float, y: float, z: float, vx: float, vy: float, vz: float
    ):
        self.current_position = [x, y, z]

    def move_home(self, axis_to_move_home: List[bool] = [True, True, True]):
        self.home = axis_to_move_home