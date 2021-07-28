from abc import ABC
from typing import List, Optional
import threading
from unittest.mock import DEFAULT
import numpy as np
from time import sleep

from farmbot_controller.interface_move_controller import MoveControllerInterface
from .move_controller import MoveController


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


USE_CURRENT_SPEED = None


class MovePlanifier(threading.Thread, MoveControllerInterface):

    DEFAULT_SPEED = np.array([1, 1, 1])

    def __init__(self, move_controller: MoveController):
        super().__init__()
        self.lock = threading.Lock()
        self._current_position = np.array([0.0, 0.0, 0.0])
        self._max_speed = 1
        self._wanted_speed = 0
        self._current_home = np.array([0, 0, 0])
        self._current_destination = self._current_home
        self.keep_going = True
        self.move_controller = move_controller

    def move_to_point_at_given_speed(
        self, x: float, y: float, z: float, vx: float, vy: float, vz: float
    ):
        self._current_destination = np.array([float(x), float(y), float(z)])
        self._wanted_speed = np.array([float(vx), float(vy), float(vz)])
        self.move_controller.go_to_destination(
            self._current_destination, self._wanted_speed
        )

    def move_home(self, axis_to_move_home: List[bool] = [True, True, True]):
        """ exemple : move_home([True, False, False]) to move home only the x axis"""
        home_axis_order = np.array([int(axis) for axis in axis_to_move_home])
        final_position = (
            home_axis_order * self._current_home
            + (1 - home_axis_order) * self._current_position
        )

        self.move_to_point_at_given_speed(
            *(np.concatenate(final_position, MovePlanifier.DEFAULT_SPEED))
        )

    async def stop(self):
        await self.move_controller.stop()