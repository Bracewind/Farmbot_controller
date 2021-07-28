from enum import Enum
import numpy as np
from time import sleep
import asyncio


class FarmbotState(Enum):

    READY = 0
    MOVING = 100
    STOPPING = 200
    ENCOUNTERED_ERROR = 300


class MoveController:
    def __init__(self, position_controller) -> None:
        self._current_position: np.ndarray = np.array([0, 0, 0])
        self._current_destination: np.ndarray = self._current_position
        self._current_speed_vector: np.ndarray = np.array([0, 0, 0])
        self.current_state = FarmbotState.READY
        self._current_task = None
        self._moverate = 0.05
        self._lock = asyncio.Lock()

        self._position_controller = position_controller

    @property
    def current_position(self):
        return self._current_position

    @property
    def current_speed_vector(self):
        return self._current_speed_vector

    def go_to_destination(self, destination: np.ndarray, speed: np.ndarray):
        self.current_state = FarmbotState.MOVING
        self._current_speed_vector = speed
        self._current_destination = destination
        self._current_task = asyncio.create_task(self._move_task())

    async def stop(self):
        self.current_state = FarmbotState.STOPPING
        if self._current_task is not None:
            await self._current_task
            self._current_task = None
        self.current_state = FarmbotState.READY

    async def _move_task(self):
        while (
            np.linalg.norm(self._current_destination - self._current_position)
            < np.linalg.norm(self._current_speed_vector)
        ) and self.current_state == FarmbotState.MOVING:
            await asyncio.sleep(self._moverate)
            print("Hello")
            self._current_position += self.current_speed_vector * self._moverate
            self._position_controller.send_wanted_position(
                self._current_position.y,
                self._current_position.x,
                -self._current_position.z,
            )

        # TODO: recalculate speed vector needed ?
        if self.current_state == FarmbotState.MOVING:
            self._current_position = self._current_destination
            self._current_speed_vector = np.array([0.0, 0.0, 0.0])
            self.current_state = FarmbotState.READY

    async def wait_ready(self):
        if self._current_task is not None:
            await self._current_task
        while self.current_state != FarmbotState.READY:
            await asyncio.sleep(1)
