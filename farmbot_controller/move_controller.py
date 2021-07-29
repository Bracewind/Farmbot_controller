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
        self._current_position: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._current_destination: np.ndarray = self._current_position
        self._current_speed_vector: np.ndarray = np.array([0.0, 0.0, 0.0])
        self.current_state = FarmbotState.READY
        self._current_task = None
        self._moverate = 0.05
        self._lock = asyncio.Lock()

        self._position_controller = position_controller

        self._position_precision = 0.001

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

    def increment_or_stop(
        self,
        current_coordinate: np.ndarray,
        target_coordinate: np.ndarray,
        speed_wanted: np.ndarray,
    ):
        new_coordinate = current_coordinate + speed_wanted

        # determine if speed has the same direction than the vector going to the target
        sign = (target_coordinate - new_coordinate) * speed_wanted >= 0.0

        # if same sign, we are getting close to target => take new_coordinate
        # else, we are going away => means we already went past target => we stop at the target
        result = sign * new_coordinate + (1 - sign) * target_coordinate
        return sign, result

    async def _move_task(self):
        target_not_reached = np.array([True, True, True])

        while target_not_reached.any() and self.current_state == FarmbotState.MOVING:
            speed = self.current_speed_vector * self._moverate
            target_not_reached, self._current_position = self.increment_or_stop(
                self._current_position, self._current_destination, speed
            )

            self._position_controller.send_wanted_position(
                self._current_position[1],
                self._current_position[0],
                -self._current_position[2],
            )
            await asyncio.sleep(self._moverate)

        self.current_state = FarmbotState.READY

    async def wait_ready(self):
        if self._current_task is not None:
            await self._current_task
        while self.current_state != FarmbotState.READY:
            await asyncio.sleep(1)
