from typing import List

from farmbot_controller.interface_move_controller import (
    MoveControllerInterface,
)


class GCodeParser:
    def __init__(self, move_controller: MoveControllerInterface):
        self.move_controller = move_controller
        self.array_exec = {
            "G00": self._move_to_point_at_given_speed,
            "G28": self._move_home,
            "F11": self._home_yaxis,
            "F12": self._home_xaxis,
            "F13": self._home_xaxis,
        }

    def execute(self, gcode_line: str):
        parse_line = gcode_line.split(" ")
        self.array_exec[parse_line[0]](parse_line[1:])

    def _move_to_point_at_given_speed(self, param: List[str]):
        self.move_controller.move_to_point_at_given_speed(
            *[float(elem[1:]) for elem in param]
        )

    def _move_home(self, param):
        self.move_controller.move_home()

    def _home_xaxis(self, param):
        self.move_controller.move_home([True, False, False])

    def _home_yaxis(self, param):
        self.move_controller.move_home([False, True, False])

    def _home_zaxis(self, param):
        self.move_controller.move_home([False, False, True])
