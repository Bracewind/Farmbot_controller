from enum import Enum
import re

from typing import Dict, List, Match

from farmbot_controller.interface_move_controller import (
    MoveControllerInterface,
)

NO_PARAMETERS = ""

X_ID = "x"
Y_ID = "y"
Z_ID = "z"

VX_ID = "a"
VY_ID = "b"
VZ_ID = "c"


class GCodeException(Exception):
    pass


def group_to_dict(list_param: List[str], match_result: Match[str]):
    return {key: match_result.group(key) for key in list_param}


class GCodeParser:
    def __init__(self, move_controller: MoveControllerInterface):
        move_parameter_pattern = fr"X(?P<{X_ID}>[^ ]*) Y(?P<{Y_ID}>[^ ]*) Z(?P<{Z_ID}>[^ ]*) A(?P<{VX_ID}>[^ ]*) B(?P<{VY_ID}>[^ ]*) C(?P<{VZ_ID}>[^ ]*)"
        self.move_controller = move_controller
        self.array_exec = {
            "G0": (move_parameter_pattern, self._move_to_point_at_given_speed),
            "G00": (move_parameter_pattern, self._move_to_point_at_given_speed),
            "G28": (NO_PARAMETERS, self._move_home),
            "F11": (NO_PARAMETERS, self._home_yaxis),
            "F12": (NO_PARAMETERS, self._home_xaxis),
            "F13": (NO_PARAMETERS, self._home_xaxis),
        }

    def execute(self, gcode_line: str):
        parse_line = gcode_line.split(" ")
        parameter_pattern, function_to_exec = self.array_exec[parse_line[0]]
        if parameter_pattern == NO_PARAMETERS:
            function_to_exec()
        else:
            parameters_found = re.search(parameter_pattern, gcode_line)
            if parameters_found == None:
                raise GCodeException(
                    f"the given gcode line was parsed as {parse_line[0]} which requires parameters, \
                    but parameters were not parsed correctly (pattern used: {parameter_pattern}). The line was {gcode_line}"
                )
            function_to_exec(parameters_found.groupdict())

    def _move_to_point_at_given_speed(self, param: Dict[str, str]):
        try:
            self.move_controller.move_to_point_at_given_speed(
                float(param[X_ID]),
                float(param[Y_ID]),
                float(param[Z_ID]),
                float(param[VX_ID]),
                float(param[VY_ID]),
                float(param[VZ_ID]),
            )
        except ValueError as e:
            raise ValueError(
                f"a parameter could not been converted to float, change separator ',' to '.' if this is the error. \
                The other parameters were : {str(param)}"
            ) from e

    def _move_home(self):
        self.move_controller.move_home()

    def _home_xaxis(self):
        self.move_controller.move_home([True, False, False])

    def _home_yaxis(self):
        self.move_controller.move_home([False, True, False])

    def _home_zaxis(self):
        self.move_controller.move_home([False, False, True])
