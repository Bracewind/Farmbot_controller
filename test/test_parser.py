from typing import List, Tuple
import pytest
from time import sleep

from unittest.mock import Mock

import numpy as np
import asyncio
import re

from farmbot_controller.gcode_parser import GCodeParser
from farmbot_controller.move_controller import MoveController, FarmbotState
from farmbot_controller.move_planifier import MovePlanifier
from .stub_move_controller import StubMoveController


@pytest.fixture
def setup_stub_env():
    moveController = StubMoveController()
    return [GCodeParser(moveController), moveController]


X_ID = "x"
Y_ID = "y"
Z_ID = "z"

VX_ID = "a"
VY_ID = "b"
VZ_ID = "c"


def test_re():
    move_parameter_pattern = fr"X(?P<{X_ID}>[^ ]*) Y(?P<{Y_ID}>[^ ]*) Z(?P<{Z_ID}>[^ ]*) A(?P<{VX_ID}>[^ ]*) B(?P<{VY_ID}>[^ ]*) C(?P<{VZ_ID}>[^ ;]*)"
    value = re.search(
        move_parameter_pattern,
        "G00 X234.452 Y123.123 Z154.120 A20 B23 C56 jil",
    )
    if value is not None:
        val = value.group(VX_ID)
        val2 = value.group(Y_ID)
        c = 4
    assert False, "value should not be None"


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "gcode_line, expected_result",
    [
        ("G00 X1 Y1 Z1 A1 B1 C1", [1, 1, 1]),
        ("G00 X1.2 Y1 Z1 A1 B1 C1.1", [1.2, 1, 1]),
    ],
)
def test_one_line_control(setup_stub_env, gcode_line, expected_result):
    gcode_parser = setup_stub_env[0]
    move_controller = setup_stub_env[1]
    gcode_parser.execute(gcode_line)
    assert move_controller.current_position == expected_result


@pytest.mark.asyncio
async def test_movement():
    position_controller = Mock()
    move_controller = MoveController(position_controller)

    move_controller.go_to_destination(np.array([1, 1, 1]), 1)
    await move_controller.wait_ready()
    assert (move_controller.current_position == np.array([1, 1, 1])).all()
    assert move_controller.current_state == FarmbotState.READY


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "gcode_line, expected_result, position_controller_expected_call",
    [
        ("G0 X1 Y1 Z1 A0.5 B0.5 C0.5", [1, 1, 1], 40),
        ("G00 X1.2 Y1 Z1 A1 B1 C1.1", [1.2, 1, 1], 24),
        ("G00 X2 Y1 Z1 A1 B1 C1", [2, 1, 1], 40),
        ("G00 X2 Y2 Z1 A1 B1 C1", [2, 2, 1], 40),
        ("G00 X1 Y2 Z1 A1 B1 C1", [1, 2, 1], 40),
    ],
)
async def test_scenario(gcode_line, expected_result, position_controller_expected_call):
    position_controller = Mock()
    move_controller = MoveController(position_controller)
    move_planifier = MovePlanifier(move_controller)
    gcode_parser = GCodeParser(move_planifier)
    gcode_parser.execute(gcode_line)
    await move_controller.wait_ready()
    assert (move_controller.current_position == np.array(expected_result)).all()
    assert len(position_controller.method_calls) == position_controller_expected_call
