import numpy as np
import pytest

from farmbot_controller.gcode_parser import GCodeParser
from farmbot_controller.move_controller import MoveController, FarmbotState
from farmbot_controller.move_planifier import MovePlanifier
from farmbot_controller.state_publisher import StatePublisher


@pytest.mark.asyncio
async def test_home():
    position_controller = StatePublisher()
    move_controller = MoveController(position_controller)
    move_planifier = MovePlanifier(move_controller)
    gcode_parser = GCodeParser(move_planifier)

    gcode_lines = ["F11", "F12", "F13"]
    for gcode_line in gcode_lines:
        gcode_parser.execute(gcode_line)
        await move_controller.wait_ready()


@pytest.mark.asyncio
async def test_move():

    gcode_lines = [
        "G00 X0 Y1 Z0 A1 B1 C1",
        "G00 X1 Y1 Z0.5 A1 B1 C1",
        "G00 X1 Y2 Z0.5 A1 B1 C1",
        "G00 X0 Y2 Z0.5 A-1 B1 C1",
        "G00 X0 Y1 Z0 A1 B-1 C-1",
    ]
    position_controller = StatePublisher()
    move_controller = MoveController(position_controller)
    move_planifier = MovePlanifier(move_controller)
    gcode_parser = GCodeParser(move_planifier)
    print("hello")

    i = 0
    while True:
        gcode_line = gcode_lines[i]
        gcode_parser.execute(gcode_line)
        await move_controller.wait_ready()
        i = (i + 1) % 5
