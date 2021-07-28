import asyncio

from .state_publisher import StatePublisher
from .move_controller import MoveController
from .move_planifier import MovePlanifier
from .gcode_parser import GCodeParser


async def main():
    gcode_lines = [
        "G00 1 1 1 1 1 1",
        "G00 2 1 1 1 1 1",
        "G00 2 2 1 1 1 1",
        "G00 1 2 1 1 1 1",
    ]
    position_controller = StatePublisher()
    move_controller = MoveController(position_controller)
    move_planifier = MovePlanifier(move_controller)
    gcode_parser = GCodeParser(move_planifier)

    for gcode_line in gcode_lines:
        gcode_parser.execute(gcode_line)
        await move_controller.wait_ready()


if __name__ == "__main__":
    asyncio.run(main())