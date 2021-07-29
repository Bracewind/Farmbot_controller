import asyncio
from time import sleep

import logging


from .state_publisher import StatePublisher
from .move_controller import MoveController
from .move_planifier import MovePlanifier
from .gcode_parser import GCodeParser


async def main_async():
    gcode_lines = [
        "G00 X0 Y1 Z0 1 1 1",
        "G00 X1 Y1 Z0.5 1 1 1",
        "G00 X1 Y2 Z0.5 1 1 1",
        "G00 X0 Y2 Z0.5 -1 1 1",
        "G00 X0 Y1 Z0 1 -1 -1",
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


def main():
    asyncio.run(main_async())
    while True:
        sleep(5)


if __name__ == "__main__":
    main()