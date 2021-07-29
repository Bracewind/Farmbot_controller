import pytest
from unittest.mock import Mock
import asyncio
import numpy as np

from farmbot_controller.move_controller import MoveController


@pytest.mark.parametrize(
    "current_position, wanted_position, expected_result, position_not_reached",
    [
        ([0, 0, 0], [2, 3, 3], [1, 1, 1], [True, True, True]),
        ([0, 0, 0], [0, 3, 3], [0, 1, 1], [False, True, True]),
        ([0, 1.5, 0], [0, 2, 3], [0, 2, 1], [False, False, True]),
    ],
)
def test_increment_or_stop(
    current_position,
    wanted_position,
    expected_result,
    position_not_reached,
):
    m = MoveController(Mock())

    new_position_not_reached, new_position = m.increment_or_stop(
        np.array(current_position),
        np.array(wanted_position),
        np.array([1, 1, 1]),
    )

    print(new_position)
    print(new_position_not_reached)

    assert (new_position == expected_result).all()
    assert (new_position_not_reached == position_not_reached).all()


@pytest.mark.asyncio
async def test_go_to_destination():
    m = MoveController(Mock())

    m.go_to_destination(np.array([2, 1, 1]), np.array([1, 1, 1]))
    await m.wait_ready()
