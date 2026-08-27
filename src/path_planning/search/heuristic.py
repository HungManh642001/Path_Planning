"""Heuristic distance estimator for A* graph search."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING


if TYPE_CHECKING:
    from path_planning.search.state import State


def euclidean_heuristic(state: State, goal_state: State) -> float:
    """Estimate remaining Euclidean distance from state to goal_state.

    Args:
        state: Current search state node.
        goal_state: Target goal state node.

    Returns:
        Admissible straight-line Euclidean distance in metres.
    """
    dx = goal_state.waypoint[0] - state.waypoint[0]
    dy = goal_state.waypoint[1] - state.waypoint[1]
    return math.sqrt(dx * dx + dy * dy)
