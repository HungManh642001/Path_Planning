"""Heuristic distance estimator for A* search."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING


if TYPE_CHECKING:
    from path_planning.search.state import State


def euclidean_heuristic(state: State, goal_state: State) -> float:
    """Estimate remaining distance from state to goal_state."""
    dx = goal_state.waypoint[0] - state.waypoint[0]
    dy = goal_state.waypoint[1] - state.waypoint[1]
    return math.sqrt(dx * dx + dy * dy)
