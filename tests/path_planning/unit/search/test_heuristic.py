"""Kiểm thử đơn vị cho hàm heuristic khoảng cách trong module search.heuristic."""

from __future__ import annotations

import math

from path_planning.geometry.spatial import distance
from path_planning.search.heuristic import euclidean_heuristic
from path_planning.search.state import State
from path_planning.types import Point


def test_euclidean_heuristic_at_goal_returns_zero() -> None:
    """Kiểm tra ước lượng heuristic tại chính mục tiêu đích có giá trị bằng 0."""
    # Arrange
    goal_wp: Point = (100000.0, 100000.0)
    goal_state = State(goal_wp, 0.0)
    current_state = State(goal_wp, 0.0)

    # Act
    h_cost = euclidean_heuristic(current_state, goal_state)

    # Assert
    assert h_cost == 0.0


def test_euclidean_heuristic_matches_straight_line_distance() -> None:
    """Kiểm tra giá trị heuristic bằng chính xác khoảng cách đường chim bay Euclidean."""
    # Arrange
    current_state = State((0.0, 0.0), 0.0)
    goal_state = State((3000.0, 4000.0), 0.0)

    # Act
    h_cost = euclidean_heuristic(current_state, goal_state)

    # Assert
    assert h_cost == 5000.0


def test_euclidean_heuristic_satisfies_triangle_inequality() -> None:
    """Kiểm tra heuristic thỏa mãn bất đẳng thức tam giác (tính nhất quán consistent)."""
    # Arrange
    state_u = State((0.0, 0.0), 0.0)
    state_v = State((5000.0, 5000.0), 0.0)
    state_goal = State((20000.0, 20000.0), 0.0)

    # Act
    h_u = euclidean_heuristic(state_u, state_goal)
    h_v = euclidean_heuristic(state_v, state_goal)
    cost_uv = distance(state_u.waypoint, state_v.waypoint)

    # Assert
    assert h_u <= cost_uv + h_v + 1e-9
