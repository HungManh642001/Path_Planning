"""Kiểm thử đơn vị cho module lấy mẫu quỹ đạo hiển thị render.sampling."""

from __future__ import annotations

import math

from path_planning.render.sampling import (
    build_full_path,
    sample_trajectory,
    turn_markers,
)
from path_planning.types import PlannerState, PreprocessedScenario


def test_build_full_path_joins_origin_waypoints_and_target(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra nối điểm cất cánh O và đích T vào chuỗi waypoint nội bộ."""
    # Arrange
    interior: list[PlannerState] = [
        ((100000.0, 100000.0), 0.785),
        ((350000.0, 350000.0), 0.785),
    ]

    # Act
    full = build_full_path(interior, sample_preprocessed_scenario)

    # Assert
    assert len(full) == 4
    assert full[0][0] == sample_preprocessed_scenario["start_pos"]
    assert full[-1][0] == sample_preprocessed_scenario["goal_pos"]


def test_sample_trajectory_straight_mode_returns_discrete_polyline() -> None:
    """Kiểm tra lấy mẫu đường bay chế độ 'straight' tạo polyline các đoạn thẳng liên tiếp."""
    # Arrange
    path: list[PlannerState] = [
        ((0.0, 0.0), 0.0),
        ((10000.0, 0.0), 0.0),
        ((10000.0, 10000.0), math.pi / 2),
    ]
    turn_radius = 8000.0

    # Act
    samples = sample_trajectory(path, turn_radius=turn_radius, mode="straight")

    # Assert
    assert len(samples) >= 3
    assert samples[0] == (0.0, 0.0)
    assert samples[-1] == (10000.0, 10000.0)


def test_sample_trajectory_dubins_mode_inserts_curved_fillet_points() -> None:
    """Kiểm tra lấy mẫu chế độ 'dubins' tạo thêm các điểm cung lượn bo tròn tại góc ngoặt."""
    # Arrange
    path: list[PlannerState] = [
        ((0.0, 0.0), 0.0),
        ((20000.0, 0.0), 0.0),
        ((20000.0, 20000.0), math.pi / 2),
    ]
    turn_radius = 8000.0

    # Act
    samples_straight = sample_trajectory(path, turn_radius=turn_radius, mode="straight")
    samples_dubins = sample_trajectory(path, turn_radius=turn_radius, mode="dubins")

    # Assert
    assert len(samples_dubins) > len(samples_straight)


def test_turn_markers_extracts_arc_bisector_points() -> None:
    """Kiểm tra trích xuất điểm phân giác tâm cung rẽ để hiển thị marker."""
    # Arrange
    path: list[PlannerState] = [
        ((0.0, 0.0), 0.0),
        ((20000.0, 0.0), 0.0),
        ((20000.0, 20000.0), math.pi / 2),
    ]
    turn_radius = 8000.0

    # Act
    markers = turn_markers(path, turn_radius=turn_radius)

    # Assert
    assert len(markers) == 1
    assert markers[0]["angle_deg"] > 0.0
