"""Kiểm thử đơn vị cho thuật toán làm mượt quy hoạch động trong module trajectory.smoothing."""

from __future__ import annotations

import math

from path_planning.collision.detector import CollisionDetector
from path_planning.trajectory.smoothing import smooth_path
from path_planning.types import PlannerState, Point, PreprocessedScenario


def test_smooth_path_with_less_than_three_waypoints_returns_unchanged_path(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra đường bay có dưới 3 điểm không cần làm mượt và trả về nguyên bản."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    short_path: list[PlannerState] = [
        ((50000.0, 50000.0), 0.0),
        ((100000.0, 50000.0), 0.0),
    ]

    # Act
    smoothed = smooth_path(
        short_path,
        origin=sample_preprocessed_scenario["start_pos"],
        target=sample_preprocessed_scenario["goal_pos"],
        collision_detector=detector,
    )

    # Assert
    assert smoothed == short_path


def test_smooth_path_shortcuts_collinear_intermediate_waypoints() -> None:
    """Kiểm tra quy hoạch động loại bỏ các điểm thẳng hàng không cần thiết trên đường bay."""
    # Arrange: Scenario rỗng hoàn toàn không có vật cản trên đường bay
    empty_scenario: PreprocessedScenario = {
        "start_pos": (50000.0, 50000.0),
        "goal_pos": (450000.0, 450000.0),
        "start_state": {"waypoint": (50000.0, 50000.0), "heading": math.pi / 4},
        "goal_state": {"waypoint": (450000.0, 450000.0), "heading": math.pi / 4},
        "start_heading": math.pi / 4,
        "goal_heading": math.pi / 4,
        "map_bounds": (500000.0, 500000.0),
        "safezones": None,
        "circle_obstacles": [],
        "polygon_obstacles": [],
        "all_obstacles": [],
        "alpha_max_rad": math.pi / 2,
    }
    detector = CollisionDetector(empty_scenario)
    origin: Point = (50000.0, 50000.0)
    target: Point = (450000.0, 450000.0)
    # 4 điểm nằm thẳng hàng từ (60000, 60000) đến (400000, 400000)
    collinear_path: list[PlannerState] = [
        ((60000.0, 60000.0), math.pi / 4),
        ((120000.0, 120000.0), math.pi / 4),
        ((250000.0, 250000.0), math.pi / 4),
        ((400000.0, 400000.0), math.pi / 4),
    ]

    # Act
    smoothed = smooth_path(
        collinear_path,
        origin=origin,
        target=target,
        collision_detector=detector,
        start_heading=math.pi / 4,
        goal_heading=math.pi / 4,
    )

    # Assert
    assert len(smoothed) < len(collinear_path)
    assert smoothed[0][0] == collinear_path[0][0]
    assert smoothed[-1][0] == collinear_path[-1][0]
