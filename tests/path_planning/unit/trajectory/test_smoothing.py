"""Kiểm thử đơn vị cho thuật toán làm mượt quy hoạch động trong module trajectory.smoothing."""

from __future__ import annotations

import math

from path_planning import config
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


def test_smooth_path_drops_straight_pass_through_waypoints() -> None:
    """Kiểm tra quy hoạch động loại bỏ các điểm đi thẳng không đổi hướng trên đường bay."""
    # Arrange: Tạo kịch bản trống với điểm rẽ 90 độ và 3 điểm thẳng hàng trung gian
    corner: Point = (20000.0, 0.0)
    goal: Point = (20000.0, 200000.0)
    pass_through: list[Point] = [
        (20000.0, 40000.0),
        (20000.0, 80000.0),
        (20000.0, 120000.0),
    ]
    empty_scenario: PreprocessedScenario = {
        "start_pos": (0.0, 0.0),
        "goal_pos": goal,
        "start_state": {"waypoint": (0.0, 0.0), "heading": 0.0},
        "goal_state": {"waypoint": goal, "heading": None},
        "start_heading": 0.0,
        "goal_heading": None,
        "map_bounds": (config.MAP_WIDTH, config.MAP_HEIGHT),
        "safezones": None,
        "circle_obstacles": [],
        "polygon_obstacles": [],
        "all_obstacles": [],
        "alpha_max_rad": math.radians(90.0),
    }
    detector = CollisionDetector(empty_scenario)
    raw_path: list[PlannerState] = (
        [(corner, math.pi / 2)]
        + [(w, math.pi / 2) for w in pass_through]
        + [(goal, math.pi / 2)]
    )

    # Act
    smoothed = smooth_path(
        raw_path,
        origin=(0.0, 0.0),
        target=goal,
        collision_detector=detector,
        start_heading=0.0,
        goal_heading=None,
        is_goal_heading_free=True,
    )

    # Assert
    kept_points = [w for w, _ in smoothed]
    for pt in pass_through:
        assert pt not in kept_points
    assert corner in kept_points
