"""Kiểm thử tích hợp cho toàn bộ luồng quy hoạch đường bay plan_trajectory."""

from __future__ import annotations

import math

from path_planning import config
from path_planning.planner import KinodynamicAstar, plan_trajectory
from path_planning.scenario.preprocessing import prepare_scenario
from path_planning.types import Scenario
from path_planning.validation.oracle import path_is_valid


def test_plan_trajectory_with_fixed_goal_heading_produces_oracle_valid_path() -> None:
    """Kiểm tra lập kế hoạch đường bay end-to-end với góc hướng tiếp cận đích cố định."""
    # Arrange
    scenario: Scenario = {
        "start": (50000.0, 50000.0),
        "start_heading": math.pi / 4,
        "goal": (450000.0, 450000.0),
        "goal_heading": math.pi / 4,
        "map_bounds": (config.MAP_WIDTH, config.MAP_HEIGHT),
        "safezones": None,
        "islands": [],
        "dynamic_obstacles": [
            ((250000.0, 250000.0), 30000.0),
        ],
        "obstacles": [
            {"type": "circle", "center": (250000.0, 250000.0), "radius": 30000.0}
        ],
    }
    prep = prepare_scenario(scenario)

    # Act
    result = plan_trajectory(prep, time_budget_s=15.0)

    # Assert
    assert result["is_success"] is True
    assert result["path"] is not None
    assert len(result["path"]) >= 2
    validation = path_is_valid(
        result["path"],
        circle_obstacles=[((250000.0, 250000.0), 30000.0)],
        polygon_obstacles=[],
    )
    assert validation.is_ok is True


def test_plan_trajectory_with_free_goal_heading_produces_oracle_valid_path() -> None:
    """Kiểm tra lập kế hoạch đường bay end-to-end ở chế độ không ràng buộc hướng đích (free-goal)."""
    # Arrange
    scenario: Scenario = {
        "start": (50000.0, 50000.0),
        "start_heading": 0.0,
        "goal": (400000.0, 300000.0),
        "goal_heading": None,
        "map_bounds": (config.MAP_WIDTH, config.MAP_HEIGHT),
        "safezones": None,
        "islands": [],
        "dynamic_obstacles": [
            ((200000.0, 150000.0), 25000.0),
        ],
        "obstacles": [
            {"type": "circle", "center": (200000.0, 150000.0), "radius": 25000.0}
        ],
    }
    prep = prepare_scenario(scenario)

    # Act
    result = plan_trajectory(prep, time_budget_s=15.0)

    # Assert
    assert result["is_success"] is True
    assert result["path"] is not None
    validation = path_is_valid(
        result["path"],
        circle_obstacles=[((200000.0, 150000.0), 25000.0)],
        polygon_obstacles=[],
    )
    assert validation.is_ok is True


def test_plan_trajectory_with_blocked_takeoff_detects_failure_cleanly() -> None:
    """Kiểm tra phát hiện lỗi khi tia cất cánh xuất phát bị vật cản chặn đứng."""
    # Arrange: Đặt vật cản ngay trên tia cất cánh cách start 2000m (< L0 4000m)
    scenario: Scenario = {
        "start": (50000.0, 50000.0),
        "start_heading": 0.0,
        "goal": (450000.0, 450000.0),
        "goal_heading": 0.0,
        "map_bounds": (config.MAP_WIDTH, config.MAP_HEIGHT),
        "safezones": None,
        "islands": [],
        "dynamic_obstacles": [
            ((52000.0, 50000.0), 5000.0),
        ],
        "obstacles": [
            {"type": "circle", "center": (52000.0, 50000.0), "radius": 5000.0}
        ],
    }
    prep = prepare_scenario(scenario)

    # Act
    result = plan_trajectory(prep, time_budget_s=5.0)

    # Assert
    assert result["is_success"] is False
    assert result["path"] is None


def test_kinodynamic_astar_class_facade_matches_module_function() -> None:
    """Kiểm tra giao diện hướng đối tượng KinodynamicAstar hoạt động nhất quán."""
    # Arrange
    scenario: Scenario = {
        "start": (50000.0, 50000.0),
        "start_heading": 0.0,
        "goal": (350000.0, 50000.0),
        "goal_heading": 0.0,
        "map_bounds": (config.MAP_WIDTH, config.MAP_HEIGHT),
        "safezones": None,
        "islands": [],
        "dynamic_obstacles": [],
        "obstacles": [],
    }
    prep = prepare_scenario(scenario)

    # Act
    planner = KinodynamicAstar(prep)
    path, stats = planner.plan()

    # Assert
    assert path is not None
    assert len(path) >= 2
    assert stats["is_search_failed"] is False
