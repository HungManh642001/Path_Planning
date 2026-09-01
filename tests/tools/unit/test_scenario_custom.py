"""Kiểm thử đơn vị cho module scenario_custom (khởi tạo và chuyển đổi kịch bản tùy biến)."""

from __future__ import annotations

import math

from tools.qa_suite.core.scenario_custom import (
    add_circle_obstacle,
    add_polygon_obstacle,
    build_custom_scenario,
    clear_all_obstacles,
    remove_last_obstacle,
    scenario_from_dict,
    scenario_from_json,
    scenario_to_dict,
    scenario_to_json,
    update_goal_position,
    update_start_position,
)


def test_build_custom_scenario_creates_valid_structure() -> None:
    """Kiểm thử build_custom_scenario tạo cấu trúc Scenario hợp lệ."""
    scenario = build_custom_scenario(
        start=(10000.0, 20000.0),
        start_heading=math.radians(90.0),
        goal=(80000.0, 90000.0),
        goal_heading=math.radians(0.0),
        map_bounds=(100000.0, 100000.0),
        dynamic_obstacles=[((50000.0, 50000.0), 10000.0)],
        islands=[[(30000.0, 30000.0), (40000.0, 30000.0), (35000.0, 40000.0)]],
    )

    assert scenario["start"] == (10000.0, 20000.0)
    assert scenario["goal"] == (80000.0, 90000.0)
    assert len(scenario["dynamic_obstacles"]) == 1
    assert len(scenario["islands"]) == 1
    assert len(scenario["obstacles"]) == 2
    assert scenario["obstacles"][0]["type"] == "circle"
    assert scenario["obstacles"][1]["type"] == "polygon"


def test_scenario_json_round_trip() -> None:
    """Kiểm thử chuyển đổi qua lại giữa Scenario và JSON."""
    scenario = build_custom_scenario(
        start=(5000.0, 5000.0),
        start_heading=0.0,
        goal=(50000.0, 50000.0),
        goal_heading=None,
        dynamic_obstacles=[((20000.0, 20000.0), 5000.0)],
    )

    json_str = scenario_to_json(scenario)
    assert isinstance(json_str, str)
    assert "start" in json_str

    reconstructed = scenario_from_json(json_str)
    assert reconstructed["start"] == scenario["start"]
    assert reconstructed["goal"] == scenario["goal"]
    assert reconstructed["goal_heading"] is None
    assert len(reconstructed["dynamic_obstacles"]) == 1


def test_scenario_dict_conversion() -> None:
    """Kiểm thử chuyển đổi scenario sang dict và ngược lại."""
    scenario = build_custom_scenario(
        start=(100.0, 200.0),
        start_heading=1.0,
        goal=(300.0, 400.0),
        goal_heading=2.0,
        safezones=[[(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0), (0.0, 1000.0)]],
    )
    s_dict = scenario_to_dict(scenario)
    reconstructed = scenario_from_dict(s_dict)
    assert reconstructed["start"] == (100.0, 200.0)
    assert reconstructed["safezones"] is not None
    assert len(reconstructed["safezones"]) == 1


def test_scenario_mutation_helpers() -> None:
    """Kiểm thử chuỗi các hàm mutation helpers và tính bất biến (immutability)."""
    scenario = build_custom_scenario(
        start=(10000.0, 10000.0),
        start_heading=0.0,
        goal=(90000.0, 90000.0),
        goal_heading=None,
    )
    # Add circle
    s2 = add_circle_obstacle(scenario, center=(50000.0, 50000.0), radius=15000.0)
    assert len(s2["dynamic_obstacles"]) == 1
    assert len(s2["obstacles"]) == 1
    assert s2["obstacles"][0]["type"] == "circle"
    assert s2["obstacles"][0]["center"] == (50000.0, 50000.0)
    assert s2["obstacles"][0]["radius"] == 15000.0
    assert scenario["dynamic_obstacles"] == []  # Immutability check
    assert scenario["obstacles"] == []

    # Add polygon
    poly = [(20000.0, 20000.0), (30000.0, 20000.0), (25000.0, 30000.0)]
    s3 = add_polygon_obstacle(s2, vertices=poly)
    assert len(s3["islands"]) == 1
    assert len(s3["obstacles"]) == 2
    assert s3["obstacles"][1]["type"] == "polygon"
    assert s3["obstacles"][1]["polygon"] == poly
    assert s2["islands"] == []
    assert len(s2["obstacles"]) == 1

    # Remove last obstacle (removes polygon)
    s4 = remove_last_obstacle(s3)
    assert len(s4["obstacles"]) == 1
    assert len(s4["islands"]) == 0
    assert len(s4["dynamic_obstacles"]) == 1
    assert len(s3["obstacles"]) == 2  # Immutability check

    # Clear all obstacles
    s5 = clear_all_obstacles(s3)
    assert len(s5["obstacles"]) == 0
    assert len(s5["dynamic_obstacles"]) == 0
    assert len(s5["islands"]) == 0
    assert len(s3["obstacles"]) == 2  # Immutability check

    # Update Start and Goal
    s6 = update_start_position(s5, start=(12000.0, 14000.0), heading_rad=1.57)
    assert s6["start"] == (12000.0, 14000.0)
    assert s6["start_heading"] == 1.57
    assert s5["start"] == (10000.0, 10000.0)  # Immutability check

    s7 = update_goal_position(s6, goal=(85000.0, 88000.0), heading_rad=0.0)
    assert s7["goal"] == (85000.0, 88000.0)
    assert s7["goal_heading"] == 0.0

    # Test clear_heading
    s8 = update_goal_position(s7, goal=(85000.0, 88000.0), clear_heading=True)
    assert s8["goal_heading"] is None
    assert s7["goal_heading"] == 0.0  # Immutability check

    assert s6["goal"] == (90000.0, 90000.0)  # Immutability check


def test_remove_last_obstacle_edge_cases() -> None:
    """Kiểm thử xóa obstacle khi danh sách rỗng hoặc xóa circle."""
    scenario = build_custom_scenario(
        start=(1000.0, 1000.0),
        start_heading=0.0,
        goal=(5000.0, 5000.0),
    )
    # Removing from empty scenario returns empty
    s_empty = remove_last_obstacle(scenario)
    assert s_empty["obstacles"] == []
    assert s_empty["dynamic_obstacles"] == []
    assert s_empty["islands"] == []

    # Add circle then remove
    s_circle = add_circle_obstacle(scenario, center=(2000.0, 2000.0), radius=500.0)
    s_removed = remove_last_obstacle(s_circle)
    assert len(s_removed["obstacles"]) == 0
    assert len(s_removed["dynamic_obstacles"]) == 0


def test_update_positions_preserve_heading_when_none() -> None:
    """Kiểm thử update_start_position và update_goal_position giữ nguyên heading khi heading_rad là None."""
    scenario = build_custom_scenario(
        start=(1000.0, 1000.0),
        start_heading=1.23,
        goal=(5000.0, 5000.0),
        goal_heading=2.34,
    )
    s_start = update_start_position(scenario, start=(2000.0, 2000.0))
    assert s_start["start"] == (2000.0, 2000.0)
    assert s_start["start_heading"] == 1.23

    s_goal = update_goal_position(scenario, goal=(6000.0, 6000.0))
    assert s_goal["goal"] == (6000.0, 6000.0)
    assert s_goal["goal_heading"] == 2.34
