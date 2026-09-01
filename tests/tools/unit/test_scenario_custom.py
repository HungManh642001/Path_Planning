"""Kiểm thử đơn vị cho module scenario_custom (khởi tạo và chuyển đổi kịch bản tùy biến)."""

from __future__ import annotations

import math

from tools.qa_suite.core.scenario_custom import (
    build_custom_scenario,
    scenario_from_dict,
    scenario_from_json,
    scenario_to_dict,
    scenario_to_json,
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
