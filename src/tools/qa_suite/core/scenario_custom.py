# pyright: reportMissingTypeArgument=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownParameterType=false
"""Module hỗ trợ khởi tạo và chuyển đổi kịch bản tùy biến (Custom Scenario).

Cung cấp các hàm tạo Scenario dict từ tọa độ người dùng nhập, chuyển đổi hai chiều
với định dạng JSON, và nạp kịch bản từ file cấu hình ngoài.
"""

from __future__ import annotations

import json
from typing import cast

from path_planning import config
from path_planning.types import (
    CircleGeometry,
    Obstacle,
    Point,
    PolygonCoords,
    Scenario,
)


def build_custom_scenario(
    start: tuple[float, float],
    start_heading: float,
    goal: tuple[float, float],
    goal_heading: float | None = None,
    map_bounds: tuple[float, float] = (config.MAP_WIDTH, config.MAP_HEIGHT),
    dynamic_obstacles: list[tuple[tuple[float, float], float]] | None = None,
    islands: list[list[tuple[float, float]]] | None = None,
    safezones: list[list[tuple[float, float]]] | None = None,
) -> Scenario:
    """Khởi tạo một Scenario dict hợp lệ từ các trường dữ liệu tùy chỉnh.

    Args:
        start: Tọa độ điểm cất cánh (x, y) (m).
        start_heading: Hướng cất cánh ban đầu (rad).
        goal: Tọa độ điểm mục tiêu đích (x, y) (m).
        goal_heading: Hướng tiếp cận đích (rad), hoặc None nếu tự do (free heading).
        map_bounds: Kích thước khung bản đồ (width, height) (m).
        dynamic_obstacles: Danh sách vật cản tròn [((cx, cy), r), ...].
        islands: Danh sách đảo đa giác [[(x1, y1), (x2, y2), ...], ...].
        safezones: Danh sách vùng an toàn cho phép bay, hoặc None nếu toàn map.

    Returns:
        Scenario TypedDict đầy đủ các trường chuẩn.
    """
    circles: list[CircleGeometry] = dynamic_obstacles or []
    polygons: list[PolygonCoords] = islands or []

    obstacles: list[Obstacle] = []
    for center, radius in circles:
        obstacles.append(
            {
                "type": "circle",
                "center": (float(center[0]), float(center[1])),
                "radius": float(radius),
            }
        )
    for poly in polygons:
        obstacles.append(
            {
                "type": "polygon",
                "polygon": [(float(p[0]), float(p[1])) for p in poly],
            }
        )

    return {
        "start": (float(start[0]), float(start[1])),
        "start_heading": float(start_heading),
        "goal": (float(goal[0]), float(goal[1])),
        "goal_heading": float(goal_heading) if goal_heading is not None else None,
        "map_bounds": (float(map_bounds[0]), float(map_bounds[1])),
        "safezones": (
            [[(float(p[0]), float(p[1])) for p in zone] for zone in safezones]
            if safezones
            else None
        ),
        "islands": [[(float(p[0]), float(p[1])) for p in poly] for poly in polygons],
        "dynamic_obstacles": [
            ((float(c[0]), float(c[1])), float(r)) for c, r in circles
        ],
        "obstacles": obstacles,
    }


def scenario_to_dict(scenario: Scenario) -> dict[str, object]:
    """Chuyển đổi Scenario dict sang dạng JSON-serializable dictionary."""
    safezones = scenario.get("safezones")
    safezones_data = (
        [[list(p) for p in zone] for zone in safezones]
        if safezones is not None
        else None
    )
    return {
        "start": [scenario["start"][0], scenario["start"][1]],
        "start_heading": scenario["start_heading"],
        "goal": [scenario["goal"][0], scenario["goal"][1]],
        "goal_heading": scenario["goal_heading"],
        "map_bounds": [scenario["map_bounds"][0], scenario["map_bounds"][1]],
        "safezones": safezones_data,
        "islands": [[list(p) for p in poly] for poly in scenario.get("islands", [])],
        "dynamic_obstacles": [
            [[c[0], c[1]], r] for c, r in scenario.get("dynamic_obstacles", [])
        ],
    }


def scenario_from_dict(data: dict[str, object]) -> Scenario:
    """Tái tạo Scenario dict từ một JSON-serializable dictionary.

    Raises:
        ValueError: Nếu dữ liệu thiếu các trường bắt buộc (start, goal, start_heading).
    """
    if "start" not in data or "goal" not in data or "start_heading" not in data:
        raise ValueError(
            "Scenario data must contain 'start', 'goal', and 'start_heading'"
        )

    start_raw = cast(list[float], data["start"])
    goal_raw = cast(list[float], data["goal"])
    start_pt: Point = (float(start_raw[0]), float(start_raw[1]))
    goal_pt: Point = (float(goal_raw[0]), float(goal_raw[1]))
    start_h = float(cast(float, data["start_heading"]))
    goal_h_val = data.get("goal_heading")
    goal_h = float(cast(float, goal_h_val)) if goal_h_val is not None else None

    map_bounds_raw = cast(list[float] | None, data.get("map_bounds"))
    if map_bounds_raw:
        map_bounds: tuple[float, float] = (
            float(map_bounds_raw[0]),
            float(map_bounds_raw[1]),
        )
    else:
        map_bounds = (config.MAP_WIDTH, config.MAP_HEIGHT)

    circles: list[CircleGeometry] = []
    if "dynamic_obstacles" in data and isinstance(data["dynamic_obstacles"], list):
        for item in data["dynamic_obstacles"]:
            if isinstance(item, list) and len(item) == 2:
                c_pt = (float(item[0][0]), float(item[0][1]))
                r = float(item[1])
                circles.append((c_pt, r))

    islands: list[PolygonCoords] = []
    if "islands" in data and isinstance(data["islands"], list):
        for poly in data["islands"]:
            if isinstance(poly, list) and len(poly) >= 3:
                islands.append([(float(p[0]), float(p[1])) for p in poly])

    safezones: list[PolygonCoords] | None = None
    if "safezones" in data and isinstance(data["safezones"], list):
        safezones = [
            [(float(p[0]), float(p[1])) for p in zone]
            for zone in data["safezones"]
            if isinstance(zone, list) and len(zone) >= 3
        ]

    return build_custom_scenario(
        start=start_pt,
        start_heading=start_h,
        goal=goal_pt,
        goal_heading=goal_h,
        map_bounds=map_bounds,
        dynamic_obstacles=circles,
        islands=islands,
        safezones=safezones,
    )


def scenario_to_json(scenario: Scenario, indent: int = 2) -> str:
    """Xuất Scenario ra chuỗi định dạng JSON."""
    return json.dumps(scenario_to_dict(scenario), indent=indent)


def scenario_from_json(json_str: str) -> Scenario:
    """Nạp Scenario từ chuỗi JSON."""
    data = cast(dict[str, object], json.loads(json_str))
    return scenario_from_dict(data)
