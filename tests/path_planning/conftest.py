"""Fixtures dùng chung cho toàn bộ các bài kiểm thử thuật toán Path Planning."""

from __future__ import annotations

import pytest

from path_planning import config
from path_planning.scenario.preprocessing import prepare_scenario
from path_planning.types import (
    CircleGeometry,
    Point,
    PolygonCoords,
    PreprocessedScenario,
    Scenario,
)


@pytest.fixture
def sample_origin_and_target() -> tuple[Point, Point]:
    """Cung cấp cặp tọa độ điểm cất cánh (origin) và đích (target) tiêu chuẩn.

    Returns:
        Tuple chứa tọa độ điểm xuất phát O và điểm đích T tính bằng mét.
    """
    origin: Point = (50000.0, 50000.0)
    target: Point = (450000.0, 450000.0)
    return origin, target


@pytest.fixture
def sample_circle_obstacles() -> list[CircleGeometry]:
    """Cung cấp danh sách các chướng ngại vật hình tròn mẫu.

    Returns:
        Danh sách các bộ ((cx, cy), r) tính bằng mét.
    """
    return [
        ((150000.0, 150000.0), 20000.0),
        ((300000.0, 300000.0), 25000.0),
    ]


@pytest.fixture
def sample_polygon_obstacles() -> list[PolygonCoords]:
    """Cung cấp danh sách các đảo đa giác chướng ngại vật mẫu.

    Returns:
        Danh sách danh sách đỉnh đa giác [(x, y), ...].
    """
    return [
        [
            (200000.0, 100000.0),
            (230000.0, 100000.0),
            (230000.0, 130000.0),
            (200000.0, 130000.0),
        ]
    ]


@pytest.fixture
def sample_clean_scenario(
    sample_origin_and_target: tuple[Point, Point],
    sample_circle_obstacles: list[CircleGeometry],
    sample_polygon_obstacles: list[PolygonCoords],
) -> Scenario:
    """Cung cấp một kịch bản nhiệm vụ hoàn chỉnh chuẩn chưa tiền xử lý.

    Args:
        sample_origin_and_target: Tọa độ xuất phát và đích.
        sample_circle_obstacles: Danh sách vật cản tròn.
        sample_polygon_obstacles: Danh sách đảo đa giác.

    Returns:
        Dictionary đối tượng Scenario hợp lệ.
    """
    start, goal = sample_origin_and_target
    return {
        "start": start,
        "start_heading": 0.7853981633974483,  # 45 deg
        "goal": goal,
        "goal_heading": 0.7853981633974483,
        "map_bounds": (config.MAP_WIDTH, config.MAP_HEIGHT),
        "safezones": None,
        "islands": sample_polygon_obstacles,
        "dynamic_obstacles": sample_circle_obstacles,
        "obstacles": [
            {"type": "circle", "center": c[0], "radius": c[1]}
            for c in sample_circle_obstacles
        ]
        + [{"type": "polygon", "polygon": p} for p in sample_polygon_obstacles],
    }


@pytest.fixture
def sample_preprocessed_scenario(sample_clean_scenario: Scenario) -> PreprocessedScenario:
    """Cung cấp kịch bản đã tiền xử lý hoàn chỉnh sẵn sàng cho thuật toán tìm kiếm.

    Args:
        sample_clean_scenario: Kịch bản gốc.

    Returns:
        Dictionary PreprocessedScenario đã giãn nở và offset điểm mút.
    """
    return prepare_scenario(sample_clean_scenario)
