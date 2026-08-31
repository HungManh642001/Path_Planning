"""Kiểm thử đơn vị cho bộ kiểm định Oracle độc lập trong module validation.oracle."""

from __future__ import annotations

import math

from path_planning import config
from path_planning.types import CircleGeometry, PlannerState, PolygonCoords
from path_planning.validation.oracle import (
    ValidationResult,
    arcs_clear,
    path_is_valid,
    segments_clear,
    straight_segments_ok,
    turn_angles_ok,
)


def test_validation_result_ok_factory_method() -> None:
    """Kiểm tra phương thức factory ValidationResult.ok() trả về trạng thái hợp lệ."""
    # Arrange & Act
    res = ValidationResult.ok()

    # Assert
    assert res.is_ok is True
    assert res.detail == "ok"


def test_path_is_valid_on_clear_straight_path_returns_ok() -> None:
    """Kiểm tra đường bay thẳng hoàn toàn ở kịch bản rỗng vượt qua toàn bộ 4 cổng kiểm định."""
    # Arrange
    circles: list[CircleGeometry] = []
    polygons: list[PolygonCoords] = []
    path: list[PlannerState] = [
        ((50000.0, 50000.0), 0.0),
        ((150000.0, 50000.0), 0.0),
        ((450000.0, 50000.0), 0.0),
    ]

    # Act
    res = path_is_valid(
        path,
        circle_obstacles=circles,
        polygon_obstacles=polygons,
        alpha_max_rad=config.ALPHA_MAX_RAD,
        l0=config.L0,
        dss=config.DSS,
        turn_radius=config.R,
    )

    # Assert
    assert res.is_ok is True
    assert res.detail == "ok"


def test_segments_clear_detects_obstacle_collision() -> None:
    """Kiểm tra cổng segments_clear phát hiện va chạm với chướng ngại vật tròn."""
    # Arrange
    circles: list[CircleGeometry] = [((150000.0, 50000.0), 20000.0)]
    polygons: list[PolygonCoords] = []
    colliding_path: list[PlannerState] = [
        ((50000.0, 50000.0), 0.0),
        ((250000.0, 50000.0), 0.0),
    ]

    # Act
    res = segments_clear(
        colliding_path,
        circle_obstacles=circles,
        polygon_obstacles=polygons,
    )

    # Assert
    assert res.is_ok is False
    assert "blocked" in res.detail


def test_turn_angles_ok_detects_excessive_corner_turn() -> None:
    """Kiểm tra cổng turn_angles_ok từ chối góc ngoặt vượt quá alpha_max."""
    # Arrange: Quay 135 độ (> alpha_max 90 độ)
    sharp_turn_path: list[PlannerState] = [
        ((50000.0, 50000.0), 0.0),
        ((100000.0, 50000.0), 0.0),
        ((70000.0, 10000.0), -2.35),
    ]

    # Act
    res = turn_angles_ok(sharp_turn_path, alpha_max_rad=math.radians(90.0))

    # Assert
    assert res.is_ok is False
    assert "turn angle" in res.detail


def test_straight_segments_ok_detects_insufficient_takeoff_length() -> None:
    """Kiểm tra cổng straight_segments_ok từ chối đoạn cất cánh ngắn hơn L0."""
    # Arrange: Đoạn đầu chỉ dài 1000m (< L0 4000m)
    short_takeoff_path: list[PlannerState] = [
        ((50000.0, 50000.0), 0.0),
        ((51000.0, 50000.0), 0.0),
        ((60000.0, 60000.0), 0.785),
    ]

    # Act
    res = straight_segments_ok(
        short_takeoff_path,
        l0=4000.0,
        dss=12000.0,
        turn_radius=8000.0,
    )

    # Assert
    assert res.is_ok is False
    assert "< L0=" in res.detail


def test_arcs_clear_detects_fillet_arc_obstacle_collision() -> None:
    """Kiểm tra cổng arcs_clear phát hiện cung lượn bo góc cắt qua vật cản."""
    # Arrange: Vật cản nằm ngay tại góc cua bên trong của cung rẽ
    circles: list[CircleGeometry] = [((95000.0, 55000.0), 5000.0)]
    polygons: list[PolygonCoords] = []
    corner_path: list[PlannerState] = [
        ((50000.0, 50000.0), 0.0),
        ((100000.0, 50000.0), 0.0),
        ((100000.0, 100000.0), math.pi / 2),
    ]

    # Act
    res = arcs_clear(
        corner_path,
        turn_radius=8000.0,
        circle_obstacles=circles,
        polygon_obstacles=polygons,
    )

    # Assert
    assert res.is_ok is False
    assert "blocked" in res.detail
