"""Kiểm thử đơn vị cho module geometry.arc (hình học kiểm tra biên đường tròn)."""

from __future__ import annotations

from path_planning.geometry.arc import (
    is_point_on_any_circle_boundary,
    is_point_on_circle_boundary,
)
from path_planning.types import CircleGeometry, Point


def test_is_point_on_circle_boundary_with_exact_radius_returns_true() -> None:
    """Kiểm tra điểm nằm chính xác trên bán kính đường tròn trả về True."""
    # Arrange
    center: Point = (100.0, 100.0)
    radius = 50.0
    point_on_rim: Point = (150.0, 100.0)
    tol = 1.0

    # Act
    is_on = is_point_on_circle_boundary(point_on_rim, center, radius, tol)

    # Assert
    assert is_on is True


def test_is_point_on_circle_boundary_with_outside_tolerance_returns_false() -> None:
    """Kiểm tra điểm vượt ngoài ngưỡng dung sai bán kính trả về False."""
    # Arrange
    center: Point = (100.0, 100.0)
    radius = 50.0
    point_off_rim: Point = (160.0, 100.0)
    tol = 1.0

    # Act
    is_on = is_point_on_circle_boundary(point_off_rim, center, radius, tol)

    # Assert
    assert is_on is False


def test_is_point_on_any_circle_boundary_with_matching_circle_returns_true() -> None:
    """Kiểm tra điểm thuộc biên của một trong các đường tròn trả về True."""
    # Arrange
    circles: list[CircleGeometry] = [
        ((0.0, 0.0), 10.0),
        ((100.0, 100.0), 20.0),
    ]
    query_point: Point = (120.0, 100.0)

    # Act
    is_on = is_point_on_any_circle_boundary(query_point, circles, tol=1.0)

    # Assert
    assert is_on is True
