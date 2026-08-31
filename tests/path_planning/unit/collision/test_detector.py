"""Kiểm thử đơn vị cho module collision.detector (động cơ kiểm tra va chạm không gian)."""

from __future__ import annotations

import math

from path_planning.collision.detector import CollisionDetector
from path_planning.types import Point, PreprocessedScenario


def test_collision_detector_initialization_with_preprocessed_scenario(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra khởi tạo động cơ CollisionDetector thành công từ kịch bản mẫu."""
    # Arrange & Act
    detector = CollisionDetector(sample_preprocessed_scenario)

    # Assert
    assert len(detector.circles) == len(
        sample_preprocessed_scenario["circle_obstacles"]
    )
    assert len(detector.polygons) == len(
        sample_preprocessed_scenario["polygon_obstacles"]
    )


def test_is_collision_free_with_clear_segment_returns_true(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra đoạn thẳng nằm hoàn toàn ở vùng nước mở không va chạm trả về True."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    p1: Point = (50000.0, 50000.0)
    p2: Point = (60000.0, 60000.0)

    # Act
    is_clear = detector.is_collision_free(p1, p2)

    # Assert
    assert is_clear is True


def test_is_collision_free_with_segment_piercing_circle_returns_false(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra đoạn thẳng đâm xuyên qua chướng ngại vật tròn trả về False."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    circle_center, _ = sample_preprocessed_scenario["circle_obstacles"][0]
    p_start: Point = (circle_center[0] - 50000.0, circle_center[1])
    p_end: Point = (circle_center[0] + 50000.0, circle_center[1])

    # Act
    is_clear = detector.is_collision_free(p_start, p_end)

    # Assert
    assert is_clear is False


def test_is_corner_arc_clear_with_open_water_turn_returns_true(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra cung lượn bo góc ở vùng trống không va chạm trả về True."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    h_in = 0.0
    corner: Point = (60000.0, 50000.0)
    w_next: Point = (60000.0, 70000.0)

    # Act
    arc_clear = detector.is_corner_arc_clear(h_in, corner, w_next)

    # Assert
    assert arc_clear is True


def test_is_sector_clear_with_empty_sector_returns_true(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra hình quạt vành khuyên không chứa vật cản trả về True."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    center: Point = (50000.0, 50000.0)
    r_in = 1000.0
    r_out = 5000.0
    phi_a = 0.0
    phi_b = math.pi / 2

    # Act
    sector_clear = detector.is_sector_clear(center, r_in, r_out, phi_a, phi_b)

    # Assert
    assert sector_clear is True


def test_on_circle_boundary_with_boundary_point_returns_true(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra điểm nằm trên bán kính vật cản tròn được nhận diện chính xác."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    center, radius = sample_preprocessed_scenario["circle_obstacles"][0]
    point_on_rim: Point = (center[0] + radius, center[1])

    # Act
    is_on_boundary = detector.on_circle_boundary(point_on_rim, tol=1.0)

    # Assert
    assert is_on_boundary is True


def test_is_in_bounds_with_point_inside_map_limits_returns_true(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra điểm nằm trong phạm vi kích thước bản đồ trả về True."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    inside_point: Point = (250000.0, 250000.0)

    # Act
    in_bounds = detector.is_in_bounds(inside_point)

    # Assert
    assert in_bounds is True


def test_is_in_bounds_with_point_outside_map_limits_returns_false(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra điểm vượt ra ngoài biên bản đồ trả về False."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    outside_point: Point = (-100.0, 250000.0)

    # Act
    in_bounds = detector.is_in_bounds(outside_point)

    # Assert
    assert in_bounds is False


def test_ray_chord_clear_memoization_caches_repeated_queries(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra memoization bảng tra cứu tia quét lưu vết các khoảng cách an toàn."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    memo: dict[float, list[float]] = {}
    ray = 0.0
    dist = 5000.0
    p1: Point = (50000.0, 50000.0)
    p2: Point = (55000.0, 50000.0)

    # Act
    first_query = detector.ray_chord_clear(memo, ray, dist, p1, p2)
    second_query = detector.ray_chord_clear(memo, ray, dist, p1, p2)

    # Assert
    assert first_query is True
    assert second_query is True
    assert ray in memo
    assert dist in memo[ray]
