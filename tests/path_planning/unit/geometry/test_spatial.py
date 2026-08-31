"""Kiểm thử đơn vị cho các hàm hình học không gian 2D trong module geometry.spatial."""

from __future__ import annotations

import math

from path_planning import config
from path_planning.geometry.spatial import (
    angle_diff,
    angle_to_heading,
    circle_tangent_points,
    distance,
    inflate_polygon,
    point_to_line_distance,
    state_to_tuple,
)
from path_planning.types import Point, PolygonCoords


def test_distance_with_identical_points_returns_zero() -> None:
    """Kiểm tra khoảng cách giữa hai điểm trùng nhau bằng 0."""
    # Arrange (Chuẩn bị)
    point: Point = (100.0, 200.0)

    # Act (Thực thi)
    result = distance(point, point)

    # Assert (Xác nhận)
    assert result == 0.0


def test_distance_with_known_3_4_5_triangle_returns_five() -> None:
    """Kiểm tra khoảng cách tam giác tỉ lệ 3-4-5 trả về cạnh huyền 5."""
    # Arrange
    p1: Point = (0.0, 0.0)
    p2: Point = (3000.0, 4000.0)

    # Act
    result = distance(p1, p2)

    # Assert
    assert result == 5000.0


def test_angle_to_heading_with_cardinal_directions_returns_correct_radians() -> None:
    """Kiểm tra góc phương vị đến các hướng chính Đông, Bắc, Tây, Nam."""
    # Arrange
    origin: Point = (0.0, 0.0)
    east: Point = (10.0, 0.0)
    north: Point = (0.0, 10.0)
    west: Point = (-10.0, 0.0)
    south: Point = (0.0, -10.0)

    # Act & Assert
    assert math.isclose(angle_to_heading(origin, east), 0.0, abs_tol=1e-9)
    assert math.isclose(angle_to_heading(origin, north), math.pi / 2, abs_tol=1e-9)
    assert math.isclose(abs(angle_to_heading(origin, west)), math.pi, abs_tol=1e-9)
    assert math.isclose(angle_to_heading(origin, south), -math.pi / 2, abs_tol=1e-9)


def test_angle_diff_with_wraparound_angles_normalizes_to_pi_range() -> None:
    """Kiểm tra độ lệch góc giữa hai hướng bay được chuẩn hóa về [-pi, pi]."""
    # Arrange
    heading_a = math.pi * 0.95
    heading_b = -math.pi * 0.95

    # Act
    diff = angle_diff(heading_a, heading_b)

    # Assert
    assert math.isclose(diff, -math.pi * 0.1, abs_tol=1e-9)


def test_point_to_line_distance_with_perpendicular_projection_returns_offset() -> None:
    """Kiểm tra khoảng cách từ điểm chiếu vuông góc tới đoạn thẳng."""
    # Arrange
    point: Point = (5.0, 10.0)
    line_start: Point = (0.0, 0.0)
    line_end: Point = (10.0, 0.0)

    # Act
    dist = point_to_line_distance(point, line_start, line_end)

    # Assert
    assert math.isclose(dist, 10.0, abs_tol=1e-9)


def test_point_to_line_distance_with_point_beyond_endpoint_returns_endpoint_distance() -> (
    None
):
    """Kiểm tra khoảng cách khi điểm nằm ngoài phạm vi đoạn thẳng lấy khoảng cách tới đầu mút."""
    # Arrange
    point: Point = (15.0, 0.0)
    line_start: Point = (0.0, 0.0)
    line_end: Point = (10.0, 0.0)

    # Act
    dist = point_to_line_distance(point, line_start, line_end)

    # Assert
    assert math.isclose(dist, 5.0, abs_tol=1e-9)


def test_point_to_line_distance_with_degenerate_line_returns_point_distance() -> None:
    """Kiểm tra khoảng cách khi đoạn thẳng suy biến thành 1 điểm."""
    # Arrange
    point: Point = (3.0, 4.0)
    degenerate_point: Point = (0.0, 0.0)

    # Act
    dist = point_to_line_distance(point, degenerate_point, degenerate_point)

    # Assert
    assert math.isclose(dist, 5.0, abs_tol=1e-9)


def test_inflate_polygon_with_positive_margin_expands_boundaries() -> None:
    """Kiểm tra hàm giãn nở đa giác mở rộng diện tích ra ngoài theo khoảng cách offset."""
    # Arrange
    box: PolygonCoords = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    margin = 5.0

    # Act
    inflated = inflate_polygon(box, margin)

    # Assert
    assert len(inflated) >= 4
    min_x = min(x for x, _ in inflated)
    max_x = max(x for x, _ in inflated)
    assert min_x < 0.0
    assert max_x > 10.0


def test_circle_tangent_points_with_external_point_returns_two_valid_tangents() -> None:
    """Kiểm tra tìm tiếp điểm từ điểm bên ngoài đường tròn trả về 2 tiếp điểm chính xác."""
    # Arrange
    center: Point = (0.0, 0.0)
    radius = 5.0
    external_point: Point = (10.0, 0.0)

    # Act
    tangents = circle_tangent_points(external_point, center, radius)

    # Assert
    assert len(tangents) == 2
    for t in tangents:
        # Khoảng cách từ tâm đến tiếp điểm phải bằng bán kính
        assert math.isclose(distance(t, center), radius, abs_tol=1e-7)
        # Góc tiếp tuyến với bán kính phải vuông góc (định lý Pytago: d^2 + r^2 = hyp^2)
        hyp = distance(external_point, center)
        leg1 = distance(external_point, t)
        assert math.isclose(leg1 * leg1 + radius * radius, hyp * hyp, abs_tol=1e-6)


def test_circle_tangent_points_with_interior_point_returns_empty_list() -> None:
    """Kiểm tra tìm tiếp điểm từ điểm nằm trong đường tròn trả về danh sách rỗng."""
    # Arrange
    center: Point = (0.0, 0.0)
    radius = 10.0
    interior_point: Point = (2.0, 3.0)

    # Act
    tangents = circle_tangent_points(interior_point, center, radius)

    # Assert
    assert tangents == []


def test_state_to_tuple_quantization_is_deterministic_and_matches_quantum() -> None:
    """Kiểm tra lượng tử hóa tọa độ và hướng bay thành ô lưới rời rạc LatticeKey."""
    # Arrange
    waypoint: Point = (12345.6, 67890.1)
    heading = 0.5235987755982988  # 30 deg

    # Act
    key1 = state_to_tuple(waypoint, heading)
    key2 = state_to_tuple(waypoint, heading)

    # Assert
    assert key1 == key2
    expected_x = int(waypoint[0] // config.STATE_POS_QUANTUM)
    expected_y = int(waypoint[1] // config.STATE_POS_QUANTUM)
    assert key1[0] == expected_x
    assert key1[1] == expected_y
