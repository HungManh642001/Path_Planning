"""Kiểm thử đơn vị cho module geometry.arc (hình học cung tròn và bám biên)."""

from __future__ import annotations

import math

from path_planning.geometry.arc import (
    arc_angle,
    arc_waypoints,
    bitangent_departures,
    departure_point,
    has_angular_overlap,
    is_point_on_any_circle_boundary,
    is_point_on_circle_boundary,
    riding_sense,
    sector_polygon,
    tangent_heading,
)
from path_planning.geometry.spatial import distance
from path_planning.types import CircleGeometry, Point


def test_riding_sense_with_tangent_counter_clockwise_heading_returns_one() -> None:
    """Kiểm tra nhận diện chiều quay ngược chiều kim đồng hồ (+1 CCW)."""
    # Arrange
    center: Point = (0.0, 0.0)
    radius = 1000.0
    point: Point = (1000.0, 0.0)
    heading_ccw = math.pi / 2  # Hướng Bắc khi đang ở cực Đông

    # Act
    sense = riding_sense(point, heading_ccw, center, radius)

    # Assert
    assert sense == 1


def test_riding_sense_with_tangent_clockwise_heading_returns_minus_one() -> None:
    """Kiểm tra nhận diện chiều quay thuận chiều kim đồng hồ (-1 CW)."""
    # Arrange
    center: Point = (0.0, 0.0)
    radius = 1000.0
    point: Point = (1000.0, 0.0)
    heading_cw = -math.pi / 2  # Hướng Nam khi đang ở cực Đông

    # Act
    sense = riding_sense(point, heading_cw, center, radius)

    # Assert
    assert sense == -1


def test_riding_sense_with_off_boundary_point_returns_zero() -> None:
    """Kiểm tra điểm không nằm trên biên đường tròn trả về 0."""
    # Arrange
    center: Point = (0.0, 0.0)
    radius = 1000.0
    point_far: Point = (2000.0, 0.0)
    heading = math.pi / 2

    # Act
    sense = riding_sense(point_far, heading, center, radius)

    # Assert
    assert sense == 0


def test_riding_sense_with_radial_non_tangent_heading_returns_zero() -> None:
    """Kiểm tra điểm trên biên nhưng hướng bay hướng tâm (không tiếp tuyến) trả về 0."""
    # Arrange
    center: Point = (0.0, 0.0)
    radius = 1000.0
    point: Point = (1000.0, 0.0)
    heading_radial = 0.0  # Hướng thẳng ra ngoài

    # Act
    sense = riding_sense(point, heading_radial, center, radius)

    # Assert
    assert sense == 0


def test_tangent_heading_with_cardinal_points_computes_exact_tangent_vector() -> None:
    """Kiểm tra tính góc tiếp tuyến theo chiều CCW và CW tại điểm mút."""
    # Arrange
    center: Point = (0.0, 0.0)
    point_east: Point = (100.0, 0.0)

    # Act
    heading_ccw = tangent_heading(point_east, center, 1)
    heading_cw = tangent_heading(point_east, center, -1)

    # Assert
    assert math.isclose(heading_ccw, math.pi / 2, abs_tol=1e-9)
    assert math.isclose(heading_cw, -math.pi / 2, abs_tol=1e-9)


def test_arc_angle_with_quarter_circle_sweep_returns_pi_over_two() -> None:
    """Kiểm tra góc quét cung tròn góc vuông (1/4 vòng tròn) bằng pi/2."""
    # Arrange
    center: Point = (0.0, 0.0)
    start_pt: Point = (100.0, 0.0)
    end_pt: Point = (0.0, 100.0)

    # Act
    angle = arc_angle(start_pt, end_pt, center, 1)

    # Assert
    assert math.isclose(angle, math.pi / 2, abs_tol=1e-9)


def test_arc_waypoints_generates_circumscribed_polygon_points() -> None:
    """Kiểm tra sinh chuỗi waypoint dọc theo cung tròn ngoại tiếp."""
    # Arrange
    center: Point = (0.0, 0.0)
    radius = 1000.0
    start_pt: Point = (1000.0, 0.0)
    dphi = math.pi / 2  # Góc quét 90 độ
    sense = 1
    theta_max_rad = math.radians(30.0)

    # Act
    waypoints = arc_waypoints(center, radius, start_pt, dphi, sense, theta_max_rad)

    # Assert
    assert len(waypoints) >= 2
    for wp, heading in waypoints:
        # Bán kính các điểm ngoại tiếp phải lớn hơn hoặc bằng bán kính đường tròn
        assert distance(wp, center) >= radius - 1e-6
        assert -math.pi <= heading <= math.pi


def test_departure_point_with_external_target_finds_correct_tangent_contact() -> None:
    """Kiểm tra tìm điểm rời tiếp tuyến bám đường tròn hướng về mục tiêu ngoài."""
    # Arrange
    center: Point = (0.0, 0.0)
    radius = 1000.0
    target: Point = (3000.0, 0.0)

    # Act
    dep_pt = departure_point(target, center, radius, 1)

    # Assert
    assert dep_pt is not None
    assert math.isclose(distance(dep_pt, center), radius, abs_tol=1e-6)


def test_bitangent_departures_with_two_disjoint_circles_finds_tangent_pair() -> None:
    """Kiểm tra tìm cặp điểm tiếp tuyến chung bitangent giữa 2 hình tròn rời nhau."""
    # Arrange
    c1: Point = (0.0, 0.0)
    r1 = 1000.0
    c2: Point = (5000.0, 0.0)
    r2 = 1000.0

    # Act
    bitangents = bitangent_departures(c1, r1, c2, r2, 1)

    # Assert
    assert len(bitangents) >= 1
    for p1, p2 in bitangents:
        assert math.isclose(distance(p1, c1), r1, abs_tol=1e-3)
        assert math.isclose(distance(p2, c2), r2, abs_tol=1e-3)


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


def test_sector_polygon_creates_valid_annular_sector() -> None:
    """Kiểm tra tạo danh sách đỉnh đa giác hình quạt vành khuyên không suy biến."""
    # Arrange
    center: Point = (0.0, 0.0)
    r_in = 800.0
    r_out = 1200.0
    phi_a = 0.0
    phi_b = math.pi / 2

    # Act
    poly_coords = sector_polygon(center, r_in, r_out, phi_a, phi_b)

    # Assert
    assert len(poly_coords) >= 4


def test_has_angular_overlap_with_intersecting_intervals_returns_true() -> None:
    """Kiểm tra hai khoảng góc có giao nhau trả về True."""
    # Arrange
    a1, a2 = 0.0, math.pi / 2
    b1, b2 = math.pi / 4, math.pi * 0.75

    # Act
    has_overlap = has_angular_overlap(a1, a2, b1, b2)

    # Assert
    assert has_overlap is True


def test_has_angular_overlap_with_disjoint_intervals_returns_false() -> None:
    """Kiểm tra hai khoảng góc không giao nhau trả về False."""
    # Arrange
    a1, a2 = 0.0, math.pi / 4
    b1, b2 = math.pi / 2, math.pi

    # Act
    has_overlap = has_angular_overlap(a1, a2, b1, b2)

    # Assert
    assert has_overlap is False
