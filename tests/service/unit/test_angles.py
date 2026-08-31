"""Kiểm thử đơn vị cho module chuyển đổi góc hướng service.vtx_service.angles."""

from __future__ import annotations

import math

import pytest

from service.vtx_service.angles import bearing_deg_to_math_rad, math_rad_to_bearing_deg

KNOWN_DIRECTIONS = [
    (0.0, math.pi / 2),  # Bắc (+y)
    (90.0, 0.0),  # Đông (+x)
    (180.0, -math.pi / 2),  # Nam (-y)
    (270.0, math.pi),  # Tây (-x)
    (45.0, math.pi / 4),  # Đông Bắc
]


@pytest.mark.parametrize("bearing_deg,expected_rad", KNOWN_DIRECTIONS)
def test_bearing_deg_to_math_rad_converts_cardinal_directions(
    bearing_deg: float,
    expected_rad: float,
) -> None:
    """Kiểm tra chuyển đổi phương vị 4 hướng chính và hướng chéo sang radian toán học."""
    # Arrange & Act
    got = bearing_deg_to_math_rad(bearing_deg)

    # Assert
    assert math.isclose(math.cos(got), math.cos(expected_rad), abs_tol=1e-12)
    assert math.isclose(math.sin(got), math.sin(expected_rad), abs_tol=1e-12)


@pytest.mark.parametrize("bearing_deg", [0.0, 12.5, 90.0, 179.9, 180.0, 270.0, 359.99])
def test_angle_conversions_round_trip_is_stable(bearing_deg: float) -> None:
    """Kiểm tra tính ổn định hai chiều: bearing -> math_rad -> bearing."""
    # Arrange & Act
    back = math_rad_to_bearing_deg(bearing_deg_to_math_rad(bearing_deg))

    # Assert
    assert math.isclose(back, bearing_deg, abs_tol=1e-9)


def test_math_rad_to_bearing_deg_normalizes_to_single_turn() -> None:
    """Kiểm tra góc phương vị kết quả luôn được chuẩn hóa trong khoảng [0, 360)."""
    # Arrange & Act
    bearing = math_rad_to_bearing_deg(bearing_deg_to_math_rad(730.0))

    # Assert
    assert 0.0 <= bearing < 360.0


def test_bearing_increases_clockwise_and_decreases_math_angle() -> None:
    """Kiểm tra tính chất: Phương vị tăng theo chiều kim đồng hồ tương ứng góc toán học giảm."""
    # Arrange & Act
    north = bearing_deg_to_math_rad(0.0)
    slightly_east_of_north = bearing_deg_to_math_rad(10.0)

    # Assert
    assert slightly_east_of_north < north


def test_bearing_deg_to_math_rad_output_range_within_pi_bounds() -> None:
    """Kiểm tra dải đầu ra của góc radian luôn nằm trong [-pi, pi]."""
    # Arrange & Act & Assert
    for degrees in range(0, 360, 7):
        rad = bearing_deg_to_math_rad(float(degrees))
        assert -math.pi <= rad <= math.pi
