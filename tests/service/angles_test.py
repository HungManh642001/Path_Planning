"""Quy ước hướng là chỗ dễ sinh lỗi nhất của toàn service.

Một đường bay lệch 90 độ hoặc bị gương vẫn hợp lệ về hình học, nên mọi test hình
học khác đều bỏ lọt loại lỗi này. Nó phải bị chặn ở đây.
"""

from __future__ import annotations

import math

import pytest

from service.vtx_service.angles import bearing_deg_to_math_rad, math_rad_to_bearing_deg


# phương vị (độ, thuận kim đồng hồ từ bắc) -> heading toán học (rad, ngược kim
# đồng hồ từ +x). Quy ước: +y bắc, +x đông.
KNOWN = [
    (0.0, math.pi / 2),  # bắc  -> +y
    (90.0, 0.0),  # đông -> +x
    (180.0, -math.pi / 2),  # nam  -> -y
    (270.0, math.pi),  # tây  -> -x
    (45.0, math.pi / 4),  # đông bắc
]


@pytest.mark.parametrize("bearing_deg,expected_rad", KNOWN)
def test_cardinal_directions(bearing_deg: float, expected_rad: float) -> None:
    got = bearing_deg_to_math_rad(bearing_deg)
    assert math.isclose(math.cos(got), math.cos(expected_rad), abs_tol=1e-12)
    assert math.isclose(math.sin(got), math.sin(expected_rad), abs_tol=1e-12)


@pytest.mark.parametrize("bearing_deg", [0.0, 12.5, 90.0, 179.9, 180.0, 270.0, 359.99])
def test_round_trip_is_stable(bearing_deg: float) -> None:
    back = math_rad_to_bearing_deg(bearing_deg_to_math_rad(bearing_deg))
    assert math.isclose(back, bearing_deg, abs_tol=1e-9)


def test_result_is_normalised_to_a_single_turn() -> None:
    assert 0.0 <= math_rad_to_bearing_deg(bearing_deg_to_math_rad(730.0)) < 360.0


def test_negative_and_wrapped_bearings_agree() -> None:
    # So HƯỚNG, không so cách biểu diễn: -pi và +pi là cùng một hướng.
    a = bearing_deg_to_math_rad(-90.0)
    b = bearing_deg_to_math_rad(270.0)
    assert math.isclose(math.cos(a), math.cos(b), abs_tol=1e-12)
    assert math.isclose(math.sin(a), math.sin(b), abs_tol=1e-12)


def test_bearing_increases_clockwise_not_counterclockwise() -> None:
    """Phép thử phân biệt hai quy ước. Ai lật dấu thì test này đỏ."""
    north = bearing_deg_to_math_rad(0.0)
    slightly_east_of_north = bearing_deg_to_math_rad(10.0)
    # Quay thuận kim đồng hồ trên mặt đất = GIẢM góc toán học.
    assert slightly_east_of_north < north


def test_range_is_what_the_docstring_promises() -> None:
    """Khẳng định này đã bắt lỗi trong bản nháp đầu của module.

    Cách viết bằng số học modulo trả về 4,712 rad ở phương vị 180 độ - đúng
    HƯỚNG, nên mọi test cos/sin ở trên vẫn xanh, nhưng ngoài dải docstring hứa.
    Sai lệch kiểu đó chỉ lộ ra ở downstream, nơi có ai đó so hai góc trực tiếp.
    """
    for degrees in range(0, 360, 7):
        assert -math.pi <= bearing_deg_to_math_rad(float(degrees)) <= math.pi
