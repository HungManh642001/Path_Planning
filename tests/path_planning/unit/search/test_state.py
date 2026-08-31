"""Kiểm thử đơn vị cho class State và lượng tử hóa ô lưới trong module search.state."""

from __future__ import annotations

import math

import pytest

from path_planning.search.state import State, state_to_tuple
from path_planning.types import Point


def test_state_initialization_with_heading_sets_trigonometric_caches() -> None:
    """Kiểm tra khởi tạo nút State với hướng bay tự động tính trước cos và sin."""
    # Arrange
    waypoint: Point = (1000.0, 2000.0)
    heading = math.pi / 4  # 45 deg

    # Act
    state = State(waypoint, heading)

    # Assert
    assert state.waypoint == waypoint
    assert state.heading == heading
    assert state.cos_h is not None and math.isclose(state.cos_h, math.cos(heading))
    assert state.sin_h is not None and math.isclose(state.sin_h, math.sin(heading))
    assert state.g_cost == float("inf")
    assert state.h_cost == 0.0
    assert state.parent is None


def test_state_initialization_with_none_heading_leaves_caches_none() -> None:
    """Kiểm tra khởi tạo State mục tiêu tự do (heading=None) giữ cos/sin là None."""
    # Arrange
    waypoint: Point = (1000.0, 2000.0)

    # Act
    state = State(waypoint, None)

    # Assert
    assert state.waypoint == waypoint
    assert state.heading is None
    assert state.cos_h is None
    assert state.sin_h is None


def test_state_compute_key_with_none_heading_raises_type_error() -> None:
    """Kiểm tra tính khóa ô lưới khi heading là None ném ra ngoại lệ TypeError."""
    # Arrange
    state = State((1000.0, 2000.0), None)

    # Act & Assert
    with pytest.raises(TypeError, match="a headingless goal target has no lattice key"):
        state._compute_key()


def test_state_equality_and_hashing_matches_discretized_lattice() -> None:
    """Kiểm tra so sánh bằng nhau và băm 2 đối tượng State rơi vào cùng ô lưới."""
    # Arrange
    state_a = State((1000.1, 2000.1), 0.523)
    state_b = State((1000.2, 2000.2), 0.524)

    # Act & Assert
    assert state_a == state_b
    assert hash(state_a) == hash(state_b)


def test_state_repr_formats_waypoint_and_heading() -> None:
    """Kiểm tra định dạng chuỗi đại diện State chứa thông tin tọa độ."""
    # Arrange
    state = State((100.0, 200.0), 0.0)

    # Act
    repr_str = repr(state)

    # Assert
    assert "100.0" in repr_str
    assert "200.0" in repr_str


def test_state_to_tuple_discretizes_coordinates_into_bins() -> None:
    """Kiểm tra hàm state_to_tuple chia tọa độ thành các bin số nguyên rời rạc."""
    # Arrange
    wp: Point = (250000.0, 150000.0)
    heading = math.pi / 2

    # Act
    key = state_to_tuple(wp, heading)

    # Assert
    assert isinstance(key, tuple)
    assert len(key) == 3
    assert all(isinstance(v, int) for v in key)
