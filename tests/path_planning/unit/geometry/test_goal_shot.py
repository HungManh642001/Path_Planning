"""Kiểm thử đơn vị cho module geometry.goal_shot (nghiệm giải tích 2 góc rẽ về đích)."""

from __future__ import annotations

import math

import pytest

from path_planning.geometry.goal_shot import (
    TwoCornerCandidate,
    build_goal_cone,
    two_corner_candidates,
)
from path_planning.geometry.spatial import distance
from path_planning.types import Point


def test_two_corner_candidate_dataclass_attributes() -> None:
    """Kiểm tra khởi tạo đối tượng TwoCornerCandidate với đầy đủ trường dữ liệu."""
    # Arrange & Act
    cand = TwoCornerCandidate(
        corner=(1000.0, 2000.0),
        leg1_heading=0.5,
        arrival_heading=1.0,
        budget_corner=500.0,
        budget_goal=600.0,
        total_length=1100.0,
        leg1_len=550.0,
        leg2_len=550.0,
    )

    # Assert
    assert cand.corner == (1000.0, 2000.0)
    assert cand.leg1_heading == 0.5
    assert cand.arrival_heading == 1.0
    assert cand.budget_corner == 500.0
    assert cand.budget_goal == 600.0
    assert cand.total_length == 1100.0
    assert cand.leg1_len == 550.0
    assert cand.leg2_len == 550.0


def test_build_goal_cone_computes_expected_entries() -> None:
    """Kiểm tra sinh trước mảng nón hướng tiếp cận đích."""
    # Arrange
    goal_heading = 0.0
    turn_radius = 8000.0
    alpha_max = math.radians(45.0)
    num_cone = 5

    # Act
    cone = build_goal_cone(goal_heading, turn_radius, alpha_max, num_cone=num_cone)

    # Assert
    assert len(cone) == 5
    for heading, cos_h, sin_h, reserve in cone:
        assert math.isclose(cos_h, math.cos(heading), rel_tol=1e-7)
        assert math.isclose(sin_h, math.sin(heading), rel_tol=1e-7)
        assert reserve >= 0.0

    with pytest.raises(ValueError, match="num_cone must be >= 2"):
        build_goal_cone(goal_heading, turn_radius, alpha_max, num_cone=1)


def test_two_corner_candidates_with_aligned_headings_generates_feasible_corners() -> (
    None
):
    """Kiểm tra sinh ứng viên giải tích 2 góc rẽ khi góc hướng xuất phát và đích cùng chiều."""
    # Arrange
    p1: Point = (50000.0, 50000.0)
    h1 = 0.0  # Hướng Đông
    p2: Point = (250000.0, 100000.0)
    h2 = 0.0  # Hướng Đông
    turn_radius = 8000.0
    alpha_max = math.radians(90.0)
    min_straight = 10.0
    straight_budget = 20000.0
    min_straight_in = 4000.0

    # Act
    candidates = two_corner_candidates(
        p1,
        h1,
        p2,
        h2,
        turn_radius,
        alpha_max,
        min_straight,
        straight_budget,
        min_straight_in,
        num_dir=8,
        num_cone=5,
    )

    # Assert
    assert len(candidates) > 0
    for cand in candidates:
        assert isinstance(cand, TwoCornerCandidate)
        assert distance(p1, cand.corner) > 0.0
        assert distance(cand.corner, p2) > 0.0
        assert math.isclose(cand.leg1_len, distance(p1, cand.corner), rel_tol=1e-7)
        assert math.isclose(cand.leg2_len, distance(cand.corner, p2), rel_tol=1e-7)
        assert math.isclose(
            cand.total_length, cand.leg1_len + cand.leg2_len, rel_tol=1e-7
        )
        assert cand.budget_corner >= min_straight
        assert cand.budget_goal >= min_straight


def test_two_corner_candidates_with_precomputed_cone() -> None:
    """Kiểm tra sinh ứng viên với nón hướng tiếp cận được tính toán trước."""
    # Arrange
    p1: Point = (50000.0, 50000.0)
    h1 = 0.0
    p2: Point = (250000.0, 100000.0)
    h2 = 0.0
    turn_radius = 8000.0
    alpha_max = math.radians(90.0)
    min_straight = 10.0
    straight_budget = 20000.0
    min_straight_in = 4000.0

    cone = build_goal_cone(h2, turn_radius, alpha_max, num_cone=5)

    # Act
    candidates = two_corner_candidates(
        p1,
        h1,
        p2,
        h2,
        turn_radius,
        alpha_max,
        min_straight,
        straight_budget,
        min_straight_in,
        num_dir=8,
        cone=cone,
    )

    # Assert
    assert len(candidates) > 0
    for cand in candidates:
        assert isinstance(cand, TwoCornerCandidate)
        assert cand.leg1_len > 0.0
        assert cand.leg2_len > 0.0


def test_two_corner_candidates_with_insufficient_straight_budget_returns_empty() -> (
    None
):
    """Kiểm tra khi ngân sách đoạn thẳng không đủ bù đắp góc rẽ ban đầu thì không sinh ứng viên."""
    # Arrange
    p1: Point = (50000.0, 50000.0)
    h1 = 0.0
    p2: Point = (100000.0, 100000.0)
    h2 = 0.0
    turn_radius = 8000.0
    alpha_max = math.radians(90.0)
    min_straight = 10.0
    insufficient_budget = 10.0  # Quá nhỏ so với reserve cần thiết
    min_straight_in = 4000.0

    # Act
    candidates = two_corner_candidates(
        p1,
        h1,
        p2,
        h2,
        turn_radius,
        alpha_max,
        min_straight,
        insufficient_budget,
        min_straight_in,
    )

    # Assert
    assert candidates == []
