"""Kiểm thử đơn vị cho bộ sinh trạng thái kế tiếp trong module search.successors."""

from __future__ import annotations

import math

from path_planning.collision.detector import CollisionDetector
from path_planning.search.state import State
from path_planning.search.successors import SuccessorGenerator
from path_planning.types import PreprocessedScenario


def test_seed_start_corners_generates_valid_takeoff_states(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra gieo các góc rẽ xuất phát ban đầu dọc theo tia cất cánh l1 >= L0."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    goal_state = State(
        sample_preprocessed_scenario["goal_state"]["waypoint"],
        sample_preprocessed_scenario["goal_state"]["heading"],
    )
    generator = SuccessorGenerator(
        sample_preprocessed_scenario,
        detector,
        origin=sample_preprocessed_scenario["start_pos"],
        target=sample_preprocessed_scenario["goal_pos"],
        goal_state=goal_state,
    )

    # Act
    corners = generator.seed_start_corners()

    # Assert
    assert len(corners) > 0
    start_pos = sample_preprocessed_scenario["start_pos"]
    for corner in corners:
        assert corner.is_start_corner is True
        assert corner.heading == sample_preprocessed_scenario["start_state"]["heading"]
        # Khoảng cách từ điểm xuất phát tới góc rẽ phải >= L0
        assert math.dist(start_pos, corner.waypoint) >= 4000.0


def test_get_next_states_expands_candidates_from_start_corner(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra sinh các trạng thái kế tiếp từ một điểm rẽ cất cánh hợp lệ."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    goal_state = State(
        sample_preprocessed_scenario["goal_state"]["waypoint"],
        sample_preprocessed_scenario["goal_state"]["heading"],
    )
    generator = SuccessorGenerator(
        sample_preprocessed_scenario,
        detector,
        origin=sample_preprocessed_scenario["start_pos"],
        target=sample_preprocessed_scenario["goal_pos"],
        goal_state=goal_state,
    )
    corners = generator.seed_start_corners()
    current_state = corners[0]

    # Act
    successors = generator.get_next_states(current_state)

    # Assert
    assert len(successors) > 0
    for succ_state, step_cost in successors:
        assert isinstance(succ_state, State)
        assert step_cost > 0.0
        assert succ_state.heading is not None


def test_doan_trinh_budget_accounting_checks_straight_segment_reserves(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra tính toán và cập nhật ngân sách đoạn bay thẳng đoản trình."""
    # Arrange
    detector = CollisionDetector(sample_preprocessed_scenario)
    goal_state = State(
        sample_preprocessed_scenario["goal_state"]["waypoint"],
        sample_preprocessed_scenario["goal_state"]["heading"],
    )
    generator = SuccessorGenerator(
        sample_preprocessed_scenario,
        detector,
        origin=sample_preprocessed_scenario["start_pos"],
        target=sample_preprocessed_scenario["goal_pos"],
        goal_state=goal_state,
    )
    current_state = State((50000.0, 50000.0), 0.0)
    current_state.straight_budget = 10000.0
    current_state.min_straight_in = 4000.0

    # Act
    # Bước nhảy đủ dài, góc quay nhỏ
    budget_valid = generator.doan_trinh(
        current_state, leg_len=15000.0, turn=math.radians(15.0)
    )
    # Bước nhảy quá ngắn không đủ bù dự trữ rẽ
    budget_invalid = generator.doan_trinh(
        current_state, leg_len=1000.0, turn=math.radians(80.0)
    )

    # Assert
    assert budget_valid is not None
    assert budget_valid > 0.0
    assert budget_invalid is None
