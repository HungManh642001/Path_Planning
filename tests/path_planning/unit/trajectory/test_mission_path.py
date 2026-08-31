"""Kiểm thử đơn vị cho hàm ghép nối đường bay nhiệm vụ trong module trajectory.mission_path."""

from __future__ import annotations

import math

from path_planning.trajectory.mission_path import full_mission_path
from path_planning.types import PlannerState, PreprocessedScenario


def test_full_mission_path_prepends_origin_and_appends_target(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra thêm điểm cất cánh O vào đầu và điểm đích T vào cuối đường bay."""
    # Arrange
    interior_path: list[PlannerState] = [
        ((100000.0, 100000.0), 0.785),
        ((350000.0, 350000.0), 0.785),
    ]

    # Act
    full_path = full_mission_path(interior_path, sample_preprocessed_scenario)

    # Assert
    assert len(full_path) == 4
    assert full_path[0][0] == sample_preprocessed_scenario["start_pos"]
    assert full_path[-1][0] == sample_preprocessed_scenario["goal_pos"]


def test_full_mission_path_leaves_already_complete_path_alone(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra không bị nhân đôi điểm mút khi gọi lại full_mission_path lần thứ hai."""
    # Arrange
    interior_path: list[PlannerState] = [
        ((100000.0, 100000.0), 0.785),
        ((350000.0, 350000.0), 0.785),
    ]
    full_once = full_mission_path(interior_path, sample_preprocessed_scenario)

    # Act
    full_twice = full_mission_path(full_once, sample_preprocessed_scenario)

    # Assert
    assert len(full_twice) == len(full_once)
    assert full_twice[0][0] == sample_preprocessed_scenario["start_pos"]
    assert full_twice[-1][0] == sample_preprocessed_scenario["goal_pos"]


def test_full_mission_path_with_none_preprocessed_returns_unchanged_path() -> None:
    """Kiểm tra khi kịch bản tiền xử lý là None trả về danh sách đường bay nguyên bản."""
    # Arrange
    interior_path: list[PlannerState] = [((100000.0, 100000.0), 0.785)]

    # Act
    result = full_mission_path(interior_path, None)

    # Assert
    assert result == interior_path


def test_full_mission_path_computes_arrival_bearing_in_free_goal_mode(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra tính hướng bay tiếp cận vào T dựa trên phương vị cạnh cuối khi goal_heading là None."""
    # Arrange
    free_goal_scenario = dict(sample_preprocessed_scenario)
    free_goal_scenario["goal_heading"] = None
    target = free_goal_scenario["goal_pos"]
    last_wp = (target[0] - 10000.0, target[1])
    interior_path: list[PlannerState] = [(last_wp, 0.0)]

    # Act
    full_path = full_mission_path(interior_path, free_goal_scenario)  # type: ignore[arg-type]

    # Assert
    assert len(full_path) == 3
    final_heading = full_path[-1][1]
    assert math.isclose(final_heading, 0.0, abs_tol=1e-6)
