"""Kiểm thử tích hợp cho cơ chế kiểm soát hạn mức thời gian thực thi (time_budget_s)."""

from __future__ import annotations

import math

import pytest

from path_planning import config
from path_planning.planner import KinodynamicAstar
from path_planning.types import PreprocessedScenario


def test_injected_budget_overrides_global_config(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra truyền tham số time_budget_s ghi đè giá trị cấu hình mặc định."""
    # Arrange
    injected_budget = 2.5

    # Act
    planner = KinodynamicAstar(
        sample_preprocessed_scenario, time_budget_s=injected_budget
    )

    # Assert
    assert planner.time_budget_s == 2.5


def test_budget_falls_back_to_config_default_when_none(
    sample_preprocessed_scenario: PreprocessedScenario,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Kiểm tra khi time_budget_s là None sẽ lấy giá trị mặc định từ config.TIME_BUDGET_S."""
    # Arrange
    monkeypatch.setattr(config, "TIME_BUDGET_S", 7.5)

    # Act
    planner = KinodynamicAstar(sample_preprocessed_scenario, time_budget_s=None)

    # Assert
    assert planner.time_budget_s == 7.5


@pytest.mark.parametrize("invalid_budget", [0.0, -5.0, math.inf, math.nan])
def test_invalid_or_non_positive_budget_raises_value_error(
    sample_preprocessed_scenario: PreprocessedScenario,
    invalid_budget: float,
) -> None:
    """Kiểm tra các giá trị hạn mức thời gian không hợp lệ (<= 0, inf, nan) bị từ chối với ValueError."""
    # Arrange & Act & Assert
    with pytest.raises(ValueError, match="time budget must be"):
        KinodynamicAstar(sample_preprocessed_scenario, time_budget_s=invalid_budget)


def test_search_stats_reflects_injected_time_budget(
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra báo cáo thống kê trả về đúng hạn mức thời gian được phân bổ."""
    # Arrange
    planner = KinodynamicAstar(sample_preprocessed_scenario, time_budget_s=12.0)

    # Act
    planner.plan()
    stats = planner.get_search_stats()

    # Assert
    assert stats["time_budget_s"] == 12.0
    assert "max_iterations" not in stats
