"""Kiểm thử đơn vị cho module vẽ đồ thị matplotlib render.visualizer."""

from __future__ import annotations

import matplotlib
import matplotlib.pyplot as plt

from path_planning import config
from path_planning.render.visualizer import _content_extents, _plot_extents, plot_scenario
from path_planning.types import PreprocessedScenario, Scenario

# Chạy backend không tương tác để test trên CI/headless
matplotlib.use("Agg")


def test_content_extents_tightly_fits_features_and_path(
    sample_clean_scenario: Scenario,
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra tính bounding box vừa khít nội dung kịch bản và đường bay."""
    # Arrange & Act
    (xmin, xmax), (ymin, ymax) = _content_extents(
        sample_clean_scenario, sample_preprocessed_scenario
    )

    # Assert
    assert xmin < sample_clean_scenario["start"][0]
    assert xmax > sample_clean_scenario["start"][0]
    assert ymin < sample_clean_scenario["start"][1]
    assert ymax > sample_clean_scenario["start"][1]


def test_plot_extents_covers_entire_map(
    sample_clean_scenario: Scenario,
) -> None:
    """Kiểm tra tính bounding box toàn cảnh bao phủ toàn bộ bản đồ."""
    # Arrange & Act
    (xmin, xmax), (ymin, ymax) = _plot_extents(sample_clean_scenario)

    # Assert
    assert xmin <= 0.0
    assert xmax >= config.MAP_WIDTH
    assert ymin <= 0.0
    assert ymax >= config.MAP_HEIGHT


def test_plot_scenario_executes_without_exceptions(
    sample_clean_scenario: Scenario,
    sample_preprocessed_scenario: PreprocessedScenario,
) -> None:
    """Kiểm tra hàm plot_scenario render thành công đồ thị matplotlib."""
    # Arrange & Act
    fig = plot_scenario(
        sample_clean_scenario,
        sample_preprocessed_scenario,
        result=None,
        fit="map",
    )

    # Assert
    assert fig is not None
    plt.close(fig)
