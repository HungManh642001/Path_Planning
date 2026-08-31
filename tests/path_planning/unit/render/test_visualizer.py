"""Kiểm thử đơn vị cho module vẽ đồ thị matplotlib render.visualizer."""

from __future__ import annotations

import matplotlib
import matplotlib.pyplot as plt

from path_planning import config
from path_planning.render.visualizer import _content_extents, _plot_extents, plot_scenario
from path_planning.types import PlannerState, Scenario

# Chạy backend không tương tác để test trên CI/headless
matplotlib.use("Agg")


def test_content_extents_tightly_fits_features_and_path() -> None:
    """Kiểm tra tính bounding box vừa khít nội dung kịch bản và đường bay."""
    # Arrange
    scenario: Scenario = {
        "start": (10000.0, 10000.0),
        "start_heading": 0.0,
        "goal": (50000.0, 50000.0),
        "goal_heading": 0.0,
        "map_bounds": (config.MAP_WIDTH, config.MAP_HEIGHT),
        "safezones": None,
        "islands": [],
        "dynamic_obstacles": [((30000.0, 30000.0), 5000.0)],
        "obstacles": [],
    }
    path: list[PlannerState] = [((10000.0, 10000.0), 0.0), ((50000.0, 50000.0), 0.0)]

    # Act
    xmin, xmax, ymin, ymax = _content_extents(scenario, path, margin_ratio=0.1)

    # Assert
    assert xmin < 10000.0
    assert xmax > 50000.0
    assert ymin < 10000.0
    assert ymax > 50000.0


def test_plot_extents_respects_view_mode_switch() -> None:
    """Kiểm tra chuyển đổi bounding box giữa chế độ 'map' toàn cảnh và 'content' thu phóng."""
    # Arrange
    scenario: Scenario = {
        "start": (10000.0, 10000.0),
        "start_heading": 0.0,
        "goal": (50000.0, 50000.0),
        "goal_heading": 0.0,
        "map_bounds": (config.MAP_WIDTH, config.MAP_HEIGHT),
        "safezones": None,
        "islands": [],
        "dynamic_obstacles": [],
        "obstacles": [],
    }
    path: list[PlannerState] = [((10000.0, 10000.0), 0.0), ((50000.0, 50000.0), 0.0)]

    # Act
    map_ext = _plot_extents(scenario, path, view="map")
    content_ext = _plot_extents(scenario, path, view="content")

    # Assert
    # View map phải phủ toàn bộ kích thước bản đồ
    assert map_ext == (0.0, config.MAP_WIDTH, 0.0, config.MAP_HEIGHT)
    # View content phải nhỏ hơn nhiều so với toàn bộ bản đồ
    assert content_ext[1] - content_ext[0] < config.MAP_WIDTH


def test_plot_scenario_executes_without_exceptions() -> None:
    """Kiểm tra hàm plot_scenario render thành công đồ thị matplotlib."""
    # Arrange
    scenario: Scenario = {
        "start": (50000.0, 50000.0),
        "start_heading": 0.0,
        "goal": (100000.0, 100000.0),
        "goal_heading": 0.0,
        "map_bounds": (config.MAP_WIDTH, config.MAP_HEIGHT),
        "safezones": None,
        "islands": [],
        "dynamic_obstacles": [],
        "obstacles": [],
    }
    path: list[PlannerState] = [
        ((50000.0, 50000.0), 0.0),
        ((100000.0, 100000.0), 0.0),
    ]

    # Act
    fig = plot_scenario(scenario, path=path, show=False)

    # Assert
    assert fig is not None
    plt.close(fig)
