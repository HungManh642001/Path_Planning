"""Kiểm thử đơn vị cho các hàm tiền xử lý kịch bản trong module scenario.preprocessing."""

from __future__ import annotations

import math

from path_planning import config
from path_planning.scenario.preprocessing import (
    calculate_end_state,
    calculate_start_state,
    inflate_obstacles,
    prepare_scenario,
)
from path_planning.types import Obstacle, Point, Scenario


def test_calculate_start_state_offsets_takeoff_along_heading_ray() -> None:
    """Kiểm tra tính điểm W_1 cách điểm cất cánh O đúng khoảng cách L0."""
    # Arrange
    origin: Point = (0.0, 0.0)
    heading = 0.0
    l0 = 4000.0

    # Act
    start_state = calculate_start_state(origin, heading, l0=l0)

    # Assert
    assert start_state["waypoint"] == (4000.0, 0.0)
    assert start_state["heading"] == 0.0


def test_calculate_end_state_with_fixed_heading_offsets_target_backward() -> None:
    """Kiểm tra tính điểm W_{n-1} lùi về sau điểm đích T một khoảng DSS."""
    # Arrange
    target: Point = (100000.0, 100000.0)
    heading = math.pi / 2
    dss = 12000.0

    # Act
    end_state = calculate_end_state(target, heading, dss=dss)

    # Assert
    assert end_state is not None
    assert math.isclose(end_state["waypoint"][0], 100000.0, abs_tol=1e-6)
    assert math.isclose(end_state["waypoint"][1], 88000.0, abs_tol=1e-6)
    assert end_state["heading"] == math.pi / 2


def test_calculate_end_state_with_none_heading_returns_target_directly() -> None:
    """Kiểm tra khi goal_heading là None (free-goal) trả về thẳng tọa độ đích T."""
    # Arrange
    target: Point = (100000.0, 100000.0)

    # Act
    end_state = calculate_end_state(target, None, dss=12000.0)

    # Assert
    assert end_state is not None
    assert end_state["waypoint"] == target
    assert end_state["heading"] is None


def test_inflate_obstacles_expands_circles_and_polygons() -> None:
    """Kiểm tra mở rộng bán kính hình tròn và đa giác theo khoảng cách an toàn safe_margin."""
    # Arrange
    raw_obstacles: list[Obstacle] = [
        {"type": "circle", "center": (100000.0, 100000.0), "radius": 20000.0},
        {
            "type": "polygon",
            "polygon": [
                (200000.0, 100000.0),
                (230000.0, 100000.0),
                (230000.0, 130000.0),
                (200000.0, 130000.0),
            ],
        },
    ]
    safe_margin = 1000.0

    # Act
    inflated = inflate_obstacles(raw_obstacles, safe_margin=safe_margin)

    # Assert
    assert len(inflated) == len(raw_obstacles)
    assert inflated[0]["radius"] == 21000.0


def test_prepare_scenario_packages_all_preprocessed_fields(
    sample_clean_scenario: Scenario,
) -> None:
    """Kiểm tra hàm prepare_scenario đóng gói đầy đủ các trường dữ liệu tiền xử lý."""
    # Arrange & Act
    prep = prepare_scenario(sample_clean_scenario)

    # Assert
    assert "start_pos" in prep
    assert "goal_pos" in prep
    assert "start_state" in prep
    assert "goal_state" in prep
    assert "circle_obstacles" in prep
    assert "polygon_obstacles" in prep
    assert "obstacles" in prep
    assert prep["alpha_max_rad"] == config.ALPHA_MAX_RAD
