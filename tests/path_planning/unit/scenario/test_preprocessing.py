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
    """Kiểm tra tính điểm W_1 cách O một đoạn d1 = L0 + R*tan(alpha_max/2)."""
    # Arrange
    origin: Point = (0.0, 0.0)
    heading = 0.0
    l0 = 4000.0
    turn_radius = 8000.0
    alpha_max_rad = math.radians(90.0)

    # Act
    start_state = calculate_start_state(
        origin,
        heading,
        l0=l0,
        turn_radius=turn_radius,
        alpha_max_rad=alpha_max_rad,
    )

    # Assert: d1 = 4000 + 8000 * tan(45 deg) = 12000.0
    assert math.isclose(start_state["waypoint"][0], 12000.0, abs_tol=1e-6)
    assert math.isclose(start_state["waypoint"][1], 0.0, abs_tol=1e-6)
    assert start_state["heading"] == 0.0
    assert start_state["distance_from_origin"] == 12000.0


def test_calculate_end_state_with_fixed_heading_offsets_target_backward() -> None:
    """Kiểm tra tính điểm W_{n-1} lùi về sau đích T một đoạn dn = DSS + R*tan(alpha_max/2)."""
    # Arrange
    target: Point = (100000.0, 100000.0)
    heading = math.pi / 2
    dss = 12000.0
    turn_radius = 8000.0
    alpha_max_rad = math.radians(90.0)

    # Act
    end_state = calculate_end_state(
        target,
        heading,
        dss=dss,
        turn_radius=turn_radius,
        alpha_max_rad=alpha_max_rad,
    )

    # Assert: dn = 12000 + 8000 * tan(45 deg) = 20000.0
    assert end_state is not None
    assert math.isclose(end_state["waypoint"][0], 100000.0, abs_tol=1e-6)
    assert math.isclose(end_state["waypoint"][1], 80000.0, abs_tol=1e-6)
    assert end_state["heading"] == math.pi / 2
    assert end_state["distance_to_target"] == 20000.0


def test_prepare_scenario_handles_free_goal_heading_mode() -> None:
    """Kiểm tra prepare_scenario khi goal_heading là None thiết lập goal_state tại đích T."""
    # Arrange
    scenario: Scenario = {
        "start": (50000.0, 50000.0),
        "start_heading": 0.0,
        "goal": (450000.0, 450000.0),
        "goal_heading": None,
        "map_bounds": (config.MAP_WIDTH, config.MAP_HEIGHT),
        "safezones": None,
        "islands": [],
        "dynamic_obstacles": [],
        "obstacles": [],
    }

    # Act
    prep = prepare_scenario(scenario)

    # Assert
    assert prep["goal_state"]["waypoint"] == (450000.0, 450000.0)
    assert prep["goal_state"]["heading"] is None


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
