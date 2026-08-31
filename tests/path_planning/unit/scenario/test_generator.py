"""Kiểm thử đơn vị cho bộ sinh ngẫu nhiên kịch bản trong module scenario.generator."""

from __future__ import annotations

from path_planning import config
from path_planning.scenario.generator import (
    create_scenario,
    generate_dynamic_obstacles,
    generate_random_islands,
)
from path_planning.types import Point


def test_generate_random_islands_creates_requested_count_within_bounds() -> None:
    """Kiểm tra sinh đúng số lượng đảo đa giác ngẫu nhiên trong biên bản đồ."""
    # Arrange
    map_bounds = (config.MAP_WIDTH, config.MAP_HEIGHT)
    start: Point = (50000.0, 50000.0)
    goal: Point = (450000.0, 450000.0)
    count = 3
    seed = 42

    # Act
    islands = generate_random_islands(
        count, map_bounds, start=start, goal=goal, seed=seed
    )

    # Assert
    assert len(islands) == count
    for poly in islands:
        assert len(poly) >= 3
        for x, y in poly:
            assert 0.0 <= x <= config.MAP_WIDTH
            assert 0.0 <= y <= config.MAP_HEIGHT


def test_generate_dynamic_obstacles_avoids_start_and_goal_clearance() -> None:
    """Kiểm tra sinh vật cản tròn ngẫu nhiên giữ khoảng cách an toàn với điểm xuất phát và đích."""
    # Arrange
    map_bounds = (config.MAP_WIDTH, config.MAP_HEIGHT)
    count = 4
    seed = 123
    start: Point = (50000.0, 50000.0)
    goal: Point = (450000.0, 450000.0)

    # Act
    circles = generate_dynamic_obstacles(
        count, map_bounds, start=start, goal=goal, seed=seed
    )

    # Assert
    assert len(circles) == count
    for (_cx, _cy), r in circles:
        assert r > 0.0


def test_create_scenario_is_deterministic_with_same_seed() -> None:
    """Kiểm tra tính quyết định: Cùng một giá trị seed sinh ra kịch bản giống nhau 100%."""
    # Arrange
    params = {
        "num_islands": 2,
        "num_dynamic_obstacles": 3,
        "seed": 999,
        "start": (50000.0, 50000.0),
        "start_heading": 0.0,
        "goal": (450000.0, 450000.0),
        "goal_heading": 0.0,
        "map_bounds": (config.MAP_WIDTH, config.MAP_HEIGHT),
        "safezones": None,
    }

    # Act
    scen1 = create_scenario(params)
    scen2 = create_scenario(params)

    # Assert
    assert scen1["islands"] == scen2["islands"]
    assert scen1["dynamic_obstacles"] == scen2["dynamic_obstacles"]
