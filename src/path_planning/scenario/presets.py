"""Tập hợp 16 kịch bản nhiệm vụ chuẩn (benchmark presets)."""

from __future__ import annotations

import math
from collections.abc import Callable

from path_planning.scenario.generator import create_scenario
from path_planning.types import Obstacle, PolygonCoords, Scenario


def scenario1_open_ocean() -> Scenario:
    """Scenario 1: Open ocean - no obstacles.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (2000, 2000),
            "start_heading": math.pi / 4,  # 45 degrees
            "goal": (450000, 450000),
            "goal_heading": math.pi / 4,
            "num_islands": 0,
            "num_dynamic_obstacles": 0,
            "seed": 42,
        }
    )


def scenario2_single_obstacle() -> Scenario:
    """Scenario 2: Single large obstacle in the way.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (2000, 2000),
            "start_heading": math.pi / 4,
            "goal": (450000, 450000),
            "goal_heading": math.pi / 4,
            "num_islands": 1,
            "num_dynamic_obstacles": 1,
            "seed": 42,
        }
    )


def scenario3_narrow_gap() -> Scenario:
    """Scenario 3: Two obstacles very close together (narrow gap).

    Returns:
        Scenario: The configured mission scenario.
    """
    scenario = create_scenario(
        {
            "start": (2000, 2000),
            "start_heading": math.pi / 4,
            "goal": (450000, 450000),
            "goal_heading": math.pi / 4,
            "num_islands": 0,
            "num_dynamic_obstacles": 0,
            "seed": 99,
        }
    )

    # Manually add two close islands. Coordinates are written as floats because
    # a PolygonCoords ring is list[tuple[float, float]] and list is invariant;
    # the values are unchanged.
    island1: PolygonCoords = [
        (22000.0, 20000.0),
        (24000.0, 20000.0),
        (24000.0, 22000.0),
        (22000.0, 22000.0),
    ]
    island2: PolygonCoords = [
        (26000.0, 20000.0),
        (28000.0, 20000.0),
        (28000.0, 22000.0),
        (26000.0, 22000.0),
    ]

    hand_placed: list[Obstacle] = [
        {"type": "polygon", "polygon": island1},
        {"type": "polygon", "polygon": island2},
    ]
    scenario["islands"] = [island1, island2]
    scenario["obstacles"] = hand_placed

    return scenario


def scenario4_complex_maze() -> Scenario:
    """Scenario 4: Complex maze with many obstacles.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (1000, 1000),
            "start_heading": 0,
            "goal": (480000, 480000),
            "goal_heading": 0,
            "num_islands": 12,  # Reduced from 20 for better traversability
            "num_dynamic_obstacles": 6,  # Reduced from 10
            "seed": 12345,
        }
    )


# ============ EASY SCENARIOS (Few obstacles, simple paths) ============


def scenario5_sparse_islands() -> Scenario:
    """Scenario 5: Easy - Sparse islands, plenty of open water.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (5000, 5000),
            "start_heading": math.pi / 4,
            "goal": (450000, 450000),
            "goal_heading": math.pi / 4,
            "num_islands": 3,
            "num_dynamic_obstacles": 1,
            "seed": 111,
        }
    )


def scenario6_coastal_path() -> Scenario:
    """Scenario 6: Easy - Light coastal dynamic obstacles, open corridor.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (10000, 10000),
            "start_heading": 0,
            "goal": (480000, 480000),
            "goal_heading": 0,
            "num_islands": 2,
            "num_dynamic_obstacles": 2,
            "seed": 222,
        }
    )


def scenario7_diagonal_crossing() -> Scenario:
    """Scenario 7: Easy - Minimal obstacles, diagonal crossing.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (20000, 20000),
            "start_heading": math.pi / 4,
            "goal": (470000, 470000),
            "goal_heading": math.pi / 4,
            "num_islands": 4,
            "num_dynamic_obstacles": 0,
            "seed": 333,
        }
    )


def scenario8_open_with_dynamic_obstacles() -> Scenario:
    """Scenario 8: Easy - Open terrain with scattered dynamic obstacles.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (10000, 250000),
            "start_heading": 0,
            "goal": (480000, 250000),
            "goal_heading": 0,
            "num_islands": 1,
            "num_dynamic_obstacles": 3,
            "seed": 444,
        }
    )


# ============ MEDIUM SCENARIOS (Moderate complexity) ============


def scenario9_island_archipelago() -> Scenario:
    """Scenario 9: Medium - Archipelago with multiple islands.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (5000, 250000),
            "start_heading": 0,
            "goal": (490000, 250000),
            "goal_heading": 0,
            "num_islands": 8,
            "num_dynamic_obstacles": 2,
            "seed": 555,
        }
    )


def scenario10_dense_dynamic_obstacles() -> Scenario:
    """Scenario 10: Medium - Dense dynamic obstacle field with some islands.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (50000, 50000),
            "start_heading": math.pi / 4,
            "goal": (450000, 450000),
            "goal_heading": math.pi / 4,
            "num_islands": 3,
            "num_dynamic_obstacles": 8,
            "seed": 666,
        }
    )


def scenario11_serpentine_route() -> Scenario:
    """Scenario 11: Medium - Serpentine path through obstacle field.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (50000, 100000),
            "start_heading": 0,
            "goal": (450000, 400000),
            "goal_heading": 0,
            "num_islands": 7,
            "num_dynamic_obstacles": 4,
            "seed": 777,
        }
    )


def scenario12_perimeter_dynamic_obstacles() -> Scenario:
    """Scenario 12: Medium - Goal protected by perimeter dynamic obstacles.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (10000, 250000),
            "start_heading": 0,
            "goal": (480000, 250000),
            "goal_heading": 0,
            "num_islands": 6,
            "num_dynamic_obstacles": 5,
            "seed": 888,
        }
    )


# ============ HARD SCENARIOS (High complexity, many obstacles) ============


def scenario13_dense_island_field() -> Scenario:
    """Scenario 13: Hard - Very dense island field.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (25000, 25000),
            "start_heading": math.pi / 3,
            "goal": (475000, 475000),
            "goal_heading": math.pi / 3,
            "num_islands": 18,
            "num_dynamic_obstacles": 3,
            "seed": 999,
        }
    )


def scenario14_combined_obstacles() -> Scenario:
    """Scenario 14: Hard - Combined island and dynamic obstacle.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (30000, 30000),
            "start_heading": 0,
            "goal": (470000, 470000),
            "goal_heading": 0,
            "num_islands": 12,
            "num_dynamic_obstacles": 10,
            "seed": 1111,
        }
    )


def scenario15_narrow_channel() -> Scenario:
    """Scenario 15: Hard - Forced through narrow channels between obstacles.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (50000, 250000),
            "start_heading": 0,
            "goal": (450000, 250000),
            "goal_heading": 0,
            "num_islands": 15,
            "num_dynamic_obstacles": 4,
            "seed": 2222,
        }
    )


def scenario16_extreme_complexity() -> Scenario:
    """Scenario 16: Very Hard - Extreme complexity test.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (10000, 10000),
            "start_heading": math.pi / 6,
            "goal": (490000, 490000),
            "goal_heading": math.pi / 6,
            "num_islands": 20,
            "num_dynamic_obstacles": 12,
            "seed": 3333,
        }
    )


def scenario17_reversed_approach_open() -> Scenario:
    """Scenario 17: the seeker must arrive flying BACK along the outbound leg.

    ``goal_heading`` is 180 deg from the start->goal bearing, so no straight run
    at the goal can turn onto it in one corner (that needs a turn > ALPHA_MAX):
    the terminal is a genuine turn-around. Every other preset here approaches
    within 45 deg of the outbound bearing, which left the whole regime
    unmeasured -- and the analytic goal shot exists precisely for it.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (50000, 250000),
            "start_heading": 0,
            "goal": (430000, 250000),
            "goal_heading": math.pi,
            "num_islands": 0,
            "num_dynamic_obstacles": 0,
            "seed": 4242,
        }
    )


def scenario18_reversed_approach_cluttered() -> Scenario:
    """Scenario 18: the same turn-around, with obstacles to turn around inside.

    Returns:
        Scenario: The configured mission scenario.
    """
    return create_scenario(
        {
            "start": (50000, 100000),
            "start_heading": math.pi / 4,
            "goal": (400000, 400000),
            "goal_heading": -3 * math.pi / 4,
            "num_islands": 8,
            "num_dynamic_obstacles": 5,
            "seed": 4343,
        }
    )


def get_all_scenarios() -> dict[str, Callable[[], Scenario]]:
    """Return all 18 predefined scenarios organized by difficulty.

    Returns:
        dict[str, Callable]: Mapping of scenario names to builder functions.
    """
    return {
        # Original scenarios
        "scenario_01_open_ocean": scenario1_open_ocean,
        "scenario_02_single_obstacle": scenario2_single_obstacle,
        "scenario_03_narrow_gap": scenario3_narrow_gap,
        "scenario_04_complex_maze": scenario4_complex_maze,
        # Easy scenarios
        "scenario_05_sparse_islands": scenario5_sparse_islands,
        "scenario_06_coastal_path": scenario6_coastal_path,
        "scenario_07_diagonal_crossing": scenario7_diagonal_crossing,
        "scenario_08_open_with_dynamic_obstacles": (
            scenario8_open_with_dynamic_obstacles
        ),
        # Medium scenarios
        "scenario_09_island_archipelago": scenario9_island_archipelago,
        "scenario_10_dense_dynamic_obstacles": (scenario10_dense_dynamic_obstacles),
        "scenario_11_serpentine_route": scenario11_serpentine_route,
        "scenario_12_perimeter_dynamic_obstacles": (
            scenario12_perimeter_dynamic_obstacles
        ),
        # Hard scenarios
        "scenario_13_dense_island_field": scenario13_dense_island_field,
        "scenario_14_combined_obstacles": scenario14_combined_obstacles,
        "scenario_15_narrow_channel": scenario15_narrow_channel,
        "scenario_16_extreme_complexity": scenario16_extreme_complexity,
        # Reversed approach: goal_heading points back down the outbound leg, so
        # the terminal needs two corners. Nothing above covers this.
        "scenario_17_reversed_approach_open": scenario17_reversed_approach_open,
        "scenario_18_reversed_approach_cluttered": (
            scenario18_reversed_approach_cluttered
        ),
    }
