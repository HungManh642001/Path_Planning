"""Mock map generator.

Builds synthetic mission scenarios -- islands and dynamic obstacles -- for
testing the path planner. Every scenario, including the 16 named presets, draws
its obstacles from these seeded generators; only the endpoints and counts are
hand-set.
"""

from __future__ import annotations

import math
import random
from collections.abc import Callable
from dataclasses import dataclass

from shapely import Point as ShapelyPoint, Polygon as ShapelyPolygon

from path_planning import config
from path_planning.core.types import (
    CircleGeometry,
    MapBounds,
    Obstacle,
    Point,
    PolygonCoords,
    Scenario,
    ScenarioConfig,
    Topology,
)


_MAX_PLACEMENT_ATTEMPTS = 1000
"""Consecutive rejected placements before a generator gives up on the rest."""


@dataclass(frozen=True)
class _StartGoalGeometry:
    """The start-goal line, precomputed once per generator call.

    Attributes:
        mx: Midpoint x between start and goal.
        my: Midpoint y between start and goal.
        angle_start_goal: Bearing from start to goal (rad).
        angle_perp: The perpendicular to that bearing (rad).
        dist_sg: Separation between start and goal (m).
    """

    mx: float
    my: float
    angle_start_goal: float
    angle_perp: float
    dist_sg: float


def _start_goal_geometry(start: Point, goal: Point) -> _StartGoalGeometry:
    """Derive the start-goal line the topology samplers place obstacles against."""
    angle_start_goal = math.atan2(goal[1] - start[1], goal[0] - start[0])
    return _StartGoalGeometry(
        mx=(start[0] + goal[0]) / 2,
        my=(start[1] + goal[1]) / 2,
        angle_start_goal=angle_start_goal,
        angle_perp=angle_start_goal + math.pi / 2,
        dist_sg=math.hypot(goal[0] - start[0], goal[1] - start[1]),
    )


def _sample_center(
    topology: Topology, map_bounds: MapBounds, geom: _StartGoalGeometry
) -> Point:
    """Draw one candidate obstacle centre, clamped into the middle 80% of the map.

    Shared by both generators -- they placed centres with the same 20 lines and
    the same three topologies, and a divergence between them would be silent.

    The draw ORDER is part of the contract: scenarios are reproduced from a
    seed, so every branch must consume exactly the random values it consumed
    before (two uniforms, two gausses, or t-then-noise).

    Args:
        topology: Placement strategy.
        map_bounds: The ``(width, height)`` map rectangle.
        geom: Precomputed start-goal line geometry.

    Returns:
        A candidate centre inside the middle 80% of the map.

    Raises:
        ValueError: If ``topology`` is not one of the three known strategies.
    """
    width, height = map_bounds
    if topology == "random":
        center_x = random.uniform(width * 0.1, width * 0.9)
        center_y = random.uniform(height * 0.1, height * 0.9)
    elif topology == "center_cluster":
        # Gaussian around the midpoint between start and goal.
        center_x = random.gauss(geom.mx, geom.dist_sg / 3)
        center_y = random.gauss(geom.my, geom.dist_sg / 3)
    elif topology == "wall_block":
        # Along a line perpendicular to the start-goal line.
        t = random.uniform(-150000, 150000)
        noise = random.uniform(-geom.dist_sg / 3, geom.dist_sg / 3)
        center_x = (
            geom.mx
            + t * math.cos(geom.angle_perp)
            + noise * math.cos(geom.angle_start_goal)
        )
        center_y = (
            geom.my
            + t * math.sin(geom.angle_perp)
            + noise * math.sin(geom.angle_start_goal)
        )
    else:
        # Previously this fell through with center_x unbound: a NameError on the
        # first pass, or -- worse -- silently reusing the previous iteration's
        # centre on later ones, so every obstacle landed on the same spot.
        raise ValueError(
            f"unknown topology {topology!r}; expected 'random', 'center_cluster' or 'wall_block'"
        )
    return (
        max(width * 0.1, min(center_x, width * 0.9)),
        max(height * 0.1, min(center_y, height * 0.9)),
    )


def _clears_endpoints(shape: ShapelyPolygon, start: Point, goal: Point) -> bool:
    """Test that a candidate obstacle keeps the spawn clearance from both endpoints.

    The buffer used to be ``config.EPS`` (1e-6 m), which let an obstacle touch
    the start point and left the mandatory takeoff or seeker leg born blocked.

    Args:
        shape: The candidate obstacle geometry.
        start: Start position.
        goal: Goal position.

    Returns:
        ``True`` if both endpoints keep ``config.SPAWN_CLEARANCE_M``.
    """
    return (
        shape.distance(ShapelyPoint(start)) >= config.SPAWN_CLEARANCE_M
        and shape.distance(ShapelyPoint(goal)) >= config.SPAWN_CLEARANCE_M
    )


def generate_random_islands(
    num_islands: int,
    map_bounds: MapBounds,
    start: Point,
    goal: Point,
    *,
    topology: Topology = "random",
    seed: int | None = None,
) -> list[PolygonCoords]:
    """Generate non-overlapping island polygons with irregular shapes.

    Args:
        num_islands: Number of islands to place.
        map_bounds: The ``(width, height)`` map rectangle.
        start: Start position, kept clear of obstacles.
        goal: Goal position, kept clear of obstacles.
        topology: Placement strategy.
        seed: Random seed for reproducibility.

    Returns:
        The placed islands, each an open ring of ``(x, y)`` vertices. Fewer than
        ``num_islands`` if the map is too tight to fit them all.
    """
    if seed is not None:
        random.seed(seed)

    islands: list[PolygonCoords] = []
    placed: list[ShapelyPolygon] = []  # Polygon form of `islands`, for separation
    attempts = 0
    geom = _start_goal_geometry(start, goal)

    while len(islands) < num_islands and attempts < _MAX_PLACEMENT_ATTEMPTS:
        center_x, center_y = _sample_center(topology, map_bounds, geom)
        size = random.uniform(config.ISLAND_SIZE_MIN, config.ISLAND_SIZE_MAX)
        num_vertices = random.randint(
            config.ISLAND_VERTICES_MIN, config.ISLAND_VERTICES_MAX
        )

        # Irregular star-like polygon: each vertex radius perturbed independently.
        island: PolygonCoords = []
        for i in range(num_vertices):
            angle = 2 * math.pi * i / num_vertices
            radius = size * random.uniform(0.6, 1.0)
            island.append(
                (
                    center_x + radius * math.cos(angle),
                    center_y + radius * math.sin(angle),
                )
            )

        island_polygon = ShapelyPolygon(island)

        # Islands must not overlap each other, the same rule the circle
        # generator enforces. Compared as real polygon distance rather than a
        # centre-distance heuristic, so irregular shapes are handled exactly.
        valid = all(
            island_polygon.distance(p) >= config.ISLAND_MIN_SEPARATION_M for p in placed
        ) and _clears_endpoints(island_polygon, start, goal)

        if valid:
            islands.append(island)
            placed.append(island_polygon)
            attempts = 0
        else:
            attempts += 1

    return islands


def generate_dynamic_obstacles(
    num_sites: int,
    map_bounds: MapBounds,
    start: Point,
    goal: Point,
    *,
    topology: Topology = "random",
    seed: int | None = None,
) -> list[CircleGeometry]:
    """Generate non-overlapping circular dynamic obstacles.

    Args:
        num_sites: Number of obstacles to place.
        map_bounds: The ``(width, height)`` map rectangle.
        start: Start position, kept clear of obstacles.
        goal: Goal position, kept clear of obstacles.
        topology: Placement strategy.
        seed: Random seed for reproducibility.

    Returns:
        The placed obstacles as ``(center, radius)``. Fewer than ``num_sites``
        if the map is too tight to fit them all.
    """
    if seed is not None:
        random.seed(seed)

    dynamic_obstacles: list[CircleGeometry] = []
    attempts = 0
    geom = _start_goal_geometry(start, goal)

    while len(dynamic_obstacles) < num_sites and attempts < _MAX_PLACEMENT_ATTEMPTS:
        center = _sample_center(topology, map_bounds, geom)
        radius = random.uniform(config.OBSTACLE_RADIUS_MIN, config.OBSTACLE_RADIUS_MAX)

        # Separation is measured between the two BOUNDARIES (r_i + r_j + gap),
        # not against a flat 2*max_radius heuristic: charging every pair the
        # worst-case radius made the effective spacing 100.5 km on a 500 km map,
        # which capped the map at ~13 circles however many were requested.
        valid = all(
            math.hypot(center[0] - other_center[0], center[1] - other_center[1])
            >= radius + other_radius + config.CIRCLE_MIN_SEPARATION_M
            for other_center, other_radius in dynamic_obstacles
        ) and _clears_endpoints(ShapelyPoint(center).buffer(radius), start, goal)

        if valid:
            dynamic_obstacles.append((center, radius))
            attempts = 0
        else:
            attempts += 1

    return dynamic_obstacles


def create_scenario(scenario_config: ScenarioConfig) -> Scenario:
    """Build a complete scenario: endpoints, obstacles and the unified obstacle list.

    Args:
        scenario_config: The recipe. ``start`` and ``goal`` are mandatory; the
            generator knobs (``num_islands``, ``num_dynamic_obstacles``,
            ``topology``, ``seed``, ``map_bounds``, ``safezones``) all default.
            A ``goal_heading`` of ``None`` selects free-goal mode.

    Returns:
        The generated scenario.

    Raises:
        ValueError: If ``start`` or ``goal`` is missing. Both are required
            because the topology samplers place obstacles relative to the
            start-goal line.
    """
    start = scenario_config.get("start")
    goal = scenario_config.get("goal")
    if start is None or goal is None:
        raise ValueError("scenario_config requires both 'start' and 'goal'")

    map_bounds = scenario_config.get(
        "map_bounds", (config.MAP_WIDTH, config.MAP_HEIGHT)
    )
    topology = scenario_config.get("topology", "random")
    seed = scenario_config.get("seed")

    islands = generate_random_islands(
        scenario_config.get("num_islands", 0),
        map_bounds,
        start,
        goal,
        topology=topology,
        seed=seed,
    )
    dynamic_obstacles = generate_dynamic_obstacles(
        scenario_config.get("num_dynamic_obstacles", 0),
        map_bounds,
        start,
        goal,
        topology=topology,
        seed=seed,
    )

    obstacles: list[Obstacle] = []
    for island in islands:
        obstacles.append({"type": "polygon", "polygon": island})
    for center, radius in dynamic_obstacles:
        obstacles.append({"type": "circle", "center": center, "radius": radius})

    return {
        "start": start,
        "start_heading": scenario_config.get("start_heading", 0),
        "goal": goal,
        # None => free terminal approach direction (the planner chooses it).
        "goal_heading": scenario_config.get("goal_heading"),
        "map_bounds": map_bounds,
        # Optional operating areas: a LIST of polygons, each a list of (x, y)
        # vertices. The aircraft must stay inside their union. None/empty =>
        # fall back to the config.MAP_WIDTH/HEIGHT rectangle.
        "safezones": scenario_config.get("safezones"),
        "islands": islands,
        "dynamic_obstacles": dynamic_obstacles,
        "obstacles": obstacles,
    }


# ============ PREDEFINED SCENARIOS ============


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
        dict[str, Callable]: A dictionary mapping scenario names to their builder functions.
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
        "scenario_08_open_with_dynamic_obstacles": scenario8_open_with_dynamic_obstacles,
        # Medium scenarios
        "scenario_09_island_archipelago": scenario9_island_archipelago,
        "scenario_10_dense_dynamic_obstacles": scenario10_dense_dynamic_obstacles,
        "scenario_11_serpentine_route": scenario11_serpentine_route,
        "scenario_12_perimeter_dynamic_obstacles": scenario12_perimeter_dynamic_obstacles,
        # Hard scenarios
        "scenario_13_dense_island_field": scenario13_dense_island_field,
        "scenario_14_combined_obstacles": scenario14_combined_obstacles,
        "scenario_15_narrow_channel": scenario15_narrow_channel,
        "scenario_16_extreme_complexity": scenario16_extreme_complexity,
        # Reversed approach: goal_heading points back down the outbound leg, so
        # the terminal needs two corners. Nothing above covers this.
        "scenario_17_reversed_approach_open": scenario17_reversed_approach_open,
        "scenario_18_reversed_approach_cluttered": scenario18_reversed_approach_cluttered,
    }
