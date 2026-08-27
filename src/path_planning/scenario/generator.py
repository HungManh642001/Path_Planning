"""Bộ sinh kịch bản ngẫu nhiên theo thuật toán (Procedural Generator).

Xây dựng kịch bản bay giả lập gồm đảo đa giác và chướng ngại vật tròn.
Đảm bảo tính tái lập qua seed và khoảng cách cách ly giữa các vật cản.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from shapely import Point as ShapelyPoint, Polygon as ShapelyPolygon

from path_planning import config
from path_planning.geometry import spatial as su
from path_planning.types import (
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
    """Hình học đoạn thẳng nối điểm xuất phát và đích.

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
    """Tính trước các thông số hình học của đường nối start-goal."""
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
    """Lấy mẫu tọa độ tâm chướng ngại vật trong phạm vi 80% diện tích giữa bản đồ.

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
            f"unknown topology {topology!r}; expected 'random', "
            "'center_cluster' or 'wall_block'"
        )
    return (
        max(width * 0.1, min(center_x, width * 0.9)),
        max(height * 0.1, min(center_y, height * 0.9)),
    )


def _clears_endpoints(shape: ShapelyPolygon, start: Point, goal: Point) -> bool:
    """Kiểm tra vật cản có cách ly an toàn khỏi điểm cất cánh và đích không.

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
    """Sinh danh sách các đảo đa giác ngẫu nhiên không giao nhau.

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
    """Sinh danh sách các chướng ngại vật hình tròn không giao nhau.

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
    """Tạo kịch bản nhiệm vụ hoàn chỉnh gồm điểm đầu cuối và tập chướng ngại vật.

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


def generate_random_scenario(seed: int = 42) -> Scenario:
    """Sinh kịch bản ngẫu nhiên hoàn chỉnh với đảo và chướng ngại vật tròn.

    Args:
        seed: Random seed for reproducibility.

    Returns:
        Scenario: Dictionary containing map bounds, start/goal, islands, and dynamic
            obstacles.
    """
    random.seed(seed)

    # Map bounds
    map_bounds = (config.MAP_WIDTH, config.MAP_HEIGHT)
    width, height = map_bounds

    # Random start and goal positions within the map bounds
    while True:
        start = (
            random.uniform(width * 0.1, width * 0.9),
            random.uniform(height * 0.1, height * 0.9),
        )
        goal = (
            random.uniform(width * 0.1, width * 0.9),
            random.uniform(height * 0.1, height * 0.9),
        )
        if su.distance(start, goal) > 400000:  # Ensure start and goal are not too close
            break

    heading_start_to_goal = su.angle_to_heading(start, goal)

    topologies: tuple[Topology, ...] = ("random", "center_cluster", "wall_block")
    topology = random.choices(topologies, weights=[0.1, 0.45, 0.45])[0]

    return create_scenario(
        {
            "map_bounds": map_bounds,
            "start": start,
            "start_heading": heading_start_to_goal
            + random.uniform(
                -math.pi / 2, math.pi / 2
            ),  # Add some randomness to the start heading
            "goal": goal,
            "goal_heading": heading_start_to_goal
            + random.uniform(
                -math.pi / 2, math.pi / 2
            ),  # Add some randomness to the goal heading
            "num_islands": random.randint(0, 20),
            "num_dynamic_obstacles": random.randint(0, 20),
            "topology": topology,
            "seed": seed,
        }
    )
