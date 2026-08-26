"""Preprocessing and boundary calculation.

Inflates obstacles by the operator stand-off margin and derives the offset
start/goal waypoint states the search actually targets. Distances are metres,
angles radians.
"""

from __future__ import annotations

import math

from path_planning import config
from path_planning.core import spatial_utils as su
from path_planning.core.types import (
    CircleGeometry,
    GoalState,
    InflatedObstacleSets,
    Obstacle,
    Point,
    PolygonCoords,
    PreprocessedScenario,
    Scenario,
    StartState,
)


def inflation_ring(safe_margin: float = config.SAFE_MARGIN) -> float:
    """Return the obstacle boundary offset used for display.

    There is a SINGLE ring, ``safe_margin`` -- exactly what
    :func:`inflate_obstacles` applies. This used to return a PAIR because a
    second ring added a ``R*(1/cos(alpha_max/2)-1)`` turn term reserving the
    worst-case fillet bulge; that term is gone (the search checks each arc
    exactly instead), so the two values had become identical and the only caller
    was already discarding the second.

    Args:
        safe_margin: The operator's minimum stand-off distance (m).

    Returns:
        The boundary offset in metres.
    """
    return safe_margin


def inflate_obstacles(
    obstacles: list[Obstacle], safe_margin: float = config.SAFE_MARGIN
) -> list[Obstacle]:
    """Inflate obstacle boundaries by the stand-off margin.

    Obstacles stay independent -- no early convex hull, so a corridor between
    two of them survives.

    Args:
        obstacles: Raw obstacle records.
        safe_margin: The operator's minimum stand-off distance (m).

    Returns:
        New obstacle records with boundaries offset outward; inputs are not
        mutated.
    """
    # SAFE_MARGIN only. The old `R*(1/cos(alpha_max/2)-1)` turn term covered the
    # worst-case bulge of a fillet arc into the corner it cuts; it is gone
    # because the search now checks that bulge EXACTLY, per corner, with the
    # real turn angle (`_is_corner_arc_clear`). Sized for alpha_max and applied to
    # every obstacle, the term closed 49% of all corridors between obstacle
    # pairs (measured, 1536 pairs) - a straight transit paid the same 3.3 km as
    # a 90-degree corner. Inflation is now purely the operator's minimum
    # stand-off distance. See docs/superpowers/specs/2026-08-08-obstacle-
    # inflation-safe-margin-design.md.
    inflated: list[Obstacle] = []
    for obstacle in obstacles:
        if obstacle["type"] == "circle":
            circle = obstacle.copy()
            circle["radius"] = obstacle["radius"] + safe_margin
            inflated.append(circle)
        else:
            polygon = obstacle.copy()
            polygon["polygon"] = su.inflate_polygon(obstacle["polygon"], safe_margin)
            inflated.append(polygon)
    return inflated


def calculate_start_state(
    origin: Point,
    init_heading: float,
    l0: float = config.L0,
    turn_radius: float = config.R,
    alpha_max_rad: float = config.ALPHA_MAX_RAD,
) -> StartState:
    """Compute the first waypoint ``W_1`` and its heading after takeoff.

    From the dynamics ``d_1 = l_1 + R * tan(alpha_1 / 2)`` under the constraint
    ``l_1 >= L_0``. ``W_1`` is placed at distance ``d_1`` along ``init_heading``.

    Args:
        origin: Takeoff point ``O``.
        init_heading: Initial heading (rad).
        l0: Minimum straight distance for level-flight stabilisation (m).
        turn_radius: Vehicle turn radius (m).
        alpha_max_rad: Maximum turn angle allowed (rad).

    Returns:
        The start state: waypoint, heading, straight length ``l_1`` and
        distance ``d_1`` from the origin.
    """
    straight_length = l0
    # Conservative: reserve tangent length for the worst-case first turn
    # alpha_1 = alpha_max, so d_1 = L0 + R*tan(alpha_max/2) and l_1 = L0
    # exactly (l_1 >= L0 holds).
    distance_from_origin = straight_length + turn_radius * math.tan(alpha_max_rad / 2)
    return {
        "waypoint": (
            origin[0] + distance_from_origin * math.cos(init_heading),
            origin[1] + distance_from_origin * math.sin(init_heading),
        ),
        "heading": init_heading,
        "straight_length": straight_length,
        "distance_from_origin": distance_from_origin,
    }


def calculate_end_state(
    target: Point,
    target_heading: float,
    dss: float = config.DSS,
    turn_radius: float = config.R,
    alpha_max_rad: float = config.ALPHA_MAX_RAD,
) -> GoalState:
    """Compute the final waypoint ``W_{n-1}`` before terminal sensor lock.

    From the dynamics ``d_n = l_n + d_ss + R * tan(alpha_{n-1} / 2)`` with
    ``l_n = 0``, so ``d_n = d_ss + R * tan(alpha_{n-1} / 2)``.

    Args:
        target: Goal position ``T``.
        target_heading: Required final approach heading (rad).
        dss: Straight run-in distance for terminal camera sensor lock (m).
        turn_radius: Vehicle turn radius (m).
        alpha_max_rad: Maximum turn angle allowed (rad).

    Returns:
        The goal state: waypoint, heading, engagement distance and the distance
        back from the target.
    """
    distance_to_target = dss + turn_radius * math.tan(alpha_max_rad / 2)
    # Work backwards from the target along the approach heading.
    return {
        "waypoint": (
            target[0] - distance_to_target * math.cos(target_heading),
            target[1] - distance_to_target * math.sin(target_heading),
        ),
        "heading": target_heading,
        "engagement_distance": dss,
        "distance_to_target": distance_to_target,
    }


def compute_inflated_obstacles(
    obstacles: list[Obstacle], safe_margin: float = config.SAFE_MARGIN
) -> InflatedObstacleSets:
    """Inflate all obstacles and split them into the per-type sets the search uses.

    Args:
        obstacles: Raw obstacle records.
        safe_margin: The operator's minimum stand-off distance (m).

    Returns:
        The inflated obstacle list plus its circle and polygon views.
    """
    inflated = inflate_obstacles(obstacles, safe_margin)
    circle_obstacles: list[CircleGeometry] = []
    polygon_obstacles: list[PolygonCoords] = []
    for obstacle in inflated:
        if obstacle["type"] == "circle":
            circle_obstacles.append((obstacle["center"], obstacle["radius"]))
        else:
            polygon_obstacles.append(obstacle["polygon"])
    return {
        "inflated_obstacles": inflated,
        "circle_obstacles": circle_obstacles,
        "polygon_obstacles": polygon_obstacles,
    }


def prepare_scenario(
    scenario: Scenario,
    turn_radius: float = config.R,
    l0: float = config.L0,
    dss: float = config.DSS,
    safe_margin: float = config.SAFE_MARGIN,
    alpha_max_rad: float = config.ALPHA_MAX_RAD,
) -> PreprocessedScenario:
    """Prepare a scenario for the search: inflate obstacles and offset endpoints.

    Args:
        scenario: A scenario dict from :mod:`core.map_generator`.
        turn_radius: Vehicle turn radius (m).
        l0: Minimum straight distance for takeoff stabilisation (m).
        dss: Straight run-in distance for terminal sensor lock (m).
        safe_margin: Distance to expand obstacle boundaries by (m).
        alpha_max_rad: Maximum turn angle allowed (rad).

    Returns:
        The preprocessed scenario the planner consumes.
    """
    start_state = calculate_start_state(
        scenario["start"], scenario["start_heading"], l0, turn_radius, alpha_max_rad
    )
    goal_heading = scenario["goal_heading"]
    if goal_heading is None:
        # Free terminal approach direction: there is no fixed goal_heading to
        # offset W_{n-1} along, so the search targets T itself and the final
        # searched edge becomes the straight seeker run-in (>= DSS, any
        # direction). heading=None flags free mode for the planner.
        goal_state: GoalState = {
            "waypoint": scenario["goal"],
            "heading": None,
            "engagement_distance": dss,
            "distance_to_target": dss,
        }
    else:
        goal_state = calculate_end_state(
            scenario["goal"], goal_heading, dss, turn_radius, alpha_max_rad
        )

    inflated = compute_inflated_obstacles(scenario["obstacles"], safe_margin)

    # Raw (uninflated) obstacle sets, threaded through for callers that want to
    # measure or draw the true obstacle. They are NO LONGER the arc-clearance
    # reference: straight legs and turn arcs both clear the inflated set
    # (raw + SAFE_MARGIN) now that inflation carries no turn term - see
    # path_validation.path_is_valid.
    raw_circles: list[CircleGeometry] = [
        (o["center"], o["radius"])
        for o in scenario["obstacles"]
        if o["type"] == "circle"
    ]
    raw_polygons: list[PolygonCoords] = [
        o["polygon"] for o in scenario["obstacles"] if o["type"] == "polygon"
    ]

    return {
        "start_state": start_state,
        "goal_state": goal_state,
        "start_pos": scenario["start"],
        "goal_pos": scenario["goal"],
        "start_heading": scenario["start_heading"],
        "goal_heading": goal_heading,
        "turn_radius": turn_radius,
        "alpha_max_rad": alpha_max_rad,
        "safe_margin": safe_margin,
        "obstacles": inflated["inflated_obstacles"],
        "circle_obstacles": inflated["circle_obstacles"],
        "polygon_obstacles": inflated["polygon_obstacles"],
        "raw_circle_obstacles": raw_circles,
        "raw_polygon_obstacles": raw_polygons,
        "islands": scenario.get("islands", []),
        "dynamic_obstacles": scenario.get("dynamic_obstacles", []),
        # Per-scenario operating area / bounds. `safezones` is an optional list
        # of polygons (the aircraft must stay inside their union); `map_bounds`
        # is the legacy (width, height) rectangle. Both are threaded through so
        # the planner can constrain the search to them instead of the global
        # config.MAP_WIDTH/HEIGHT.
        "safezones": scenario.get("safezones"),
        "map_bounds": scenario.get("map_bounds"),
    }
