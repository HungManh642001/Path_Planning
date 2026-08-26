"""Tests for the per-scenario `safezones` operating-area constraint.

`safezones` is an optional LIST of polygons (each a list of (x, y)). The aircraft
must stay inside their UNION: this constrains both every generated waypoint
(`_in_bounds`) and every edge/chord (`_check_collision`). When absent the planner
falls back to the legacy config.MAP_WIDTH/HEIGHT rectangle.
"""

import math

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from path_planning import config
from path_planning.core import (
    kinodynamic_astar as astar,
    map_generator as mg,
    preprocessing as prep,
)
from path_planning.core.kinodynamic_astar import KinodynamicAstar


def _make_planner(
    safezones=None, start=(10.0, 10.0), goal=(90.0, 90.0), map_bounds=None
):
    """Construct a planner over an empty obstacle field with given safezones.

    Bypasses prepare_scenario so the unit tests can use small, hand-chosen
    polygons independent of R / L0 / DSS offsets.
    """
    preprocessed = {
        "start_state": {"waypoint": start, "heading": 0.0},
        "goal_state": {"waypoint": goal, "heading": 0.0},
        "start_pos": start,
        "goal_pos": goal,
        "start_heading": 0.0,
        "goal_heading": 0.0,
        "turn_radius": config.R,
        "alpha_max_rad": config.ALPHA_MAX_RAD,
        "safe_margin": config.SAFE_MARGIN,
        "obstacles": [],
        "circle_obstacles": [],
        "polygon_obstacles": [],
        "islands": [],
        "dynamic_obstacles": [],
        "safezones": safezones,
        "map_bounds": map_bounds,
    }
    return KinodynamicAstar(preprocessed)


# --------------------------------------------------------------------------- #
# _in_bounds
# --------------------------------------------------------------------------- #


def test_in_bounds_single_zone_inside_outside_and_boundary():
    square = [(0, 0), (100, 0), (100, 100), (0, 100)]
    planner = _make_planner(safezones=[square])

    assert planner._in_bounds((50, 50))  # interior
    assert not planner._in_bounds((150, 50))  # outside to the right
    assert not planner._in_bounds((-5, 50))  # outside to the left
    assert planner._in_bounds((0, 50))  # exactly on the boundary (covers)


def test_in_bounds_union_of_multiple_zones():
    # Two disjoint squares with a gap between x in (100, 200).
    left = [(0, 0), (100, 0), (100, 100), (0, 100)]
    right = [(200, 0), (300, 0), (300, 100), (200, 100)]
    planner = _make_planner(safezones=[left, right])

    assert planner._in_bounds((50, 50))  # inside left zone
    assert planner._in_bounds((250, 50))  # inside right zone
    assert not planner._in_bounds((150, 50))  # in the gap between zones


def test_in_bounds_rectangle_with_explicit_map_bounds():
    # An EXPLICIT map_bounds ⇒ the axis-aligned rectangle [0, w] x [0, h].
    planner = _make_planner(
        safezones=None, map_bounds=(config.MAP_WIDTH, config.MAP_HEIGHT)
    )
    assert planner._safezone is None
    assert planner._in_bounds((config.MAP_WIDTH / 2, config.MAP_HEIGHT / 2))
    assert not planner._in_bounds((config.MAP_WIDTH + 1, 10))
    assert not planner._in_bounds((-1, 10))


def test_in_bounds_permissive_without_any_operating_area():
    # No safezone AND no explicit map_bounds ⇒ permissive: the legacy 500 km
    # config default is not enforced (it is meaningless for a scenario that
    # lives elsewhere, e.g. a real mission at y ~ 1.15e6).
    planner = _make_planner(safezones=None, map_bounds=None)
    assert planner._safezone is None
    assert planner._has_explicit_bounds is False
    assert planner._in_bounds((config.MAP_WIDTH + 1, 10))
    assert planner._in_bounds((465395.9, 1151760.6))  # a real out-of-500km point
    assert planner._in_bounds((-1, -1))


# --------------------------------------------------------------------------- #
# _check_collision segment containment (the key non-convex / multi-zone cases)
# --------------------------------------------------------------------------- #


def test_check_collision_rejects_chord_leaving_nonconvex_zone():
    # U-shape opening upward: two prongs with an outside gap between x in (40,60).
    u_shape = [
        (0, 0),
        (100, 0),
        (100, 100),
        (60, 100),
        (60, 40),
        (40, 40),
        (40, 100),
        (0, 100),
    ]
    planner = _make_planner(safezones=[u_shape])

    # A chord across the gap between the prongs exits the operating area.
    assert not planner._check_collision((20, 80), (80, 80))
    # A chord that stays within the left prong is fine.
    assert planner._check_collision((10, 10), (30, 90))


def test_check_collision_rejects_chord_crossing_gap_between_zones():
    left = [(0, 0), (100, 0), (100, 100), (0, 100)]
    right = [(200, 0), (300, 0), (300, 100), (200, 100)]
    planner = _make_planner(safezones=[left, right])

    # A chord bridging the two disjoint zones passes through the outside gap.
    assert not planner._check_collision((50, 50), (250, 50))
    # A chord fully inside the right zone is fine.
    assert planner._check_collision((210, 10), (290, 90))


def test_check_collision_allows_interior_chord_without_safezones():
    planner = _make_planner(safezones=None)
    assert planner._check_collision((1000, 1000), (2000, 2000))


# --------------------------------------------------------------------------- #
# End-to-end: an open-water plan is confined to the safezones
# --------------------------------------------------------------------------- #


def _diagonal_band(start, goal, half_width):
    """Rotated-rectangle corridor of the given half-width around start->goal."""
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    n = math.hypot(dx, dy)
    px, py = -dy / n, dx / n  # unit perpendicular
    ox, oy = px * half_width, py * half_width
    return [
        (start[0] + ox, start[1] + oy),
        (start[0] - ox, start[1] - oy),
        (goal[0] - ox, goal[1] - oy),
        (goal[0] + ox, goal[1] + oy),
    ]


def test_open_scenario_path_stays_inside_safezones():
    scenario = mg.scenario1_open_ocean()
    band = _diagonal_band(scenario["start"], scenario["goal"], half_width=80000.0)
    scenario["safezones"] = [band]

    pre = prep.prepare_scenario(scenario)
    assert pre["safezones"] == [band]  # propagated through preprocessing

    result = astar.plan_trajectory(pre)
    assert result["success"], "open-water plan with a generous corridor should succeed"

    union = unary_union([Polygon(band)])
    for waypoint, _heading in result["path"]:
        assert union.covers(Point(waypoint)), f"waypoint {waypoint} left the safezones"


def test_backward_compat_open_scenario_without_safezones():
    scenario = mg.scenario1_open_ocean()
    assert scenario["safezones"] is None  # default

    pre = prep.prepare_scenario(scenario)
    result = astar.plan_trajectory(pre)
    assert result["success"]
    assert result["planner"]._safezone is None
