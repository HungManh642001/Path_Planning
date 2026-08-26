"""Shared type vocabulary for the planning pipeline.

Two dict shapes flow through every module here: the ``scenario`` produced by
:mod:`core.map_generator` and the ``preprocessed`` dict produced by
:func:`core.preprocessing.prepare_scenario`. They were previously untyped, so
every consumer re-guessed the key set and a typo produced a ``KeyError`` at
runtime rather than a diagnostic. Declaring them once as ``TypedDict`` keeps the
existing dict representation bit-for-bit (no construction or access path
changes) while making the contract checkable.

Units follow the pipeline convention throughout: distances in metres, angles in
radians.
"""

from __future__ import annotations

import sys
from typing import Literal, TypedDict

if sys.version_info >= (3, 11):
    from typing import NotRequired
else:
    from typing_extensions import NotRequired

# --- Primitive geometry -------------------------------------------------------

Point = tuple[float, float]
"""A planar position ``(x, y)`` in metres."""

PolygonCoords = list[Point]
"""An open polygon ring: vertices without the repeated closing point."""

CircleGeometry = tuple[Point, float]
"""A circle as ``(center, radius)``."""

MapBounds = tuple[float, float]
"""The legacy operating rectangle ``(width, height)`` anchored at the origin."""

PlannerState = tuple[Point, float]
"""A search state: ``(waypoint, heading)``. Paths are lists of these."""

LatticeKey = tuple[int, int, int]
"""A quantised state key used for search dedup; see ``state_to_tuple``."""

Topology = Literal["random", "center_cluster", "wall_block"]
"""Obstacle placement strategies understood by the generators."""

WrapSense = Literal[-1, 1]
"""Direction of travel along a circle boundary: +1 counter-clockwise, -1 clockwise."""

RidingSense = Literal[-1, 0, 1]
"""A :data:`WrapSense`, or 0 when the state does not ride the circle at all."""


# --- Obstacles ----------------------------------------------------------------


class CircleObstacle(TypedDict):
    """A circular (dynamic) obstacle."""

    type: Literal["circle"]
    center: Point
    radius: float


class PolygonObstacle(TypedDict):
    """A polygonal (island) obstacle."""

    type: Literal["polygon"]
    polygon: PolygonCoords


Obstacle = CircleObstacle | PolygonObstacle
"""Tagged union of obstacle records, discriminated on ``type``."""


# --- Scenario -----------------------------------------------------------------


class ScenarioConfig(TypedDict, total=False):
    """Caller-supplied recipe for :func:`core.map_generator.create_scenario`.

    Every key is optional at the type level because callers routinely omit the
    generator knobs, but ``start`` and ``goal`` are validated at runtime: the
    obstacle samplers place obstacles relative to the start-goal line and cannot
    run without them.
    """

    start: Point
    start_heading: float
    goal: Point
    goal_heading: float | None
    num_islands: int
    num_dynamic_obstacles: int
    map_bounds: MapBounds
    safezones: list[PolygonCoords] | None
    topology: Topology
    seed: int | None


class Scenario(TypedDict):
    """A generated mission: endpoints plus the obstacle field.

    ``islands`` and ``dynamic_obstacles`` are the raw per-type views;
    ``obstacles`` is the unified tagged-union list the rest of the pipeline
    consumes. A ``goal_heading`` of ``None`` selects free-goal mode, in which
    the planner chooses the terminal approach direction.
    """

    start: Point
    start_heading: float
    goal: Point
    goal_heading: float | None
    map_bounds: MapBounds
    safezones: list[PolygonCoords] | None
    islands: list[PolygonCoords]
    dynamic_obstacles: list[CircleGeometry]
    obstacles: list[Obstacle]


# --- Search results -----------------------------------------------------------


class SearchStats(TypedDict):
    """Counters describing how a single search ran.

    Attributes:
        iterations: Nodes popped from the open set.
        time_budget_s: The wall-clock budget the search actually ran under -
            the caller's value, or ``config.TIME_BUDGET_S`` when none was
            given. It is the search's only stop condition.
        budget_bound: Whether the search was cut off by that budget. A
            first-class field, not a detail: a budget-bound search is one whose
            answer depends on the machine it ran on, and "no path" and "ran out
            of clock" are different claims.
        open_set_size: Nodes still queued when the search ended.
        search_failed: Whether the search ended without reaching the goal.
        closed_set_size: Distinct lattice cells expanded. Reported by the main
            planner only; the v0 planner omits it.
    """

    iterations: int
    time_budget_s: float
    budget_bound: bool
    open_set_size: int
    search_failed: bool
    closed_set_size: NotRequired[int]


class PlanResultView(TypedDict):
    """The part of a planner result that consumers outside the planner read.

    Both planners return a richer dict that also carries the planner instance;
    this view is what the renderer, the GUI and the reporting scripts actually
    need, and it keeps them from importing a planner module just for a type.

    Attributes:
        path: The planned interior waypoints, or ``None`` if planning failed.
        success: Whether the independent oracle accepted the full mission path.
        failure_reason: ``None`` on success, otherwise why planning failed.
        stats: Search counters.
    """

    path: list[PlannerState] | None
    success: bool
    failure_reason: str | None
    stats: SearchStats


# --- Preprocessed scenario ----------------------------------------------------


class StartState(TypedDict):
    """The first searched waypoint ``W_1``, offset from takeoff point ``O``."""

    waypoint: Point
    heading: float
    straight_length: NotRequired[float]
    distance_from_origin: NotRequired[float]


class GoalState(TypedDict):
    """The last searched waypoint ``W_{n-1}``, offset back from target ``T``.

    In free-goal mode ``heading`` is ``None`` and ``waypoint`` is ``T`` itself:
    there is no fixed approach direction to offset along, so the final searched
    edge becomes the seeker run-in.
    """

    waypoint: Point
    heading: float | None
    engagement_distance: NotRequired[float]
    distance_to_target: NotRequired[float]


class InflatedObstacleSets(TypedDict):
    """Obstacles inflated by the stand-off margin, split by type for the search."""

    inflated_obstacles: list[Obstacle]
    circle_obstacles: list[CircleGeometry]
    polygon_obstacles: list[PolygonCoords]


class PreprocessedScenario(TypedDict):
    """A scenario prepared for the search: inflated obstacles and offset endpoints.

    ``circle_obstacles`` / ``polygon_obstacles`` carry the stand-off margin and
    are what the planner collides against. The ``raw_*`` counterparts are the
    true obstacles, threaded through for measurement and drawing only.

    Only the keys the search reads unconditionally are required. The rest are
    ``NotRequired`` because callers legitimately hand-build a partial dict --
    unit tests bypass :func:`core.preprocessing.prepare_scenario` to work with
    small hand-chosen geometry -- and the consumers already read them through
    ``.get``. Marking them required would make the type checker bless code that
    raises ``KeyError`` on those inputs.
    """

    start_state: StartState
    goal_state: GoalState
    turn_radius: float
    alpha_max_rad: float
    circle_obstacles: list[CircleGeometry]
    polygon_obstacles: list[PolygonCoords]
    safezones: list[PolygonCoords] | None
    map_bounds: MapBounds | None
    start_pos: NotRequired[Point]
    goal_pos: NotRequired[Point]
    start_heading: NotRequired[float]
    goal_heading: NotRequired[float | None]
    safe_margin: NotRequired[float]
    obstacles: NotRequired[list[Obstacle]]
    raw_circle_obstacles: NotRequired[list[CircleGeometry]]
    raw_polygon_obstacles: NotRequired[list[PolygonCoords]]
    islands: NotRequired[list[PolygonCoords]]
    dynamic_obstacles: NotRequired[list[CircleGeometry]]
