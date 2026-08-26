"""Kinodynamic A* path planning (readability-first variant, and the one that ships).

Searches over ``(waypoint, heading)`` states under the vehicle's turn-angle and
minimum-straight (đoản-trình) constraints, then shortens the result with an
exact subsequence DP. Distances are metres, angles radians.

This began as a readability-first sibling of ``core/kinodynamic_astar.py`` --
simpler ``WRAP_STEP_M`` straight continuation instead of arc-hop successors, and
shared helpers instead of the main file's hand-inlined hot loops. It is also the
planner ``batch_random_test`` actually imports, so when the two files conflict
this one is the standard and the main file follows it.
"""

from __future__ import annotations

import heapq
import math
import time
from collections import defaultdict
from typing import TYPE_CHECKING, NamedTuple, TypedDict

from shapely.geometry import LineString, Polygon
from shapely.geometry import Point as ShapelyPoint
from shapely.ops import unary_union
from shapely.prepared import prep as shp_prep

from path_planning import config
from path_planning.core import goal_shot as gshot
from path_planning.core import mission as mission
from path_planning.core import path_validation as pv
from path_planning.core import spatial_utils as su

if TYPE_CHECKING:
    from path_planning.core.types import (
        LatticeKey,
        PlannerState,
        Point,
        PreprocessedScenario,
        SearchStats,
    )

# Hot-path alias of su.angle_diff (module-global lookup, called ~10x per
# candidate). Not a second definition -- see the note in spatial_utils.
_angle_diff = su.angle_diff

# Squared candidate-distance floor, compared against dx*dx + dy*dy (squared
# because the quantity is -- see config.CANDIDATE_MIN_DIST_M).
_CAND_MIN_D2 = max(config.CANDIDATE_MIN_DIST_M, config.GEOM_EPS_M) ** 2


class PlanResult(TypedDict):
    """The outcome of :func:`plan_trajectory`.

    Attributes:
        path: The planned interior waypoints, or ``None`` if planning failed.
        success: True only when a path was found AND the independent oracle
            accepted the full mission path.
        failure_reason: ``None`` on success, otherwise ``'no_path'``,
            ``'start_leg_blocked'``, ``'goal_leg_blocked'`` or the oracle's own
            rejection detail.
        stats: Search counters.
        planner: The planner instance, for callers that want to inspect it.
    """

    path: list[PlannerState] | None
    success: bool
    failure_reason: str | None
    stats: SearchStats
    planner: KinodynamicAstar


class _DpEntry(NamedTuple):
    """One frontier entry of the smoothing DP.

    Attributes:
        budget: Straight length left on the chord into the current waypoint,
            after that chord's NEAR fillet reserve.
        cost: Path length so far plus the per-waypoint penalty.
        prev_key: The ``(u, v)`` state this entry was reached from, or ``None``
            for a first chord leaving the takeoff point.
        prev_entry: The entry within ``prev_key`` this one extends.
    """

    budget: float
    cost: float
    prev_key: tuple[int, int] | None
    prev_entry: _DpEntry | None


class State:
    """One search node: a waypoint plus the heading the vehicle holds there.

    Attributes:
        waypoint: Position ``(x, y)``.
        heading: Heading in radians, or ``None`` for the free-goal TARGET, which
            carries no arrival heading. A state with ``heading is None`` is only
            ever a search target and is never expanded, hashed or flown.
        cos_h: Cached ``cos(heading)``; ``None`` exactly when ``heading`` is.
        sin_h: Cached ``sin(heading)``; ``None`` exactly when ``heading`` is.
        parent: The state this one was reached from.
        g_cost: Cost from the start.
        h_cost: Heuristic estimate to the goal.
        straight_budget: Straight length left on the incoming leg after its near
            fillet reserve.
        min_straight_in: Đoản-trình floor the incoming leg must still satisfy.
        is_start_corner: Whether this is a seeded takeoff corner.
        via: ``(pivot, heading)`` of an intermediate straight-through waypoint on
            the incoming edge when this state was reached by a pivot slide: the
            vehicle flies straight through the parent and turns only at ``pivot``.
    """

    def __init__(self, waypoint: Point, heading: float | None) -> None:
        """Initialise a state at ``waypoint`` holding ``heading``."""
        self.waypoint: Point = waypoint
        self.heading: float | None = heading
        # Unit vector of `heading`, cached because _pivot_candidate needs it for
        # EVERY candidate (~120 per expansion) and heading never changes.
        self.cos_h: float | None = math.cos(heading) if heading is not None else None
        self.sin_h: float | None = math.sin(heading) if heading is not None else None
        self.parent: State | None = None
        self.g_cost: float = float("inf")
        self.h_cost: float = 0.0
        self.straight_budget: float = float("inf")
        self.min_straight_in: float = config.MIN_STRAIGHT_M
        self.is_start_corner: bool = False
        self.via: tuple[Point, float] | None = None
        # Dedup key cache. waypoint/heading never change after construction, and
        # the search hashes/compares each state hundreds of times (measured:
        # 768k state_to_tuple calls, 7.4% of runtime, over 20 scenarios).
        # Computed LAZILY on first hash/eq, because a free-goal goal_state
        # carries heading=None and must stay constructible — it is a target,
        # never hashed.
        self._key: LatticeKey | None = None

    def _compute_key(self) -> LatticeKey:
        """Quantise this state onto the search lattice.

        Raises:
            TypeError: If this is a headingless target, which is never hashed.
        """
        if self.heading is None:
            raise TypeError("a headingless goal target has no lattice key")
        return su.state_to_tuple(self.waypoint, self.heading)

    def __hash__(self) -> int:
        """Hash on the quantised search lattice, caching the key on first use."""
        key = self._key
        if key is None:
            key = self._key = self._compute_key()
        return hash(key)

    def __eq__(self, other: object) -> bool:
        """Compare on the quantised search lattice.

        Kept inline rather than delegating: the search compares states millions
        of times, so this is the hottest method in the planner.
        """
        if not isinstance(other, State):
            return NotImplemented
        key = self._key
        if key is None:
            key = self._key = self._compute_key()
        other_key = other._key
        if other_key is None:
            other_key = other._key = other._compute_key()
        return key == other_key

    def __lt__(self, other: State) -> bool:
        """Order by f = g + w*h, so the heap pops the most promising state."""
        return (self.g_cost + config.HEURISTIC_WEIGHT * self.h_cost) < (
            other.g_cost + config.HEURISTIC_WEIGHT * other.h_cost
        )

    def __repr__(self) -> str:
        """Return a compact debug representation with the heading in degrees."""
        heading = "none" if self.heading is None else f"{math.degrees(self.heading):.1f}°"
        return f"State(wp={self.waypoint}, h={heading})"


class KinodynamicAstar:
    """Kinodynamic A* planner over ``(waypoint, heading)`` states.

    Successors are generated dynamically rather than from a precomputed graph:
    tangent points to inflated circles, lifted polygon hull vertices and the
    goal (Strategy A), plus a bounded radial fan as an escape valve
    (Strategy B).
    """

    def __init__(
        self,
        preprocessed_scenario: PreprocessedScenario,
        time_budget_s: float | None = None,
    ) -> None:
        """Build the planner's obstacle caches, endpoints and seeded start corners.

        Args:
            preprocessed_scenario: Output of
                :func:`core.preprocessing.prepare_scenario`.
            time_budget_s: Wall-clock budget for the search, in seconds. This
                is the search's ONLY stop condition. ``None`` means "the caller
                did not say", and falls back to ``config.TIME_BUDGET_S``; it
                does NOT mean unlimited, which is not an option any more.

        Raises:
            ValueError: If the resolved budget is not a finite number > 0, or
                if the scenario is missing ``start_pos`` / ``goal_pos``.
        """
        self.scenario = preprocessed_scenario
        self.time_budget_s = config.resolve_time_budget_s(time_budget_s)
        # start_pos, goal_pos and the two mandatory leg lengths are optional at
        # the TYPE level because render and GUI callers legitimately hand
        # full_mission_path a partial mapping. A planner, however, cannot run
        # without them, so they are resolved once here: a missing key becomes a
        # clear error at construction instead of a KeyError from somewhere deep
        # inside the search.
        origin = preprocessed_scenario.get("start_pos")
        target = preprocessed_scenario.get("goal_pos")
        if origin is None or target is None:
            raise ValueError(
                "preprocessed scenario needs both 'start_pos' and 'goal_pos'; "
                "build it with core.preprocessing.prepare_scenario"
            )
        self._origin: Point = origin
        self._target: Point = target
        self._l0 = preprocessed_scenario["start_state"].get("straight_length", config.L0)
        # Lift applied when CONSTRUCTING geometry, so a tangent chord is
        # strictly clear of the exact-checked boundary instead of landing on it.
        # Two separate reasons, added not merged: CONSTRUCTION_CLEARANCE_M is an
        # operational stand-off (may be 0), GEOM_EPS_M is the rounding guard
        # (never 0). Without it 43% of tangents fall inside the circle by ~1e-11 m
        # and are rejected by their own collision test.
        self._construct_delta = config.CONSTRUCTION_CLEARANCE_M + config.GEOM_EPS_M

        self._polygons = [Polygon(coords) for coords in preprocessed_scenario["polygon_obstacles"]]
        # Plain-float bboxes so a chord/arc can be rejected against an obstacle
        # without building any geometry. Measured over 40 scenarios: 82% of the
        # circle tests in _check_collision and 97.6% of those in
        # _corner_arc_clear are against an obstacle that cannot reach the query
        # at all, and they were costing a full point-to-segment distance each.
        self._circles: list[tuple[float, float, float]] = [
            (c[0], c[1], r) for c, r in preprocessed_scenario["circle_obstacles"]
        ]
        self._poly_bboxes: list[tuple[float, float, float, float]] = [
            p.bounds for p in self._polygons
        ]
        # Vertex candidates are LIFTED off the hull by the same
        # _construct_delta that circle tangent points are built on. Without it
        # polygons were the one obstacle type whose navigation targets sat
        # EXACTLY on the boundary they must clear, which is what put the
        # boundary case in front of shapely on every chord that ends at, passes
        # through, or runs along a hull edge. A mitre buffer offsets every edge
        # perpendicular by delta and keeps the corner count.
        self._poly_vertices: list[Point] = []
        for poly in self._polygons:
            hull = poly.convex_hull.buffer(self._construct_delta, join_style="mitre")
            self._poly_vertices.extend((float(x), float(y)) for x, y in hull.exterior.coords[:-1])

        safezones = preprocessed_scenario.get("safezones")
        self._safezone = unary_union([Polygon(sz) for sz in safezones]) if safezones else None
        self._safezone_prep = shp_prep(self._safezone) if self._safezone is not None else None
        map_bounds = preprocessed_scenario.get("map_bounds")
        self._has_explicit_bounds = map_bounds is not None
        self._bounds_w, self._bounds_h = (
            map_bounds if map_bounds else (config.MAP_WIDTH, config.MAP_HEIGHT)
        )

        self.start_state = State(
            preprocessed_scenario["start_state"]["waypoint"],
            preprocessed_scenario["start_state"]["heading"],
        )
        self.start_state.g_cost = 0
        self.start_state.straight_budget = math.dist(origin, self.start_state.waypoint)

        self.goal_state = State(
            preprocessed_scenario["goal_state"]["waypoint"],
            preprocessed_scenario["goal_state"]["heading"],
        )

        self._free_goal = preprocessed_scenario.get("goal_heading") is None
        self._dss = preprocessed_scenario["goal_state"].get("engagement_distance", config.DSS)

        # Search variables.
        self.open_set: list[tuple[float, int, State]] = []
        self.closed_set: set[State] = set()
        # NOTE: deliberately no came_from dict. State hashing quantises to a
        # coarse lattice, so a lattice-keyed parent map lets two distinct
        # candidates collide and splice the reconstruction onto a parent whose
        # transition was never collision-checked. Parents live on the State
        # object instead. (The dict was still being allocated here, unread,
        # long after the search stopped using it.)
        self.g_scores: defaultdict[State, float] = defaultdict(lambda: float("inf"))

        self.iteration_count = 0
        self.nodes_expanded = 0
        self.nodes_generated = 0
        # Set by search() when the wall-clock budget, not the frontier, ended
        # the loop. Kept apart from search_failed: "I looked everywhere and
        # there is no path" and "I ran out of clock" are different answers.
        self.budget_bound = False
        self.R = preprocessed_scenario["turn_radius"]
        self.alpha_max_rad = preprocessed_scenario["alpha_max_rad"]
        # Turn limit used when BUILDING and accepting geometry. Padded towards
        # feasibility (i.e. SUBTRACTED) so a corner constructed hard against the
        # limit still reads as legal when the oracle recomputes the angle from
        # waypoint geometry — measured, that recomputation overshoots by up to
        # 1.1e-15 rad, which an exact oracle would reject.
        self._alpha_build = self.alpha_max_rad - config.GEOM_EPS_RAD
        # Cosine of the widest turn the cheap prefilter may reject outright.
        self._turn_cos_guard = math.cos(
            min(math.pi, self._alpha_build + config.TURN_PREFILTER_BAND_RAD)
        )

        self.start_corners: list[State] = self._seed_start_corners()

        # Fan distance rungs: rung j is the shortest leg still affording a next
        # turn beta_j, in tan-uniform capability buckets.
        num_rungs = max(1, int(config.NUM_FAN_DISTANCES))
        tan_half_max = math.tan(self._alpha_build / 2)
        self._fan_rungs = [
            self.R * (j / num_rungs) * tan_half_max + config.RADIAL_FAN_STEP_M
            for j in range(1, num_rungs + 1)
        ]

        # Which gate rejected the most recent _pivot_candidate (None on
        # success); lets the caller retry only the ones worth sliding.
        self._last_reject: str | None = None

        self.search_failed = False

        # Search route before arc expansion/smoothing (set on success);
        # used to verify discretisation invariance.
        self.raw_route: list[PlannerState] | None = None

        self.num_strategy_b = config.NUM_STRATEGY_B

        # Is this mission's approach REVERSED? Resolved once, because it is a
        # property of the MISSION and not of any state: the angle between
        # goal_heading and the start -> goal bearing. Below alpha_max a straight
        # run at the goal can still turn onto goal_heading in one corner, which
        # the ordinary Strategy-A goal candidate already builds, so the shot is
        # pure overhead there; above it one corner cannot and the shot's two are
        # the only way to finish. See the config note for the premium.
        self._shot_armed = False
        if not self._free_goal:
            goal_heading = self.goal_state.heading
            if goal_heading is not None:
                travel = su.angle_to_heading(self._origin, self._target)
                reversal = abs(_angle_diff(goal_heading, travel))
                self._shot_armed = reversal >= config.deg_to_rad(config.GOAL_SHOT_MIN_REVERSAL_DEG)

    def _seed_start_corners(self) -> list[State]:
        """Seed the takeoff corners the search is rooted at.

        Instead of one worst-case ``W_1``, ``NUM_START_CORNERS`` corners are
        placed on the takeoff ray at ``d_i = L0 + R*tan(alpha_i/2)`` in
        tan-uniform buckets. A corner seeded for ``alpha_i`` affords any first
        turn ``alpha <= alpha_i`` while keeping the takeoff straight
        ``l_1 >= L0`` exactly. Corners outside the operating area, or whose
        ``O -> corner`` leg collides, are dropped -- the single legacy corner
        could land inside an inflated obstacle and kill the whole plan.

        Returns:
            The feasible corners; empty means the start is blocked.
        """
        origin = self._origin
        takeoff_heading = self.scenario["start_state"]["heading"]
        l0 = self._l0
        num_corners = max(1, int(config.NUM_START_CORNERS))
        tan_max = math.tan(self._alpha_build / 2.0)

        corners: list[State] = []
        for i in range(1, num_corners + 1):
            # +GEOM_EPS_M: build l1 strictly longer than L0 so the oracle's
            # exact `l1 >= L0` test survives its own recomputation.
            distance = l0 + config.GEOM_EPS_M + self.R * (i / num_corners) * tan_max
            corner = (
                origin[0] + distance * math.cos(takeoff_heading),
                origin[1] + distance * math.sin(takeoff_heading),
            )
            if not self._in_bounds(corner):
                continue
            if not self._check_collision(origin, corner):
                continue
            state = State(corner, takeoff_heading)
            state.g_cost = distance
            state.straight_budget = distance
            state.min_straight_in = l0
            state.is_start_corner = True
            corners.append(state)
        return corners

    def heuristic(self, state: State, goal_state: State) -> float:
        """Estimate the remaining cost from ``state`` to ``goal_state``.

        Args:
            state: The state being scored.
            goal_state: The search target.

        Returns:
            The straight-line distance in metres, which never overestimates.
        """
        dx = goal_state.waypoint[0] - state.waypoint[0]
        dy = goal_state.waypoint[1] - state.waypoint[1]
        return math.sqrt(dx * dx + dy * dy)

    def _doan_trinh(
        self,
        current: State,
        seg_len: float,
        turn_at_current: float,
        far_reserve: float = 0.0,
        advance: float = 0.0,
    ) -> float | None:
        """Apply the exact đoản-trình (minimum-straight) check to one edge.

        The check is split across the two events its two turns become known:
        the deferred far end of the incoming leg, and the near end of the new one.

        Args:
            current: The state being expanded.
            seg_len: Length of the new leg (m).
            turn_at_current: Turn angle at ``current`` (rad).
            far_reserve: Fillet reserve already known at the far end (m).
            advance: Pivot slide distance; the vehicle flies straight THROUGH
                ``current`` for this much further before turning, so the incoming
                run is that much longer -- sliding can only ADD budget.

        Returns:
            The straight budget left on the new leg, or ``None`` if the edge
            violates the constraint.
        """
        reserve = self.R * math.tan(turn_at_current / 2.0)
        # Deferred far-end check of `current`'s incoming segment.
        if current.straight_budget + advance - reserve < current.min_straight_in:
            return None
        budget = seg_len - reserve
        if budget - far_reserve < config.MIN_STRAIGHT_M:
            return None
        return budget

    def get_next_states(self, current_state: State) -> list[tuple[State, float]]:
        """Generate the successors of a state, with their transition costs.

        Three sources, in order: a straight wrap step off a circle boundary;
        Strategy-A tangent/vertex/goal candidates; and the Strategy-B radial fan.

        ``config.NUM_STRATEGY_B`` budgets only ONE of the fan's firing
        conditions — an occluded goal from a non-start-corner state that already
        has successors. Start-corner, goal-clear and no-successor firings bypass
        it entirely (89% of all firings, measured over 100 free seeds), and it
        is a single counter for the whole search, not a per-path one.

        Args:
            current_state: The state being expanded.

        Returns:
            ``(successor, transition_cost)`` pairs.

        Raises:
            TypeError: If ``current_state`` carries no heading, which means a
                goal TARGET was mistakenly expanded.
        """
        heading = current_state.heading
        if heading is None:
            raise TypeError("cannot expand a headingless goal target")

        successors: list[tuple[State, float]] = []
        position = current_state.waypoint

        # --- Wrap step: straight continuation off a circle boundary ---
        if self._on_circle_boundary(position):
            forward = (
                position[0] + config.WRAP_STEP_M * math.cos(heading),
                position[1] + config.WRAP_STEP_M * math.sin(heading),
            )
            if self._in_bounds(forward) and self._check_collision(position, forward):
                successors.append((State(forward, heading), config.WRAP_STEP_M))

        # --- Strategy A: dynamic tangent / vertex / goal candidates ---
        goal_wp = self.goal_state.waypoint
        candidates: list[Point] = []
        for center, radius in self.scenario["circle_obstacles"]:
            candidates.extend(
                su.circle_tangent_points(position, center, radius + self._construct_delta)
            )
        candidates.extend(self._poly_vertices)
        candidates.append(goal_wp)

        # Which gate turned the GOAL candidate away, if any. Recorded here rather
        # than read off `self._last_reject` after the loop: the goal happens to
        # be the last candidate today, and a gate that depends on that ordering
        # would break silently the day it is not.
        goal_reject: str | None = None
        for node in candidates:
            dx = node[0] - position[0]
            dy = node[1] - position[1]
            if dx * dx + dy * dy < _CAND_MIN_D2:
                continue
            result = self._pivot_candidate(current_state, node, 0.0)
            if result is None and config.NUM_PIVOT_SLIDES > 0 and self._last_reject == "arc":
                # Only an ARC rejection is worth retrying: sliding forward can
                # only INCREASE the turn (so a candidate already over alpha_max
                # is hopeless), and a blocked chord is almost never unblocked
                # by moving the pivot.
                result = self._slide_pivot(current_state, node)
            if result is not None:
                successors.append(result)
            elif node is goal_wp:
                goal_reject = self._last_reject

        if (
            successors
            and not self._check_collision(position, goal_wp)
            and not current_state.is_start_corner
        ):
            if self.num_strategy_b <= 0:
                return successors
            self.num_strategy_b -= 1
        elif (
            config.FAN_SKIP_ON_SHORT_RUNIN
            and self._free_goal
            and goal_reject == "goal"
            and successors
            and not current_state.is_start_corner
        ):
            # FREE-goal mode only: the goal is in the clear, and the only thing
            # wrong with flying straight at it is that the leg cannot supply the
            # d_ss run-in. The fan cannot fix that -- its legs leave at
            # +-alpha_max or straight ahead, at fixed rungs, never aimed at the
            # goal -- so it just floods the lattice near the target. Measured
            # over 300 free-goal seeds: 1,108 firings, ZERO waypoints on any
            # delivered route.
            #
            # Scoped to free goals ON PURPOSE. In FIXED-goal mode this same
            # rejection means "cannot turn onto goal_heading", which is a
            # different and much harder problem: it is 43.6% of all firings
            # there and DOES carry the route (143 waypoints). The fan is a poor
            # tool for it, but it is the only tool v0 has until the analytic
            # goal shot is ported -- dropping it there costs +0.426% with one
            # seed at +40%.
            return successors

        # --- Strategy B: radial fan, an escape valve against long detours ---
        num_directions = config.RADIAL_FAN_DIRECTIONS
        for i in range(num_directions):
            heading_offset = -self._alpha_build + 2 * self._alpha_build * i / (num_directions - 1)
            next_heading = heading + heading_offset
            near_reserve = math.tan(abs(heading_offset) / 2.0) * self.R
            turn = abs(_angle_diff(next_heading, heading))
            cos_next = math.cos(next_heading)
            sin_next = math.sin(next_heading)
            for rung in self._fan_rungs:
                distance_next = near_reserve + rung
                next_waypoint = (
                    position[0] + distance_next * cos_next,
                    position[1] + distance_next * sin_next,
                )
                # Cheapest gate first. đoản trình is pure arithmetic and
                # rejects ~31% of the legs that used to reach the fillet-arc
                # gate — the most expensive check in the planner — after paying
                # for it. Strategy A already orders it this way.
                budget = self._doan_trinh(current_state, distance_next, turn)
                if budget is None:
                    continue
                if not self._in_bounds(next_waypoint):
                    continue
                if not self._check_collision(position, next_waypoint):
                    continue
                # A fan leg turns at P just like a Strategy-A corner does, so its
                # fillet needs the same gate. Skipping it here is invisible on
                # sparse maps and costs 11/1000 missions on dense ones.
                if config.ARC_CLEARANCE_CHECK and not self._corner_arc_clear(
                    heading, position, next_waypoint
                ):
                    continue
                successor = State(next_waypoint, next_heading)
                successor.straight_budget = budget
                successors.append((successor, distance_next + config.TURN_PENALTY_WEIGHT * turn))
        return successors

    def _pivot_candidate(
        self, current: State, node: Point, advance: float
    ) -> tuple[State, float] | None:
        """Build one Strategy-A edge, turning ``advance`` m along the incoming ray.

        ``advance = 0`` is the plain corner and is behaviour-identical to the
        pre-slide code.

        Args:
            current: The state being expanded.
            node: The candidate waypoint to turn toward.
            advance: How far past ``current`` to slide the pivot (m).

        Returns:
            The ``(successor, cost)`` pair, or ``None`` if any gate rejects the
            edge -- in which case the rejecting gate is recorded in
            ``self._last_reject``.

        Raises:
            TypeError: If ``current`` carries no heading.
        """
        position = current.waypoint
        heading = current.heading
        ux = current.cos_h
        uy = current.sin_h
        if heading is None or ux is None or uy is None:
            raise TypeError("cannot expand a headingless goal target")

        pivot = (
            (position[0] + advance * ux, position[1] + advance * uy) if advance > 0.0 else position
        )
        dx = node[0] - pivot[0]
        dy = node[1] - pivot[1]
        seg_len = math.hypot(dx, dy)
        # 55% of candidates die on the turn limit, and the exact test costs two
        # atan2 plus a sin and a cos to find that out. cos(turn) = dot / seg_len
        # needs one multiply-add, so reject here anything over the limit by more
        # than TURN_PREFILTER_BAND_RAD. Deliberately conservative: a candidate
        # anywhere near the limit still gets the exact test below, so this can
        # never decide a borderline case (see the config note).
        if dx * ux + dy * uy < self._turn_cos_guard * seg_len:
            self._last_reject = "turn"
            return None
        heading_to_node = su.angle_to_heading(pivot, node)
        turn = abs(_angle_diff(heading_to_node, heading))
        if turn > self._alpha_build:
            self._last_reject = "turn"
            return None

        # Far-end reserve: 0 for an interior waypoint (its turn is unknown here);
        # at the goal the terminal turn onto goal_heading is known, so reserve it.
        far_reserve = 0.0
        if node is self.goal_state.waypoint:
            if self._free_goal:
                if seg_len - self.R * math.tan(turn / 2.0) < self._dss:
                    self._last_reject = "goal"
                    return None
            else:
                goal_heading = self.goal_state.heading
                if goal_heading is None:
                    raise TypeError("fixed-goal mode requires a goal heading")
                final_turn = abs(_angle_diff(goal_heading, heading_to_node))
                if final_turn > self._alpha_build:
                    self._last_reject = "goal"
                    return None
                far_reserve = self.R * math.tan(final_turn / 2.0)

        budget = self._doan_trinh(current, seg_len, turn, far_reserve, advance)
        if budget is None:
            self._last_reject = "doan_trinh"
            return None
        if advance > 0.0:
            # The slide is new flying: the extension leg must be clear and stay
            # inside the operating area.
            if not self._in_bounds(pivot):
                self._last_reject = "bounds"
                return None
            if not self._check_collision(position, pivot):
                self._last_reject = "ext_leg"
                return None
        if not self._check_collision(pivot, node):
            self._last_reject = "los"
            return None
        if config.ARC_CLEARANCE_CHECK and not self._corner_arc_clear(heading, pivot, node):
            self._last_reject = "arc"
            return None

        self._last_reject = None
        successor = State(node, heading_to_node)
        successor.straight_budget = budget
        if advance > 0.0:
            # Stored with the INCOMING heading: the vehicle reaches the pivot
            # still on `heading` (it flew straight through P) and turns only there.
            successor.via = (pivot, heading)
        return successor, advance + seg_len + config.TURN_PENALTY_WEIGHT * turn

    def _slide_pivot(self, current: State, node: Point) -> tuple[State, float] | None:
        """Retry an arc-rejected candidate from pivots slid FORWARD along the ray.

        ``P' = P + d*h_in`` keeps the incoming DIRECTION, so the parent's corner,
        its turn reserve and every ancestor stay valid and ``_doan_trinh`` only
        gains budget. Sliding along the outer bisector instead would rotate the
        incoming leg and force ancestors to be re-validated, which does not
        terminate. The resulting turn ``|atan2(b, a - d)|`` grows with ``d``
        (cap ``d <= a - b/tan(alpha_max)``), so retry points are tan-uniform
        buckets of that turn, smallest slide first.

        Args:
            current: The state being expanded.
            node: The candidate waypoint the plain corner failed to reach.

        Returns:
            The first feasible ``(successor, cost)`` pair, or ``None``.

        Raises:
            TypeError: If ``current`` carries no heading.
        """
        position = current.waypoint
        heading = current.heading
        if heading is None:
            raise TypeError("cannot expand a headingless goal target")
        ux, uy = math.cos(heading), math.sin(heading)
        vx = node[0] - position[0]
        vy = node[1] - position[1]
        along = vx * ux + vy * uy  # along-track component of V - P
        if along <= 0.0:  # abeam or behind: sliding only hurts
            return None
        cross = abs(vx * -uy + vy * ux)  # cross-track component
        if cross < config.GEOM_EPS_M:  # collinear: there is no corner
            return None

        turn_without_slide = math.atan2(cross, along)
        num_slides = int(config.NUM_PIVOT_SLIDES)
        tan_half_max = math.tan(self._alpha_build / 2.0)
        for i in range(1, num_slides + 1):
            turn_i = 2.0 * math.atan((i / num_slides) * tan_half_max)
            if turn_i <= turn_without_slide:
                continue  # this bucket is behind us (d <= 0)
            if turn_i >= math.pi / 2.0 - config.GEOM_EPS_RAD:
                slide = along  # the perpendicular foot
            else:
                slide = along - cross / math.tan(turn_i)
            if slide <= config.MIN_PIVOT_SLIDE_M:
                continue
            result = self._pivot_candidate(current, node, slide)
            if result is not None:
                return result
        return None

    def _corner_arc_clear(self, h_in: float, w: Point, w_next: Point) -> bool:
        """Test whether the radius-R fillet arc rounding corner ``w`` is clear.

        Uses the oracle's own arc GEOMETRY so the search weighs the same arc the
        final validation will. The whole arc is tested as ONE polyline rather
        than segment by segment: the shapely work (LineString, tree query,
        safezone ``covers``) is what costs, and doing it per sub-segment made
        this gate 41% of run time.

        A polygon hit is taken at face value from ``'T********'``. That predicate
        also fires on an arc merely GRAZING a hull edge, which the oracle would
        forgive -- so this gate is fractionally stricter than the validator, in
        the safe direction: it can only ever decline a candidate, never approve
        one the oracle rejects. Resolving the difference needs
        ``pv.interior_overlap_length``, and lifting the polygon vertex candidates
        (see ``_poly_vertices``) removed every occurrence it had to resolve.

        Args:
            h_in: Incoming heading at the corner (rad).
            w: The corner waypoint.
            w_next: The waypoint the outgoing leg heads for.

        Returns:
            ``True`` if the arc clears every obstacle and stays in the operating
            area. A collinear corner has no arc and is trivially clear.
        """
        prev = (w[0] - math.cos(h_in), w[1] - math.sin(h_in))
        pts = pv.arc_points(prev, w, w_next, self.R, n=config.ARC_CHECK_SAMPLES)
        if not pts:
            return True

        ax0 = min(p[0] for p in pts)
        ax1 = max(p[0] for p in pts)
        ay0 = min(p[1] for p in pts)
        ay1 = max(p[1] for p in pts)

        # Circles: scalar point-to-segment, no geometry objects built, and only
        # for a circle whose bbox can reach the arc's. Without the prefilter
        # every arc paid ARC_CHECK_SAMPLES-1 distances against EVERY circle;
        # 97.6% of those pairs cannot touch (measured over 40 scenarios).
        for cx, cy, radius in self._circles:
            if cx + radius < ax0 or cx - radius > ax1 or cy + radius < ay0 or cy - radius > ay1:
                continue
            center = (cx, cy)
            for j in range(len(pts) - 1):
                if su.point_to_line_distance(center, pts[j], pts[j + 1]) < radius:
                    return False

        line: LineString | None = None
        for idx, (bx0, by0, bx1, by1) in enumerate(self._poly_bboxes):
            if ax1 < bx0 or bx1 < ax0 or ay1 < by0 or by1 < ay0:
                continue
            if line is None:
                line = LineString(pts)
            if self._polygons[idx].relate_pattern(line, "T********"):
                return False

        if self._safezone is not None:
            if line is None:
                line = LineString(pts)
            if not self._safezone.covers(line):
                return False
        return True

    def _check_collision(self, p1: Point, p2: Point) -> bool:
        """Test whether the straight segment ``p1`` -> ``p2`` is flyable.

        Args:
            p1: Segment start.
            p2: Segment end.

        Returns:
            ``True`` if the segment clears every obstacle and stays inside the
            operating area; ``False`` on any collision.
        """
        x0, x1 = (p1[0], p2[0]) if p1[0] <= p2[0] else (p2[0], p1[0])
        y0, y1 = (p1[1], p2[1]) if p1[1] <= p2[1] else (p2[1], p1[1])

        # Circles: the exact distance only for one whose bbox can reach the
        # chord's. A centre further than `radius` outside the chord's bounding
        # box is further than `radius` from the chord itself.
        for cx, cy, radius in self._circles:
            if cx + radius < x0 or cx - radius > x1 or cy + radius < y0 or cy - radius > y1:
                continue
            if su.point_to_line_distance((cx, cy), p1, p2) < radius:
                return False

        # Polygons: same prefilter, and the LineString is only built once some
        # bbox overlaps — on open water it is never built at all.
        line: LineString | None = None
        for idx, (bx0, by0, bx1, by1) in enumerate(self._poly_bboxes):
            if x1 < bx0 or bx1 < x0 or y1 < by0 or by1 < y0:
                continue
            if line is None:
                line = LineString([p1, p2])
            if self._polygons[idx].relate_pattern(line, "T********"):
                return False

        if self._safezone is not None:
            if line is None:
                line = LineString([p1, p2])
            if not self._safezone.covers(line):
                return False
        return True

    def _on_circle_boundary(self, point: Point, tol: float | None = None) -> bool:
        """Test whether a point rides an inflated circle's boundary.

        The tolerance must TRACK the construction lift: tangent points are built
        at ``r + _construct_delta``, so a tol below that classifies every one of
        them as off-boundary and silently switches the wrap step off entirely
        (measured: 13.0% of expansions ride a boundary at a 1e-9 lift, 0.0% at
        1e-3 -- which is what made a larger lift look like a 2.4% path cost).

        Args:
            point: The position to classify.
            tol: Boundary tolerance (m); defaults to the construction lift plus
                the rounding guard.

        Returns:
            ``True`` if the point is within ``tol`` of any inflated boundary.
        """
        if tol is None:
            tol = self._construct_delta + config.GEOM_EPS_M
        return any(
            abs(math.hypot(point[0] - center[0], point[1] - center[1]) - radius) < tol
            for center, radius in self.scenario["circle_obstacles"]
        )

    def _in_bounds(self, point: Point) -> bool:
        """Test whether a point lies inside the operating area.

        A ``safezones`` union takes precedence; otherwise an EXPLICIT
        ``map_bounds`` rectangle applies; with neither, everything is in bounds.
        The old code always fell back to the global ``config.MAP_WIDTH/HEIGHT``
        box, which silently rejected every waypoint of scenarios living outside
        it.

        Args:
            point: The position to test.

        Returns:
            ``True`` if the point is inside the operating area.
        """
        if self._safezone_prep is not None:
            return self._safezone_prep.covers(ShapelyPoint(*point))
        if not self._has_explicit_bounds:
            return True
        x, y = point
        return 0 < x < self._bounds_w and 0 < y < self._bounds_h

    def _check_fixed_legs(self) -> bool:
        """Test the mandatory ``W_{n-1} -> T`` seeker run-in for collisions.

        Returns:
            ``True`` if the fixed approach leg is clear.
        """
        return self._check_collision(self.goal_state.waypoint, self._target)

    def _ray_chord_clear(
        self,
        memo: dict[float, list[float]],
        ray: float,
        distance: float,
        p1: Point,
        p2: Point,
    ) -> bool:
        """Collision-test a chord, reusing what is already known about its ray.

        Every chord on one ray is a prefix of every longer chord on that ray, so
        a CLEAR verdict at some distance proves every shorter chord and a BLOCKED
        verdict proves every longer one. Pure memoisation: it changes how many
        chords are tested, never the verdict.

        Args:
            memo: Ray key -> ``[longest known clear, shortest known blocked]``.
            ray: The ray's heading, used as its key.
            distance: How far along the ray ``p2`` sits from ``p1``.
            p1: Chord start.
            p2: Chord end.

        Returns:
            ``True`` if the chord is flyable.
        """
        span = memo.get(ray)
        if span is None:
            span = memo[ray] = [0.0, float("inf")]
        if distance <= span[0]:
            return True
        if distance >= span[1]:
            return False
        if self._check_collision(p1, p2):
            span[0] = distance
            return True
        span[1] = distance
        return False

    def _try_goal_shot(self, current: State) -> State | None:
        """Build an analytic 2-corner manoeuvre from ``current`` to the goal.

        Fixed-goal mode only. Returns the goal State with parents linked back to
        ``current``, or None when nothing angle-, length- or collision-feasible
        exists.

        Args:
            current: The state to shoot from.

        Returns:
            The goal State, ready for reconstruction, or ``None``.

        Raises:
            TypeError: If ``current`` or the goal carries no heading.
        """
        if self._free_goal:
            return None
        goal_wp = self.goal_state.waypoint
        goal_heading = self.goal_state.heading
        heading = current.heading
        if goal_heading is None or heading is None:
            raise TypeError("the goal shot requires both headings in fixed-goal mode")

        candidates = gshot.two_corner_candidates(
            current.waypoint,
            heading,
            goal_wp,
            goal_heading,
            self.R,
            self._alpha_build,
            config.MIN_STRAIGHT_M,
            current.straight_budget,
            current.min_straight_in,
            num_dir=config.GOAL_SHOT_DIRS,
            num_cone=config.GOAL_SHOT_CONE,
        )
        base_g = self.g_scores[current]
        # Every corner sharing a leg1_heading lies on ONE ray out of `current`,
        # and every corner sharing an arrival_heading lies on one back-ray into
        # the goal, so the two legs are memoised per ray. Measured over 300
        # fixed-goal seeds: 39.1 collision checks per shot, of which most are
        # re-tests of a ray already settled by a longer or shorter chord.
        leg1_memo: dict[float, list[float]] = {}
        leg2_memo: dict[float, list[float]] = {}
        position = current.waypoint
        for candidate in candidates:
            corner = candidate.corner
            leg1_heading = candidate.leg1_heading
            arrival_heading = candidate.arrival_heading
            if not self._ray_chord_clear(
                leg1_memo, leg1_heading, math.dist(position, corner), position, corner
            ):
                continue
            if not self._ray_chord_clear(
                leg2_memo, arrival_heading, math.dist(corner, goal_wp), corner, goal_wp
            ):
                continue
            # The shot SYNTHESISES corners that never pass through
            # get_next_states, so nothing else arc-checks them. Leaving them
            # unchecked lets an unflyable fillet reach the final path, which the
            # oracle then rejects as path_self_collision (measured on seed 964
            # in the main planner). The third corner, at the goal turning onto
            # goal_heading, matters too: gw -> T is flown.
            if config.ARC_CLEARANCE_CHECK:
                if not self._corner_arc_clear(heading, current.waypoint, corner):
                    continue
                if not self._corner_arc_clear(leg1_heading, corner, goal_wp):
                    continue
                if not self._corner_arc_clear(arrival_heading, goal_wp, self._target):
                    continue

            corner_state = State(corner, leg1_heading)
            corner_state.parent = current
            turn_1 = abs(_angle_diff(leg1_heading, heading))
            corner_state.g_cost = (
                base_g + math.dist(current.waypoint, corner) + config.TURN_PENALTY_WEIGHT * turn_1
            )
            corner_state.straight_budget = candidate.budget_corner

            goal_state = State(goal_wp, arrival_heading)
            goal_state.parent = corner_state
            turn_2 = abs(_angle_diff(arrival_heading, leg1_heading))
            goal_state.g_cost = (
                corner_state.g_cost
                + math.dist(corner, goal_wp)
                + config.TURN_PENALTY_WEIGHT * turn_2
            )
            goal_state.straight_budget = candidate.budget_goal
            return goal_state
        return None

    def search(self) -> list[PlannerState] | None:
        """Run the kinodynamic A* search.

        The goal is accepted only when the state is within
        ``config.GOAL_THRESHOLD`` AND the arrival is feasible: in fixed-goal mode
        the arrival heading must be within ``alpha_max`` of ``goal_heading``; in
        free-goal mode the final leg must supply a straight run-in of at least
        ``DSS``.

        Returns:
            The searched waypoints, or ``None`` if none was found.
            ``budget_bound`` then says which of the two ways the search ended:
            the frontier ran out (a real "no path"), or the clock did.
        """
        started_at = time.perf_counter()
        budget_s = self.time_budget_s

        if not self.start_corners:
            self.search_failed = True
            return None

        for corner in self.start_corners:
            corner.h_cost = self.heuristic(corner, self.goal_state)
            heapq.heappush(
                self.open_set,
                (
                    corner.g_cost + config.HEURISTIC_WEIGHT * corner.h_cost,
                    self.iteration_count,
                    corner,
                ),
            )
            if corner.g_cost < self.g_scores[corner]:
                self.g_scores[corner] = corner.g_cost

        while self.open_set:
            # The budget is the ONLY stop condition. The loop otherwise runs
            # until the frontier is exhausted, which is the honest "no path".
            if (time.perf_counter() - started_at) > budget_s:
                self.budget_bound = True
                break

            self.iteration_count += 1
            _, _, current = heapq.heappop(self.open_set)

            if current in self.closed_set:
                continue

            self.closed_set.add(current)
            self.nodes_expanded += 1

            if len(self.open_set) <= 1 and self.num_strategy_b <= 0:
                self.num_strategy_b = config.NUM_STRATEGY_B

            # Analytic terminal shot, FIXED-goal mode only. The manoeuvre is
            # INJECTED into OPEN with its true g and h = 0 rather than returned:
            # A* still has to pick it as the cheapest frontier node, so the shot
            # prunes the adverse-approach flood without overriding the search.
            # Free-goal mode does not need it and does not get it — the Euclid
            # heuristic is only blind near a goal that has a required heading.
            if (
                config.GOAL_SHOT_ENABLED
                and self._shot_armed
                and (self.iteration_count % config.GOAL_SHOT_EVERY_N) == 0
            ):
                shot = self._try_goal_shot(current)
                if shot is not None and shot.g_cost < self.g_scores.get(shot, float("inf")):
                    self.g_scores[shot] = shot.g_cost
                    shot.h_cost = 0.0
                    heapq.heappush(
                        self.open_set,
                        (
                            shot.g_cost + config.HEURISTIC_WEIGHT * shot.h_cost,
                            self.iteration_count,
                            shot,
                        ),
                    )

            dist_to_goal = math.sqrt(
                (current.waypoint[0] - self.goal_state.waypoint[0]) ** 2
                + (current.waypoint[1] - self.goal_state.waypoint[1]) ** 2
            )

            if dist_to_goal < config.GOAL_THRESHOLD:
                reached = self._goal_reached(current)
                if reached:
                    return self._reconstruct_path(current)

            for next_state, transition_cost in self.get_next_states(current):
                if next_state in self.closed_set:
                    continue

                tentative_g = self.g_scores[current] + transition_cost
                if tentative_g < self.g_scores.get(next_state, float("inf")):
                    next_state.parent = current
                    self.g_scores[next_state] = tentative_g
                    next_state.g_cost = tentative_g
                    next_state.h_cost = self.heuristic(next_state, self.goal_state)
                    heapq.heappush(
                        self.open_set,
                        (
                            next_state.g_cost + config.HEURISTIC_WEIGHT * next_state.h_cost,
                            self.iteration_count,
                            next_state,
                        ),
                    )

        self.search_failed = True
        return None

    def _goal_reached(self, current: State) -> bool:
        """Test whether a state within the goal threshold may actually terminate.

        Args:
            current: The state being examined, already within
                ``config.GOAL_THRESHOLD`` of the goal.

        Returns:
            ``True`` if the arrival is kinodynamically feasible.
        """
        if self._free_goal:
            # The final leg IS the seeker run-in, so it must be long enough
            # after the fillet at its far end is paid for.
            parent = current.parent
            if parent is None or parent.heading is None:
                return False
            seg = math.dist(parent.waypoint, current.waypoint)
            bearing = su.angle_to_heading(parent.waypoint, current.waypoint)
            turn_at_prev = abs(_angle_diff(bearing, parent.heading))
            return seg - self.R * math.tan(turn_at_prev / 2.0) >= self._dss

        goal_heading = self.goal_state.heading
        if goal_heading is None or current.heading is None:
            return False
        return abs(_angle_diff(goal_heading, current.heading)) <= self._alpha_build

    def _reconstruct_path(self, state: State) -> list[PlannerState]:
        """Walk parent pointers back to the start, expanding pivot slides.

        A ``via`` pivot is a real waypoint -- the vehicle flies straight through
        its parent and turns there -- so it is emitted before its own waypoint.

        Args:
            state: The goal state reached by the search.

        Returns:
            The searched waypoints in flight order.
        """
        states: list[State] = []
        current: State | None = state
        while current is not None:
            states.append(current)
            current = current.parent
        states.reverse()

        path: list[PlannerState] = []
        for st in states:
            if st.via is not None:
                path.append(st.via)
            heading = st.heading
            if heading is None:
                raise TypeError("reconstructed path contains a headingless state")
            path.append((st.waypoint, heading))
        return path

    def get_search_stats(self) -> SearchStats:
        """Return the counters describing how the last search ran."""
        return {
            "iterations": self.iteration_count,
            "time_budget_s": self.time_budget_s,
            "budget_bound": self.budget_bound,
            "open_set_size": len(self.open_set),
            "search_failed": self.search_failed,
        }

    def smooth_path(self, path: list[PlannerState]) -> list[PlannerState]:
        """Return the shortest FEASIBLE subsequence of the path, by exact DP over O..T.

        A greedy shortcutter cannot do this: đoản trình couples adjacent chords
        through the turn they share, so dropping a waypoint sharpens the turn at
        its neighbour and retroactively steals straight length from the chord
        INTO that neighbour. The DP carries that coupling in its state, the same
        way the search does with ``State.straight_budget``::

            state  (u, v) = last two kept waypoints
            budget        = straight left on chord u->v after its NEAR fillet
            step (u,v) -> (v,w) reveals the turn at v, which is the FAR fillet
                          of u->v and the NEAR fillet of v->w

        so every chord is validated with both fillets known. ``O`` and ``T`` are
        nodes of the graph, which is what enforces ``l1 >= L0`` (no turn is
        available at ``O``, so the first chord must lie on the takeoff ray) and
        the ``>= DSS`` run-in. Entries per state are kept under dominance: more
        budget AND lower cost. Cost charges ``SMOOTH_NODE_PENALTY_M`` per kept
        waypoint, so that among equal-length subsequences the shortest one wins
        -- a waypoint flown straight through adds exactly zero length and would
        otherwise survive by chance.

        Args:
            path: The searched waypoints.

        Returns:
            The smoothed subsequence, or the input unchanged when the path is
            too short, too long to smooth, or the DP finds nothing.
        """
        if len(path) < 3:
            return path

        origin = self._origin
        target = self._target
        waypoints: list[Point] = [w for w, _ in path]
        head = 0
        if math.dist(origin, waypoints[0]) > 1.0:
            waypoints.insert(0, (origin[0], origin[1]))
            head = 1
        tail = 0
        if math.dist(target, waypoints[-1]) > 1.0:
            waypoints.append((target[0], target[1]))
            tail = 1
        count = len(waypoints)
        if count < 3 or count > config.SMOOTH_MAX_NODES:
            return path

        turn_radius = self.R
        # The true limit, NOT the build reserve. Every corner the DP weighs is
        # defined by waypoints that already exist, and it measures them with the
        # oracle's own formula, bit for bit -- so this gate IS the oracle's
        # check, not a construction that needs padding away from the limit.
        # Using _alpha_build here re-measures the search's own corners against a
        # limit 1e-9 rad tighter than the one they were built at: a corner built
        # AT the limit reads back as alpha_max - 1e-9 + ~3e-15 rad and rejects,
        # which kills every continuation out of it and drops the whole DP into
        # its "found nothing" fallback -- smoothing silently does nothing.
        alpha_max = self.alpha_max_rad
        l0 = self._l0
        dss = self._dss
        # Length tie-break: a waypoint flown straight through costs zero length,
        # so without this the DP keeps or drops it arbitrarily.
        node_cost = config.SMOOTH_NODE_PENALTY_M
        start_h = self.scenario["start_state"]["heading"]
        # Only meaningful when T really is the terminal node we appended.
        goal_h = None if (self._free_goal or not tail) else self.scenario.get("goal_heading")

        # Chord geometry, computed once. `clear` uses the planner's own collision
        # test so the smoothed path obeys the safezone too, not just obstacles.
        dist = [[0.0] * count for _ in range(count)]
        brg = [[0.0] * count for _ in range(count)]
        clear = [[False] * count for _ in range(count)]
        for i in range(count):
            for j in range(i + 1, count):
                dist[i][j] = math.dist(waypoints[i], waypoints[j])
                brg[i][j] = math.atan2(
                    waypoints[j][1] - waypoints[i][1], waypoints[j][0] - waypoints[i][0]
                )
                clear[i][j] = self._check_collision(waypoints[i], waypoints[j])

        arc_memo: dict[tuple[int, int, int], bool] = {}

        def arc_ok(u: int, v: int, w: int) -> bool:
            """Memoised fillet-arc gate for the corner at ``v`` between ``u`` and ``w``."""
            if not config.ARC_CLEARANCE_CHECK:
                return True
            hit = arc_memo.get((u, v, w))
            if hit is None:
                hit = self._corner_arc_clear(brg[u][v], waypoints[v], waypoints[w])
                arc_memo[(u, v, w)] = hit
            return hit

        # by_cur[v][u] = [entry, ...]
        by_cur: defaultdict[int, dict[int, list[_DpEntry]]] = defaultdict(dict)
        for j in range(1, count):
            if not clear[0][j]:
                continue
            if abs(_angle_diff(brg[0][j], start_h)) > config.TAKEOFF_RAY_TOL_RAD:
                continue
            by_cur[j][0] = [_DpEntry(dist[0][j], dist[0][j] + node_cost, None, None)]

        best: tuple[tuple[int, int], float, _DpEntry] | None = None
        for v in range(1, count):
            for u, entries in by_cur[v].items():
                for entry in entries:
                    budget, cost = entry.budget, entry.cost
                    if v == count - 1:
                        # Terminal: the fillet at T is zero, so the whole
                        # remaining budget is the seeker run-in.
                        # Fixed goal: T is not a plain node — the run-in must be
                        # flown along goal_heading, so the last chord has to lie
                        # on the approach ray (the mirror of the takeoff-ray
                        # rule at O). Without this the DP drops W_{n-1} and
                        # arrives on the wrong heading.
                        if (
                            goal_h is not None
                            and abs(_angle_diff(brg[u][v], goal_h)) > config.APPROACH_RAY_TOL_RAD
                        ):
                            continue
                        if budget >= dss and (best is None or cost < best[1]):
                            best = ((u, v), cost, entry)
                        continue
                    for w in range(v + 1, count):
                        if not clear[v][w]:
                            continue
                        turn = abs(_angle_diff(brg[v][w], brg[u][v]))
                        if turn > alpha_max:
                            continue
                        reserve = turn_radius * math.tan(turn / 2.0)
                        # Far end of chord u->v, now that the turn at v is known.
                        need = l0 if u == 0 else config.MIN_STRAIGHT_M
                        if budget - reserve < need:
                            continue
                        if not arc_ok(u, v, w):
                            continue
                        new_budget = dist[v][w] - reserve
                        new_cost = cost + dist[v][w] + node_cost
                        bucket = by_cur[w].setdefault(v, [])
                        if any(
                            e.budget >= new_budget - 1e-9 and e.cost <= new_cost + 1e-9
                            for e in bucket
                        ):
                            continue
                        bucket[:] = [
                            e
                            for e in bucket
                            if not (new_budget >= e.budget - 1e-9 and new_cost <= e.cost + 1e-9)
                        ]
                        bucket.append(_DpEntry(new_budget, new_cost, (u, v), entry))

        if best is None:
            return path

        key, _cost, entry_opt = best
        seq: list[int] = []
        entry: _DpEntry | None = entry_opt
        while entry is not None:
            seq.append(key[1])
            prev_key, prev_entry = entry.prev_key, entry.prev_entry
            if prev_key is None:
                seq.append(key[0])
                break
            key, entry = prev_key, prev_entry
        seq.reverse()

        out: list[PlannerState] = []
        for idx in range(1 if head else 0, len(seq) - 1 if tail else len(seq)):
            node = seq[idx]
            heading = brg[seq[idx - 1]][node] if idx > 0 else path[0][1]
            out.append((waypoints[node], heading))
        return out if len(out) >= 1 else path

    def plan(self, verbose: bool = False) -> PlanResult:
        """Run the feasibility gates, the search, the smoother and the oracle.

        Args:
            verbose: Print progress information to stdout.

        Returns:
            The plan result; on failure ``path`` may still carry the rejected
            route for inspection, with ``success`` false and ``failure_reason``
            set.
        """
        # Feasibility gates first, each with its own honest reason:
        # - start blocked: every seeded takeoff corner was infeasible (O inside
        #   an inflated obstacle, or the takeoff ray collides / leaves the area).
        # - goal leg blocked: the mandatory W_{n-1}->T run-in hits an obstacle.
        if not self.start_corners:
            return self._result(None, False, "start_leg_blocked")
        if not self._check_fixed_legs():
            return self._result(None, False, "goal_leg_blocked")

        if verbose:
            print("Starting A* search...")
        path = self.search()
        if verbose:
            stats = self.get_search_stats()
            print(
                f"Search completed: {stats['iterations']} iterations in "
                f"<= {stats['time_budget_s']:g} s"
                + (" (budget exhausted)" if stats["budget_bound"] else "")
            )
        if path is None:
            return self._result(None, False, "no_path")

        path = self.smooth_path(path)

        # Final whole-path oracle. The search validates each edge as it goes, but
        # arc expansion, smoothing, and the fixed O->W1 / W_{n-1}->T legs (added
        # outside the search) can still leave a full O..T path that violates
        # collision OR the đoản-trình (min-straight) constraint — e.g. two turns
        # ending up too close, so a middle segment's usable straight goes
        # negative. Re-validate the whole path with the INDEPENDENT oracle so
        # success really means oracle-valid; a path that fails is reported as an
        # honest failure, not returned as a silent bad plan. This is exactly the
        # invariant asserted by tests/oracle_validity_test.py. Straight legs AND
        # turn arcs are both checked against the INFLATED obstacles:
        # path_is_valid's raw_* escape hatch is left unset on purpose, because
        # inflation no longer carries a turn term for a fillet to bulge into.
        full = mission.full_mission_path(path, self.scenario)
        valid, failure_reason = pv.path_is_valid(
            full,
            self.scenario["circle_obstacles"],
            self.scenario["polygon_obstacles"],
            turn_radius=self.scenario["turn_radius"],
            alpha_max_rad=self.scenario["alpha_max_rad"],
            l0=self._l0,
            dss=self._dss,
        )
        if not valid:
            return self._result(path, False, failure_reason)

        if verbose:
            print(f"Path found with {len(path)} waypoints")
        return self._result(path, True, None)

    def _result(
        self, path: list[PlannerState] | None, success: bool, reason: str | None
    ) -> PlanResult:
        """Package a path, its verdict and the search stats into one result."""
        return {
            "path": path,
            "success": success,
            "failure_reason": reason,
            "stats": self.get_search_stats(),
            "planner": self,
        }


def plan_trajectory(
    preprocessed_scenario: PreprocessedScenario,
    verbose: bool = False,
    time_budget_s: float | None = None,
) -> PlanResult:
    """Plan an autonomous aircraft trajectory end to end.

    ``success`` means the INDEPENDENT oracle accepted the whole mission path,
    not merely that the search returned something.

    Args:
        preprocessed_scenario: Output of
            :func:`core.preprocessing.prepare_scenario`.
        verbose: Print progress information to stdout.
        time_budget_s: Wall-clock budget for the search, in seconds, or
            ``None`` to use ``config.TIME_BUDGET_S``. It is the search's only
            stop condition; see :class:`KinodynamicAstar`.

    Returns:
        The plan result. ``result['stats']['budget_bound']`` tells a caller
        whether the answer was cut short by the clock.
    """
    if verbose:
        print("Initializing Kinodynamic A*...")
    return KinodynamicAstar(preprocessed_scenario, time_budget_s=time_budget_s).plan(
        verbose=verbose
    )
