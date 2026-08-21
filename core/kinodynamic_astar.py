"""Kinodynamic A* path planning for autonomous aircraft trajectories.

Searches over ``(waypoint, heading)`` states under the vehicle's turn-angle and
minimum-straight (đoản-trình) constraints, then shortens the result with an
exact subsequence DP. Distances are metres, angles radians.

This is the feature-rich sibling of ``core/kinodynamic_astar_v0.py``. What lives
only here, deliberately: arc-hop successors that ride a circle's boundary at
true arc-length cost, the analytic 2-corner goal shot, the per-path
``STRATEGY_B_CONSECUTIVE`` valve, and the interior-overlap machinery that tells
a real polygon crossing apart from a graze. v0 is the standard the two share;
this file follows it.
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

import config
import core.arc_geometry as ag
import core.goal_shot as gshot
import core.mission as mission
import core.path_validation as pv
import core.spatial_utils as su

if TYPE_CHECKING:
    from core.types import (
        CircleGeometry,
        LatticeKey,
        PlannerState,
        Point,
        PreprocessedScenario,
        SearchStats,
        WrapSense,
    )

# Hot-path alias of su.angle_diff (module-global lookup, called ~10x per
# candidate). Not a second definition -- see the note in spatial_utils.
_angle_diff = su.angle_diff

# Fixed clearance bulge for riding arcs: circumscribed-polygon vertices for
# any expansion step <= 45 deg stay within r / cos(pi/8) of the center.
_ARC_CLEAR_BULGE = 1.0 / math.cos(math.pi / 8.0)

# Minimum usable straight-flight length (đoản trình) between two waypoints, in
# metres. The value lives in config so the two planners cannot drift apart; the
# local alias is kept because it is read on the hot path.
_MIN_STRAIGHT_M = config.MIN_STRAIGHT_M

# Squared candidate-distance floor, compared against dx*dx + dy*dy. Squared
# because the quantity is: `config.EPS` was once compared against squared metres
# under a metres name, which turned a "1 um" constant into a 1 mm cutoff.
_CAND_MIN_D2 = max(config.CANDIDATE_MIN_DIST_M, config.GEOM_EPS_M) ** 2

# Shortest polygon-interior overlap that counts as a collision, in metres --
# the OWNER of this number is the oracle (core/path_validation), and the planner
# borrows it on purpose: a planner stricter than its own validator rejects
# flyable chords. 'T********' alone matches a zero-extent touch, so a chord that
# grazes a hull VERTEX reads as a hit. That is not hypothetical -- it made the
# collision test non-monotone under splitting (batch_random_test seed 0: two
# collinear chords each clear, their union "blocked" on an overlap of 2.9e-11 m
# at the shared vertex), which is exactly the chord smooth_path needs in order
# to drop the pass-through waypoint sitting on that vertex.
# Hot-path alias, not a second definition.
_POLY_TOUCH_TOL_M = pv.POLYGON_TOUCH_TOL_M

# How far the first smoothed chord may deviate from start_heading (radians).
# No turn is available at the takeoff point, so the first kept waypoint must sit
# on the takeoff ray; the tolerance only absorbs float noise (the measured spread
# between bearing(O -> W1) and start_heading is at most 4e-15 rad).
# Hot-path alias of the config knob; not a second definition.
_TAKEOFF_RAY_TOL_RAD = config.TAKEOFF_RAY_TOL_RAD


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
        arc_from: ``(center, radius, arc_start, sense)`` when this state was
            reached by riding a circle boundary; expanded back into waypoints at
            reconstruction time.
        via: ``(pivot, heading)`` of an INTERMEDIATE straight-through waypoint on
            the incoming edge when this state was reached by an along-ray pivot
            slide: the vehicle flies straight through the parent's candidate
            corner to ``pivot`` and only turns there.
        straight_budget: Straight length left on the INCOMING segment after its
            near-end turn reserve -- the budget still available to this
            waypoint's own turn. The đoản-trình far-end check is deferred to
            this state's expansion, where its outgoing turn is known (no
            alpha_max worst case). ``inf`` means no straight constraint is
            carried in (start state, arc-ride departures).
        min_straight_in: Straight length the INCOMING segment must still keep.
            Normal states need the generic minimum; seeded start corners
            override it with ``L0`` so the takeoff leg is enforced exactly.
        is_start_corner: Whether this is a seeded takeoff corner. Corner
            expansions are exempt from the global Strategy-B valve budget: all K
            corners expand while the goal is still occluded, so ``K >
            NUM_STRATEGY_B`` would drain the valve at takeoff and starve
            mid-course reorientation for the whole search.
        consec_b: Consecutive Strategy-B (radial fan) count on the path that
            reached this state: 0 if the incoming edge was NOT a fan expansion,
            else ``parent.consec_b + 1``. Used only when
            ``STRATEGY_B_CONSECUTIVE`` gates the fan per-path instead of by the
            global valve budget.
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
        self.arc_from: tuple[Point, float, Point, WrapSense] | None = None
        self.via: tuple[Point, float] | None = None
        self.straight_budget: float = float("inf")
        self.min_straight_in: float = _MIN_STRAIGHT_M
        self.is_start_corner: bool = False
        self.consec_b: int = 0
        # Dedup key cache: waypoint/heading never change after construction,
        # and the search hashes/compares each state hundreds of times
        # (measured ~1M state_to_tuple calls on 5 hard seeds). Computed
        # LAZILY on first hash/eq — a free-goal goal_state carries
        # heading=None and must stay constructible (it is never hashed).
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
    arc hops that ride an inflated circle's boundary, Strategy-A tangent/vertex/
    goal candidates, and a budgeted Strategy-B radial fan.
    """

    def __init__(self, preprocessed_scenario: PreprocessedScenario) -> None:
        """Build the planner's obstacle caches, endpoints and seeded start corners.

        Args:
            preprocessed_scenario: Output of
                :func:`core.preprocessing.prepare_scenario`.
        """
        self.scenario = preprocessed_scenario
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
        # Operational stand-off + rounding guard, added never merged. Every
        # piece of geometry this planner CONSTRUCTS is lifted by it.
        self._construct_delta = config.CONSTRUCTION_CLEARANCE_M + config.GEOM_EPS_M
        self._polygons = [Polygon(coords) for coords in preprocessed_scenario["polygon_obstacles"]]
        # Plain-float bboxes for the manual prefilter in _check_collision /
        # _sector_clear. At N <= ~20 polygons a scalar bbox loop beats the
        # STRtree python dispatch, and — the real win — the query geometry
        # (LineString / sector quad) is only CONSTRUCTED when some bbox
        # overlaps: measured ~50% of hard-seed wall time was shapely object
        # construction on queries that hit nothing.
        self._poly_bboxes: list[tuple[float, float, float, float]] = [
            p.bounds for p in self._polygons
        ]
        # Circles pre-unpacked to plain floats, as v0 already keeps them. The
        # hot loops read this hundreds of thousands of times and the nested
        # `for (cx, cy), radius in ...` paid a tuple unpack on every circle of
        # every call.
        #
        # CONTRACT: the obstacle field is FROZEN at construction. _check_collision
        # and _corner_arc_clear read this list, not scenario['circle_obstacles'],
        # so assigning to that key on a live planner has no effect on collision
        # checking. v0 has worked this way since 494e85d; main now matches.
        # tests/kinodynamic_arc_hop_test.py does assign to it (to block a fixed
        # leg), which an earlier version of this comment wrongly claimed nothing
        # in the repo does -- build a new planner instead of mutating one.
        self._circles: list[tuple[float, float, float]] = [
            (c[0], c[1], r) for c, r in preprocessed_scenario["circle_obstacles"]
        ]
        # Shrunk copies for the deep-hit short-circuit in _check_collision (see
        # config.POLYGON_DEEP_HIT_INSET_M). buffer() can return empty or a
        # MultiPolygon; an empty one simply never short-circuits.
        self._polygons_deep = [p.buffer(-config.POLYGON_DEEP_HIT_INSET_M) for p in self._polygons]
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

        # NOTE for _corner_arc_clear: the search-time turn-arc check uses the
        # INFLATED sets above — the same ones the straight-leg check uses. It
        # used to have its OWN sets, aliased here, because inflation carried a
        # `R*(1/cos(alpha_max/2)-1)` turn term and a fillet arc was designed to
        # bulge into exactly that band, so arcs were checked against RAW. With
        # the turn term gone there is no band to bulge into, and checking arcs
        # against raw would let a turn dip inside the operator's minimum
        # stand-off (measured: 97.9 m of true clearance on a run configured for
        # 500 m). The four _arc_* aliases are gone with the distinction they
        # encoded; arcs and straights both honour SAFE_MARGIN.
        self._arc_check_n = max(2, int(config.ARC_CHECK_SAMPLES))

        # Operating areas (safezones). When one or more polygons are supplied the
        # aircraft must stay inside their UNION — both every generated waypoint
        # (_in_bounds) and every edge/chord (_check_collision) are constrained to
        # it. The union (a Polygon or MultiPolygon) is prepared once so the
        # repeated point/segment containment tests on the hot search path are
        # cheap. When absent, fall back to the rectangle from the scenario's
        # map_bounds, else the global config.MAP_WIDTH/HEIGHT.
        safezones = preprocessed_scenario.get("safezones")
        self._safezone = unary_union([Polygon(sz) for sz in safezones]) if safezones else None
        self._safezone_prep = shp_prep(self._safezone) if self._safezone is not None else None
        map_bounds = preprocessed_scenario.get("map_bounds")
        # Only enforce a rectangular bound when one is EXPLICITLY supplied. The
        # global config.MAP_WIDTH/HEIGHT is a legacy 500 km default that is
        # meaningless for scenarios living elsewhere (e.g. real missions at
        # y ~ 1.15e6); enforcing it there would reject every waypoint. When
        # neither a safezone nor an explicit map_bounds is given, _in_bounds is
        # permissive (the search is still bounded by obstacles, candidates,
        # MAX_ITERATIONS and the time budget).
        self._has_explicit_bounds = map_bounds is not None
        self._bounds_w, self._bounds_h = (
            map_bounds if map_bounds else (config.MAP_WIDTH, config.MAP_HEIGHT)
        )

        self.start_state = State(
            preprocessed_scenario["start_state"]["waypoint"],
            preprocessed_scenario["start_state"]["heading"],
        )
        self.start_state.g_cost = 0
        # The incoming O->W1 leg's straight length (near turn at O is 0). The
        # far-end (turn at W1) đoản-trình is then deferred to W1's expansion.
        self.start_state.straight_budget = math.dist(origin, self.start_state.waypoint)

        self.goal_state = State(
            preprocessed_scenario["goal_state"]["waypoint"],
            preprocessed_scenario["goal_state"]["heading"],
        )

        # Free terminal approach mode: goal_heading is None. The search then
        # targets T itself (goal_state.waypoint == goal_pos) and the final edge
        # into T must be a straight run-in of length >= DSS in a search-chosen
        # direction (no fixed approach heading, no terminal turn).
        self._free_goal = preprocessed_scenario.get("goal_heading") is None
        self._dss = preprocessed_scenario["goal_state"].get("engagement_distance", config.DSS)

        # Search variables. NOTE: there is deliberately NO came_from dict —
        # State hashing quantises to a coarse lattice (1000 m / 3 deg), so a
        # lattice-keyed parent map lets two distinct candidates collide and
        # splice the reconstruction onto a parent whose transition was never
        # collision-checked ("phantom edges"). Parents are stored per-object
        # (State.parent), so every reconstructed edge is exactly a validated
        # transition.
        self.open_set: list[tuple[float, int, State]] = []
        self.closed_set: set[State] = set()
        self.g_scores: defaultdict[State, float] = defaultdict(lambda: float("inf"))

        self.iteration_count = 0
        self.max_iterations = config.MAX_ITERATIONS
        self.R = preprocessed_scenario["turn_radius"]
        self.alpha_max_rad = preprocessed_scenario["alpha_max_rad"]
        # Turn limit used when BUILDING and accepting geometry: padded towards
        # feasibility (SUBTRACTED), so a corner built hard against the limit is
        # still legal when the oracle recomputes the angle from waypoint
        # geometry. Measured, that recomputation overshoots by up to 1.1e-15 rad.
        self._alpha_build = self.alpha_max_rad - config.GEOM_EPS_RAD
        # Cosine of the widest turn the cheap prefilter may reject outright.
        self._turn_cos_guard = math.cos(
            min(math.pi, self._alpha_build + config.TURN_PREFILTER_BAND_RAD)
        )

        self.start_corners: list[State] = self._seed_start_corners()

        # Pre-computed constants (depend only on R / alpha_max / config, all
        # fixed for the planner's lifetime) hoisted out of the per-expansion
        # hot loops. Values are byte-identical to computing them inline.
        # Fan distance rungs, as the part of a fan leg BEYOND its near reserve:
        # rung j = far reserve for a next turn beta_j + the straight pad, with
        # tan(beta_j/2) = (j/M)*tan(alpha_max/2) (tan-uniform, exactly like the
        # start corners). Rung j is the shortest leg that still affords a next
        # turn beta <= beta_j, so the search can pick a tight pivot when it only
        # needs a gentle turn instead of always paying the worst case. The last
        # rung (j = M) is the full alpha_max reserve, i.e. the legacy single
        # distance — a pivot that can bridge a constrained goal-approach slot
        # (seed 4: a halved reach forced an 88 km detour there).
        num_rungs = max(1, int(config.NUM_FAN_DISTANCES))
        tan_half_max = math.tan(self._alpha_build / 2)
        self._fan_rungs = [
            self.R * (j / num_rungs) * tan_half_max + config.RADIAL_FAN_STEP_M
            for j in range(1, num_rungs + 1)
        ]
        self._arc_sample_step = math.radians(config.ARC_SAMPLE_STEP_DEG)
        self._arc_sample_n = round(2.0 * math.pi / self._arc_sample_step)

        # Whether the state being expanded rides any circle boundary; set as a
        # side effect of _arc_hop_successors (which already evaluates
        # riding_sense per circle) so get_next_states need not recompute it.
        self._riding = False

        # Which gate rejected the most recent _pivot_candidate (None on
        # success). Same side-channel style as _riding: it lets the caller ask
        # "was this an ARC rejection?" — the only kind worth retrying with an
        # along-ray slide — without _pivot_candidate returning a richer type on
        # its hot path.
        self._last_reject: str | None = None

        self.search_failed = False

        # Search route before arc expansion/smoothing (set on success);
        # used to verify discretisation invariance.
        self.raw_route: list[PlannerState] | None = None

        self.num_strategy_b = config.NUM_STRATEGY_B
        # Global safety valve for the HYBRID (consecutive) Strategy-B mode:
        # TOTAL occluded-reorient fan firings allowed before a hard cut-off.
        self._sb_global = config.STRATEGY_B_GLOBAL_CAP

        # Lazy memo of arc-hop departure candidates, keyed by
        # (circle_index, sense). The candidate list (bitangent departures to
        # every other circle + departure points to every polygon vertex and
        # the goal) depends only on which circle/sense is being ridden, not
        # on the current position P, so it is computed once per (circle, s)
        # and reused on every later ride of that same circle+sense. Keyed by
        # index into self.scenario['circle_obstacles'] (hashable and stable;
        # the (center, radius) tuples themselves are also hashable but the
        # index avoids float-tuple hashing on every ride).
        self._dep_cache: dict[tuple[int, WrapSense], list[Point]] = {}

    def _seed_start_corners(self) -> list[State]:
        """Seed the takeoff corners the search is rooted at.

        Instead of rooting the search at the single worst-case ``W_1``
        (``L0 + R*tan(alpha_max/2)`` along the takeoff ray), seed ``K`` corner
        states at ``d_i = L0 + R*tan(a_i/2)`` with tan-uniform buckets
        ``tan(a_i/2) = (i/K)*tan(alpha_max/2)``, ``i = 1..K`` (bucket ``K`` is
        the legacy ``W_1``, so ``NUM_START_CORNERS = 1`` is exactly legacy). A
        corner seeded for ``a_i`` affords any first turn ``alpha <= a_i`` while
        keeping the takeoff straight ``l1 >= L0`` EXACTLY.

        Corners that leave the operating area, or whose takeoff leg
        ``O -> corner`` collides, are NOT seeded -- feasibility recovery near
        obstacles and safezone edges, where the old fixed ``W_1`` could land
        inside an inflated zone and kill the whole plan.

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
            # +GEOM_EPS_M so the built takeoff straight is strictly longer than
            # L0 and survives the oracle's exact `l1 >= L0` recomputation.
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
            # True along-ray cost from O. All corners share the same O origin,
            # so relative costs between corners are exact (the legacy single
            # root could use g=0 because its offset was a common constant).
            state.g_cost = distance
            state.straight_budget = distance
            state.min_straight_in = l0
            state.is_start_corner = True
            corners.append(state)
        return corners

    def heuristic(self, state: State, goal_state: State) -> float:
        """Estimate the remaining cost from ``state`` to ``goal_state``.

        The old ``dist + R * heading_diff`` term was inadmissible because heading
        is corrected gradually while travelling, so it over-estimated the
        remaining cost and could make A* return suboptimal paths.

        Args:
            state: The state being scored.
            goal_state: The search target.

        Returns:
            The straight-line distance in metres, an admissible lower bound.
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

        The check is split across the two events its two turns become known.
        ``turn_at_current`` (the turn AT ``current``, from its incoming heading
        onto this new segment) eats the incoming segment's far end AND the new
        segment's near end. ``far_reserve`` is the new segment's far-end bite
        when it is already known (the terminal turn onto the goal); 0 otherwise,
        in which case that check is deferred to the new state's own expansion.

        Args:
            current: The state being expanded.
            seg_len: Length of the new leg (m).
            turn_at_current: Turn angle at ``current`` (rad).
            far_reserve: Fillet reserve already known at the far end (m).
            advance: Along-ray pivot slide (see :meth:`_slide_pivot`): the
                vehicle flies straight THROUGH ``current`` for this much further
                before turning, so the incoming run is that much longer and the
                turn happens at the slid pivot. Since the direction is unchanged
                this only ever ADDS budget, which is the whole point of sliding
                along the ray.

        Returns:
            The new state's ``straight_budget`` (new segment length minus its
            near reserve) when both ends have room, else ``None``. The deferred
            far-end check uses ``current``'s own ``min_straight_in`` threshold.
        """
        reserve = self.R * math.tan(turn_at_current / 2.0)
        # Deferred far-end check of `current`'s incoming segment.
        if current.straight_budget + advance - reserve < current.min_straight_in:
            return None
        budget = seg_len - reserve
        if budget - far_reserve < _MIN_STRAIGHT_M:
            return None
        return budget

    def get_next_states(self, current_state: State) -> list[tuple[State, float]]:
        """Generate the successors of a state, with their transition costs.

        Three sources: arc hops along any circle boundary this state rides;
        Strategy-A tangent points to circles, lifted polygon hull vertices and
        the goal; and the Strategy-B radial fan.

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

        # --- Arc-hop: ride any circle boundary this state is tangent to ---
        # All riding/tangent geometry is built on the lifted radius so
        # constructed chords are strictly clear of the exact-checked inflated
        # boundary. Two separate reasons, deliberately added rather than merged:
        # CONSTRUCTION_CLEARANCE_M is an operational stand-off (free to be 0),
        # GEOM_EPS_M is the float-rounding guard that must never be 0.
        delta = self._construct_delta
        successors.extend(self._arc_hop_successors(current_state))
        riding = self._riding  # set as a side effect of _arc_hop_successors

        # --- Strategy A: dynamic tangent / vertex / goal candidates ---
        goal_wp = self.goal_state.waypoint
        candidates: list[Point] = []
        for center, radius in self.scenario["circle_obstacles"]:
            candidates.extend(su.circle_tangent_points(position, center, radius + delta))
        candidates.extend(self._poly_vertices)
        candidates.append(goal_wp)

        for node in candidates:
            dx = node[0] - position[0]
            dy = node[1] - position[1]
            if dx * dx + dy * dy < _CAND_MIN_D2:
                continue
            result = self._pivot_candidate(current_state, node, 0.0)
            if result is None and config.NUM_PIVOT_SLIDES > 0 and self._last_reject == "arc":
                # Only an ARC rejection is worth retrying. Sliding forward can
                # only INCREASE the turn, so a candidate already over alpha_max
                # is hopeless; and a blocked chord is almost never unblocked by
                # moving the pivot (measured: 1.0%). Retrying every rejection
                # regardless costs 4 extra collision-checked attempts on the
                # candidates that dominate dense maps, which is what made
                # iterations collapse 45335 -> 13851 at K=4.
                result = self._slide_pivot(current_state, node)
            if result is not None:
                successors.append(result)

        # NOTE: it is tempting to skip the fan entirely when the goal is
        # already a valid successor ("the fan is only branching noise in open
        # water" — tests/kinodynamic_arc_hop_test.py::test_no_radial_fan_in_
        # open_water). The search de-duplicates on a coarse lattice
        # (STATE_POS_QUANTUM, STATE_HEADING_QUANTUM_DEG), so it is NOT exactly
        # optimal, and the fan's "redundant" pivots act as lattice diversity
        # rather than noise. Gate the BUDGET here, not whether the fan fires.
        #
        # The number this comment used to quote — "that costs seed 4 88 km
        # (534.9 vs 446.9)" — NO LONGER REPRODUCES: re-measured 2026-08-21, that
        # exact gate costs +0.0376% (free) / +0.0483% (fixed) and saves nothing.
        # The conclusion stands on a different case: suppressing the fan on
        # every line-of-sight-clear expansion costs seed 51 +73.5%.
        if successors and not riding and not self._check_collision(position, goal_wp):
            # Escape valve: while the goal is occluded, a few budgeted fan
            # expansions provide cheap reorientation moves (e.g. an adverse
            # initial heading) that tangent/vertex candidates cannot express;
            # without this the search can commit to a long detour (seed 319:
            # 978.8 km vs 728.9 km with the valve). Start corners are exempt
            # from the budget: all K corners expand while the goal is still
            # occluded, so with K > budget they would drain the valve at
            # takeoff and starve mid-course reorientation (seed 964: 546.9 km
            # vs 481.2 km with the exemption).
            if config.STRATEGY_B_CONSECUTIVE:
                # HYBRID: per-path cap (at most NUM_STRATEGY_B fan waypoints in a
                # row on one path — every path independent, a non-fan step
                # resets consec_b) AND a global safety valve on TOTAL fan firings
                # (hard cap, no re-arm) so the per-path rule cannot blow up the
                # frontier on pathological maps.
                if current_state.consec_b >= config.NUM_STRATEGY_B:
                    return successors
                if self._sb_global <= 0:
                    return successors
                self._sb_global -= 1
            elif not current_state.is_start_corner:
                if self.num_strategy_b <= 0:
                    return successors
                self.num_strategy_b -= 1

        # --- Strategy B: radial fan — a pure fallback when no candidate is
        # valid, PLUS extra leave-the-boundary options while riding a circle:
        # following the boundary to a tangent departure point is not always
        # optimal, so the fan lets the search leave the boundary between
        # departure points. ---
        num_directions = config.RADIAL_FAN_DIRECTIONS
        for i in range(num_directions):
            heading_offset = -self._alpha_build + 2 * self._alpha_build * i / (num_directions - 1)
            next_heading = heading + heading_offset
            # Near reserve of this direction — the bite the turn AT P takes out
            # of the new leg. Depends only on the direction, so it is hoisted
            # out of the rung loop. The straight-ahead direction reserves
            # nothing, which is what retired the old WRAP_STEP_M special case.
            near_reserve = math.tan(abs(heading_offset) / 2.0) * self.R
            turn = abs(_angle_diff(next_heading, heading))
            cos_next = math.cos(next_heading)
            sin_next = math.sin(next_heading)
            for rung in self._fan_rungs:
                distance_m = near_reserve + rung
                next_waypoint = (
                    position[0] + distance_m * cos_next,
                    position[1] + distance_m * sin_next,
                )
                # Cheapest gate first. đoản trình is pure arithmetic and rejects
                # 12.2% of the legs that used to reach the fillet-arc gate — the
                # most expensive check in the planner — only AFTER paying for it
                # (measured over 20 scenarios: 10,014 of the 82,032 fan legs that
                # passed the arc gate). Strategy A and v0 already order it this
                # way; this planner was the last one that did not. Pure
                # reordering of side-effect-free predicates: the emitted
                # successors are unchanged.
                budget = self._doan_trinh(current_state, distance_m, turn)
                if budget is None:
                    continue
                if not self._in_bounds(next_waypoint):
                    continue
                if not self._check_collision(position, next_waypoint):
                    continue
                if config.ARC_CLEARANCE_CHECK and not self._corner_arc_clear(
                    heading, position, next_waypoint
                ):
                    continue
                successor = State(next_waypoint, next_heading)
                successor.straight_budget = budget
                # This successor was reached by a fan (Strategy B) expansion:
                # extend the consecutive-B chain (other successor types leave
                # consec_b at its 0 default, resetting the chain).
                successor.consec_b = current_state.consec_b + 1
                successors.append((successor, distance_m + config.TURN_PENALTY_WEIGHT * turn))

        return successors

    def _pivot_candidate(
        self, current: State, node: Point, advance: float
    ) -> tuple[State, float] | None:
        """Build one Strategy-A candidate edge, pivoting along the incoming ray.

        ``advance = 0.0`` is the plain corner at ``current`` and is bit-identical
        to the pre-slide code; ``advance > 0`` flies straight through ``current``
        to ``P' = P + advance*h`` and turns there instead. The returned state's
        waypoint is always ``node``; a positive slide is recorded in
        ``State.via`` and expanded back into the path at reconstruction.

        Args:
            current: The state being expanded.
            node: The candidate waypoint to turn toward.
            advance: How far past ``current`` to slide the pivot (m).

        Returns:
            The ``(successor, cost)`` pair, or ``None`` if any gate rejects the
            edge -- in which case the rejecting gate is recorded in
            ``self._last_reject``.

        Raises:
            TypeError: If ``current`` carries no heading, or fixed-goal mode is
                active without a goal heading.
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

        # Far-end reserve of this segment: 0 for an interior waypoint (its turn
        # is unknown here, deferred to that waypoint's expansion); for the
        # terminal goal the far turn IS known now (onto goal_heading, or 0 in
        # free mode), so reserve it exactly.
        far_reserve = 0.0
        if node is self.goal_state.waypoint:
            if self._free_goal:
                # Free approach: the edge INTO T is the straight seeker run-in.
                # Its USABLE straight length (after the turn fillet at the pivot
                # bites R*tan(turn/2)) must be at least DSS — checking the raw
                # distance would let the fillet steal into the seeker leg.
                # Heading already points at T; _check_collision below keeps it
                # clear; no fixed goal_heading terminal turn.
                if seg_len - self.R * math.tan(turn / 2.0) < self._dss:
                    self._last_reject = "goal"
                    return None
            else:
                # At the final waypoint W_{n-1} the vehicle must turn from the
                # approach heading onto goal_heading; that terminal turn must
                # also be feasible and reserves its bite.
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
            # inside the operating area. (At advance = 0 this is a no-op leg.)
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
        """Retry a rejected candidate from pivots slid FORWARD along the incoming ray.

        Why forward along the ray and not along the outer bisector: the bisector
        rotates the incoming leg, so the parent's corner, its turn reserve and
        every ancestor would have to be re-validated -- and re-validating can
        move them in turn, which is the non-terminating version of this idea.
        Sliding along the ray keeps the incoming DIRECTION and only lengthens
        the leg, so nothing upstream changes and ``_doan_trinh`` only gains
        budget. Nothing is mutated either: this emits an ADDITIONAL successor,
        so there is no fixed point to iterate towards.

        The cost of that safety: with ``h_in`` as the x-axis and
        ``V - P = (a, b)``, the resulting turn is ``|atan2(b, a - d)|``, which
        grows with ``d``. The repair therefore inflates the very fillet it is
        repairing, and ``d`` is capped at ``a - |b|/tan(alpha_max)``. Retry
        positions are parametrised by that resulting turn in tan-uniform
        capability buckets, smallest first so the shortest repair wins.

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
        if cross < 1e-9:  # collinear: there is no corner
            return None

        turn_without_slide = math.atan2(cross, along)
        num_slides = int(config.NUM_PIVOT_SLIDES)
        tan_half_max = math.tan(self._alpha_build / 2.0)
        for i in range(1, num_slides + 1):
            turn_i = 2.0 * math.atan((i / num_slides) * tan_half_max)
            if turn_i <= turn_without_slide:
                continue  # this bucket is behind us (d <= 0)
            # At a right angle the slide reaches the perpendicular foot exactly,
            # and tan() would blow up; below it the foot is pulled back by the
            # cross-track offset. The right-angle test is a degenerate-geometry
            # guard, not a tunable, so it stays an inline epsilon.
            at_right_angle = turn_i >= math.pi / 2.0 - 1e-9
            slide = along if at_right_angle else along - cross / math.tan(turn_i)
            if slide <= config.MIN_PIVOT_SLIDE_M:
                continue
            result = self._pivot_candidate(current, node, slide)
            if result is not None:
                return result
        return None

    def _arc_hop_successors(self, current_state: State) -> list[tuple[State, float]]:
        """Generate successors that ride an inflated circle's boundary.

        For each target (a bitangent departure toward another circle, or a
        tangent from a polygon hull vertex or the goal), hop along the boundary
        arc to the departure point where leaving is tangent-continuous. The
        emitted state IS the departure point; the straight leg to the target is
        found by Strategy A on the next expansion (zero turn there). Cost is the
        true arc length, so the search graph no longer depends on any wrap
        discretisation.

        Also sets ``self._riding`` as a side effect, since it already evaluates
        ``riding_sense`` per circle.

        Args:
            current_state: The state being expanded.

        Returns:
            ``(departure_state, arc_length_cost)`` pairs.

        Raises:
            TypeError: If ``current_state`` carries no heading.
        """
        position = current_state.waypoint
        heading = current_state.heading
        if heading is None:
            raise TypeError("cannot expand a headingless goal target")

        goal_wp = self.goal_state.waypoint
        delta = self._construct_delta
        successors: list[tuple[State, float]] = []
        self._riding = False  # recomputed each expansion; read by get_next_states

        circles: list[CircleGeometry] = self.scenario["circle_obstacles"]
        for idx, (center, radius) in enumerate(circles):
            # All riding geometry is BUILT on the lifted radius r_ride so
            # every constructed chord/tangent keeps >= delta true clearance
            # from the exact-checked inflated boundary.
            r_ride = radius + delta
            sense = ag.riding_sense(position, heading, center, r_ride)
            if sense == 0:
                continue
            # Riding this circle (regardless of whether it yields a departure
            # below) — matches the old any(riding_sense != 0) test exactly.
            self._riding = True
            # A state that is itself an arc-hop departure point of this same
            # circle+sense must not regenerate ride candidates: every departure
            # on this ride was already enumerated from the ride-start state,
            # and regenerating them with shorter residual arcs creates
            # near-duplicate states that collide on the dedup lattice (stale
            # arc_from -> self-crossing reconstruction).
            arc_from = current_state.arc_from
            if (
                arc_from is not None
                and arc_from[0] == center
                and arc_from[1] == r_ride
                and arc_from[3] == sense
            ):
                continue
            phi0 = math.atan2(position[1] - center[1], position[0] - center[0])
            max_wrap = self._max_clear_wrap(center, r_ride, phi0, sense)
            if max_wrap <= 1e-6:
                continue

            deps = self._departure_points(idx, center, radius, r_ride, sense, goal_wp, delta)
            for dep in deps:
                dphi = ag.arc_angle(position, dep, center, sense)
                if dphi < 1e-3 or dphi > max_wrap:
                    continue
                successor = State(dep, ag.tangent_heading(dep, center, sense))
                successor.arc_from = (center, r_ride, position, sense)
                # A ride ENDS ON THE ARC: at the departure point the vehicle is
                # still turning, so there is no straight segment yet for a
                # fillet to bite into. Leaving straight_budget at its `inf`
                # default let _doan_trinh wave through ANY turn <= alpha_max
                # right at the departure — contradicting this method's own
                # contract ("zero turn there") and producing paths the oracle
                # rejects: batch_random_test seed 6 rode a circle and then banked
                # 90 deg on the spot, whose 8000 m reserve had to come out of the
                # ride's last 4535 m expansion chord (usable straight -4683 m,
                # reported as path_self_collision).
                #
                # 0.0 with a -1 m floor means: only a tangent-continuous
                # continuation may leave the ride; any real turn (reserve
                # > 1 m, i.e. alpha > 0.015 deg at R = 8000) is deferred until a
                # straight leg has actually been flown. The floor absorbs float
                # noise in the tangent heading, nothing more.
                successor.straight_budget = 0.0
                successor.min_straight_in = -1.0
                successors.append((successor, r_ride * dphi))
        return successors

    def _departure_points(
        self,
        idx: int,
        center: Point,
        radius: float,
        r_ride: float,
        sense: WrapSense,
        goal_wp: Point,
        delta: float,
    ) -> list[Point]:
        """Return the tangent-continuous departure points for one ride, memoised.

        The candidate list depends only on which circle and sense are ridden,
        not on the current position, so it is computed once per ``(circle,
        sense)`` and reused on every later ride of that same pair.

        Args:
            idx: Index of the ridden circle, used as the cache key.
            center: Centre of the ridden circle.
            radius: Inflated radius of the ridden circle (m).
            r_ride: Lifted riding radius (m).
            sense: Wrap sense of the ride.
            goal_wp: The goal waypoint, itself a departure target.
            delta: Construction lift applied to the other circles (m).

        Returns:
            The departure points on the ridden boundary.
        """
        cache_key = (idx, sense)
        cached = self._dep_cache.get(cache_key)
        if cached is not None:
            return cached

        deps: list[Point] = []
        for c2, r2 in self.scenario["circle_obstacles"]:
            if c2 == center and r2 == radius:
                continue
            # Both circles lifted: the bitangent segment keeps delta clearance
            # from BOTH inflated boundaries.
            deps.extend(
                dep for dep, _arr in ag.bitangent_departures(center, r_ride, c2, r2 + delta, sense)
            )
        for vertex in self._poly_vertices:
            dep = ag.departure_point(vertex, center, r_ride, sense)
            if dep is not None:
                deps.append(dep)
        dep = ag.departure_point(goal_wp, center, r_ride, sense)
        if dep is not None:
            deps.append(dep)
        self._dep_cache[cache_key] = deps
        return deps

    def _max_clear_wrap(self, center: Point, r_ride: float, phi0: float, sense: WrapSense) -> float:
        """Find how far the vehicle may ride a boundary before the corridor is blocked.

        Per ``ARC_SAMPLE_STEP_DEG`` slice the checked region is the TRUE annular
        sector ``[r_ride, r_ride * _ARC_CLEAR_BULGE]`` -- everything an output
        arc-expansion chord (any step <= 45 deg) can occupy. The old
        polyline-at-bulge sweep validated only the thin outer ring and missed
        obstacles intruding the annulus below it (structural gap, seed 155). The
        ridden circle itself never reaches the annulus (its disk ends at
        ``r_ride - CONSTRUCTION_CLEARANCE_M``), so no self-exemption is needed.

        Args:
            center: Centre of the ridden circle.
            r_ride: Lifted riding radius (m).
            phi0: Starting polar angle on the boundary (rad).
            sense: Wrap sense of travel.

        Returns:
            The maximal ridable angle (rad), quantised DOWN to
            ``ARC_SAMPLE_STEP_DEG``, so the result is conservative and
            independent of ``ARC_WAYPOINT_STEP_DEG``.
        """
        r_out = r_ride * _ARC_CLEAR_BULGE
        step = self._arc_sample_step
        phi_prev = phi0
        for k in range(1, self._arc_sample_n + 1):
            phi_next = phi0 + sense * k * step
            probe = (
                center[0] + r_out * math.cos(phi_next),
                center[1] + r_out * math.sin(phi_next),
            )
            if not self._in_bounds(probe) or not self._sector_clear(
                center, r_ride, r_out, phi_prev, phi_next
            ):
                return (k - 1) * step
            phi_prev = phi_next
        return 2.0 * math.pi

    def _sector_clear(
        self, center: Point, r_in: float, r_out: float, phi_a: float, phi_b: float
    ) -> bool:
        """Test whether an annular sector is free of obstacles.

        Circles are exact (zero tolerance) via closed-form radial/angular
        interval overlap, conservative because it uses the disk's polar bounding
        box, a superset of the disk. Polygons use a padded sector quadrilateral
        behind a bbox prefilter, then the interior predicate.

        Args:
            center: Centre of the annulus.
            r_in: Inner radius (m).
            r_out: Outer radius (m).
            phi_a: One angular edge (rad).
            phi_b: The other angular edge (rad).

        Returns:
            ``True`` if nothing intrudes the sector.
        """
        lo, hi = (phi_a, phi_b) if phi_a <= phi_b else (phi_b, phi_a)
        for c2, r2 in self.scenario["circle_obstacles"]:
            dx, dy = c2[0] - center[0], c2[1] - center[1]
            d = math.hypot(dx, dy)
            if d - r2 >= r_out or d + r2 <= r_in:
                continue  # no radial overlap with the annulus
            if d <= r2:
                return False  # annulus centre inside the obstacle
            theta = math.atan2(dy, dx)
            half = math.asin(min(1.0, r2 / d))
            if ag.angular_overlap(theta - half, theta + half, lo, hi):
                return False

        if self._poly_bboxes:
            pts = ag.sector_polygon(center, r_in, r_out, lo, hi)
            qx0 = min(p[0] for p in pts)
            qx1 = max(p[0] for p in pts)
            qy0 = min(p[1] for p in pts)
            qy1 = max(p[1] for p in pts)
            quad: Polygon | None = None
            for i, (bx0, by0, bx1, by1) in enumerate(self._poly_bboxes):
                if qx1 < bx0 or bx1 < qx0 or qy1 < by0 or by1 < qy0:
                    continue  # bbox-disjoint: exactly what STRtree skipped
                if quad is None:
                    quad = Polygon(pts)
                if self._polygons[i].relate_pattern(quad, "T********"):
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
        # Check against circle obstacles — EXACT: any penetration of the
        # inflated boundary is a collision, zero tolerance. Boundary-riding
        # geometry stays acceptable because it is CONSTRUCTED on radius
        # r + CONSTRUCTION_CLEARANCE_M, so legitimate tangent chords carry a
        # true clearance margin instead of a forgiven intrusion. Inlined
        # point-to-SEGMENT distance (squared): the segment length dd is
        # computed once (not once per circle as point_to_line_distance did),
        # and each circle costs a few arithmetic ops with no function-call
        # dispatch. `d² < r²` is exactly the old `dist < r`. Read from the
        # pre-unpacked self._circles (see __init__).
        p1x, p1y = p1
        sx = p2[0] - p1x
        sy = p2[1] - p1y
        dd = sx * sx + sy * sy
        # Chord bbox, also reused by the polygon prefilter below. A centre
        # further than `radius` outside it is further than `radius` from the
        # chord, so the arithmetic below can be skipped outright: measured over
        # 40 scenarios, that is 82.3% of the pairs.
        gx0, gx1 = (p1x, p2[0]) if p1x <= p2[0] else (p2[0], p1x)
        gy0, gy1 = (p1y, p2[1]) if p1y <= p2[1] else (p2[1], p1y)
        if dd == 0.0:  # degenerate segment
            for cx, cy, radius in self._circles:
                if cx + radius < gx0 or cx - radius > gx1 or cy + radius < gy0 or cy - radius > gy1:
                    continue
                relx = cx - p1x
                rely = cy - p1y
                if relx * relx + rely * rely < radius * radius:
                    return False
        else:
            for cx, cy, radius in self._circles:
                if cx + radius < gx0 or cx - radius > gx1 or cy + radius < gy0 or cy - radius > gy1:
                    continue
                relx = cx - p1x
                rely = cy - p1y
                t = (relx * sx + rely * sy) / dd
                if t < 0.0:
                    t = 0.0
                elif t > 1.0:
                    t = 1.0
                ex = relx - t * sx
                ey = rely - t * sy
                if ex * ex + ey * ey < radius * radius:
                    return False

        # Check against polygon obstacles. A segment is blocked ONLY when it
        # enters a polygon's INTERIOR (DE-9IM interior/interior overlap).
        # Merely touching the boundary is allowed: this lets a waypoint sit on
        # a polygon corner (the corners ARE navigation goals) and lets a
        # segment run ALONG an edge to hug the obstacle boundary. The manual
        # bbox loop is the same prefilter STRtree.query performed, minus its
        # per-call dispatch — and the LineString is only constructed when a
        # bbox overlaps, which on open water is almost never.
        line: LineString | None = None
        if self._poly_bboxes:
            for i, (bx0, by0, bx1, by1) in enumerate(self._poly_bboxes):
                if gx1 < bx0 or bx1 < gx0 or gy1 < by0 or by1 < gy0:
                    continue
                if line is None:
                    line = LineString([p1, p2])
                poly = self._polygons[i]
                if not poly.relate_pattern(line, "T********"):
                    continue
                deep = self._polygons_deep[i]
                if not deep.is_empty and deep.relate_pattern(line, "T********"):
                    return False
                if pv.interior_overlap_length(poly, line) > _POLY_TOUCH_TOL_M:
                    return False

        # Safezone containment: the WHOLE chord must stay inside the operating
        # area. Endpoint checks (_in_bounds) are not enough — smoothing shortcuts
        # a chord to a far waypoint, and for a non-convex safezone that chord can
        # exit the area even when both endpoints are inside. `covers` allows the
        # chord to run along the boundary.
        if self._safezone is not None:
            if line is None:
                line = LineString([p1, p2])
            if not self._safezone.covers(line):
                return False
        return True

    def _corner_arc_clear(self, h_in: float, w: Point, w_next: Point, exact: bool = False) -> bool:
        """Test whether the radius-R fillet arc rounding corner ``w`` is clear.

        The arc clears the INFLATED obstacles (raw + SAFE_MARGIN, the same set
        the straight legs clear). It mirrors :func:`core.path_validation.arc_points`
        GEOMETRY so the search weighs the same arc the final oracle will, instead
        of committing to it.

        Hot path -- same prefilter economy as :meth:`_check_collision`: the whole
        arc is sampled once, its bbox computed, and an obstacle is only tested
        when its bbox overlaps the arc's; polygons get ONE LineString for the
        entire arc polyline, not one per segment. An open-water corner therefore
        costs just the sample loop plus bbox compares, with no shapely
        construction at all.

        Args:
            h_in: Incoming heading; the arc is tangent to it (rad).
            w: The corner waypoint.
            w_next: The next waypoint; the arc is tangent to ``w -> w_next``.
            exact: How a polygon "hit" is RESOLVED, and a MEASURED choice rather
                than a principled one. Bare ``'T********'`` also fires on an arc
                that merely grazes a hull edge; measuring the true interior
                overlap tells the two apart. The smoother passes ``True`` because
                it has ONE chord per pair of waypoints and a false hit there
                costs a waypoint that marks no manoeuvre. The search keeps the
                conservative verdict because it has thousands of alternatives and
                the dedup lattice makes quality non-monotone in successor count:
                measured over 300 scenarios, resolving hits exactly in the SEARCH
                too changed one route from 296.75 km to 319.49 km (+7.7%) and
                bought nothing on a second 300-scenario sample. Conservative here
                is also the safe direction -- it only ever declines a candidate.

        Returns:
            ``True`` if the arc is clear. A collinear (no-op) corner has no arc
            and is trivially clear.
        """
        ux, uy = math.cos(h_in), math.sin(h_in)
        vx = w_next[0] - w[0]
        vy = w_next[1] - w[1]
        dv = math.hypot(vx, vy)
        if dv < 1e-9:
            return True
        vx /= dv
        vy /= dv
        cross = ux * vy - uy * vx
        dot = ux * vx + uy * vy
        alpha = abs(math.atan2(cross, dot))
        if alpha < 1e-6:
            return True

        turn_radius = self.R
        tangent = turn_radius * math.tan(alpha / 2.0)
        entry = (w[0] - ux * tangent, w[1] - uy * tangent)  # tangent point, incoming leg
        s = 1.0 if cross > 0 else -1.0
        n_in = (-uy * s, ux * s)
        cx0 = entry[0] + turn_radius * n_in[0]  # arc centre
        cy0 = entry[1] + turn_radius * n_in[1]
        start = math.atan2(entry[1] - cy0, entry[0] - cx0)
        n = self._arc_check_n
        pts: list[Point] = [entry]
        ax0 = ax1 = entry[0]
        ay0 = ay1 = entry[1]
        for k in range(1, n + 1):
            ang = start + s * alpha * (k / n)
            px = cx0 + turn_radius * math.cos(ang)
            py = cy0 + turn_radius * math.sin(ang)
            pts.append((px, py))
            if px < ax0:
                ax0 = px
            elif px > ax1:
                ax1 = px
            if py < ay0:
                ay0 = py
            elif py > ay1:
                ay1 = py

        # Circles: inline point-to-segment, only for a circle whose bbox meets
        # the arc bbox (grown by its radius).
        for cx, cy, radius in self._circles:
            if cx + radius < ax0 or cx - radius > ax1 or cy + radius < ay0 or cy - radius > ay1:
                continue
            r2 = radius * radius
            for j in range(n):
                p1x, p1y = pts[j]
                sx = pts[j + 1][0] - p1x
                sy = pts[j + 1][1] - p1y
                dd = sx * sx + sy * sy
                relx = cx - p1x
                rely = cy - p1y
                if dd == 0.0:
                    if relx * relx + rely * rely < r2:
                        return False
                    continue
                tt = (relx * sx + rely * sy) / dd
                if tt < 0.0:
                    tt = 0.0
                elif tt > 1.0:
                    tt = 1.0
                ex = relx - tt * sx
                ey = rely - tt * sy
                if ex * ex + ey * ey < r2:
                    return False

        # Polygons: one LineString for the whole arc polyline, tested only
        # against polygons whose bbox overlaps the arc bbox.
        if self._poly_bboxes:
            line: LineString | None = None
            for i, (bx0, by0, bx1, by1) in enumerate(self._poly_bboxes):
                if ax1 < bx0 or bx1 < ax0 or ay1 < by0 or by1 < ay0:
                    continue
                if line is None:
                    line = LineString(pts)
                poly = self._polygons[i]
                if not poly.relate_pattern(line, "T********"):
                    continue
                if not exact:
                    return False
                deep = self._polygons_deep[i]
                if not deep.is_empty and deep.relate_pattern(line, "T********"):
                    return False
                if pv.interior_overlap_length(poly, line) > _POLY_TOUCH_TOL_M:
                    return False
        return True

    def _check_fixed_legs(self) -> bool:
        """Test the mandatory ``W_{n-1} -> T`` seeker run-in for collisions.

        Returns:
            ``True`` if the fixed approach leg is clear.
        """
        return self._check_collision(self.goal_state.waypoint, self._target)

    def _in_bounds(self, point: Point) -> bool:
        """Test whether a point lies inside the operating area.

        With a safezone polygon the point must be COVERED by it, so a point
        exactly on the operating-area boundary is allowed. Otherwise, with an
        EXPLICIT ``map_bounds``, the axis-aligned rectangle ``[0, w] x [0, h]``
        applies. With no operating area configured at all this is permissive:
        the legacy 500 km config default is not a real constraint for a scenario
        that lives elsewhere.

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

    def search(self) -> list[PlannerState] | None:
        """Run the kinodynamic A* search.

        Returns:
            The reconstructed waypoints, or ``None`` if no path was found within
            the iteration cap or the wall-clock budget.
        """
        started_at = time.perf_counter()
        budget_s = config.TIME_BUDGET_S

        # Seed every feasible start corner. If none survived construction
        # (takeoff ray blocked / outside the operating area), the start is
        # blocked: fail fast and honestly.
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
            # Two corners can share a lattice cell when the bucket spacing is
            # below STATE_POS_QUANTUM; keep the cheaper g per cell.
            if corner.g_cost < self.g_scores[corner]:
                self.g_scores[corner] = corner.g_cost

        while self.open_set and self.iteration_count < self.max_iterations:
            if budget_s is not None and (time.perf_counter() - started_at) > budget_s:
                break
            self.iteration_count += 1

            _, _, current = heapq.heappop(self.open_set)
            if current in self.closed_set:
                continue
            self.closed_set.add(current)

            # Escape-valve re-arm: the fan's budget (NUM_STRATEGY_B) is
            # global and never replenished, so a map that needs reorientation
            # moves in more than one region can spend the whole budget early
            # and then starve (seed 963: open set exhausts to 0 under budget
            # 3, but succeeds if the budget is unlimited). Re-arm only when
            # the frontier is nearly dead (<=1 state left right after a pop)
            # AND the budget is exhausted, so the per-phase cap of 3 still
            # suppresses fan noise everywhere the search is healthy; it only
            # gets a fresh budget as a last resort against outright failure.
            if len(self.open_set) <= 1 and self.num_strategy_b <= 0:
                self.num_strategy_b = config.NUM_STRATEGY_B

            # Analytic terminal shot: analytically construct a 2-corner manoeuvre
            # straight to the aligned goal and INJECT it into OPEN with its true
            # g (h = 0). A* accepts it via the normal goal-accept block only when
            # it is the cheapest frontier node, so the shot prunes the
            # adverse-heading flood WITHOUT regressing path quality. Fixed-goal
            # mode only.
            if (
                config.GOAL_SHOT_ENABLED
                and not self._free_goal
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
            if dist_to_goal < config.GOAL_THRESHOLD and self._goal_reached(current):
                return self._reconstruct_path(current)

            for next_state, transition_cost in self.get_next_states(current):
                if next_state in self.closed_set:
                    continue

                tentative_g = self.g_scores[current] + transition_cost
                if tentative_g < self.g_scores.get(next_state, float("inf")):
                    # Better path found. The parent is stored on the successor
                    # OBJECT (written exactly once per object — each successor
                    # is freshly constructed), so reconstruction follows the
                    # exact validated transition even when a later, distinct
                    # candidate wins this lattice cell's g-score.
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

        Reaching the goal region is not enough. In FIXED-goal mode the vehicle
        must arrive able to turn onto the approach heading within ``alpha_max``;
        a state that flew straight into the region can be close but badly
        misaligned, and accepting it would force an over-limit terminal turn at
        ``W_{n-1}``. In FREE-goal mode the incoming edge IS the run-in, so its
        USABLE straight length (after the fillet at the previous waypoint bites
        ``R*tan(turn/2)``) must be at least ``DSS`` -- checking only the raw
        distance would accept an edge whose fillet steals into the seeker leg.

        Args:
            current: The state being examined, already within
                ``config.GOAL_THRESHOLD`` of the goal.

        Returns:
            ``True`` if the arrival is kinodynamically feasible.
        """
        if self._free_goal:
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

    def _try_goal_shot(self, current: State) -> State | None:
        """Construct an analytic 2-corner connect from ``current`` to the goal.

        Fixed-goal mode only. Scans 2-corner candidates (turn <= alpha_max at
        ``current`` -> straight -> corner C -> turn <= alpha_max -> arrive at the
        goal waypoint within alpha_max of ``goal_heading``), exact-collision-
        checks the two straight legs, and on the first valid candidate builds the
        corner and goal States with parent pointers linked back to ``current``.

        The emitted manoeuvre is validated identically to any search edge: each
        leg passes :meth:`_check_collision` and the đoản-trình reserves are
        enforced inside ``two_corner_candidates``, so the returned path is valid.

        Args:
            current: The state to shoot from.

        Returns:
            The goal State, ready for reconstruction, or ``None`` if free-goal
            mode is active or no candidate survives.

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
            _MIN_STRAIGHT_M,
            current.straight_budget,
            current.min_straight_in,
            num_dir=config.GOAL_SHOT_DIRS,
            num_cone=config.GOAL_SHOT_CONE,
        )
        base_g = self.g_scores[current]
        for candidate in candidates:
            corner = candidate.corner
            leg1_heading = candidate.leg1_heading
            arrival_heading = candidate.arrival_heading
            if not self._check_collision(current.waypoint, corner):
                continue
            if not self._check_collision(corner, goal_wp):
                continue
            # The shot SYNTHESISES two corners — at `current` and at C — that
            # never pass through get_next_states, so nothing else arc-checks
            # them. That was harmless while inflation carried the alpha_max turn
            # term (it proved every fillet for free); with inflation reduced to
            # SAFE_MARGIN an unchecked corner reaches the final path and the
            # oracle rejects the whole plan (seed 964: path_self_collision).
            # The third corner, at gw turning onto goal_heading, is checked too:
            # the flown leg gw -> T is part of the mission path.
            if config.ARC_CLEARANCE_CHECK:
                if not self._corner_arc_clear(heading, current.waypoint, corner):
                    continue
                if not self._corner_arc_clear(leg1_heading, corner, goal_wp):
                    continue
                if not self._corner_arc_clear(arrival_heading, goal_wp, self._target):
                    continue

            # Leg 1: current -> C (stored heading = leg bearing).
            corner_state = State(corner, leg1_heading)
            corner_state.parent = current
            turn_1 = abs(_angle_diff(leg1_heading, heading))
            corner_state.g_cost = (
                base_g + math.dist(current.waypoint, corner) + config.TURN_PENALTY_WEIGHT * turn_1
            )
            corner_state.straight_budget = candidate.budget_corner
            # Leg 2: C -> goal (stored heading = arrival bearing).
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

    def _reconstruct_path(self, state: State) -> list[PlannerState]:
        """Walk parent pointers back to the start, expanding arc hops and slides.

        Arc-hop transitions become circumscribed-polygon waypoints -- output-time
        discretisation only; the searched route itself is stored in
        ``self.raw_route``. ``via`` pivots ARE real waypoints of the searched
        route (the vehicle flies straight through the parent and turns at the
        pivot), so they belong in ``raw_route`` too.

        Walking per-object parent pointers means every emitted edge is exactly a
        transition that passed ``_check_collision`` and the turn / đoản-trình
        gates at creation time. In particular ``arc_from``'s frozen arc start
        equals the parent's waypoint by object identity -- no healing needed.

        Args:
            state: The goal state reached by the search.

        Returns:
            The flown waypoints in order.

        Raises:
            TypeError: If any state on the chain carries no heading.
        """
        states = [state]
        current = state
        while current.parent is not None:
            current = current.parent
            states.append(current)
        states.reverse()

        raw_route: list[PlannerState] = []
        for st in states:
            if st.via is not None:
                raw_route.append(st.via)
            heading = st.heading
            if heading is None:
                raise TypeError("reconstructed path contains a headingless state")
            raw_route.append((st.waypoint, heading))
        self.raw_route = raw_route

        theta_out = math.radians(config.ARC_WAYPOINT_STEP_DEG)
        path: list[PlannerState] = []
        prev_wp: Point | None = None
        for st in states:
            if st.arc_from is not None and prev_wp is not None:
                center, radius, arc_start, sense = st.arc_from
                dphi = ag.arc_angle(arc_start, st.waypoint, center, sense)
                path.extend(ag.arc_waypoints(center, radius, arc_start, dphi, sense, theta_out))
            if st.via is not None:
                path.append(st.via)
            heading = st.heading
            if heading is None:
                raise TypeError("reconstructed path contains a headingless state")
            path.append((st.waypoint, heading))
            prev_wp = st.waypoint
        return path

    def get_search_stats(self) -> SearchStats:
        """Return the counters describing how the last search ran."""
        return {
            "iterations": self.iteration_count,
            "max_iterations": self.max_iterations,
            "open_set_size": len(self.open_set),
            "closed_set_size": len(self.closed_set),
            "search_failed": self.search_failed,
        }

    def smooth_path(self, path: list[PlannerState]) -> list[PlannerState]:
        """Return the shortest FEASIBLE subsequence of the path, by exact DP.

        The path is short (median 9 waypoints once ``O`` and ``T`` are included,
        21 at the worst measured), so the optimum is affordable and there is no
        reason to settle for a greedy shortcut plus a fallback.

        Why a plain shortcutter cannot do this: đoản trình couples adjacent
        chords through the turn they share, so dropping a waypoint sharpens the
        turn at its neighbour, which retroactively steals straight length from
        the chord INTO that neighbour. A one-chord-at-a-time scan cannot see
        that, so the previous implementation checked the whole result afterwards
        and threw ALL of it away on violation -- measured over 200 seeds, that
        discarded 25% of its own work (46 of 48 discards were turn-arc
        clearance, which the greedy scan never looked at while choosing) and
        captured only 11 km of the 56 km available.

        The DP instead carries the coupling in its state, exactly the way the
        search does with ``State.straight_budget``::

            state  (u, v)  = the last two kept waypoints
            budget         = straight left on chord u->v after its NEAR fillet
            step   (u,v) -> (v,w) reveals the turn at v, which is the FAR fillet
                           of chord u->v and the NEAR fillet of chord v->w

        so every chord is validated with both of its fillets known, at the first
        moment both are known. ``O`` and ``T`` are nodes of the graph rather
        than something a guard patches up afterwards, which is what enforces the
        takeoff leg (``l1 >= L0``, and no turn is available at ``O`` so the first
        chord must lie along ``start_heading``) and the terminal seeker run-in
        (``>= DSS``).

        Several predecessors can reach the same ``(u, v)`` with different
        budgets, so entries are kept under dominance: more budget AND lower cost
        wins. In practice that leaves one or two entries per state.

        Cost is length plus ``SMOOTH_NODE_PENALTY_M`` per kept waypoint. Length
        alone leaves ties the DP breaks by chance: a waypoint the vehicle flies
        STRAIGHT through -- a pivot slide, a fan rung -- adds exactly zero
        length, so it survives or not depending on iteration order. The penalty
        makes the shortest subsequence also the one with the fewest waypoints.

        Args:
            path: The reconstructed waypoints.

        Returns:
            A path in the same shape as the input (interior waypoints, ``O`` not
            included; ``T`` only if the input ended there). Falls back to the
            input unchanged if the DP finds nothing, which can only happen when
            the input itself violates the model.
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
        if count < 3:
            return path
        # The DP is O(m^3) transitions with an arc check each. That is nothing at
        # the sizes this planner produces, but a pathological path should not be
        # allowed to spend the whole time budget here.
        if count > config.SMOOTH_MAX_NODES:
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
        # Fixed-goal approach ray; only meaningful when T really is the terminal
        # node appended above (see the terminal branch of the DP below).
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
            key = (u, v, w)
            hit = arc_memo.get(key)
            if hit is None:
                hit = self._corner_arc_clear(brg[u][v], waypoints[v], waypoints[w], exact=True)
                arc_memo[key] = hit
            return hit

        # by_cur[v][u] = [entry, ...]
        by_cur: defaultdict[int, dict[int, list[_DpEntry]]] = defaultdict(dict)
        for j in range(1, count):
            if not clear[0][j]:
                continue
            # No turn is available at O: the vehicle leaves along start_heading,
            # so the first kept waypoint must sit on that ray.
            if abs(_angle_diff(brg[0][j], start_h)) > _TAKEOFF_RAY_TOL_RAD:
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
                        #
                        # In FIXED-goal mode T is not a plain node: the run-in
                        # must be FLOWN along goal_heading, so the last chord has
                        # to lie on the approach ray — the mirror of the
                        # takeoff-ray rule at O. Without this the DP drops
                        # W_{n-1} whenever that shortens the path and arrives on
                        # the wrong heading (measured: 3/16 named scenarios,
                        # scenario_04 off by 45.5 deg; 16/28 on a fixed-goal
                        # adverse suite, up to 61 deg). The oracle cannot catch
                        # it either — path_validation derives every angle from
                        # waypoint geometry and never compares the arrival
                        # bearing against goal_heading.
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
                        need = l0 if u == 0 else _MIN_STRAIGHT_M
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
            print(f"Search completed: {stats['iterations']}/{stats['max_iterations']} iterations")
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
    preprocessed_scenario: PreprocessedScenario, verbose: bool = False
) -> PlanResult:
    """Plan an autonomous aircraft trajectory end to end.

    ``success`` means the INDEPENDENT oracle accepted the whole mission path,
    not merely that the search returned something.

    Args:
        preprocessed_scenario: Output of
            :func:`core.preprocessing.prepare_scenario`.
        verbose: Print progress information to stdout.

    Returns:
        The plan result.
    """
    if verbose:
        print("Initializing Kinodynamic A*...")
    return KinodynamicAstar(preprocessed_scenario).plan(verbose=verbose)
