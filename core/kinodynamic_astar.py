"""
Kinodynamic A* Path Planning Module
Core algorithm for autonomous aircraft trajectory planning with dynamic constraints
"""

import heapq
import math
from collections import defaultdict
import numpy as np
from shapely.geometry import Polygon, LineString, Point
from shapely.prepared import prep as shp_prep
from shapely.ops import unary_union
import config
import core.spatial_utils as su
import core.preprocessing as prep
import core.arc_geometry as ag
import core.goal_shot as gshot
import core.path_validation as pv


def _angle_diff(a, b):
    """Smallest signed difference a-b normalised to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


# Fixed clearance bulge for riding arcs: circumscribed-polygon vertices for
# any expansion step <= 45 deg stay within r / cos(pi/8) of the center.
_ARC_CLEAR_BULGE = 1.0 / math.cos(math.pi / 8.0)

# Minimum usable straight-flight length (đoản trình) between two waypoints, in
# metres. Matches the threshold historically used by validate_kinodynamics.
# The value lives in config so the two planners cannot drift apart; the local
# alias is kept because it is read on the hot path.
_MIN_STRAIGHT_M = config.MIN_STRAIGHT_M

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


class State:
    """Represents an autonomous aircraft state: (waypoint, heading)"""
    
    def __init__(self, waypoint, heading):
        self.waypoint = waypoint  # (x, y)
        self.heading = heading  # radians
        self.parent = None
        self.g_cost = float('inf')  # Cost from start
        self.h_cost = 0  # Heuristic to goal
        self.arc_from = None  # (center, radius, arc_start_pt, s) if reached via arc hop
        # (pivot, heading) of an INTERMEDIATE straight-through waypoint on the
        # incoming edge, when this state was reached by an along-ray pivot slide
        # (see _slide_pivot): the aircraft flies straight through the parent's
        # candidate corner to `pivot` and only turns there. Expanded back into
        # the path by _reconstruct_path, exactly like arc_from.
        self.via = None
        # Remaining straight length of the INCOMING segment after its near-end
        # turn reserve — the budget still available to the far-end (this
        # waypoint's) turn. Set exactly at creation; the đoản-trình far-end
        # check is deferred to this state's own expansion, where its outgoing
        # turn is known (no alpha_max worst-case). inf = no straight constraint
        # carried in (start state, arc-ride departures).
        self.straight_budget = float('inf')
        # Required straight length of the INCOMING segment (đoản-trình
        # threshold used by the deferred far-end check). Normal states need
        # the generic minimum; seeded start corners override this with L0 so
        # the takeoff stabilization leg is enforced exactly.
        self.min_straight_in = _MIN_STRAIGHT_M
        # Dedup key cache: waypoint/heading never change after construction,
        # and the search hashes/compares each state hundreds of times
        # (measured ~1M state_to_tuple calls on 5 hard seeds). Computed
        # LAZILY on first hash/eq — a free-goal goal_state carries
        # heading=None and must stay constructible (it is never hashed).
        self._key = None
        # Seeded start corner? Corner expansions are exempt from the global
        # Strategy-B valve budget: all K corners expand while the goal is
        # still occluded, so K > NUM_STRATEGY_B would drain the valve at
        # takeoff and starve mid-course reorientation for the whole search.
        self.is_start_corner = False
        # Consecutive Strategy-B (radial fan) count on the path that reached
        # this state: 0 if the incoming edge was NOT a fan expansion, else
        # parent.consec_b + 1. Used only when STRATEGY_B_CONSECUTIVE gates the
        # fan by "no more than NUM_STRATEGY_B fan waypoints in a row on one
        # path" (the per-path semantics) instead of the global valve budget.
        self.consec_b = 0

    def __hash__(self):
        k = self._key
        if k is None:
            k = self._key = su.state_to_tuple(self.waypoint, self.heading)
        return hash(k)

    def __eq__(self, other):
        k = self._key
        if k is None:
            k = self._key = su.state_to_tuple(self.waypoint, self.heading)
        ko = other._key
        if ko is None:
            ko = other._key = su.state_to_tuple(other.waypoint, other.heading)
        return k == ko
    
    def __lt__(self, other):
        """For priority queue comparison"""
        return (self.g_cost + config.HEURISTIC_WEIGHT * self.h_cost) < \
               (other.g_cost + config.HEURISTIC_WEIGHT * other.h_cost)
    
    def __repr__(self):
        return f"State(wp={self.waypoint}, h={math.degrees(self.heading):.1f}°)"


class KinodynamicAstar:
    """Kinodynamic A* path planner for autonomous aircraft trajectory"""
    
    def __init__(self, preprocessed_scenario):
        """
        Initialize the planner.

        Args:
            preprocessed_scenario: Output from preprocessing.prepare_scenario()
        """

        self.scenario = preprocessed_scenario
        self._polygons = [Polygon(coords) for coords in preprocessed_scenario['polygon_obstacles']]
        # Plain-float bboxes for the manual prefilter in _check_collision /
        # _sector_clear. At N <= ~20 polygons a scalar bbox loop beats the
        # STRtree python dispatch, and — the real win — the query geometry
        # (LineString / sector quad) is only CONSTRUCTED when some bbox
        # overlaps: measured ~50% of hard-seed wall time was shapely object
        # construction on queries that hit nothing.
        self._poly_bboxes = [p.bounds for p in self._polygons]
        # Shrunk copies for the deep-hit short-circuit in _check_collision (see
        # config.POLYGON_DEEP_HIT_INSET_M). buffer() can return empty or a
        # MultiPolygon; an empty one simply never short-circuits.
        self._polygons_deep = [p.buffer(-config.POLYGON_DEEP_HIT_INSET_M)
                               for p in self._polygons]
        self._poly_vertices = []
        for poly in self._polygons:
            self._poly_vertices.extend(poly.convex_hull.exterior.coords[:-1])

        # Obstacle sets for the search-time turn-arc clearance check
        # (_corner_arc_clear). These are the INFLATED sets, i.e. raw +
        # SAFE_MARGIN — the same ones the straight-leg check uses.
        #
        # They used to be the RAW sets, because inflation carried a
        # `R*(1/cos(alpha_max/2)-1)` turn term and a fillet arc was designed to
        # bulge into exactly that band. With the turn term gone there is no band
        # to bulge into, and checking arcs against raw would let a turn dip
        # inside the operator's minimum stand-off (measured: 97.9 m of true
        # clearance on a run configured for 500 m). Arcs and straights now both
        # honour SAFE_MARGIN.
        self._arc_circles = self.scenario['circle_obstacles']
        self._arc_polygons = self._polygons
        self._arc_poly_bboxes = self._poly_bboxes
        self._arc_polygons_deep = self._polygons_deep
        self._arc_check_n = max(2, int(config.ARC_CHECK_SAMPLES))

        # Operating areas (safezones). When one or more polygons are supplied the
        # aircraft must stay inside their UNION — both every generated waypoint
        # (_in_bounds) and every edge/chord (_check_collision) are constrained to
        # it. The union (a Polygon or MultiPolygon) is prepared once so the
        # repeated point/segment containment tests on the hot search path are
        # cheap. When absent, fall back to the rectangle from the scenario's
        # map_bounds, else the global config.MAP_WIDTH/HEIGHT (unchanged legacy
        # behaviour).
        safezones = preprocessed_scenario.get('safezones')
        self._safezone = unary_union([Polygon(sz) for sz in safezones]) if safezones else None
        self._safezone_prep = shp_prep(self._safezone) if self._safezone is not None else None
        map_bounds = preprocessed_scenario.get('map_bounds')
        # Only enforce a rectangular bound when one is EXPLICITLY supplied. The
        # global config.MAP_WIDTH/HEIGHT is a legacy 500 km default that is
        # meaningless for scenarios living elsewhere (e.g. real missions at
        # y ~ 1.15e6); enforcing it there would reject every waypoint. When
        # neither a safezone nor an explicit map_bounds is given, _in_bounds is
        # permissive (the search is still bounded by obstacles, candidates,
        # MAX_ITERATIONS and the time budget).
        self._has_explicit_bounds = map_bounds is not None
        self._bounds_w, self._bounds_h = map_bounds if map_bounds else (config.MAP_WIDTH, config.MAP_HEIGHT)

        # Start and goal states
        self.start_state = State(
            preprocessed_scenario['start_state']['waypoint'],
            preprocessed_scenario['start_state']['heading']
        )
        self.start_state.g_cost = 0
        # The incoming O->W1 leg's straight length (near turn at O is 0). The
        # far-end (turn at W1) đoản-trình is then deferred to W1's expansion.
        self.start_state.straight_budget = math.dist(
            preprocessed_scenario['start_pos'], self.start_state.waypoint)

        self.goal_state = State(
            preprocessed_scenario['goal_state']['waypoint'],
            preprocessed_scenario['goal_state']['heading']
        )

        # Free terminal approach mode: goal_heading is None. The search then
        # targets T itself (goal_state.waypoint == goal_pos) and the final edge
        # into T must be a straight run-in of length >= DSS in a search-chosen
        # direction (no fixed approach heading, no terminal turn).
        self._free_goal = preprocessed_scenario.get('goal_heading') is None
        self._dss = preprocessed_scenario['goal_state'].get('engagement_distance', config.DSS)

        # Search variables. NOTE: there is deliberately NO came_from dict —
        # State hashing quantises to a coarse lattice (1000 m / 3°), so a
        # lattice-keyed parent map lets two distinct candidates collide and
        # splice the reconstruction onto a parent whose transition was never
        # collision-checked ("phantom edges"). Parents are stored per-object
        # (State.parent), so every reconstructed edge is exactly a validated
        # transition.
        self.open_set = []
        self.closed_set = set()
        self.g_scores = defaultdict(lambda: float('inf'))
        
        self.iteration_count = 0
        self.max_iterations = config.MAX_ITERATIONS
        self.R = preprocessed_scenario['turn_radius']
        self.alpha_max_rad = preprocessed_scenario['alpha_max_rad']
        # Turn limit used when BUILDING and accepting geometry: padded towards
        # feasibility (SUBTRACTED), so a corner built hard against the limit is
        # still legal when the oracle recomputes the angle from waypoint
        # geometry. Measured, that recomputation overshoots by up to 1.1e-15 rad.
        self._alpha_build = self.alpha_max_rad - config.GEOM_EPS_RAD

        # Seeded start corners: instead of rooting the search at the single
        # worst-case W1 (L0 + R*tan(alpha_max/2) along the takeoff ray), seed
        # K corner states at d_i = L0 + R*tan(a_i/2) with tan-uniform buckets
        # tan(a_i/2) = (i/K)*tan(alpha_max/2), i = 1..K (bucket K == legacy
        # W1, so NUM_START_CORNERS = 1 is exactly legacy). A corner seeded for
        # a_i affords any first turn alpha <= a_i while keeping the takeoff
        # straight l1 >= L0 EXACTLY (straight_budget + min_straight_in = L0).
        # Corners that leave the operating area or whose takeoff leg O->corner
        # collides are NOT seeded — feasibility recovery near obstacles and
        # safezone edges, where the old fixed W1 could land inside an inflated
        # zone and kill the whole plan.
        O = preprocessed_scenario['start_pos']
        u_start = preprocessed_scenario['start_state']['heading']
        L0_start = preprocessed_scenario['start_state'].get('straight_length', config.L0)
        K = max(1, int(config.NUM_START_CORNERS))
        tan_max = math.tan(self._alpha_build / 2.0)
        self.start_corners = []
        for i in range(1, K + 1):
            # +GEOM_EPS_M so the built takeoff straight is strictly longer than
            # L0 and survives the oracle's exact `l1 >= L0` recomputation.
            d_i = L0_start + config.GEOM_EPS_M + self.R * (i / K) * tan_max
            corner = (O[0] + d_i * math.cos(u_start),
                      O[1] + d_i * math.sin(u_start))
            if not self._in_bounds(corner):
                continue
            if not self._check_collision(O, corner):
                continue
            st = State(corner, u_start)
            # True along-ray cost from O. All corners share the same O origin,
            # so relative costs between corners are exact (the legacy single
            # root could use g=0 because its offset was a common constant).
            st.g_cost = d_i
            st.straight_budget = d_i
            st.min_straight_in = L0_start
            st.is_start_corner = True
            self.start_corners.append(st)

        # Pre-computed constants (depend only on R / alpha_max / config, all
        # fixed for the planner's lifetime) hoisted out of the per-expansion
        # hot loops. Values are byte-identical to computing them inline.
        # Fan distance rungs, as the part of a fan leg BEYOND its near reserve:
        # rung j = far reserve for a next turn beta_j + the straight pad, with
        # tan(beta_j/2) = (j/M)*tan(alpha_max/2) (tan-uniform, exactly like the
        # start corners above). Rung j is the shortest leg that still affords a
        # next turn beta <= beta_j, so the search can pick a tight pivot when it
        # only needs a gentle turn instead of always paying the worst case.
        # The last rung (j = M) is the full alpha_max reserve, i.e. the legacy
        # single distance — a pivot that can bridge a constrained goal-approach
        # slot (seed 4: a halved reach forced an 88 km detour there).
        M = max(1, int(config.NUM_FAN_DISTANCES))
        tan_half_max = math.tan(self._alpha_build / 2)
        self._fan_rungs = [self.R * (j / M) * tan_half_max
                           + config.RADIAL_FAN_STEP_M
                           for j in range(1, M + 1)]
        self._arc_sample_step = math.radians(config.ARC_SAMPLE_STEP_DEG)
        self._arc_sample_n = int(round(2.0 * math.pi / self._arc_sample_step))

        # Whether the state being expanded rides any circle boundary; set as a
        # side effect of _arc_hop_successors (which already evaluates
        # riding_sense per circle) so get_next_states need not recompute it.
        self._riding = False

        # Which gate rejected the most recent _pivot_candidate (None on
        # success). Same side-channel style as _riding: it lets the caller ask
        # "was this an ARC rejection?" — the only kind worth retrying with an
        # along-ray slide — without _pivot_candidate returning a richer type on
        # its hot path.
        self._last_reject = None

        # Track if search failed
        self.search_failed = False

        # Search route before arc expansion/smoothing (set on success);
        # used to verify discretisation invariance.
        self.raw_route = None

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
        self._dep_cache = {}

    def heuristic(self, state, goal_state):
        """
        Admissible lower-bound heuristic: straight-line distance to the goal
        waypoint. The old `dist + R * heading_diff` term was inadmissible
        because heading is corrected gradually while travelling, so it
        over-estimated remaining cost and could cause A* to return suboptimal
        paths.
        """
        dx = goal_state.waypoint[0] - state.waypoint[0]
        dy = goal_state.waypoint[1] - state.waypoint[1]
        return math.sqrt(dx * dx + dy * dy)

    def _doan_trinh(self, current, seg_len, turn_at_current, far_reserve=0.0,
                    advance=0.0):
        """Exact đoản-trình (min straight-segment) check for the edge
        current -> new, split across the two events its two turns become known.

        `turn_at_current` (the turn AT `current`, from its incoming heading onto
        this new segment) eats the incoming segment's far end AND the new
        segment's near end. `far_reserve` is the new segment's far-end bite when
        it is already known (terminal turn onto the goal); 0 otherwise, in which
        case that check is deferred to the new state's own expansion.

        `advance` is the along-ray pivot slide (_slide_pivot): the aircraft
        flies straight THROUGH `current` for a further `advance` metres before
        turning, so the incoming straight run is that much longer and the turn
        happens at the slid pivot, not at `current`. Since the direction is
        unchanged this only ever ADDS budget — the constraint cannot be broken
        by sliding, which is the whole point of sliding along the ray.

        Returns the new state's `straight_budget` (new segment length minus the
        near reserve) when both ends have room, else None. The deferred
        far-end check of `current`'s incoming segment uses `current`'s own
        `min_straight_in` threshold (generic minimum, or L0 for a seeded
        start corner).
        """
        reserve = self.R * math.tan(turn_at_current / 2.0)
        # Deferred far-end check of `current`'s incoming segment.
        if current.straight_budget + advance - reserve < current.min_straight_in:
            return None
        budget = seg_len - reserve
        if budget - far_reserve < _MIN_STRAIGHT_M:
            return None
        return budget

    def get_next_states(self, current_state):
        """Dynamic successors: tangent points to circles + polygon hull vertices +
        the goal; radial fan as a fallback when no graph candidate is valid."""
        successors = []
        P = current_state.waypoint
        h = current_state.heading

        # --- Arc-hop: ride any circle boundary this state is tangent to ---
        # All riding/tangent geometry is built on the lifted radius so
        # constructed chords are strictly clear of the exact-checked inflated
        # boundary. Two separate reasons, deliberately added rather than merged:
        # CONSTRUCTION_CLEARANCE_M is an operational stand-off (free to be 0),
        # GEOM_EPS_M is the float-rounding guard that must never be 0.
        delta = config.CONSTRUCTION_CLEARANCE_M + config.GEOM_EPS_M
        successors.extend(self._arc_hop_successors(current_state))
        riding = self._riding      # set as a side effect of _arc_hop_successors

        # --- Strategy A: dynamic tangent / vertex / goal candidates ---
        goal_wp = self.goal_state.waypoint
        candidates = []
        for center, radius in self.scenario['circle_obstacles']:
            candidates.extend(su.circle_tangent_points(P, center, radius + delta))
        candidates.extend(self._poly_vertices)
        candidates.append(goal_wp)

        for node in candidates:
            dx = node[0] - P[0]
            dy = node[1] - P[1]
            if dx * dx + dy * dy < 10000:        # skip ~within 100 m
                continue
            res = self._pivot_candidate(current_state, node, 0.0)
            if (res is None and config.NUM_PIVOT_SLIDES > 0
                    and self._last_reject == 'arc'):
                # Only an ARC rejection is worth retrying. Sliding forward can
                # only INCREASE the turn, so a candidate already over alpha_max
                # is hopeless; and a blocked chord is almost never unblocked by
                # moving the pivot (measured: 1.0%). Retrying every rejection
                # regardless costs 4 extra collision-checked attempts on the
                # candidates that dominate dense maps, which is what made
                # iterations collapse 45335 -> 13851 at K=4.
                res = self._slide_pivot(current_state, node)
            if res is not None:
                successors.append(res)

        # NOTE: it is tempting to skip the fan entirely when the goal is
        # already a valid successor ("the fan is only branching noise in open
        # water" — tests/kinodynamic_arc_hop_test.py::test_no_radial_fan_in_
        # open_water). Measured: that costs seed 4 88 km (534.9 vs 446.9).
        # The search de-duplicates on a coarse lattice (STATE_POS_QUANTUM,
        # STATE_HEADING_QUANTUM_DEG), so it is NOT exactly optimal, and the
        # fan's "redundant" pivots act as lattice diversity rather than noise.
        # Gate the BUDGET here, not whether the fan fires.
        if successors and not riding and not self._check_collision(P, goal_wp):
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

        # --- Strategy B: radial fan — pure fallback when no candidate is
        # valid, PLUS extra leave-the-boundary options while riding a circle:
        # following the boundary to a tangent departure point is not always
        # optimal, so the fan lets the search leave the boundary between
        # departure points. ---
            
        num_directions = config.RADIAL_FAN_DIRECTIONS
        for i in range(num_directions):
            heading_offset = -self._alpha_build + 2 * self._alpha_build * i / (num_directions - 1)
            next_heading = h + heading_offset
            # Near reserve of this direction — the bite the turn AT P takes out
            # of the new leg. Depends only on the direction, so it is hoisted
            # out of the rung loop. The straight-ahead direction reserves
            # nothing, which is what retired the old WRAP_STEP_M special case.
            near_reserve = math.tan(abs(heading_offset) / 2.0) * self.R
            turn = abs(_angle_diff(next_heading, h))
            cos_h = math.cos(next_heading)
            sin_h = math.sin(next_heading)
            for rung in self._fan_rungs:
                distance_m = near_reserve + rung
                next_waypoint = (P[0] + distance_m * cos_h,
                                 P[1] + distance_m * sin_h)
                if not self._in_bounds(next_waypoint):
                    continue
                if not self._check_collision(P, next_waypoint):
                    continue
                if config.ARC_CLEARANCE_CHECK and not self._corner_arc_clear(h, P, next_waypoint):
                    continue
                budget = self._doan_trinh(current_state, distance_m, turn)
                if budget is None:
                    continue
                cost = distance_m + config.TURN_PENALTY_WEIGHT * turn
                nxt = State(next_waypoint, next_heading)
                nxt.straight_budget = budget
                # This successor was reached by a fan (Strategy B) expansion:
                # extend the consecutive-B chain (other successor types leave
                # nxt.consec_b at its 0 default, resetting the chain).
                nxt.consec_b = current_state.consec_b + 1
                successors.append((nxt, cost))

        return successors

    def _pivot_candidate(self, current, node, advance):
        """One Strategy-A candidate edge, pivoting `advance` metres along the
        incoming ray. `advance = 0.0` is the plain corner at `current` and is
        bit-identical to the pre-slide code; `advance > 0` flies straight
        through `current` to P' = P + advance*h and turns there instead.

        Returns `(State, cost)` or None. The returned state's waypoint is
        always `node`; a positive slide is recorded in `State.via` and expanded
        back into the path by _reconstruct_path.
        """
        P = current.waypoint
        h = current.heading
        if advance > 0.0:
            pivot = (P[0] + advance * math.cos(h), P[1] + advance * math.sin(h))
        else:
            pivot = P
        dx = node[0] - pivot[0]
        dy = node[1] - pivot[1]
        seg_len = math.hypot(dx, dy)
        heading_to_node = su.angle_to_heading(pivot, node)
        turn = abs(_angle_diff(heading_to_node, h))
        if turn > self._alpha_build:
            self._last_reject = 'turn'
            return None
        # Far-end reserve of this segment: 0 for an interior waypoint (its
        # turn is unknown here, deferred to that waypoint's expansion); for
        # the terminal goal the far turn IS known now (onto goal_heading, or
        # 0 in free mode), so reserve it exactly.
        far_reserve = 0.0
        if node is self.goal_state.waypoint:
            if self._free_goal:
                # Free approach: the edge INTO T is the straight seeker
                # run-in. Its USABLE straight length (after the turn fillet
                # at the pivot bites R*tan(turn/2)) must be at least DSS —
                # checking the raw distance would let the fillet steal into the
                # seeker leg. Heading already points at T; _check_collision
                # below keeps it clear; no fixed goal_heading terminal turn.
                if seg_len - self.R * math.tan(turn / 2.0) < self._dss:
                    self._last_reject = 'goal'
                    return None
            else:
                # At the final waypoint W_{n-1} the autonomous aircraft must
                # turn from the approach heading onto goal_heading; that
                # terminal turn must also be feasible and reserves its bite.
                final_turn = abs(_angle_diff(self.goal_state.heading, heading_to_node))
                if final_turn > self._alpha_build:
                    self._last_reject = 'goal'
                    return None
                far_reserve = self.R * math.tan(final_turn / 2.0)
        budget = self._doan_trinh(current, seg_len, turn, far_reserve, advance)
        if budget is None:
            self._last_reject = 'doan_trinh'
            return None
        if advance > 0.0:
            # The slide is new flying: the extension leg must be clear and stay
            # inside the operating area. (At advance = 0 this is a no-op leg.)
            if not self._in_bounds(pivot):
                self._last_reject = 'bounds'
                return None
            if not self._check_collision(P, pivot):
                self._last_reject = 'ext_leg'
                return None
        if not self._check_collision(pivot, node):
            self._last_reject = 'los'
            return None
        if config.ARC_CLEARANCE_CHECK and not self._corner_arc_clear(h, pivot, node):
            self._last_reject = 'arc'
            return None
        self._last_reject = None
        nxt = State(node, heading_to_node)
        nxt.straight_budget = budget
        if advance > 0.0:
            # Stored with the INCOMING heading: the aircraft reaches the pivot
            # still on h (it flew straight through P) and turns only there.
            nxt.via = (pivot, h)
        return nxt, advance + seg_len + config.TURN_PENALTY_WEIGHT * turn

    def _slide_pivot(self, current, node):
        """Retry a rejected Strategy-A candidate from pivots slid FORWARD along
        the incoming ray, P' = P + d*h_in.

        Why forward along the ray and not along the outer bisector: the bisector
        rotates the incoming leg, so the parent's corner, its turn reserve and
        every ancestor would have to be re-validated — and re-validating can
        move them in turn, which is the non-terminating version of this idea.
        Sliding along the ray keeps the incoming DIRECTION and only lengthens
        the leg, so nothing upstream changes and _doan_trinh only gains budget.
        Nothing is mutated either: this emits an ADDITIONAL successor, so there
        is no fixed point to iterate towards.

        The cost of that safety: with h_in as the x-axis and V - P = (a, b), the
        resulting turn is |atan2(b, a - d)|, which grows with d. The repair
        therefore inflates the very fillet it is repairing, and d is capped at
        a - |b|/tan(alpha_max) (= a at alpha_max = 90 deg). Retry positions are
        parametrised by that resulting turn in tan-uniform capability buckets
        (config.NUM_PIVOT_SLIDES), smallest first so the shortest repair wins.
        """
        P = current.waypoint
        h = current.heading
        ux, uy = math.cos(h), math.sin(h)
        vx = node[0] - P[0]
        vy = node[1] - P[1]
        a = vx * ux + vy * uy               # along-track component of V - P
        if a <= 0.0:                        # abeam or behind: sliding only hurts
            return None
        b = abs(vx * -uy + vy * ux)         # cross-track component
        if b < 1e-9:                        # collinear: there is no corner
            return None
        alpha0 = math.atan2(b, a)           # the turn without any slide
        K = int(config.NUM_PIVOT_SLIDES)
        tan_half_max = math.tan(self._alpha_build / 2.0)
        for i in range(1, K + 1):
            alpha_i = 2.0 * math.atan((i / K) * tan_half_max)
            if alpha_i <= alpha0:
                continue                    # this bucket is behind us (d <= 0)
            if alpha_i >= math.pi / 2.0 - 1e-9:
                d = a                       # the perpendicular foot
            else:
                d = a - b / math.tan(alpha_i)
            if d <= 1.0:
                continue
            res = self._pivot_candidate(current, node, d)
            if res is not None:
                return res
        return None

    def _arc_hop_successors(self, current_state):
        """Successors that ride an inflated circle's boundary.

        For each target (bitangent departure toward another circle, tangent
        from a polygon hull vertex or the goal), hop along the boundary arc to
        the departure point where leaving is tangent-continuous. The emitted
        state is the departure point itself; the straight leg to the target is
        found by Strategy A on the next expansion (zero turn there). Cost is
        the true arc length. Replaces the old discretized wrap-step model;
        the search graph no longer depends on any wrap discretisation.
        """
        P = current_state.waypoint
        h = current_state.heading
        goal_wp = self.goal_state.waypoint
        delta = config.CONSTRUCTION_CLEARANCE_M + config.GEOM_EPS_M
        successors = []
        self._riding = False   # recomputed each expansion; read by get_next_states
        for idx, (center, radius) in enumerate(self.scenario['circle_obstacles']):
            # All riding geometry is BUILT on the lifted radius r_ride so
            # every constructed chord/tangent keeps >= delta true clearance
            # from the exact-checked inflated boundary.
            r_ride = radius + delta
            s = ag.riding_sense(P, h, center, r_ride)
            if s == 0:
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
            af = current_state.arc_from
            if af is not None and af[0] == center and af[1] == r_ride and af[3] == s:
                continue
            phi0 = math.atan2(P[1] - center[1], P[0] - center[0])
            max_wrap = self._max_clear_wrap(center, r_ride, phi0, s)
            if max_wrap <= 1e-6:
                continue
            cache_key = (idx, s)
            deps = self._dep_cache.get(cache_key)
            if deps is None:
                deps = []
                for c2, r2 in self.scenario['circle_obstacles']:
                    if c2 == center and r2 == radius:
                        continue
                    # Both circles lifted: the bitangent segment keeps delta
                    # clearance from BOTH inflated boundaries.
                    deps.extend(dep for dep, _arr in
                                ag.bitangent_departures(center, r_ride, c2, r2 + delta, s))
                for vertex in self._poly_vertices:
                    dep = ag.departure_point(vertex, center, r_ride, s)
                    if dep is not None:
                        deps.append(dep)
                dep = ag.departure_point(goal_wp, center, r_ride, s)
                if dep is not None:
                    deps.append(dep)
                self._dep_cache[cache_key] = deps
            for dep in deps:
                dphi = ag.arc_angle(P, dep, center, s)
                if dphi < 1e-3 or dphi > max_wrap:
                    continue
                nxt = State(dep, ag.tangent_heading(dep, center, s))
                nxt.arc_from = (center, r_ride, P, s)
                # A ride ENDS ON THE ARC: at the departure point the aircraft is
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
                nxt.straight_budget = 0.0
                nxt.min_straight_in = -1.0
                successors.append((nxt, r_ride * dphi))
        return successors

    def _max_clear_wrap(self, center, r_ride, phi0, s):
        """Maximal angle (rad) the aircraft can ride this boundary from phi0 in
        direction s before the swept corridor hits another obstacle or leaves
        the map. Per ARC_SAMPLE_STEP_DEG slice, the checked region is the TRUE
        annular sector [r_ride, r_ride * _ARC_CLEAR_BULGE] — everything an
        output arc-expansion chord (any step <= 45 deg) can occupy. The old
        polyline-at-bulge sweep validated only the thin outer ring and missed
        obstacles intruding the annulus below it (structural gap, seed 155).
        The ridden circle itself never reaches the annulus (its disk ends at
        r_ride - CONSTRUCTION_CLEARANCE_M), so no self-exemption is needed.
        Conservative: quantised down to ARC_SAMPLE_STEP_DEG; the fixed 45-deg
        bulge keeps the result independent of ARC_WAYPOINT_STEP_DEG."""
        r_out = r_ride * _ARC_CLEAR_BULGE
        step = self._arc_sample_step
        n = self._arc_sample_n
        phi_prev = phi0
        for k in range(1, n + 1):
            phi_next = phi0 + s * k * step
            p = (center[0] + r_out * math.cos(phi_next),
                 center[1] + r_out * math.sin(phi_next))
            if (not self._in_bounds(p)
                    or not self._sector_clear(center, r_ride, r_out, phi_prev, phi_next)):
                return (k - 1) * step
            phi_prev = phi_next
        return 2.0 * math.pi

    def _sector_clear(self, center, r_in, r_out, phi_a, phi_b):
        """True iff the annular sector [r_in, r_out] x [phi_a, phi_b] around
        `center` is free of obstacles. Exact (zero tolerance) for circles via
        closed-form radial/angular interval overlap (conservative: the disk's
        polar bounding box, a superset of the disk); polygons via a padded
        sector quadrilateral (bbox prefilter, then the interior predicate)."""
        lo, hi = (phi_a, phi_b) if phi_a <= phi_b else (phi_b, phi_a)
        for c2, r2 in self.scenario['circle_obstacles']:
            dx, dy = c2[0] - center[0], c2[1] - center[1]
            d = math.hypot(dx, dy)
            if d - r2 >= r_out or d + r2 <= r_in:
                continue                     # no radial overlap with the annulus
            if d <= r2:
                return False                 # annulus center inside the obstacle
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
            quad = None
            for i, (bx0, by0, bx1, by1) in enumerate(self._poly_bboxes):
                if qx1 < bx0 or bx1 < qx0 or qy1 < by0 or by1 < qy0:
                    continue        # bbox-disjoint: exactly what STRtree skipped
                if quad is None:
                    quad = Polygon(pts)
                if self._polygons[i].relate_pattern(quad, 'T********'):
                    return False
        return True

    def _check_collision(self, p1, p2):
        """
        Check if line segment from p1 to p2 collides with any obstacle.
        Returns True if collision-free, False otherwise.
        """

        # Check against circle obstacles — EXACT: any penetration of the
        # inflated boundary is a collision, zero tolerance. Boundary-riding
        # geometry stays acceptable because it is CONSTRUCTED on radius
        # r + CONSTRUCTION_CLEARANCE_M, so legitimate tangent chords carry a
        # true clearance margin instead of a forgiven intrusion. Inlined
        # point-to-SEGMENT distance (squared): the segment length dd is
        # computed once (not once per circle as point_to_line_distance did),
        # and each circle costs a few arithmetic ops with no function-call
        # dispatch. `d² < r²` is exactly the old `dist < r`. Read live from
        # scenario['circle_obstacles'] (no cache) so the check reflects any
        # post-construction obstacle change, as before.
        p1x, p1y = p1
        sx = p2[0] - p1x
        sy = p2[1] - p1y
        dd = sx * sx + sy * sy
        if dd == 0.0:                              # degenerate segment
            for (cx, cy), radius in self.scenario['circle_obstacles']:
                relx = cx - p1x
                rely = cy - p1y
                if relx * relx + rely * rely < radius * radius:
                    return False
        else:
            for (cx, cy), radius in self.scenario['circle_obstacles']:
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
        line = None
        if self._poly_bboxes:
            gx0, gx1 = (p1x, p2[0]) if p1x <= p2[0] else (p2[0], p1x)
            gy0, gy1 = (p1y, p2[1]) if p1y <= p2[1] else (p2[1], p1y)
            for i, (bx0, by0, bx1, by1) in enumerate(self._poly_bboxes):
                if gx1 < bx0 or bx1 < gx0 or gy1 < by0 or by1 < gy0:
                    continue
                if line is None:
                    line = LineString([p1, p2])
                poly = self._polygons[i]
                if not poly.relate_pattern(line, 'T********'):
                    continue
                deep = self._polygons_deep[i]
                if not deep.is_empty and deep.relate_pattern(line, 'T********'):
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

    def _corner_arc_clear(self, h_in, w, w_next, exact=False):
        """True iff the radius-R fillet arc rounding corner `w` clears all RAW
        obstacles. `h_in` is the incoming heading (the arc is tangent to it),
        `w_next` the next waypoint (the arc is tangent to w->w_next). Mirrors
        path_validation._arc_points GEOMETRY so the search weighs the same arc
        the final oracle will (path_self_collision), instead of committing to it;
        how a polygon hit on that arc is RESOLVED depends on `exact` below.
        No-op turn (collinear) => trivially clear.

        Hot path — same prefilter economy as _check_collision: the whole arc is
        sampled once, its bbox computed, and an obstacle is only tested when its
        bbox overlaps the arc's; polygons get ONE LineString for the entire arc
        polyline (not one per segment). An open-water corner therefore costs
        just the sample loop + bbox compares, no shapely construction.

        `exact` picks how a polygon "hit" is resolved, and it is a MEASURED
        choice, not a principled one. Bare 'T********' also fires on an arc that
        merely grazes a hull edge; measuring the true interior overlap (see
        pv.interior_overlap_length) tells the two apart. The smoother passes
        exact=True because it has ONE chord per pair of waypoints and a false
        hit there costs a waypoint that marks no manoeuvre. The search keeps the
        conservative verdict because it has thousands of alternatives and the
        dedup lattice makes quality non-monotone in successor count: measured
        over 300 scenarios, resolving hits exactly in the SEARCH too changed one
        route from 296.75 km to 319.49 km (+7.7%) and bought nothing on a second
        300-scenario sample. Conservative here is also the safe direction -- it
        only ever declines a candidate.
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
        R = self.R
        t = R * math.tan(alpha / 2.0)
        A = (w[0] - ux * t, w[1] - uy * t)          # tangent point on incoming leg
        s = 1.0 if cross > 0 else -1.0
        n_in = (-uy * s, ux * s)
        C = (A[0] + R * n_in[0], A[1] + R * n_in[1])  # arc centre
        start = math.atan2(A[1] - C[1], A[0] - C[0])
        n = self._arc_check_n
        pts = [A]
        ax0 = ax1 = A[0]
        ay0 = ay1 = A[1]
        for k in range(1, n + 1):
            ang = start + s * alpha * (k / n)
            px = C[0] + R * math.cos(ang)
            py = C[1] + R * math.sin(ang)
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
        for (cx, cy), radius in self._arc_circles:
            if cx + radius < ax0 or cx - radius > ax1 or \
                    cy + radius < ay0 or cy - radius > ay1:
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
        if self._arc_poly_bboxes:
            line = None
            for i, (bx0, by0, bx1, by1) in enumerate(self._arc_poly_bboxes):
                if ax1 < bx0 or bx1 < ax0 or ay1 < by0 or by1 < ay0:
                    continue
                if line is None:
                    line = LineString(pts)
                poly = self._arc_polygons[i]
                if not poly.relate_pattern(line, 'T********'):
                    continue
                if not exact:
                    return False
                deep = self._arc_polygons_deep[i]
                if not deep.is_empty and deep.relate_pattern(line, 'T********'):
                    return False
                if pv.interior_overlap_length(poly, line) > _POLY_TOUCH_TOL_M:
                    return False
        return True

    def _check_fixed_legs(self):
        """Validate the fixed takeoff/approach legs W_{n-1}->T.
        Returns True if the fixed legs are collision-free, False otherwise.
        """
        T = self.scenario['goal_pos']
        if not self._check_collision(self.goal_state.waypoint, T):
            return False
        return True

    def _in_bounds(self, point):
        """Check if point is inside the operating area.

        With a safezone polygon: point must be covered by it (`covers`, so a
        point exactly on the operating-area boundary is allowed). Else, with an
        EXPLICIT map_bounds: the axis-aligned rectangle [0, w] x [0, h]. Else
        (no operating area configured): permissive — the legacy 500 km config
        default is not a real constraint for a scenario that lives elsewhere.
        """
        if self._safezone_prep is not None:
            return self._safezone_prep.covers(Point(*point))
        if not self._has_explicit_bounds:
            return True
        x, y = point
        return (0 < x < self._bounds_w and
                0 < y < self._bounds_h)
    
    def search(self):
        """
        Execute Kinodynamic A* search.
        
        Returns:
            Path (list of (waypoint, heading) tuples) or None if no path found
        """
        
        import time
        _start = time.perf_counter()
        _budget = config.TIME_BUDGET_S

        # Initialize
        # Seed every feasible start corner. If none survived construction
        # (takeoff ray blocked / outside the operating area), the start is
        # blocked: fail fast and honestly.
        if not self.start_corners:
            self.search_failed = True
            return None
        for corner in self.start_corners:
            corner.h_cost = self.heuristic(corner, self.goal_state)
            heapq.heappush(self.open_set, (
                corner.g_cost + config.HEURISTIC_WEIGHT * corner.h_cost,
                self.iteration_count,
                corner
            ))
            # Two corners can share a lattice cell when the bucket spacing is
            # below STATE_POS_QUANTUM; keep the cheaper g per cell.
            if corner.g_cost < self.g_scores[corner]:
                self.g_scores[corner] = corner.g_cost

        while self.open_set and self.iteration_count < self.max_iterations:
            if _budget is not None and (time.perf_counter() - _start) > _budget:
                break
            self.iteration_count += 1

            # Pop best state from open set
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

            # Analytic terminal shot: analytically construct a 2-corner maneuver
            # straight to the aligned goal and INJECT it into OPEN with its true
            # g (h = 0). A* accepts it via the normal goal-accept block only when
            # it is the cheapest frontier node, so the shot prunes the
            # adverse-heading flood WITHOUT regressing path quality. Fixed-goal
            # mode only.
            if (config.GOAL_SHOT_ENABLED and not self._free_goal
                    and (self.iteration_count % config.GOAL_SHOT_EVERY_N) == 0):
                shot = self._try_goal_shot(current)
                if shot is not None:
                    tentative_g = shot.g_cost
                    if tentative_g < self.g_scores.get(shot, float('inf')):
                        self.g_scores[shot] = tentative_g
                        shot.h_cost = 0.0
                        heapq.heappush(self.open_set, (
                            shot.g_cost + config.HEURISTIC_WEIGHT * shot.h_cost,
                            self.iteration_count, shot))

            # Check if reached goal
            dist_to_goal = math.sqrt(
                (current.waypoint[0] - self.goal_state.waypoint[0])**2 +
                (current.waypoint[1] - self.goal_state.waypoint[1])**2
            )
            
            if dist_to_goal < config.GOAL_THRESHOLD:
                if self._free_goal:
                    # Free approach: T is reached via the straight run-in edge.
                    # Guard that the incoming edge is a valid run-in — its USABLE
                    # straight length (after the turn fillet at the previous
                    # waypoint bites R*tan(turn/2)) must be >= DSS, so there is
                    # room both to bank onto the run-in AND for the full DSS
                    # seeker leg. Checking only the raw distance would accept an
                    # edge whose fillet steals into the seeker leg, or a fan/wrap
                    # successor that lands on T without a proper run-in.
                    if current.parent is not None:
                        seg = math.dist(current.parent.waypoint, current.waypoint)
                        bearing = su.angle_to_heading(current.parent.waypoint, current.waypoint)
                        turn_at_prev = abs(_angle_diff(bearing, current.parent.heading))
                        usable = seg - self.R * math.tan(turn_at_prev / 2.0)
                        if usable >= self._dss:
                            return self._reconstruct_path(current)
                else:
                    # Reaching the goal region is not enough: the autonomous aircraft must arrive
                    # able to turn onto the approach heading within alpha_max. A state
                    # that wrap-stepped / flew straight into the region can be close but
                    # badly misaligned; accepting it would force a > alpha_max terminal
                    # turn at W_{n-1}. Require an aligned arrival; otherwise keep
                    # searching (the goal_wp candidate provides an aligned approach).
                    approach_turn = abs(_angle_diff(self.goal_state.heading, current.heading))
                    if approach_turn <= self._alpha_build:
                        return self._reconstruct_path(current)
            
            # Expand neighbors
            successors = self.get_next_states(current)
            
            for next_state, transition_cost in successors:
                if next_state in self.closed_set:
                    continue
                
                tentative_g = self.g_scores[current] + transition_cost
                
                if tentative_g < self.g_scores.get(next_state, float('inf')):
                    # Better path found. The parent is stored on the successor
                    # OBJECT (written exactly once per object — each successor
                    # is freshly constructed), so reconstruction follows the
                    # exact validated transition even when a later, distinct
                    # candidate wins this lattice cell's g-score.
                    next_state.parent = current
                    self.g_scores[next_state] = tentative_g
                    next_state.g_cost = tentative_g
                    next_state.h_cost = self.heuristic(next_state, self.goal_state)
                    
                    heapq.heappush(self.open_set, (
                        next_state.g_cost + config.HEURISTIC_WEIGHT * next_state.h_cost,
                        self.iteration_count,
                        next_state
                    ))

        # No path found
        self.search_failed = True
        return None
    
    def _try_goal_shot(self, current):
        """Analytic 2-corner connect from `current` to the aligned goal.

        Fixed-goal mode only. Scans 2-corner candidates (turn <= alpha_max at
        current -> straight -> corner C -> turn <= alpha_max -> arrive at the
        goal waypoint within alpha_max of goal_heading), exact-collision-checks
        the two straight legs, and on the first valid candidate builds the
        corner + goal States with parent pointers linked back to `current`.
        Returns the goal State (ready for _reconstruct_path) or None.

        The emitted maneuver is validated identically to any search edge:
        each leg passes _check_collision and the đoản-trình reserves are
        enforced inside two_corner_candidates, so the returned path is valid.
        """
        if self._free_goal:
            return None
        gw = self.goal_state.waypoint
        gh = self.goal_state.heading
        cands = gshot.two_corner_candidates(
            current.waypoint, current.heading, gw, gh,
            self.R, self._alpha_build, _MIN_STRAIGHT_M,
            current.straight_budget, current.min_straight_in,
            num_dir=config.GOAL_SHOT_DIRS, num_cone=config.GOAL_SHOT_CONE)
        base_g = self.g_scores[current]
        for _total, C, d1, phi, budget_C, budget_W in cands:
            if not self._check_collision(current.waypoint, C):
                continue
            if not self._check_collision(C, gw):
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
                if not self._corner_arc_clear(current.heading, current.waypoint, C):
                    continue
                if not self._corner_arc_clear(d1, C, gw):
                    continue
                T = self.scenario.get('goal_pos')
                if T is not None and not self._corner_arc_clear(phi, gw, T):
                    continue
            # Leg 1: current -> C (stored heading = leg bearing d1).
            c_state = State(C, d1)
            c_state.parent = current
            a1 = abs(_angle_diff(d1, current.heading))
            c_state.g_cost = (base_g + math.dist(current.waypoint, C)
                              + config.TURN_PENALTY_WEIGHT * a1)
            c_state.straight_budget = budget_C
            # Leg 2: C -> goal (stored heading = arrival bearing phi).
            w_state = State(gw, phi)
            w_state.parent = c_state
            a2 = abs(_angle_diff(phi, d1))
            w_state.g_cost = (c_state.g_cost + math.dist(C, gw)
                              + config.TURN_PENALTY_WEIGHT * a2)
            w_state.straight_budget = budget_W
            return w_state
        return None

    def _reconstruct_path(self, state):
        """Reconstruct start->state, expanding arc-hop transitions into
        circumscribed-polygon waypoints (output-time discretisation only;
        the searched route itself is stored in self.raw_route).

        Walks per-object parent pointers, so every emitted edge is exactly a
        transition that passed _check_collision / validate_kinodynamics at
        creation time. In particular, arc_from's frozen arc_start equals the
        parent's waypoint by object identity — no healing needed."""
        states = [state]
        current = state
        while current.parent is not None:
            current = current.parent
            states.append(current)
        states.reverse()

        # `via` pivots are real waypoints of the searched route (the aircraft
        # flies straight through the parent and turns at the pivot), so they
        # belong in raw_route too — unlike arc expansion, which is pure output
        # discretisation.
        self.raw_route = []
        for st in states:
            if st.via is not None:
                self.raw_route.append(st.via)
            self.raw_route.append((st.waypoint, st.heading))

        theta_out = math.radians(config.ARC_WAYPOINT_STEP_DEG)
        path = []
        prev_wp = None
        for st in states:
            if st.arc_from is not None and prev_wp is not None:
                center, radius, arc_start, s = st.arc_from
                dphi = ag.arc_angle(arc_start, st.waypoint, center, s)
                path.extend(ag.arc_waypoints(center, radius, arc_start, dphi, s, theta_out))
            if st.via is not None:
                path.append(st.via)
            path.append((st.waypoint, st.heading))
            prev_wp = st.waypoint
        return path
    
    def smooth_path(self, path):
        """Shortest FEASIBLE subsequence of the reconstructed path, by exact DP.

        The path is short (median 9 waypoints once O and T are included, 21 at
        the worst measured), so the optimum is affordable and there is no reason
        to settle for a greedy shortcut plus a fallback.

        Why a plain shortcutter cannot do this. Đoản trình couples adjacent
        chords through the turn they share: dropping a waypoint sharpens the turn
        at its neighbour, which retroactively steals straight length from the
        chord INTO that neighbour. A one-chord-at-a-time scan cannot see that, so
        the previous implementation checked the whole result afterwards and threw
        ALL of it away on violation — measured over 200 seeds, that discarded
        25% of its own work (46 of 48 discards were turn-arc clearance, which the
        greedy scan never looked at while choosing) and captured only 11 km of
        the 56 km available.

        The DP instead carries the coupling in the state, exactly the way the
        search itself does with `State.straight_budget`:

            state  (u, v)  = the last two kept waypoints
            budget         = straight left on chord u->v after its NEAR fillet
            step   (u,v) -> (v,w) reveals the turn at v, which is the FAR fillet
                           of chord u->v and the NEAR fillet of chord v->w

        so every chord is validated with both of its fillets known, at the first
        moment both are known. O and T are nodes of the graph rather than
        something a guard patches up afterwards, which is what enforces the
        takeoff leg (l1 >= L0, and no turn is available at O so the first chord
        must lie along start_heading) and the terminal seeker run-in (>= DSS).

        Several predecessors can reach the same (u, v) with different budgets, so
        entries are kept under dominance: more budget AND lower cost wins. In
        practice that leaves one or two entries per state.

        Cost is length plus SMOOTH_NODE_PENALTY_M per kept waypoint. Length
        alone leaves ties the DP breaks by chance: a waypoint the aircraft flies
        STRAIGHT through -- a pivot slide, a fan rung -- adds exactly zero
        length, so it survives or not depending on iteration order. The penalty
        makes the shortest subsequence also the one with the fewest waypoints.

        Returns a path in the same shape as the input (interior waypoints, O not
        included; T only if the input ended there). Falls back to the input
        unchanged if the DP finds nothing, which can only happen when the input
        itself violates the model.
        """
        if len(path) < 3:
            return path

        O = self.scenario.get('start_pos')
        T = self.scenario.get('goal_pos')
        wps = [w for w, _ in path]
        head = 0
        if O is not None and math.dist(O, wps[0]) > 1.0:
            wps = [tuple(O)] + wps
            head = 1
        tail = 0
        if T is not None and math.dist(T, wps[-1]) > 1.0:
            wps = wps + [tuple(T)]
            tail = 1
        m = len(wps)
        if m < 3:
            return path
        # The DP is O(m^3) transitions with an arc check each. That is nothing at
        # the sizes this planner produces, but a pathological path should not be
        # allowed to spend the whole time budget here.
        if m > config.SMOOTH_MAX_NODES:
            return path

        R = self.R
        # The true limit, NOT the build reserve. Every corner the DP weighs is
        # defined by waypoints that already exist, and it measures them with the
        # oracle's own formula, bit for bit -- so this gate IS the oracle's
        # check, not a construction that needs padding away from the limit.
        # Using _alpha_build here re-measures the search's own corners against a
        # limit 1e-9 rad tighter than the one they were built at: a corner built
        # AT the limit reads back as alpha_max - 1e-9 + ~3e-15 rad and rejects,
        # which kills every continuation out of it and drops the whole DP into
        # its "found nothing" fallback -- smoothing silently does nothing.
        amax = self.alpha_max_rad
        L0 = self.scenario['start_state'].get('straight_length', config.L0)
        dss = self._dss
        # Length tie-break: a waypoint flown straight through costs zero length,
        # so without this the DP keeps or drops it arbitrarily.
        node_cost = config.SMOOTH_NODE_PENALTY_M
        start_h = self.scenario['start_state']['heading']
        # Fixed-goal approach ray; only meaningful when T really is the terminal
        # node appended above (see the terminal branch of the DP below).
        goal_h = None if (self._free_goal or not tail) else self.scenario.get('goal_heading')

        # Chord geometry, computed once. `clear` uses the planner's own collision
        # test so the smoothed path obeys the safezone too, not just obstacles.
        dist = [[0.0] * m for _ in range(m)]
        brg = [[0.0] * m for _ in range(m)]
        clear = [[False] * m for _ in range(m)]
        for i in range(m):
            for j in range(i + 1, m):
                dist[i][j] = math.dist(wps[i], wps[j])
                brg[i][j] = math.atan2(wps[j][1] - wps[i][1], wps[j][0] - wps[i][0])
                clear[i][j] = self._check_collision(wps[i], wps[j])

        arc_memo = {}

        def arc_ok(u, v, w):
            if not config.ARC_CLEARANCE_CHECK:
                return True
            key = (u, v, w)
            hit = arc_memo.get(key)
            if hit is None:
                hit = self._corner_arc_clear(brg[u][v], wps[v], wps[w], exact=True)
                arc_memo[key] = hit
            return hit

        # entry = (budget, cost, prev_key, prev_entry); by_cur[v][u] = [entry...]
        by_cur = defaultdict(dict)
        for j in range(1, m):
            if not clear[0][j]:
                continue
            # No turn is available at O: the aircraft leaves along start_heading,
            # so the first kept waypoint must sit on that ray.
            if abs(_angle_diff(brg[0][j], start_h)) > _TAKEOFF_RAY_TOL_RAD:
                continue
            by_cur[j][0] = [(dist[0][j], dist[0][j] + node_cost, None, None)]

        best = None
        for v in range(1, m):
            for u, entries in by_cur[v].items():
                for entry in entries:
                    budget, cost = entry[0], entry[1]
                    if v == m - 1:
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
                        if (goal_h is not None
                                and abs(_angle_diff(brg[u][v], goal_h)) > config.APPROACH_RAY_TOL_RAD):
                            continue
                        if budget >= dss and (best is None or cost < best[1]):
                            best = ((u, v), cost, entry)
                        continue
                    for w in range(v + 1, m):
                        if not clear[v][w]:
                            continue
                        turn = abs(_angle_diff(brg[v][w], brg[u][v]))
                        if turn > amax:
                            continue
                        reserve = R * math.tan(turn / 2.0)
                        # Far end of chord u->v, now that the turn at v is known.
                        need = L0 if u == 0 else _MIN_STRAIGHT_M
                        if budget - reserve < need:
                            continue
                        if not arc_ok(u, v, w):
                            continue
                        nb = dist[v][w] - reserve
                        nc = cost + dist[v][w] + node_cost
                        lst = by_cur[w].setdefault(v, [])
                        if any(b >= nb - 1e-9 and c <= nc + 1e-9 for b, c, _, _ in lst):
                            continue
                        lst[:] = [e for e in lst
                                  if not (nb >= e[0] - 1e-9 and nc <= e[1] + 1e-9)]
                        lst.append((nb, nc, (u, v), entry))

        if best is None:
            return path

        key, _cost, entry = best
        seq = []
        while entry is not None:
            seq.append(key[1])
            prev_key, prev_entry = entry[2], entry[3]
            if prev_key is None:
                seq.append(key[0])
                break
            key, entry = prev_key, prev_entry
        seq.reverse()

        out = []
        for idx in range(1 if head else 0, len(seq) - 1 if tail else len(seq)):
            node = seq[idx]
            h = brg[seq[idx - 1]][node] if idx > 0 else path[0][1]
            out.append((wps[node], h))
        return out if len(out) >= 1 else path

    def get_search_stats(self):
        """Return search statistics"""
        return {
            'iterations': self.iteration_count,
            'max_iterations': self.max_iterations,
            'open_set_size': len(self.open_set),
            'closed_set_size': len(self.closed_set),
            'search_failed': self.search_failed,
        }


def _full_mission_path(path, preprocessed):
    """Prepend takeoff O and append goal T so the path spans the whole mission
    O..T (the search only produces the interior W_1..W_{n-1} waypoints).

    Mirrors render.trajectory.build_full_path exactly so the final oracle here
    validates the SAME path the render layer / oracle tests build; the two are
    kept consistent by tests/oracle_validity_test.py (which builds its full
    path via render.trajectory.build_full_path and asserts this function's
    verdict). Kept here rather than imported to avoid a core->render dependency.
    """
    wps = list(path)
    O = preprocessed.get('start_pos')
    T = preprocessed.get('goal_pos')
    sh = preprocessed.get('start_heading', 0.0)
    gh = preprocessed.get('goal_heading', 0.0)
    if O is not None and (not wps or math.dist(O, wps[0][0]) > 1.0):
        wps = [(tuple(O), sh)] + wps
    if T is not None and (not wps or math.dist(T, wps[-1][0]) > 1.0):
        # Free-goal mode leaves goal_heading None; the arrival heading is then
        # the bearing of the final leg into T.
        if gh is None:
            gh = math.atan2(T[1] - wps[-1][0][1], T[0] - wps[-1][0][0]) if wps else 0.0
        wps = wps + [(tuple(T), gh)]
    return wps


def plan_trajectory(preprocessed_scenario, verbose=False):
    """
    High-level function to plan a autonomous aircraft trajectory.
    
    Args:
        preprocessed_scenario: Output from preprocessing.prepare_scenario()
        verbose: Print progress information
    
    Returns:
        Dict with:
            - 'path': List of (waypoint, heading) tuples
            - 'success': Bool indicating if planning succeeded AND the
              returned path (fixed legs + body) is collision-free
            - 'failure_reason': None on success; else one of
              'no_path', 'start_leg_blocked', 'goal_leg_blocked',
              'path_self_collision'
            - 'stats': Search statistics
            - 'planner': KinodynamicAstar object
    """
    
    if verbose:
        print("Initializing Kinodynamic A*...")

    planner = KinodynamicAstar(preprocessed_scenario)

    def _result(path, success, reason):
        return {
            'path': path,
            'success': success,
            'failure_reason': reason,
            'stats': planner.get_search_stats(),
            'planner': planner,
        }

    # Feasibility gates first, each with its own honest reason:
    # - start blocked: every seeded takeoff corner was infeasible (O inside an
    #   inflated obstacle, or the whole takeoff ray collides / leaves the area).
    # - goal leg blocked: the mandatory W_{n-1}->T seeker run-in hits an obstacle.
    if not planner.start_corners:
        return _result(None, False, 'start_leg_blocked')
    if not planner._check_fixed_legs():
        return _result(None, False, 'goal_leg_blocked')

    if verbose:
        print("Starting A* search...")
    path = planner.search()
    if verbose:
        stats = planner.get_search_stats()
        print(f"Search completed: {stats['iterations']}/{stats['max_iterations']} iterations")
    if path is None:
        return _result(None, False, 'no_path')

    path = planner.smooth_path(path)

    # Final whole-path oracle. The search validates each edge as it goes, but
    # arc expansion, smoothing, and the fixed O->W1 / W_{n-1}->T legs (added
    # outside the search) can still leave a full O..T path that violates
    # collision OR the đoản-trình (min-straight) constraint — e.g. two turns
    # ending up too close, so a middle segment's usable straight goes negative.
    # Re-validate the whole path with the INDEPENDENT oracle so success really
    # means oracle-valid; a path that fails is reported as an honest failure,
    # not returned as a silent bad plan. This is exactly the invariant asserted
    # by tests/oracle_validity_test.py. Straight legs are checked against the
    # inflated obstacles (full margin); turn arcs against the raw obstacles
    # (arcs are designed to bulge into the inflation band).
    full = _full_mission_path(path, preprocessed_scenario)
    valid, failure_reason = pv.path_is_valid(
        full,
        preprocessed_scenario['circle_obstacles'],
        preprocessed_scenario['polygon_obstacles'],
        R=preprocessed_scenario['turn_radius'], 
        alpha_max_rad=preprocessed_scenario['alpha_max_rad'],
        L0=preprocessed_scenario['start_state']['straight_length'],
        dss=preprocessed_scenario['goal_state']['engagement_distance'])
    if not valid:
        return _result(path, False, failure_reason)

    if verbose:
        print(f"Path found with {len(path)} waypoints")
    return _result(path, True, None)
