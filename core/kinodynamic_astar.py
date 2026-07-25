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
from core.heuristic_field import GoalDistanceField


def _angle_diff(a, b):
    """Smallest signed difference a-b normalised to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


# Fixed clearance bulge for riding arcs: circumscribed-polygon vertices for
# any expansion step <= 45 deg stay within r / cos(pi/8) of the center.
_ARC_CLEAR_BULGE = 1.0 / math.cos(math.pi / 8.0)

# Minimum usable straight-flight length (đoản trình) between two waypoints, in
# metres. Matches the threshold historically used by validate_kinodynamics.
_MIN_STRAIGHT_M = 10.0


class State:
    """Represents an autonomous aircraft state: (waypoint, heading)"""
    
    def __init__(self, waypoint, heading):
        self.waypoint = waypoint  # (x, y)
        self.heading = heading  # radians
        self.parent = None
        self.g_cost = float('inf')  # Cost from start
        self.h_cost = 0  # Heuristic to goal
        self.arc_from = None  # (center, radius, arc_start_pt, s) if reached via arc hop
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
        self._poly_vertices = []
        for poly in self._polygons:
            self._poly_vertices.extend(poly.convex_hull.exterior.coords[:-1])

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
        tan_max = math.tan(self.alpha_max_rad / 2.0)
        self.start_corners = []
        for i in range(1, K + 1):
            d_i = L0_start + self.R * (i / K) * tan_max
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

        # Admissible goal-distance field (heuristic tightening). ARMED only
        # when EVERY surviving corner's straight chord to the goal is
        # blocked, and BUILT lazily by search() only after
        # config.HEURISTIC_FIELD_LAZY_ITERS iterations without finishing —
        # proof of real Euclid flooding. Easy maps (most: 734/869 random
        # seeds finish under the threshold) never pay the ~0.3 s build, and
        # any build failure degrades to the plain Euclid heuristic (the
        # field must never be able to fail a plan).
        self._goal_field = None
        self._field_pending = bool(self.start_corners) and all(
            not self._check_collision(c.waypoint, self.goal_state.waypoint)
            for c in self.start_corners)
        # Density-aware eager build: on maps with many obstacles Euclid is very
        # loose, so the field earns its build cost immediately — skip the lazy
        # delay (effective threshold 0) once the obstacle count reaches
        # config.HEURISTIC_FIELD_EAGER_OBSTACLES. Sparse maps keep the lazy
        # threshold, where an early build would be pure overhead. The field is
        # admissible either way, so this changes only speed, not the result.
        n_obstacles = len(self.scenario['circle_obstacles']) + len(self._polygons)
        self._field_lazy_iters = (
            0 if n_obstacles >= config.HEURISTIC_FIELD_EAGER_OBSTACLES
            else config.HEURISTIC_FIELD_LAZY_ITERS)

        # Adversity-gated weighted A*: the adverse-heading flood (goal_heading
        # opposing the start->goal bearing) is the slow case; an inflated weight
        # collapses it but is inadmissible, so apply it ONLY when the terminal is
        # adverse — aligned/normal maps and free-goal mode keep the base weight
        # and thus exact path quality (see config.HEURISTIC_WEIGHT_DENSE).
        self._weight = config.HEURISTIC_WEIGHT
        _gh = preprocessed_scenario.get('goal_heading')
        if not self._free_goal and _gh is not None:
            O = preprocessed_scenario['start_pos']
            T = preprocessed_scenario['goal_pos']
            bearing_OT = math.atan2(T[1] - O[1], T[0] - O[0])
            if abs(_angle_diff(_gh, bearing_OT)) >= math.radians(
                    config.HEURISTIC_WEIGHT_ADVERSE_DEG):
                self._weight = config.HEURISTIC_WEIGHT_DENSE

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
        tan_half_max = math.tan(self.alpha_max_rad / 2)
        self._fan_rungs = [self.R * (j / M) * tan_half_max
                           + config.RADIAL_FAN_STEP_M
                           for j in range(1, M + 1)]
        self._arc_sample_step = math.radians(config.ARC_SAMPLE_STEP_DEG)
        self._arc_sample_n = int(round(2.0 * math.pi / self._arc_sample_step))

        # Whether the state being expanded rides any circle boundary; set as a
        # side effect of _arc_hop_successors (which already evaluates
        # riding_sense per circle) so get_next_states need not recompute it.
        self._riding = False

        # Track if search failed
        self.search_failed = False

        # Search route before arc expansion/smoothing (set on success);
        # used to verify discretisation invariance.
        self.raw_route = None

        self.num_strategy_b = config.NUM_STRATEGY_B

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

    def _build_goal_field(self):
        """Build the armed goal-distance field and re-order OPEN under the
        tightened heuristic. Called by search() at the lazy threshold.

        Mid-search heuristic switching keeps the search exact: both
        heuristics are CONSISTENT (Euclid trivially; the field query is a
        max of 1-Lipschitz cone terms, and every edge costs at least the
        Euclid distance between its endpoints), so nodes closed in the
        Euclid phase already have exact g; from there the standard
        inductive argument applies to the re-ordered frontier under the new
        consistent h. Any build failure disarms the field (pure Euclid).
        """
        self._field_pending = False
        try:
            self._goal_field = GoalDistanceField(self.scenario)
        except Exception:
            self._goal_field = None
            return
        rebuilt = []
        for _f, cnt, st in self.open_set:
            st.h_cost = self.heuristic(st, self.goal_state)
            rebuilt.append((
                st.g_cost + self._weight * st.h_cost, cnt, st))
        heapq.heapify(rebuilt)
        self.open_set = rebuilt

    def heuristic(self, state, goal_state):
        """
        Admissible lower-bound heuristic: straight-line distance to the goal
        waypoint, tightened by the goal-distance field (max of two lower
        bounds is a lower bound) when one was built. The old
        `dist + R * heading_diff` term was inadmissible because heading is
        corrected gradually while travelling, so it over-estimated remaining
        cost and could cause A* to return suboptimal paths.
        """
        dx = goal_state.waypoint[0] - state.waypoint[0]
        dy = goal_state.waypoint[1] - state.waypoint[1]
        h = math.sqrt(dx * dx + dy * dy)
        if self._goal_field is not None:
            hf = self._goal_field.query(state.waypoint)
            if hf > h:
                return hf
        return h
    
    def _doan_trinh(self, current, seg_len, turn_at_current, far_reserve=0.0):
        """Exact đoản-trình (min straight-segment) check for the edge
        current -> new, split across the two events its two turns become known.

        `turn_at_current` (the turn AT `current`, from its incoming heading onto
        this new segment) eats the incoming segment's far end AND the new
        segment's near end. `far_reserve` is the new segment's far-end bite when
        it is already known (terminal turn onto the goal); 0 otherwise, in which
        case that check is deferred to the new state's own expansion.

        Returns the new state's `straight_budget` (new segment length minus the
        near reserve) when both ends have room, else None. The deferred
        far-end check of `current`'s incoming segment uses `current`'s own
        `min_straight_in` threshold (generic minimum, or L0 for a seeded
        start corner).
        """
        reserve = self.R * math.tan(turn_at_current / 2.0)
        # Deferred far-end check of `current`'s incoming segment.
        if current.straight_budget - reserve < current.min_straight_in:
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
        # All riding/tangent geometry is built on r + CONSTRUCTION_CLEARANCE_M
        # so constructed chords are strictly clear of the exact-checked
        # inflated boundary (see config.CONSTRUCTION_CLEARANCE_M).
        delta = config.CONSTRUCTION_CLEARANCE_M
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
            heading_to_node = su.angle_to_heading(P, node)
            turn = abs(_angle_diff(heading_to_node, h))
            if turn > self.alpha_max_rad:
                continue
            # Far-end reserve of this segment: 0 for an interior waypoint (its
            # turn is unknown here, deferred to that waypoint's expansion); for
            # the terminal goal the far turn IS known now (onto goal_heading, or
            # 0 in free mode), so reserve it exactly.
            far_reserve = 0.0
            if node is goal_wp:
                if self._free_goal:
                    # Free approach: the edge INTO T is the straight seeker
                    # run-in. Its USABLE straight length (after the turn fillet
                    # at P bites R*tan(turn/2)) must be at least DSS — checking
                    # the raw distance would let the fillet steal into the
                    # seeker leg. Heading already points at T; _check_collision
                    # below keeps it clear; no fixed goal_heading terminal turn.
                    if math.hypot(dx, dy) - self.R * math.tan(turn / 2.0) < self._dss:
                        continue
                else:
                    # At the final waypoint W_{n-1} the autonomous aircraft must
                    # turn from the approach heading onto goal_heading; that
                    # terminal turn must also be feasible and reserves its bite.
                    final_turn = abs(_angle_diff(self.goal_state.heading, heading_to_node))
                    if final_turn > self.alpha_max_rad:
                        continue
                    far_reserve = self.R * math.tan(final_turn / 2.0)
            budget = self._doan_trinh(current_state, math.hypot(dx, dy), turn, far_reserve)
            if budget is None:
                continue
            if not self._check_collision(P, node):
                continue
            cost = math.hypot(dx, dy) + config.TURN_PENALTY_WEIGHT * turn
            nxt = State(node, heading_to_node)
            nxt.straight_budget = budget
            successors.append((nxt, cost))

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
            if not current_state.is_start_corner:
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
            heading_offset = -self.alpha_max_rad + 2 * self.alpha_max_rad * i / (num_directions - 1)
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
                budget = self._doan_trinh(current_state, distance_m, turn)
                if budget is None:
                    continue
                cost = distance_m + config.TURN_PENALTY_WEIGHT * turn
                nxt = State(next_waypoint, next_heading)
                nxt.straight_budget = budget
                successors.append((nxt, cost))

        return successors

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
        delta = config.CONSTRUCTION_CLEARANCE_M
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
                if self._polygons[i].relate_pattern(line, 'T********'):
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
                corner.g_cost + self._weight * corner.h_cost,
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

            # Lazy heuristic tightening: the search running this long is the
            # proof of Euclid flooding that justifies the field's build cost.
            if (self._field_pending and
                    self.iteration_count >= self._field_lazy_iters):
                self._build_goal_field()

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
                            shot.g_cost + self._weight * shot.h_cost,
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
                        if usable >= self._dss - config.EPS:
                            return self._reconstruct_path(current)
                else:
                    # Reaching the goal region is not enough: the autonomous aircraft must arrive
                    # able to turn onto the approach heading within alpha_max. A state
                    # that wrap-stepped / flew straight into the region can be close but
                    # badly misaligned; accepting it would force a > alpha_max terminal
                    # turn at W_{n-1}. Require an aligned arrival; otherwise keep
                    # searching (the goal_wp candidate provides an aligned approach).
                    approach_turn = abs(_angle_diff(self.goal_state.heading, current.heading))
                    if approach_turn <= self.alpha_max_rad:
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
                        next_state.g_cost + self._weight * next_state.h_cost,
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
        # Alignment gate (config.GOAL_SHOT_ALIGN_GATE): when the approach
        # bearing to the goal waypoint is already within alpha_max of
        # goal_heading, a 1-corner terminal (the normal Strategy-A goal
        # candidate) can arrive legally, so building the 625-candidate grid is
        # wasted work. Skips ~100% on aligned maps, ~0% on the adverse maps
        # where the shot is load-bearing (measured; see config).
        if config.GOAL_SHOT_ALIGN_GATE:
            br = math.atan2(gw[1] - current.waypoint[1],
                            gw[0] - current.waypoint[0])
            if abs(_angle_diff(gh, br)) <= self.alpha_max_rad:
                return None
        cands = gshot.two_corner_candidates(
            current.waypoint, current.heading, gw, gh,
            self.R, self.alpha_max_rad, _MIN_STRAIGHT_M,
            current.straight_budget, current.min_straight_in,
            num_dir=config.GOAL_SHOT_DIRS, num_cone=config.GOAL_SHOT_CONE)
        base_g = self.g_scores[current]
        for _total, C, d1, phi, budget_C, budget_W in cands:
            if not self._check_collision(current.waypoint, C):
                continue
            if not self._check_collision(C, gw):
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

        self.raw_route = [(st.waypoint, st.heading) for st in states]

        theta_out = math.radians(config.ARC_WAYPOINT_STEP_DEG)
        path = []
        prev_wp = None
        for st in states:
            if st.arc_from is not None and prev_wp is not None:
                center, radius, arc_start, s = st.arc_from
                dphi = ag.arc_angle(arc_start, st.waypoint, center, s)
                path.extend(ag.arc_waypoints(center, radius, arc_start, dphi, s, theta_out))
            path.append((st.waypoint, st.heading))
            prev_wp = st.waypoint
        return path
    
    def smooth_path(self, path):
        """
        Smooth the path by shortcutting to the FARTHEST reachable waypoint.

        The old greedy only tried to skip ONE waypoint at a time (anchor ->
        path[i+1]) and appended path[i] the moment that single-step shortcut
        failed — so a clear, feasible long jump anchor -> path[i+k] was never
        tested once an intermediate onward-turn blocked the one-ahead step,
        leaving detours in the path. Here, from each kept anchor we scan from
        the farthest waypoint inward and jump straight to the farthest one whose
        direct chord is (a) collision-free (exact), (b) kinodynamically valid at
        the anchor (turn <= alpha_max + đoản trình), and (c) whose onward turn at
        the target stays feasible (terminal turn onto goal_heading for the last
        waypoint). Endpoints path[0]/path[-1] are preserved; every kept edge is
        exact-collision-checked and validated, so the result stays valid.

        Args:
            path: List of (waypoint, heading) tuples

        Returns:
            Smoothed path
        """
        if len(path) < 3:
            return path

        n = len(path)
        smoothed = [path[0]]
        i = 0
        while i < n - 1:
            anchor_wp = smoothed[-1][0]
            # Geometric inbound heading at the anchor (bearing from the previous
            # KEPT waypoint); the first anchor uses the start heading.
            if len(smoothed) >= 2:
                anchor_h = su.angle_to_heading(smoothed[-2][0], anchor_wp)
            else:
                anchor_h = path[0][1]

            best = i + 1
            for j in range(n - 1, i, -1):
                target_wp = path[j][0]
                heading_to = su.angle_to_heading(anchor_wp, target_wp)
                # First-anchor L0 guard: when the anchor is path[0] (the
                # seeded takeoff corner), a shortcut changes the first turn
                # alpha_1, and the incoming O->corner leg must still keep
                # l1 = d(O, corner) - R*tan(alpha_1/2) >= L0. Legacy code was
                # safe implicitly via the alpha_max reserve in W1's placement;
                # minimal corners need the guard explicit.
                if len(smoothed) == 1:
                    a1_new = abs(_angle_diff(heading_to, anchor_h))
                    d0 = math.dist(self.scenario['start_pos'], anchor_wp)
                    l0_req = self.scenario['start_state']['straight_length']
                    if d0 - self.R * math.tan(a1_new / 2.0) < l0_req - config.EPS:
                        continue
                # Onward waypoint after target = the far-end turn of the
                # anchor->target chord. Known here (path[j+1], or the goal leg),
                # so pass it to validate BOTH ends of the chord exactly instead
                # of the alpha_max worst case. For the free-goal terminal there
                # is no onward turn (the chord is the straight run-in into T).
                if j == n - 1 and self._free_goal:
                    onward_wp = onward_h = None
                elif j == n - 1:
                    # The flown leg is path[-1] -> T = goal_pos at goal_heading;
                    # use those, not the offset goal_state.waypoint which sits up
                    # to GOAL_THRESHOLD away and would spuriously fail the length.
                    onward_wp = self.scenario['goal_pos']
                    onward_h = self.scenario['goal_heading']
                else:
                    onward_wp = path[j + 1][0]
                    onward_h = su.angle_to_heading(target_wp, onward_wp)

                is_valid, _ = prep.validate_kinodynamics(
                    anchor_wp, anchor_h, target_wp, heading_to,
                    w_next_next=onward_wp, heading_next_next=onward_h,
                    R=self.R, alpha_max=self.alpha_max_rad)
                if not is_valid:
                    continue

                # Onward feasibility: the terminal run-in must stay >= DSS in
                # free mode; otherwise the target->onward turn must be flyable.
                if j == n - 1 and self._free_goal:
                    # Free run-in: USABLE straight length (after the turn fillet
                    # at the anchor) must stay >= DSS, matching the search's
                    # goal-candidate rule, so a shortcut cannot steal the fillet
                    # bite out of the seeker leg.
                    turn_anchor = abs(_angle_diff(heading_to, anchor_h))
                    usable = math.dist(anchor_wp, target_wp) - self.R * math.tan(turn_anchor / 2.0)
                    is_next_valid = usable >= self._dss - config.EPS
                else:
                    is_next_valid, _ = prep.validate_kinodynamics(
                        target_wp, heading_to, onward_wp, onward_h,
                        R=self.R, alpha_max=self.alpha_max_rad)
                if is_next_valid and self._check_collision(anchor_wp, target_wp):
                    best = j
                    break

            smoothed.append(path[best])
            i = best

        return smoothed
    
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
    valid = pv.path_is_valid(
        full,
        preprocessed_scenario['circle_obstacles'],
        preprocessed_scenario['polygon_obstacles'],
        planner.R, planner.alpha_max_rad, config.L0, config.DSS,
        raw_circle_obstacles=preprocessed_scenario.get('raw_circle_obstacles'),
        raw_polygon_obstacles=preprocessed_scenario.get('raw_polygon_obstacles'),
        circle_tol=config.CIRCLE_GRAZE_TOL_M)
    if not valid:
        return _result(path, False, 'path_self_collision')

    if verbose:
        print(f"Path found with {len(path)} waypoints")
    return _result(path, True, None)
