"""
Kinodynamic A* Path Planning Module
Core algorithm for missile trajectory planning with dynamic constraints
"""

import heapq
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
from shapely.geometry import Polygon, LineString, Point
from shapely.prepared import prep as shp_prep
from shapely.ops import unary_union

import config
import core.spatial_utils as su
import core.preprocessing as prep
import core.path_validation as pv

import random


def _angle_diff(a, b):
    """Smallest signed difference a-b normalised to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class State:
    """Represents a missile state: (waypoint, heading)"""
    
    def __init__(self, waypoint, heading):
        self.waypoint = waypoint  # (x, y)
        self.heading = heading  # radians
        # Unit vector of `heading`, cached because _pivot_candidate needs it for
        # EVERY candidate (~120 per expansion) and heading never changes.
        # None in free-goal mode, where goal_state carries no arrival heading;
        # that state is only ever a TARGET, never expanded, so it never needs it.
        self.cos_h = math.cos(heading) if heading is not None else None
        self.sin_h = math.sin(heading) if heading is not None else None
        self.parent = None
        self.g_cost = float('inf')  # Cost from start
        self.h_cost = 0  # Heuristic to goal
        self.straight_budget = float('inf')
        self.min_straight_in = config.MIN_STRAIGHT_M
        self.is_start_corner = False
        # (pivot, heading) of an intermediate straight-through waypoint on the
        # incoming edge when this state was reached by a pivot slide: the
        # aircraft flies straight through the parent and turns only at `pivot`.
        self.via = None
        # Dedup key cache. waypoint/heading never change after construction, and
        # the search hashes/compares each state hundreds of times (measured:
        # 768k state_to_tuple calls, 7.4% of runtime, over 20 scenarios).
        # Computed LAZILY on first hash/eq, because a free-goal goal_state
        # carries heading=None and must stay constructible — it is a target,
        # never hashed.
        self._key = None

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
    """Kinodynamic A* path planner for missile trajectory"""
    
    def __init__(self, preprocessed_scenario):
        self.scenario = preprocessed_scenario        
        # Lift applied when CONSTRUCTING geometry, so a tangent chord is
        # strictly clear of the exact-checked boundary instead of landing on it.
        # Two separate reasons, added not merged: CONSTRUCTION_CLEARANCE_M is an
        # operational stand-off (may be 0), GEOM_EPS_M is the rounding guard
        # (never 0). Without it 43% of tangents fall inside the circle by ~1e-11 m
        # and are rejected by their own collision test.
        self._construct_delta = (config.CONSTRUCTION_CLEARANCE_M
                                 + config.GEOM_EPS_M)

        self._polygons = [Polygon(coords) for coords in preprocessed_scenario['polygon_obstacles']]
        # Plain-float bboxes so a chord/arc can be rejected against an obstacle
        # without building any geometry. Measured over 40 scenarios: 82% of the
        # circle tests in _check_collision and 97.6% of those in
        # _corner_arc_clear are against an obstacle that cannot reach the query
        # at all, and they were costing a full point-to-segment distance each.
        self._circles = [(c[0], c[1], r) for c, r in
                         preprocessed_scenario['circle_obstacles']]
        self._poly_bboxes = [p.bounds for p in self._polygons]
        # Vertex candidates are LIFTED off the hull by the same
        # _construct_delta that circle tangent points are built on. Without it
        # polygons were the one obstacle type whose navigation targets sat
        # EXACTLY on the boundary they must clear, which is what put the
        # boundary case in front of shapely on every chord that ends at, passes
        # through, or runs along a hull edge. A mitre buffer offsets every edge
        # perpendicular by delta and keeps the corner count.
        self._poly_vertices = []
        for poly in self._polygons:
            hull = poly.convex_hull.buffer(self._construct_delta, join_style=2)
            self._poly_vertices.extend(hull.exterior.coords[:-1])

        safezones = preprocessed_scenario.get('safezones')
        self._safezone = unary_union([Polygon(sz) for sz in safezones]) if safezones else None
        self._safezone_prep = shp_prep(self._safezone) if self._safezone is not None else None
        map_bounds = preprocessed_scenario.get('map_bounds')
        self._has_explicit_bounds = map_bounds is not None
        self._bounds_w, self._bounds_h = map_bounds if map_bounds else (config.MAP_WIDTH, config.MAP_HEIGHT)

        # Start and goal states
        self.start_state = State(
            preprocessed_scenario['start_state']['waypoint'],
            preprocessed_scenario['start_state']['heading']
        )
        self.start_state.g_cost = 0
        self.start_state.straight_budget = math.dist(
            preprocessed_scenario['start_pos'], self.start_state.waypoint
        )
        
        self.goal_state = State(
            preprocessed_scenario['goal_state']['waypoint'],
            preprocessed_scenario['goal_state']['heading']
        )

        self._free_goal = preprocessed_scenario.get('goal_heading') is None
        self.distance_to_target = preprocessed_scenario['goal_state'].get('distance_to_target')
        self._dss = preprocessed_scenario['goal_state'].get('engagement_distance', config.DSS)

        # Search variables
        self.open_set = []
        self.closed_set = set()
        self.came_from = {}
        self.g_scores = defaultdict(lambda: float('inf'))
        
        self.iteration_count = 0
        self.nodes_expanded = 0
        self.nodes_generated = 0
        self.max_iterations = config.MAX_ITERATIONS
        self.R = preprocessed_scenario['turn_radius']
        self.alpha_max_rad = preprocessed_scenario['alpha_max_rad']
        # Turn limit used when BUILDING and accepting geometry. Padded towards
        # feasibility (i.e. SUBTRACTED) so a corner constructed hard against the
        # limit still reads as legal when the oracle recomputes the angle from
        # waypoint geometry — measured, that recomputation overshoots by up to
        # 1.1e-15 rad, which an exact oracle would reject.
        self._alpha_build = self.alpha_max_rad - config.GEOM_EPS_RAD
        # Cosine of the widest turn the cheap prefilter may reject outright.
        self._turn_cos_guard = math.cos(min(math.pi, self._alpha_build
                                            + config.TURN_PREFILTER_BAND_RAD))

        # Seeded start corners
        O = preprocessed_scenario['start_pos']
        u_start = preprocessed_scenario['start_state']['heading']
        L0_start = preprocessed_scenario['start_state'].get('straight_length', config.L0)
        K = max(1, int(config.NUM_START_CORNERS))
        tan_max = math.tan(self._alpha_build / 2.0)
        self.start_corners = []
        for i in range(1, K + 1):
            # +GEOM_EPS_M: build l1 strictly longer than L0 so the oracle's
            # exact `l1 >= L0` test survives its own recomputation.
            d_i = L0_start + config.GEOM_EPS_M + self.R * (i / K) * tan_max
            corner = (O[0] + d_i * math.cos(u_start),
                      O[1] + d_i * math.sin(u_start))
            if not self._in_bounds(corner):
                continue
            if not self._check_collision(O, corner):
                continue
            st = State(corner, u_start)
            st.g_cost = d_i
            st.straight_budget = d_i
            st.min_straight_in = L0_start
            st.is_start_corner = True
            self.start_corners.append(st)
        
        # Fan distance rungs
        M = max(1, int(config.NUM_FAN_DISTANCES))
        tan_half_max = math.tan(self._alpha_build / 2)
        self._fan_rungs = [self.R * (j / M) * tan_half_max
                           + config.RADIAL_FAN_STEP_M
                           for j in range(1, M + 1)]
        
        # Which gate rejected the most recent _pivot_candidate (None on
        # success); lets the caller retry only the ones worth sliding.
        self._last_reject = None

        # Track if search failed
        self.search_failed = False

        # Search route before arc expansion/smoothing (set on success);
        # used to verify discretisation invariance.
        self.raw_route = None

        self.num_strategy_b = config.NUM_STRATEGY_B

    def heuristic(self, state, goal_state):
        """Euclidean heuristic — unchanged for guided mode (no local minima risk)."""
        dx = goal_state.waypoint[0] - state.waypoint[0]
        dy = goal_state.waypoint[1] - state.waypoint[1]
        return math.sqrt(dx * dx + dy * dy)

    def _doan_trinh(self, current, seg_len, turn_at_current, far_reserve=0.0,
                    advance=0.0):
        """Exact đoản-trình (min straight-segment) check for the edge
        current -> new, split across the two events its two turns become known.

        `advance` is the pivot slide (_slide_pivot): the aircraft flies straight
        THROUGH `current` for a further `advance` m before turning, so the
        incoming run is that much longer — sliding can only ADD budget.
        """
        reserve = self.R * math.tan(turn_at_current / 2.0)
        # Deferred far-end check of `current`'s incoming segment.
        if current.straight_budget + advance - reserve < current.min_straight_in:
            return None
        budget = seg_len - reserve
        if budget - far_reserve < config.MIN_STRAIGHT_M:
            return None
        return budget
    

    def get_next_states(self, current_state):
        """Dynamic successors with optional guidance cost bonus (Option 3)."""
        successors = []
        P = current_state.waypoint
        h = current_state.heading

        # --- Wrap step: straight continuation off a circle boundary ---
        if self._on_circle_boundary(P):
            nx = (P[0] + config.WRAP_STEP_M * math.cos(h),
                  P[1] + config.WRAP_STEP_M * math.sin(h))
            if self._in_bounds(nx) and self._check_collision(P, nx):
                wrap_cost = config.WRAP_STEP_M
                successors.append((State(nx, h), wrap_cost))

        # --- Strategy A: dynamic tangent / vertex / goal candidates ---
        goal_wp = self.goal_state.waypoint
        candidates = []
        for center, radius in self.scenario['circle_obstacles']:
            candidates.extend(su.circle_tangent_points(
                P, center, radius + self._construct_delta))
        candidates.extend(self._poly_vertices)
        candidates.append(goal_wp)

        for node in candidates:
            dx = node[0] - P[0]
            dy = node[1] - P[1]
            # Degenerate zero-length edge. Compared in SQUARED metres, so the
            # threshold must be squared too — `< config.EPS` was 1e-6 m^2, i.e.
            # a 1 mm cutoff wearing a 1 um label.
            if dx * dx + dy * dy < config.GEOM_EPS_M * config.GEOM_EPS_M:
                continue
            res = self._pivot_candidate(current_state, node, 0.0)
            if (res is None and config.NUM_PIVOT_SLIDES > 0
                    and self._last_reject == 'arc'):
                # Only an ARC rejection is worth retrying: sliding forward can
                # only INCREASE the turn (so a candidate already over alpha_max
                # is hopeless), and a blocked chord is almost never unblocked
                # by moving the pivot.
                res = self._slide_pivot(current_state, node)
            if res is not None:
                successors.append(res)

        if successors and not self._check_collision(P, goal_wp):
            if not current_state.is_start_corner:
                if self.num_strategy_b <= 0:
                    return successors
                self.num_strategy_b -= 1

        # --- Strategy B: radial fan fallback (no graph candidate was valid) ---
        num_directions = config.RADIAL_FAN_DIRECTIONS
        # distance = self.R * math.tan(self.alpha_max_rad / 2) + config.RAIDAL_FAN_STEP_M
        for i in range(num_directions):
            heading_offset = -self._alpha_build + 2 * self._alpha_build * i / (num_directions - 1)
            next_heading = h + heading_offset
            near_reserve = math.tan(abs(heading_offset) / 2.0) * self.R
            turn = abs(_angle_diff(next_heading, h))
            cos_h = math.cos(next_heading)
            sin_h = math.sin(next_heading)
            for rung in self._fan_rungs:
                distance_next = near_reserve + rung
                nx = P[0] + distance_next * cos_h
                ny = P[1] + distance_next * sin_h
                next_waypoint = (nx, ny)
                # Cheapest gate first. đoản trình is pure arithmetic and
                # rejects ~31% of the legs that used to reach the fillet-arc
                # gate — the most expensive check in the planner — after paying
                # for it. Strategy A already orders it this way.
                budget = self._doan_trinh(current_state, distance_next, turn)
                if budget is None:
                    continue
                if not self._in_bounds(next_waypoint):
                    continue
                if not self._check_collision(P, next_waypoint):
                    continue
                # A fan leg turns at P just like a Strategy-A corner does, so its
                # fillet needs the same gate. Skipping it here is invisible on
                # sparse maps and costs 11/1000 missions on dense ones.
                if config.ARC_CLEARANCE_CHECK and not self._corner_arc_clear(h, P, next_waypoint):
                    continue
                cost = distance_next + config.TURN_PENALTY_WEIGHT * turn
                nxt = State(next_waypoint, next_heading)
                nxt.straight_budget = budget
                successors.append((nxt, cost))
        return successors

    def _pivot_candidate(self, current, node, advance):
        """One Strategy-A edge, turning `advance` m further along the incoming
        ray (`advance = 0` is the plain corner). Returns (State, cost) or None,
        recording the rejecting gate in self._last_reject.
        """
        P = current.waypoint
        h = current.heading
        ux = current.cos_h
        uy = current.sin_h
        if advance > 0.0:
            pivot = (P[0] + advance * ux, P[1] + advance * uy)
        else:
            pivot = P
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
            self._last_reject = 'turn'
            return None
        heading_to_node = su.angle_to_heading(pivot, node)
        turn = abs(_angle_diff(heading_to_node, h))
        if turn > self._alpha_build:
            self._last_reject = 'turn'
            return None
        # Far-end reserve: 0 for an interior waypoint (its turn is unknown here);
        # at the goal the terminal turn onto goal_heading is known, so reserve it.
        far_reserve = 0.0
        if node is self.goal_state.waypoint:
            if self._free_goal:
                if seg_len - self.R * math.tan(turn / 2.0) < self._dss:
                    self._last_reject = 'goal'
                    return None
            else:
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
            # inside the operating area.
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
        """Retry an arc-rejected candidate from pivots slid FORWARD along the
        incoming ray, P' = P + d*h_in: the direction is unchanged, so no
        ancestor needs re-validating. The turn |atan2(b, a - d)| grows with d
        (cap d <= a - b/tan(alpha_max)), so retry points are tan-uniform buckets
        of the resulting turn, smallest slide first. See CLAUDE.md for why.
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
        if b < config.GEOM_EPS_M:           # collinear: there is no corner
            return None
        alpha0 = math.atan2(b, a)           # the turn without any slide
        K = int(config.NUM_PIVOT_SLIDES)
        tan_half_max = math.tan(self._alpha_build / 2.0)
        for i in range(1, K + 1):
            alpha_i = 2.0 * math.atan((i / K) * tan_half_max)
            if alpha_i <= alpha0:
                continue                    # this bucket is behind us (d <= 0)
            if alpha_i >= math.pi / 2.0 - config.GEOM_EPS_RAD:
                d = a                       # the perpendicular foot
            else:
                d = a - b / math.tan(alpha_i)
            if d <= config.MIN_PIVOT_SLIDE_M:
                continue
            res = self._pivot_candidate(current, node, d)
            if res is not None:
                return res
        return None

    def _corner_arc_clear(self, h_in, w, w_next):
        """True iff the radius-R fillet arc rounding corner `w` is clear, using
        the oracle's own arc GEOMETRY so the search weighs the same arc the final
        validation will. `h_in` is the incoming heading.

        The whole arc is tested as ONE polyline rather than segment by segment:
        the shapely work (LineString, tree query, safezone `covers`) is what
        costs, and doing it per sub-segment made this gate 41% of run time.

        A polygon hit is taken at face value from 'T********'. That predicate
        also fires on an arc merely GRAZING a hull edge, which the oracle would
        forgive -- so this gate is fractionally stricter than the validator, in
        the safe direction: it can only ever decline a candidate, never approve
        one the oracle rejects. Resolving the difference needs
        pv.interior_overlap_length, and lifting the polygon vertex candidates
        (see _poly_vertices) removed every occurrence it had to resolve.
        """
        prev = (w[0] - math.cos(h_in), w[1] - math.sin(h_in))
        pts = pv._arc_points(prev, w, w_next, self.R, n=config.ARC_CHECK_SAMPLES)
        if not pts:
            return True

        ax0 = min(p[0] for p in pts); ax1 = max(p[0] for p in pts)
        ay0 = min(p[1] for p in pts); ay1 = max(p[1] for p in pts)

        # Circles: scalar point-to-segment, no geometry objects built, and only
        # for a circle whose bbox can reach the arc's. Without the prefilter
        # every arc paid ARC_CHECK_SAMPLES-1 distances against EVERY circle;
        # 97.6% of those pairs cannot touch (measured over 40 scenarios).
        for cx, cy, radius in self._circles:
            if cx + radius < ax0 or cx - radius > ax1 or \
                    cy + radius < ay0 or cy - radius > ay1:
                continue
            center = (cx, cy)
            for j in range(len(pts) - 1):
                if su.point_to_line_distance(center, pts[j], pts[j + 1]) < radius:
                    return False

        line = None
        for idx, (bx0, by0, bx1, by1) in enumerate(self._poly_bboxes):
            if ax1 < bx0 or bx1 < ax0 or ay1 < by0 or by1 < ay0:
                continue
            if line is None:
                line = LineString(pts)
            poly = self._polygons[idx]
            if poly.relate_pattern(line, 'T********'):
                return False

        if self._safezone is not None:
            if line is None:
                line = LineString(pts)
            if not self._safezone.covers(line):
                return False
        return True

    def _check_collision(self, p1, p2):
        """
        Check if line segment from p1 to p2 collides with any obstacle.
        Returns True if collision-free, False otherwise.
        """
        x0, x1 = (p1[0], p2[0]) if p1[0] <= p2[0] else (p2[0], p1[0])
        y0, y1 = (p1[1], p2[1]) if p1[1] <= p2[1] else (p2[1], p1[1])

        # Circles: the exact distance only for one whose bbox can reach the
        # chord's. A centre further than `radius` outside the chord's bounding
        # box is further than `radius` from the chord itself.
        for cx, cy, radius in self._circles:
            if cx + radius < x0 or cx - radius > x1 or \
                    cy + radius < y0 or cy - radius > y1:
                continue
            if su.point_to_line_distance((cx, cy), p1, p2) < radius:
                return False

        # Polygons: same prefilter, and the LineString is only built once some
        # bbox overlaps — on open water it is never built at all.
        line = None
        for idx, (bx0, by0, bx1, by1) in enumerate(self._poly_bboxes):
            if x1 < bx0 or bx1 < x0 or y1 < by0 or by1 < y0:
                continue
            if line is None:
                line = LineString([p1, p2])
            poly = self._polygons[idx]
            if poly.relate_pattern(line, 'T********'):
                return False
        
        if self._safezone is not None:
            if line is None:
                line = LineString([p1, p2])
            if not self._safezone.covers(line):
                return False
        return True
    
    def _on_circle_boundary(self, point, tol=None):
        """True if `point` lies on (within tol of) any inflated circle boundary.

        The tolerance must TRACK the construction lift: tangent points are built
        at r + _construct_delta, so a tol below that classifies every one of them
        as off-boundary and silently switches the wrap step off entirely
        (measured: 13.0% of expansions ride a boundary at a 1e-9 lift, 0.0% at
        1e-3 — which is what made a larger lift look like a 2.4% path cost).
        """
        if tol is None:
            tol = self._construct_delta + config.GEOM_EPS_M
        for center, radius in self.scenario['circle_obstacles']:
            if abs(math.hypot(point[0] - center[0], point[1] - center[1]) - radius) < tol:
                return True
        return False
    def _in_bounds(self, point):
        """Check if point is within map bounds."""
        if self._safezone_prep is not None:
            return self._safezone_prep.covers(Point(*point))
        if not self._has_explicit_bounds:
            return True
        x, y = point
        return (0 < x < self._bounds_w and
                0 < y < self._bounds_h)
    def _check_fixed_legs(self):
        """Validate the fixed takeoff/approach legs W_{n-1} -> T"""
        T = self.scenario['goal_pos']
        if not self._check_collision(self.goal_state.waypoint, T):
            return False
        return True
    
    def search(self):
        """Execute Kinodynamic A* search."""

        import time
        _start = time.perf_counter()
        _budget = config.TIME_BUDGET_S

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
            if corner.g_cost < self.g_scores[corner]:
                self.g_scores[corner] = corner.g_cost

        while self.open_set and self.iteration_count < self.max_iterations:
            if _budget is not None and (time.perf_counter() - _start) > _budget:
                break

            self.iteration_count += 1
            popped = heapq.heappop(self.open_set)
            current = popped[-1]

            if current in self.closed_set:
                continue

            self.closed_set.add(current)
            self.nodes_expanded += 1

            if len(self.open_set) <= 1 and self.num_strategy_b <= 0:
                self.num_strategy_b = config.NUM_STRATEGY_B

            dist_to_goal = math.sqrt(
                (current.waypoint[0] - self.goal_state.waypoint[0])**2 +
                (current.waypoint[1] - self.goal_state.waypoint[1])**2
            )

            if dist_to_goal < config.GOAL_THRESHOLD:
                if self._free_goal:
                    if current.parent is not None:
                        seg = math.dist(current.parent.waypoint, current.waypoint)
                        bearing = su.angle_to_heading(current.parent.waypoint, current.waypoint)
                        turn_at_prev = abs(_angle_diff(bearing, current.parent.heading))
                        usable = seg - self.R * math.tan(turn_at_prev / 2.0)
                        if usable >= self._dss:
                            return self._reconstruct_path(current)
                else:
                    approach_turn = abs(_angle_diff(self.goal_state.heading, current.heading))
                    if approach_turn <= self._alpha_build:
                        return self._reconstruct_path(current)

            successors = self.get_next_states(current)

            for next_state, transition_cost in successors:
                if next_state in self.closed_set:
                    continue

                tentative_g = self.g_scores[current] + transition_cost

                if tentative_g < self.g_scores.get(next_state, float('inf')):
                    next_state.parent = current
                    self.g_scores[next_state] = tentative_g
                    next_state.g_cost = tentative_g
                    next_state.h_cost = self.heuristic(next_state, self.goal_state)


                    heapq.heappush(self.open_set, (next_state.g_cost + config.HEURISTIC_WEIGHT * next_state.h_cost, self.iteration_count, next_state))

        self.search_failed = True
        return None
    
    def _reconstruct_path(self, state):
        """Reconstruct path from start to state, expanding pivot slides: a
        `via` pivot is a real waypoint (the aircraft flies straight through its
        parent and turns there), so it is emitted before its own waypoint."""
        states = []
        current = state
        while current is not None:
            states.append(current)
            current = current.parent
        states.reverse()

        path = []
        for st in states:
            if st.via is not None:
                path.append(st.via)
            path.append((st.waypoint, st.heading))
        return path
    
    def smooth_path(self, path):
        """Shortest FEASIBLE subsequence of the path, by exact DP over O..T.

        A greedy shortcutter cannot do this: đoản trình couples adjacent chords
        through the turn they share, so dropping a waypoint sharpens the turn at
        its neighbour and retroactively steals straight length from the chord
        INTO that neighbour. The DP carries that coupling in the state, the same
        way the search does with State.straight_budget:

            state  (u, v) = last two kept waypoints
            budget        = straight left on chord u->v after its NEAR fillet
            step (u,v) -> (v,w) reveals the turn at v, which is the FAR fillet
                          of u->v and the NEAR fillet of v->w

        so every chord is validated with both fillets known. O and T are nodes
        of the graph, which is what enforces l1 >= L0 (no turn is available at O,
        so the first chord must lie on the takeoff ray) and the >= DSS run-in.
        Entries per state are kept under dominance: more budget AND lower cost.
        Cost charges SMOOTH_NODE_PENALTY_M per kept waypoint, so that among
        equal-length subsequences the shortest one wins -- a waypoint flown
        straight through adds exactly zero length and would otherwise survive by
        chance. Falls back to the input when the DP finds nothing.
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
        if m < 3 or m > config.SMOOTH_MAX_NODES:
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
        # Only meaningful when T really is the terminal node we appended.
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
            hit = arc_memo.get((u, v, w))
            if hit is None:
                hit = self._corner_arc_clear(brg[u][v], wps[v], wps[w])
                arc_memo[(u, v, w)] = hit
            return hit

        # entry = (budget, cost, prev_key, prev_entry); by_cur[v][u] = [entry...]
        by_cur = defaultdict(dict)
        for j in range(1, m):
            if not clear[0][j]:
                continue
            if abs(_angle_diff(brg[0][j], start_h)) > config.TAKEOFF_RAY_TOL_RAD:
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
                        # Fixed goal: T is not a plain node — the run-in must be
                        # flown along goal_heading, so the last chord has to lie
                        # on the approach ray (the mirror of the takeoff-ray
                        # rule at O). Without this the DP drops W_{n-1} and
                        # arrives on the wrong heading.
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
                        need = L0 if u == 0 else config.MIN_STRAIGHT_M
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
        return {
            'iterations': self.iteration_count,
            'max_iterations': self.max_iterations,
            'open_set_size': len(self.open_set),
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
