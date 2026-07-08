"""
Kinodynamic A* Path Planning Module
Core algorithm for autonomous aircraft trajectory planning with dynamic constraints
"""

import heapq
import math
from collections import defaultdict
import numpy as np
from shapely.geometry import Polygon, LineString
from shapely import STRtree

import config
import core.spatial_utils as su
import core.preprocessing as prep
import core.arc_geometry as ag


def _angle_diff(a, b):
    """Smallest signed difference a-b normalised to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


# Fixed clearance bulge for riding arcs: circumscribed-polygon vertices for
# any expansion step <= 45 deg stay within r / cos(pi/8) of the center.
_ARC_CLEAR_BULGE = 1.0 / math.cos(math.pi / 8.0)


class State:
    """Represents an autonomous aircraft state: (waypoint, heading)"""
    
    def __init__(self, waypoint, heading):
        self.waypoint = waypoint  # (x, y)
        self.heading = heading  # radians
        self.parent = None
        self.g_cost = float('inf')  # Cost from start
        self.h_cost = 0  # Heuristic to goal
        self.arc_from = None  # (center, radius, arc_start_pt, s) if reached via arc hop
    
    def __hash__(self):
        return hash(su.state_to_tuple(self.waypoint, self.heading))
    
    def __eq__(self, other):
        return (su.state_to_tuple(self.waypoint, self.heading) ==
                su.state_to_tuple(other.waypoint, other.heading))
    
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
        self._poly_tree = STRtree(self._polygons) if self._polygons else None
        self._poly_vertices = []
        for poly in self._polygons:
            self._poly_vertices.extend(poly.convex_hull.exterior.coords[:-1])

        # Start and goal states
        self.start_state = State(
            preprocessed_scenario['start_state']['waypoint'],
            preprocessed_scenario['start_state']['heading']
        )
        self.start_state.g_cost = 0
        
        self.goal_state = State(
            preprocessed_scenario['goal_state']['waypoint'],
            preprocessed_scenario['goal_state']['heading']
        )
        
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

    def heuristic(self, state, goal_state):
        """
        Admissible Euclidean lower-bound heuristic.
        Returns straight-line distance to the goal waypoint.
        The old `dist + R * heading_diff` term was inadmissible because heading
        is corrected gradually while travelling, so it over-estimated remaining
        cost and could cause A* to return suboptimal paths.
        """
        dx = goal_state.waypoint[0] - state.waypoint[0]
        dy = goal_state.waypoint[1] - state.waypoint[1]
        return math.sqrt(dx * dx + dy * dy)
    
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
        riding = any(ag.riding_sense(P, h, center, radius + delta) != 0
                     for center, radius in self.scenario['circle_obstacles'])

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
            # At the final waypoint W_{n-1} the autonomous aircraft must turn from the approach
            # heading onto goal_heading; that terminal turn must also be feasible.
            if node is goal_wp:
                final_turn = abs(_angle_diff(self.goal_state.heading, heading_to_node))
                if final_turn > self.alpha_max_rad:
                    continue
            is_valid, _ = prep.validate_kinodynamics(
                P, h, node, heading_to_node, R=self.R, alpha_max=self.alpha_max_rad)
            if not is_valid:
                continue
            if not self._check_collision(P, node):
                continue
            cost = math.hypot(dx, dy) + config.TURN_PENALTY_WEIGHT * turn
            successors.append((State(node, heading_to_node), cost))

        if successors and not riding and not self._check_collision(P, goal_wp):
            # Escape valve: while the goal is occluded, a few budgeted fan
            # expansions provide cheap reorientation moves (e.g. an adverse
            # initial heading) that tangent/vertex candidates cannot express;
            # without this the search can commit to a long detour (seed 319:
            # 978.8 km vs 728.9 km with the valve).
            if self._check_collision(P, goal_wp) or self.num_strategy_b <= 0:
                return successors
            self.num_strategy_b -= 1

        # --- Strategy B: radial fan — pure fallback when no candidate is
        # valid, PLUS extra leave-the-boundary options while riding a circle:
        # following the boundary to a tangent departure point is not always
        # optimal, so the fan lets the search leave the boundary between
        # departure points. ---
            
        num_directions = config.RADIAL_FAN_DIRECTIONS
        distance = 2 * self.R * math.tan(self.alpha_max_rad / 2) + config.RADIAL_FAN_STEP_M
        for i in range(num_directions):
            heading_offset = -self.alpha_max_rad + 2 * self.alpha_max_rad * i / (num_directions - 1)
            next_heading = h + heading_offset
            nx = P[0] + distance * math.cos(next_heading)
            ny = P[1] + distance * math.sin(next_heading)
            next_waypoint = (nx, ny)
            if not self._in_bounds(next_waypoint):
                continue
            if not self._check_collision(P, next_waypoint):
                continue
            is_valid, _ = prep.validate_kinodynamics(
                P, h, next_waypoint, next_heading, R=self.R, alpha_max=self.alpha_max_rad)
            if not is_valid:
                continue
            turn = abs(_angle_diff(next_heading, h))
            cost = distance + config.TURN_PENALTY_WEIGHT * turn
            successors.append((State(next_waypoint, next_heading), cost))

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
        for idx, (center, radius) in enumerate(self.scenario['circle_obstacles']):
            # All riding geometry is BUILT on the lifted radius r_ride so
            # every constructed chord/tangent keeps >= delta true clearance
            # from the exact-checked inflated boundary.
            r_ride = radius + delta
            s = ag.riding_sense(P, h, center, r_ride)
            if s == 0:
                continue
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
        step = math.radians(config.ARC_SAMPLE_STEP_DEG)
        n = int(round(2.0 * math.pi / step))
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
        sector quadrilateral against the STRtree + interior predicate."""
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
        if self._poly_tree is not None:
            quad = Polygon(ag.sector_polygon(center, r_in, r_out, lo, hi))
            for idx in self._poly_tree.query(quad):
                if self._polygons[idx].relate_pattern(quad, 'T********'):
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

        # Check against polygon obstacles via spatial index. A segment is blocked
        # ONLY when it enters a polygon's INTERIOR (DE-9IM interior/interior
        # overlap). Merely touching the boundary is allowed: this lets a waypoint
        # sit on a polygon corner (the corners ARE navigation goals) and lets a
        # segment run ALONG an edge to hug the obstacle boundary. The STRtree gives
        # a bounding-box prefilter; the exact predicate runs only on candidates.
        if self._poly_tree is not None:
            line = LineString([p1, p2])
            for idx in self._poly_tree.query(line):
                if self._polygons[idx].relate_pattern(line, 'T********'):
                    return False
        return True

    def _check_fixed_legs(self, path):
        """Validate the fixed takeoff/approach legs W_{n-1}->T.
        Returns True if the fixed legs are collision-free, False otherwise.
        """
        T = self.scenario['goal_pos']
        if not self._check_collision(self.goal_state.waypoint, T):
            return False
        return True

    def _in_bounds(self, point):
        """Check if point is within map bounds"""
        x, y = point
        # bounds = self.scenario['start_state']['waypoint']  # Just a rough bound
        
        # Allow some overshoot
        margin = 0
        return (-margin < x < config.MAP_WIDTH + margin and
                -margin < y < config.MAP_HEIGHT + margin)
    
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
        self.start_state.h_cost = self.heuristic(self.start_state, self.goal_state)
        heapq.heappush(self.open_set, (
            self.start_state.g_cost + config.HEURISTIC_WEIGHT * self.start_state.h_cost,
            self.iteration_count,
            self.start_state
        ))
        self.g_scores[self.start_state] = 0

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

            # Check if reached goal
            dist_to_goal = math.sqrt(
                (current.waypoint[0] - self.goal_state.waypoint[0])**2 +
                (current.waypoint[1] - self.goal_state.waypoint[1])**2
            )
            
            if dist_to_goal < config.GOAL_THRESHOLD:
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
                        next_state.g_cost + config.HEURISTIC_WEIGHT * next_state.h_cost,
                        self.iteration_count,
                        next_state
                    ))

        # No path found
        self.search_failed = True
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
                is_valid, _ = prep.validate_kinodynamics(
                    anchor_wp, anchor_h, target_wp, heading_to,
                    R=self.R, alpha_max=self.alpha_max_rad)
                if not is_valid:
                    continue
                # Onward turn at the target: for the last body waypoint it is
                # the terminal turn onto the goal approach (the flown leg is
                # path[-1] -> T = goal_pos at goal_heading; use those, not the
                # offset goal_state.waypoint which sits up to GOAL_THRESHOLD away
                # and would spuriously fail the đoản-trình length check).
                if j == n - 1:
                    onward_wp = self.scenario['goal_pos']
                    onward_h = self.scenario['goal_heading']
                else:
                    onward_wp = path[j + 1][0]
                    onward_h = su.angle_to_heading(target_wp, onward_wp)
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

    # Run A* search (dynamic successors)
    planner = KinodynamicAstar(preprocessed_scenario)

    legs_ok = planner._check_fixed_legs(path)
    path = None
    if legs_ok:
        if verbose:
            print("Starting A* search...")
        
        path = planner.search()
        
        if verbose:
            stats = planner.get_search_stats()
            print(f"Search completed: {stats['iterations']}/{stats['max_iterations']} iterations")
            if path:
                print(f"Path found with {len(path)} waypoints")
                print(path)
            else:
                print("No path found")
    
    # Smooth path if found
    if path:
        path = planner.smooth_path(path)

    # Final self-validation: a plan is only a success if the returned path is
    # actually flyable. Search checks segments as it goes, but arc expansion,
    # smoothing, and the fixed O->W1 / W_{n-1}->T legs (added outside the
    # search) can carry collisions that were never verified in final form. 

    return {
        'path': path,
        'success': path is not None and legs_ok,
        'stats': planner.get_search_stats(),
        'planner': planner,
    }
