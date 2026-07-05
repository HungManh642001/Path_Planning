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
        
        # Search variables
        self.open_set = []
        self.closed_set = set()
        self.came_from = {}
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
        successors.extend(self._arc_hop_successors(current_state))
        riding = any(ag.riding_sense(P, h, center, radius) != 0
                     for center, radius in self.scenario['circle_obstacles'])

        # --- Strategy A: dynamic tangent / vertex / goal candidates ---
        goal_wp = self.goal_state.waypoint
        candidates = []
        for center, radius in self.scenario['circle_obstacles']:
            candidates.extend(su.circle_tangent_points(P, center, radius))
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

        if successors and not riding:
            return successors

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
        successors = []
        for center, radius in self.scenario['circle_obstacles']:
            s = ag.riding_sense(P, h, center, radius)
            if s == 0:
                continue
            # A state that is itself an arc-hop departure point of this same
            # circle+sense must not regenerate ride candidates: every departure
            # on this ride was already enumerated from the ride-start state,
            # and regenerating them with shorter residual arcs creates
            # near-duplicate states that collide on the dedup lattice (stale
            # arc_from -> self-crossing reconstruction).
            af = current_state.arc_from
            if af is not None and af[0] == center and af[1] == radius and af[3] == s:
                continue
            phi0 = math.atan2(P[1] - center[1], P[0] - center[0])
            max_wrap = self._max_clear_wrap(center, radius, phi0, s)
            if max_wrap <= 1e-6:
                continue
            deps = []
            for c2, r2 in self.scenario['circle_obstacles']:
                if c2 == center and r2 == radius:
                    continue
                deps.extend(dep for dep, _arr in
                            ag.bitangent_departures(center, radius, c2, r2, s))
            for vertex in self._poly_vertices:
                dep = ag.departure_point(vertex, center, radius, s)
                if dep is not None:
                    deps.append(dep)
            dep = ag.departure_point(goal_wp, center, radius, s)
            if dep is not None:
                deps.append(dep)
            for dep in deps:
                dphi = ag.arc_angle(P, dep, center, s)
                if dphi < 1e-3 or dphi > max_wrap:
                    continue
                nxt = State(dep, ag.tangent_heading(dep, center, s))
                nxt.arc_from = (center, radius, P, s)
                successors.append((nxt, radius * dphi))
        return successors

    def _max_clear_wrap(self, center, radius, phi0, s):
        """Maximal angle (rad) the aircraft can ride this boundary from phi0 in
        direction s before the bulged arc (circumscribed-vertex radius) hits
        another obstacle or leaves the map. One sweep bounds every arc-hop
        candidate on this circle, instead of checking each arc separately.
        Conservative: quantised down to ARC_SAMPLE_STEP_DEG."""
        r_check = radius * _ARC_CLEAR_BULGE
        step = math.radians(config.ARC_SAMPLE_STEP_DEG)
        n = int(round(2.0 * math.pi / step))
        prev = (center[0] + r_check * math.cos(phi0),
                center[1] + r_check * math.sin(phi0))
        for k in range(1, n + 1):
            a = phi0 + s * k * step
            p = (center[0] + r_check * math.cos(a),
                 center[1] + r_check * math.sin(a))
            if (not self._in_bounds(p)
                    or not self._check_collision(prev, p, skip_circle=(center, radius))):
                return (k - 1) * step
            prev = p
        return 2.0 * math.pi

    def _check_collision(self, p1, p2, skip_circle=None):
        """
        Check if line segment from p1 to p2 collides with any obstacle.
        Returns True if collision-free, False otherwise.
        skip_circle=(center, radius) exempts the circle being ridden by an
        arc-clearance sweep (its own boundary is not an obstacle to itself).
        """

        # Check against circle obstacles. A small grazing tolerance lets tangent /
        # wrap segments ride the inflated boundary (they dip a few metres inside the
        # ~13 km inflation band by discretisation but never approach the raw obstacle).
        for center, radius in self.scenario['circle_obstacles']:
            if skip_circle is not None and center == skip_circle[0] and radius == skip_circle[1]:
                continue
            dist = su.point_to_line_distance(center, p1, p2)
            if dist < radius - config.CIRCLE_GRAZE_TOL_M:
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
                    # Better path found
                    self.came_from[next_state] = current
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
        the searched route itself is stored in self.raw_route)."""
        states = [state]
        current = state
        while current in self.came_from:
            current = self.came_from[current]
            states.append(current)
        states.reverse()

        self.raw_route = [(st.waypoint, st.heading) for st in states]

        theta_out = math.radians(config.ARC_WAYPOINT_STEP_DEG)
        path = []
        prev_wp = None
        for st in states:
            if st.arc_from is not None and prev_wp is not None:
                center, radius, arc_start, s = st.arc_from
                # Quantized dedup can rewire came_from so the frozen arc_start
                # belongs to a different ancestor than the chain's actual
                # predecessor; the geometric truth is the previous waypoint,
                # which lies on the circle for any genuine arc transition.
                d_prev = math.hypot(prev_wp[0] - center[0], prev_wp[1] - center[1])
                if abs(d_prev - radius) <= 2.0:
                    arc_start = prev_wp
                dphi = ag.arc_angle(arc_start, st.waypoint, center, s)
                path.extend(ag.arc_waypoints(center, radius, arc_start, dphi, s, theta_out))
            path.append((st.waypoint, st.heading))
            prev_wp = st.waypoint
        return path
    
    def smooth_path(self, path):
        """
        Smooth the path by removing unnecessary waypoints.
        
        Args:
            path: List of (waypoint, heading) tuples
        
        Returns:
            Smoothed path
        """
        if len(path) < 3:
            return path
        
        smoothed = [path[0]]

        i = 1
        while i < len(path) - 1:
            # Always shortcut FROM the last kept point (smoothed[-1]), not path[i-1].
            # Using path[i-1] is a bug: after a skip, path[i-1] is a discarded node.
            prev_wp, prev_h = smoothed[-1]
            # Geometric inbound heading at prev_wp (the arc there is governed by the
            # bearing from the previous KEPT waypoint, not the stored A* heading).
            if len(smoothed) >= 2:
                prev_h = su.angle_to_heading(smoothed[-2][0], prev_wp)
            next_wp, next_h = path[i + 1]

            # Try to shortcut from last-kept to next: skip path[i]
            heading_to_next = su.angle_to_heading(prev_wp, next_wp)
            is_valid, _ = prep.validate_kinodynamics(
                prev_wp, prev_h,
                next_wp, heading_to_next,
                R=self.R, alpha_max=self.alpha_max_rad
            )
            # Skipping path[i] changes the ARRIVAL direction at the next waypoint,
            # so its onward turn must be re-checked (the old code only validated the
            # turn at prev_wp). If next_wp is the last waypoint, its onward turn is
            # the terminal turn onto goal_heading. Without this check, smoothing can
            # bend the approach past alpha_max even when the search path was valid.
            if i + 1 == len(path) - 1:
                onward_wp, onward_heading = self.goal_state.waypoint, self.goal_state.heading
            else:
                onward_wp, onward_heading = path[i + 2]

            is_next_valid, _ = prep.validate_kinodynamics(
                next_wp, heading_to_next,
                onward_wp, onward_heading,
                R=self.R, alpha_max=self.alpha_max_rad
            )
            if (is_valid and is_next_valid
                    and self._check_collision(prev_wp, next_wp)):
                # Can skip current point
                i += 1
                continue

            smoothed.append(path[i])
            i += 1

        smoothed.append(path[-1])
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
            - 'success': Bool indicating if planning succeeded
            - 'stats': Search statistics
            - 'planner': KinodynamicAstar object
    """
    
    if verbose:
        print("Initializing Kinodynamic A*...")

    # Run A* search (dynamic successors)
    planner = KinodynamicAstar(preprocessed_scenario)
    
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
    
    return {
        'path': path,
        'success': path is not None,
        'stats': planner.get_search_stats(),
        'planner': planner,
    }
