"""
Kinodynamic A* Path Planning Module
Core algorithm for missile trajectory planning with dynamic constraints
"""

import heapq
import math
from collections import defaultdict
import numpy as np
from shapely.geometry import Polygon, LineString
from shapely import STRtree

import config
import spatial_utils as su
import preprocessing as prep


def _angle_diff(a, b):
    """Smallest signed difference a-b normalised to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class State:
    """Represents a missile state: (waypoint, heading)"""
    
    def __init__(self, waypoint, heading):
        self.waypoint = waypoint  # (x, y)
        self.heading = heading  # radians
        self.parent = None
        self.g_cost = float('inf')  # Cost from start
        self.h_cost = 0  # Heuristic to goal
    
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
    """Kinodynamic A* path planner for missile trajectory"""
    
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

        # --- Wrap step: straight continuation off a circle boundary ---
        # A point ON a circle has no tangent to that circle, so the planner cannot
        # tangent further around it. Flying STRAIGHT (same heading) for WRAP_STEP_M
        # steps just off the circle so the next expansion can tangent further along
        # it. This step is zero-turn, so it carries no đoản trình arc reservation.
        if self._on_circle_boundary(P):
            nx = (P[0] + config.WRAP_STEP_M * math.cos(h),
                  P[1] + config.WRAP_STEP_M * math.sin(h))
            if self._in_bounds(nx) and self._check_collision(P, nx):
                successors.append((State(nx, h), config.WRAP_STEP_M))

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
            # At the final waypoint W_{n-1} the missile must turn from the approach
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

        if successors:
            return successors

        # --- Strategy B: radial fan fallback (no graph candidate was valid) ---
        num_directions = 11
        distance = 2 * self.R * math.tan(self.alpha_max_rad / 2)
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

    
    def _check_collision(self, p1, p2):
        """
        Check if line segment from p1 to p2 collides with any obstacle.
        Returns True if collision-free, False otherwise.
        """
        
        # Check against circle obstacles. A small grazing tolerance lets tangent /
        # wrap segments ride the inflated boundary (they dip a few metres inside the
        # ~13 km inflation band by discretisation but never approach the raw obstacle).
        for center, radius in self.scenario['circle_obstacles']:
            dist = su.point_to_line_distance(center, p1, p2)
            if dist < radius - config.CIRCLE_GRAZE_TOL_M:
                return False
        
        # Check against polygon obstacles via spatial index. A segment is blocked
        # ONLY when it enters a polygon's INTERIOR (DE-9IM interior/interior
        # overlap). Merely touching the boundary is allowed: this lets a waypoint
        # sit on a polygon corner (the corners ARE navigation targets) and lets a
        # segment run ALONG an edge to hug the obstacle boundary. The STRtree gives
        # a bounding-box prefilter; the exact predicate runs only on candidates.
        if self._poly_tree is not None:
            line = LineString([p1, p2])
            for idx in self._poly_tree.query(line):
                if self._polygons[idx].relate_pattern(line, 'T********'):
                    return False
        return True
    
    def _on_circle_boundary(self, point, tol=1.0):
        """True if `point` lies on (within tol of) any inflated circle boundary."""
        for center, radius in self.scenario['circle_obstacles']:
            if abs(math.hypot(point[0] - center[0], point[1] - center[1]) - radius) < tol:
                return True
        return False

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
                # Reaching the goal region is not enough: the missile must arrive
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
        """Reconstruct path from start to state"""
        path = []
        current = state
        
        while current in self.came_from:
            path.append((current.waypoint, current.heading))
            current = self.came_from[current]
        
        path.append((self.start_state.waypoint, self.start_state.heading))
        path.reverse()
        
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
                onward_heading = self.goal_state.heading
            else:
                onward_heading = su.angle_to_heading(next_wp, path[i + 2][0])
            downstream_turn = abs(_angle_diff(onward_heading, heading_to_next))
            if (is_valid and downstream_turn <= self.alpha_max_rad
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
    High-level function to plan a missile trajectory.
    
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
