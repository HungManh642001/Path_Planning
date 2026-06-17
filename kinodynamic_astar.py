"""
Kinodynamic A* Path Planning Module
Core algorithm for missile trajectory planning with dynamic constraints
"""

import heapq
import math
from collections import defaultdict
import numpy as np
from shapely.geometry import Polygon, LineString

import config
import spatial_utils as su
import preprocessing as prep
import graph_builder as gb


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
    
    def __init__(self, preprocessed_scenario, tangent_graph=None):
        """
        Initialize the planner.
        
        Args:
            preprocessed_scenario: Output from preprocessing.prepare_scenario()
            tangent_graph: TangentGraph object (optional, will be generated if not provided)
        """
        
        self.scenario = preprocessed_scenario
        self.tangent_graph = tangent_graph
        self._polygons = [Polygon(coords) for coords in preprocessed_scenario['polygon_obstacles']]
        
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
        Compute heuristic distance from state to goal.
        Simple Euclidean distance with heading consideration.
        """
        dx = goal_state.waypoint[0] - state.waypoint[0]
        dy = goal_state.waypoint[1] - state.waypoint[1]
        
        dist = math.sqrt(dx**2 + dy**2)
        
        # Add heading mismatch penalty
        heading_diff = abs(state.heading - goal_state.heading)
        heading_diff = min(heading_diff, 2*math.pi - heading_diff)
        
        return dist + self.R * heading_diff
    
    def get_next_states(self, current_state):
        """
        Generate successor states from current state.
        Uses tangent graph nodes as primary navigation with fallback to radial sampling.
        
        Returns:
            List of (next_state, transition_cost) tuples
        """
        
        successors = []
        
        # Strategy 1: expand graph-adjacent nodes (+ direct goal attempt)
        if self.tangent_graph is not None:
            neighbors = self.tangent_graph.get_neighbors(current_state.waypoint)
            goal_wp = self.goal_state.waypoint
            candidates = [wp for wp, _cost in neighbors]
            if goal_wp not in candidates:
                candidates.append(goal_wp)
            for node in candidates:
                dx = node[0] - current_state.waypoint[0]
                dy = node[1] - current_state.waypoint[1]
                if dx * dx + dy * dy < 10000:
                    continue
                heading_to_node = su.angle_to_heading(current_state.waypoint, node)
                is_valid, _ = prep.validate_kinodynamics(
                    current_state.waypoint, current_state.heading,
                    node, heading_to_node,
                    R=self.R, alpha_max=self.alpha_max_rad
                )
                if is_valid and self._check_collision(current_state.waypoint, node):
                    successors.append((State(node, heading_to_node), math.sqrt(dx * dx + dy * dy)))
        
        # Strategy 2: Radial sampling (12 directions) for exploration
        num_directions = 11
        for i in range(num_directions):
            heading_offset = -self.alpha_max_rad +  2 * self.alpha_max_rad * i / (num_directions - 1)
            next_heading = current_state.heading + heading_offset
            
            # Variable distance based on search density
            distance = 2 * self.R * math.tan(self.alpha_max_rad / 2)  
            
            next_x = current_state.waypoint[0] + distance * math.cos(next_heading)
            next_y = current_state.waypoint[1] + distance * math.sin(next_heading)
            next_waypoint = (next_x, next_y)
            
            # Check bounds
            if not self._in_bounds(next_waypoint):
                continue
            
            # Check collision-free path
            if not self._check_collision(current_state.waypoint, next_waypoint):
                continue
            is_valid, _ = prep.validate_kinodynamics(
                current_state.waypoint, current_state.heading,
                next_waypoint, next_heading,
                R=self.R, alpha_max=self.alpha_max_rad)
            if not is_valid:
                continue
            if not self._arc_clear(current_state.waypoint, current_state.heading, next_heading):
                continue
            successors.append((State(next_waypoint, next_heading), distance))
    
        return successors  # Return all successors (no artificial limit)

    
    def _check_collision(self, p1, p2):
        """
        Check if line segment from p1 to p2 collides with any obstacle.
        Returns True if collision-free, False otherwise.
        """
        
        # Check against circle obstacles
        for center, radius in self.scenario['circle_obstacles']:
            dist = su.point_to_line_distance(center, p1, p2)
            if dist < radius - 1e-6:  # Small tolerance
                return False
        
        # Check against polygon obstacles
        line = LineString([p1, p2])
        for polygon in self._polygons:
            if line.intersects(polygon):
                return False
        
        return True
    
    def _arc_clear(self, w, h_in, h_out, n=12):
        import path_validation as pv
        alpha = abs(pv._norm(h_out - h_in))
        if alpha < 1e-9:
            return True
        t = self.R * math.tan(alpha / 2)
        u = (math.cos(h_in), math.sin(h_in))
        v = (math.cos(h_out), math.sin(h_out))
        A = (w[0] - u[0] * t, w[1] - u[1] * t)
        Bp = (w[0] + v[0] * t, w[1] + v[1] * t)
        pts = pv._arc_points(A, w, Bp, self.R, n)
        for j in range(len(pts) - 1):
            if not self._check_collision(pts[j], pts[j + 1]):
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
        
        # Initialize
        self.start_state.h_cost = self.heuristic(self.start_state, self.goal_state)
        heapq.heappush(self.open_set, (
            self.start_state.g_cost + config.HEURISTIC_WEIGHT * self.start_state.h_cost,
            self.iteration_count,
            self.start_state
        ))
        self.g_scores[self.start_state] = 0
        
        while self.open_set and self.iteration_count < self.max_iterations:
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
                # Found path
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
            if self._check_collision(prev_wp, next_wp):
                # Check kinodynamic constraints from last-kept heading to shortcut
                heading_to_next = su.angle_to_heading(prev_wp, next_wp)
                is_valid, _ = prep.validate_kinodynamics(
                    prev_wp, prev_h,
                    next_wp, heading_to_next,
                    alpha_max=self.alpha_max_rad
                )
                # Also check arc clearance at the shortcut join point
                if is_valid and self._arc_clear(prev_wp, prev_h, heading_to_next):
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
    
    # Generate tangent graph
    tangent_graph = gb.generate_bitangents(
        preprocessed_scenario['circle_obstacles'],
        preprocessed_scenario['polygon_obstacles'],
        filter_los=True
    )
    
    # Extend with start/goal connections
    tangent_graph = gb.extend_tangent_graph_with_start_goal(
        tangent_graph,
        preprocessed_scenario['start_state']['waypoint'],
        preprocessed_scenario['start_state']['heading'],
        preprocessed_scenario['goal_state']['waypoint'],
        preprocessed_scenario['goal_state']['heading'],
        preprocessed_scenario['circle_obstacles'],
        preprocessed_scenario['polygon_obstacles'],
    )
    
    if verbose:
        print(f"Tangent graph: {len(tangent_graph.nodes)} nodes, {len(tangent_graph.edges)} edges")
        print(f"Tangent graph: {tangent_graph.nodes}")
    
    # Run A* search
    planner = KinodynamicAstar(preprocessed_scenario, tangent_graph)
    
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
            print("No path found - triggering Lazy Convex Hull fallback")
    
    # Smooth path if found
    if path:
        path = planner.smooth_path(path)
    
    return {
        'path': path,
        'success': path is not None,
        'stats': planner.get_search_stats(),
        'planner': planner,
        'tangent_graph': tangent_graph,
    }
