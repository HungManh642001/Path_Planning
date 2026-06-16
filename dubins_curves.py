"""
Dubins Path Module
Computes smooth trajectories using Dubins curves (circular arcs + straight lines)
Based on Dubins' theorem for shortest paths with bounded curvature
"""

import math
import numpy as np
from scipy.optimize import minimize_scalar
import config


class DubinsPath:
    """Represents a Dubins path from start to goal"""
    
    def __init__(self, start_pos, start_heading, goal_pos, goal_heading, radius):
        """
        Initialize Dubins path calculator.
        
        Args:
            start_pos: (x, y) starting position
            start_heading: Starting heading angle (radians)
            goal_pos: (x, y) goal position
            goal_heading: Goal heading angle (radians)
            radius: Turn radius (curvature = 1/radius)
        """
        self.start_pos = start_pos
        self.start_heading = start_heading
        self.goal_pos = goal_pos
        self.goal_heading = goal_heading
        self.radius = radius
        
        self.path_length = None
        self.path_type = None  # e.g., "LSR", "RSL", "LRL", etc.
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _circle_center(self, pos, heading, turn_direction):
        """
        Calculate center of turning circle.
        
        Args:
            pos: Current position (x, y)
            heading: Current heading (radians)
            turn_direction: 1 for left turn, -1 for right turn
        
        Returns:
            Center point (x, y)
        """
        perp_heading = heading + turn_direction * math.pi / 2
        center_x = pos[0] + self.radius * math.cos(perp_heading)
        center_y = pos[1] + self.radius * math.sin(perp_heading)
        return (center_x, center_y)
    
    def _tangent_points(self, center1, center2, turn1, turn2):
        """
        Calculate tangent points between two circles.
        
        Args:
            center1: Center of first circle (x, y)
            center2: Center of second circle (x, y)
            turn1: Turn direction of first circle (1 or -1)
            turn2: Turn direction of second circle (1 or -1)
        
        Returns:
            List of [(tangent1_on_circle1, tangent1_on_circle2), ...]
        """
        tangents = []
        
        dx = center2[0] - center1[0]
        dy = center2[1] - center1[1]
        d = math.sqrt(dx**2 + dy**2)
        
        if d < 1e-10:
            return tangents
        
        # Angle to line connecting centers
        angle_between = math.atan2(dy, dx)
        
        # For same-direction turns (L-L or R-R)
        if turn1 == turn2:
            if d < 2 * self.radius:
                return tangents
            
            # External tangent
            angle_offset = math.acos(2 * self.radius / d)
            angle = angle_between + turn1 * angle_offset
            
            t1_x = center1[0] + self.radius * math.sin(angle)
            t1_y = center1[1] - self.radius * math.cos(angle)
            
            t2_x = center2[0] + self.radius * math.sin(angle)
            t2_y = center2[1] - self.radius * math.cos(angle)
            
            tangents.append(((t1_x, t1_y), (t2_x, t2_y)))
        
        # For opposite-direction turns (L-R or R-L)
        else:
            if d < 2 * self.radius:
                # Internal tangent
                angle = angle_between + turn1 * math.pi / 2
            else:
                angle_offset = math.acos(2 * self.radius / d)
                angle = angle_between + turn1 * angle_offset
            
            t1_x = center1[0] + self.radius * math.sin(angle)
            t1_y = center1[1] - self.radius * math.cos(angle)
            
            t2_x = center2[0] - self.radius * math.sin(angle)
            t2_y = center2[1] + self.radius * math.cos(angle)
            
            tangents.append(((t1_x, t1_y), (t2_x, t2_y)))
        
        return tangents
    
    def compute_lsr(self):
        """
        Compute LSR (Left-Straight-Right) Dubins path.
        """
        center_start_l = self._circle_center(self.start_pos, self.start_heading, 1)
        center_goal_r = self._circle_center(self.goal_pos, self.goal_heading, -1)
        
        tangents = self._tangent_points(center_start_l, center_goal_r, 1, -1)
        if not tangents:
            return None
        
        tangent = tangents[0]
        
        # Arc lengths
        angle_start = math.atan2(
            tangent[0][1] - center_start_l[1],
            tangent[0][0] - center_start_l[0]
        )
        heading_start = self._normalize_angle(self.start_heading)
        perp_start = heading_start + math.pi / 2
        
        arc_start_angle = self._normalize_angle(angle_start - perp_start)
        arc_start_length = abs(arc_start_angle) * self.radius
        
        angle_goal = math.atan2(
            tangent[1][1] - center_goal_r[1],
            tangent[1][0] - center_goal_r[0]
        )
        heading_goal = self._normalize_angle(self.goal_heading)
        perp_goal = heading_goal - math.pi / 2
        
        arc_goal_angle = self._normalize_angle(angle_goal - perp_goal)
        arc_goal_length = abs(arc_goal_angle) * self.radius
        
        # Straight segment length
        straight_length = math.sqrt(
            (tangent[1][0] - tangent[0][0])**2 +
            (tangent[1][1] - tangent[0][1])**2
        )
        
        total_length = arc_start_length + straight_length + arc_goal_length
        
        return {
            'type': 'LSR',
            'length': total_length,
            'arc_start': arc_start_length,
            'straight': straight_length,
            'arc_end': arc_goal_length,
            'center_start': center_start_l,
            'center_goal': center_goal_r,
            'tangent_start': tangent[0],
            'tangent_goal': tangent[1],
        }
    
    def compute_rsl(self):
        """Compute RSL (Right-Straight-Left) Dubins path"""
        center_start_r = self._circle_center(self.start_pos, self.start_heading, -1)
        center_goal_l = self._circle_center(self.goal_pos, self.goal_heading, 1)
        
        tangents = self._tangent_points(center_start_r, center_goal_l, -1, 1)
        if not tangents:
            return None
        
        tangent = tangents[0]
        
        # Similar calculations as LSR
        angle_start = math.atan2(
            tangent[0][1] - center_start_r[1],
            tangent[0][0] - center_start_r[0]
        )
        heading_start = self._normalize_angle(self.start_heading)
        perp_start = heading_start - math.pi / 2
        
        arc_start_angle = self._normalize_angle(angle_start - perp_start)
        arc_start_length = abs(arc_start_angle) * self.radius
        
        angle_goal = math.atan2(
            tangent[1][1] - center_goal_l[1],
            tangent[1][0] - center_goal_l[0]
        )
        heading_goal = self._normalize_angle(self.goal_heading)
        perp_goal = heading_goal + math.pi / 2
        
        arc_goal_angle = self._normalize_angle(angle_goal - perp_goal)
        arc_goal_length = abs(arc_goal_angle) * self.radius
        
        straight_length = math.sqrt(
            (tangent[1][0] - tangent[0][0])**2 +
            (tangent[1][1] - tangent[0][1])**2
        )
        
        total_length = arc_start_length + straight_length + arc_goal_length
        
        return {
            'type': 'RSL',
            'length': total_length,
            'arc_start': arc_start_length,
            'straight': straight_length,
            'arc_end': arc_goal_length,
            'center_start': center_start_r,
            'center_goal': center_goal_l,
            'tangent_start': tangent[0],
            'tangent_goal': tangent[1],
        }
    
    def compute_lrl(self):
        """Compute LRL (Left-Right-Left) Dubins path"""
        center_start_l = self._circle_center(self.start_pos, self.start_heading, 1)
        center_goal_l = self._circle_center(self.goal_pos, self.goal_heading, 1)
        
        d = math.sqrt(
            (center_goal_l[0] - center_start_l[0])**2 +
            (center_goal_l[1] - center_start_l[1])**2
        )
        
        if d > 4 * self.radius:
            return None
        
        # Middle circle
        dx = center_goal_l[0] - center_start_l[0]
        dy = center_goal_l[1] - center_start_l[1]
        
        angle_between = math.atan2(dy, dx)
        mid_distance = d / 2
        angle_offset = math.acos(d / (4 * self.radius))
        
        angle_mid = angle_between + angle_offset
        center_mid_x = center_start_l[0] + 2 * self.radius * math.cos(angle_mid)
        center_mid_y = center_start_l[1] + 2 * self.radius * math.sin(angle_mid)
        center_mid = (center_mid_x, center_mid_y)
        
        # Arc angles
        angle_start_to_mid = math.atan2(
            center_mid[1] - center_start_l[1],
            center_mid[0] - center_start_l[0]
        )
        
        heading_start = self._normalize_angle(self.start_heading)
        perp_start = heading_start + math.pi / 2
        
        arc_start_angle = self._normalize_angle(angle_start_to_mid - perp_start)
        arc_start_length = abs(arc_start_angle) * self.radius
        
        angle_mid_to_goal = math.atan2(
            center_goal_l[1] - center_mid[1],
            center_goal_l[0] - center_mid[0]
        )
        
        perp_mid = angle_start_to_mid + math.pi
        arc_mid_angle = self._normalize_angle(angle_mid_to_goal - perp_mid)
        arc_mid_length = abs(arc_mid_angle) * self.radius
        
        heading_goal = self._normalize_angle(self.goal_heading)
        perp_goal = heading_goal + math.pi / 2
        
        arc_goal_angle = self._normalize_angle(perp_goal - angle_mid_to_goal)
        arc_goal_length = abs(arc_goal_angle) * self.radius
        
        total_length = arc_start_length + arc_mid_length + arc_goal_length
        
        return {
            'type': 'LRL',
            'length': total_length,
            'arc_start': arc_start_length,
            'arc_mid': arc_mid_length,
            'arc_end': arc_goal_length,
            'center_start': center_start_l,
            'center_mid': center_mid,
            'center_goal': center_goal_l,
        }
    
    def compute_rrl(self):
        """Compute RRL (Right-Right-Left) Dubins path"""
        center_start_r = self._circle_center(self.start_pos, self.start_heading, -1)
        center_goal_r = self._circle_center(self.goal_pos, self.goal_heading, -1)
        
        d = math.sqrt(
            (center_goal_r[0] - center_start_r[0])**2 +
            (center_goal_r[1] - center_start_r[1])**2
        )
        
        if d > 4 * self.radius:
            return None
        
        dx = center_goal_r[0] - center_start_r[0]
        dy = center_goal_r[1] - center_start_r[1]
        
        angle_between = math.atan2(dy, dx)
        angle_offset = math.acos(d / (4 * self.radius))
        
        angle_mid = angle_between - angle_offset
        center_mid_x = center_start_r[0] + 2 * self.radius * math.cos(angle_mid)
        center_mid_y = center_start_r[1] + 2 * self.radius * math.sin(angle_mid)
        center_mid = (center_mid_x, center_mid_y)
        
        # Arc calculations (similar pattern)
        angle_start_to_mid = math.atan2(
            center_mid[1] - center_start_r[1],
            center_mid[0] - center_start_r[0]
        )
        
        heading_start = self._normalize_angle(self.start_heading)
        perp_start = heading_start - math.pi / 2
        
        arc_start_angle = self._normalize_angle(angle_start_to_mid - perp_start)
        arc_start_length = abs(arc_start_angle) * self.radius
        
        angle_mid_to_goal = math.atan2(
            center_goal_r[1] - center_mid[1],
            center_goal_r[0] - center_mid[0]
        )
        
        perp_mid = angle_start_to_mid - math.pi
        arc_mid_angle = self._normalize_angle(angle_mid_to_goal - perp_mid)
        arc_mid_length = abs(arc_mid_angle) * self.radius
        
        heading_goal = self._normalize_angle(self.goal_heading)
        perp_goal = heading_goal - math.pi / 2
        
        arc_goal_angle = self._normalize_angle(perp_goal - angle_mid_to_goal)
        arc_goal_length = abs(arc_goal_angle) * self.radius
        
        total_length = arc_start_length + arc_mid_length + arc_goal_length
        
        return {
            'type': 'RRL',
            'length': total_length,
            'arc_start': arc_start_length,
            'arc_mid': arc_mid_length,
            'arc_end': arc_goal_length,
            'center_start': center_start_r,
            'center_mid': center_mid,
            'center_goal': center_goal_r,
        }
    
    def shortest_path(self):
        """
        Find shortest Dubins path among all 6 possible types.
        
        Returns:
            Dictionary describing shortest path
        """
        candidates = []
        
        for path_func in [self.compute_lsr, self.compute_rsl, 
                         self.compute_lrl, self.compute_rrl]:
            path = path_func()
            if path:
                candidates.append(path)
        
        if not candidates:
            return None
        
        # Return path with minimum length
        shortest = min(candidates, key=lambda p: p['length'])
        self.path_length = shortest['length']
        self.path_type = shortest['type']
        
        return shortest
    
    def sample_path(self, num_samples=100):
        """
        Sample points along the shortest Dubins path.
        
        Args:
            num_samples: Number of points to sample
        
        Returns:
            List of (x, y, heading) tuples
        """
        path = self.shortest_path()
        if not path:
            return []
        
        samples = []
        total_length = path['length']
        
        # Sample uniformly along path length
        for i in range(num_samples):
            dist_along = (i / (num_samples - 1)) * total_length if num_samples > 1 else 0
            
            # Determine which segment we're in
            sample_point = self._sample_at_distance(path, dist_along)
            if sample_point:
                samples.append(sample_point)
        
        return samples
    
    def _sample_at_distance(self, path, distance):
        """
        Get point at specific distance along path.
        
        Returns:
            (x, y, heading) tuple or None
        """
        # Simplified: just interpolate along the path
        # More sophisticated sampling would follow actual Dubins curve geometry
        
        path_type = path.get('type', '')
        
        if 'LSR' in path_type or 'RSL' in path_type:
            # Two arcs + one straight segment
            arc_start = path.get('arc_start', 0)
            straight = path.get('straight', 0)
            
            if distance < arc_start:
                # In first arc
                frac = distance / arc_start
            elif distance < arc_start + straight:
                # In straight segment
                frac = (distance - arc_start) / straight
            else:
                # In final arc
                frac = (distance - arc_start - straight) / path.get('arc_end', 1)
            
            # Linear interpolation for now (simplified)
            frac = min(1.0, max(0.0, frac))
            x = self.start_pos[0] + frac * (self.goal_pos[0] - self.start_pos[0])
            y = self.start_pos[1] + frac * (self.goal_pos[1] - self.start_pos[1])
            
            # Interpolate heading
            heading = self.start_heading + frac * (self.goal_heading - self.start_heading)
            
            return (x, y, heading)
        
        return None


def compute_dubins_path_between_waypoints(waypoints, turn_radius):
    """
    Compute Dubins paths between all consecutive waypoints.
    
    Args:
        waypoints: List of (pos, heading) tuples
        turn_radius: Missile turn radius
    
    Returns:
        List of DubinsPath objects
    """
    dubins_paths = []
    
    for i in range(len(waypoints) - 1):
        start_wp, start_heading = waypoints[i]
        goal_wp, goal_heading = waypoints[i + 1]
        
        dubins = DubinsPath(start_wp, start_heading, goal_wp, goal_heading, turn_radius)
        dubins_paths.append(dubins)
    
    return dubins_paths


def sample_all_dubins_paths(dubins_paths, samples_per_segment=50):
    """
    Sample all Dubins paths to get smooth trajectory points.
    
    Args:
        dubins_paths: List of DubinsPath objects
        samples_per_segment: Number of samples per path segment
    
    Returns:
        List of (x, y, heading) tuples representing smooth trajectory
    """
    all_samples = []
    
    for i, dubins_path in enumerate(dubins_paths):
        samples = dubins_path.sample_path(samples_per_segment)
        
        # Skip first sample of subsequent segments (avoid duplicates)
        if i > 0 and samples:
            samples = samples[1:]
        
        all_samples.extend(samples)
    
    return all_samples
