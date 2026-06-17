"""
Spatial Utilities Module
Provides geometric operations: distance, angle, tangent lines, polygon inflation, etc.
"""

import math
import numpy as np
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
from scipy.spatial import ConvexHull
import config


class Vector2D:
    """Simple 2D vector class"""
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
    
    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scalar):
        return Vector2D(self.x * scalar, self.y * scalar)
    
    def __truediv__(self, scalar):
        return Vector2D(self.x / scalar, self.y / scalar)
    
    def dot(self, other):
        return self.x * other.x + self.y * other.y
    
    def cross(self, other):
        """2D cross product (returns scalar)"""
        return self.x * other.y - self.y * other.x
    
    def length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)
    
    def normalize(self):
        """Return normalized vector"""
        length = self.length()
        if length < 1e-10:
            return Vector2D(0, 0)
        return self / length
    
    def angle(self):
        """Return angle in radians from positive x-axis"""
        return math.atan2(self.y, self.x)
    
    def perpendicular(self):
        """Return perpendicular vector (90 degrees CCW)"""
        return Vector2D(-self.y, self.x)
    
    def rotate(self, angle_rad):
        """Rotate vector by angle (radians) counter-clockwise"""
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        new_x = self.x * cos_a - self.y * sin_a
        new_y = self.x * sin_a + self.y * cos_a
        return Vector2D(new_x, new_y)
    
    def to_tuple(self):
        return (self.x, self.y)
    
    def __repr__(self):
        return f"Vector2D({self.x:.2f}, {self.y:.2f})"


def distance(p1, p2):
    """Calculate Euclidean distance between two points"""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def angle_between_vectors(v1, v2):
    """Calculate angle between two vectors (in radians)"""
    cos_angle = v1.dot(v2) / (v1.length() * v2.length() + 1e-10)
    cos_angle = max(-1, min(1, cos_angle))  # Clamp to [-1, 1]
    return math.acos(cos_angle)


def angle_to_heading(p1, p2):
    """Calculate heading angle from p1 to p2 (radians from positive x-axis)"""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.atan2(dy, dx)


def point_at_distance_and_heading(start_point, distance, heading):
    """Calculate point at given distance and heading from start point"""
    x = start_point[0] + distance * math.cos(heading)
    y = start_point[1] + distance * math.sin(heading)
    return (x, y)


def point_to_line_distance(point, line_start, line_end):
    """Calculate perpendicular distance from point to line segment"""
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end
    
    # Vector from line_start to line_end
    dx = x2 - x1
    dy = y2 - y1
    
    if dx == 0 and dy == 0:
        return distance(point, line_start)
    
    # Parameter t for projection
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    
    # Closest point on line segment
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    
    return distance(point, (closest_x, closest_y))


def line_circle_intersection(p1, p2, center, radius):
    """
    Find intersection points of a line segment (p1, p2) with a circle.
    Returns list of intersection points (0, 1, or 2 points).
    """
    x1, y1 = p1
    x2, y2 = p2
    cx, cy = center
    
    # Line vector
    dx = x2 - x1
    dy = y2 - y1
    
    # Vector from p1 to center
    fx = x1 - cx
    fy = y1 - cy
    
    # Quadratic equation: t^2(dx^2+dy^2) + 2t(fx*dx+fy*dy) + (fx^2+fy^2-r^2) = 0
    a = dx * dx + dy * dy
    b = 2 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    
    discriminant = b * b - 4 * a * c
    
    if discriminant < 0 or abs(a) < 1e-10:
        return []
    
    discriminant = math.sqrt(discriminant)
    t1 = (-b - discriminant) / (2 * a)
    t2 = (-b + discriminant) / (2 * a)
    
    intersections = []
    for t in [t1, t2]:
        if 0 <= t <= 1:
            ix = x1 + t * dx
            iy = y1 + t * dy
            intersections.append((ix, iy))
    
    return intersections


def line_polygon_intersection(p1, p2, polygon_coords):
    """
    Check if line segment (p1, p2) intersects with polygon.
    polygon_coords: list of (x, y) tuples representing polygon vertices
    """
    polygon = Polygon(polygon_coords)
    line = LineString([p1, p2])
    
    return line.intersects(polygon)


def line_of_sight(p1, p2, obstacles):
    """
    Check if there's a clear line of sight between p1 and p2.
    obstacles: list of (center, radius) for circles or polygon vertices for polygons
    Returns True if line is clear, False if blocked.
    """
    line = LineString([p1, p2])
    
    for obstacle in obstacles:
        if isinstance(obstacle, dict):
            # Circle obstacle
            if 'center' in obstacle and 'radius' in obstacle:
                center = obstacle['center']
                radius = obstacle['radius']
                # Check if line is close enough to circle
                dist = point_to_line_distance(center, p1, p2)
                if dist < radius:
                    return False
            # Polygon obstacle
            elif 'polygon' in obstacle:
                polygon = Polygon(obstacle['polygon'])
                if line.intersects(polygon):
                    return False
    
    return True


def inflate_circle(center, radius, inflation):
    """Inflate a circle by adding to radius"""
    return (center, radius + inflation)


def inflate_polygon(polygon_coords, inflation):
    """
    Inflate a polygon by expanding its boundary.
    Uses Shapely's buffer operation.
    """
    polygon = Polygon(polygon_coords)
    expanded = polygon.buffer(inflation)
    
    if expanded.geom_type == 'Polygon':
        return list(expanded.exterior.coords[:-1])  # Exclude closing point
    elif expanded.geom_type == 'MultiPolygon':
        # Return largest polygon
        largest = max(expanded.geoms, key=lambda p: p.area)
        return list(largest.exterior.coords[:-1])
    else:
        return polygon_coords


def compute_tangent_lines(circle1, circle2):
    """
    Compute external bitangent lines between two circles.
    circle1, circle2: (center, radius) tuples
    Returns list of tangent lines as ((x1, y1), (x2, y2))
    """
    c1, r1 = circle1
    c2, r2 = circle2
    
    cx1, cy1 = c1
    cx2, cy2 = c2
    
    # Distance between centers
    d = distance(c1, c2)
    
    if d < abs(r1 - r2) + 1e-10:
        # One circle inside another
        return []
    
    if d < 1e-10:
        # Circles at same location
        return []
    
    tangents = []
    
    # Compute angle to line connecting centers
    angle_centers = math.atan2(cy2 - cy1, cx2 - cx1)
    
    # External tangents (both circles on same side)
    for sign in [1, -1]:
        if abs(r1 - r2) < 1e-10:
            # Circles have same radius
            angle_offset = math.pi / 2
        else:
            # Angle offset for tangent line
            # Clamp the argument to asin to [-1, 1]
            arg = (r2 - r1 * sign) / (d + 1e-10)
            arg = max(-1, min(1, arg))  # Clamp to valid range
            angle_offset = math.asin(arg)
        
        angle = angle_centers + sign * angle_offset
        
        # Tangent points on circles
        t1_x = cx1 + r1 * math.sin(angle)
        t1_y = cy1 - r1 * math.cos(angle)
        
        t2_x = cx2 + r2 * math.sin(angle)
        t2_y = cy2 - r2 * math.cos(angle)
        
        tangents.append(((t1_x, t1_y), (t2_x, t2_y)))
    
    return tangents


def convex_hull_points(points):
    """
    Compute convex hull of a set of points.
    Returns list of points forming the convex hull in counter-clockwise order.
    """
    if len(points) < 3:
        return points
    
    points_array = np.array(points)
    hull = ConvexHull(points_array)
    hull_points = [tuple(points_array[i]) for i in hull.vertices]
    
    return hull_points


def polygon_from_circles(circles):
    """
    Create a convex hull polygon from multiple circles (by their centers).
    circles: list of (center, radius) tuples
    """
    centers = [c[0] for c in circles]
    hull_points = convex_hull_points(centers)
    return hull_points


def check_angle_constraint(heading_before, heading_after):
    """
    Check if turn angle satisfies constraint |Δheading| ≤ ALPHA_MAX.
    heading_before, heading_after: angles in radians
    Returns: angle_delta (magnitude), is_valid (bool)
    """
    # Normalize angles to [-π, π]
    delta = heading_after - heading_before
    delta = math.atan2(math.sin(delta), math.cos(delta))
    
    angle_delta = abs(delta)
    is_valid = angle_delta <= config.ALPHA_MAX_RAD
    
    return angle_delta, is_valid


def calculate_straight_segment_length(heading1, heading2, R):
    """
    Calculate minimum straight segment length l_{i+1} between two turns.
    For segment d_{i+1} = l_{i+1} + R*(tan(α_i/2) + tan(α_{i+1}/2))
    
    Given turn angles and radius, returns the straight segment length.
    This is simplified: returns R to ensure valid positive length.
    """
    alpha1 = abs(heading2 - heading1)
    alpha1 = min(alpha1, 2 * math.pi - alpha1)  # Keep in [0, π]
    
    # Ensure minimum turn radius constraint is satisfied
    if alpha1 > config.ALPHA_MAX_RAD:
        return None
    
    # Minimum straight length to maintain safety
    min_length = R * (math.tan(alpha1 / 2) + 0.1)
    return max(min_length, 100.0)


def state_to_tuple(waypoint, heading):
    """Quantise (waypoint, heading) onto the search lattice for hashing/dedup."""
    q = config.STATE_POS_QUANTUM
    hq = math.radians(config.STATE_HEADING_QUANTUM_DEG)
    hx = int(waypoint[0] // q)
    hy = int(waypoint[1] // q)
    hh = round(math.atan2(math.sin(heading), math.cos(heading)) / hq)
    return (hx, hy, hh)
