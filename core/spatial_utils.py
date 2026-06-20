"""
Spatial Utilities Module
Geometric helpers used by the planner: distance, headings, point-to-segment
distance, polygon inflation, state quantisation, and circle tangent points.
"""

import math
from shapely.geometry import Polygon

import config


def distance(p1, p2):
    """Euclidean distance between two points."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def angle_to_heading(p1, p2):
    """Heading angle from p1 to p2 (radians from the positive x-axis)."""
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


def point_to_line_distance(point, line_start, line_end):
    """Perpendicular distance from a point to a line segment."""
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return distance(point, line_start)
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    return distance(point, (x1 + t * dx, y1 + t * dy))


def inflate_polygon(polygon_coords, inflation):
    """Inflate a polygon outward by `inflation` (Shapely buffer).

    Mitre join keeps sharp corners (few real vertices for navigation) and the
    result contains the round Minkowski buffer, so arc-clearance is preserved.
    """
    expanded = Polygon(polygon_coords).buffer(
        inflation, join_style='mitre', mitre_limit=config.POLYGON_MITRE_LIMIT)
    if expanded.geom_type == 'Polygon':
        return list(expanded.exterior.coords[:-1])          # drop the closing point
    if expanded.geom_type == 'MultiPolygon':
        largest = max(expanded.geoms, key=lambda p: p.area)
        return list(largest.exterior.coords[:-1])
    return polygon_coords


def state_to_tuple(waypoint, heading):
    """Quantise (waypoint, heading) onto the search lattice for hashing/dedup."""
    q = config.STATE_POS_QUANTUM
    hq = math.radians(config.STATE_HEADING_QUANTUM_DEG)
    hx = int(waypoint[0] // q)
    hy = int(waypoint[1] // q)
    hh = round(math.atan2(math.sin(heading), math.cos(heading)) / hq)
    return (hx, hy, hh)


def circle_tangent_points(point, center, radius):
    """Tangent points on a circle from an external point.

    Returns the two points where lines from `point` touch the circle, or []
    if `point` is inside or on the circle (no real tangent).
    """
    px, py = point
    cx, cy = center
    dx, dy = px - cx, py - cy
    d2 = dx * dx + dy * dy
    if d2 <= radius * radius + 1e-9:
        return []
    d = math.sqrt(d2)
    theta = math.atan2(dy, dx)          # center -> point direction
    alpha = math.acos(radius / d)       # half-angle of the tangent cone
    return [
        (cx + radius * math.cos(theta + alpha), cy + radius * math.sin(theta + alpha)),
        (cx + radius * math.cos(theta - alpha), cy + radius * math.sin(theta - alpha)),
    ]
