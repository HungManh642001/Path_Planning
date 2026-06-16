"""Validators for produced planner paths (spec: Điều kiện ràng buộc đường bay).

A path is a list of (waypoint, heading) tuples with waypoint = (x, y).
These functions are deliberately independent of the planner internals so
tests can assert validity without trusting the code under review.
"""
import math
from shapely.geometry import Polygon, LineString


# NOTE: intentionally re-implements spatial_utils.point_to_line_distance rather than
# importing it, so this validator stays independent of the planner code it validates.
def _point_to_segment_distance(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def _segment_clear(a, b, circle_obstacles, polygon_obstacles, tol=1e-6):
    for center, radius in circle_obstacles:
        # Leniency: a path grazing the inflated boundary to within `tol` meters
        # is accepted; tol is subtracted (not added) so only genuine penetration fails.
        if _point_to_segment_distance(center, a, b) < radius - tol:
            return False
    line = LineString([a, b])
    for coords in polygon_obstacles:
        if line.intersects(Polygon(coords)):
            return False
    return True


def segments_clear(path, circle_obstacles, polygon_obstacles):
    """True iff every straight segment between consecutive waypoints is clear."""
    for i in range(len(path) - 1):
        a = path[i][0]
        b = path[i + 1][0]
        if not _segment_clear(a, b, circle_obstacles, polygon_obstacles):
            return False
    return True
