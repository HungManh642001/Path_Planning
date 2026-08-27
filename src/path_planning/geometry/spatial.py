"""Geometric helpers used by the planner.

Distance, headings, point-to-segment distance, polygon inflation, state
quantisation and circle tangent points. Distances are metres, angles radians.

Note on typing: shapely 2.1.2 ships no ``py.typed``, so a strict checker infers
its signatures from source and gets them narrower than the runtime contract --
``Polygon.buffer`` is inferred to return ``Polygon`` when it can genuinely
return a ``MultiPolygon``. The runtime branch below is therefore correct and the
checker's "unnecessary" verdict is not; the suppression is scoped to this module
and to the shapely-inference rules alone.
"""
# pyright: reportUnnecessaryIsInstance=false, reportUnknownVariableType=false
# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false
# pyright: reportUnknownLambdaType=false

from __future__ import annotations

import math

from shapely.geometry import MultiPolygon, Polygon

from path_planning import config
from path_planning.types import LatticeKey, Point, PolygonCoords


def distance(p1: Point, p2: Point) -> float:
    """Compute the Euclidean distance between two points.

    Args:
        p1: First point.
        p2: Second point.

    Returns:
        The distance in metres.
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def angle_to_heading(p1: Point, p2: Point) -> float:
    """Compute the heading from ``p1`` to ``p2``.

    Args:
        p1: Origin point.
        p2: Target point.

    Returns:
        The heading in radians, measured from the positive x-axis.
    """
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


def angle_diff(a: float, b: float) -> float:
    """Compute the smallest signed difference ``a - b``.

    Both planners alias this at module level (``_angle_diff = su.angle_diff``)
    because it is read on the hot path. Two other copies stay where they are on
    purpose, and neither is an oversight: ``path_validation._norm`` keeps the
    oracle independent of the code it validates, and ``goal_shot._angdiff``
    keeps that module free of every import but ``math`` -- importing this one
    would drag shapely and config into a file whose whole contract is pure
    geometry.

    Args:
        a: Minuend angle in radians.
        b: Subtrahend angle in radians.

    Returns:
        The difference normalised to ``[-pi, pi]``.
    """
    return math.atan2(math.sin(a - b), math.cos(a - b))


def point_to_line_distance(point: Point, line_start: Point, line_end: Point) -> float:
    """Compute the perpendicular distance from a point to a line segment.

    Args:
        point: The query point.
        line_start: First endpoint of the segment.
        line_end: Second endpoint of the segment.

    Returns:
        The distance in metres; the distance to the shared endpoint if the
        segment is degenerate.
    """
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return distance(point, line_start)
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    return distance(point, (x1 + t * dx, y1 + t * dy))


def _exterior_coords(polygon: Polygon) -> PolygonCoords:
    """Extract a polygon's exterior ring without its repeated closing point."""
    return [(float(x), float(y)) for x, y in polygon.exterior.coords[:-1]]


def inflate_polygon(polygon_coords: PolygonCoords, inflation: float) -> PolygonCoords:
    """Inflate a polygon outward by ``inflation`` metres.

    Mitre join keeps sharp corners (few real vertices for navigation) and the
    result contains the round Minkowski buffer, so arc clearance is preserved.

    Args:
        polygon_coords: The polygon ring to inflate.
        inflation: Outward offset in metres; non-positive returns a copy.

    Returns:
        The inflated ring, or the input unchanged if the buffer degenerates.
    """
    # buffer(0) is a CLEANING operation in shapely, not a no-op: a self-touching
    # ring splits into a MultiPolygon and the branch below would silently keep
    # only the largest piece, shrinking the obstacle. Reachable now that
    # inflation is just SAFE_MARGIN, which may legitimately be 0.
    if inflation <= 0.0:
        return list(polygon_coords)
    expanded = Polygon(polygon_coords).buffer(
        inflation, join_style="mitre", mitre_limit=config.POLYGON_MITRE_LIMIT
    )
    if isinstance(expanded, Polygon):
        return _exterior_coords(expanded)
    if isinstance(expanded, MultiPolygon):
        largest = max(expanded.geoms, key=lambda p: p.area)
        return _exterior_coords(largest)
    return polygon_coords


def state_to_tuple(waypoint: Point, heading: float) -> LatticeKey:
    """Quantise a state onto the search lattice for hashing and dedup.

    Args:
        waypoint: The state position.
        heading: The state heading in radians.

    Returns:
        The lattice cell as ``(x_index, y_index, heading_index)``.
    """
    q = config.STATE_POS_QUANTUM
    hq = math.radians(config.STATE_HEADING_QUANTUM_DEG)
    hx = int(waypoint[0] // q)
    hy = int(waypoint[1] // q)
    hh = round(math.atan2(math.sin(heading), math.cos(heading)) / hq)
    return (hx, hy, hh)


def circle_tangent_points(point: Point, center: Point, radius: float) -> list[Point]:
    """Find the tangent points on a circle from an external point.

    Args:
        point: The external point the tangent lines emanate from.
        center: Circle centre.
        radius: Circle radius in metres.

    Returns:
        The two tangency points, or an empty list if ``point`` lies inside or
        on the circle, where no real tangent exists.
    """
    px, py = point
    cx, cy = center
    dx, dy = px - cx, py - cy
    d2 = dx * dx + dy * dy
    if d2 <= radius * radius + 1e-9:
        return []
    d = math.sqrt(d2)
    theta = math.atan2(dy, dx)  # center -> point direction
    alpha = math.acos(radius / d)  # half-angle of the tangent cone
    return [
        (cx + radius * math.cos(theta + alpha), cy + radius * math.sin(theta + alpha)),
        (cx + radius * math.cos(theta - alpha), cy + radius * math.sin(theta - alpha)),
    ]
