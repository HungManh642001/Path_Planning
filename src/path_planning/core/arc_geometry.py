"""Pure geometry for arc-hop successors.

Riding detection, tangent headings, bitangents between circles, departure
points, and output-time arc expansion.

Wrap sense convention: ``+1`` rides a circle counter-clockwise, ``-1``
clockwise, computed as ``sign(cross(point - center, heading_vector))``.

No planner or config imports: every tolerance and step is a parameter.
"""

from __future__ import annotations

import math

from path_planning.core import spatial_utils as su
from path_planning.core.types import (
    PlannerState,
    Point,
    PolygonCoords,
    RidingSense,
    WrapSense,
)


def riding_sense(
    point: Point,
    heading: float,
    center: Point,
    radius: float,
    pos_tol: float = 1.0,
    ang_tol: float = 8.72e-3,
) -> RidingSense:
    """Detect whether a state rides a circle's boundary, and in which direction.

    Riding means ``point`` lies on the boundary (within ``pos_tol``) and
    ``heading`` is tangent to the circle there (within ``ang_tol``).

    Args:
        point: The state position.
        heading: The state heading (rad).
        center: Circle centre.
        radius: Circle radius (m).
        pos_tol: Tolerance on the boundary distance (m).
        ang_tol: Tolerance on the radial/heading dot product (~sin 0.5 deg).

    Returns:
        The wrap sense ``+1`` or ``-1``, or ``0`` if the state is not riding.
    """
    dx, dy = point[0] - center[0], point[1] - center[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-9 or abs(dist - radius) > pos_tol:
        return 0
    ux, uy = dx / dist, dy / dist
    hx, hy = math.cos(heading), math.sin(heading)
    if abs(ux * hx + uy * hy) > ang_tol:
        return 0
    return 1 if (ux * hy - uy * hx) > 0 else -1


def tangent_heading(point: Point, center: Point, sense: WrapSense) -> float:
    """Compute the heading of tangent travel at a boundary point.

    Args:
        point: A point on the circle boundary.
        center: Circle centre.
        sense: Wrap sense of travel.

    Returns:
        The tangent travel heading (rad).
    """
    return math.atan2(sense * (point[0] - center[0]), -sense * (point[1] - center[1]))


def arc_angle(start: Point, end: Point, center: Point, sense: WrapSense) -> float:
    """Compute the angle swept from one boundary point to another.

    Args:
        start: Arc start point on the boundary.
        end: Arc end point on the boundary.
        center: Circle centre.
        sense: Wrap sense of travel.

    Returns:
        The swept angle in ``[0, 2*pi)`` radians.
    """
    a0 = math.atan2(start[1] - center[1], start[0] - center[0])
    a1 = math.atan2(end[1] - center[1], end[0] - center[0])
    return (sense * (a1 - a0)) % (2.0 * math.pi)


def departure_point(
    target: Point, center: Point, radius: float, sense: WrapSense
) -> Point | None:
    """Find the boundary point from which leaving toward a target is tangent-continuous.

    Args:
        target: The external point being departed toward.
        center: Circle centre.
        radius: Circle radius (m).
        sense: Wrap sense of travel along the boundary.

    Returns:
        The tangency point, or ``None`` if ``target`` lies inside or on the
        circle, where no tangent departure exists.
    """
    for dep in su.circle_tangent_points(target, center, radius):
        nx = (dep[0] - center[0]) / radius
        ny = (dep[1] - center[1]) / radius
        # Velocity at dep for this sense is sense * perp_ccw(n) = (-s*ny, s*nx).
        if (-sense * ny) * (target[0] - dep[0]) + (sense * nx) * (
            target[1] - dep[1]
        ) > 0:
            return dep
    return None


def bitangent_departures(
    c1: Point, r1: float, c2: Point, r2: float, sense: WrapSense
) -> list[tuple[Point, Point]]:
    """Enumerate bitangent lines between two circles, filtered by wrap sense.

    Construction: a bitangent touches circle 1 at ``c1 + r1*n`` and circle 2 at
    ``c2 + sigma*r2*n`` for a unit normal ``n`` with ``n.D_hat = (r1 -
    sigma*r2)/d``, where ``sigma=+1`` gives the outer pair and ``sigma=-1`` the
    inner (crossing) pair.

    Args:
        c1: Centre of the circle being departed.
        r1: Radius of the circle being departed (m).
        c2: Centre of the circle being approached.
        r2: Radius of the circle being approached (m).
        sense: Wrap sense on circle 1; only consistent departures are kept.

    Returns:
        Up to two ``(departure_on_c1, arrival_on_c2)`` pairs; empty if the
        circles are concentric or no bitangent matches the sense.
    """
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return []
    ux, uy = dx / d, dy / d
    out: list[tuple[Point, Point]] = []
    for sigma in (1.0, -1.0):
        k = (r1 - sigma * r2) / d
        if abs(k) > 1.0:
            continue
        root = math.sqrt(max(0.0, 1.0 - k * k))
        for pm in (1.0, -1.0):
            nx = k * ux - pm * root * uy
            ny = k * uy + pm * root * ux
            dep = (c1[0] + r1 * nx, c1[1] + r1 * ny)
            arr = (c2[0] + sigma * r2 * nx, c2[1] + sigma * r2 * ny)
            tx, ty = arr[0] - dep[0], arr[1] - dep[1]
            if math.hypot(tx, ty) < 1e-6:
                continue  # circles touch: degenerate tangent
            if (-sense * ny) * tx + (sense * nx) * ty > 0:  # sense-consistent at dep
                out.append((dep, arr))
    return out


def arc_waypoints(
    center: Point,
    radius: float,
    start_pt: Point,
    dphi: float,
    sense: WrapSense,
    theta_max_rad: float,
) -> list[PlannerState]:
    """Expand a boundary arc into circumscribed-polygon vertices.

    The arc is replaced by ``n = ceil(dphi / theta_max_rad)`` equal turns of
    ``theta = dphi/n``; vertex ``k`` is the intersection of consecutive tangent
    lines, at radius ``radius / cos(theta/2)`` on the bisector. Headings are the
    outgoing tangent directions, so the turn at every vertex is exactly
    ``theta <= theta_max_rad`` and every chord of the chain is tangent to the
    circle, never inside it.

    Args:
        center: Circle centre.
        radius: Circle radius (m).
        start_pt: Arc start point on the boundary.
        dphi: Swept angle (rad).
        sense: Wrap sense of travel.
        theta_max_rad: Maximum turn per emitted vertex (rad).

    Returns:
        ``(vertex, outgoing_heading)`` pairs, excluding the arc endpoints; empty
        for a degenerate sweep.
    """
    if dphi <= 1e-9:
        return []
    n = max(1, math.ceil(dphi / theta_max_rad))
    step = dphi / n
    rv = radius / math.cos(step / 2.0)
    phi0 = math.atan2(start_pt[1] - center[1], start_pt[0] - center[0])
    out: list[PlannerState] = []
    for k in range(n):
        mid = phi0 + sense * step * (k + 0.5)
        vertex = (center[0] + rv * math.cos(mid), center[1] + rv * math.sin(mid))
        nxt = phi0 + sense * step * (k + 1)
        tangent_pt = (
            center[0] + radius * math.cos(nxt),
            center[1] + radius * math.sin(nxt),
        )
        out.append((vertex, tangent_heading(tangent_pt, center, sense)))
    return out


def angular_overlap(a0: float, a1: float, b0: float, b1: float) -> bool:
    """Test whether two angular intervals overlap on the circle.

    Wrap-aware: intervals are compared modulo ``2*pi``.

    Args:
        a0: Low end of the first interval (rad).
        a1: High end of the first interval (rad), ``>= a0``.
        b0: Low end of the second interval (rad).
        b1: High end of the second interval (rad), ``>= b0``.

    Returns:
        ``True`` if the intervals overlap. An interval at least ``2*pi`` wide
        overlaps everything.
    """
    two_pi = 2.0 * math.pi
    wa = a1 - a0
    wb = b1 - b0
    if wa >= two_pi or wb >= two_pi:
        return True
    a = a0 % two_pi
    b = b0 % two_pi
    return ((b - a) % two_pi) < wa or ((a - b) % two_pi) < wb


def sector_polygon(
    center: Point, r_in: float, r_out: float, phi_a: float, phi_b: float
) -> PolygonCoords:
    """Build a quadrilateral covering an annular sector.

    Intended for narrow slices (a few degrees). The outer radius is padded by
    ``1/cos(width/2)`` so the quad's outer edge (a chord) fully covers the true
    outer ARC -- without the pad the chord's sagitta would leave an uncovered
    sliver of tens of metres at these radii. The inner edge is the chord at
    ``r_in``, which over-covers slightly INWARD: conservative, so it may flag
    obstacles hugging just inside ``r_in`` but never misses one in the sector.

    Args:
        center: Circle centre.
        r_in: Inner radius of the sector (m).
        r_out: Outer radius of the sector (m).
        phi_a: One angular edge of the sector (rad).
        phi_b: The other angular edge of the sector (rad).

    Returns:
        A 4-point ring, with no repeated closing point.
    """
    width = abs(phi_b - phi_a)
    r_out_pad = r_out / math.cos(min(width, math.pi / 2) / 2.0)
    ca, sa = math.cos(phi_a), math.sin(phi_a)
    cb, sb = math.cos(phi_b), math.sin(phi_b)
    return [
        (center[0] + r_in * ca, center[1] + r_in * sa),
        (center[0] + r_in * cb, center[1] + r_in * sb),
        (center[0] + r_out_pad * cb, center[1] + r_out_pad * sb),
        (center[0] + r_out_pad * ca, center[1] + r_out_pad * sa),
    ]
