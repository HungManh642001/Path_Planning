"""Pure geometry for arc-hop successors: riding detection, tangent headings,
bitangents between circles, departure points, and output-time arc expansion.

Wrap sense convention: s = +1 rides a circle counter-clockwise, s = -1
clockwise; s = sign(cross(P - center, heading_vector)).
No planner or config imports — all tolerances/steps are parameters.
"""
import math

import core.spatial_utils as su


def riding_sense(P, h, center, r, pos_tol=1.0, ang_tol=8.72e-3):
    """0 if (P, h) does not ride circle (center, r); else the wrap sense ±1.

    Riding means: P on the boundary (within pos_tol meters) AND heading h
    tangent to the circle at P (|dot(radial, heading)| < ang_tol ~ sin 0.5°).
    """
    dx, dy = P[0] - center[0], P[1] - center[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-9 or abs(dist - r) > pos_tol:
        return 0
    ux, uy = dx / dist, dy / dist
    hx, hy = math.cos(h), math.sin(h)
    if abs(ux * hx + uy * hy) > ang_tol:
        return 0
    return 1 if (ux * hy - uy * hx) > 0 else -1


def tangent_heading(P, center, s):
    """Heading (radians) of tangent travel at boundary point P, wrap sense s."""
    return math.atan2(s * (P[0] - center[0]), -s * (P[1] - center[1]))


def arc_angle(P, Q, center, s):
    """Angle in [0, 2π) swept from P to Q around center, in direction s."""
    a0 = math.atan2(P[1] - center[1], P[0] - center[0])
    a1 = math.atan2(Q[1] - center[1], Q[0] - center[0])
    return (s * (a1 - a0)) % (2.0 * math.pi)


def departure_point(X, center, r, s):
    """Tangent point on circle (center, r) from which leaving toward the
    external point X is tangent-continuous for wrap sense s. None if X is
    inside or on the circle."""
    for dep in su.circle_tangent_points(X, center, r):
        nx = (dep[0] - center[0]) / r
        ny = (dep[1] - center[1]) / r
        # velocity at dep for sense s is s * perp_ccw(n) = (-s*ny, s*nx)
        if (-s * ny) * (X[0] - dep[0]) + (s * nx) * (X[1] - dep[1]) > 0:
            return dep
    return None


def bitangent_departures(c1, r1, c2, r2, s):
    """Bitangent lines of circles (c1, r1) and (c2, r2), filtered to those
    departing circle 1 consistently with wrap sense s.

    Construction: a bitangent touches circle 1 at c1 + r1*n and circle 2 at
    c2 + sigma*r2*n for a unit normal n with n·D̂ = (r1 - sigma*r2)/d, where
    sigma=+1 gives the outer pair and sigma=-1 the inner (crossing) pair.
    Returns [(dep_on_c1, arr_on_c2), ...] (0..2 entries after filtering).
    """
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return []
    ux, uy = dx / d, dy / d
    out = []
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
            if (-s * ny) * tx + (s * nx) * ty > 0:  # sense-consistent at dep
                out.append((dep, arr))
    return out


def arc_waypoints(center, r, start_pt, dphi, s, theta_max_rad):
    """Expand a boundary arc into circumscribed-polygon vertices.

    The arc starts at start_pt (on the circle) and sweeps dphi (rad) in
    direction s. It is replaced by n = ceil(dphi / theta_max_rad) equal turns
    of theta = dphi/n each; vertex k is the intersection of consecutive
    tangent lines, at radius r / cos(theta/2) on the bisector. Headings are
    the outgoing tangent directions, so the turn angle at every vertex is
    exactly theta <= theta_max_rad, and every chord of the resulting chain is
    tangent to the circle (never inside it).

    Returns [(vertex, heading_out), ...] excluding the arc endpoints.
    """
    if dphi <= 1e-9:
        return []
    n = max(1, int(math.ceil(dphi / theta_max_rad)))
    step = dphi / n
    rv = r / math.cos(step / 2.0)
    phi0 = math.atan2(start_pt[1] - center[1], start_pt[0] - center[0])
    out = []
    for k in range(n):
        mid = phi0 + s * step * (k + 0.5)
        vertex = (center[0] + rv * math.cos(mid), center[1] + rv * math.sin(mid))
        nxt = phi0 + s * step * (k + 1)
        tangent_pt = (center[0] + r * math.cos(nxt), center[1] + r * math.sin(nxt))
        out.append((vertex, tangent_heading(tangent_pt, center, s)))
    return out


def angular_overlap(a0, a1, b0, b1):
    """True iff angular intervals [a0, a1] and [b0, b1] overlap on the circle.

    Each interval is given as (lo, hi) with hi >= lo in radians (any absolute
    values; widths <= 2*pi assumed meaningful — a width >= 2*pi overlaps
    everything). Wrap-aware: intervals are compared modulo 2*pi.
    """
    two_pi = 2.0 * math.pi
    wa = a1 - a0
    wb = b1 - b0
    if wa >= two_pi or wb >= two_pi:
        return True
    a = a0 % two_pi
    b = b0 % two_pi
    return ((b - a) % two_pi) < wa or ((a - b) % two_pi) < wb


def sector_polygon(center, r_in, r_out, phi_a, phi_b):
    """Quadrilateral covering the annular sector [r_in, r_out] x [phi_a, phi_b].

    Intended for narrow slices (a few degrees). The outer radius is padded by
    1/cos(width/2) so the quad's outer edge (a chord) fully covers the true
    outer ARC — without the pad the chord's sagitta would leave an uncovered
    sliver of tens of metres at these radii. The inner edge is the chord at
    r_in, which over-covers slightly INWARD (conservative: may flag obstacles
    hugging just inside r_in; never misses one inside the sector).

    Returns a 4-point coordinate list (no closing point).
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
