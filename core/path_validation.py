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
    # A segment is blocked ONLY when it enters a polygon's INTERIOR (DE-9IM
    # interior/interior overlap, pattern 'T********'). Touching the boundary is
    # allowed: a waypoint may sit on a polygon corner (corners are valid navigation
    # targets, like circle tangent points), and a segment may run ALONG an edge to
    # hug the obstacle boundary. Interior penetration still fails.
    line = LineString([a, b])
    for coords in polygon_obstacles:
        if Polygon(coords).relate_pattern(line, 'T********'):
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


def _seg_heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _norm(delta):
    return math.atan2(math.sin(delta), math.cos(delta))


def turn_angles(path):
    """Turn angle (rad, magnitude) at each interior waypoint, from segment geometry."""
    angles = []
    for i in range(1, len(path) - 1):
        h_in = _seg_heading(path[i - 1][0], path[i][0])
        h_out = _seg_heading(path[i][0], path[i + 1][0])
        angles.append(abs(_norm(h_out - h_in)))
    return angles


def turn_angles_ok(path, alpha_max_rad):
    return all(a <= alpha_max_rad + 1e-9 for a in turn_angles(path))


def _seg_len(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def straight_segments_ok(path, R, L0, dss):
    """Check đoản trình straight-portion constraints from the spec.

    Returns (ok, detail). alpha at interior waypoints comes from turn_angles();
    endpoints have no turn before/after them (alpha = 0 at O and at T).
    """
    n_seg = len(path) - 1
    if n_seg < 1:
        return True, "trivial"
    alphas = [0.0] + turn_angles(path) + [0.0]  # alpha at each waypoint index
    for i in range(n_seg):
        d = _seg_len(path[i][0], path[i + 1][0])
        a_i = alphas[i]
        a_next = alphas[i + 1]
        l = d - R * (math.tan(a_i / 2) + math.tan(a_next / 2))
        if i == 0:                       # first đoản trình: l1 >= L0
            if l < L0 - 1.0:
                return False, f"first segment l={l:.1f} < L0={L0}"
        elif i == n_seg - 1:             # last đoản trình: ln = l - dss >= 0
            if l - dss < -1.0:
                return False, f"last segment usable l={l - dss:.1f} < 0"
        else:                            # middle: l > 0
            if l < 1.0:
                return False, f"middle segment {i} l={l:.1f} <= 0"
    return True, "ok"


def _unit(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    return (dx / d, dy / d) if d > 0 else (0.0, 0.0)


def _arc_points(w_prev, w, w_next, R, n=24):
    """Sample the radius-R turn arc that replaces corner w."""
    u = _unit(w_prev, w)      # incoming direction
    v = _unit(w, w_next)      # outgoing direction
    alpha = abs(_norm(math.atan2(v[1], v[0]) - math.atan2(u[1], u[0])))
    if alpha < 1e-9:
        return []
    t = R * math.tan(alpha / 2)              # tangent length along each leg
    A = (w[0] - u[0] * t, w[1] - u[1] * t)   # tangent point on incoming leg
    s = 1.0 if (u[0] * v[1] - u[1] * v[0]) > 0 else -1.0   # left(+)/right(-) turn
    n_in = (-u[1] * s, u[0] * s)             # inward normal of incoming leg
    C = (A[0] + R * n_in[0], A[1] + R * n_in[1])   # arc centre
    start = math.atan2(A[1] - C[1], A[0] - C[0])
    pts = []
    for k in range(n + 1):
        ang = start + s * alpha * (k / n)
        pts.append((C[0] + R * math.cos(ang), C[1] + R * math.sin(ang)))
    return pts


def arcs_clear(path, R, circle_obstacles, polygon_obstacles):
    """True iff every turn arc clears all obstacles."""
    for i in range(1, len(path) - 1):
        pts = _arc_points(path[i - 1][0], path[i][0], path[i + 1][0], R)
        for j in range(len(pts) - 1):
            if not _segment_clear(pts[j], pts[j + 1], circle_obstacles, polygon_obstacles):
                return False
    return True


def path_is_valid(path, circle_obstacles, polygon_obstacles, R, alpha_max_rad, L0, dss,
                  raw_circle_obstacles=None, raw_polygon_obstacles=None):
    """One-call full validity gate used by later phases.

    Straight segments must clear the INFLATED obstacles (keeping the full safety
    margin on the straight legs). Turn arcs, however, are designed to bulge into
    the inflation band by up to R*(1/cos(alpha_max/2)-1) and only need to clear
    the RAW obstacle — so when the raw obstacle sets are supplied, arcs are
    validated against them. They default to the inflated sets for backward
    compatibility (correct for circle-tangent paths, whose arcs bulge outward).
    """
    if not path or len(path) < 2:
        return False
    if not segments_clear(path, circle_obstacles, polygon_obstacles):
        return False
    if not turn_angles_ok(path, alpha_max_rad):
        return False
    arc_circles = raw_circle_obstacles if raw_circle_obstacles is not None else circle_obstacles
    arc_polys = raw_polygon_obstacles if raw_polygon_obstacles is not None else polygon_obstacles
    if not arcs_clear(path, R, arc_circles, arc_polys):
        return False
    ok, _ = straight_segments_ok(path, R, L0, dss)
    return ok
