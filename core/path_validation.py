"""Validators for produced planner paths (spec: Điều kiện ràng buộc đường bay).

A path is a list of (waypoint, heading) tuples with waypoint = (x, y).
These functions are deliberately independent of the planner internals so
tests can assert validity without trusting the code under review.
"""
import math
from shapely.geometry import Polygon, LineString


# Shortest interior overlap with a polygon that this validator can still tell
# apart from a tangency (metres).
#
# This is NOT a forgiveness of the path — it is the resolution limit of the
# validator's OWN arithmetic, in the same family as TURN_RESERVE_TOL_M below.
# The exact predicate is "positive-length interior overlap", but shapely cannot
# compute "positive" to better than about one ULP of the coordinates: a fillet
# arc TANGENT to a hull edge (the normal case, since inflation is only
# SAFE_MARGIN and hull vertices then sit exactly on the raw polygon) reports an
# overlap of a few nanometres. Measured on the three paths that a zero threshold
# rejected: 8.1e-9 m, 4.4e-9 m and 5.8e-11 m — the last is 0.06 nanometres. At 0
# the oracle rejects flyable missions because of its own rounding (3 of 300
# scenarios, v0).
#
# 1e-6 m is 100x above that noise and still 6 orders below anything operational
# (SAFE_MARGIN is metres; a real crossing runs to kilometres). It was 1e-3 m,
# which WAS a genuine forgiveness — 1000x more than the artefact needs.
POLYGON_TOUCH_TOL_M = 1e-6

# A turn whose fillet bites less than this many metres out of the straight is
# not a turn: the waypoint is flown straight through and does not split the
# straight run (see straight_segments_ok). At R = 8000 m this is a turn of
# 1.4e-5 degrees, so it only ever absorbs float noise in collinear waypoints,
# never a manoeuvre.
#
# This is the one number here that is NOT a check, and it deliberately keeps a
# tolerance. It CLASSIFIES waypoints (does this one split the straight run?)
# rather than comparing a quantity against a limit. Driving it to 0 would make
# every float-noise reserve at a collinear waypoint split the run, manufacturing
# zero-length segments that then fail the exact `l > 0` test — the checker would
# reject paths for its own rounding. The planner emits genuinely collinear
# waypoints as a matter of course (arc-hop departures leave tangentially, and a
# pivot slide flies straight THROUGH its parent), so this case is the norm, not
# an edge. Like every classifier tolerance, it must stay above construction
# noise: keep it well clear of GEOM_EPS_M.
TURN_RESERVE_TOL_M = 1e-6


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


def interior_overlap_length(poly, line):
    """Length of `line` that lies strictly INSIDE `poly`, boundary excluded.

    `poly.intersection(line).length` is the wrong quantity for this: it measures
    the overlap with the CLOSED polygon (interior UNION boundary), so a chord
    that legitimately runs ALONG a hull edge scores its whole edge-following
    stretch -- kilometres -- even though it never enters the interior. Measured
    on batch_random_test seed 194: 11533.475 m of "overlap", of which 11533.475 m
    is on the boundary and 0.0 m is inside. Subtracting the boundary part leaves
    the penetration itself.

    Shared with the planners on purpose: a planner stricter than its own oracle
    rejects flyable chords, and one more lenient produces paths the oracle then
    fails. There must be exactly one answer to "how far does this chord go
    inside this polygon".
    """
    return poly.intersection(line).length - poly.boundary.intersection(line).length


def _segment_clear(a, b, circle_obstacles, polygon_obstacles):
    for center, radius in circle_obstacles:
        # EXACT: a circle is hit iff the segment comes closer than its radius.
        # No leniency — a check never forgives a real intrusion, otherwise a
        # reported clearance is not the true one. Planner-made chords keep their
        # margin because the geometry is BUILT on radius + CONSTRUCTION_CLEARANCE_M
        # + GEOM_EPS_M. Measured over 120 scenarios: the closest any accepted
        # segment came was 0.112 m OUTSIDE the boundary.
        if _point_to_segment_distance(center, a, b) < radius:
            return False
    # A segment is blocked ONLY when it enters a polygon's INTERIOR (DE-9IM
    # interior/interior overlap, pattern 'T********'). Touching the boundary is
    # allowed: a waypoint may sit on a polygon corner (corners are valid navigation
    # goals, like circle tangent points), and a segment may run ALONG an edge to
    # hug the obstacle boundary. Interior penetration still fails.
    #
    # The interior overlap must have POSITIVE MEASURE. 'T********' also matches a
    # zero-length touch, and that is not a hypothetical: a fillet arc is TANGENT
    # to a polygon edge whenever the pivot is a hull vertex and the outgoing leg
    # runs along a hull edge -- the normal case once inflation is only
    # SAFE_MARGIN, since the hull vertices then lie exactly on the raw polygon.
    # The discretised arc's tangency point lands a float-hair inside and shapely
    # reports a Point of length 0 as an interior intersection (batch_random_test
    # seed 166). Requiring positive length keeps every real crossing -- those are
    # metres to kilometres long -- while ignoring a touch that has no extent.
    #
    # It must be the INTERIOR length, not the closed-polygon one -- see
    # interior_overlap_length. 'T********' can also disagree with the geometry it
    # is derived from: on seed 194 it reports a dimension-1 interior overlap for
    # a chord whose interior overlap measures exactly 0.0 m, because the chord
    # runs along an edge and the two predicates node it differently. Measuring
    # settles it.
    line = LineString([a, b])
    for coords in polygon_obstacles:
        poly = Polygon(coords)
        if not poly.relate_pattern(line, 'T********'):
            continue
        if interior_overlap_length(poly, line) > POLYGON_TOUCH_TOL_M:
            return False
    return True


def segments_clear(path, circle_obstacles, polygon_obstacles):
    """True iff every straight segment between consecutive waypoints is clear."""
    for i in range(len(path) - 1):
        a = path[i][0]
        b = path[i + 1][0]
        if not _segment_clear(a, b, circle_obstacles, polygon_obstacles):
            return False, f"segment {i} blocked ({a} -> {b})"
    return True, "ok"


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
    angles = turn_angles(path)
    for i, a in enumerate(angles):
        if a > alpha_max_rad:            # exact; the planner builds to alpha_max - GEOM_EPS_RAD
            return False, f"wp[{i + 1}] {path[i + 1][0]} turn angle {math.degrees(a):.3f}° > alpha_max {math.degrees(alpha_max_rad):.3f}°"
    return True, "ok"


def _seg_len(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def straight_segments_ok(path, R, L0, dss):
    """Check đoản trình straight-portion constraints from the spec.

    Returns (ok, detail). alpha at interior waypoints comes from turn_angles();
    endpoints have no turn before/after them (alpha = 0 at O and at T).

    The constraint is about the straight FLOWN BETWEEN TWO TURNS, which is not
    the same as a segment between two waypoints: a waypoint whose turn is zero
    is flown straight through and does not split the straight run. The planner
    emits such waypoints routinely -- an arc-hop departure leaves tangentially,
    and an along-ray pivot slide deliberately flies straight through the
    original candidate before turning at the slid pivot -- so consecutive
    segments are merged across them here. Charging each segment the full reserve
    of the turns at both its endpoints instead double-counts the split and
    rejects flyable paths (batch_random_test seed 117: a 1701 m segment between
    two collinear waypoints was charged the 4000 m fillet of the next real turn,
    which in fact reaches back harmlessly into the 4100 m segment before it).
    """
    n_seg = len(path) - 1
    if n_seg < 1:
        return True, "trivial"
    alphas = [0.0] + turn_angles(path) + [0.0]  # alpha at each waypoint index
    reserves = [R * math.tan(a / 2) for a in alphas]

    # Waypoints that actually bend the path delimit the straight runs; the
    # endpoints always do (they carry reserve 0 and bound the first/last run).
    breaks = [0]
    breaks.extend(i for i in range(1, n_seg) if reserves[i] > TURN_RESERVE_TOL_M)
    breaks.append(n_seg)

    for k in range(len(breaks) - 1):
        i, j = breaks[k], breaks[k + 1]
        d = sum(_seg_len(path[m][0], path[m + 1][0]) for m in range(i, j))
        l = d - reserves[i] - reserves[j]
        span = f"wp[{i}]..wp[{j}]" if j > i + 1 else f"segment {i}"
        # All three are EXACT: the 1 m allowances they used to carry were
        # forgiving real violations. Feasibility comes from the construction
        # side — start corners are built at L0 + GEOM_EPS_M, turns at
        # alpha_max - GEOM_EPS_RAD. Measured worst margins over 120 scenarios of
        # accepted paths: L0 +9.96e-9 m, DSS +413 m, middle +96 m.
        if i == 0:                       # first đoản trình: l1 >= L0
            if l < L0:
                return False, f"first {span} l={l:.3f} < L0={L0}"
        elif j == n_seg:                 # last đoản trình: ln = l - dss >= 0
            if l - dss < 0.0:
                return False, f"last {span} usable l={l - dss:.3f} < 0"
        else:                            # middle: l > 0
            if l <= 0.0:
                return False, f"middle {span} l={l:.3f} <= 0"
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
                return False, f"turn arc at wp[{i}] {path[i][0]} blocked ({pts[j]} -> {pts[j + 1]})"
    return True, "ok"


def path_is_valid(path, circle_obstacles, polygon_obstacles, R, alpha_max_rad, L0, dss,
                  raw_circle_obstacles=None, raw_polygon_obstacles=None):
    """One-call full validity gate used by later phases.

    Straight segments AND turn arcs must both clear the INFLATED obstacles,
    which are now simply raw + SAFE_MARGIN: the whole flown path honours the
    operator's minimum stand-off.

    The `raw_*` parameters are a legacy escape hatch. They existed because
    inflation used to carry a `R*(1/cos(alpha_max/2)-1)` turn term and a fillet
    arc was designed to bulge into exactly that band, so arcs were validated
    against the raw obstacle instead. With the turn term gone there is no band,
    and passing raw sets here would let a turn dip inside SAFE_MARGIN. Leave
    them unset unless you are deliberately reproducing the old model.
    """
    if not path or len(path) < 2:
        return False, "path too short"
    ok, reason = segments_clear(path, circle_obstacles, polygon_obstacles)
    if not ok:
        return False, f"segments blocked: {reason}"
    ok, reason = turn_angles_ok(path, alpha_max_rad)
    if not ok:
        return False, f"turn angles invalid: {reason}"
    arc_circles = raw_circle_obstacles if raw_circle_obstacles is not None else circle_obstacles
    arc_polys = raw_polygon_obstacles if raw_polygon_obstacles is not None else polygon_obstacles
    ok, reason = arcs_clear(path, R, arc_circles, arc_polys)
    if not ok:
        return False, f"turn arcs blocked: {reason}"
    ok, reason = straight_segments_ok(path, R, L0, dss)
    if not ok:
        return False, f"straight segments invalid: {reason}"
    return True, "ok"
