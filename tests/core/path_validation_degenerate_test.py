"""Oracle regressions for two degenerate-geometry cases the planner produces
routinely once inflation is only SAFE_MARGIN.

Both were found by tracing batch_random_test failures whose paths the search had
correctly accepted:

  * a waypoint the aircraft flies STRAIGHT THROUGH (turn ~ 0) does not split the
    straight run, so the đoản-trình reserve of the next real turn may reach back
    across it (batch_random_test seed 117);
  * a fillet arc that is TANGENT to a polygon edge — which is the normal case
    when the pivot is a hull vertex and the outgoing leg runs along a hull edge
    — must not be reported as blocked because the discretised arc's tangency
    point lands a float-hair inside (seed 166).
"""

import math

from path_planning.validation import oracle as pv


def _wp(pts):
    """(waypoint, heading) pairs; headings are unused by these validators."""
    out = []
    for i, p in enumerate(pts):
        nxt = pts[i + 1] if i + 1 < len(pts) else pts[i - 1]
        out.append((p, math.atan2(nxt[1] - p[1], nxt[0] - p[0])))
    return out


# --------------------------------------------------------------------------
# 1. A straight-through waypoint must not split the straight run.
# --------------------------------------------------------------------------

R = 8000.0
L0 = 4000.0
DSS = 23000.0


def _step(p, heading_deg, dist):
    h = math.radians(heading_deg)
    return (p[0] + dist * math.cos(h), p[1] + dist * math.sin(h))


def test_collinear_waypoint_does_not_split_the_straight_run():
    """B is flown straight through, so the run A->B->C is one 5801 m straight.

    The fillet at C bites 4000 m of it, which is more than segment B->C alone
    (1701 m) but leaves 390 m once the 4100 m of A->B is counted too. This is
    batch_random_test seed 117 reduced to its bones: flyable, and the old
    per-segment rule rejected it.
    """
    O = (0.0, 0.0)
    A = _step(O, 0.0, 12000.0)
    B = _step(A, 20.0, 4100.0)  # turn 20 deg at A
    C = _step(B, 20.0, 1701.0)  # collinear: no turn at B
    D = _step(C, 20.0 + 53.130102354156, 40000.0)  # turn 53.13 deg at C

    path = _wp([O, A, B, C, D])
    alphas = pv.turn_angles(path)
    assert alphas[1] < 1e-12, f"B must be collinear, got {alphas[1]}"
    assert abs(R * math.tan(alphas[2] / 2.0) - 4000.0) < 1.0

    # the offending piece on its own really is too short for that fillet
    assert 1701.0 - 4000.0 < 0.0

    _res = pv.straight_segments_ok(path, R, L0, DSS)
    is_ok, reason = _res.is_ok, _res.detail
    assert is_ok, reason


def test_short_segment_between_two_real_turns_is_still_rejected():
    """The merge must not become a blanket amnesty: give B a real 40 deg turn
    and the two reserves genuinely collide, so the path stays invalid."""
    O = (0.0, 0.0)
    A = _step(O, 0.0, 12000.0)
    B = _step(A, 20.0, 4100.0)
    C = _step(B, 60.0, 1701.0)  # real turn at B
    D = _step(C, 60.0 + 53.130102354156, 40000.0)

    path = _wp([O, A, B, C, D])
    assert pv.turn_angles(path)[1] > math.radians(39.0)
    _res = pv.straight_segments_ok(path, R, L0, DSS)
    is_ok, reason = _res.is_ok, _res.detail
    assert not is_ok
    assert "l=" in reason


# --------------------------------------------------------------------------
# 2. A fillet tangent to a polygon edge is not a penetration.
# --------------------------------------------------------------------------

# Verbatim from batch_random_test seed 166 (the generator has drifted twice, so
# the geometry is pinned here rather than regenerated). `w` and `w_next` are
# ADJACENT vertices of the polygon, so the outgoing leg runs along its edge and
# the fillet's end tangent point lies exactly on that edge.
#
# FULL float precision matters: rounding these to 6 decimals moves the waypoints
# ~1e-7 m off the vertices, which puts a 1424 m stretch of the "edge" inside the
# polygon and manufactures a completely different failure.
SEED166_PREV = (301486.29458046495, 117440.26233684534)
SEED166_W = (278710.0631609177, 174347.77656841837)
SEED166_NEXT = (275850.3521766922, 178581.5497836887)
SEED166_POLY = [
    (286938.203922804, 180651.60756382032),
    (285006.944194373, 186743.43810365137),
    (278778.1688026068, 186657.04824654825),
    (273476.4403378112, 183864.88103284375),
    (275850.3521766922, 178581.5497836887),
    (278710.0631609177, 174347.77656841837),
    (284296.2943234772, 175450.90377605768),
]


def test_fillet_tangent_to_polygon_edge_is_clear():
    path = _wp([SEED166_PREV, SEED166_W, SEED166_NEXT])
    _res = pv.arcs_clear(path, R, [], [SEED166_POLY])
    is_ok, reason = _res.is_ok, _res.detail
    assert is_ok, reason


def test_arc_actually_entering_a_polygon_is_still_blocked():
    """Guard against the tolerance turning into a blanket amnesty: shift the
    corner deep inside the polygon and the arc must be rejected."""
    inside = (280000.0, 182000.0)
    path = _wp([SEED166_PREV, inside, SEED166_NEXT])
    _res = pv.arcs_clear(path, R, [], [SEED166_POLY])
    is_ok, reason = _res.is_ok, _res.detail
    assert not is_ok
    assert "blocked" in reason


def test_segment_running_along_a_polygon_edge_is_clear():
    """The same tolerance must keep the documented behaviour that a leg may hug
    an obstacle boundary (here: exactly along one edge, vertex to vertex)."""
    path = _wp([SEED166_W, SEED166_NEXT])
    _res = pv.segments_clear(path, [], [SEED166_POLY])
    is_ok, reason = _res.is_ok, _res.detail
    assert is_ok, reason
