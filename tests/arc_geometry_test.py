"""Unit tests for core/arc_geometry.py (pure geometry, no planner)."""
import math

import core.arc_geometry as ag
import core.path_validation as pv
import core.spatial_utils as su

C = (100000.0, 100000.0)
R_C = 20000.0


def test_riding_sense_ccw():
    # Point due east of center, heading north => CCW tangent
    P = (C[0] + R_C, C[1])
    assert ag.riding_sense(P, math.pi / 2, C, R_C) == 1


def test_riding_sense_cw():
    P = (C[0] + R_C, C[1])
    assert ag.riding_sense(P, -math.pi / 2, C, R_C) == -1


def test_riding_sense_rejects_off_boundary():
    P = (C[0] + R_C + 50.0, C[1])  # 50 m off the boundary; tol is 1 m
    assert ag.riding_sense(P, math.pi / 2, C, R_C) == 0


def test_riding_sense_rejects_non_tangent_heading():
    P = (C[0] + R_C, C[1])
    assert ag.riding_sense(P, math.pi / 4, C, R_C) == 0  # 45 deg off tangent


def test_tangent_heading_matches_sense():
    P = (C[0] + R_C, C[1])
    assert math.isclose(ag.tangent_heading(P, C, +1), math.pi / 2, abs_tol=1e-9)
    assert math.isclose(ag.tangent_heading(P, C, -1), -math.pi / 2, abs_tol=1e-9)


def test_arc_angle_quarter_turns():
    P = (C[0] + R_C, C[1])  # polar angle 0
    Q = (C[0], C[1] + R_C)  # polar angle pi/2
    assert math.isclose(ag.arc_angle(P, Q, C, +1), math.pi / 2, rel_tol=1e-9)
    assert math.isclose(ag.arc_angle(P, Q, C, -1), 3 * math.pi / 2, rel_tol=1e-9)


def test_arc_angle_same_point_is_zero():
    P = (C[0] + R_C, C[1])
    assert ag.arc_angle(P, P, C, +1) == 0.0


def test_departure_point_picks_sense_consistent_tangent():
    X = (C[0] + 100000.0, C[1])  # far east of the circle
    dep_ccw = ag.departure_point(X, C, R_C, +1)
    dep_cw = ag.departure_point(X, C, R_C, -1)
    assert dep_ccw is not None and dep_cw is not None
    # CCW wrap leaves from below the center-line, CW from above.
    assert dep_ccw[1] < C[1]
    assert dep_cw[1] > C[1]
    for s, dep in ((+1, dep_ccw), (-1, dep_cw)):
        # On the boundary
        assert math.isclose(math.hypot(dep[0] - C[0], dep[1] - C[1]), R_C, rel_tol=1e-9)
        # Leave direction actually points toward X
        h = ag.tangent_heading(dep, C, s)
        to_x = math.atan2(X[1] - dep[1], X[0] - dep[0])
        assert abs(math.atan2(math.sin(h - to_x), math.cos(h - to_x))) < 1e-6


def test_departure_point_inside_returns_none():
    assert ag.departure_point((C[0] + 1000.0, C[1]), C, R_C, +1) is None


def test_bitangent_departures_disjoint_circles():
    c1, r1 = (0.0, 0.0), 10000.0
    c2, r2 = (100000.0, 0.0), 10000.0
    res = ag.bitangent_departures(c1, r1, c2, r2, +1)
    assert len(res) == 2  # one outer + one inner survive the sense filter
    for dep, arr in res:
        assert math.isclose(math.hypot(dep[0] - c1[0], dep[1] - c1[1]), r1, rel_tol=1e-9)
        assert math.isclose(math.hypot(arr[0] - c2[0], arr[1] - c2[1]), r2, rel_tol=1e-9)
        # The line dep->arr is tangent to both circles.
        assert math.isclose(su.point_to_line_distance(c1, dep, arr), r1, rel_tol=1e-6)
        assert math.isclose(su.point_to_line_distance(c2, dep, arr), r2, rel_tol=1e-6)
        # Departure is sense-consistent: tangent heading at dep points at arr.
        h = ag.tangent_heading(dep, c1, +1)
        to_arr = math.atan2(arr[1] - dep[1], arr[0] - dep[0])
        assert abs(math.atan2(math.sin(h - to_arr), math.cos(h - to_arr))) < 1e-6
    # CCW wrap on c1 must include the bottom outer tangent.
    assert any(dep[1] < 0 for dep, _ in res)


def test_bitangent_departures_overlapping_circles_outer_only():
    c1, r1 = (0.0, 0.0), 20000.0
    c2, r2 = (30000.0, 0.0), 20000.0  # overlapping: inner tangents impossible
    res = ag.bitangent_departures(c1, r1, c2, r2, +1)
    assert len(res) == 1


def test_bitangent_departures_concentric_returns_empty():
    assert ag.bitangent_departures((0.0, 0.0), 10000.0, (0.0, 0.0), 5000.0, +1) == []


def test_arc_waypoints_quarter_wrap_vertex_geometry():
    start = (C[0] + R_C, C[1])  # polar angle 0
    dphi = math.pi / 2
    theta = math.radians(30.0)
    wps = ag.arc_waypoints(C, R_C, start, dphi, +1, theta)
    assert len(wps) == 3  # ceil(90/30) = 3 vertices
    rv = R_C / math.cos((dphi / 3) / 2)
    for v, _h in wps:
        assert math.isclose(math.hypot(v[0] - C[0], v[1] - C[1]), rv, rel_tol=1e-9)


def test_arc_waypoint_chain_turns_and_clearance():
    """The full chain start -> vertices -> end must satisfy exactly what the
    oracle checks: every turn <= theta and no chord entering the circle."""
    start = (C[0] + R_C, C[1])
    dphi = 1.75 * math.pi  # long wrap
    theta = math.radians(30.0)
    wps = ag.arc_waypoints(C, R_C, start, dphi, +1, theta)
    end = (C[0] + R_C * math.cos(dphi), C[1] + R_C * math.sin(dphi))
    chain = [(start, 0.0)] + wps + [(end, 0.0)]
    for a in pv.turn_angles(chain):
        assert a <= theta + 1e-9
    for i in range(len(chain) - 1):
        d = su.point_to_line_distance(C, chain[i][0], chain[i + 1][0])
        assert d >= R_C - 1e-6  # chords are tangent, never inside


def test_arc_waypoints_cw_sense():
    start = (C[0] + R_C, C[1])
    theta = math.radians(30.0)
    wps = ag.arc_waypoints(C, R_C, start, math.pi / 2, -1, theta)
    assert len(wps) == 3
    assert all(v[1] < C[1] for v, _h in wps)  # CW from angle 0 goes below


def test_arc_waypoints_zero_angle_empty():
    start = (C[0] + R_C, C[1])
    assert ag.arc_waypoints(C, R_C, start, 0.0, +1, math.radians(30.0)) == []
