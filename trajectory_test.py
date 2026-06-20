import math
import trajectory as tr


def test_straight_mode_returns_waypoint_polyline():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0), ((100.0, 100.0), math.pi / 2)]
    pts = tr.sample_trajectory(path, R=8000.0, mode='straight')
    assert pts == [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]


def test_dubins_mode_is_continuous_and_hits_endpoints():
    path = [((0.0, 0.0), 0.0), ((40000.0, 0.0), 0.0), ((40000.0, 40000.0), math.pi / 2)]
    pts = tr.sample_trajectory(path, R=8000.0, mode='dubins')
    assert len(pts) >= 3
    assert math.hypot(pts[0][0] - 0.0, pts[0][1] - 0.0) < 1.0
    assert math.hypot(pts[-1][0] - 40000.0, pts[-1][1] - 40000.0) < 1.0
    gaps = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            for i in range(len(pts) - 1)]
    assert max(gaps) < 8000.0, "dubins trajectory must be continuous (no segment dropped)"


def test_single_waypoint_returns_itself():
    assert tr.sample_trajectory([((1.0, 2.0), 0.0)], R=8000.0, mode='dubins') == [(1.0, 2.0)]


def _circle_radius_through(p1, p2, p3):
    ax, ay = p1; bx, by = p2; cx, cy = p3
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay)
          + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx)
          + (cx * cx + cy * cy) * (bx - ax)) / d
    return math.hypot(ax - ux, ay - uy)


def test_dubins_turn_arc_has_waypoint_at_midpoint_and_radius_R():
    # The flown turn arc must use the real turn radius R and put the WAYPOINT
    # exactly at the middle of the arc (user requirement).
    R = 8000.0
    path = [((0.0, 0.0), 0.0), ((100000.0, 0.0), 0.0), ((100000.0, 100000.0), math.pi / 2)]
    turns = tr.turn_markers(path, R)
    assert len(turns) == 1
    t = turns[0]
    assert t['mid'] == (100000.0, 0.0)                       # waypoint == arc midpoint
    r = _circle_radius_through(t['start'], t['mid'], t['end'])
    assert abs(r - R) < 1.0                                   # arc radius == R
    # midpoint is symmetric: equidistant from the two turn endpoints
    assert abs(math.dist(t['mid'], t['start']) - math.dist(t['mid'], t['end'])) < 1.0


def test_dubins_path_passes_through_each_waypoint():
    R = 8000.0
    path = [((0.0, 0.0), 0.0), ((100000.0, 0.0), 0.0), ((100000.0, 100000.0), math.pi / 2)]
    pts = tr.sample_trajectory(path, R, mode='dubins')
    W = (100000.0, 0.0)
    assert min(math.dist(p, W) for p in pts) < 1.0           # path goes through the waypoint


def test_build_full_path_prepends_launch_and_appends_target():
    pre = {'start_pos': (0.0, 0.0), 'goal_pos': (500.0, 0.0),
           'start_heading': 0.0, 'goal_heading': 0.0}
    path = [((100.0, 0.0), 0.0), ((400.0, 0.0), 0.0)]
    full = tr.build_full_path(path, pre)
    assert full[0][0] == (0.0, 0.0)                          # launch O first
    assert full[-1][0] == (500.0, 0.0)                       # target T last
    assert len(full) == 4


def test_build_full_path_no_duplicate_when_already_at_endpoints():
    pre = {'start_pos': (100.0, 0.0), 'goal_pos': (400.0, 0.0),
           'start_heading': 0.0, 'goal_heading': 0.0}
    path = [((100.0, 0.0), 0.0), ((400.0, 0.0), 0.0)]
    full = tr.build_full_path(path, pre)
    assert len(full) == 2                                     # no O/T duplication
