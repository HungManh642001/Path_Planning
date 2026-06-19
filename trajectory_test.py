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
