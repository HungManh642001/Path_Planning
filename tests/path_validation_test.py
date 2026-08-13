"""Oracle tolerance behavior — circle_tol forgives sub-tolerance circle grazes
but never a polygon interior hit; default stays strict."""
import math
import core.path_validation as pv

# A single inflated circle at the origin, radius 1000; a horizontal segment
# whose closest approach to the center is (radius - depth).
C = (0.0, 0.0)
R = 1000.0


def _seg_at_depth(depth):
    y = R - depth              # closest-approach distance from center
    return (-5000.0, y), (5000.0, y)


def test_default_tol_is_strict():
    a, b = _seg_at_depth(0.5)   # 0.5 m inside the inflated boundary
    assert pv._segment_clear(a, b, [(C, R)], []) is False


def test_circle_tol_forgives_subtolerance_graze():
    a, b = _seg_at_depth(0.5)
    assert pv._segment_clear(a, b, [(C, R)], [], circle_tol=1.0) is True


def test_circle_tol_still_rejects_beyond_tolerance():
    a, b = _seg_at_depth(2.0)   # 2 m inside, tol only 1 m
    assert pv._segment_clear(a, b, [(C, R)], [], circle_tol=1.0) is False


def test_clear_segment_passes():
    a, b = _seg_at_depth(-50.0)  # 50 m OUTSIDE the boundary
    assert pv._segment_clear(a, b, [(C, R)], [], circle_tol=1.0) is True


def test_polygon_interior_fails_regardless_of_circle_tol():
    # Segment straight through a square's interior; circle_tol must not forgive it.
    square = [(-100.0, -100.0), (100.0, -100.0), (100.0, 100.0), (-100.0, 100.0)]
    a, b = (-500.0, 0.0), (500.0, 0.0)
    assert pv._segment_clear(a, b, [], [square], circle_tol=1000.0) is False


def test_path_is_valid_threads_circle_tol():
    # Two-waypoint path grazing the circle 0.5 m; strict default rejects,
    # circle_tol=1.0 accepts (turn/segment-length checks are trivially ok here).
    a, b = _seg_at_depth(0.5)
    path = [(a, 0.0), (b, 0.0)]
    common = dict(circle_obstacles=[(C, R)], polygon_obstacles=[],
                  R=8000.0, alpha_max_rad=math.radians(90), L0=4000.0, dss=23000.0,
                  raw_circle_obstacles=[(C, 1.0)], raw_polygon_obstacles=[])
    # path_is_valid reports (ok, reason) so plan_trajectory can surface WHICH
    # constraint failed; only the verdict matters here.
    ok_strict, reason = pv.path_is_valid(path, **common)
    assert ok_strict is False, reason
    assert 'segment' in reason
    ok_lenient, _ = pv.path_is_valid(path, **common, circle_tol=1.0)
    assert ok_lenient is True
