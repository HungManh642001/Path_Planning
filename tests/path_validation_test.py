"""Oracle checks are EXACT — no allowance, at any depth.

This file used to assert the opposite: that `circle_tol` forgave a
sub-tolerance graze. That parameter is gone. A check never forgives a real
intrusion, because a clearance the oracle reports must be the clearance the
aircraft actually has; feasibility is bought on the CONSTRUCTION side instead
(geometry is built on radius + CONSTRUCTION_CLEARANCE_M + GEOM_EPS_M, turns at
alpha_max - GEOM_EPS_RAD, the takeoff straight at L0 + GEOM_EPS_M).
"""
import math
import core.path_validation as pv

# A single inflated circle at the origin, radius 1000; a horizontal segment
# whose closest approach to the center is (radius - depth).
C = (0.0, 0.0)
R = 1000.0


def _seg_at_depth(depth):
    y = R - depth              # closest-approach distance from center
    return (-5000.0, y), (5000.0, y)


def test_any_penetration_fails_however_shallow():
    for depth in (0.5, 1e-3, 1e-6, 1e-9):
        a, b = _seg_at_depth(depth)
        assert pv._segment_clear(a, b, [(C, R)], []) is False, \
            f'{depth} m of penetration was forgiven'


def test_clear_segment_passes():
    a, b = _seg_at_depth(-50.0)  # 50 m OUTSIDE the boundary
    assert pv._segment_clear(a, b, [(C, R)], []) is True


def test_grazing_exactly_on_the_boundary_passes():
    """Distance == radius is not a hit: the test is `< radius`. This is the case
    the construction lift exists to keep on the safe side of."""
    a, b = _seg_at_depth(0.0)
    assert pv._segment_clear(a, b, [(C, R)], []) is True


def test_polygon_interior_fails():
    square = [(-100.0, -100.0), (100.0, -100.0), (100.0, 100.0), (-100.0, 100.0)]
    a, b = (-500.0, 0.0), (500.0, 0.0)
    assert pv._segment_clear(a, b, [], [square]) is False


def test_polygon_boundary_touch_passes():
    """Running ALONG an edge is allowed — a hull vertex is a legal navigation
    target — because only a positive-length INTERIOR overlap counts."""
    square = [(-100.0, -100.0), (100.0, -100.0), (100.0, 100.0), (-100.0, 100.0)]
    a, b = (-500.0, 100.0), (500.0, 100.0)      # along the top edge
    assert pv._segment_clear(a, b, [], [square]) is True


def test_path_is_valid_rejects_a_grazing_path():
    a, b = _seg_at_depth(0.5)
    path = [(a, 0.0), (b, 0.0)]
    ok, reason = pv.path_is_valid(
        path, circle_obstacles=[(C, R)], polygon_obstacles=[],
        R=8000.0, alpha_max_rad=math.radians(90), L0=4000.0, dss=23000.0,
        raw_circle_obstacles=[(C, 1.0)], raw_polygon_obstacles=[])
    assert ok is False, reason
    assert 'segment' in reason


def test_straight_run_checks_carry_no_metre_of_slack():
    """L0 used to forgive 1 m; now landing exactly on it passes and a millimetre
    under fails. (A two-waypoint path is a single run, which the checker treats
    as the FIRST one, so this exercises the L0 branch.)"""
    R_turn, L0, dss = 8000.0, 4000.0, 23000.0
    for total, expect in ((L0, True), (L0 - 1e-3, False)):
        path = [((0.0, 0.0), 0.0), ((total, 0.0), 0.0)]
        ok, reason = pv.straight_segments_ok(path, R_turn, L0, dss)
        assert ok is expect, f'total={total}: {reason}'


def test_turn_angle_check_carries_no_slack():
    """A turn one float-hair over alpha_max is over alpha_max."""
    amax = math.radians(90)
    over = amax * (1 + 1e-12)
    path = [((0.0, 0.0), 0.0), ((1000.0, 0.0), 0.0),
            ((1000.0 + 1000.0 * math.cos(over), 1000.0 * math.sin(over)), 0.0)]
    ok, _ = pv.turn_angles_ok(path, amax)
    assert ok is False
