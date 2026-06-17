import math
import path_validation as pv


def test_segment_clear_returns_true_when_no_obstacle():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0)]
    assert pv.segments_clear(path, circle_obstacles=[], polygon_obstacles=[]) is True


def test_segment_blocked_by_circle_on_the_line():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0)]
    assert pv.segments_clear(path, circle_obstacles=[((50.0, 0.0), 10.0)],
                             polygon_obstacles=[]) is False


def test_segment_clear_when_circle_far_from_line():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0)]
    assert pv.segments_clear(path, circle_obstacles=[((50.0, 1000.0), 10.0)],
                             polygon_obstacles=[]) is True


def test_segment_blocked_by_polygon():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0)]
    poly = [(40.0, -10.0), (60.0, -10.0), (60.0, 10.0), (40.0, 10.0)]
    assert pv.segments_clear(path, circle_obstacles=[], polygon_obstacles=[poly]) is False


def test_multi_segment_path_blocked_on_second_leg():
    # 3 waypoints; first leg clear, second leg passes through a circle.
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0), ((100.0, 100.0), 0.0)]
    blocking = ((100.0, 50.0), 10.0)  # sits on the second (vertical) leg
    assert pv.segments_clear(path, circle_obstacles=[blocking], polygon_obstacles=[]) is False


def test_turn_angles_straight_line_is_zero():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0), ((200.0, 0.0), 0.0)]
    angles = pv.turn_angles(path)
    assert len(angles) == 1
    assert abs(angles[0]) < 1e-9


def test_turn_angles_right_angle():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0), ((100.0, 100.0), 0.0)]
    angles = pv.turn_angles(path)
    assert abs(angles[0] - math.pi / 2) < 1e-9


def test_turn_angle_limit_ok():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0), ((200.0, 50.0), 0.0)]
    assert pv.turn_angles_ok(path, alpha_max_rad=math.radians(30.0)) is True


def test_turn_angle_limit_violated():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0), ((150.0, 100.0), 0.0)]
    assert pv.turn_angles_ok(path, alpha_max_rad=math.radians(30.0)) is False


def test_straight_segment_lengths_positive_for_long_legs():
    path = [((0.0, 0.0), 0.0), ((100000.0, 0.0), 0.0),
            ((200000.0, 10000.0), 0.0), ((300000.0, 20000.0), 0.0)]
    ok, detail = pv.straight_segments_ok(path, R=8000.0, L0=4000.0, dss=23000.0)
    assert ok is True, detail


def test_arc_clear_when_no_obstacle():
    path = [((0.0, 0.0), 0.0), ((100000.0, 0.0), 0.0), ((200000.0, 50000.0), 0.0)]
    assert pv.arcs_clear(path, R=8000.0, circle_obstacles=[], polygon_obstacles=[]) is True


def test_arc_blocked_by_obstacle_on_inside_of_turn():
    # Corner at (100000,0) turning left; inside of the turn is +y.
    path = [((0.0, 0.0), 0.0), ((100000.0, 0.0), 0.0), ((100000.0, 100000.0), 0.0)]
    blocking = ((100000.0 - 3000.0, 3000.0), 1500.0)
    assert pv.arcs_clear(path, R=8000.0, circle_obstacles=[blocking],
                         polygon_obstacles=[]) is False
