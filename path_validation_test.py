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
