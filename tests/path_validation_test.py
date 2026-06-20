import math
import core.path_validation as pv


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


def test_segment_ending_at_polygon_vertex_is_clear():
    # A waypoint that sits exactly ON a polygon vertex (the corners ARE valid
    # navigation targets, like circle tangent points). A segment that only TOUCHES
    # the polygon at its endpoint vertex must be reported clear, not a collision.
    square = [(60000.0, -20000.0), (60000.0, 20000.0),
              (90000.0, 20000.0), (90000.0, -20000.0)]
    vertex = (60000.0, 20000.0)            # a corner of the square
    a = (0.0, 0.0)                         # outside, segment ends at the vertex
    assert pv.segments_clear([(a, 0.0), (vertex, 0.0)],
                             circle_obstacles=[], polygon_obstacles=[square]) is True


def test_segment_through_polygon_interior_still_blocked():
    # The endpoint-touch leniency must NOT clear a segment that crosses the interior.
    square = [(60000.0, -20000.0), (60000.0, 20000.0),
              (90000.0, 20000.0), (90000.0, -20000.0)]
    left = (50000.0, 0.0)                  # outside, left of square
    right = (100000.0, 0.0)               # outside, right of square; segment cuts through
    assert pv.segments_clear([(left, 0.0), (right, 0.0)],
                             circle_obstacles=[], polygon_obstacles=[square]) is False


def test_path_is_valid_checks_arcs_against_raw_obstacles():
    # Real planner output around a polygon: the turn arc at a polygon CORNER is
    # designed to bulge into the inflation band (it must only clear the RAW
    # obstacle). path_is_valid must therefore validate arcs against the RAW
    # obstacles when they are supplied, not the inflated ones.
    import core.map_generator as mg
    import core.preprocessing as prep
    import core.kinodynamic_astar as astar
    import config

    scenario = mg.scenario9_island_archipelago()
    pre = prep.prepare_scenario(scenario)
    res = astar.plan_trajectory(pre, verbose=False)
    assert res['success']
    path = res['path']

    raw_circles, raw_polys = [], []
    for o in scenario['obstacles']:
        if o['type'] == 'circle':
            raw_circles.append((o['center'], o['radius']))
        else:
            raw_polys.append(o['polygon'])

    assert pv.path_is_valid(
        path, pre['circle_obstacles'], pre['polygon_obstacles'],
        pre['turn_radius'], pre['alpha_max_rad'], config.L0, config.DSS,
        raw_circle_obstacles=raw_circles, raw_polygon_obstacles=raw_polys) is True


def test_segment_along_polygon_edge_is_clear():
    # Boundary-following is allowed: a segment running ALONG a polygon edge (on the
    # boundary, not through the interior) is clear.
    square = [(60000.0, -20000.0), (60000.0, 20000.0),
              (90000.0, 20000.0), (90000.0, -20000.0)]
    a, b = (60000.0, -20000.0), (60000.0, 20000.0)   # the left edge
    assert pv.segments_clear([(a, 0.0), (b, 0.0)],
                             circle_obstacles=[], polygon_obstacles=[square]) is True
