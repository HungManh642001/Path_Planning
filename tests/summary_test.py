import math
import gui.summary as gs


def _fake_preprocessed():
    return {'turn_radius': 8000.0, 'alpha_max_rad': math.radians(90.0),
            'circle_obstacles': [], 'polygon_obstacles': []}


def test_summary_of_straight_two_point_path():
    path = [((0.0, 0.0), 0.0), ((100000.0, 0.0), 0.0)]
    result = {'success': True, 'path': path,
              'stats': {'iterations': 7}}
    s = gs.compute_summary(result, _fake_preprocessed(), raw_circles=[], raw_polys=[],
                           render_mode='straight', runtime_s=0.012)
    assert s['success'] is True
    assert abs(s['distance_km'] - 100.0) < 1e-6
    assert s['num_waypoints'] == 2
    assert s['num_turns'] == 0
    assert s['runtime_ms'] == 12.0
    assert s['iterations'] == 7


def test_summary_counts_turn_and_reports_max_angle():
    path = [((0.0, 0.0), 0.0), ((100000.0, 0.0), 0.0), ((100000.0, 100000.0), math.pi / 2)]
    result = {'success': True, 'path': path, 'stats': {'iterations': 3}}
    s = gs.compute_summary(result, _fake_preprocessed(), raw_circles=[], raw_polys=[],
                           render_mode='straight', runtime_s=0.0)
    assert s['num_turns'] == 1
    assert abs(s['max_turn_deg'] - 90.0) < 1e-6


def test_summary_handles_failure():
    result = {'success': False, 'path': None, 'stats': {'iterations': 50000}}
    s = gs.compute_summary(result, _fake_preprocessed(), raw_circles=[], raw_polys=[],
                           render_mode='dubins', runtime_s=0.9)
    assert s['success'] is False
    assert s['distance_km'] == 0.0
    assert s['num_waypoints'] == 0
    assert s['valid'] is False
