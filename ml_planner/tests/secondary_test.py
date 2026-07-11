import math
from ml_planner.secondary import handcrafted_secondary


def test_clear_line_equals_euclid():
    # No obstacles between waypoint and goal -> pure Euclid distance.
    d = handcrafted_secondary((0.0, 0.0), (100.0, 0.0), [])
    assert abs(d - 100.0) < 1e-9


def test_blocking_circle_inflates_estimate():
    # A circle straddling the straight line adds a detour penalty.
    obstacles = [((50.0, 0.0), 10.0)]
    blocked = handcrafted_secondary((0.0, 0.0), (100.0, 0.0), obstacles)
    clear = handcrafted_secondary((0.0, 0.0), (100.0, 0.0), [])
    assert blocked > clear


def test_offline_circle_does_not_penalize():
    # A circle far from the line must not change the estimate.
    obstacles = [((50.0, 10_000.0), 10.0)]
    d = handcrafted_secondary((0.0, 0.0), (100.0, 0.0), obstacles)
    assert abs(d - 100.0) < 1e-9
