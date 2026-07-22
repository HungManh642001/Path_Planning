"""Tests for the admissible goal-distance-field heuristic."""
import math

import pytest

import config
import core.preprocessing as prep
from core.heuristic_field import GoalDistanceField

CENTER = (250000.0, 250000.0)
RAW_R = 30000.0


def circle_scenario():
    """One raw circle dead-center between a west start and an east goal."""
    return {
        'start': (50000.0, 250000.0), 'start_heading': 0.0,
        'goal': (450000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [],
        'dynamic_obstacles': [(CENTER, RAW_R)],
        'obstacles': [{'type': 'circle', 'center': CENTER, 'radius': RAW_R}],
        'map_bounds': (500000.0, 500000.0),
    }


def shortest_around_circle(p, q, c, r):
    """Exact continuous shortest distance p->q avoiding one disc (c, r)."""
    d1, d2 = math.dist(p, c), math.dist(q, c)
    px, py = p
    qx, qy = q
    cx, cy = c
    dx, dy = qx - px, qy - py
    t = max(0.0, min(1.0, ((cx - px) * dx + (cy - py) * dy) / (dx * dx + dy * dy)))
    if math.dist((px + t * dx, py + t * dy), c) >= r:
        return math.dist(p, q)
    tang = math.sqrt(max(d1 * d1 - r * r, 0.0)) + math.sqrt(max(d2 * d2 - r * r, 0.0))
    ang = (math.acos(max(-1.0, min(1.0, ((px - cx) * (qx - cx) + (py - cy) * (qy - cy)) / (d1 * d2))))
           - math.acos(max(-1.0, min(1.0, r / d1)))
           - math.acos(max(-1.0, min(1.0, r / d2))))
    return tang + r * max(ang, 0.0)


def test_circle_field_is_admissible_lower_bound():
    pre = prep.prepare_scenario(circle_scenario())
    field = GoalDistanceField(pre)
    goal = pre['goal_state']['waypoint']
    (c, r_inf), = pre['circle_obstacles']
    import random
    rng = random.Random(7)
    checked = 0
    for _ in range(300):
        p = (rng.uniform(5000, 495000), rng.uniform(5000, 495000))
        if math.dist(p, c) < r_inf + 1.0:
            continue                    # inside the inflated disc: no admissibility claim
        true_d = shortest_around_circle(p, goal, c, r_inf)
        assert field.query(p) <= true_d + 1.0, f"h violates lower bound at {p}"
        checked += 1
    assert checked > 200


def test_query_outside_grid_returns_neg_inf():
    pre = prep.prepare_scenario(circle_scenario())
    field = GoalDistanceField(pre)
    assert field.query((-1e6, -1e6)) == -math.inf
