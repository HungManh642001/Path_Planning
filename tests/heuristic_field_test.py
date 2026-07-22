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


def corridor_scenario():
    """L-shaped safezone; no map_bounds (production-style unbounded world).
    True paths must follow the L: ~490 km vs ~346 km Euclid."""
    r1 = [(0.0, 0.0), (300000.0, 0.0), (300000.0, 60000.0), (0.0, 60000.0)]
    r2 = [(240000.0, 0.0), (300000.0, 0.0), (300000.0, 300000.0), (240000.0, 300000.0)]
    return {
        'start': (30000.0, 30000.0), 'start_heading': 0.0,
        'goal': (270000.0, 280000.0), 'goal_heading': math.pi / 2,
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
        'safezones': [r1, r2],
    }


def test_corridor_field_prices_the_safezone_detour():
    pre = prep.prepare_scenario(corridor_scenario())
    field = GoalDistanceField(pre)
    goal = pre['goal_state']['waypoint']
    p = (30000.0, 30000.0)
    euclid = math.dist(p, goal)
    q = field.query(p)
    # adds value: the corridor forces a detour Euclid cannot see. Observed
    # q = 387.3 km vs euclid 324.9 km (ratio 1.192) vs continuous truth
    # ~403.5 km via the inner corner (240k, 60k) — 96% tight after the
    # stretch discount and 2-cell slack.
    assert q >= 1.15 * euclid
    # stays admissible: the exact shortest L-path through the inner corner
    l_true = (math.dist(p, (240000.0, 60000.0))
              + math.dist((240000.0, 60000.0), goal))
    assert q <= l_true + 1.0


def test_permissive_world_border_seeding_is_sound():
    """No safezone, no map_bounds: a path may leave the gridded area, so the
    field must never exceed Euclid-through-the-border alternatives."""
    scn = circle_scenario()
    del scn['map_bounds']
    pre = prep.prepare_scenario(scn)
    field = GoalDistanceField(pre)
    goal = pre['goal_state']['waypoint']
    (c, r_inf), = pre['circle_obstacles']
    import random
    rng = random.Random(11)
    for _ in range(200):
        p = (rng.uniform(-100000, 600000), rng.uniform(-100000, 600000))
        if math.dist(p, c) < r_inf + 1.0:
            continue
        true_d = shortest_around_circle(p, goal, c, r_inf)
        assert field.query(p) <= true_d + 1.0
