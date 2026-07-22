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


import core.kinodynamic_astar as astar
from batch_random_test import generate_random_scenario

# Mission lengths (km) with the pure-Euclid heuristic, re-measured 2026-07-22
# on the current tree (post valve/fan-rung work, TIME_BUDGET_S=None). The
# field may only match or shorten, +5 km slack for tie-break noise (same
# threshold as the 1000-seed A/B protocol).
BASELINE_KM = {4: 446.1, 79: 502.3, 92: 521.4, 123: 445.3, 155: 472.2,
               187: 438.7, 242: 480.1, 272: 457.7, 496: 479.4, 612: 442.6,
               964: 480.8}


@pytest.fixture
def no_time_budget(monkeypatch):
    monkeypatch.setattr(config, 'TIME_BUDGET_S', None)


def _mission_km(pre, res):
    pts = [pre['start_pos']] + [p for p, _h in res['path']] + [pre['goal_pos']]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:])) / 1000.0


@pytest.mark.parametrize('seed', sorted(BASELINE_KM))
def test_oracle_witness_and_no_regression(seed, no_time_budget):
    pre = prep.prepare_scenario(generate_random_scenario(seed=seed))
    res = astar.plan_trajectory(pre, verbose=False)
    assert res['success']
    assert _mission_km(pre, res) <= BASELINE_KM[seed] + 5.0
    # admissibility witness: h at the path's first state must not exceed the
    # cost actually flown from there (body polyline + snap to goal waypoint)
    probe = astar.KinodynamicAstar(pre)          # fresh: builds field per gating
    first = astar.State(res['path'][0][0], res['path'][0][1])
    body = [p for p, _h in res['path']]
    flown = sum(math.dist(a, b) for a, b in zip(body, body[1:]))
    flown += math.dist(body[-1], probe.goal_state.waypoint)
    assert probe.heuristic(first, probe.goal_state) <= flown + 1.0


def test_field_cuts_expansions_on_occluded_seed(no_time_budget, monkeypatch):
    """Seed 187: the real detour (+8.3% over straight-line) exceeds the
    field's stretch discount, so the Euclid-optimistic basin gets pruned.
    Measured 2026-07-22 with the 16-connected field: 265 vs 832 iterations
    (ratio 0.32); the 0.6 gate leaves margin for lattice noise. A synthetic
    single-wall map is NOT a substitute: Strategy A jumps straight to its
    hull vertices (46 iterations either way), so no basin ever floods.
    """
    pre = prep.prepare_scenario(generate_random_scenario(seed=187))
    res_field = astar.plan_trajectory(pre, verbose=False)

    class _Boom:
        def __init__(self, pre):
            raise RuntimeError("disabled")
    monkeypatch.setattr(astar, 'GoalDistanceField', _Boom)
    pre_e = prep.prepare_scenario(generate_random_scenario(seed=187))
    res_euclid = astar.plan_trajectory(pre_e, verbose=False)

    assert res_field['success'] and res_euclid['success']
    it_f = res_field['stats']['iterations']
    it_e = res_euclid['stats']['iterations']
    assert it_f < 0.6 * it_e, f"field {it_f} vs euclid {it_e}: expected >40% cut"
    # equal-quality guard on the same map
    assert abs(_mission_km(pre, res_field) - _mission_km(pre_e, res_euclid)) <= 5.0


def open_water_scenario():
    return {
        'start': (50000.0, 250000.0), 'start_heading': 0.0,
        'goal': (450000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
        'map_bounds': (500000.0, 500000.0),
    }


def test_gating_open_water_builds_no_field():
    planner = astar.KinodynamicAstar(prep.prepare_scenario(open_water_scenario()))
    assert planner._goal_field is None


def test_gating_occluded_goal_builds_field():
    # circle dead-center: every corner chord to the engage point is blocked
    planner = astar.KinodynamicAstar(prep.prepare_scenario(circle_scenario()))
    assert planner._goal_field is not None


def test_heuristic_is_max_of_euclid_and_field():
    planner = astar.KinodynamicAstar(prep.prepare_scenario(circle_scenario()))
    gs = planner.goal_state
    p = (150000.0, 250000.0)          # in the circle's shadow
    st = astar.State(p, 0.0)
    h = planner.heuristic(st, gs)
    assert h >= math.dist(p, gs.waypoint) - 1e-6
    assert h >= planner._goal_field.query(p) - 1e-6


def test_field_failure_falls_back_to_euclid(monkeypatch):
    class _Boom:
        def __init__(self, pre):
            raise RuntimeError("boom")
    monkeypatch.setattr(astar, 'GoalDistanceField', _Boom)
    pre = prep.prepare_scenario(circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    assert planner._goal_field is None
    res = astar.plan_trajectory(prep.prepare_scenario(circle_scenario()), verbose=False)
    assert res['success']


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
