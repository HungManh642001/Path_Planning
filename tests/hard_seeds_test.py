"""Integration gates on the seeds that exposed WRAP_STEP_M sensitivity.

Baseline = best planner-path distance of the two old configs
(results1_fail_v1: WRAP=10000, results1_fail_v2: WRAP=5000).
TIME_BUDGET_S is disabled so results are deterministic; wall time is
asserted separately against the 5 s budget.
"""
import math
import time

import pytest

import config
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.path_validation as pv
import render.trajectory as tr
from batch_random_test import generate_random_scenario

# (seed, distance ceiling in m). Ceilings: baseline * 1.05 for the two
# pathological seeds (huge headroom vs their bad runs: 762 km / 815 km),
# baseline * 1.10 for ordinary hard seeds.
CASES = [
    (125, 579700.0 * 1.05),   # scenario 126: v2 detoured to 762 km
    (981, 581200.0 * 1.05),   # scenario 982: v1 self-looped to 815 km
    (319, 681200.0 * 1.10),   # scenario 320
    (674, 531600.0 * 1.10),   # scenario 675
]


def _plan(seed):
    scn = generate_random_scenario(seed=seed)
    pre = prep.prepare_scenario(scn)
    t0 = time.perf_counter()
    result = astar.plan_trajectory(pre)
    elapsed = time.perf_counter() - t0
    return scn, pre, result, elapsed


@pytest.mark.parametrize("seed,max_dist", CASES)
def test_hard_seed_valid_fast_and_near_baseline(seed, max_dist, monkeypatch):
    monkeypatch.setattr(config, 'TIME_BUDGET_S', None)
    scn, pre, result, elapsed = _plan(seed)
    assert result['success'], f"seed {seed} failed to plan"
    assert elapsed < 5.0, f"seed {seed} took {elapsed:.2f}s"

    full = tr.build_full_path(result['path'], pre)
    raw_circles = [(o['center'], o['radius'])
                   for o in scn['obstacles'] if o['type'] == 'circle']
    raw_polys = [o['polygon'] for o in scn['obstacles'] if o['type'] == 'polygon']
    assert pv.path_is_valid(
        full, pre['circle_obstacles'], pre['polygon_obstacles'],
        config.R, config.ALPHA_MAX_RAD, config.L0, config.DSS,
        raw_circle_obstacles=raw_circles, raw_polygon_obstacles=raw_polys), \
        f"seed {seed}: oracle rejected the path"

    # Same basis as the recorded baselines: planner path, no O/T endpoints.
    path = result['path']
    dist = sum(math.dist(path[i][0], path[i + 1][0]) for i in range(len(path) - 1))
    assert dist <= max_dist, f"seed {seed}: {dist / 1000:.1f} km > ceiling"


def test_route_invariant_to_arc_waypoint_step(monkeypatch):
    """THE root-cause test: the searched route must not depend on the output
    discretisation step (it did depend on WRAP_STEP_M before)."""
    monkeypatch.setattr(config, 'TIME_BUDGET_S', None)
    routes, iterations = [], []
    for theta in (20.0, 30.0, 45.0):
        monkeypatch.setattr(config, 'ARC_WAYPOINT_STEP_DEG', theta)
        _scn, _pre, result, _elapsed = _plan(125)
        assert result['success']
        routes.append(result['planner'].raw_route)
        iterations.append(result['stats']['iterations'])
    assert routes[0] == routes[1] == routes[2]
    assert iterations[0] == iterations[1] == iterations[2]
