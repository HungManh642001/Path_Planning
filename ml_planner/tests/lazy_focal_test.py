import math

import pytest

import core.kinodynamic_astar as astar
import core.map_generator as mg
import core.preprocessing as prep
from ml_planner.focal_astar import FocalKinodynamicAstar
from ml_planner.lazy_focal import LazyFocalKinodynamicAstar


def _path_len(path):
    total = 0.0
    for (a, _), (b, _) in zip(path, path[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def _mission_cost(pre, path):
    return math.dist(pre['start_pos'], path[0][0]) + _path_len(path)


def _base_cost(scenario_func):
    pre = prep.prepare_scenario(scenario_func())
    res = astar.plan_trajectory(pre, verbose=False)
    assert res['success']
    return _mission_cost(pre, res['path'])


def test_lazy_equals_eager_on_obstacle_free_map():
    # No obstacles -> every deferred edge is valid -> deferral changes
    # nothing: identical path, identical expansion count.
    scen = mg.scenario1_open_ocean
    pre_e = prep.prepare_scenario(scen())
    eager = FocalKinodynamicAstar(pre_e, focal_eps=0.05)
    path_e = eager.search()
    pre_l = prep.prepare_scenario(scen())
    lazy = LazyFocalKinodynamicAstar(pre_l, focal_eps=0.05)
    path_l = lazy.search()
    assert path_e is not None and path_l is not None
    assert lazy.iteration_count == eager.iteration_count
    assert abs(_mission_cost(pre_l, path_l) - _mission_cost(pre_e, path_e)) < 1e-6


@pytest.mark.parametrize("scenario_func", [
    mg.scenario4_complex_maze,
    mg.scenario12_perimeter_dynamic_obstacles,
    mg.scenario13_dense_island_field,
    mg.scenario16_extreme_complexity,
])
def test_lazy_epsilon_bound_holds(scenario_func):
    # The non-negotiable contract: pure-lazy (corridor=None) stays within
    # 1.05x the base planner on obstacle maps.
    base = _base_cost(scenario_func)
    pre = prep.prepare_scenario(scenario_func())
    lazy = LazyFocalKinodynamicAstar(pre, focal_eps=0.05)
    path = lazy.search()
    assert path is not None
    path = lazy.smooth_path(path)
    assert _mission_cost(pre, path) <= 1.05 * base + 1e-6


def test_lazy_pays_fewer_real_checks_than_eager():
    scen = mg.scenario4_complex_maze
    pre_e = prep.prepare_scenario(scen())
    eager = FocalKinodynamicAstar(pre_e, focal_eps=0.05)
    assert eager.search() is not None
    pre_l = prep.prepare_scenario(scen())
    lazy = LazyFocalKinodynamicAstar(pre_l, focal_eps=0.05)
    assert lazy.search() is not None
    assert lazy.collision_checks < eager.collision_checks


def test_goal_chords_never_deferred():
    # Every accepted goal arrival must ride an already-validated edge; the
    # trap must exclude p2 == goal waypoint (also keeps the valve LOS test
    # honest at the same call site).
    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    lazy = LazyFocalKinodynamicAstar(pre, focal_eps=0.05)
    path = lazy.search()
    assert path is not None
    # The state whose waypoint is nearest the goal must be validated.
    import config
    gwp = lazy.goal_state.waypoint
    # walk the returned path's final state via g_scores objects: the search
    # only returns via _goal_reached(current) where current popped validated;
    # assert no un-validated state sits within GOAL_THRESHOLD of the goal.
    for st in lazy.g_scores:
        d = math.hypot(st.waypoint[0] - gwp[0], st.waypoint[1] - gwp[1])
        if d < config.GOAL_THRESHOLD:
            assert getattr(st, 'edge_validated', True) is True


def test_no_path_map_terminates_with_none():
    # Goal sealed inside a ring of overlapping circles: base fails, lazy must
    # also conclude no-path (finite), not hang on optimistic frontier.
    goal = (200_000.0, 0.0)
    ring = []
    for k in range(8):
        ang = 2.0 * math.pi * k / 8
        ring.append(((goal[0] + 40_000.0 * math.cos(ang),
                      goal[1] + 40_000.0 * math.sin(ang)), 30_000.0))
    scen = {'start': (0.0, 0.0), 'start_heading': 0.0,
            'goal': goal, 'goal_heading': 0.0,
            'islands': [], 'dynamic_obstacles': ring,
            'obstacles': [{'type': 'circle', 'center': c, 'radius': r}
                          for c, r in ring]}
    pre = prep.prepare_scenario(scen)
    base = astar.plan_trajectory(pre, verbose=False)
    assert not base['success']
    pre2 = prep.prepare_scenario(scen)
    lazy = LazyFocalKinodynamicAstar(pre2, focal_eps=0.05)
    assert lazy.search() is None
