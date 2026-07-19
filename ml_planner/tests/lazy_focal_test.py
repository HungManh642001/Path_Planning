import math

import numpy as np
import pytest

import core.kinodynamic_astar as astar
import core.map_generator as mg
import core.preprocessing as prep
from ml_planner.focal_astar import FocalKinodynamicAstar
from ml_planner.lazy_focal import LazyFocalKinodynamicAstar
from ml_planner import raster
from ml_planner.corridor import Corridor
from ml_planner.plan import plan_trajectory_lazy


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


def _corridor_with_mask(pre, fill, grid_res=32):
    """Helper to build a Corridor mask with start/goal cells True, rest fill."""
    aff = raster.compute_crop(pre, grid_res)
    mask = np.full((grid_res, grid_res), fill, dtype=bool)
    for pt in (pre['start_pos'], pre['goal_pos']):
        gx, gy = aff.world_to_grid(*pt)
        # Use truncation (int) not rounding, matching Corridor.contains convention
        # that cells are [i, i+1) half-open intervals.
        ix, iy = int(gx), int(gy)
        if 0 <= iy < grid_res and 0 <= ix < grid_res:
            mask[iy, ix] = True
    return Corridor(mask, aff)


@pytest.mark.parametrize("scenario_func", [
    mg.scenario4_complex_maze,
    mg.scenario12_perimeter_dynamic_obstacles,
])
def test_money_all_false_corridor_still_bound(scenario_func):
    # THE money test: a maximally wrong corridor (nothing admitted except the
    # start/goal cells) may only cost time — the admit-all fallback must
    # still produce a valid path within the 1.05x bound.
    base = _base_cost(scenario_func)
    pre = prep.prepare_scenario(scenario_func())
    cor = _corridor_with_mask(pre, fill=False)
    res = plan_trajectory_lazy(pre, corridor=cor, focal_eps=0.05)
    assert res['success']
    assert _mission_cost(pre, res['path']) <= 1.05 * base + 1e-6


def test_all_true_corridor_equals_pure_lazy():
    # An all-True corridor admits everything -> identical to corridor=None.
    scen = mg.scenario4_complex_maze
    pre_a = prep.prepare_scenario(scen())
    cor = _corridor_with_mask(pre_a, fill=True)
    res_a = plan_trajectory_lazy(pre_a, corridor=cor, focal_eps=0.05)
    pre_b = prep.prepare_scenario(scen())
    res_b = plan_trajectory_lazy(pre_b, corridor=None, focal_eps=0.05)
    assert res_a['success'] and res_b['success']
    assert res_a['stats']['iterations'] == res_b['stats']['iterations']
    assert abs(_mission_cost(pre_a, res_a['path'])
               - _mission_cost(pre_b, res_b['path'])) < 1e-6


def test_plan_trajectory_lazy_stats_contract():
    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    res = plan_trajectory_lazy(pre, focal_eps=0.05)
    assert res['success']
    assert res['stats']['collision_checks'] > 0
    assert res['path'] is not None


def test_invalidated_states_die_no_revalidation_churn():
    # Regression (LC-T6 benchmark collapse, seeds 6001/6002/7005...): a state
    # whose deferred edge fails validation must be DEAD, not merely dropped
    # from g_scores — the liveness predicate `g_cost <= g_scores.get(st, inf)`
    # is vacuously true after the deletion, so the corpse stayed "live" in
    # OPEN, was re-admitted to FOCAL on every refill and re-paid the real
    # collision check on every pop (observed: 98k rejected re-validations in
    # 321 iterations, wall-clock budget exhausted -> spurious no-path on maps
    # the eager planner solves in ~150 iterations).
    from batch_random_test import generate_random_scenario
    scen = generate_random_scenario(7005)
    pre = prep.prepare_scenario(scen)
    lazy = LazyFocalKinodynamicAstar(pre, focal_eps=0.05)

    # id(state) -> [state, attempts]; the state ref pins the object so a
    # garbage-collected corpse can't recycle its id into a false double-count.
    validations = {}
    orig = lazy._validate_on_pop

    def counting_vop(state):
        if not getattr(state, 'edge_validated', True):
            entry = validations.setdefault(id(state), [state, 0])
            entry[1] += 1
        return orig(state)

    lazy._validate_on_pop = counting_vop
    path = lazy.search()
    assert path is not None, "lazy must solve the map the eager planner solves"
    assert validations, "map must actually exercise deferred validation"
    assert max(n for _, n in validations.values()) == 1, (
        "each deferred edge must pay its real collision check at most once")
