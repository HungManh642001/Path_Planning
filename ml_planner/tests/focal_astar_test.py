import math

import pytest

import core.kinodynamic_astar as astar
import core.map_generator as mg
import core.preprocessing as prep
from core.kinodynamic_astar import State
from ml_planner.focal_astar import FocalKinodynamicAstar


def _prep(scenario_func):
    return prep.prepare_scenario(scenario_func())


def test_instanti_and_secondary_default():
    pre = _prep(mg.scenario2_single_obstacle)
    planner = FocalKinodynamicAstar(pre, focal_eps=0.05)
    # secondary_h falls back to hand-crafted: finite, >= straight-line to goal.
    start = planner.start_state
    val = planner.secondary_h(start)
    gwp = planner.goal_state.waypoint
    euclid = math.hypot(gwp[0] - start.waypoint[0], gwp[1] - start.waypoint[1])
    assert math.isfinite(val)
    assert val >= euclid - 1e-6


def test_custom_secondary_used():
    pre = _prep(mg.scenario1_open_ocean)
    planner = FocalKinodynamicAstar(pre, focal_eps=0.0, secondary=lambda st: 42.0)
    assert planner.secondary_h(planner.start_state) == 42.0


def _path_len(path):
    total = 0.0
    for (a, _), (b, _) in zip(path, path[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def _mission_cost(pre, path):
    # Corner-invariant total cost from O: include the O->first-corner takeoff
    # leg that the returned path (which starts at a seeded start corner) omits.
    # Without it, equally-optimal runs that settle on different TIED start
    # corners report different body lengths despite identical mission cost.
    return math.dist(pre['start_pos'], path[0][0]) + _path_len(path)


def _optimal_cost(scenario_func):
    pre = prep.prepare_scenario(scenario_func())
    res = astar.plan_trajectory(pre, verbose=False)
    assert res['success'], "base planner must solve the fixture"
    return _mission_cost(pre, res['path'])


def test_eps_zero_matches_optimal_cost():
    # focal_eps=0 with a Euclid secondary must reproduce the optimal cost.
    scen = mg.scenario2_single_obstacle
    opt = _optimal_cost(scen)
    pre = prep.prepare_scenario(scen())
    gwp = pre['goal_state']['waypoint']
    planner = FocalKinodynamicAstar(
        pre, focal_eps=0.0,
        secondary=lambda st: math.hypot(st.waypoint[0] - gwp[0], st.waypoint[1] - gwp[1]),
    )
    path = planner.search()
    assert path is not None
    assert abs(_mission_cost(pre, path) - opt) < 1.0  # meters; both optimal


def test_focal_respects_epsilon_bound():
    # focal_eps=0.05 with the hand-crafted secondary: cost <= 1.05 * optimal.
    scen = mg.scenario2_single_obstacle
    opt = _optimal_cost(scen)
    pre = prep.prepare_scenario(scen())
    planner = FocalKinodynamicAstar(pre, focal_eps=0.05)
    path = planner.search()
    assert path is not None
    assert _mission_cost(pre, path) <= 1.05 * opt + 1e-6


@pytest.mark.parametrize("scenario_func", [
    mg.scenario4_complex_maze,
    mg.scenario12_perimeter_dynamic_obstacles,
    mg.scenario13_dense_island_field,
    mg.scenario16_extreme_complexity,
])
def test_epsilon_bound_holds_on_obstacle_scenarios(scenario_func):
    # The real guarantee (user's requirement): focal must not be worse than the
    # CURRENT algorithm (base) by more than epsilon = 5%. Verified on obstacle
    # maps that actually force detours -- scenario2 alone is unobstructed and
    # gives false confidence. Reproducing the base EXACTLY at eps=0 is NOT
    # required: both base and focal are non-reopening lattice searches, so a
    # sub-0.1% tie-break difference is expected and accepted (documented
    # decision). The meaningful contract is the 1.05x ceiling.
    pre = prep.prepare_scenario(scenario_func())
    base = astar.plan_trajectory(pre, verbose=False)
    assert base['success']
    base_cost = _mission_cost(pre, base['path'])

    pre2 = prep.prepare_scenario(scenario_func())
    planner = FocalKinodynamicAstar(pre2, focal_eps=0.05)
    path = planner.search()
    assert path is not None
    path = planner.smooth_path(path)
    assert _mission_cost(pre2, path) <= 1.05 * base_cost + 1e-6


def test_goal_reached_accepts_aligned_rejects_misaligned():
    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    planner = FocalKinodynamicAstar(pre, focal_eps=0.05)
    gwp = planner.goal_state.waypoint
    gh = planner.goal_state.heading
    # Aligned arrival exactly at the goal waypoint -> accepted (returns a path).
    aligned = State(gwp, gh)
    aligned.parent = planner.start_state
    assert planner._goal_reached(aligned) is not None
    # Misaligned by more than alpha_max -> rejected.
    misaligned = State(gwp, gh + planner.alpha_max_rad + 0.5)
    misaligned.parent = planner.start_state
    assert planner._goal_reached(misaligned) is None
    # Far from the goal -> rejected regardless of heading.
    far = State((gwp[0] + 1e6, gwp[1]), gh)
    far.parent = planner.start_state
    assert planner._goal_reached(far) is None
