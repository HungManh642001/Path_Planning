import math

import core.map_generator as mg
import core.preprocessing as prep
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
