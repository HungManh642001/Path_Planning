import math

import core.map_generator as mg
import core.preprocessing as prep
import core.kinodynamic_astar as astar
from ml_planner.plan import plan_trajectory_focal, path_length


def _mission(pre, path):
    return math.dist(pre['start_pos'], path[0][0]) + path_length(path)


def test_guidance_flag_falls_back_when_unavailable(monkeypatch):
    # When no model is available, secondary='guidance' must degrade to the
    # hand-crafted secondary and behave exactly like secondary=None. Force the
    # unavailable case deterministically (a real guidance.onnx may be present in
    # ml_planner/models/ on the dev machine).
    import ml_planner.plan as plan_mod
    monkeypatch.setattr(plan_mod, 'make_guidance_secondary', lambda *a, **k: (None, False))
    scen = mg.scenario4_complex_maze()
    r_guided = plan_trajectory_focal(prep.prepare_scenario(scen), secondary='guidance')
    r_default = plan_trajectory_focal(prep.prepare_scenario(scen), secondary=None)
    assert r_guided['success'] == r_default['success']
    assert r_guided['success']
    pre = prep.prepare_scenario(scen)
    assert abs(_mission(pre, r_guided['path']) - _mission(pre, r_default['path'])) < 1e-6


def test_synthetic_guidance_secondary_keeps_bound():
    # A guidance-shaped callable (distance-to-goal) as the focal secondary must
    # still respect the epsilon=5% bound vs the base optimal.
    scen = mg.scenario12_perimeter_dynamic_obstacles()
    pre = prep.prepare_scenario(scen)
    base = astar.plan_trajectory(pre, verbose=False)
    assert base['success']
    base_mission = _mission(pre, base['path'])

    goal = pre['goal_state']['waypoint']
    secondary = lambda st: math.hypot(st.waypoint[0] - goal[0], st.waypoint[1] - goal[1])
    r = plan_trajectory_focal(prep.prepare_scenario(scen), focal_eps=0.05, secondary=secondary)
    assert r['success']
    assert _mission(pre, r['path']) <= 1.05 * base_mission + 1e-6
