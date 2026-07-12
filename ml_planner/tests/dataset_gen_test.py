import math

import core.map_generator as mg
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.spatial_utils as su
from ml_planner.dataset_gen import _RecordingAstar, backward_costs, _no_budget


def _mission(pre, path):
    body = sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for (a, _), (b, _) in zip(path, path[1:]))
    return math.dist(pre['start_pos'], path[0][0]) + body


def test_backward_costs_reconstruct_base_optimum():
    scen = mg.scenario2_single_obstacle()
    pre = prep.prepare_scenario(scen)
    base = astar.plan_trajectory(pre, verbose=False)
    assert base['success']
    base_mission = _mission(pre, base['path'])

    with _no_budget():
        planner = _RecordingAstar(prep.prepare_scenario(scen))
        path = planner.search()
    assert path is not None
    goal_key = su.state_to_tuple(*planner.raw_route[-1])
    costs = backward_costs(planner.edges, goal_key)
    assert costs[goal_key] == 0.0
    assert len(costs) > 1

    # Optimal mission cost = min over seeded start corners of
    # (dist(O, corner) == corner.g_cost) + cost_to_go(corner).
    best = min(
        c.g_cost + costs.get(su.state_to_tuple(c.waypoint, c.heading), float('inf'))
        for c in planner.start_corners
    )
    assert abs(best - base_mission) <= 0.02 * base_mission
