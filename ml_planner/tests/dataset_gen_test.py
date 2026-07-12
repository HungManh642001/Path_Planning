import os
import math

import numpy as np

import core.map_generator as mg
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.spatial_utils as su
from ml_planner.dataset_gen import _RecordingAstar, backward_costs, _no_budget, rasterize_labels, generate_sample, export_dataset


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


def test_generate_sample_shapes_and_mask():
    scen = mg.scenario2_single_obstacle()
    sample = generate_sample(scen, grid_res=64)
    assert sample is not None
    assert sample['channels'].shape == (4, 64, 64)
    assert sample['label'].shape == (64, 64)
    assert sample['mask'].shape == (64, 64)
    assert sample['mask'].sum() > 0                      # some cells labeled
    # Labeled cells carry finite non-negative cost-to-go.
    labeled = sample['label'][sample['mask'] > 0]
    assert np.all(np.isfinite(labeled)) and np.all(labeled >= 0.0)


def test_export_dataset_roundtrip(tmp_path):
    out = os.path.join(tmp_path, "ds.npz")
    n = export_dataset([mg.scenario1_open_ocean(), mg.scenario2_single_obstacle()],
                       out, grid_res=64)
    assert n >= 1
    data = np.load(out)
    assert data['channels'].shape == (n, 4, 64, 64)
    assert data['label'].shape == (n, 64, 64)
    assert data['mask'].shape == (n, 64, 64)
