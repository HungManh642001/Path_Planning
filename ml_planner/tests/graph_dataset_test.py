import math

import numpy as np

import core.preprocessing as prep
from ml_planner.graph import build_graph
from ml_planner.graph_dataset import (SNAP_RADIUS_M, snap_labels,
                                      generate_graph_sample, write_shard,
                                      load_shards)


def _empty_scenario():
    return {'start': (0.0, 0.0), 'start_heading': 0.0,
            'goal': (100_000.0, 0.0), 'goal_heading': 0.0,
            'islands': [], 'dynamic_obstacles': [], 'obstacles': []}


def test_snap_labels_nearest_and_masking():
    g = build_graph(prep.prepare_scenario(_empty_scenario()))   # 2 nodes
    near_goal = (100_000.0 - 500.0, 0.0)        # 500 m from the goal node
    far_away = (50_000.0, 90_000.0)             # > SNAP_RADIUS_M from both
    key2wp = {('a',): near_goal, ('b',): far_away}
    costs = {('a',): 10_000.0, ('b',): 1.0}
    label, mask = snap_labels(g, costs, key2wp)
    assert mask[g.goal_idx] == 1.0
    assert abs(label[g.goal_idx] - (10_000.0 + 500.0)) < 1e-6
    assert mask[g.start_idx] == 0.0             # nothing within snap radius


def test_snap_labels_takes_min_over_candidates():
    g = build_graph(prep.prepare_scenario(_empty_scenario()))
    key2wp = {('a',): (100_000.0 - 500.0, 0.0), ('b',): (100_000.0 - 400.0, 0.0)}
    costs = {('a',): 10_000.0, ('b',): 20_000.0}
    label, mask = snap_labels(g, costs, key2wp)
    assert abs(label[g.goal_idx] - 10_500.0) < 1e-6     # min, not last


def test_generate_graph_sample_on_trivial_map():
    sample = generate_graph_sample(_empty_scenario())
    if sample is None:          # oracle may legitimately fail on a degenerate map
        return
    m = sample['node_feat'].shape[0]
    assert sample['label'].shape == (m,) and sample['mask'].shape == (m,)
    assert sample['edges'].dtype == np.int32
    assert float(sample['scale']) > 0


def test_shard_roundtrip(tmp_path):
    rng = np.random.default_rng(0)
    samples = []
    for m, e in ((5, 4), (3, 2)):
        samples.append(dict(
            node_feat=rng.normal(size=(m, 7)).astype(np.float32),
            edges=rng.integers(0, m, size=(e, 2)).astype(np.int32),
            edge_feat=rng.normal(size=(e, 2)).astype(np.float32),
            label=rng.normal(size=m).astype(np.float32),
            mask=(rng.random(m) > 0.5).astype(np.float32),
            scale=np.float64(123.0)))
    path = str(tmp_path / "graph_dataset_000.npz")
    write_shard(path, samples)
    loaded = load_shards(str(tmp_path))
    assert len(loaded) == 2
    for orig, back in zip(samples, loaded):
        for k in ('node_feat', 'edges', 'edge_feat', 'label', 'mask'):
            assert np.array_equal(orig[k], back[k]), k
        assert float(orig['scale']) == float(back['scale'])
