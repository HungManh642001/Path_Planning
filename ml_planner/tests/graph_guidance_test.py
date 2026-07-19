import math

import numpy as np
import pytest

import core.preprocessing as prep
import ml_planner.config as mlcfg
from ml_planner.graph import build_graph
from ml_planner.graph_guidance import (GraphGuidance, make_graph_secondary,
                                       mpnn_forward)


def _scenario():
    circles = [((250_000.0, 250_000.0), 20_000.0)]
    return {'start': (20_000.0, 250_000.0), 'start_heading': 0.0,
            'goal': (480_000.0, 250_000.0), 'goal_heading': 0.0,
            'islands': [], 'dynamic_obstacles': list(circles),
            'obstacles': [{'type': 'circle', 'center': c, 'radius': r}
                          for c, r in circles]}


def _random_weights(path, hidden=8, rounds=2, node_dim=7, edge_dim=2, seed=0):
    rng = np.random.default_rng(seed)
    def w(*shape):
        return rng.normal(scale=0.3, size=shape).astype(np.float32)
    arrays = {
        'enc.weight': w(hidden, node_dim), 'enc.bias': w(hidden),
        'msg.0.weight': w(hidden, 2 * hidden + edge_dim), 'msg.0.bias': w(hidden),
        'msg.2.weight': w(hidden, hidden), 'msg.2.bias': w(hidden),
        'upd.weight_ih': w(3 * hidden, hidden), 'upd.weight_hh': w(3 * hidden, hidden),
        'upd.bias_ih': w(3 * hidden), 'upd.bias_hh': w(3 * hidden),
        'dec.0.weight': w(hidden, hidden), 'dec.0.bias': w(hidden),
        'dec.2.weight': w(1, hidden), 'dec.2.bias': w(1),
    }
    np.savez(path, __meta__=np.asarray([hidden, rounds, node_dim, edge_dim],
                                       dtype=np.int64), **arrays)
    return arrays


def test_unavailable_without_model(tmp_path):
    gg = GraphGuidance(model_path=str(tmp_path / "missing.npz"))
    assert gg.available is False
    cb, ok = make_graph_secondary(prep.prepare_scenario(_scenario()),
                                  model_path=str(tmp_path / "missing.npz"))
    assert cb is None and ok is False


def test_values_lower_bounded_by_euclid(tmp_path):
    path = str(tmp_path / "w.npz")
    _random_weights(path)
    gg = GraphGuidance(model_path=path)
    assert gg.available
    pre = prep.prepare_scenario(_scenario())
    gg.build_field(pre)
    goal = np.asarray(pre['goal_pos'])
    euclid = np.hypot(gg.graph.nodes[:, 0] - goal[0], gg.graph.nodes[:, 1] - goal[1])
    assert np.all(gg.values >= euclid - 1e-3)


def test_lookup_blends_distance_and_value(tmp_path):
    path = str(tmp_path / "w.npz")
    _random_weights(path)
    gg = GraphGuidance(model_path=path)
    pre = prep.prepare_scenario(_scenario())
    gg.build_field(pre)
    wp = (100_000.0, 260_000.0)
    v = gg.lookup(wp)
    d, idx = gg.graph.kdtree.query(wp, k=min(3, len(gg.graph.nodes)))
    expect = float(np.min(np.atleast_1d(d) + gg.values[np.atleast_1d(idx)]))
    assert abs(v - expect) < 1e-9


def test_secondary_callable_contract(tmp_path):
    path = str(tmp_path / "w.npz")
    _random_weights(path)
    pre = prep.prepare_scenario(_scenario())
    cb, ok = make_graph_secondary(pre, model_path=path)
    assert ok is True

    class _FakeState:
        waypoint = (60_000.0, 250_000.0)
    assert cb(_FakeState()) > 0.0


def test_numpy_matches_torch_golden():
    torch = pytest.importorskip('torch')
    from ml_planner.train.train_graph import MPNN
    torch.manual_seed(0)
    model = MPNN(node_dim=7, edge_dim=2, hidden=8, rounds=2).double()
    rng = np.random.default_rng(1)
    m, e = 6, 7
    node_feat = rng.normal(size=(m, 7))
    edges = rng.integers(0, m, size=(e, 2)).astype(np.int32)
    edge_feat = rng.random(size=(e, 2))
    ei = torch.tensor(np.concatenate([edges.T.astype(np.int64),
                                      edges.T[::-1].astype(np.int64)], axis=1))
    ea = torch.tensor(np.concatenate([edge_feat, edge_feat], axis=0))
    with torch.no_grad():
        r_torch = model(torch.tensor(node_feat), ei, ea).numpy()
    weights = {k: v.detach().numpy() for k, v in model.state_dict().items()}
    r_np = mpnn_forward(weights, node_feat, edges, edge_feat, rounds=2)
    assert np.allclose(r_np, r_torch, atol=1e-10)
