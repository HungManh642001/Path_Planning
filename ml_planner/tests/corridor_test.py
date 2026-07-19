import numpy as np

import core.preprocessing as prep
from ml_planner.corridor import Corridor, build_corridor
from ml_planner.graph_guidance import GraphGuidance
from ml_planner import raster


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


def test_contains_membership_and_out_of_crop():
    pre = prep.prepare_scenario(_scenario())
    aff = raster.compute_crop(pre, 16)
    mask = np.zeros((16, 16), dtype=bool)
    mask[3, 5] = True
    cor = Corridor(mask, aff)
    x, y = aff.grid_to_world(5.5, 3.5)          # center of cell [iy=3, ix=5]
    assert cor.contains(x, y) is True
    x2, y2 = aff.grid_to_world(1.5, 1.5)
    assert cor.contains(x2, y2) is False
    assert cor.contains(-1e9, -1e9) is False    # far outside crop


def test_build_corridor_none_without_model(tmp_path):
    gg = GraphGuidance(model_path=str(tmp_path / "missing.npz"))
    pre = prep.prepare_scenario(_scenario())
    assert build_corridor(pre, gg) is None
    assert build_corridor(pre, None) is None


def test_build_corridor_start_goal_always_inside(tmp_path):
    path = str(tmp_path / "w.npz")
    _random_weights(path)
    gg = GraphGuidance(model_path=path)
    assert gg.available
    pre = prep.prepare_scenario(_scenario())
    cor = build_corridor(pre, gg, delta=0.0)     # tightest corridor
    assert cor is not None
    assert cor.contains(*pre['start_pos']) is True
    assert cor.contains(*pre['goal_pos']) is True


def test_build_corridor_deterministic(tmp_path):
    path = str(tmp_path / "w.npz")
    _random_weights(path)
    gg = GraphGuidance(model_path=path)
    pre = prep.prepare_scenario(_scenario())
    c1 = build_corridor(pre, gg)
    c2 = build_corridor(pre, gg)
    assert np.array_equal(c1.mask, c2.mask)
