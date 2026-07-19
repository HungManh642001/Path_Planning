import numpy as np
import pytest

torch = pytest.importorskip('torch')

from ml_planner.train.train_graph import MPNN, train, save_weights


def _tiny_graphs(n=6, m=5, e=6, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        out.append(dict(
            node_feat=rng.normal(size=(m, 7)).astype(np.float32),
            edges=rng.integers(0, m, size=(e, 2)).astype(np.int32),
            edge_feat=rng.random(size=(e, 2)).astype(np.float32),
            label=rng.random(size=m).astype(np.float32) * 1000.0,
            mask=np.ones(m, dtype=np.float32),
            scale=np.float64(1000.0)))
    return out


def test_mpnn_output_shape_and_nonnegative():
    model = MPNN(hidden=8, rounds=2)
    g = _tiny_graphs(1)[0]
    x = torch.tensor(g['node_feat'])
    ei = torch.tensor(np.concatenate([g['edges'].T, g['edges'].T[::-1]], axis=1),
                      dtype=torch.long)
    ea = torch.tensor(np.concatenate([g['edge_feat'], g['edge_feat']], axis=0))
    r = model(x, ei, ea)
    assert r.shape == (5,)
    assert bool((r >= 0).all())


def test_training_reduces_loss_and_saves(tmp_path):
    graphs = _tiny_graphs()
    out = str(tmp_path / "w.npz")
    first, last = train(graphs, out, epochs=30, hidden=8, rounds=2,
                        lr=1e-2, val_frac=0.0, device='cpu')
    assert last < first
    z = np.load(out)
    assert 'enc.weight' in z.files and '__meta__' in z.files
    assert list(z['__meta__']) == [8, 2, 7, 2]
