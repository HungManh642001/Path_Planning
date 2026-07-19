"""Tangent-graph GNN guidance: one numpy MPNN forward per problem, k-NN
lookup per state. Mirrors guidance.Guidance's contract; falls back cleanly
(available=False) when the weight file is absent or malformed.

The forward pass replicates ml_planner/train/train_graph.py::MPNN exactly
(same state_dict key names; torch.nn.GRUCell gate order r|z|n) — parity is
pinned by the golden test in graph_guidance_test.py.
"""
import os

import numpy as np

import ml_planner.config as mlcfg
from ml_planner.graph import build_graph

LARGE = 1e18
LOOKUP_K = 3


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def _softplus(x):
    return np.logaddexp(0.0, x)


def _gru_cell(x, h, w):
    """torch.nn.GRUCell: gates chunked [reset | update | new]."""
    gi = x @ w['upd.weight_ih'].T + w['upd.bias_ih']
    gh = h @ w['upd.weight_hh'].T + w['upd.bias_hh']
    ir, iz, inn = np.split(gi, 3, axis=1)
    hr, hz, hn = np.split(gh, 3, axis=1)
    r = _sigmoid(ir + hr)
    z = _sigmoid(iz + hz)
    n = np.tanh(inn + r * hn)
    return (1.0 - z) * n + z * h


def mpnn_forward(weights, node_feat, edges, edge_feat, rounds):
    """Numpy mirror of MPNN.forward. `edges` are the undirected (E,2) graph
    edges; both directions are materialized here, matching training."""
    w = weights
    x = np.asarray(node_feat, dtype=np.float64)
    h = np.maximum(x @ w['enc.weight'].T + w['enc.bias'], 0.0)
    e = np.asarray(edges, dtype=np.int64)
    src = np.concatenate([e[:, 0], e[:, 1]])
    dst = np.concatenate([e[:, 1], e[:, 0]])
    ea = np.concatenate([edge_feat, edge_feat], axis=0).astype(np.float64)
    for _ in range(rounds):
        m = np.concatenate([h[src], h[dst], ea], axis=1)
        m = np.maximum(m @ w['msg.0.weight'].T + w['msg.0.bias'], 0.0)
        m = m @ w['msg.2.weight'].T + w['msg.2.bias']
        agg = np.zeros_like(h)
        np.add.at(agg, dst, m)
        h = _gru_cell(agg, h, w)
    out = np.maximum(h @ w['dec.0.weight'].T + w['dec.0.bias'], 0.0)
    return _softplus(out @ w['dec.2.weight'].T + w['dec.2.bias']).squeeze(-1)


class GraphGuidance:
    """Loads MPNN weights; builds one per-node value field per problem."""

    def __init__(self, model_path=mlcfg.GRAPH_MODEL_PATH):
        self.model_path = model_path
        self.available = False
        self.weights = None
        self.rounds = None
        self.graph = None
        self.values = None
        if os.path.exists(model_path):
            try:
                z = np.load(model_path)
                meta = z['__meta__']
                self.weights = {k: z[k].astype(np.float64)
                                for k in z.files if k != '__meta__'}
                self.rounds = int(meta[1])
                self.available = True
            except Exception:
                self.available = False

    def build_field(self, preprocessed):
        g = build_graph(preprocessed)
        r = mpnn_forward(self.weights, g.node_feat, g.edges, g.edge_feat,
                         self.rounds)
        euclid = g.node_feat[:, 0].astype(np.float64) * g.scale
        self.values = euclid + r * g.scale          # meters; >= Euclid by construction
        self.graph = g

    def lookup(self, waypoint):
        if self.graph is None or len(self.graph.nodes) == 0:
            return LARGE
        k = min(LOOKUP_K, len(self.graph.nodes))
        d, idx = self.graph.kdtree.query(waypoint, k=k)
        d = np.atleast_1d(np.asarray(d, dtype=np.float64))
        idx = np.atleast_1d(idx)
        return float(np.min(d + self.values[idx]))


# Loaded weight files cached by path so repeated planning calls in one process
# don't reload them (build_field still runs per problem) — same pattern as
# guidance._GUIDANCE_CACHE.
_GRAPH_GUIDANCE_CACHE = {}


def _cached(model_path):
    g = _GRAPH_GUIDANCE_CACHE.get(model_path)
    if g is None:
        g = GraphGuidance(model_path)
        _GRAPH_GUIDANCE_CACHE[model_path] = g
    return g


def make_graph_secondary(preprocessed, model_path=None, guidance_obj=None):
    """(secondary_callable, True), or (None, False) when no model is available
    or build_field fails (caller falls back to hand-crafted)."""
    g = guidance_obj if guidance_obj is not None else _cached(
        model_path or mlcfg.GRAPH_MODEL_PATH)
    if not g.available:
        return None, False
    try:
        g.build_field(preprocessed)
    except Exception:
        return None, False
    return (lambda state: g.lookup(state.waypoint)), True
