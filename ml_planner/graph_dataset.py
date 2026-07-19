"""Build the GNN graph dataset (parallel, sharded).

Per scenario: unbudgeted oracle solve (dataset_gen's exploring labeler +
backward Dijkstra) -> per-node cost-to-go labels snapped from the explored
waypoints -> variable-size graph tensors. Shards concatenate graphs along
axis 0 with offset arrays; edge indices stay LOCAL to each graph.

Usage:
  python -m ml_planner.graph_dataset [START] [NSEEDS] [NPROC] [TARGET]
e.g.
  python -m ml_planner.graph_dataset 0 2400 6 2000
"""
import contextlib
import glob
import multiprocessing as mp
import os
import sys
import time

import numpy as np

import config
import core.preprocessing as prep
import ml_planner.dataset_gen as dg
from ml_planner.build_dataset import hard_scenario
from ml_planner.graph import build_graph

SNAP_RADIUS_M = 2.0 * config.STATE_POS_QUANTUM      # 2 km on the search lattice
MAX_EXPLORE = 6000
MIN_LABELED = 8            # discard scenarios that label almost nothing
SHARD_SIZE = 400
WORKER_BUDGET_S = 15.0
OUT_DIR = os.path.join(os.path.dirname(__file__), "data")


def snap_labels(graph, costs, key2wp):
    """V_label(v) = min over labeled oracle waypoints w within SNAP_RADIUS_M
    of [cost_to_go(w) + dist(v, w)] (a relaxation upper bound); mask=0 where
    no labeled waypoint is near."""
    m = len(graph.nodes)
    label = np.full(m, np.inf, dtype=np.float64)
    for key, c in costs.items():
        wp = key2wp.get(key)
        if wp is None:
            continue
        idxs = graph.kdtree.query_ball_point(wp, r=SNAP_RADIUS_M)
        if not idxs:
            continue
        d = np.hypot(graph.nodes[idxs, 0] - wp[0], graph.nodes[idxs, 1] - wp[1])
        label[idxs] = np.minimum(label[idxs], c + d)
    mask = np.isfinite(label)
    return (np.where(mask, label, 0.0).astype(np.float32),
            mask.astype(np.float32))


def generate_graph_sample(scenario, max_explore=MAX_EXPLORE):
    """Oracle-label one scenario onto its tangent graph. None when the oracle
    never reaches the goal or labels too few nodes."""
    with dg._no_budget():
        planner = dg._ExploringAstar(prep.prepare_scenario(scenario),
                                     max_explore=max_explore)
        found = planner.explore()
    if not found:
        return None
    pre = planner.scenario
    costs = dg.backward_costs(planner.edges, planner.goal_key)
    g = build_graph(pre)
    label, mask = snap_labels(g, costs, planner.key2wp)
    if mask.sum() < MIN_LABELED:
        return None
    return dict(node_feat=g.node_feat, edges=g.edges, edge_feat=g.edge_feat,
                label=label, mask=mask, scale=np.float64(g.scale))


def write_shard(path, samples):
    """Concatenate variable-size graphs with offset arrays."""
    node_counts = np.asarray([s['node_feat'].shape[0] for s in samples], np.int64)
    edge_counts = np.asarray([s['edges'].shape[0] for s in samples], np.int64)
    np.savez_compressed(
        path,
        node_feat=np.concatenate([s['node_feat'] for s in samples], axis=0),
        edges=np.concatenate([s['edges'] for s in samples], axis=0),
        edge_feat=np.concatenate([s['edge_feat'] for s in samples], axis=0),
        label=np.concatenate([s['label'] for s in samples]),
        mask=np.concatenate([s['mask'] for s in samples]),
        node_offsets=np.concatenate([[0], np.cumsum(node_counts)]),
        edge_offsets=np.concatenate([[0], np.cumsum(edge_counts)]),
        scale=np.asarray([float(s['scale']) for s in samples], np.float64),
    )


def load_shards(data_dir, pattern="graph_dataset_*.npz"):
    """Read every shard back into a list of per-graph dicts.

    Each npz member is decompressed exactly ONCE per shard and per-graph
    slices are COPIED out of it. Re-indexing the NpzFile per graph would
    decompress the full member on every access and return slices that pin
    those full-size buffers alive -- on a ~1.6k-graph dataset that exhausts
    Colab's RAM (the process dies with ^C) before training even starts.
    """
    out = []
    for path in sorted(glob.glob(os.path.join(data_dir, pattern))):
        with np.load(path) as z:
            nf, ed, ef = z['node_feat'], z['edges'], z['edge_feat']
            lb, mk = z['label'], z['mask']
            no, eo, sc = z['node_offsets'], z['edge_offsets'], z['scale']
            for i in range(len(no) - 1):
                out.append(dict(
                    node_feat=nf[no[i]:no[i + 1]].copy(),
                    edges=ed[eo[i]:eo[i + 1]].copy(),
                    edge_feat=ef[eo[i]:eo[i + 1]].copy(),
                    label=lb[no[i]:no[i + 1]].copy(),
                    mask=mk[no[i]:no[i + 1]].copy(),
                    scale=np.float64(sc[i])))
    return out


def _init_worker():
    """Cap the oracle inside workers (same pattern as build_dataset)."""
    @contextlib.contextmanager
    def _capped():
        old_t, old_i = config.TIME_BUDGET_S, config.MAX_ITERATIONS
        config.TIME_BUDGET_S = WORKER_BUDGET_S
        config.MAX_ITERATIONS = 2_000_000
        try:
            yield
        finally:
            config.TIME_BUDGET_S = old_t
            config.MAX_ITERATIONS = old_i
    dg._no_budget = _capped


def _worker(seed):
    t0 = time.perf_counter()
    try:
        sample = generate_graph_sample(hard_scenario(seed))
    except Exception as e:
        return dict(seed=seed, ok=False, err=f"{type(e).__name__}: {e}",
                    dt=time.perf_counter() - t0)
    if sample is None:
        return dict(seed=seed, ok=False, err=None, dt=time.perf_counter() - t0)
    return dict(seed=seed, ok=True, sample=sample,
                labeled=int(sample['mask'].sum()), dt=time.perf_counter() - t0)


def main():
    start = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    nseeds = int(sys.argv[2]) if len(sys.argv) > 2 else 2400
    nproc = int(sys.argv[3]) if len(sys.argv) > 3 else max(1, (os.cpu_count() or 2) - 1)
    target = int(sys.argv[4]) if len(sys.argv) > 4 else 2000

    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"GRAPH dataset: seeds {start}..{start + nseeds - 1}, {nproc} procs, "
          f"target={target}", flush=True)
    t0 = time.perf_counter()
    buf, shard, solved, skipped, errored = [], 0, 0, 0, 0
    with mp.Pool(nproc, initializer=_init_worker) as pool:
        for res in pool.imap_unordered(_worker, range(start, start + nseeds)):
            if res['ok']:
                solved += 1
                buf.append(res['sample'])
                if len(buf) >= SHARD_SIZE:
                    write_shard(os.path.join(
                        OUT_DIR, f"graph_dataset_{shard:03d}.npz"), buf)
                    print(f"  == wrote shard {shard:03d}: {len(buf)} graphs",
                          flush=True)
                    buf, shard = [], shard + 1
            elif res['err']:
                errored += 1
                print(f"  seed {res['seed']}: ERROR {res['err']}", flush=True)
            else:
                skipped += 1
            if solved >= target:
                pool.terminate()
                break
    if buf:
        write_shard(os.path.join(OUT_DIR, f"graph_dataset_{shard:03d}.npz"), buf)
        print(f"  == wrote shard {shard:03d}: {len(buf)} graphs", flush=True)
    print(f"done: solved={solved} skipped={skipped} errored={errored} "
          f"in {time.perf_counter() - t0:.0f}s", flush=True)


if __name__ == '__main__':
    main()
