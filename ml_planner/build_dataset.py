"""Build a HARD-scenario guidance dataset (parallel, sharded).

Generates a hard scenario distribution (many islands, cluster/wall topology,
adverse start/goal headings) and labels each with dense cost-to-go via
`dataset_gen.generate_sample`. Runs oracle solves across CPU cores and writes
sharded `.npz` files into ml_planner/data/ to bound peak RAM.

Usage:
  python -m ml_planner.build_dataset [START] [NSEEDS] [NPROC] [TARGET]
e.g.
  python -m ml_planner.build_dataset 0 2400 6 2000
"""
import contextlib
import math
import multiprocessing as mp
import os
import random
import sys
import time

import numpy as np

import config
import core.map_generator as mg
import core.spatial_utils as su
import ml_planner.config as mlcfg
import ml_planner.dataset_gen as dg

# Per-scenario safety cap (a worker never hangs on a pathological seed).
WORKER_BUDGET_S = 15.0
MAX_EXPLORE = 6000          # denser labels for hard maps than the default 4000
SHARD_SIZE = 150           # samples per output shard (bounds peak RAM ~0.5GB)
OUT_DIR = os.path.join(os.path.dirname(__file__), "data")


def hard_scenario(seed):
    """A deliberately hard map: far start/goal, many islands, cluster/wall
    topology, and ADVERSE start/goal headings (large offset from the direct
    bearing) so the search must reorient -- exactly where Euclid is loose."""
    random.seed(seed)
    w, h = config.MAP_WIDTH, config.MAP_HEIGHT
    while True:
        start = (random.uniform(0.1 * w, 0.9 * w), random.uniform(0.1 * h, 0.9 * h))
        goal = (random.uniform(0.1 * w, 0.9 * w), random.uniform(0.1 * h, 0.9 * h))
        if su.distance(start, goal) > 400000:
            break
    bearing = su.angle_to_heading(start, goal)

    def adverse():
        return bearing + random.choice([-1, 1]) * random.uniform(math.pi / 2, math.pi)

    return mg.create_scenario({
        'map_bounds': (w, h),
        'start': start,
        'goal': goal,
        'start_heading': adverse(),
        'goal_heading': adverse(),
        'num_islands': random.randint(12, 20),
        'num_dynamic_obstacles': random.randint(5, 15),
        'topology': random.choice(['center_cluster', 'wall_block']),
        'seed': seed,
    })


def _init_worker():
    """Force generate_sample's oracle to respect a wall-clock cap (its own
    _no_budget would otherwise remove all caps and risk a hang)."""
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
        scen = hard_scenario(seed)
        sample = dg.generate_sample(scen, grid_res=mlcfg.GRID_RES, max_explore=MAX_EXPLORE)
    except Exception as e:
        return dict(seed=seed, ok=False, err=f"{type(e).__name__}: {e}", dt=time.perf_counter() - t0)
    if sample is None:
        return dict(seed=seed, ok=False, err=None, dt=time.perf_counter() - t0)
    return dict(seed=seed, ok=True, sample=sample,
                islands=len(scen.get('islands', [])),
                labeled=int(sample['mask'].sum()), dt=time.perf_counter() - t0)


def _write_shard(shard_idx, buf):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f"guidance_dataset_{shard_idx:03d}.npz")
    np.savez_compressed(
        path,
        channels=np.stack([s['channels'] for s in buf]),
        label=np.stack([s['label'] for s in buf]),
        mask=np.stack([s['mask'] for s in buf]),
        affine=np.stack([s['affine'] for s in buf]),
    )
    print(f"  == wrote shard {shard_idx:03d}: {len(buf)} samples -> {path}", flush=True)


def main():
    start = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    nseeds = int(sys.argv[2]) if len(sys.argv) > 2 else 2400
    nproc = int(sys.argv[3]) if len(sys.argv) > 3 else max(1, (os.cpu_count() or 2) - 1)
    target = int(sys.argv[4]) if len(sys.argv) > 4 else 2000

    print(f"HARD dataset: seeds {start}..{start+nseeds-1}, {nproc} procs, "
          f"grid={mlcfg.GRID_RES}, target={target}", flush=True)
    t0 = time.perf_counter()
    buf, shard, solved, skipped, errored = [], 0, 0, 0, 0
    labeled_total = 0

    with mp.Pool(nproc, initializer=_init_worker) as pool:
        for r in pool.imap_unordered(_worker, range(start, start + nseeds), chunksize=1):
            if not r['ok']:
                if r['err']:
                    errored += 1
                    print(f"[seed {r['seed']}] ERROR {r['err']}", flush=True)
                else:
                    skipped += 1
                continue
            buf.append(r['sample'])
            solved += 1
            labeled_total += r['labeled']
            print(f"[seed {r['seed']}] ok islands={r['islands']} labeled={r['labeled']} {r['dt']:.1f}s "
                  f"({solved}/{target})", flush=True)
            if len(buf) >= SHARD_SIZE:
                _write_shard(shard, buf); shard += 1; buf = []
            if solved >= target:
                break
    if buf:
        _write_shard(shard, buf); shard += 1

    dt = time.perf_counter() - t0
    print(f"\nDONE: solved={solved} skipped={skipped} errored={errored} shards={shard} "
          f"labeled_cells={labeled_total} (mean {labeled_total/max(1,solved):.0f}/sample) "
          f"in {dt/60:.1f} min -> {OUT_DIR}/guidance_dataset_*.npz", flush=True)


if __name__ == "__main__":
    main()
