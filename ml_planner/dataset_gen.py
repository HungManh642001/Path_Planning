"""Generate dense cost-to-go training data from oracle solves.

Runs the focal planner at focal_eps=0 (== optimal) with no time/iteration
budget, records every explored edge, and runs a backward Dijkstra from the
reached goal to label every explored lattice cell with its true cost-to-go.
Uses core/* read-only.
"""

import contextlib
import heapq
from collections import defaultdict

import numpy as np

import config
import core.preprocessing as prep
import core.spatial_utils as su
from ml_planner.focal_astar import FocalKinodynamicAstar
import ml_planner.raster as raster
import ml_planner.config as mlcfg


@contextlib.contextmanager
def _no_budget():
    """Temporarily remove the wall-clock and iteration caps (restored after)."""
    old_t, old_i = config.TIME_BUDGET_S, config.MAX_ITERATIONS
    config.TIME_BUDGET_S = None
    config.MAX_ITERATIONS = 10 ** 8
    try:
        yield
    finally:
        config.TIME_BUDGET_S = old_t
        config.MAX_ITERATIONS = old_i


class _RecordingAstar(FocalKinodynamicAstar):
    """Optimal focal search that records every generated edge (u -> v, cost)."""

    def __init__(self, preprocessed_scenario):
        super().__init__(preprocessed_scenario, focal_eps=0.0)
        self.edges = []
        self.key2wp = {}

    def get_next_states(self, current):
        successors = super().get_next_states(current)
        u = su.state_to_tuple(current.waypoint, current.heading)
        self.key2wp.setdefault(u, current.waypoint)
        for st, cost in successors:
            v = su.state_to_tuple(st.waypoint, st.heading)
            self.key2wp.setdefault(v, st.waypoint)
            self.edges.append((u, v, cost))
        return successors


def backward_costs(edges, goal_key):
    """Exact cost-to-go for every node that can reach goal_key, via Dijkstra
    on the reversed edge set."""
    radj = defaultdict(list)
    for u, v, c in edges:
        radj[v].append((u, c))
    dist = {goal_key: 0.0}
    pq = [(0.0, goal_key)]
    while pq:
        d, node = heapq.heappop(pq)
        if d > dist.get(node, float('inf')):
            continue
        for u, c in radj[node]:
            nd = d + c
            if nd < dist.get(u, float('inf')):
                dist[u] = nd
                heapq.heappush(pq, (nd, u))
    return dist


def rasterize_labels(costs, key2wp, affine, grid_res):
    """Min cost-to-go per grid cell (field is position-only, so aggregate the
    best achievable cost at each cell). Returns (label, mask) float32."""
    label = np.full((grid_res, grid_res), np.inf, dtype=np.float64)
    for key, c in costs.items():
        wp = key2wp.get(key)
        if wp is None:
            continue
        gx, gy = affine.world_to_grid(*wp)
        ix, iy = int(round(gx)), int(round(gy))
        if 0 <= iy < grid_res and 0 <= ix < grid_res and c < label[iy, ix]:
            label[iy, ix] = c
    mask = np.isfinite(label)
    label = np.where(mask, label, 0.0).astype(np.float32)
    return label, mask.astype(np.float32)


def generate_sample(scenario, grid_res=None):
    """Run the oracle, backward-label, rasterize. None if unsolved."""
    if grid_res is None:
        grid_res = mlcfg.GRID_RES
    with _no_budget():
        planner = _RecordingAstar(prep.prepare_scenario(scenario))
        path = planner.search()
    if path is None or planner.raw_route is None:
        return None
    pre = planner.scenario
    goal_key = su.state_to_tuple(*planner.raw_route[-1])
    costs = backward_costs(planner.edges, goal_key)
    affine = raster.compute_crop(pre, grid_res)
    channels = raster.build_channels(pre, affine, grid_res)
    label, mask = rasterize_labels(costs, planner.key2wp, affine, grid_res)
    return {
        'channels': channels,
        'label': label,
        'mask': mask,
        'affine': np.array([affine.x0, affine.y0, affine.scale, grid_res],
                           dtype=np.float64),
    }


def export_dataset(scenarios, out_path, grid_res=None):
    """Write a compressed .npz stacking every solved sample. Returns count."""
    if grid_res is None:
        grid_res = mlcfg.GRID_RES
    chans, labels, masks, affines = [], [], [], []
    for scenario in scenarios:
        sample = generate_sample(scenario, grid_res)
        if sample is None:
            continue
        chans.append(sample['channels'])
        labels.append(sample['label'])
        masks.append(sample['mask'])
        affines.append(sample['affine'])
    if not chans:
        raise ValueError("no solvable scenarios produced a sample")
    np.savez_compressed(
        out_path,
        channels=np.stack(chans), label=np.stack(labels),
        mask=np.stack(masks), affine=np.stack(affines),
    )
    return len(chans)
