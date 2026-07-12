"""Generate dense cost-to-go training data from oracle solves.

Runs the focal planner at focal_eps=0 (== optimal) with no time/iteration
budget, records every explored edge, and runs a backward Dijkstra from the
reached goal to label every explored lattice cell with its true cost-to-go.
Uses core/* read-only.
"""

import contextlib
import heapq
import itertools
import time
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


# Expansions the exploring labeler runs. It does NOT stop at the goal: it
# keeps expanding in A* (f) order so backward-Dijkstra labels a DENSE
# neighborhood around the optimal corridor, not just the thin path the plain
# solve explores. Higher = denser labels but slower / more memory.
DEFAULT_MAX_EXPLORE = 4000


class _ExploringAstar(_RecordingAstar):
    """Records edges like _RecordingAstar but expands in A* (f) order WITHOUT
    stopping at the goal, up to max_explore expansions (or the time budget /
    open exhaustion). It notes the first goal-accepting state's key
    (`goal_key`) as the backward-Dijkstra source. This densifies the labeled
    region far beyond the plain optimal solve."""

    def __init__(self, preprocessed_scenario, max_explore=DEFAULT_MAX_EXPLORE):
        super().__init__(preprocessed_scenario)
        self.max_explore = max_explore
        self.goal_key = None

    def explore(self):
        """Best-first (f = g + h_euclid) expansion recording all edges; returns
        True once a goal-accepting state was seen (so it can be labeled)."""
        _start = time.perf_counter()
        _budget = config.TIME_BUDGET_S
        if not self.start_corners:
            return False
        counter = itertools.count()
        open_heap = []
        for corner in self.start_corners:
            corner.h_cost = self.heuristic(corner, self.goal_state)
            if corner.g_cost < self.g_scores[corner]:
                self.g_scores[corner] = corner.g_cost
            heapq.heappush(open_heap, (corner.g_cost + corner.h_cost, next(counter), corner))
        expanded = 0
        while open_heap and expanded < self.max_explore:
            if _budget is not None and (time.perf_counter() - _start) > _budget:
                break
            _, _, current = heapq.heappop(open_heap)
            if current in self.closed_set:
                continue
            self.closed_set.add(current)
            expanded += 1
            if self.goal_key is None and self._goal_reached(current) is not None:
                self.goal_key = su.state_to_tuple(current.waypoint, current.heading)
            for nxt, tcost in self.get_next_states(current):
                if nxt in self.closed_set:
                    continue
                tentative_g = self.g_scores[current] + tcost
                if tentative_g < self.g_scores.get(nxt, float('inf')):
                    nxt.parent = current
                    self.g_scores[nxt] = tentative_g
                    nxt.g_cost = tentative_g
                    nxt.h_cost = self.heuristic(nxt, self.goal_state)
                    heapq.heappush(open_heap, (tentative_g + nxt.h_cost, next(counter), nxt))
        return self.goal_key is not None


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


def generate_sample(scenario, grid_res=None, max_explore=DEFAULT_MAX_EXPLORE):
    """Run the exploring labeler, backward-label, rasterize. None if the goal
    was never reached within the exploration budget."""
    if grid_res is None:
        grid_res = mlcfg.GRID_RES
    with _no_budget():
        planner = _ExploringAstar(prep.prepare_scenario(scenario), max_explore=max_explore)
        found = planner.explore()
    if not found:
        return None
    pre = planner.scenario
    costs = backward_costs(planner.edges, planner.goal_key)
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
