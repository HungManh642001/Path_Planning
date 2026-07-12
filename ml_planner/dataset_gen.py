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
import core.spatial_utils as su
from ml_planner.focal_astar import FocalKinodynamicAstar


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
