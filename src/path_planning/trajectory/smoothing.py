"""Exact subsequence DP path smoothing."""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass
from typing import TYPE_CHECKING

from path_planning import config
from path_planning.geometry.spatial import angle_diff


if TYPE_CHECKING:
    from path_planning.collision.detector import CollisionDetector
    from path_planning.types import PlannerState, Point


@dataclass(frozen=True)
class _DpEntry:
    """One candidate prefix kept for a state (u, v) under dominance."""

    budget: float
    cost: float
    prev_key: tuple[int, int] | None
    prev_entry: _DpEntry | None


def smooth_path(
    path: list[PlannerState],
    origin: Point,
    target: Point,
    collision_detector: CollisionDetector,
    *,
    turn_radius: float = config.R,
    alpha_max_rad: float = config.ALPHA_MAX_RAD,
    l0: float = config.L0,
    dss: float = config.DSS,
    start_heading: float = 0.0,
    goal_heading: float | None = None,
    is_goal_heading_free: bool = False,
) -> list[PlannerState]:
    """Return the shortest FEASIBLE subsequence of the path, by exact DP over O..T."""
    if len(path) < 3:
        return path

    waypoints: list[Point] = [w for w, _ in path]
    head = 0
    if math.dist(origin, waypoints[0]) > 1.0:
        waypoints.insert(0, (origin[0], origin[1]))
        head = 1
    tail = 0
    if math.dist(target, waypoints[-1]) > 1.0:
        waypoints.append((target[0], target[1]))
        tail = 1
    count = len(waypoints)
    if count < 3 or count > config.SMOOTH_MAX_NODES:
        return path

    node_cost = config.SMOOTH_NODE_PENALTY_M
    goal_h = None if (is_goal_heading_free or not tail) else goal_heading

    dist = [[0.0] * count for _ in range(count)]
    brg = [[0.0] * count for _ in range(count)]
    clear = [[False] * count for _ in range(count)]
    for i in range(count):
        for j in range(i + 1, count):
            dist[i][j] = math.dist(waypoints[i], waypoints[j])
            brg[i][j] = math.atan2(
                waypoints[j][1] - waypoints[i][1], waypoints[j][0] - waypoints[i][0]
            )
            clear[i][j] = collision_detector.is_collision_free(
                waypoints[i], waypoints[j]
            )

    arc_memo: dict[tuple[int, int, int], bool] = {}

    def arc_ok(u: int, v: int, w: int) -> bool:
        if not config.ARC_CLEARANCE_CHECK:
            return True
        hit = arc_memo.get((u, v, w))
        if hit is None:
            hit = collision_detector.is_corner_arc_clear(
                brg[u][v], waypoints[v], waypoints[w]
            )
            arc_memo[(u, v, w)] = hit
        return hit

    by_cur: defaultdict[int, dict[int, list[_DpEntry]]] = defaultdict(dict)
    for j in range(1, count):
        if not clear[0][j]:
            continue
        if abs(angle_diff(brg[0][j], start_heading)) > config.TAKEOFF_RAY_TOL_RAD:
            continue
        by_cur[j][0] = [_DpEntry(dist[0][j], dist[0][j] + node_cost, None, None)]

    best: tuple[tuple[int, int], float, _DpEntry] | None = None
    for v in range(1, count):
        for u, entries in by_cur[v].items():
            for entry in entries:
                budget, cost = entry.budget, entry.cost
                if v == count - 1:
                    if (
                        goal_h is not None
                        and abs(angle_diff(brg[u][v], goal_h))
                        > config.APPROACH_RAY_TOL_RAD
                    ):
                        continue
                    if budget >= dss and (best is None or cost < best[1]):
                        best = ((u, v), cost, entry)
                    continue
                for w in range(v + 1, count):
                    if not clear[v][w]:
                        continue
                    turn = abs(angle_diff(brg[v][w], brg[u][v]))
                    if turn > alpha_max_rad:
                        continue
                    reserve = turn_radius * math.tan(turn / 2.0)
                    need = l0 if u == 0 else config.MIN_STRAIGHT_M
                    if budget - reserve < need:
                        continue
                    if not arc_ok(u, v, w):
                        continue
                    new_budget = dist[v][w] - reserve
                    new_cost = cost + dist[v][w] + node_cost
                    bucket = by_cur[w].setdefault(v, [])
                    if any(
                        e.budget >= new_budget - 1e-9 and e.cost <= new_cost + 1e-9
                        for e in bucket
                    ):
                        continue
                    bucket[:] = [
                        e
                        for e in bucket
                        if not (
                            new_budget >= e.budget - 1e-9 and new_cost <= e.cost + 1e-9
                        )
                    ]
                    bucket.append(_DpEntry(new_budget, new_cost, (u, v), entry))

    if best is None:
        return path

    key, _cost, entry_opt = best
    seq: list[int] = []
    cur_entry: _DpEntry | None = entry_opt
    while cur_entry is not None:
        seq.append(key[1])
        prev_key, prev_entry = cur_entry.prev_key, cur_entry.prev_entry
        if prev_key is None:
            seq.append(key[0])
            break
        key, cur_entry = prev_key, prev_entry
    seq.reverse()

    out: list[PlannerState] = []
    for idx in range(1 if head else 0, len(seq) - 1 if tail else len(seq)):
        node = seq[idx]
        heading = brg[seq[idx - 1]][node] if idx > 0 else path[0][1]
        out.append((waypoints[node], heading))
    return out if len(out) >= 1 else path
