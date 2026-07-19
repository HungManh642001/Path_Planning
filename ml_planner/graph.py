"""Explicit tangent/bitangent graph over a preprocessed scenario.

Nodes are the waypoint candidates the kinodynamic search navigates between
(bitangent touch points on inflated circles, tangent points from start/goal,
polygon hull vertices, start, goal); edges are collision-free chords plus
boundary arcs. The GNN guidance consumes this graph; nothing in core/ does.

All circle geometry is built on radius r + config.CONSTRUCTION_CLEARANCE_M,
mirroring the planner's construction-side clearance convention.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

import config
import core.spatial_utils as su
from core.kinodynamic_astar import KinodynamicAstar


EDGE_CHORD = 0.0
EDGE_ARC = 1.0
KNN_FILL_K = 6      # local-connectivity fill beyond the tangent structure


@dataclass
class Graph:
    nodes: np.ndarray       # (M, 2) float64 world coords
    node_feat: np.ndarray   # (M, 7) float32, spec §6
    edges: np.ndarray       # (E, 2) int32, undirected, a < b, deduplicated — except one intentional parallel pair per 2-node ring (both complementary arcs)
    edge_feat: np.ndarray   # (E, 2) float32: [length/D, EDGE_CHORD|EDGE_ARC]
    kdtree: cKDTree
    start_idx: int
    goal_idx: int
    scale: float            # D = start-goal distance (meters)


def bitangent_points(c1, r1, c2, r2):
    """Touch-point pairs of the up-to-4 bitangent segments between two circles.

    Returns [(p_on_circle1, p_on_circle2), ...] — external bitangents first
    (exist unless one circle contains the other), then internal ones (exist
    only when the circles are disjoint). [] for concentric circles.

    Construction: a bitangent touches circle1 at polar angle phi where
    phi = theta ± acos((r1 - s·r2)/d), theta = angle(c1→c2), s = +1 external /
    -1 internal; it touches circle2 at phi (external) or phi + pi (internal).
    """
    (x1, y1), (x2, y2) = c1, c2
    d = math.hypot(x2 - x1, y2 - y1)
    if d < 1e-9:
        return []
    theta = math.atan2(y2 - y1, x2 - x1)
    pairs = []
    for s in (+1.0, -1.0):              # +1 external, -1 internal
        cosval = (r1 - s * r2) / d
        if abs(cosval) > 1.0:
            continue                    # this bitangent family does not exist
        alpha = math.acos(cosval)
        for side in (+1.0, -1.0):
            phi1 = theta + side * alpha
            phi2 = phi1 if s > 0 else phi1 + math.pi
            p1 = (x1 + r1 * math.cos(phi1), y1 + r1 * math.sin(phi1))
            p2 = (x2 + r2 * math.cos(phi2), y2 + r2 * math.sin(phi2))
            pairs.append((p1, p2))
    return pairs


def build_graph(preprocessed):
    pre = preprocessed
    checker = KinodynamicAstar(pre)     # planner's exact collision predicate
    delta = config.CONSTRUCTION_CLEARANCE_M
    circles = [((cx, cy), r + delta) for (cx, cy), r in pre['circle_obstacles']]
    start = tuple(pre['start_pos'])
    goal = tuple(pre['goal_pos'])
    D = max(su.distance(start, goal), 1.0)

    nodes = [start, goal]
    owner = [-1, -1]                    # circle index a node sits on (-1 none)
    chord_pairs = [(0, 1)]              # candidate chord edges (checked below)

    # Tangent points from start/goal to every circle.
    for ci, (c, rc) in enumerate(circles):
        for src_idx, p in ((0, start), (1, goal)):
            for t in su.circle_tangent_points(p, c, rc):
                nodes.append(t)
                owner.append(ci)
                chord_pairs.append((src_idx, len(nodes) - 1))

    # Bitangent touch points between every circle pair.
    for i in range(len(circles)):
        for j in range(i + 1, len(circles)):
            for p_i, p_j in bitangent_points(circles[i][0], circles[i][1],
                                             circles[j][0], circles[j][1]):
                nodes.append(p_i); owner.append(i)
                nodes.append(p_j); owner.append(j)
                chord_pairs.append((len(nodes) - 2, len(nodes) - 1))

    # Polygon hull vertices; boundary segments are chord candidates.
    for poly in pre['polygon_obstacles']:
        first = len(nodes)
        n = len(poly)
        for v in poly:
            nodes.append((float(v[0]), float(v[1])))
            owner.append(-1)
        for k in range(n):
            chord_pairs.append((first + k, first + (k + 1) % n))

    pts = np.asarray(nodes, dtype=np.float64)
    seen = set()
    edges, efeat = [], []

    def add_edge(i, j, etype, length=None):
        if i == j:
            return
        a, b = (i, j) if i < j else (j, i)
        if (a, b) in seen:
            return
        seen.add((a, b))
        L = su.distance(nodes[a], nodes[b]) if length is None else length
        edges.append((a, b))
        efeat.append((L / D, etype))

    # Chord edges: only when the planner's exact collision predicate is clear.
    for i, j in chord_pairs:
        if checker._check_collision(nodes[i], nodes[j]):
            add_edge(i, j, EDGE_CHORD)

    # Arc edges between angularly consecutive same-circle nodes. Deliberately
    # NOT collision-checked: the straight chord between boundary nodes dips
    # inside its own circle for any span > ~2*sqrt(2*delta/r) rad, so
    # _check_collision would reject nearly all of them; the edge is a
    # message-passing conduit carrying the true arc length, and the planner's
    # arc-hop machinery re-validates real geometry at search time.
    for ci, (c, rc) in enumerate(circles):
        ring = [k for k in range(len(nodes)) if owner[k] == ci]
        if len(ring) < 2:
            continue
        ring.sort(key=lambda k: math.atan2(nodes[k][1] - c[1], nodes[k][0] - c[0]))
        pairs = list(zip(ring, ring[1:] + ring[:1]))
        if len(ring) == 2:
            pairs = pairs[:1]       # zip repeats the same pair; handle both arcs below
        for a, b in pairs:
            phi_a = math.atan2(nodes[a][1] - c[1], nodes[a][0] - c[0])
            phi_b = math.atan2(nodes[b][1] - c[1], nodes[b][0] - c[0])
            dphi = (phi_b - phi_a) % (2.0 * math.pi)
            add_edge(a, b, EDGE_ARC, length=rc * dphi)
            if len(ring) == 2:
                # Complementary arc: a distinct riding option that the
                # undirected (min,max) dedup would otherwise silently drop.
                aa, bb = (a, b) if a < b else (b, a)
                edges.append((aa, bb))
                efeat.append(((rc * (2.0 * math.pi - dphi)) / D, EDGE_ARC))

    # kNN fill for local connectivity (clear segments only).
    tree = cKDTree(pts)
    k = min(KNN_FILL_K + 1, len(pts))
    if k >= 2:
        _, nbrs = tree.query(pts, k=k)
        nbrs = np.atleast_2d(nbrs)
        for i in range(len(pts)):
            for j in nbrs[i][1:]:
                j = int(j)
                a, b = (i, j) if i < j else (j, i)
                if (a, b) in seen:
                    continue
                if checker._check_collision(nodes[i], nodes[j]):
                    add_edge(i, j, EDGE_CHORD)

    # Node features (spec §6), normalized by D.
    bearing_sg = su.angle_to_heading(start, goal)
    feat = np.zeros((len(pts), 7), dtype=np.float32)
    dx = goal[0] - pts[:, 0]
    dy = goal[1] - pts[:, 1]
    feat[:, 0] = np.hypot(dx, dy) / D
    ang = np.arctan2(dy, dx) - bearing_sg
    feat[:, 1] = np.sin(ang)
    feat[:, 2] = np.cos(ang)
    feat[1, 1] = 0.0                    # goal node: direction undefined, sin=0
    feat[1, 2] = 1.0                    # cos=1 keeps the unit-norm invariant
    feat[:, 3] = [circles[o][1] / D if o >= 0 else 0.0 for o in owner]
    feat[1, 4] = 1.0                    # is_goal
    feat[0, 5] = 1.0                    # is_start
    feat[:, 6] = 1.0                    # safezone-distance slot (constant this phase)

    edges_arr = (np.asarray(edges, dtype=np.int32) if edges
                 else np.zeros((0, 2), dtype=np.int32))
    efeat_arr = (np.asarray(efeat, dtype=np.float32) if efeat
                 else np.zeros((0, 2), dtype=np.float32))
    return Graph(nodes=pts, node_feat=feat, edges=edges_arr, edge_feat=efeat_arr,
                 kdtree=tree, start_idx=0, goal_idx=1, scale=float(D))
