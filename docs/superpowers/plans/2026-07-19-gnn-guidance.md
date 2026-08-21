# GNN Tangent-Graph Guidance (Prototype) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A GNN secondary heuristic for the focal A* planner that runs on an explicit tangent/bitangent graph (ms-scale inference), evaluated head-to-head against base / hand-crafted / CNN in the existing benchmark.

**Architecture:** Four new files in `ml_planner/` (graph builder, dataset builder, numpy inference + secondary, GPU trainer) plus an extension of `ml_planner/benchmark.py`. Nothing in `core/` changes; the planner consumes the GNN through the exact same `(callback, available)` secondary contract as the CNN. Spec: `docs/superpowers/specs/2026-07-19-gnn-guidance-design.md`.

**Tech Stack:** numpy + scipy (`cKDTree`) at plan time; PyTorch (hand-rolled MPNN, no PyG) at train time only; weights ship as a plain `.npz`.

## Global Constraints

- Do NOT modify anything under `core/` — the collision predicate is reused by *instantiating* `KinodynamicAstar` and calling its `_check_collision`.
- No new runtime dependency: production inference is pure numpy from `ml_planner/models/graph_guidance.npz`; PyTorch imports live only in `ml_planner/train/train_graph.py` and torch-gated tests (`pytest.importorskip('torch')`).
- Missing/broken model file must degrade to the hand-crafted secondary exactly like the CNN does (`(None, False)` contract).
- Node features are 7-dim, edge features 2-dim, normalized by `D = dist(start_pos, goal_pos)` — copied verbatim from spec §6. Feature 7 (safezone distance) is constant `1.0` this phase.
- `V̂(v) = dist(v, goal_pos) + softplus(r(v))·D` — the ≥-Euclid property is structural, never post-hoc clamped.
- Test files are named `*_test.py` (committed); `test_*.py` is gitignored scratch.
- Run tests from repo root: `python -m pytest -q` (pytest.ini sets pythonpath).
- Commit messages end with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.

---

### Task 1: Bitangent geometry (`bitangent_points`)

**Files:**
- Create: `ml_planner/graph.py` (module header + this one function)
- Test: `ml_planner/tests/graph_test.py`

**Interfaces:**
- Produces: `bitangent_points(c1, r1, c2, r2) -> list[((x,y),(x,y))]` — up to 4 pairs `(point_on_circle1, point_on_circle2)`, external bitangents first; internal bitangents omitted when the circles overlap (`d < r1+r2`); `[]` when concentric or one circle contains the other.

- [ ] **Step 1: Write the failing tests**

Create `ml_planner/tests/graph_test.py`:

```python
import math

import numpy as np

from ml_planner.graph import bitangent_points


def _tangency_ok(p_on_1, p_on_2, c1, c2):
    """The bitangent chord must be perpendicular to both touch radii."""
    vx, vy = p_on_2[0] - p_on_1[0], p_on_2[1] - p_on_1[1]
    r1x, r1y = p_on_1[0] - c1[0], p_on_1[1] - c1[1]
    r2x, r2y = p_on_2[0] - c2[0], p_on_2[1] - c2[1]
    L = math.hypot(vx, vy)
    return (abs(vx * r1x + vy * r1y) / L < 1e-9
            and abs(vx * r2x + vy * r2y) / L < 1e-9)


def test_bitangent_two_unit_circles_exact():
    c1, c2 = (0.0, 0.0), (4.0, 0.0)
    pairs = bitangent_points(c1, 1.0, c2, 1.0)
    assert len(pairs) == 4
    for p1, p2 in pairs:
        assert abs(math.hypot(p1[0] - c1[0], p1[1] - c1[1]) - 1.0) < 1e-9
        assert abs(math.hypot(p2[0] - c2[0], p2[1] - c2[1]) - 1.0) < 1e-9
        assert _tangency_ok(p1, p2, c1, c2)
    # External bitangents of equal circles are horizontal lines y=±1.
    ext = sorted(pairs[:2], key=lambda pr: pr[0][1])
    assert np.allclose(ext[0][0], (0.0, -1.0)) and np.allclose(ext[0][1], (4.0, -1.0))
    assert np.allclose(ext[1][0], (0.0, 1.0)) and np.allclose(ext[1][1], (4.0, 1.0))


def test_bitangent_overlapping_circles_drop_internal():
    # d=3 < r1+r2=4: internal bitangents vanish, external survive.
    pairs = bitangent_points((0.0, 0.0), 2.0, (3.0, 0.0), 2.0)
    assert len(pairs) == 2
    for p1, p2 in pairs:
        assert _tangency_ok(p1, p2, (0.0, 0.0), (3.0, 0.0))


def test_bitangent_concentric_returns_empty():
    assert bitangent_points((5.0, 5.0), 2.0, (5.0, 5.0), 1.0) == []
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ml_planner/tests/graph_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner.graph'` (or ImportError).

- [ ] **Step 3: Write the implementation**

Create `ml_planner/graph.py`:

```python
"""Explicit tangent/bitangent graph over a preprocessed scenario.

Nodes are the waypoint candidates the kinodynamic search navigates between
(bitangent touch points on inflated circles, tangent points from start/goal,
polygon hull vertices, start, goal); edges are collision-free chords plus
boundary arcs. The GNN guidance consumes this graph; nothing in core/ does.

All circle geometry is built on radius r + config.CONSTRUCTION_CLEARANCE_M,
mirroring the planner's construction-side clearance convention.
"""
import math

import config


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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest ml_planner/tests/graph_test.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add ml_planner/graph.py ml_planner/tests/graph_test.py
git commit -m "feat(ml_planner): bitangent touch-point geometry for the tangent graph

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: Graph construction (`build_graph`)

**Files:**
- Modify: `ml_planner/graph.py` (append)
- Test: `ml_planner/tests/graph_test.py` (append)

**Interfaces:**
- Consumes: `bitangent_points` (Task 1); `core.kinodynamic_astar.KinodynamicAstar` (its `_check_collision(p1, p2) -> bool`, True = clear); `core.spatial_utils.circle_tangent_points(point, center, radius)`, `.distance`, `.angle_to_heading`; preprocessed keys `start_pos`, `goal_pos`, `circle_obstacles`, `polygon_obstacles`.
- Produces:
  - `Graph` dataclass: `nodes (M,2) float64`, `node_feat (M,7) float32`, `edges (E,2) int32` (undirected, `a < b`, deduplicated), `edge_feat (E,2) float32` (`[length/D, type]`, type 0.0=chord / 1.0=arc), `kdtree: scipy cKDTree` over `nodes`, `start_idx=0`, `goal_idx=1`, `scale: float` (= D meters).
  - `build_graph(preprocessed) -> Graph` — deterministic per scenario.
  - Module constants `EDGE_CHORD = 0.0`, `EDGE_ARC = 1.0`, `KNN_FILL_K = 6`.

**Design notes the implementer must keep (from spec §4.1 + review):**
- Chord-type edges (bitangent chords, start/goal tangent legs, start↔goal direct, polygon boundary segments, kNN fill) are admitted only when the planner's `_check_collision` says clear.
- ARC edges are added **without** a collision check: the chord between two same-circle boundary nodes almost always dips inside its own circle (at r≈20 km any chord subtending > ~0.02 rad does), so `_check_collision` would reject virtually all of them. The arc edge is a message-passing conduit carrying the true arc length; the planner's own arc-hop machinery re-validates geometry at search time.
- kNN fill edges give local connectivity so message passing works even where tangent structure is sparse; only clear ones are kept.

- [ ] **Step 1: Write the failing tests**

Append to `ml_planner/tests/graph_test.py`:

```python
import core.preprocessing as prep
from ml_planner.graph import build_graph, EDGE_CHORD, EDGE_ARC


def _circle_scenario():
    circles = [((250_000.0, 250_000.0), 20_000.0), ((150_000.0, 300_000.0), 15_000.0)]
    return {
        'start': (20_000.0, 250_000.0), 'start_heading': 0.0,
        'goal': (480_000.0, 250_000.0), 'goal_heading': 0.0,
        'islands': [], 'dynamic_obstacles': list(circles),
        'obstacles': [{'type': 'circle', 'center': c, 'radius': r} for c, r in circles],
    }


def _seg_center_dist(p, q, c):
    px, py = p; qx, qy = q; cx, cy = c
    sx, sy = qx - px, qy - py
    dd = sx * sx + sy * sy
    if dd == 0.0:
        return math.hypot(cx - px, cy - py)
    t = max(0.0, min(1.0, ((cx - px) * sx + (cy - py) * sy) / dd))
    return math.hypot(px + t * sx - cx, py + t * sy - cy)


def test_build_graph_node_census_and_determinism():
    pre = prep.prepare_scenario(_circle_scenario())
    g1, g2 = build_graph(pre), build_graph(pre)
    assert np.array_equal(g1.nodes, g2.nodes)
    assert np.array_equal(g1.edges, g2.edges)
    assert g1.start_idx == 0 and g1.goal_idx == 1
    # 2 disjoint circles: start(2/circle=4) + goal(4) tangent points
    # + 4 bitangents x 2 touch points = 8  ->  2 + 4 + 4 + 8 = 18 nodes.
    assert len(g1.nodes) == 18
    assert g1.node_feat.shape == (18, 7)
    assert g1.edge_feat.shape[1] == 2


def test_chord_edges_clear_of_inflated_circles():
    pre = prep.prepare_scenario(_circle_scenario())
    g = build_graph(pre)
    chords = g.edges[g.edge_feat[:, 1] == EDGE_CHORD]
    assert len(chords) > 0
    for a, b in chords:
        for c, r in pre['circle_obstacles']:
            assert _seg_center_dist(g.nodes[a], g.nodes[b], c) >= r - 1e-6


def test_arc_edges_connect_same_circle_nodes():
    pre = prep.prepare_scenario(_circle_scenario())
    g = build_graph(pre)
    delta = 1.0  # config.CONSTRUCTION_CLEARANCE_M
    arcs = g.edges[g.edge_feat[:, 1] == EDGE_ARC]
    assert len(arcs) > 0
    for a, b in arcs:
        on_same = False
        for c, r in pre['circle_obstacles']:
            rc = r + delta
            da = abs(math.hypot(g.nodes[a][0] - c[0], g.nodes[a][1] - c[1]) - rc)
            db = abs(math.hypot(g.nodes[b][0] - c[0], g.nodes[b][1] - c[1]) - rc)
            if da < 1e-6 and db < 1e-6:
                on_same = True
        assert on_same


def test_node_features_normalized_and_flagged():
    pre = prep.prepare_scenario(_circle_scenario())
    g = build_graph(pre)
    D = g.scale
    # feat 0: dist-to-goal/D — exactly 0 at the goal node, 1 at the start node.
    assert abs(g.node_feat[g.goal_idx, 0]) < 1e-6
    assert abs(g.node_feat[g.start_idx, 0] - 1.0) < 1e-6
    # flags
    assert g.node_feat[g.goal_idx, 4] == 1.0 and g.node_feat[g.start_idx, 5] == 1.0
    assert np.all(g.node_feat[:, 6] == 1.0)          # safezone slot, this phase
    # sin/cos are unit
    assert np.allclose(g.node_feat[:, 1] ** 2 + g.node_feat[:, 2] ** 2, 1.0, atol=1e-5)


def test_empty_map_graph_is_start_goal_edge():
    scen = {'start': (0.0, 0.0), 'start_heading': 0.0,
            'goal': (100_000.0, 0.0), 'goal_heading': 0.0,
            'islands': [], 'dynamic_obstacles': [], 'obstacles': []}
    g = build_graph(prep.prepare_scenario(scen))
    assert len(g.nodes) == 2
    assert len(g.edges) == 1 and tuple(g.edges[0]) == (0, 1)
```

- [ ] **Step 2: Run tests to verify the new ones fail**

Run: `python -m pytest ml_planner/tests/graph_test.py -v`
Expected: Task-1 tests pass; new tests FAIL with `ImportError: cannot import name 'build_graph'`.

- [ ] **Step 3: Write the implementation**

Append to `ml_planner/graph.py`:

```python
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

import core.spatial_utils as su
from core.kinodynamic_astar import KinodynamicAstar

EDGE_CHORD = 0.0
EDGE_ARC = 1.0
KNN_FILL_K = 6      # local-connectivity fill beyond the tangent structure


@dataclass
class Graph:
    nodes: np.ndarray       # (M, 2) float64 world coords
    node_feat: np.ndarray   # (M, 7) float32, spec §6
    edges: np.ndarray       # (E, 2) int32, undirected, a < b, deduplicated
    edge_feat: np.ndarray   # (E, 2) float32: [length/D, EDGE_CHORD|EDGE_ARC]
    kdtree: cKDTree
    start_idx: int
    goal_idx: int
    scale: float            # D = start-goal distance (meters)


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
        for a, b in zip(ring, ring[1:] + ring[:1]):
            phi_a = math.atan2(nodes[a][1] - c[1], nodes[a][0] - c[0])
            phi_b = math.atan2(nodes[b][1] - c[1], nodes[b][0] - c[0])
            dphi = (phi_b - phi_a) % (2.0 * math.pi)
            add_edge(a, b, EDGE_ARC, length=rc * dphi)

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
```

Note: `feat[1, 1] = 0.0 / feat[1, 2] = 1.0` — at the goal node `atan2(0, 0)` is
ill-defined; pin it so the unit-norm test invariant holds.

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest ml_planner/tests/graph_test.py -v`
Expected: all pass (Task 1 + Task 2 tests).

- [ ] **Step 5: Run the full suite to check nothing broke**

Run: `python -m pytest -q ml_planner/tests/`
Expected: no new failures vs the branch baseline.

- [ ] **Step 6: Commit**

```bash
git add ml_planner/graph.py ml_planner/tests/graph_test.py
git commit -m "feat(ml_planner): explicit tangent/bitangent graph builder for GNN guidance

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: Graph dataset builder (`graph_dataset.py`)

**Files:**
- Create: `ml_planner/graph_dataset.py`
- Test: `ml_planner/tests/graph_dataset_test.py`

**Interfaces:**
- Consumes: `dg._no_budget`, `dg._ExploringAstar`, `dg.backward_costs` (from `ml_planner.dataset_gen`); `build_graph` (Task 2); `hard_scenario` (from `ml_planner.build_dataset`).
- Produces:
  - `SNAP_RADIUS_M = 2.0 * config.STATE_POS_QUANTUM` (module constant, 2000 m).
  - `snap_labels(graph, costs, key2wp) -> (label (M,) float32 meters, mask (M,) float32)`.
  - `generate_graph_sample(scenario, max_explore=6000) -> dict | None` with keys `node_feat, edges, edge_feat, label, mask, scale` (per-graph arrays, edges use LOCAL indices).
  - `write_shard(path, samples)` / `load_shards(data_dir, pattern='graph_dataset_*.npz') -> list[dict]` — round-trip exact.
  - CLI: `python -m ml_planner.graph_dataset [START] [NSEEDS] [NPROC] [TARGET]` writing `ml_planner/data/graph_dataset_XXX.npz` shards.

- [ ] **Step 1: Write the failing tests**

Create `ml_planner/tests/graph_dataset_test.py`:

```python
import math

import numpy as np

import core.preprocessing as prep
from ml_planner.graph import build_graph
from ml_planner.graph_dataset import (SNAP_RADIUS_M, snap_labels,
                                      generate_graph_sample, write_shard,
                                      load_shards)


def _empty_scenario():
    return {'start': (0.0, 0.0), 'start_heading': 0.0,
            'goal': (100_000.0, 0.0), 'goal_heading': 0.0,
            'islands': [], 'dynamic_obstacles': [], 'obstacles': []}


def test_snap_labels_nearest_and_masking():
    g = build_graph(prep.prepare_scenario(_empty_scenario()))   # 2 nodes
    near_goal = (100_000.0 - 500.0, 0.0)        # 500 m from the goal node
    far_away = (50_000.0, 90_000.0)             # > SNAP_RADIUS_M from both
    key2wp = {('a',): near_goal, ('b',): far_away}
    costs = {('a',): 10_000.0, ('b',): 1.0}
    label, mask = snap_labels(g, costs, key2wp)
    assert mask[g.goal_idx] == 1.0
    assert abs(label[g.goal_idx] - (10_000.0 + 500.0)) < 1e-6
    assert mask[g.start_idx] == 0.0             # nothing within snap radius


def test_snap_labels_takes_min_over_candidates():
    g = build_graph(prep.prepare_scenario(_empty_scenario()))
    key2wp = {('a',): (100_000.0 - 500.0, 0.0), ('b',): (100_000.0 - 400.0, 0.0)}
    costs = {('a',): 10_000.0, ('b',): 20_000.0}
    label, mask = snap_labels(g, costs, key2wp)
    assert abs(label[g.goal_idx] - 10_500.0) < 1e-6     # min, not last


def test_generate_graph_sample_on_trivial_map():
    sample = generate_graph_sample(_empty_scenario())
    if sample is None:          # oracle may legitimately fail on a degenerate map
        return
    m = sample['node_feat'].shape[0]
    assert sample['label'].shape == (m,) and sample['mask'].shape == (m,)
    assert sample['edges'].dtype == np.int32
    assert float(sample['scale']) > 0


def test_shard_roundtrip(tmp_path):
    rng = np.random.default_rng(0)
    samples = []
    for m, e in ((5, 4), (3, 2)):
        samples.append(dict(
            node_feat=rng.normal(size=(m, 7)).astype(np.float32),
            edges=rng.integers(0, m, size=(e, 2)).astype(np.int32),
            edge_feat=rng.normal(size=(e, 2)).astype(np.float32),
            label=rng.normal(size=m).astype(np.float32),
            mask=(rng.random(m) > 0.5).astype(np.float32),
            scale=np.float64(123.0)))
    path = str(tmp_path / "graph_dataset_000.npz")
    write_shard(path, samples)
    loaded = load_shards(str(tmp_path))
    assert len(loaded) == 2
    for orig, back in zip(samples, loaded):
        for k in ('node_feat', 'edges', 'edge_feat', 'label', 'mask'):
            assert np.array_equal(orig[k], back[k]), k
        assert float(orig['scale']) == float(back['scale'])
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ml_planner/tests/graph_dataset_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner.graph_dataset'`.

- [ ] **Step 3: Write the implementation**

Create `ml_planner/graph_dataset.py`:

```python
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
    """Read every shard back into a list of per-graph dicts."""
    out = []
    for path in sorted(glob.glob(os.path.join(data_dir, pattern))):
        z = np.load(path)
        no, eo = z['node_offsets'], z['edge_offsets']
        for i in range(len(no) - 1):
            out.append(dict(
                node_feat=z['node_feat'][no[i]:no[i + 1]],
                edges=z['edges'][eo[i]:eo[i + 1]],
                edge_feat=z['edge_feat'][eo[i]:eo[i + 1]],
                label=z['label'][no[i]:no[i + 1]],
                mask=z['mask'][no[i]:no[i + 1]],
                scale=np.float64(z['scale'][i])))
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest ml_planner/tests/graph_dataset_test.py -v`
Expected: 4 passed (the trivial-map oracle test may return early — still a pass).

- [ ] **Step 5: Smoke the CLI on 4 seeds (single process)**

Run: `python -m ml_planner.graph_dataset 5000 4 1 2`
Expected: prints `GRAPH dataset: seeds 5000..5003 ...`, solves ≥1 graph within a few minutes, writes `ml_planner/data/graph_dataset_000.npz`. Delete the smoke shard afterwards: `rm ml_planner/data/graph_dataset_000.npz`.

- [ ] **Step 6: Commit**

```bash
git add ml_planner/graph_dataset.py ml_planner/tests/graph_dataset_test.py
git commit -m "feat(ml_planner): parallel graph dataset builder with snapped cost-to-go labels

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: GPU trainer (`train/train_graph.py`)

**Files:**
- Create: `ml_planner/train/train_graph.py`
- Create (if absent): `ml_planner/train/__init__.py` (empty — makes `from ml_planner.train.train_graph import MPNN` importable in tests)
- Test: `ml_planner/tests/train_graph_test.py`

**Interfaces:**
- Consumes: `load_shards` (Task 3).
- Produces:
  - `MPNN(node_dim=7, edge_dim=2, hidden=64, rounds=4)` torch module; `forward(x, edge_index, edge_attr) -> r (M,)` where `edge_index` is a `(2, 2E)` LongTensor already containing BOTH directions and `r ≥ 0` (softplus). Submodule names (weight-file contract, consumed verbatim by Task 5): `enc` (Linear), `msg` (Sequential Linear-ReLU-Linear), `upd` (GRUCell), `dec` (Sequential Linear-ReLU-Linear). state_dict keys: `enc.weight, enc.bias, msg.0.weight, msg.0.bias, msg.2.weight, msg.2.bias, upd.weight_ih, upd.weight_hh, upd.bias_ih, upd.bias_hh, dec.0.weight, dec.0.bias, dec.2.weight, dec.2.bias`.
  - Weight file: `np.savez(out, __meta__=np.array([hidden, rounds, node_dim, edge_dim], dtype=np.int64), **state_dict_as_numpy)`.
  - Target definition (shared with Task 5's semantics): `r_target = clip(label/D − node_feat[:,0], min=0)`; loss = masked Huber(`r_pred`, `r_target`).
  - CLI: `python ml_planner/train/train_graph.py --data-dir ml_planner/data --out ml_planner/models/graph_guidance.npz [--epochs 150] [--hidden 64] [--rounds 4] [--lr 1e-3] [--val-frac 0.1] [--device auto]`.

- [ ] **Step 1: Write the failing test**

Create `ml_planner/tests/train_graph_test.py`:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/train_graph_test.py -v`
Expected: FAIL with ModuleNotFoundError (or the whole file SKIPS if torch is not installed locally — in that case proceed; CI-less repo relies on the golden test in Task 5 run on the training machine).

- [ ] **Step 3: Write the implementation**

Create empty `ml_planner/train/__init__.py` if the file does not exist, then create `ml_planner/train/train_graph.py`:

```python
"""Standalone GPU trainer for the tangent-graph GNN guidance (spec §6).

Consumes graph_dataset_*.npz shards (ml_planner.graph_dataset), trains a small
message-passing net that predicts the residual-over-Euclid cost-to-go per
node, and saves weights + meta to a plain .npz consumed by the numpy
inference in ml_planner/graph_guidance.py.

  python ml_planner/train/train_graph.py \
      --data-dir ml_planner/data --out ml_planner/models/graph_guidance.npz
"""
import argparse
import os
import sys

import numpy as np
import torch
import torch.nn as nn

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from ml_planner.graph_dataset import load_shards          # noqa: E402


class MPNN(nn.Module):
    def __init__(self, node_dim=7, edge_dim=2, hidden=64, rounds=4):
        super().__init__()
        self.rounds = rounds
        self.enc = nn.Linear(node_dim, hidden)
        self.msg = nn.Sequential(nn.Linear(2 * hidden + edge_dim, hidden),
                                 nn.ReLU(), nn.Linear(hidden, hidden))
        self.upd = nn.GRUCell(hidden, hidden)
        self.dec = nn.Sequential(nn.Linear(hidden, hidden),
                                 nn.ReLU(), nn.Linear(hidden, 1))

    def forward(self, x, edge_index, edge_attr):
        """x (M,node_dim); edge_index (2,2E) BOTH directions; edge_attr (2E,edge_dim)."""
        h = torch.relu(self.enc(x))
        src, dst = edge_index[0], edge_index[1]
        for _ in range(self.rounds):
            m = self.msg(torch.cat([h[src], h[dst], edge_attr], dim=1))
            agg = torch.zeros_like(h).index_add_(0, dst, m)
            h = self.upd(agg, h)
        return nn.functional.softplus(self.dec(h).squeeze(-1))


def _to_batch(graphs, device):
    """Concatenate graphs; offset LOCAL edge indices; duplicate edges both ways."""
    xs, eis, eas, tgt, msk = [], [], [], [], []
    off = 0
    for g in graphs:
        m = g['node_feat'].shape[0]
        xs.append(torch.tensor(g['node_feat']))
        e = g['edges'].astype(np.int64) + off
        ei = np.concatenate([e.T, e.T[::-1]], axis=1)
        eis.append(torch.tensor(ei, dtype=torch.long))
        ea = np.concatenate([g['edge_feat'], g['edge_feat']], axis=0)
        eas.append(torch.tensor(ea))
        d = float(g['scale'])
        r_t = np.clip(g['label'] / d - g['node_feat'][:, 0], 0.0, None)
        tgt.append(torch.tensor(r_t.astype(np.float32)))
        msk.append(torch.tensor(g['mask']))
        off += m
    return (torch.cat(xs).to(device),
            torch.cat(eis, dim=1).to(device),
            torch.cat(eas).to(device),
            torch.cat(tgt).to(device),
            torch.cat(msk).to(device))


def save_weights(model, out, hidden, rounds, node_dim=7, edge_dim=2):
    arrays = {k: v.detach().cpu().numpy() for k, v in model.state_dict().items()}
    np.savez(out, __meta__=np.asarray([hidden, rounds, node_dim, edge_dim],
                                      dtype=np.int64), **arrays)


def train(graphs, out, epochs=150, hidden=64, rounds=4, lr=1e-3,
          batch_graphs=32, val_frac=0.1, device='auto', seed=0):
    """Returns (first_epoch_train_loss, last_epoch_train_loss)."""
    if device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(graphs))
    n_val = int(len(graphs) * val_frac)
    val_idx, tr_idx = idx[:n_val], idx[n_val:]
    model = MPNN(hidden=hidden, rounds=rounds).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    huber = nn.HuberLoss(reduction='none')
    first = last = None
    for ep in range(epochs):
        model.train()
        rng.shuffle(tr_idx)
        losses = []
        for b0 in range(0, len(tr_idx), batch_graphs):
            batch = [graphs[i] for i in tr_idx[b0:b0 + batch_graphs]]
            x, ei, ea, tgt, msk = _to_batch(batch, device)
            r = model(x, ei, ea)
            loss = (huber(r, tgt) * msk).sum() / msk.sum().clamp(min=1.0)
            opt.zero_grad()
            loss.backward()
            opt.step()
            losses.append(float(loss))
        tr_loss = float(np.mean(losses))
        first = tr_loss if first is None else first
        last = tr_loss
        if val_idx.size and (ep % 10 == 0 or ep == epochs - 1):
            model.eval()
            with torch.no_grad():
                x, ei, ea, tgt, msk = _to_batch([graphs[i] for i in val_idx], device)
                r = model(x, ei, ea)
                v = float((huber(r, tgt) * msk).sum() / msk.sum().clamp(min=1.0))
            print(f"epoch {ep:4d}  train {tr_loss:.5f}  val {v:.5f}", flush=True)
        elif ep % 10 == 0:
            print(f"epoch {ep:4d}  train {tr_loss:.5f}", flush=True)
    save_weights(model, out, hidden, rounds)
    print(f"saved -> {out}", flush=True)
    return first, last


def main():
    ap = argparse.ArgumentParser(description="Train the tangent-graph GNN guidance.")
    ap.add_argument('--data-dir', default=os.path.join(
        os.path.dirname(__file__), "..", "data"))
    ap.add_argument('--out', default=os.path.join(
        os.path.dirname(__file__), "..", "models", "graph_guidance.npz"))
    ap.add_argument('--epochs', type=int, default=150)
    ap.add_argument('--hidden', type=int, default=64)
    ap.add_argument('--rounds', type=int, default=4)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--val-frac', type=float, default=0.1)
    ap.add_argument('--device', default='auto')
    args = ap.parse_args()
    graphs = load_shards(args.data_dir)
    if not graphs:
        raise SystemExit(f"no graph_dataset_*.npz shards in {args.data_dir}")
    print(f"{len(graphs)} graphs loaded", flush=True)
    train(graphs, args.out, epochs=args.epochs, hidden=args.hidden,
          rounds=args.rounds, lr=args.lr, val_frac=args.val_frac,
          device=args.device)


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run tests to verify they pass (or skip cleanly without torch)**

Run: `python -m pytest ml_planner/tests/train_graph_test.py -v`
Expected: 2 passed if torch is installed; otherwise 2 skipped with reason `torch`.

- [ ] **Step 5: Commit**

```bash
git add ml_planner/train/train_graph.py ml_planner/train/__init__.py ml_planner/tests/train_graph_test.py
git commit -m "feat(ml_planner): standalone MPNN trainer for tangent-graph guidance

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 5: Numpy inference + secondary (`graph_guidance.py`)

**Files:**
- Create: `ml_planner/graph_guidance.py`
- Modify: `ml_planner/config.py` (add `GRAPH_MODEL_PATH`)
- Test: `ml_planner/tests/graph_guidance_test.py`

**Interfaces:**
- Consumes: `build_graph` (Task 2); weight-file contract incl. state_dict key names and `__meta__` layout (Task 4); `MPNN` (Task 4, golden test only).
- Produces:
  - `ml_planner.config.GRAPH_MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "graph_guidance.npz")`.
  - `mpnn_forward(weights: dict[str, np.ndarray], node_feat, edges, edge_feat, rounds) -> r (M,) float64` — numpy mirror of `MPNN.forward` (edges here are the UNDIRECTED `(E,2)` graph edges; the function doubles them internally).
  - `class GraphGuidance`: `__init__(model_path=mlcfg.GRAPH_MODEL_PATH)`, `.available`, `.build_field(preprocessed)` (sets `.graph`, `.values (M,) meters`), `.lookup(waypoint) -> float`.
  - `make_graph_secondary(preprocessed, model_path=None, guidance_obj=None) -> (callable|None, bool)` — the exact contract of `guidance.make_guidance_secondary`.

- [ ] **Step 1: Write the failing tests**

Create `ml_planner/tests/graph_guidance_test.py`:

```python
import math

import numpy as np
import pytest

import core.preprocessing as prep
import ml_planner.config as mlcfg
from ml_planner.graph import build_graph
from ml_planner.graph_guidance import (GraphGuidance, make_graph_secondary,
                                       mpnn_forward)


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
    return arrays


def test_unavailable_without_model(tmp_path):
    gg = GraphGuidance(model_path=str(tmp_path / "missing.npz"))
    assert gg.available is False
    cb, ok = make_graph_secondary(prep.prepare_scenario(_scenario()),
                                  model_path=str(tmp_path / "missing.npz"))
    assert cb is None and ok is False


def test_values_lower_bounded_by_euclid(tmp_path):
    path = str(tmp_path / "w.npz")
    _random_weights(path)
    gg = GraphGuidance(model_path=path)
    assert gg.available
    pre = prep.prepare_scenario(_scenario())
    gg.build_field(pre)
    goal = np.asarray(pre['goal_pos'])
    euclid = np.hypot(gg.graph.nodes[:, 0] - goal[0], gg.graph.nodes[:, 1] - goal[1])
    assert np.all(gg.values >= euclid - 1e-3)


def test_lookup_blends_distance_and_value(tmp_path):
    path = str(tmp_path / "w.npz")
    _random_weights(path)
    gg = GraphGuidance(model_path=path)
    pre = prep.prepare_scenario(_scenario())
    gg.build_field(pre)
    wp = (100_000.0, 260_000.0)
    v = gg.lookup(wp)
    d, idx = gg.graph.kdtree.query(wp, k=min(3, len(gg.graph.nodes)))
    expect = float(np.min(np.atleast_1d(d) + gg.values[np.atleast_1d(idx)]))
    assert abs(v - expect) < 1e-9


def test_secondary_callable_contract(tmp_path):
    path = str(tmp_path / "w.npz")
    _random_weights(path)
    pre = prep.prepare_scenario(_scenario())
    cb, ok = make_graph_secondary(pre, model_path=path)
    assert ok is True

    class _FakeState:
        waypoint = (60_000.0, 250_000.0)
    assert cb(_FakeState()) > 0.0


def test_numpy_matches_torch_golden():
    torch = pytest.importorskip('torch')
    from ml_planner.train.train_graph import MPNN
    torch.manual_seed(0)
    model = MPNN(node_dim=7, edge_dim=2, hidden=8, rounds=2).double()
    rng = np.random.default_rng(1)
    m, e = 6, 7
    node_feat = rng.normal(size=(m, 7))
    edges = rng.integers(0, m, size=(e, 2)).astype(np.int32)
    edge_feat = rng.random(size=(e, 2))
    ei = torch.tensor(np.concatenate([edges.T.astype(np.int64),
                                      edges.T[::-1].astype(np.int64)], axis=1))
    ea = torch.tensor(np.concatenate([edge_feat, edge_feat], axis=0))
    with torch.no_grad():
        r_torch = model(torch.tensor(node_feat), ei, ea).numpy()
    weights = {k: v.detach().numpy() for k, v in model.state_dict().items()}
    r_np = mpnn_forward(weights, node_feat, edges, edge_feat, rounds=2)
    assert np.allclose(r_np, r_torch, atol=1e-10)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ml_planner/tests/graph_guidance_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner.graph_guidance'`.

- [ ] **Step 3: Add the config constant**

In `ml_planner/config.py`, append after the `MODEL_PATH` line:

```python
# Tangent-graph GNN guidance weights (numpy .npz, produced by
# ml_planner/train/train_graph.py). Missing file => planner falls back to the
# hand-crafted secondary heuristic, exactly like the CNN model above.
GRAPH_MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "graph_guidance.npz")
```

- [ ] **Step 4: Write the implementation**

Create `ml_planner/graph_guidance.py`:

```python
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `python -m pytest ml_planner/tests/graph_guidance_test.py -v`
Expected: 5 passed + golden test passed (or skipped without torch).

- [ ] **Step 6: Run the full ml_planner suite**

Run: `python -m pytest -q ml_planner/tests/`
Expected: no new failures.

- [ ] **Step 7: Commit**

```bash
git add ml_planner/graph_guidance.py ml_planner/config.py ml_planner/tests/graph_guidance_test.py
git commit -m "feat(ml_planner): numpy MPNN inference + graph-guidance secondary (CNN-contract parity)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 6: Benchmark integration + offline gate + EVAL.md

**Files:**
- Modify: `ml_planner/benchmark.py`
- Modify: `ml_planner/EVAL.md` (append GNN section)
- Test: `ml_planner/tests/benchmark_test.py` (append; read the existing file first and follow its scenario/monkeypatch conventions)

**Interfaces:**
- Consumes: `GraphGuidance`, `make_graph_secondary` semantics (Task 5); `generate_graph_sample` (Task 3, for the offline gate).
- Produces:
  - `compare_one(scen_func, seed, difficulty, guidance, eps, graph_guidance=None)` — new keyword arg; row gains `gnn_success, gnn_iters, gnn_time, gnn_mission, gnn_flight, gnn_cost_ratio, gnn_flight_ratio, gnn_bound_ok, gnn_beats_hand_iters` (CSV_COLUMNS extended in the same order style).
  - `offline_graph_eval(gg, n, hard_start) -> list[dict(seed, n_labeled, spearman)]`.
  - `_verdict` prints an additional GNN acceptance block implementing spec §2 exactly (see Step 3 code).
  - CLI flags: `--gnn-offline-n` (default 10).

- [ ] **Step 1: Write the failing tests**

Read `ml_planner/tests/benchmark_test.py` first; append tests in its style. The two tests below are self-contained additions (adapt imports/fixtures only if the existing file already provides equivalents):

```python
import numpy as np

import ml_planner.benchmark as bm
from batch_random_test import generate_random_scenario


def test_compare_one_emits_gnn_columns_without_model():
    row = bm.compare_one(generate_random_scenario, 7003, 'easy',
                         guidance=None, eps=0.05, graph_guidance=None)
    for col in ('gnn_success', 'gnn_iters', 'gnn_time', 'gnn_mission',
                'gnn_flight', 'gnn_cost_ratio', 'gnn_bound_ok',
                'gnn_beats_hand_iters'):
        assert col in row
    assert all(c in bm.CSV_COLUMNS for c in
               ('gnn_success', 'gnn_iters', 'gnn_time', 'gnn_cost_ratio'))


def test_gnn_acceptance_logic():
    # speed win + quality not-worse => PASS
    assert bm.gnn_acceptance(it_g=100, it_h=200, t_g=1.0, t_h=2.0,
                             cost_g=1.010, cost_h=1.009) is True
    # quality win + time within 5% => PASS
    assert bm.gnn_acceptance(it_g=250, it_h=200, t_g=2.09, t_h=2.0,
                             cost_g=1.001, cost_h=1.009) is True
    # no win on either axis => FAIL
    assert bm.gnn_acceptance(it_g=250, it_h=200, t_g=2.5, t_h=2.0,
                             cost_g=1.010, cost_h=1.009) is False
    # quality win but time blown past 5% => FAIL
    assert bm.gnn_acceptance(it_g=250, it_h=200, t_g=2.5, t_h=2.0,
                             cost_g=1.001, cost_h=1.009) is False
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ml_planner/tests/benchmark_test.py -v`
Expected: existing tests pass; new ones FAIL (`TypeError: compare_one() got an unexpected keyword argument` / `AttributeError: gnn_acceptance`).

- [ ] **Step 3: Implement the benchmark extension**

In `ml_planner/benchmark.py`:

3a. Imports — add:

```python
from ml_planner.graph_guidance import GraphGuidance
```

3b. Add `_gnn_plan` next to `_guided_plan`:

```python
def _gnn_plan(scen, graph_guidance, eps):
    """Focal plan with the GNN per-node field as secondary; time INCLUDES
    build_field (graph build + numpy forward). Falls back to hand-crafted."""
    pre = prep.prepare_scenario(scen)
    sec = None
    if graph_guidance is not None and graph_guidance.available:
        try:
            graph_guidance.build_field(pre)
            sec = lambda st: graph_guidance.lookup(st.waypoint)
        except Exception:
            sec = None
    return plan_trajectory_focal(pre, focal_eps=eps, secondary=sec)
```

3c. Extend `compare_one` — new signature `def compare_one(scen_func, seed, difficulty, guidance, eps, graph_guidance=None):`; after the `rg, tg = _safe(...)` line add:

```python
    rn, tn = _safe(lambda: _gnn_plan(scen, graph_guidance, eps))
```

then extend the row construction so every place that iterates `(('base', rb), ('hand', rh), ('guided', rg))` also covers `('gnn', rn)`, and mirror the guided-only fields:

```python
    row['gnn_success'] = ok(rn)
    row['gnn_iters'] = rn['stats']['iterations'] if ok(rn) else ''
    row['gnn_time'] = round(tn, 3)
```

inside the base-success block add:

```python
        row['gnn_cost_ratio'] = round(row['gnn_mission'] / bm, 4) if ok(rn) else ''
        row['gnn_flight_ratio'] = round(row['gnn_flight'] / bf, 4) if ok(rn) and bf else ''
        row['gnn_bound_ok'] = (row['gnn_mission'] <= 1.05 * bm + 1e-6) if ok(rn) else ''
```

(and add the three column names to the else-branch blank-fill list), and at the end:

```python
    row['gnn_beats_hand_iters'] = (ok(rn) and ok(rh)
                                   and rn['stats']['iterations'] < rh['stats']['iterations'])
```

3d. Extend `CSV_COLUMNS` with, in order:
`'gnn_success'` (after `guided_success`), `'gnn_iters'` (after `guided_iters`), `'gnn_time'` (after `guided_time`), `'gnn_mission'`, `'gnn_flight'` (after the mission/flight groups), `'gnn_cost_ratio'`, `'gnn_flight_ratio'`, `'gnn_bound_ok'`, `'gnn_beats_hand_iters'` (at the end).

3e. Extend `planner_benchmark` signature to `planner_benchmark(guidance, graph_guidance=None, easy_seeds=(), hard_seeds=(), eps=EPS)` and pass `graph_guidance=graph_guidance` through both `compare_one` calls.

3f. Extend `_summ`: require `gnn_success` too in the all-solved filter; print a `gnn` line mirroring the guided lines; return the extra totals:

```python
    sub = [r for r in rows if r['difficulty'] == diff
           and r['base_success'] and r['hand_success'] and r['guided_success']
           and r['gnn_success']]
    ...
    it_n, t_n = sum(col('gnn_iters')), sum(col('gnn_time'))
    print(f"  iterations   base={it_b}  hand={it_h}  guided={it_g}  gnn={it_n}")
    print(f"  wall time(s) base={t_b:.1f}  hand={t_h:.1f}  guided={t_g:.1f}  gnn={t_n:.1f}")
    print(f"  mission-cost ratio vs base:  hand={statistics.mean(col('hand_cost_ratio')):.4f}  "
          f"guided={statistics.mean(col('guided_cost_ratio')):.4f}  "
          f"gnn={statistics.mean(col('gnn_cost_ratio')):.4f}")
    wins_gnn = sum(1 for r in sub if r['gnn_beats_hand_iters'])
    print(f"  gnn beats hand (fewer iters): {wins_gnn}/{len(sub)}")
    return dict(it_h=it_h, it_g=it_g, t_h=t_h, t_g=t_g, wins=wins, n=len(sub),
                it_n=it_n, t_n=t_n,
                cost_h=statistics.mean(col('hand_cost_ratio')),
                cost_n=statistics.mean(col('gnn_cost_ratio')))
```

(keep every existing print line; only additions shown).

3g. Add the acceptance function + extend `_verdict` (spec §2, copied thresholds):

```python
def gnn_acceptance(it_g, it_h, t_g, t_h, cost_g, cost_h):
    """Spec 2026-07-19 §2: win at least one axis, don't lose the other.
    speed win  = fewer total iterations AND less net wall-time than hand;
    quality win = lower mean cost-ratio than hand;
    not-worse: wall-time within +5% of hand / cost-ratio within +0.002."""
    speed_win = (it_g < it_h) and (t_g < t_h)
    quality_win = cost_g < cost_h
    time_ok = t_g <= 1.05 * t_h
    cost_ok = cost_g <= cost_h + 0.002
    return (speed_win and cost_ok) or (quality_win and time_ok)
```

and at the end of `_verdict(hard)` append:

```python
    if hard and hard['n'] and 'it_n' in hard:
        ok = gnn_acceptance(hard['it_n'], hard['it_h'], hard['t_n'], hard['t_h'],
                            hard['cost_n'], hard['cost_h'])
        print("\n=== GNN PROTOTYPE ACCEPTANCE (hard maps, vs hand-crafted) ===")
        print(f"  iters {hard['it_n']} vs {hard['it_h']} | time {hard['t_n']:.1f}s "
              f"vs {hard['t_h']:.1f}s | cost {hard['cost_n']:.4f} vs {hard['cost_h']:.4f}")
        print("  ✅ PASS — GNN wins at least one axis without losing the other."
              if ok else
              "  ❌ FAIL — GNN neither wins an axis nor holds the other (spec §2).")
```

3h. Add the offline gate + wire `main()`:

```python
def offline_graph_eval(gg, n, hard_start):
    """Spearman of predicted node values vs snapped oracle labels on held-out
    hard seeds (spec §2 early-stop gate: mean >= 0.8)."""
    from scipy.stats import spearmanr
    from ml_planner.graph_dataset import generate_graph_sample
    rows = []
    for k in range(n):
        seed = hard_start + k
        sample = generate_graph_sample(hard_scenario(seed))
        if sample is None:
            continue
        try:
            gg.build_field(prep.prepare_scenario(hard_scenario(seed)))
        except Exception:
            continue
        m = sample['mask'] > 0
        if m.sum() < 5:
            continue
        rho = spearmanr(gg.values[m], sample['label'][m]).correlation
        rows.append(dict(seed=seed, n_labeled=int(m.sum()),
                         spearman=float(rho) if rho == rho else 0.0))
    return rows
```

In `main()`: add `ap.add_argument('--gnn-offline-n', type=int, default=10)`; after the CNN `Guidance()` block add:

```python
    gg = GraphGuidance()
    print(f"graph model:   {'AVAILABLE' if gg.available else 'NOT FOUND'} "
          f"({gg.model_path})")
    if gg.available and args.gnn_offline_n > 0:
        with _capped_oracle(args.budget):
            goff = offline_graph_eval(gg, args.gnn_offline_n, args.hard_start)
        if goff:
            import statistics as st
            print(f"\n=== OFFLINE GNN accuracy ({len(goff)} hard held-out) ===")
            print(f"  Spearman: mean {st.mean(r['spearman'] for r in goff):.3f}  "
                  f"min {min(r['spearman'] for r in goff):.3f}  (gate: mean >= 0.8)")
```

and pass `graph_guidance=gg` into `planner_benchmark(...)`.

- [ ] **Step 4: Run the tests**

Run: `python -m pytest ml_planner/tests/benchmark_test.py ml_planner/tests/ -q`
Expected: all pass, including the two new tests, no regressions.

- [ ] **Step 5: Benchmark smoke without a model**

Run: `python -m ml_planner.benchmark --offline-n 0 --gnn-offline-n 0 --bench-n 2`
Expected: prints `graph model:   NOT FOUND (...)`, runs 4 scenarios, gnn columns equal hand behavior (clean fallback), CSV written with the new columns.

- [ ] **Step 6: Append the GNN section to `ml_planner/EVAL.md`**

Append at the end of the file:

```markdown
## GNN tangent-graph guidance (prototype)

The GNN secondary runs on an explicit tangent/bitangent graph (numpy MPNN,
ms-scale build+forward) — see docs/superpowers/specs/2026-07-19-gnn-guidance-design.md.

```bash
# 1) Build the graph dataset (parallel oracle solves; ~same cost as the CNN one)
python -m ml_planner.graph_dataset 0 2400 6 2000

# 2) Train on any GPU machine (torch only needed here)
python ml_planner/train/train_graph.py \
    --data-dir ml_planner/data --out ml_planner/models/graph_guidance.npz

# 3) Evaluate: offline Spearman gate (>= 0.8) + 4-way end-to-end + acceptance
python -m ml_planner.benchmark --offline-n 10 --gnn-offline-n 10 --bench-n 30
```

Acceptance (spec §2): on hard held-out maps vs hand-crafted, the GNN must win
speed (total iterations AND net wall-time) or quality (mean cost-ratio), and
must not lose the other axis (time within +5%, cost-ratio within +0.002).
With no `models/graph_guidance.npz` the gnn columns fall back to hand-crafted.
```

- [ ] **Step 7: Commit**

```bash
git add ml_planner/benchmark.py ml_planner/EVAL.md ml_planner/tests/benchmark_test.py
git commit -m "feat(ml_planner): 4-way benchmark (base/hand/cnn/gnn) with GNN acceptance verdict

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 7: Data build, training, and the go/no-go run (manual/off-machine)

No new code. Execute and record results.

- [ ] **Step 1: Build the dataset (this machine, hours-scale, parallel)**

```bash
python -m ml_planner.graph_dataset 0 2400 6 2000
```

Expected: `ml_planner/data/graph_dataset_*.npz` shards totaling ~2000 graphs. Training seeds `0..2399` stay disjoint from the benchmark's held-out ranges (offline `5000+`, end-to-end hard `6000+`, easy `7000+`).

- [ ] **Step 2: Train (GPU machine, per EVAL.md)**

```bash
python ml_planner/train/train_graph.py \
    --data-dir ml_planner/data --out ml_planner/models/graph_guidance.npz \
    --epochs 150 --hidden 64 --rounds 4
```

Expected: val loss decreasing then plateauing; `graph_guidance.npz` ≈ a few hundred KB. Copy it back to `ml_planner/models/` on this machine if trained elsewhere.

- [ ] **Step 3: Offline gate**

```bash
python -m ml_planner.benchmark --offline-n 0 --gnn-offline-n 10 --bench-n 0
```

Gate: Spearman mean ≥ 0.8. Below the gate → STOP (spec early-stop); revisit features/rounds/data volume before any end-to-end run.

- [ ] **Step 4: Full go/no-go benchmark**

```bash
python -m ml_planner.benchmark --offline-n 10 --gnn-offline-n 10 --bench-n 30
```

Record the `GNN PROTOTYPE ACCEPTANCE` verdict and `ml_planner/data/benchmark_results.csv`. PASS → commit the model (`git add -f ml_planner/models/graph_guidance.npz` if ignored) and plan Phase 2 (edge-scoring, Approach B). FAIL → keep hand-crafted default; record findings in EVAL.md.

---

## Self-Review (performed while writing)

- **Spec coverage:** §4.1 graph → Tasks 1–2; §5 dataset/labels → Task 3; §6 features/model/trainer → Tasks 2, 4; §5 planning flow + §7 fallbacks → Task 5; §2 criteria + benchmark + §8 tests → Task 6 (+ per-task tests); execution → Task 7. Arc-edge no-collision-check deviation from a literal reading of §4.1 is documented in Task 2 with rationale.
- **Placeholder scan:** none — every code step is complete.
- **Type consistency:** weight-file keys (Task 4 ↔ Task 5) match `nn.Linear`/`nn.GRUCell` state_dict names; `edges (E,2) int32` local indices used consistently (doubling happens in `_to_batch`/`mpnn_forward`); `compare_one(..., graph_guidance=None)` matches Task 6 tests; `gnn_acceptance` thresholds match spec §2.
