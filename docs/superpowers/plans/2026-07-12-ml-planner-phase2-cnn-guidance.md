# ml_planner Phase 2 — CNN Guidance-map Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a learned CNN guidance-map (Neural A* style) as the focal-search secondary heuristic — with the full data-generation and ONNX-inference pipeline built and tested in-session using a stub model, so that once a real model is trained off-machine (Colab) it drops in and works.

**Architecture:** A new `raster.py` (shared crop/affine + 4-channel builder) is consumed by both `dataset_gen.py` (runs the optimal planner, records every explored edge, backward-Dijkstra → dense cost-to-go labels → `.npz`) and `guidance.py` (loads an ONNX model, runs one forward per problem to produce a cost-to-go field, O(1) bilinear lookup per state). `plan_trajectory_focal` gains a `secondary='guidance'` flag that uses the CNN when a model exists and silently falls back to the Phase-1 hand-crafted heuristic otherwise. The ε=5% bound stays enforced by the admissible Euclid OPEN, so the CNN never affects correctness or safety.

**Tech Stack:** Python 3, numpy, shapely, matplotlib (all already in repo). `onnxruntime` for inference (optional — declared in `ml_planner/requirements-ml.txt`, tests skip when absent). `torch` is used ONLY in the off-machine training notebook, never imported by the planner.

## Global Constraints

- **Do NOT modify Phase-1 or base code**, EXCEPT one permitted light edit to `ml_planner/plan.py` to wire the `secondary='guidance'` flag (spec §8). Phase-1 behavior must be **byte-identical when `secondary` is not `'guidance'`**. `core/`, `config.py`, root `tests/`, `requirements.txt` stay untouched.
- **ε bound = 5%** stays enforced by the admissible Euclid OPEN (`FOCAL_EPS = 0.05`, weight 1). The CNN is only the FOCAL secondary heuristic; it never affects the bound or the exact collision safety check.
- **CNN I/O contract (hard, identical in dataset_gen, guidance, notebook, stubs):** input `channels` shape `(1, 4, GRID_RES, GRID_RES)` float32; output `cost_to_go` shape `(1, 1, GRID_RES, GRID_RES)` float32. `GRID_RES = 256` (from `ml_planner/config.py`). ONNX opset ≥ 11.
- **4 channels, in this exact order:** (0) inflated-obstacle occupancy, (1) safezone mask (all-ones when no safezone), (2) normalized distance-to-goal, (3) start marker. The guidance field is **position-only** (no heading).
- **Infinite map:** never assume `map_bounds`; crop is the bounding box of `{start_pos, goal_pos, obstacles}` + margin, squared, scaled to `GRID_RES`.
- **No new mandatory dependency:** the planner imports `onnxruntime` lazily; when it or the model file is absent, `Guidance.available` is False and the code falls back to hand-crafted. Tests exercising the real ONNX path use `pytest.importorskip`.
- **Tests** live in `ml_planner/tests/`, run `python -m pytest ml_planner/tests -v` from repo root. Commit trailer: blank line then exactly `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`. `git add` uses explicit paths (unrelated WIP must not be committed).
- Grid indexing convention (fixed everywhere): a field/channel array is indexed `[iy, ix]` where `ix` is the grid x from world x and `iy` is the grid y from world y.

---

## File Structure

- `ml_planner/raster.py` — `Affine` (world↔grid), `compute_crop`, `build_channels`. Shared geometry-to-raster; one responsibility, no planner state.
- `ml_planner/dataset_gen.py` — `_RecordingAstar`, `backward_costs`, `rasterize_labels`, `generate_sample`, `export_dataset`. Produces training `.npz` from oracle solves.
- `ml_planner/guidance.py` — `bilinear_lookup`, `Guidance`, `make_guidance_secondary`. ONNX inference + O(1) lookup.
- `ml_planner/plan.py` — **light edit** to wire `secondary='guidance'` (fallback-safe).
- `ml_planner/models/.gitignore`, `ml_planner/models/.gitkeep` — model dir, `.onnx` ignored.
- `ml_planner/train/train_guidance.ipynb` — off-machine training scaffold (not run in-session).
- Tests: `ml_planner/tests/raster_test.py`, `dataset_gen_test.py`, `guidance_test.py`, `guidance_integration_test.py`.

---

### Task 1: `raster.py` — Affine + compute_crop

**Files:**
- Create: `ml_planner/raster.py`
- Test: `ml_planner/tests/raster_test.py`

**Interfaces:**
- Produces: `class Affine` with `world_to_grid(x, y) -> (gx, gy)` and `grid_to_world(gx, gy) -> (x, y)`, attributes `x0, y0, scale, grid_res`. `compute_crop(preprocessed, grid_res, margin_frac=0.1) -> Affine` — square crop covering `{start_pos, goal_pos, obstacles}` + margin, scaled so the crop spans `[0, grid_res)`.

- [ ] **Step 1: Write the failing test**

Create `ml_planner/tests/raster_test.py`:

```python
import core.map_generator as mg
import core.preprocessing as prep
from ml_planner.raster import compute_crop


def _prep():
    return prep.prepare_scenario(mg.scenario2_single_obstacle())


def test_crop_contains_start_goal_and_roundtrips():
    pre = _prep()
    aff = compute_crop(pre, grid_res=256)
    for pt in (pre['start_pos'], pre['goal_pos']):
        gx, gy = aff.world_to_grid(*pt)
        assert 0.0 <= gx <= 256.0 and 0.0 <= gy <= 256.0
        x, y = aff.grid_to_world(gx, gy)
        assert abs(x - pt[0]) < 1e-3 and abs(y - pt[1]) < 1e-3


def test_crop_covers_inflated_circles():
    pre = _prep()
    aff = compute_crop(pre, grid_res=256)
    for (cx, cy), r in pre['circle_obstacles']:
        for corner in ((cx - r, cy - r), (cx + r, cy + r)):
            gx, gy = aff.world_to_grid(*corner)
            assert -1.0 <= gx <= 257.0 and -1.0 <= gy <= 257.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/raster_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner.raster'`.

- [ ] **Step 3: Write minimal implementation**

Create `ml_planner/raster.py`:

```python
"""Shared crop/affine + channel rasterization for the CNN guidance map.

Used by BOTH dataset_gen (labels) and guidance (inference) so the two agree
on exactly one crop + channel definition. Grid arrays are indexed [iy, ix]
where ix is grid-x from world-x and iy is grid-y from world-y.
"""

import numpy as np


class Affine:
    """Square-crop world<->grid mapping. gx = (x-x0)*scale, gy = (y-y0)*scale."""

    def __init__(self, x0, y0, scale, grid_res):
        self.x0 = x0
        self.y0 = y0
        self.scale = scale          # grid units per world meter
        self.grid_res = grid_res

    def world_to_grid(self, x, y):
        return ((x - self.x0) * self.scale, (y - self.y0) * self.scale)

    def grid_to_world(self, gx, gy):
        return (self.x0 + gx / self.scale, self.y0 + gy / self.scale)


def compute_crop(preprocessed, grid_res, margin_frac=0.1):
    """Square crop covering start, goal, and all obstacles, plus a margin."""
    xs = [preprocessed['start_pos'][0], preprocessed['goal_pos'][0]]
    ys = [preprocessed['start_pos'][1], preprocessed['goal_pos'][1]]
    for (cx, cy), r in preprocessed['circle_obstacles']:
        xs += [cx - r, cx + r]
        ys += [cy - r, cy + r]
    for poly in preprocessed['polygon_obstacles']:
        for px, py in poly:
            xs.append(px)
            ys.append(py)
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
    side = max(xmax - xmin, ymax - ymin)
    if side <= 0.0:
        side = 1.0
    side *= (1.0 + 2.0 * margin_frac)
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    x0 = cx - 0.5 * side
    y0 = cy - 0.5 * side
    return Affine(x0, y0, grid_res / side, grid_res)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/raster_test.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add ml_planner/raster.py ml_planner/tests/raster_test.py
git commit -m "feat(ml_planner): raster Affine + compute_crop

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: `raster.py` — build_channels

**Files:**
- Modify: `ml_planner/raster.py` (add `build_channels`)
- Test: `ml_planner/tests/raster_test.py` (add channel tests)

**Interfaces:**
- Consumes: `Affine`, `compute_crop` (Task 1).
- Produces: `build_channels(preprocessed, affine, grid_res) -> np.ndarray` of shape `(4, grid_res, grid_res)` float32; channels (0) occupancy, (1) safezone, (2) normalized distance-to-goal, (3) start marker.

- [ ] **Step 1: Write the failing test**

Add to `ml_planner/tests/raster_test.py`:

```python
import numpy as np
from ml_planner.raster import build_channels


def test_channels_shape_and_occupancy():
    pre = _prep()
    aff = compute_crop(pre, grid_res=64)
    ch = build_channels(pre, aff, grid_res=64)
    assert ch.shape == (4, 64, 64)
    assert ch.dtype == np.float32
    # A cell at an inflated circle center must be marked occupied (channel 0).
    (cx, cy), r = pre['circle_obstacles'][0]
    gx, gy = aff.world_to_grid(cx, cy)
    assert ch[0, int(gy), int(gx)] == 1.0
    # No safezone in this scenario -> channel 1 is all ones.
    assert np.all(ch[1] == 1.0)
    # Distance-to-goal channel is ~0 at the goal cell.
    ggx, ggy = aff.world_to_grid(*pre['goal_pos'])
    if 0 <= int(ggy) < 64 and 0 <= int(ggx) < 64:
        assert ch[2, int(ggy), int(ggx)] < 0.05
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/raster_test.py::test_channels_shape_and_occupancy -v`
Expected: FAIL — `ImportError: cannot import name 'build_channels'`.

- [ ] **Step 3: Write minimal implementation**

Add to `ml_planner/raster.py` (top-level, after `compute_crop`):

```python
from matplotlib.path import Path as _MplPath


def _cell_centers_world(affine, grid_res):
    idx = np.arange(grid_res, dtype=np.float64) + 0.5
    gx, gy = np.meshgrid(idx, idx)          # both shape (grid_res, grid_res), [iy, ix]
    wx = affine.x0 + gx / affine.scale
    wy = affine.y0 + gy / affine.scale
    return wx, wy


def build_channels(preprocessed, affine, grid_res):
    """(4, H, W) float32: occupancy, safezone, dist-to-goal, start marker."""
    wx, wy = _cell_centers_world(affine, grid_res)
    pts = np.column_stack([wx.ravel(), wy.ravel()])

    occ = np.zeros((grid_res, grid_res), dtype=bool)
    for (cx, cy), r in preprocessed['circle_obstacles']:
        occ |= ((wx - cx) ** 2 + (wy - cy) ** 2) < r * r
    for poly in preprocessed['polygon_obstacles']:
        occ |= _MplPath(poly).contains_points(pts).reshape(grid_res, grid_res)

    safezones = preprocessed.get('safezones')
    if safezones:
        inside = np.zeros((grid_res, grid_res), dtype=bool)
        for poly in safezones:
            inside |= _MplPath(poly).contains_points(pts).reshape(grid_res, grid_res)
        safe = inside.astype(np.float32)
    else:
        safe = np.ones((grid_res, grid_res), dtype=np.float32)

    gx_goal, gy_goal = preprocessed['goal_pos']
    dist = np.sqrt((wx - gx_goal) ** 2 + (wy - gy_goal) ** 2)
    diag = np.sqrt(2.0) * grid_res / affine.scale
    dgoal = (dist / diag).astype(np.float32)

    start = np.zeros((grid_res, grid_res), dtype=np.float32)
    sgx, sgy = affine.world_to_grid(*preprocessed['start_pos'])
    si, sj = int(round(sgy)), int(round(sgx))
    if 0 <= si < grid_res and 0 <= sj < grid_res:
        start[si, sj] = 1.0

    return np.stack([occ.astype(np.float32), safe, dgoal, start], axis=0)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/raster_test.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add ml_planner/raster.py ml_planner/tests/raster_test.py
git commit -m "feat(ml_planner): raster build_channels (4-channel input)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: `dataset_gen.py` — edge recording + backward cost-to-go

**Files:**
- Create: `ml_planner/dataset_gen.py`
- Test: `ml_planner/tests/dataset_gen_test.py`

**Interfaces:**
- Consumes: `FocalKinodynamicAstar` (Phase 1), `core.spatial_utils.state_to_tuple`, `core.preprocessing`.
- Produces:
  - `class _RecordingAstar(FocalKinodynamicAstar)` — `focal_eps=0`, records `self.edges: list[(u_key, v_key, cost)]` and `self.key2wp: dict[key, (x, y)]` during search.
  - `backward_costs(edges, goal_key) -> dict[key, float]` — exact cost-to-go by Dijkstra on the reversed edge set from `goal_key`.

- [ ] **Step 1: Write the failing test**

Create `ml_planner/tests/dataset_gen_test.py`:

```python
import math

import core.map_generator as mg
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.spatial_utils as su
from ml_planner.dataset_gen import _RecordingAstar, backward_costs, _no_budget


def _mission(pre, path):
    body = sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for (a, _), (b, _) in zip(path, path[1:]))
    return math.dist(pre['start_pos'], path[0][0]) + body


def test_backward_costs_reconstruct_base_optimum():
    scen = mg.scenario2_single_obstacle()
    pre = prep.prepare_scenario(scen)
    base = astar.plan_trajectory(pre, verbose=False)
    assert base['success']
    base_mission = _mission(pre, base['path'])

    with _no_budget():
        planner = _RecordingAstar(prep.prepare_scenario(scen))
        path = planner.search()
    assert path is not None
    goal_key = su.state_to_tuple(*planner.raw_route[-1])
    costs = backward_costs(planner.edges, goal_key)
    assert costs[goal_key] == 0.0
    assert len(costs) > 1

    # Optimal mission cost = min over seeded start corners of
    # (dist(O, corner) == corner.g_cost) + cost_to_go(corner).
    best = min(
        c.g_cost + costs.get(su.state_to_tuple(c.waypoint, c.heading), float('inf'))
        for c in planner.start_corners
    )
    assert abs(best - base_mission) <= 0.02 * base_mission
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/dataset_gen_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner.dataset_gen'`.

- [ ] **Step 3: Write minimal implementation**

Create `ml_planner/dataset_gen.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/dataset_gen_test.py -v`
Expected: PASS (1 test).

- [ ] **Step 5: Commit**

```bash
git add ml_planner/dataset_gen.py ml_planner/tests/dataset_gen_test.py
git commit -m "feat(ml_planner): dataset_gen edge recording + backward cost-to-go

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: `dataset_gen.py` — rasterize labels + sample/dataset export

**Files:**
- Modify: `ml_planner/dataset_gen.py` (add `rasterize_labels`, `generate_sample`, `export_dataset`)
- Test: `ml_planner/tests/dataset_gen_test.py` (add sample/export tests)

**Interfaces:**
- Consumes: `_RecordingAstar`, `backward_costs`, `_no_budget` (Task 3); `raster.compute_crop`, `raster.build_channels` (Tasks 1–2).
- Produces:
  - `rasterize_labels(costs, key2wp, affine, grid_res) -> (label(H,W) float32, mask(H,W) float32)` — min cost-to-go per cell; mask=1 where labeled.
  - `generate_sample(scenario, grid_res=config.GRID_RES) -> dict | None` with keys `channels (4,H,W)`, `label (H,W)`, `mask (H,W)`, `affine (x0,y0,scale,grid_res)`. `None` if the scenario is unsolved.
  - `export_dataset(scenarios, out_path, grid_res=config.GRID_RES) -> int` — writes a compressed `.npz` stacking all solved samples; returns the count.

- [ ] **Step 1: Write the failing test**

Add to `ml_planner/tests/dataset_gen_test.py`:

```python
import os
import numpy as np
from ml_planner.dataset_gen import rasterize_labels, generate_sample, export_dataset


def test_generate_sample_shapes_and_mask():
    scen = mg.scenario2_single_obstacle()
    sample = generate_sample(scen, grid_res=64)
    assert sample is not None
    assert sample['channels'].shape == (4, 64, 64)
    assert sample['label'].shape == (64, 64)
    assert sample['mask'].shape == (64, 64)
    assert sample['mask'].sum() > 0                      # some cells labeled
    # Labeled cells carry finite non-negative cost-to-go.
    labeled = sample['label'][sample['mask'] > 0]
    assert np.all(np.isfinite(labeled)) and np.all(labeled >= 0.0)


def test_export_dataset_roundtrip(tmp_path):
    out = os.path.join(tmp_path, "ds.npz")
    n = export_dataset([mg.scenario1_open_ocean(), mg.scenario2_single_obstacle()],
                       out, grid_res=64)
    assert n >= 1
    data = np.load(out)
    assert data['channels'].shape == (n, 4, 64, 64)
    assert data['label'].shape == (n, 64, 64)
    assert data['mask'].shape == (n, 64, 64)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/dataset_gen_test.py -v`
Expected: FAIL — `ImportError: cannot import name 'rasterize_labels'`.

- [ ] **Step 3: Write minimal implementation**

Add to `ml_planner/dataset_gen.py` (imports at top: add `import core.preprocessing as prep` and `import ml_planner.raster as raster`):

```python
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


def generate_sample(scenario, grid_res=config.GRID_RES):
    """Run the oracle, backward-label, rasterize. None if unsolved."""
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


def export_dataset(scenarios, out_path, grid_res=config.GRID_RES):
    """Write a compressed .npz stacking every solved sample. Returns count."""
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/dataset_gen_test.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add ml_planner/dataset_gen.py ml_planner/tests/dataset_gen_test.py
git commit -m "feat(ml_planner): dataset_gen rasterize labels + npz export

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 5: `guidance.py` — ONNX inference + bilinear lookup

**Files:**
- Create: `ml_planner/guidance.py`
- Test: `ml_planner/tests/guidance_test.py`

**Interfaces:**
- Consumes: `raster.compute_crop`, `raster.build_channels` (Tasks 1–2); `ml_planner.config`.
- Produces:
  - `bilinear_lookup(field, gx, gy) -> float | None` — bilinear sample of a `(H, W)` field at grid coords `(gx, gy)` (indexed `[iy=gy, ix=gx]`); `None` if outside `[0, H-1] x [0, W-1]`.
  - `LARGE = 1e18`.
  - `class Guidance(model_path=config.MODEL_PATH, grid_res=config.GRID_RES)` with `.available`, `.build_field(preprocessed)`, `.lookup(waypoint) -> float`.
  - `make_guidance_secondary(preprocessed, model_path=None, guidance_obj=None) -> (callable | None, bool)` — builds the field once and returns `(lambda state: guidance.lookup(state.waypoint), True)`, or `(None, False)` when no model is available.

- [ ] **Step 1: Write the failing test**

Create `ml_planner/tests/guidance_test.py`:

```python
import numpy as np
import core.map_generator as mg
import core.preprocessing as prep
from ml_planner.guidance import bilinear_lookup, Guidance, make_guidance_secondary, LARGE
from ml_planner.raster import compute_crop


def test_bilinear_interpolates_and_flags_out_of_range():
    field = np.array([[0.0, 10.0], [20.0, 30.0]], dtype=np.float32)
    assert abs(bilinear_lookup(field, 0.0, 0.0) - 0.0) < 1e-6
    assert abs(bilinear_lookup(field, 1.0, 0.0) - 10.0) < 1e-6
    assert abs(bilinear_lookup(field, 0.5, 0.5) - 15.0) < 1e-6   # center = mean
    assert bilinear_lookup(field, -0.1, 0.0) is None
    assert bilinear_lookup(field, 1.6, 0.0) is None


def test_guidance_unavailable_without_model():
    g = Guidance(model_path="/nonexistent/guidance.onnx")
    assert g.available is False


def test_make_guidance_secondary_falls_back_without_model():
    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    cb, available = make_guidance_secondary(pre, model_path="/nonexistent/guidance.onnx")
    assert available is False
    assert cb is None


class _StubGuidance:
    """Guidance-shaped stub: field = distance-to-goal in grid units."""
    available = True

    def __init__(self, grid_res=64):
        self.grid_res = grid_res
        self.field = None
        self.affine = None

    def build_field(self, preprocessed):
        self.affine = compute_crop(preprocessed, self.grid_res)
        gx, gy = self.affine.world_to_grid(*preprocessed['goal_pos'])
        iy, ix = np.mgrid[0:self.grid_res, 0:self.grid_res]
        self.field = np.sqrt((ix - gx) ** 2 + (iy - gy) ** 2).astype(np.float32)

    def lookup(self, waypoint):
        from ml_planner.guidance import bilinear_lookup as bl
        gx, gy = self.affine.world_to_grid(*waypoint)
        v = bl(self.field, gx, gy)
        return LARGE if v is None else float(v)


def test_stub_guidance_secondary_builds_and_looks_up():
    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    stub = _StubGuidance()
    cb, available = make_guidance_secondary(pre, guidance_obj=stub)
    assert available is True
    # Near the goal the guidance cost is small; near the start it is larger.
    from core.kinodynamic_astar import State
    near_goal = State(pre['goal_pos'], 0.0)
    near_start = State(pre['start_pos'], 0.0)
    assert cb(near_goal) < cb(near_start)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/guidance_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner.guidance'`.

- [ ] **Step 3: Write minimal implementation**

Create `ml_planner/guidance.py`:

```python
"""CNN guidance-map inference: one ONNX forward per problem, O(1) bilinear
lookup per state. Falls back cleanly (available=False) when the model file or
onnxruntime is absent.
"""

import math
import os

import numpy as np

import ml_planner.config as mlcfg
import ml_planner.raster as raster

LARGE = 1e18


def bilinear_lookup(field, gx, gy):
    """Bilinear sample of a (H, W) field at grid coords (gx, gy), indexed
    [iy=gy, ix=gx]. None if outside the grid."""
    h, w = field.shape
    if gx < 0.0 or gy < 0.0 or gx > w - 1 or gy > h - 1:
        return None
    ix0 = int(math.floor(gx))
    iy0 = int(math.floor(gy))
    ix1 = min(ix0 + 1, w - 1)
    iy1 = min(iy0 + 1, h - 1)
    fx = gx - ix0
    fy = gy - iy0
    v00 = field[iy0, ix0]
    v01 = field[iy0, ix1]
    v10 = field[iy1, ix0]
    v11 = field[iy1, ix1]
    return float(v00 * (1 - fx) * (1 - fy) + v01 * fx * (1 - fy)
                 + v10 * (1 - fx) * fy + v11 * fx * fy)


class Guidance:
    """Loads an ONNX cost-to-go model; builds one field per problem."""

    def __init__(self, model_path=mlcfg.MODEL_PATH, grid_res=mlcfg.GRID_RES):
        self.model_path = model_path
        self.grid_res = grid_res
        self._sess = None
        self.available = False
        self.field = None
        self.affine = None
        if os.path.exists(model_path):
            try:
                import onnxruntime as ort
                self._sess = ort.InferenceSession(
                    model_path, providers=['CPUExecutionProvider'])
                self.available = True
            except Exception:
                self.available = False

    def build_field(self, preprocessed):
        self.affine = raster.compute_crop(preprocessed, self.grid_res)
        channels = raster.build_channels(preprocessed, self.affine, self.grid_res)
        inp = channels[None].astype(np.float32)             # (1, 4, H, W)
        out = self._sess.run(['cost_to_go'], {'channels': inp})[0]
        self.field = np.asarray(out)[0, 0]                  # (H, W)

    def lookup(self, waypoint):
        gx, gy = self.affine.world_to_grid(*waypoint)
        v = bilinear_lookup(self.field, gx, gy)
        return LARGE if v is None else v


def make_guidance_secondary(preprocessed, model_path=None, guidance_obj=None):
    """Build the guidance field once and return (secondary_callable, True), or
    (None, False) when no model is available (caller falls back to hand-crafted)."""
    g = guidance_obj if guidance_obj is not None else Guidance(
        model_path or mlcfg.MODEL_PATH)
    if not g.available:
        return None, False
    g.build_field(preprocessed)
    return (lambda state: g.lookup(state.waypoint)), True
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/guidance_test.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Add the real-ONNX smoke test (skipped when deps absent)**

Add to `ml_planner/tests/guidance_test.py`:

```python
def test_real_onnx_roundtrip(tmp_path):
    onnx = __import__('pytest').importorskip("onnx")
    __import__('pytest').importorskip("onnxruntime")
    from onnx import helper, TensorProto
    # Trivial model: cost_to_go = mean over the 4 channels -> (1,1,H,W).
    node = helper.make_node("ReduceMean", ["channels"], ["cost_to_go"],
                            axes=[1], keepdims=1)
    graph = helper.make_graph(
        [node], "g",
        [helper.make_tensor_value_info("channels", TensorProto.FLOAT, [1, 4, 64, 64])],
        [helper.make_tensor_value_info("cost_to_go", TensorProto.FLOAT, [1, 1, 64, 64])],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    path = str(tmp_path / "guidance.onnx")
    onnx.save(model, path)

    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    g = Guidance(model_path=path, grid_res=64)
    assert g.available is True
    g.build_field(pre)
    assert g.field.shape == (64, 64)
    assert g.lookup(pre['goal_pos']) < LARGE
```

- [ ] **Step 6: Run the full guidance suite**

Run: `python -m pytest ml_planner/tests/guidance_test.py -v`
Expected: PASS (5 tests; `test_real_onnx_roundtrip` PASSES if `onnx`+`onnxruntime` are installed, else SKIPPED).

- [ ] **Step 7: Commit**

```bash
git add ml_planner/guidance.py ml_planner/tests/guidance_test.py
git commit -m "feat(ml_planner): guidance ONNX inference + bilinear lookup

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 6: Wire `secondary='guidance'` into `plan_trajectory_focal`

**Files:**
- Modify: `ml_planner/plan.py`
- Test: `ml_planner/tests/guidance_integration_test.py`

**Interfaces:**
- Consumes: `make_guidance_secondary` (Task 5); `FocalKinodynamicAstar`, `path_length` (Phase 1).
- Produces: `plan_trajectory_focal(preprocessed_scenario, focal_eps=None, secondary=None, verbose=False)` now accepts `secondary='guidance'` (the string) — resolves it to the CNN secondary when a model is available, else `None` (hand-crafted fallback). Any non-`'guidance'` value keeps Phase-1 behavior exactly.

- [ ] **Step 1: Write the failing test**

Create `ml_planner/tests/guidance_integration_test.py`:

```python
import math

import core.map_generator as mg
import core.preprocessing as prep
import core.kinodynamic_astar as astar
from ml_planner.plan import plan_trajectory_focal, path_length


def _mission(pre, path):
    return math.dist(pre['start_pos'], path[0][0]) + path_length(path)


def test_guidance_flag_falls_back_without_model():
    # No guidance.onnx present -> 'guidance' must degrade to the hand-crafted
    # secondary and behave exactly like secondary=None.
    scen = mg.scenario4_complex_maze()
    r_guided = plan_trajectory_focal(prep.prepare_scenario(scen), secondary='guidance')
    r_default = plan_trajectory_focal(prep.prepare_scenario(scen), secondary=None)
    assert r_guided['success'] == r_default['success']
    assert r_guided['success']
    pre = prep.prepare_scenario(scen)
    assert abs(_mission(pre, r_guided['path']) - _mission(pre, r_default['path'])) < 1e-6


def test_synthetic_guidance_secondary_keeps_bound():
    # A guidance-shaped callable (distance-to-goal) as the focal secondary must
    # still respect the epsilon=5% bound vs the base optimal.
    scen = mg.scenario12_perimeter_dynamic_obstacles()
    pre = prep.prepare_scenario(scen)
    base = astar.plan_trajectory(pre, verbose=False)
    assert base['success']
    base_mission = _mission(pre, base['path'])

    goal = pre['goal_state']['waypoint']
    secondary = lambda st: math.hypot(st.waypoint[0] - goal[0], st.waypoint[1] - goal[1])
    r = plan_trajectory_focal(prep.prepare_scenario(scen), focal_eps=0.05, secondary=secondary)
    assert r['success']
    assert _mission(pre, r['path']) <= 1.05 * base_mission + 1e-6
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/guidance_integration_test.py -v`
Expected: FAIL — `test_guidance_flag_falls_back_without_model` errors because `plan_trajectory_focal` passes the string `'guidance'` straight through to `FocalKinodynamicAstar`, which calls it as a function.

- [ ] **Step 3: Write minimal implementation**

Edit `ml_planner/plan.py`. Add this import near the top (after the existing `from ml_planner.focal_astar import FocalKinodynamicAstar`):

```python
from ml_planner.guidance import make_guidance_secondary
```

Then, inside `plan_trajectory_focal`, immediately after the docstring and before `planner = FocalKinodynamicAstar(...)`, insert:

```python
    # 'guidance' selects the learned CNN secondary when a model is available,
    # else falls back to the hand-crafted secondary (Phase-1 behavior). Any
    # other value (None or a callable) is passed through unchanged.
    resolved_secondary = secondary
    if secondary == 'guidance':
        cb, _available = make_guidance_secondary(preprocessed_scenario)
        resolved_secondary = cb            # None when unavailable -> hand-crafted
```

And change the planner construction line from `secondary=secondary` to `secondary=resolved_secondary`:

```python
    planner = FocalKinodynamicAstar(preprocessed_scenario, focal_eps=focal_eps, secondary=resolved_secondary)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/guidance_integration_test.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Run the full suite (Phase-1 regression check)**

Run: `python -m pytest ml_planner/tests -v`
Expected: PASS — all Phase-1 (17) + Phase-2 tests green; no Phase-1 test changed behavior.

- [ ] **Step 6: Commit**

```bash
git add ml_planner/plan.py ml_planner/tests/guidance_integration_test.py
git commit -m "feat(ml_planner): wire secondary='guidance' with hand-crafted fallback

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 7: Training notebook scaffold + model dir

**Files:**
- Create: `ml_planner/train/train_guidance.ipynb`
- Create: `ml_planner/models/.gitignore`, `ml_planner/models/.gitkeep`
- Test: `ml_planner/tests/notebook_test.py`

**Interfaces:**
- Produces: a valid Jupyter notebook (nbformat 4) that documents the off-machine training flow, and a `models/` dir where `guidance.onnx` will live (ignored by git).

- [ ] **Step 1: Write the failing test**

Create `ml_planner/tests/notebook_test.py`:

```python
import json
import os


def test_training_notebook_is_valid_and_covers_contract():
    path = os.path.join("ml_planner", "train", "train_guidance.ipynb")
    assert os.path.exists(path)
    with open(path) as f:
        nb = json.load(f)
    assert nb.get("nbformat") == 4
    assert isinstance(nb.get("cells"), list) and len(nb["cells"]) >= 4
    text = "\n".join("".join(c.get("source", [])) for c in nb["cells"])
    # The notebook must document the hard I/O contract and masked loss.
    for token in ("channels", "cost_to_go", "256", "mask", "onnx"):
        assert token in text


def test_models_dir_ignores_onnx():
    with open(os.path.join("ml_planner", "models", ".gitignore")) as f:
        assert "*.onnx" in f.read()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/notebook_test.py -v`
Expected: FAIL — the notebook and models dir do not exist yet.

- [ ] **Step 3: Create the model dir markers**

Create `ml_planner/models/.gitignore`:

```
*.onnx
```

Create `ml_planner/models/.gitkeep` (empty file).

- [ ] **Step 4: Create the notebook**

Create `ml_planner/train/train_guidance.ipynb` with exactly this content:

```json
{
 "cells": [
  {"cell_type": "markdown", "metadata": {}, "source": [
    "# Train the CNN guidance-map (off-machine, Colab/GPU)\n",
    "Consumes a dataset built on the planner machine via `ml_planner.dataset_gen.export_dataset`.\n",
    "**Hard I/O contract (must match `ml_planner/guidance.py`):** input `channels` (1,4,256,256) float32, output `cost_to_go` (1,1,256,256) float32, opset >= 11. Trains a small U-Net with a masked MSE loss (only labeled cells), then exports ONNX.\n",
    "torch is used ONLY here; the planner never imports it."
  ]},
  {"cell_type": "code", "metadata": {}, "execution_count": null, "outputs": [], "source": [
    "import numpy as np, torch, torch.nn as nn\n",
    "GRID_RES = 256\n",
    "data = np.load('dataset.npz')  # channels (N,4,H,W), label (N,H,W), mask (N,H,W)\n",
    "channels = torch.tensor(data['channels']); label = torch.tensor(data['label']); mask = torch.tensor(data['mask'])"
  ]},
  {"cell_type": "code", "metadata": {}, "execution_count": null, "outputs": [], "source": [
    "class UNetSmall(nn.Module):\n",
    "    def __init__(self, cin=4):\n",
    "        super().__init__()\n",
    "        self.enc = nn.Sequential(nn.Conv2d(cin,32,3,padding=1), nn.ReLU(), nn.Conv2d(32,32,3,padding=1), nn.ReLU())\n",
    "        self.head = nn.Conv2d(32,1,1)\n",
    "    def forward(self, x):\n",
    "        return self.head(self.enc(x))\n",
    "model = UNetSmall()"
  ]},
  {"cell_type": "code", "metadata": {}, "execution_count": null, "outputs": [], "source": [
    "opt = torch.optim.Adam(model.parameters(), 1e-3)\n",
    "for epoch in range(50):\n",
    "    opt.zero_grad()\n",
    "    pred = model(channels)[:,0]\n",
    "    m = mask > 0\n",
    "    loss = (((pred - label)**2) * m).sum() / m.sum().clamp(min=1)  # masked MSE\n",
    "    loss.backward(); opt.step()\n",
    "print('final masked MSE', float(loss))"
  ]},
  {"cell_type": "code", "metadata": {}, "execution_count": null, "outputs": [], "source": [
    "dummy = torch.zeros(1,4,GRID_RES,GRID_RES)\n",
    "torch.onnx.export(model, dummy, 'guidance.onnx', input_names=['channels'], output_names=['cost_to_go'], opset_version=13)\n",
    "# Copy guidance.onnx to ml_planner/models/ on the planner machine."
  ]}
 ],
 "metadata": {"language_info": {"name": "python"}},
 "nbformat": 4,
 "nbformat_minor": 5
}
```

- [ ] **Step 5: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/notebook_test.py -v`
Expected: PASS (2 tests).

- [ ] **Step 6: Commit**

```bash
git add ml_planner/train/train_guidance.ipynb ml_planner/models/.gitignore ml_planner/models/.gitkeep ml_planner/tests/notebook_test.py
git commit -m "feat(ml_planner): training notebook scaffold + model dir

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

## Self-Review

**Spec coverage:**
- §3 shared `raster.py` → Tasks 1–2. §5 dataset_gen edge-record + backward-Dijkstra + rasterize + export → Tasks 3–4. §6 CNN I/O contract → enforced in Tasks 4/5/7 (channels (4,H,W); ONNX names `channels`/`cost_to_go`; 256). §7 guidance inference + fallback → Task 5. §8 integration keeping Phase-1 behavior → Task 6. §9 tests (stub, skip-guarded ONNX, dataset sanity, integration fallback + bound) → Tasks 3–6. §10 notebook scaffold → Task 7. §2 no-mandatory-dep (onnxruntime optional; torch only in notebook) → Tasks 5/7. §2 crop/infinite-map → Task 1.
- **Deferred (spec §12, out of scope):** heading-conditioning, search-effort labels, DAgger, visibility-heuristic fallback. Correctly not tasked.

**Placeholder scan:** No TBD/TODO; every code step contains complete code; commands list expected output.

**Type consistency:** `Affine.world_to_grid/grid_to_world` and `compute_crop(preprocessed, grid_res, margin_frac)` consistent across Tasks 1–5. `build_channels(preprocessed, affine, grid_res) -> (4,H,W)` consistent Tasks 2/4/5. `_RecordingAstar.edges/key2wp`, `backward_costs(edges, goal_key)` consistent Tasks 3/4. `bilinear_lookup(field, gx, gy)`, `Guidance.available/build_field/lookup`, `make_guidance_secondary(preprocessed, model_path, guidance_obj) -> (callable|None, bool)` consistent Tasks 5/6. Grid indexing `[iy, ix]` fixed in Global Constraints and used uniformly.

---

## Notes for the implementer

- Run the whole suite any time: `python -m pytest ml_planner/tests -v`.
- `_no_budget()` mutates `config.TIME_BUDGET_S`/`MAX_ITERATIONS` at runtime and restores them in a `finally` — it does NOT edit `config.py`. Always use it as a `with` block so a raised exception still restores the caps.
- `dataset_gen` and `guidance` MUST both go through `raster.compute_crop`/`build_channels` — never re-derive crop or channels inline, or the model's inputs won't match training.
- Task 6 is the only edit to a Phase-1 file (`plan.py`); keep the change to the three inserted lines + the one construction-arg rename, and confirm `python -m pytest ml_planner/tests` still shows the Phase-1 tests green.
