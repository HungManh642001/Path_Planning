# Lazy Focal + AI Corridor (Bound-Preserving) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cut the planner's dominant cost (collision checks, ~70% of search time) by deferring them until a node is actually popped, with an AI corridor (reusing the trained GNN) steering FOCAL admission — while keeping the ε=5% bound formally intact.

**Architecture:** Two behavior-neutral hooks + a collision counter go into `ml_planner/focal_astar.py`; `ml_planner/lazy_focal.py` subclasses it (defer-at-generation via a context-armed `_check_collision` trap, validate-on-pop); `ml_planner/corridor.py` rasterizes the GNN field into a boolean grid gating FOCAL admission with an admit-all fallback. Benchmark gains `lazy` and `lcor` planners plus real-check counters for layered attribution. Spec: `docs/superpowers/specs/2026-07-19-lazy-corridor-design.md` (§4.2 as amended).

**Tech Stack:** pure Python + numpy/scipy (no new deps, no torch, no core/ changes). Reuses `ml_planner/models/graph_guidance.npz` + `graph_guidance.GraphGuidance` unchanged.

## Global Constraints

- Do NOT modify anything under `core/`.
- The ε-bound is NON-NEGOTIABLE: every mode must satisfy `mission_cost ≤ 1.05 × base` on every solved benchmark scenario (0 violations). A wrong corridor may only cost time, never the bound.
- Deferral must never change: the valve LOS test semantics (goal chords are never deferred), eager đoản-trình checks, arc-hop behavior (`_arc_hop_successors` uses `_max_clear_wrap`/`_sector_clear`, not `_check_collision` — verified), or `smooth_path`/`_check_fixed_legs` (trap disarmed outside `get_next_states`).
- Hook defaults in `FocalKinodynamicAstar` must be behavior-neutral: the existing ml_planner suite (54 pass / 3 skip at current tip) is the pin; ONE pre-existing dirty-working-tree failure `notebook_test.py::test_models_dir_ignores_onnx` is not a regression.
- Lazy modes always use the hand-crafted secondary (`secondary=None`); the GNN appears ONLY in the corridor.
- New config: `CORRIDOR_DELTA = 0.15`, `CORRIDOR_GRID_RES = 128` in `ml_planner/config.py`.
- Test files named `*_test.py`; run from repo root (`python -m pytest ...`).
- Commits end with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`; `git add` ONLY the files named per task (unrelated user WIP in the tree must never be swept in). `ml_planner/benchmark.py`/`EVAL.md` may carry a whitespace-only CRLF→LF normalization when committed — authorized, note it in the report.

---

### Task 1: Behavior-neutral hooks + collision counter in `focal_astar.py`

**Files:**
- Modify: `ml_planner/focal_astar.py`
- Modify: `ml_planner/plan.py` (surface `collision_checks` in stats)
- Test: `ml_planner/tests/focal_astar_test.py` (append)

**Interfaces:**
- Consumes: current `FocalKinodynamicAstar` (`search()` selection loop, `_refill_focal`, inline focal push, drain branch — exact code shown below).
- Produces (Task 2/4 rely on these exact names): `self.collision_checks` (int, counts REAL checks), `_check_collision(p1, p2)` override (count + `super()`), hooks `_focal_admissible(state) -> bool` (default True), `_validate_on_pop(state) -> bool` (default True), attribute `self._admit_all` (bool, default False); `plan_trajectory_focal` result gains `result['stats']['collision_checks']`.

- [ ] **Step 1: Write the failing tests**

Append to `ml_planner/tests/focal_astar_test.py`:

```python
def test_collision_checks_counted():
    pre = prep.prepare_scenario(mg.scenario4_complex_maze())
    planner = FocalKinodynamicAstar(pre, focal_eps=0.05)
    assert planner.collision_checks == 0
    path = planner.search()
    assert path is not None
    assert planner.collision_checks > 0


def test_admission_filter_reject_all_still_solves_via_admit_all():
    # A hostile admission filter must only slow the search down (drain-path
    # admit-all fallback), never starve it or break the bound.
    scen = mg.scenario4_complex_maze
    opt = _optimal_cost(scen)
    pre = prep.prepare_scenario(scen())
    planner = FocalKinodynamicAstar(pre, focal_eps=0.05)
    planner._focal_admissible = lambda st: False
    path = planner.search()
    assert path is not None
    path = planner.smooth_path(path)
    assert _mission_cost(pre, path) <= 1.05 * opt + 1e-6


def test_validate_on_pop_false_discards_without_closing():
    # Rejecting every pop must terminate with no path, not hang or crash.
    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    planner = FocalKinodynamicAstar(pre, focal_eps=0.05)
    planner._validate_on_pop = lambda st: False
    assert planner.search() is None


def test_focal_stats_include_collision_checks():
    from ml_planner.plan import plan_trajectory_focal
    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    res = plan_trajectory_focal(pre, focal_eps=0.05)
    assert res['success']
    assert res['stats']['collision_checks'] > 0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ml_planner/tests/focal_astar_test.py -v`
Expected: existing tests pass; the 4 new ones FAIL (`AttributeError: collision_checks` etc.).

- [ ] **Step 3: Implement the hooks in `focal_astar.py`**

3a. In `FocalKinodynamicAstar.__init__`, after `self._secondary = secondary` add:

```python
        self.collision_checks = 0    # REAL collision checks paid (lazy A/B metric)
        self._admit_all = False      # drain-path override of _focal_admissible
```

3b. After `secondary_h`, add the counting override and the two hooks:

```python
    def _check_collision(self, p1, p2):
        self.collision_checks += 1
        return super()._check_collision(p1, p2)

    # ---- extension points for the lazy variant (behavior-neutral here) ----
    def _focal_admissible(self, state):
        """FOCAL admission filter; the lazy+corridor subclass narrows this.
        Rejected states stay in OPEN (still bounding f_min); the drain path
        retries with _admit_all so filtering can never starve the search."""
        return True

    def _validate_on_pop(self, state):
        """Last-moment edge validation; the lazy subclass defers collision
        checks to here. Returning False discards the pop (state NOT closed)."""
        return True
```

3c. In `search()`, `_refill_focal` admission condition becomes:

```python
            for f, c, st in open_heap:
                if (f <= f_bound + config.EPS and id(st) not in in_focal
                        and _is_live(st)
                        and (self._admit_all or self._focal_admissible(st))):
                    heapq.heappush(focal_heap, (self.secondary_h(st), c, st))
                    in_focal.add(id(st))
```

3d. The FOCAL selection loop gains the pop-validation condition:

```python
            current = None
            while focal_heap:
                _, _, cand = heapq.heappop(focal_heap)
                in_focal.discard(id(cand))
                if _is_live(cand) and self._validate_on_pop(cand):
                    current = cand
                    break
```

3e. The drain branch (`if current is None:`) gains the admit-all fallback after the normal refill:

```python
            if current is None:
                _clean_open_top()
                if not open_heap:
                    break
                f_min = open_heap[0][0]
                _refill_focal(w * f_min)
                if not focal_heap and open_heap:
                    # Admission filtering (corridor) found nothing in band:
                    # admit unconditionally so filtering can only cost time,
                    # never starve the search or fake a no-path.
                    self._admit_all = True
                    try:
                        _refill_focal(w * f_min)
                    finally:
                        self._admit_all = False
                continue
```

3f. The inline focal push (successor loop) gains the same admission condition:

```python
                    if (f_min is not None and f <= w * f_min + config.EPS
                            and (self._admit_all or self._focal_admissible(next_state))):
                        heapq.heappush(focal_heap, (self.secondary_h(next_state), c, next_state))
                        in_focal.add(id(next_state))
```

3g. In `ml_planner/plan.py`, `plan_trajectory_focal` return block — surface the counter:

```python
    stats = planner.get_search_stats()
    stats['collision_checks'] = planner.collision_checks
    return {
        'path': path,
        'success': path is not None and legs_ok,
        'stats': stats,
        'planner': planner,
    }
```

(replacing the current dict that called `planner.get_search_stats()` inline; the earlier `verbose` block may keep its own `get_search_stats()` call.)

- [ ] **Step 4: Run the tests**

Run: `python -m pytest ml_planner/tests/focal_astar_test.py ml_planner/tests/plan_test.py -v`
Expected: ALL pass (existing behavior pinned + 4 new).

- [ ] **Step 5: Run the full ml_planner suite (behavior-neutrality pin)**

Run: `python -m pytest -q ml_planner/tests/`
Expected: no new failures vs baseline (the one pre-existing notebook_test failure only).

- [ ] **Step 6: Commit**

```bash
git add ml_planner/focal_astar.py ml_planner/plan.py ml_planner/tests/focal_astar_test.py
git commit -m "feat(ml_planner): behavior-neutral lazy hooks + collision-check counter in focal search

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: `lazy_focal.py` — defer-at-generation, validate-on-pop

**Files:**
- Create: `ml_planner/lazy_focal.py`
- Test: `ml_planner/tests/lazy_focal_test.py`

**Interfaces:**
- Consumes: Task-1 hooks (`_validate_on_pop`, `_focal_admissible`, `collision_checks`, counting `_check_collision`); `FocalKinodynamicAstar`.
- Produces: `LazyFocalKinodynamicAstar(preprocessed, focal_eps=None, corridor=None)`; states carry `edge_validated` (absent/True = validated); `corridor` duck-type: object with `.contains(x, y) -> bool` or None. Task 3's `Corridor` and Task 4's `plan_trajectory_lazy` rely on these exact names.

- [ ] **Step 1: Write the failing tests**

Create `ml_planner/tests/lazy_focal_test.py`:

```python
import math

import pytest

import core.kinodynamic_astar as astar
import core.map_generator as mg
import core.preprocessing as prep
from ml_planner.focal_astar import FocalKinodynamicAstar
from ml_planner.lazy_focal import LazyFocalKinodynamicAstar


def _path_len(path):
    total = 0.0
    for (a, _), (b, _) in zip(path, path[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def _mission_cost(pre, path):
    return math.dist(pre['start_pos'], path[0][0]) + _path_len(path)


def _base_cost(scenario_func):
    pre = prep.prepare_scenario(scenario_func())
    res = astar.plan_trajectory(pre, verbose=False)
    assert res['success']
    return _mission_cost(pre, res['path'])


def test_lazy_equals_eager_on_obstacle_free_map():
    # No obstacles -> every deferred edge is valid -> deferral changes
    # nothing: identical path, identical expansion count.
    scen = mg.scenario1_open_ocean
    pre_e = prep.prepare_scenario(scen())
    eager = FocalKinodynamicAstar(pre_e, focal_eps=0.05)
    path_e = eager.search()
    pre_l = prep.prepare_scenario(scen())
    lazy = LazyFocalKinodynamicAstar(pre_l, focal_eps=0.05)
    path_l = lazy.search()
    assert path_e is not None and path_l is not None
    assert lazy.iteration_count == eager.iteration_count
    assert abs(_mission_cost(pre_l, path_l) - _mission_cost(pre_e, path_e)) < 1e-6


@pytest.mark.parametrize("scenario_func", [
    mg.scenario4_complex_maze,
    mg.scenario12_perimeter_dynamic_obstacles,
    mg.scenario13_dense_island_field,
    mg.scenario16_extreme_complexity,
])
def test_lazy_epsilon_bound_holds(scenario_func):
    # The non-negotiable contract: pure-lazy (corridor=None) stays within
    # 1.05x the base planner on obstacle maps.
    base = _base_cost(scenario_func)
    pre = prep.prepare_scenario(scenario_func())
    lazy = LazyFocalKinodynamicAstar(pre, focal_eps=0.05)
    path = lazy.search()
    assert path is not None
    path = lazy.smooth_path(path)
    assert _mission_cost(pre, path) <= 1.05 * base + 1e-6


def test_lazy_pays_fewer_real_checks_than_eager():
    scen = mg.scenario4_complex_maze
    pre_e = prep.prepare_scenario(scen())
    eager = FocalKinodynamicAstar(pre_e, focal_eps=0.05)
    assert eager.search() is not None
    pre_l = prep.prepare_scenario(scen())
    lazy = LazyFocalKinodynamicAstar(pre_l, focal_eps=0.05)
    assert lazy.search() is not None
    assert lazy.collision_checks < eager.collision_checks


def test_goal_chords_never_deferred():
    # Every accepted goal arrival must ride an already-validated edge; the
    # trap must exclude p2 == goal waypoint (also keeps the valve LOS test
    # honest at the same call site).
    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    lazy = LazyFocalKinodynamicAstar(pre, focal_eps=0.05)
    path = lazy.search()
    assert path is not None
    # The state whose waypoint is nearest the goal must be validated.
    import config
    gwp = lazy.goal_state.waypoint
    # walk the returned path's final state via g_scores objects: the search
    # only returns via _goal_reached(current) where current popped validated;
    # assert no un-validated state sits within GOAL_THRESHOLD of the goal.
    for st in lazy.g_scores:
        d = math.hypot(st.waypoint[0] - gwp[0], st.waypoint[1] - gwp[1])
        if d < config.GOAL_THRESHOLD:
            assert getattr(st, 'edge_validated', True) is True


def test_no_path_map_terminates_with_none():
    # Goal sealed inside a ring of overlapping circles: base fails, lazy must
    # also conclude no-path (finite), not hang on optimistic frontier.
    goal = (200_000.0, 0.0)
    ring = []
    for k in range(8):
        ang = 2.0 * math.pi * k / 8
        ring.append(((goal[0] + 40_000.0 * math.cos(ang),
                      goal[1] + 40_000.0 * math.sin(ang)), 30_000.0))
    scen = {'start': (0.0, 0.0), 'start_heading': 0.0,
            'goal': goal, 'goal_heading': 0.0,
            'islands': [], 'dynamic_obstacles': ring,
            'obstacles': [{'type': 'circle', 'center': c, 'radius': r}
                          for c, r in ring]}
    pre = prep.prepare_scenario(scen)
    base = astar.plan_trajectory(pre, verbose=False)
    assert not base['success']
    pre2 = prep.prepare_scenario(scen)
    lazy = LazyFocalKinodynamicAstar(pre2, focal_eps=0.05)
    assert lazy.search() is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ml_planner/tests/lazy_focal_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner.lazy_focal'`.

- [ ] **Step 3: Write the implementation**

Create `ml_planner/lazy_focal.py`:

```python
"""Bound-preserving lazy variant of the focal planner.

Strategy-A / fan chord collision checks are DEFERRED at generation (the
successor is created optimistically with edge_validated=False) and paid only
when the node is actually popped for expansion. An optimistic f is <= the
true f (a collision check can only delete an edge, never cheapen it), so
f_min over OPEN remains a valid lower bound and the focal (1+eps) guarantee
is unchanged. An optional Corridor (ml_planner/corridor.py) gates FOCAL
admission only: out-of-corridor nodes sit in OPEN (still bounding f_min) and
are admitted anyway by the drain-path admit-all fallback, so a wrong model
can only cost time, never the bound.

Never deferred: chords to the goal waypoint (keeps the valve LOS test at the
same call site honest, and guarantees every accepted goal arrival rides a
validated edge), đoản-trình checks (run before collision in the core loop),
and arc-hop geometry (_max_clear_wrap/_sector_clear, not _check_collision).
"""

from ml_planner.focal_astar import FocalKinodynamicAstar


class LazyFocalKinodynamicAstar(FocalKinodynamicAstar):
    def __init__(self, preprocessed_scenario, focal_eps=None, corridor=None):
        super().__init__(preprocessed_scenario, focal_eps=focal_eps, secondary=None)
        self.corridor = corridor
        self._lazy_ctx = None       # state currently generating successors
        self._deferred_now = set()  # waypoints deferred during this arm

    # ---- defer-at-generation trap -------------------------------------
    def get_next_states(self, current_state):
        self._lazy_ctx = current_state
        self._deferred_now = set()
        try:
            successors = super().get_next_states(current_state)
        finally:
            self._lazy_ctx = None
        for st, _cost in successors:
            if st.waypoint in self._deferred_now:
                st.edge_validated = False
        return successors

    def _check_collision(self, p1, p2):
        ctx = self._lazy_ctx
        if (ctx is not None and p1 == ctx.waypoint
                and p2 != self.goal_state.waypoint):
            # Optimistically clear; the real check runs at pop time.
            self._deferred_now.add(p2)
            return True
        return super()._check_collision(p1, p2)

    # ---- validate-on-pop ----------------------------------------------
    def _validate_on_pop(self, state):
        if getattr(state, 'edge_validated', True):
            return True
        ok = self._check_collision(state.parent.waypoint, state.waypoint)
        if ok:
            state.edge_validated = True
            return True
        # Dead edge: forget this g so the lattice cell stays re-discoverable
        # through other (possibly valid) incoming edges.
        if self.g_scores.get(state) == state.g_cost:
            del self.g_scores[state]
        return False

    # ---- corridor gates FOCAL admission only ---------------------------
    def _focal_admissible(self, state):
        if self.corridor is None:
            return True
        return self.corridor.contains(state.waypoint[0], state.waypoint[1])
```

- [ ] **Step 4: Run the tests**

Run: `python -m pytest ml_planner/tests/lazy_focal_test.py -v`
Expected: 8 passed (1 + 4 parametrized + 3).

- [ ] **Step 5: Full suite**

Run: `python -m pytest -q ml_planner/tests/`
Expected: no new failures.

- [ ] **Step 6: Commit**

```bash
git add ml_planner/lazy_focal.py ml_planner/tests/lazy_focal_test.py
git commit -m "feat(ml_planner): bound-preserving lazy focal search (defer checks to pop)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: `corridor.py` — GNN field → boolean admission grid

**Files:**
- Create: `ml_planner/corridor.py`
- Modify: `ml_planner/config.py` (append two constants)
- Test: `ml_planner/tests/corridor_test.py`

**Interfaces:**
- Consumes: `ml_planner.graph_guidance.GraphGuidance` (`.available`, `.build_field(pre)`, `.graph` (`.kdtree`, `.nodes`), `.values`, `.lookup(waypoint)`); `ml_planner.raster.compute_crop`, `raster._cell_centers_world` (private sibling — acceptable within the package, same pattern as raster's own internals).
- Produces: `Corridor` (fields `mask (G,G) bool`, `affine`, `grid_res`; method `contains(x, y) -> bool`, outside-crop → False); `build_corridor(preprocessed, graph_guidance, delta=None, grid_res=None) -> Corridor | None` (None when guidance missing/unavailable or on any exception); `mlcfg.CORRIDOR_DELTA = 0.15`, `mlcfg.CORRIDOR_GRID_RES = 128`.

- [ ] **Step 1: Write the failing tests**

Create `ml_planner/tests/corridor_test.py`:

```python
import numpy as np

import core.preprocessing as prep
from ml_planner.corridor import Corridor, build_corridor
from ml_planner.graph_guidance import GraphGuidance
from ml_planner import raster


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


def test_contains_membership_and_out_of_crop():
    pre = prep.prepare_scenario(_scenario())
    aff = raster.compute_crop(pre, 16)
    mask = np.zeros((16, 16), dtype=bool)
    mask[3, 5] = True
    cor = Corridor(mask, aff)
    x, y = aff.grid_to_world(5.5, 3.5)          # center of cell [iy=3, ix=5]
    assert cor.contains(x, y) is True
    x2, y2 = aff.grid_to_world(1.5, 1.5)
    assert cor.contains(x2, y2) is False
    assert cor.contains(-1e9, -1e9) is False    # far outside crop


def test_build_corridor_none_without_model(tmp_path):
    gg = GraphGuidance(model_path=str(tmp_path / "missing.npz"))
    pre = prep.prepare_scenario(_scenario())
    assert build_corridor(pre, gg) is None
    assert build_corridor(pre, None) is None


def test_build_corridor_start_goal_always_inside(tmp_path):
    path = str(tmp_path / "w.npz")
    _random_weights(path)
    gg = GraphGuidance(model_path=path)
    assert gg.available
    pre = prep.prepare_scenario(_scenario())
    cor = build_corridor(pre, gg, delta=0.0)     # tightest corridor
    assert cor is not None
    assert cor.contains(*pre['start_pos']) is True
    assert cor.contains(*pre['goal_pos']) is True


def test_build_corridor_deterministic(tmp_path):
    path = str(tmp_path / "w.npz")
    _random_weights(path)
    gg = GraphGuidance(model_path=path)
    pre = prep.prepare_scenario(_scenario())
    c1 = build_corridor(pre, gg)
    c2 = build_corridor(pre, gg)
    assert np.array_equal(c1.mask, c2.mask)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ml_planner/tests/corridor_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner.corridor'`.

- [ ] **Step 3: Append the config constants**

Append to `ml_planner/config.py`:

```python
# ====== LAZY FOCAL + AI CORRIDOR ======
# Corridor slack: a cell belongs to the corridor when
# dist(start, cell) + V_hat(cell) <= (1 + CORRIDOR_DELTA) * V_hat(start).
CORRIDOR_DELTA = 0.15
# Boolean admission grid resolution (contains() is one array index).
CORRIDOR_GRID_RES = 128
```

- [ ] **Step 4: Write the implementation**

Create `ml_planner/corridor.py`:

```python
"""AI corridor: rasterize the GNN per-node cost-to-go into a boolean grid.

Corridor membership gates FOCAL admission only — never correctness: nodes
outside the corridor stay in OPEN (still bounding f_min) and the drain-path
admit-all fallback admits them whenever the in-corridor band drains, so a
wrong model can only cost time, never the epsilon bound.
"""

import numpy as np

import ml_planner.config as mlcfg
import ml_planner.raster as raster


class Corridor:
    def __init__(self, mask, affine):
        self.mask = mask                     # (G, G) bool, [iy, ix]
        self.affine = affine
        self.grid_res = mask.shape[0]

    def contains(self, x, y):
        gx, gy = self.affine.world_to_grid(x, y)
        ix, iy = int(gx), int(gy)
        if 0 <= iy < self.grid_res and 0 <= ix < self.grid_res:
            return bool(self.mask[iy, ix])
        return False                         # outside crop: not admitted


def build_corridor(preprocessed, graph_guidance, delta=None, grid_res=None):
    """Boolean corridor from the GNN value field, or None (clean fallback)
    when the guidance is missing/unavailable or anything fails."""
    if graph_guidance is None or not getattr(graph_guidance, 'available', False):
        return None
    try:
        delta = mlcfg.CORRIDOR_DELTA if delta is None else delta
        grid_res = mlcfg.CORRIDOR_GRID_RES if grid_res is None else grid_res
        graph_guidance.build_field(preprocessed)
        g = graph_guidance.graph
        aff = raster.compute_crop(preprocessed, grid_res)
        wx, wy = raster._cell_centers_world(aff, grid_res)
        pts = np.column_stack([wx.ravel(), wy.ravel()])
        k = min(3, len(g.nodes))
        d, idx = g.kdtree.query(pts, k=k)
        d = np.atleast_2d(d).reshape(len(pts), -1)
        idx = np.atleast_2d(idx).reshape(len(pts), -1)
        vhat = (d + graph_guidance.values[idx]).min(axis=1).reshape(grid_res, grid_res)
        ox, oy = preprocessed['start_pos']
        d_start = np.hypot(wx - ox, wy - oy)
        cap = (1.0 + delta) * graph_guidance.lookup(preprocessed['start_pos'])
        mask = (d_start + vhat) <= cap
        # Start and goal cells are corridor members by definition.
        for pt in (preprocessed['start_pos'], preprocessed['goal_pos']):
            gx, gy = aff.world_to_grid(*pt)
            ix, iy = int(round(gx)), int(round(gy))
            if 0 <= iy < grid_res and 0 <= ix < grid_res:
                mask[iy, ix] = True
        return Corridor(mask, aff)
    except Exception:
        return None
```

- [ ] **Step 5: Run the tests**

Run: `python -m pytest ml_planner/tests/corridor_test.py -v`
Expected: 4 passed.

- [ ] **Step 6: Commit**

```bash
git add ml_planner/corridor.py ml_planner/config.py ml_planner/tests/corridor_test.py
git commit -m "feat(ml_planner): boolean AI corridor from the GNN value field

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: `plan_trajectory_lazy` + corridor/lazy integration tests (money test)

**Files:**
- Modify: `ml_planner/plan.py` (append)
- Test: `ml_planner/tests/lazy_focal_test.py` (append)

**Interfaces:**
- Consumes: `LazyFocalKinodynamicAstar` (Task 2), `Corridor` (Task 3), `plan_trajectory_focal` (fallback).
- Produces: `plan_trajectory_lazy(preprocessed_scenario, corridor=None, focal_eps=None, verbose=False) -> dict` — same result contract as `plan_trajectory_focal` (`path/success/stats/planner`), `stats['collision_checks']` present; unexpected exceptions fall back to `plan_trajectory_focal`. Task 5's benchmark relies on this exact name/signature.

- [ ] **Step 1: Write the failing tests**

Append to `ml_planner/tests/lazy_focal_test.py`:

```python
import numpy as np

from ml_planner import raster
from ml_planner.corridor import Corridor
from ml_planner.plan import plan_trajectory_lazy


def _corridor_with_mask(pre, fill, grid_res=32):
    aff = raster.compute_crop(pre, grid_res)
    mask = np.full((grid_res, grid_res), fill, dtype=bool)
    for pt in (pre['start_pos'], pre['goal_pos']):
        gx, gy = aff.world_to_grid(*pt)
        ix, iy = int(round(gx)), int(round(gy))
        if 0 <= iy < grid_res and 0 <= ix < grid_res:
            mask[iy, ix] = True
    return Corridor(mask, aff)


@pytest.mark.parametrize("scenario_func", [
    mg.scenario4_complex_maze,
    mg.scenario12_perimeter_dynamic_obstacles,
])
def test_money_all_false_corridor_still_bound(scenario_func):
    # THE money test: a maximally wrong corridor (nothing admitted except the
    # start/goal cells) may only cost time — the admit-all fallback must
    # still produce a valid path within the 1.05x bound.
    base = _base_cost(scenario_func)
    pre = prep.prepare_scenario(scenario_func())
    cor = _corridor_with_mask(pre, fill=False)
    res = plan_trajectory_lazy(pre, corridor=cor, focal_eps=0.05)
    assert res['success']
    assert _mission_cost(pre, res['path']) <= 1.05 * base + 1e-6


def test_all_true_corridor_equals_pure_lazy():
    # An all-True corridor admits everything -> identical to corridor=None.
    scen = mg.scenario4_complex_maze
    pre_a = prep.prepare_scenario(scen())
    cor = _corridor_with_mask(pre_a, fill=True)
    res_a = plan_trajectory_lazy(pre_a, corridor=cor, focal_eps=0.05)
    pre_b = prep.prepare_scenario(scen())
    res_b = plan_trajectory_lazy(pre_b, corridor=None, focal_eps=0.05)
    assert res_a['success'] and res_b['success']
    assert res_a['stats']['iterations'] == res_b['stats']['iterations']
    assert abs(_mission_cost(pre_a, res_a['path'])
               - _mission_cost(pre_b, res_b['path'])) < 1e-6


def test_plan_trajectory_lazy_stats_contract():
    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    res = plan_trajectory_lazy(pre, focal_eps=0.05)
    assert res['success']
    assert res['stats']['collision_checks'] > 0
    assert res['path'] is not None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ml_planner/tests/lazy_focal_test.py -v`
Expected: earlier tests pass; new ones FAIL with `ImportError: cannot import name 'plan_trajectory_lazy'`.

- [ ] **Step 3: Write the implementation**

Append to `ml_planner/plan.py` (also add the import at the top of the file: `from ml_planner.lazy_focal import LazyFocalKinodynamicAstar`):

```python
def plan_trajectory_lazy(preprocessed_scenario, corridor=None, focal_eps=None, verbose=False):
    """Plan with the bound-preserving lazy focal search (hand secondary).

    corridor=None -> pure lazy (mechanism baseline); a Corridor gates FOCAL
    admission (AI mode). Unexpected errors fall back to the eager focal
    planner so this entry point is never less reliable than the baseline.
    """
    try:
        planner = LazyFocalKinodynamicAstar(
            preprocessed_scenario, focal_eps=focal_eps, corridor=corridor)
        legs_ok = planner._check_fixed_legs()
        path = None
        if legs_ok:
            if verbose:
                print("Starting lazy focal search...")
            path = planner.search()
        if path:
            path = planner.smooth_path(path)
        stats = planner.get_search_stats()
        stats['collision_checks'] = planner.collision_checks
        return {
            'path': path,
            'success': path is not None and legs_ok,
            'stats': stats,
            'planner': planner,
        }
    except Exception:
        return plan_trajectory_focal(preprocessed_scenario, focal_eps=focal_eps, secondary=None)
```

- [ ] **Step 4: Run the tests**

Run: `python -m pytest ml_planner/tests/lazy_focal_test.py ml_planner/tests/plan_test.py -v`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add ml_planner/plan.py ml_planner/tests/lazy_focal_test.py
git commit -m "feat(ml_planner): plan_trajectory_lazy entry point + corridor money tests

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 5: Benchmark — `lazy` + `lcor` planners, check counters, layered verdict

**Files:**
- Modify: `ml_planner/benchmark.py`
- Modify: `ml_planner/EVAL.md` (append)
- Test: `ml_planner/tests/benchmark_test.py` (append; read the file first, follow its conventions)

**Interfaces:**
- Consumes: `plan_trajectory_lazy` (Task 4), `build_corridor` (Task 3), existing `compare_one`/`_summ`/`_verdict`/`CSV_COLUMNS` structure (already 4-way base/hand/cnn/gnn).
- Produces: `compare_one(..., graph_guidance=None)` rows gain `lazy_success, lazy_iters, lazy_time, lazy_mission, lazy_flight, lazy_cost_ratio, lazy_bound_ok, lazy_checks` and the same 8 with `lcor_` prefix, plus `hand_checks`; `lazy_verdict(hard) -> None` printing the layered acceptance; CSV_COLUMNS extended.

- [ ] **Step 1: Write the failing tests**

Append to `ml_planner/tests/benchmark_test.py` (adapt imports to the file's existing style):

```python
def test_compare_one_emits_lazy_and_lcor_columns():
    row = bm.compare_one(generate_random_scenario, 7003, 'easy',
                         guidance=None, eps=0.05, graph_guidance=None)
    for col in ('lazy_success', 'lazy_iters', 'lazy_time', 'lazy_checks',
                'lcor_success', 'lcor_iters', 'lcor_time', 'lcor_checks',
                'hand_checks', 'lazy_bound_ok', 'lcor_bound_ok'):
        assert col in row
    assert all(c in bm.CSV_COLUMNS for c in
               ('lazy_iters', 'lcor_iters', 'hand_checks', 'lazy_checks', 'lcor_checks'))


def test_lazy_verdict_layers(capsys):
    hard = dict(n=5, t_h=10.0, it_h=1000, checks_h=50000,
                t_lz=8.0, it_lz=1000, checks_lz=20000, viol_lz=0,
                t_lc=7.0, it_lc=900, checks_lc=15000, viol_lc=0)
    bm.lazy_verdict(hard)
    out = capsys.readouterr().out
    assert 'LAZY' in out and 'PASS' in out
    # bound violation flips the verdict regardless of speed
    hard['viol_lc'] = 1
    bm.lazy_verdict(hard)
    out = capsys.readouterr().out
    assert 'FAIL' in out
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest ml_planner/tests/benchmark_test.py -v`
Expected: existing pass; new FAIL (`KeyError: 'lazy_success'` / `AttributeError: lazy_verdict`).

- [ ] **Step 3: Implement the benchmark extension**

3a. Imports — add:

```python
from ml_planner.corridor import build_corridor
from ml_planner.plan import plan_trajectory_focal, path_length, plan_trajectory_lazy
```

(the second line replaces the existing plan import line.)

3b. New plan wrappers next to `_gnn_plan`:

```python
def _lazy_plan(scen, eps):
    """Pure lazy focal (mechanism baseline, no model)."""
    pre = prep.prepare_scenario(scen)
    return plan_trajectory_lazy(pre, corridor=None, focal_eps=eps)


def _lcor_plan(scen, graph_guidance, eps):
    """Lazy + AI corridor; time INCLUDES build_corridor (field + raster)."""
    pre = prep.prepare_scenario(scen)
    cor = build_corridor(pre, graph_guidance)    # None on missing model
    return plan_trajectory_lazy(pre, corridor=cor, focal_eps=eps)
```

3c. In `compare_one`, after the `rn, tn = _safe(...)` line add:

```python
    rlz, tlz = _safe(lambda: _lazy_plan(scen, eps))
    rlc, tlc = _safe(lambda: _lcor_plan(scen, graph_guidance, eps))
```

then mirror the existing per-planner row logic for `('lazy', rlz)` and
`('lcor', rlc)`: success/iters/time keys, mission/flight in the loop over
planners, cost-ratio + bound-ok inside the base-success block (and blank-fill
in the else branch), exactly the shape used for `gnn`. Additionally:

```python
    def checks(r):
        return r['stats'].get('collision_checks', '') if ok(r) else ''
    row['hand_checks'] = checks(rh)
    row['lazy_checks'] = checks(rlz)
    row['lcor_checks'] = checks(rlc)
```

3d. Extend `CSV_COLUMNS` (keep the existing order, append after the gnn
group): `'lazy_success', 'lcor_success', 'lazy_iters', 'lcor_iters',
'lazy_time', 'lcor_time', 'lazy_mission', 'lcor_mission', 'lazy_flight',
'lcor_flight', 'lazy_cost_ratio', 'lcor_cost_ratio', 'lazy_bound_ok',
'lcor_bound_ok', 'hand_checks', 'lazy_checks', 'lcor_checks'`.

3e. Extend `_summ`: require `lazy_success` and `lcor_success` in the
all-solved filter; print two extra lines and return the extra totals:

```python
    it_lz, t_lz = sum(col('lazy_iters')), sum(col('lazy_time'))
    it_lc, t_lc = sum(col('lcor_iters')), sum(col('lcor_time'))
    ck_h = sum(v for v in col('hand_checks') if v != '')
    ck_lz = sum(v for v in col('lazy_checks') if v != '')
    ck_lc = sum(v for v in col('lcor_checks') if v != '')
    print(f"  lazy         iters={it_lz}  time={t_lz:.1f}s  checks={ck_lz} (hand checks={ck_h})")
    print(f"  lazy+corr    iters={it_lc}  time={t_lc:.1f}s  checks={ck_lc}")
```

and add to the returned dict: `it_lz=it_lz, t_lz=t_lz, it_lc=it_lc,
t_lc=t_lc, checks_h=ck_h, checks_lz=ck_lz, checks_lc=ck_lc,
viol_lz=sum(1 for r in sub if r['lazy_bound_ok'] is False),
viol_lc=sum(1 for r in sub if r['lcor_bound_ok'] is False)`.

3f. Add the layered verdict function and call it at the end of `main()`
(after `_verdict(hard_summ)`), guarded by `if hard_summ and 'it_lz' in hard_summ: lazy_verdict(hard_summ)`:

```python
def lazy_verdict(hard):
    """Spec 2026-07-19 (lazy-corridor) §2: LCOR must beat hand on wall-time
    with ZERO bound violations; report layered attribution (mechanism vs AI).
    Early-stop signal: if pure lazy already fails to beat hand, the corridor
    layer is moot."""
    print("\n=== LAZY / CORRIDOR ACCEPTANCE (hard maps, vs hand-crafted) ===")
    print(f"  hand  t={hard['t_h']:.1f}s  checks={hard['checks_h']}")
    print(f"  lazy  t={hard['t_lz']:.1f}s  checks={hard['checks_lz']}  "
          f"(mechanism: {100 * (1 - hard['t_lz'] / hard['t_h']):+.1f}% time vs hand)")
    print(f"  lcor  t={hard['t_lc']:.1f}s  checks={hard['checks_lc']}  "
          f"(AI increment: {100 * (1 - hard['t_lc'] / max(hard['t_lz'], 1e-9)):+.1f}% time vs lazy)")
    viol = hard['viol_lz'] + hard['viol_lc']
    if viol:
        print(f"  ❌ FAIL — {viol} epsilon-bound violation(s): NOT acceptable (non-negotiable).")
        return
    if hard['t_lz'] >= hard['t_h']:
        print("  ⚠️ EARLY-STOP SIGNAL — pure lazy does not beat hand on wall-time; "
              "corridor layer is moot on this distribution.")
    if hard['t_lc'] < hard['t_h']:
        print("  ✅ PASS — lazy+corridor beats hand-crafted on wall-time with 0 violations.")
    else:
        print("  ❌ FAIL — lazy+corridor does not beat hand-crafted on wall-time.")
```

3g. Pass-through: `planner_benchmark` and both `compare_one` call sites need
no signature change beyond what exists (graph_guidance already flows through).

- [ ] **Step 4: Run the tests**

Run: `python -m pytest ml_planner/tests/benchmark_test.py ml_planner/tests/ -q`
Expected: all pass, no regressions.

- [ ] **Step 5: No-model smoke**

Run: `python -m ml_planner.benchmark --offline-n 0 --gnn-offline-n 0 --bench-n 2`
Expected: runs 6 planners; `lcor` falls back to pure-lazy behavior (corridor
None); new columns fill; layered verdict prints (or the inconclusive path).

- [ ] **Step 6: Append to `ml_planner/EVAL.md`**

```markdown
## Lazy focal + AI corridor (bound-preserving)

Collision checks are deferred to pop time (optimistic nodes keep f_min a
valid lower bound, so the 1.05x guarantee is intact — see
docs/superpowers/specs/2026-07-19-lazy-corridor-design.md). The corridor
(GNN value field -> boolean grid) gates FOCAL admission only; a wrong model
can only cost time. Benchmark columns: `lazy_*` (mechanism baseline, no
model), `lcor_*` (lazy + corridor), plus real-check counters
`hand_checks/lazy_checks/lcor_checks` for attribution.

Acceptance: lcor must beat hand-crafted on hard-map wall-time with ZERO
epsilon-bound violations. Early-stop: if pure lazy already fails to beat
hand, stop — the corridor layer is moot.

```bash
python -m ml_planner.benchmark --offline-n 0 --gnn-offline-n 0 --bench-n 30
```
```

- [ ] **Step 7: Commit**

```bash
git add ml_planner/benchmark.py ml_planner/EVAL.md ml_planner/tests/benchmark_test.py
git commit -m "feat(ml_planner): 6-way benchmark with lazy/lcor planners and layered acceptance

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 6: Go/no-go run (execution, no new code)

- [ ] **Step 1: Full benchmark**

```bash
python -m ml_planner.benchmark --offline-n 0 --gnn-offline-n 0 --bench-n 30
```

(offline evals skipped — model unchanged since the GNN plan's gate.)

- [ ] **Step 2: Record the outcome**

Read the `LAZY / CORRIDOR ACCEPTANCE` block + per-scenario CSV. Append a
result subsection to `ml_planner/EVAL.md` (numbers: hand vs lazy vs lcor
iters/time/checks, violation count, PASS/FAIL/early-stop) and commit it:

```bash
git add ml_planner/EVAL.md
git commit -m "docs(ml_planner): record lazy/corridor go-no-go result

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

Decision rules (from spec §2): 0 violations mandatory; `t_lcor < t_hand` =
PASS (corridor becomes the recommended fast mode); pure-lazy already losing
to hand = early-stop (record, keep hand default); violations > 0 = a bug in
Tasks 1–4, not a tuning issue — stop and debug before any verdict.

---

## Self-Review (performed while writing)

- **Spec coverage:** §4.1 corridor → Task 3; §4.2 (amended) hooks/trap/validate-on-pop/admission/admit-all → Tasks 1–2; §4.3 benchmark layered → Task 5; §2 criteria + early-stop → Tasks 5–6; §5 fallback ladder → Tasks 3 (None), 4 (except → focal), 1 (admit-all); §6 tests: equivalence (T2), bound lazy (T2), money (T4), checks-counter + no-path (T2), corridor unit (T3), benchmark cols (T5). All covered.
- **Placeholder scan:** Task 5 step 3c says "mirror the existing per-planner row logic ... exactly the shape used for gnn" — acceptable because the gnn shape is IN the file being edited (implementer reads it in place), not a cross-task reference; all other steps carry complete code.
- **Type consistency:** `plan_trajectory_lazy` signature (T4) == benchmark usage (T5); `Corridor.contains(x, y)` (T3) == `_focal_admissible` usage (T2); `collision_checks` (T1) == stats key (T4) == `*_checks` columns (T5); hook names `_validate_on_pop`/`_focal_admissible`/`_admit_all` consistent T1↔T2.
