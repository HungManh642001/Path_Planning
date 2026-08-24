# ml_planner Phase 1 — Focal Search Framework Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an isolated, faster planner variant `ml_planner` that runs bounded-suboptimal focal search (A*ε) over the existing Kinodynamic A*, with a hand-crafted secondary heuristic — producing a working, ε-bounded, faster planner and the scaffolding an ML secondary heuristic will later plug into (Phase 2).

**Architecture:** A new package `ml_planner/` subclasses `core.kinodynamic_astar.KinodynamicAstar` and overrides only `search()` to implement focal search: OPEN ordered by the admissible `f = g + h_euclid` (guarantees the ε bound), FOCAL expands the node minimizing a secondary heuristic. Everything else (successor generation, exact collision checks, arc-hop, smoothing, seeded start corners) is reused by inheritance. No existing file is modified.

**Tech Stack:** Python 3, numpy, shapely (all already in repo). Standard-library `heapq`/`itertools` for the focal queues. pytest for tests.

## Global Constraints

- **Do NOT modify existing code.** `core/`, `config.py`, `tests/`, `requirements.txt` stay untouched. `ml_planner/` may `import` and reuse them read-only.
- **ε bound = 5%**, configured as `FOCAL_EPS = 0.05` (⇒ focal weight `w = 1 + FOCAL_EPS = 1.05`). The returned path cost must be `≤ (1 + FOCAL_EPS) × optimal`, a **provable** ceiling verified on the benchmark.
- **Units:** meters and radians. Search cost = **polyline length** (sum of straight chords between waypoints), identical to the base planner.
- **Infinite map:** never assume `map_bounds`; the operating area is `safezones` only (may be absent). Reuse the base planner's `_in_bounds`/`_check_collision` as-is.
- **Safety** (collision avoidance) is always enforced by the base `_check_collision` (exact, zero-tolerance). No Phase-1 component sits in the safety loop.
- **Tests** live in `ml_planner/tests/` and run with `python -m pytest ml_planner/tests -v` from the repo root (`pytest.ini` provides `pythonpath = .`; file glob `*_test.py` matches). Do not add files under the existing `tests/`.
- **Commit trailer:** every commit message ends with a blank line then `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.

---

## File Structure

- `ml_planner/__init__.py` — marks the package (empty).
- `ml_planner/config.py` — Phase-1 constants (`FOCAL_EPS`, and Phase-2 placeholders `GRID_RES`, `MODEL_PATH`). One responsibility: tunables, separate from the root `config.py`.
- `ml_planner/secondary.py` — the hand-crafted secondary heuristic (`handcrafted_secondary`). Pure geometry, no planner state.
- `ml_planner/focal_astar.py` — `FocalKinodynamicAstar(KinodynamicAstar)`: constructor, `secondary_h`, `_goal_reached`, and the `search()` override.
- `ml_planner/plan.py` — `plan_trajectory_focal()` (thin wrapper mirroring `core.kinodynamic_astar.plan_trajectory`) and `path_length()` helper.
- `ml_planner/run_ml.py` — A/B benchmark harness (base vs focal) over seeds.
- `ml_planner/tests/secondary_test.py`, `ml_planner/tests/focal_astar_test.py`, `ml_planner/tests/plan_test.py` — the test suites.
- `ml_planner/requirements-ml.txt` — extra deps for the ML pipeline (Phase 2: `onnxruntime`), declared but not needed for Phase 1 runtime.

---

### Task 1: Package scaffold + config

**Files:**
- Create: `ml_planner/__init__.py`
- Create: `ml_planner/config.py`
- Create: `ml_planner/requirements-ml.txt`
- Test: `ml_planner/tests/__init__.py`, `ml_planner/tests/config_test.py`

**Interfaces:**
- Produces: module `ml_planner.config` exposing `FOCAL_EPS: float = 0.05`, `FOCAL_WEIGHT: float = 1.05`, `GRID_RES: int = 256`, `MODEL_PATH: str`.

- [ ] **Step 1: Write the failing test**

Create `ml_planner/tests/__init__.py` (empty) and `ml_planner/tests/config_test.py`:

```python
import ml_planner.config as mlcfg


def test_focal_constants_present():
    assert mlcfg.FOCAL_EPS == 0.05
    assert abs(mlcfg.FOCAL_WEIGHT - (1.0 + mlcfg.FOCAL_EPS)) < 1e-12
    # Phase-2 placeholders exist so later tasks can import them.
    assert mlcfg.GRID_RES == 256
    assert isinstance(mlcfg.MODEL_PATH, str)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/config_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner'`.

- [ ] **Step 3: Write minimal implementation**

Create `ml_planner/__init__.py` (empty file).

Create `ml_planner/config.py`:

```python
"""Phase-1 (+ Phase-2 placeholder) tunables for the ml_planner variant.

Kept separate from the root config.py so the base planner is never touched.
"""

import os

# ====== FOCAL SEARCH (A*epsilon) ======
# Bounded-suboptimality factor: the returned path cost is guaranteed
# <= (1 + FOCAL_EPS) * optimal. 0.0 reproduces exact-optimal A*.
FOCAL_EPS = 0.05
FOCAL_WEIGHT = 1.0 + FOCAL_EPS

# ====== PHASE-2 PLACEHOLDERS (CNN guidance map; unused in Phase 1) ======
# Fixed grid resolution for the per-problem cost-to-go field.
GRID_RES = 256
# Path to the exported ONNX guidance model (produced off-machine on Colab).
# Missing file => planner falls back to the hand-crafted secondary heuristic.
MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "guidance.onnx")
```

Create `ml_planner/requirements-ml.txt`:

```
# Extra dependencies for the ml_planner ML pipeline.
# Phase 1 needs none of these at runtime (pure numpy/shapely, already in
# the root requirements.txt). Listed here for Phase 2:
#   - inference (this machine, CPU): onnxruntime
#   - training  (off-machine, Colab/GPU): torch  (installed only on Colab)
onnxruntime>=1.16
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/config_test.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add ml_planner/__init__.py ml_planner/config.py ml_planner/requirements-ml.txt ml_planner/tests/__init__.py ml_planner/tests/config_test.py
git commit -m "feat(ml_planner): scaffold package and Phase-1 config

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: Hand-crafted secondary heuristic

**Files:**
- Create: `ml_planner/secondary.py`
- Test: `ml_planner/tests/secondary_test.py`

**Interfaces:**
- Produces: `handcrafted_secondary(waypoint, goal_wp, circle_obstacles, block_penalty=1.5) -> float`, where `waypoint`/`goal_wp` are `(x, y)` tuples and `circle_obstacles` is a list of `((cx, cy), radius)` (exactly the shape of `preprocessed['circle_obstacles']`). Returns a cost-to-go estimate in meters (not admissible — a ranking signal only).

- [ ] **Step 1: Write the failing test**

Create `ml_planner/tests/secondary_test.py`:

```python
import math
from ml_planner.secondary import handcrafted_secondary


def test_clear_line_equals_euclid():
    # No obstacles between waypoint and goal -> pure Euclid distance.
    d = handcrafted_secondary((0.0, 0.0), (100.0, 0.0), [])
    assert abs(d - 100.0) < 1e-9


def test_blocking_circle_inflates_estimate():
    # A circle straddling the straight line adds a detour penalty.
    obstacles = [((50.0, 0.0), 10.0)]
    blocked = handcrafted_secondary((0.0, 0.0), (100.0, 0.0), obstacles)
    clear = handcrafted_secondary((0.0, 0.0), (100.0, 0.0), [])
    assert blocked > clear


def test_offline_circle_does_not_penalize():
    # A circle far from the line must not change the estimate.
    obstacles = [((50.0, 10_000.0), 10.0)]
    d = handcrafted_secondary((0.0, 0.0), (100.0, 0.0), obstacles)
    assert abs(d - 100.0) < 1e-9
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/secondary_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner.secondary'`.

- [ ] **Step 3: Write minimal implementation**

Create `ml_planner/secondary.py`:

```python
"""Hand-crafted secondary heuristic for focal search (Phase 1).

The secondary heuristic ranks nodes inside the FOCAL band; it need NOT be
admissible (the admissible Euclid on OPEN keeps the epsilon bound). This
one is Euclid-to-goal inflated when the straight waypoint->goal line is
blocked by an inflated circle, biasing expansion toward states that have a
clear shot at the goal. O(N) in the number of circles.
"""

import math


def _seg_point_dist_sq(px, py, ax, ay, bx, by):
    """Squared distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    sx, sy = bx - ax, by - ay
    dd = sx * sx + sy * sy
    if dd == 0.0:
        rx, ry = px - ax, py - ay
        return rx * rx + ry * ry
    t = ((px - ax) * sx + (py - ay) * sy) / dd
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    ex = (ax + t * sx) - px
    ey = (ay + t * sy) - py
    return ex * ex + ey * ey


def handcrafted_secondary(waypoint, goal_wp, circle_obstacles, block_penalty=1.5):
    """Cost-to-go estimate (meters): Euclid to goal plus a rough detour
    penalty for each inflated circle the straight line to the goal crosses."""
    px, py = waypoint
    gx, gy = goal_wp
    base = math.hypot(gx - px, gy - py)
    blocked = 0.0
    for (cx, cy), r in circle_obstacles:
        if _seg_point_dist_sq(cx, cy, px, py, gx, gy) < r * r:
            blocked += r
    return base + block_penalty * blocked
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/secondary_test.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add ml_planner/secondary.py ml_planner/tests/secondary_test.py
git commit -m "feat(ml_planner): hand-crafted secondary heuristic

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: Focal planner subclass skeleton

**Files:**
- Create: `ml_planner/focal_astar.py`
- Test: `ml_planner/tests/focal_astar_test.py`

**Interfaces:**
- Consumes: `core.kinodynamic_astar.KinodynamicAstar` and module-level `_angle_diff`; `core.spatial_utils as su`; `ml_planner.config`; `ml_planner.secondary.handcrafted_secondary`.
- Produces: class `FocalKinodynamicAstar(KinodynamicAstar)` with:
  - `__init__(self, preprocessed_scenario, focal_eps=None, secondary=None)` — `secondary` is `Callable[[State], float]` or `None` (⇒ hand-crafted default).
  - `secondary_h(self, state) -> float`.
  - `_goal_reached(self, current) -> list | None` — returns the reconstructed path when `current` is an accepted goal arrival, else `None`.
  - `search(self)` — added in Task 4.

- [ ] **Step 1: Write the failing test**

Create `ml_planner/tests/focal_astar_test.py`:

```python
import math

import core.map_generator as mg
import core.preprocessing as prep
from ml_planner.focal_astar import FocalKinodynamicAstar


def _prep(scenario_func):
    return prep.prepare_scenario(scenario_func())


def test_instanti_and_secondary_default():
    pre = _prep(mg.scenario2_single_obstacle)
    planner = FocalKinodynamicAstar(pre, focal_eps=0.05)
    # secondary_h falls back to hand-crafted: finite, >= straight-line to goal.
    start = planner.start_state
    val = planner.secondary_h(start)
    gwp = planner.goal_state.waypoint
    euclid = math.hypot(gwp[0] - start.waypoint[0], gwp[1] - start.waypoint[1])
    assert math.isfinite(val)
    assert val >= euclid - 1e-6


def test_custom_secondary_used():
    pre = _prep(mg.scenario1_open_ocean)
    planner = FocalKinodynamicAstar(pre, focal_eps=0.0, secondary=lambda st: 42.0)
    assert planner.secondary_h(planner.start_state) == 42.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/focal_astar_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner.focal_astar'`.

- [ ] **Step 3: Write minimal implementation**

Create `ml_planner/focal_astar.py`:

```python
"""Focal (A*epsilon) variant of the Kinodynamic A* planner.

Subclasses the base planner and overrides only search() to expand the
FOCAL-best node (minimum secondary heuristic) while an admissible Euclid
OPEN guarantees the (1 + focal_eps) bound. All geometry, collision, arc-hop,
smoothing, and start-corner logic is inherited unchanged.
"""

import heapq
import itertools
import math
import time

import config
import core.spatial_utils as su
from core.kinodynamic_astar import KinodynamicAstar, _angle_diff

import ml_planner.config as mlcfg
from ml_planner.secondary import handcrafted_secondary


class FocalKinodynamicAstar(KinodynamicAstar):
    def __init__(self, preprocessed_scenario, focal_eps=None, secondary=None):
        super().__init__(preprocessed_scenario)
        self.focal_eps = mlcfg.FOCAL_EPS if focal_eps is None else focal_eps
        self._secondary = secondary  # Callable[[State], float] or None

    def secondary_h(self, state):
        """Ranking heuristic for FOCAL (need not be admissible)."""
        if self._secondary is not None:
            return self._secondary(state)
        return handcrafted_secondary(
            state.waypoint,
            self.goal_state.waypoint,
            self.scenario['circle_obstacles'],
        )

    def _goal_reached(self, current):
        """Return the reconstructed path if `current` is an accepted goal
        arrival, else None. Mirrors the base search()'s goal-acceptance rules
        (free run-in >= DSS, or aligned arrival within alpha_max)."""
        dist = math.hypot(
            current.waypoint[0] - self.goal_state.waypoint[0],
            current.waypoint[1] - self.goal_state.waypoint[1],
        )
        if dist >= config.GOAL_THRESHOLD:
            return None
        if self._free_goal:
            if current.parent is not None:
                seg = math.dist(current.parent.waypoint, current.waypoint)
                bearing = su.angle_to_heading(current.parent.waypoint, current.waypoint)
                turn_at_prev = abs(_angle_diff(bearing, current.parent.heading))
                usable = seg - self.R * math.tan(turn_at_prev / 2.0)
                if usable >= self._dss - config.EPS:
                    return self._reconstruct_path(current)
            return None
        approach_turn = abs(_angle_diff(self.goal_state.heading, current.heading))
        if approach_turn <= self.alpha_max_rad:
            return self._reconstruct_path(current)
        return None
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/focal_astar_test.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add ml_planner/focal_astar.py ml_planner/tests/focal_astar_test.py
git commit -m "feat(ml_planner): focal planner subclass skeleton

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: Focal `search()` override

**Files:**
- Modify: `ml_planner/focal_astar.py` (add `search()` method to the class)
- Test: `ml_planner/tests/focal_astar_test.py` (add invariance + bound tests)

**Interfaces:**
- Consumes: inherited `self.start_corners`, `self.heuristic`, `self.get_next_states`, `self.g_scores`, `self.closed_set`, `self.goal_state`, `self.max_iterations`, `self.num_strategy_b`, `self.iteration_count`, `self.search_failed`, `self._reconstruct_path`; `self.secondary_h`, `self._goal_reached` (Task 3).
- Produces: `FocalKinodynamicAstar.search(self) -> list | None` — same return contract as the base `search()` (path list of `(waypoint, heading)`, or `None`; sets `self.search_failed`).

- [ ] **Step 1: Write the failing test**

Add to `ml_planner/tests/focal_astar_test.py`:

```python
import core.kinodynamic_astar as astar


def _path_len(path):
    total = 0.0
    for (a, _), (b, _) in zip(path, path[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def _optimal_cost(scenario_func):
    pre = prep.prepare_scenario(scenario_func())
    res = astar.plan_trajectory(pre, verbose=False)
    assert res['success'], "base planner must solve the fixture"
    return _path_len(res['path'])


def test_eps_zero_matches_optimal_cost():
    # focal_eps=0 with a Euclid secondary must reproduce the optimal cost.
    scen = mg.scenario2_single_obstacle
    opt = _optimal_cost(scen)
    pre = prep.prepare_scenario(scen())
    gwp = pre['goal_state']['waypoint']
    planner = FocalKinodynamicAstar(
        pre, focal_eps=0.0,
        secondary=lambda st: math.hypot(st.waypoint[0] - gwp[0], st.waypoint[1] - gwp[1]),
    )
    path = planner.search()
    assert path is not None
    assert abs(_path_len(path) - opt) < 1.0  # meters; both optimal


def test_focal_respects_epsilon_bound():
    # focal_eps=0.05 with the hand-crafted secondary: cost <= 1.05 * optimal.
    scen = mg.scenario2_single_obstacle
    opt = _optimal_cost(scen)
    pre = prep.prepare_scenario(scen())
    planner = FocalKinodynamicAstar(pre, focal_eps=0.05)
    path = planner.search()
    assert path is not None
    assert _path_len(path) <= 1.05 * opt + 1e-6
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/focal_astar_test.py -v`
Expected: FAIL — `AttributeError`/`TypeError` because `FocalKinodynamicAstar.search` is not yet defined (it would call the base `search()`, which ignores the secondary; `test_eps_zero_matches_optimal_cost` may still pass, but `search` must be the override — expect at least the bound test to exercise focal). If both pass by accident against the base search, proceed anyway; Step 3 installs the real override.

- [ ] **Step 3: Write minimal implementation**

Append this method inside the `FocalKinodynamicAstar` class in `ml_planner/focal_astar.py`:

```python
    def search(self):
        """Focal (A*epsilon) search. OPEN is ordered by the admissible
        f = g + h_euclid (weight 1) so f_min bounds the optimum; FOCAL holds
        every live OPEN node with f <= w * f_min and is expanded by minimum
        secondary_h. Guarantees returned cost <= w * optimal."""
        _start = time.perf_counter()
        _budget = config.TIME_BUDGET_S
        w = 1.0 + self.focal_eps

        if not self.start_corners:
            self.search_failed = True
            return None

        counter = itertools.count()
        open_heap = []      # (f, count, state) — all inserted OPEN nodes
        focal_heap = []     # (secondary, count, state) — nodes with f <= w*f_min
        in_focal = set()    # id(state) currently pushed to focal_heap
        self.open_set = open_heap  # keep get_search_stats() meaningful

        for corner in self.start_corners:
            corner.h_cost = self.heuristic(corner, self.goal_state)
            if corner.g_cost < self.g_scores[corner]:
                self.g_scores[corner] = corner.g_cost
            heapq.heappush(open_heap, (corner.g_cost + corner.h_cost, next(counter), corner))

        def _is_live(state):
            return (state not in self.closed_set and
                    state.g_cost <= self.g_scores.get(state, float('inf')))

        def _clean_open_top():
            while open_heap and not _is_live(open_heap[0][2]):
                heapq.heappop(open_heap)

        def _refill_focal(f_bound):
            for f, c, st in open_heap:
                if f <= f_bound and id(st) not in in_focal and _is_live(st):
                    heapq.heappush(focal_heap, (self.secondary_h(st), c, st))
                    in_focal.add(id(st))

        _clean_open_top()
        f_min = open_heap[0][0] if open_heap else None
        if f_min is not None:
            _refill_focal(w * f_min)

        while open_heap and self.iteration_count < self.max_iterations:
            if _budget is not None and (time.perf_counter() - _start) > _budget:
                break
            self.iteration_count += 1

            # Select the FOCAL-best live node; if FOCAL drained, refill and retry.
            current = None
            while focal_heap:
                _, _, cand = heapq.heappop(focal_heap)
                in_focal.discard(id(cand))
                if _is_live(cand):
                    current = cand
                    break
            if current is None:
                _clean_open_top()
                if not open_heap:
                    break
                f_min = open_heap[0][0]
                _refill_focal(w * f_min)
                continue

            self.closed_set.add(current)

            # Escape-valve re-arm (mirrors base search): give the fan a fresh
            # budget as a last resort when the frontier is nearly dead.
            if len(open_heap) <= 1 and self.num_strategy_b <= 0:
                self.num_strategy_b = config.NUM_STRATEGY_B

            reached = self._goal_reached(current)
            if reached is not None:
                return reached

            for next_state, transition_cost in self.get_next_states(current):
                if next_state in self.closed_set:
                    continue
                tentative_g = self.g_scores[current] + transition_cost
                if tentative_g < self.g_scores.get(next_state, float('inf')):
                    next_state.parent = current
                    self.g_scores[next_state] = tentative_g
                    next_state.g_cost = tentative_g
                    next_state.h_cost = self.heuristic(next_state, self.goal_state)
                    f = tentative_g + next_state.h_cost
                    c = next(counter)
                    heapq.heappush(open_heap, (f, c, next_state))
                    if f_min is not None and f <= w * f_min:
                        heapq.heappush(focal_heap, (self.secondary_h(next_state), c, next_state))
                        in_focal.add(id(next_state))

            # Update f_min after expansion; widen FOCAL if it rose.
            _clean_open_top()
            if open_heap:
                new_fmin = open_heap[0][0]
                if f_min is None or new_fmin > f_min:
                    f_min = new_fmin
                    _refill_focal(w * f_min)
            else:
                f_min = None

        self.search_failed = True
        return None
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/focal_astar_test.py -v`
Expected: PASS (4 tests: the 2 from Task 3 plus the 2 new).

- [ ] **Step 5: Commit**

```bash
git add ml_planner/focal_astar.py ml_planner/tests/focal_astar_test.py
git commit -m "feat(ml_planner): focal A*epsilon search override with bound tests

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 5: `plan_trajectory_focal` wrapper + `path_length`

**Files:**
- Create: `ml_planner/plan.py`
- Test: `ml_planner/tests/plan_test.py`

**Interfaces:**
- Consumes: `FocalKinodynamicAstar` (Task 4).
- Produces:
  - `path_length(path) -> float` — sum of chord lengths of a `[(waypoint, heading), ...]` path.
  - `plan_trajectory_focal(preprocessed_scenario, focal_eps=None, secondary=None, verbose=False) -> dict` with keys `path`, `success`, `stats`, `planner` (mirrors `core.kinodynamic_astar.plan_trajectory`).

- [ ] **Step 1: Write the failing test**

Create `ml_planner/tests/plan_test.py`:

```python
import core.map_generator as mg
import core.preprocessing as prep
from ml_planner.plan import plan_trajectory_focal, path_length


def test_plan_trajectory_focal_solves_open_ocean():
    pre = prep.prepare_scenario(mg.scenario1_open_ocean())
    res = plan_trajectory_focal(pre)
    assert res['success'] is True
    assert res['path'] is not None
    assert path_length(res['path']) > 0.0
    assert 'stats' in res and 'planner' in res


def test_path_length_of_two_points():
    path = [((0.0, 0.0), 0.0), ((3.0, 4.0), 0.0)]
    assert abs(path_length(path) - 5.0) < 1e-9
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/plan_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner.plan'`.

- [ ] **Step 3: Write minimal implementation**

Create `ml_planner/plan.py`:

```python
"""High-level entry point for the focal planner variant.

Mirrors core.kinodynamic_astar.plan_trajectory but drives
FocalKinodynamicAstar. Does not modify the base module.
"""

import math

from ml_planner.focal_astar import FocalKinodynamicAstar


def path_length(path):
    """Total polyline length (meters) of a [(waypoint, heading), ...] path."""
    total = 0.0
    for (a, _), (b, _) in zip(path, path[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def plan_trajectory_focal(preprocessed_scenario, focal_eps=None, secondary=None, verbose=False):
    """Plan a trajectory with focal (A*epsilon) search.

    Returns a dict with 'path', 'success', 'stats', 'planner'. Success means
    the fixed takeoff/approach legs are clear AND a body path was found, the
    same contract as the base plan_trajectory.
    """
    planner = FocalKinodynamicAstar(preprocessed_scenario, focal_eps=focal_eps, secondary=secondary)

    legs_ok = planner._check_fixed_legs()
    path = None
    if legs_ok:
        if verbose:
            print("Starting focal A* search...")
        path = planner.search()
        if verbose:
            stats = planner.get_search_stats()
            print(f"Focal search: {stats['iterations']}/{stats['max_iterations']} iterations")
            print("Path found" if path else "No path found")

    if path:
        path = planner.smooth_path(path)

    return {
        'path': path,
        'success': path is not None and legs_ok,
        'stats': planner.get_search_stats(),
        'planner': planner,
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/plan_test.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add ml_planner/plan.py ml_planner/tests/plan_test.py
git commit -m "feat(ml_planner): plan_trajectory_focal wrapper and path_length

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 6: A/B benchmark harness

**Files:**
- Create: `ml_planner/run_ml.py`
- Test: `ml_planner/tests/run_ml_test.py`

**Interfaces:**
- Consumes: `plan_trajectory_focal`, `path_length` (Task 5); `core.kinodynamic_astar.plan_trajectory`; `batch_random_test.generate_random_scenario`; `core.preprocessing`.
- Produces: `compare_seed(seed, focal_eps=None) -> dict` with keys `seed`, `base_success`, `focal_success`, `base_cost`, `focal_cost`, `base_iters`, `focal_iters`, `base_time`, `focal_time`, `cost_ratio`, `within_bound` (bool: `focal_cost <= (1+focal_eps)*base_cost` when both succeed, else `True`). And `run_benchmark(seeds, focal_eps=None) -> list[dict]`.

- [ ] **Step 1: Write the failing test**

Create `ml_planner/tests/run_ml_test.py`:

```python
from ml_planner.run_ml import compare_seed, run_benchmark


def test_compare_seed_reports_bound_and_keys():
    row = compare_seed(1, focal_eps=0.05)
    for key in ('seed', 'base_success', 'focal_success', 'base_cost',
                'focal_cost', 'cost_ratio', 'within_bound',
                'base_iters', 'focal_iters', 'base_time', 'focal_time'):
        assert key in row
    # Whenever both solve, the epsilon bound must hold.
    if row['base_success'] and row['focal_success']:
        assert row['within_bound'] is True


def test_run_benchmark_multiple_seeds():
    rows = run_benchmark([1, 2], focal_eps=0.05)
    assert len(rows) == 2
    assert all(r['within_bound'] for r in rows)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest ml_planner/tests/run_ml_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ml_planner.run_ml'`.

- [ ] **Step 3: Write minimal implementation**

Create `ml_planner/run_ml.py`:

```python
"""A/B benchmark: base optimal A* vs focal (A*epsilon) planner.

Reuses the deterministic random-scenario generator from batch_random_test so
comparisons run on identical maps. For each seed it records success, path
cost, iterations, and wall time for both planners, and verifies the focal
path never exceeds the (1 + focal_eps) bound.

Usage:  python -m ml_planner.run_ml            # default seeds 0..49
        python -m ml_planner.run_ml 200        # seeds 0..199
"""

import sys
import time

import core.preprocessing as prep
import core.kinodynamic_astar as astar
from batch_random_test import generate_random_scenario
from ml_planner.plan import plan_trajectory_focal, path_length
import ml_planner.config as mlcfg


def _timed(fn):
    t0 = time.perf_counter()
    res = fn()
    return res, time.perf_counter() - t0


def compare_seed(seed, focal_eps=None):
    eps = mlcfg.FOCAL_EPS if focal_eps is None else focal_eps
    scenario = generate_random_scenario(seed=seed)
    pre = prep.prepare_scenario(scenario)

    base_res, base_time = _timed(lambda: astar.plan_trajectory(pre, verbose=False))
    # Re-preprocess so the two runs never share mutable state.
    pre2 = prep.prepare_scenario(scenario)
    focal_res, focal_time = _timed(lambda: plan_trajectory_focal(pre2, focal_eps=eps))

    base_ok = base_res['success']
    focal_ok = focal_res['success']
    base_cost = path_length(base_res['path']) if base_ok else float('nan')
    focal_cost = path_length(focal_res['path']) if focal_ok else float('nan')
    cost_ratio = (focal_cost / base_cost) if (base_ok and focal_ok and base_cost > 0) else float('nan')
    within = True
    if base_ok and focal_ok and base_cost > 0:
        within = focal_cost <= (1.0 + eps) * base_cost + 1e-6

    return {
        'seed': seed,
        'base_success': base_ok,
        'focal_success': focal_ok,
        'base_cost': base_cost,
        'focal_cost': focal_cost,
        'cost_ratio': cost_ratio,
        'within_bound': within,
        'base_iters': base_res['stats']['iterations'],
        'focal_iters': focal_res['stats']['iterations'],
        'base_time': base_time,
        'focal_time': focal_time,
    }


def run_benchmark(seeds, focal_eps=None):
    return [compare_seed(s, focal_eps=focal_eps) for s in seeds]


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 50
    rows = run_benchmark(range(n))
    solved = [r for r in rows if r['base_success'] and r['focal_success']]
    viol = [r for r in rows if not r['within_bound']]
    print(f"seeds={n}  both-solved={len(solved)}  bound-violations={len(viol)}")
    if solved:
        avg_ratio = sum(r['cost_ratio'] for r in solved) / len(solved)
        base_it = sum(r['base_iters'] for r in solved)
        focal_it = sum(r['focal_iters'] for r in solved)
        base_t = sum(r['base_time'] for r in solved)
        focal_t = sum(r['focal_time'] for r in solved)
        print(f"avg cost ratio (focal/base) = {avg_ratio:.4f}")
        print(f"total iterations: base={base_it} focal={focal_it}  "
              f"({100.0 * (1 - focal_it / base_it):.1f}% fewer)" if base_it else "")
        print(f"total time (s):  base={base_t:.2f} focal={focal_t:.2f}  "
              f"({100.0 * (1 - focal_t / base_t):.1f}% faster)" if base_t else "")
    if viol:
        print(f"WARNING: {len(viol)} seeds violated the epsilon bound: "
              f"{[r['seed'] for r in viol]}")


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest ml_planner/tests/run_ml_test.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Run the benchmark and eyeball the win**

Run: `python -m ml_planner.run_ml 50`
Expected: a summary line with `bound-violations=0`, an `avg cost ratio` ≤ 1.05, and (hopefully) fewer focal iterations / less time. Record the numbers in the commit message.

- [ ] **Step 6: Commit**

```bash
git add ml_planner/run_ml.py ml_planner/tests/run_ml_test.py
git commit -m "feat(ml_planner): A/B benchmark harness (base vs focal)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

## Self-Review

**Spec coverage (Phase 1 scope):**
- Focal search / A*ε with Euclid admissible OPEN + secondary FOCAL, ε=5% → Tasks 3–4.
- Isolation in `ml_planner/`, no base edits, subclass reuse → Tasks 1–5.
- Compatibility flag (`FOCAL_EPS=0`, secondary=Euclid == base) → Task 4 `test_eps_zero_matches_optimal_cost`.
- Hand-crafted secondary (Phase-1 baseline) → Task 2.
- Hard-verify ε bound on the benchmark → Task 4 tests + Task 6 `within_bound`.
- Success-rate parity + speed measurement → Task 6.
- Phase-2 placeholders (`GRID_RES`, `MODEL_PATH`, `onnxruntime`) so the ML plan slots in → Task 1.
- **Deferred to the Phase-2 plan** (out of scope here): `guidance.py`/ONNX, `dataset_gen.py`, `train/train_guidance.ipynb`, wiring the CNN secondary. Noted in the spec §6/§9.

**Placeholder scan:** No TBD/TODO; every code step contains complete code; every command lists expected output.

**Type consistency:** `secondary` is `Callable[[State], float]` in Tasks 3–5; `handcrafted_secondary(waypoint, goal_wp, circle_obstacles, block_penalty=1.5)` used consistently in Tasks 2–3; `plan_trajectory_focal(...)` return dict keys (`path`/`success`/`stats`/`planner`) match Task 5 and are consumed in Task 6; `path_length` signature identical in Tasks 5–6.

---

## Notes for the implementer

- Run the whole suite any time with: `python -m pytest ml_planner/tests -v`.
- The base planner's `TIME_BUDGET_S = 5` applies to focal too (inherited). Small fixtures solve well under it.
- If `test_eps_zero_matches_optimal_cost` shows a nonzero gap, suspect FOCAL tie-breaking vs OPEN — with `eps=0` FOCAL should contain only `f == f_min` nodes; check `_refill_focal(w*f_min)` uses `w = 1.0`.
- `run_ml.py` re-preprocesses the scenario per planner (`pre` vs `pre2`) on purpose: `prepare_scenario` output is consumed mutably by the planner, so sharing it would cross-contaminate the A/B.
