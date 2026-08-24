# Goal-Distance-Field Heuristic Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the pure-Euclid A\* heuristic with a provably admissible goal-rooted grid distance field that prices obstacles and the safezone, cutting expansions on occluded scenarios with zero optimality loss.

**Architecture:** New `core/heuristic_field.py` builds, once per planner, a coarse under-blocked occupancy grid over the safezone/map bbox, runs one multi-source Dijkstra (goal + Euclid-seeded border) via `scipy.sparse.csgraph`, and answers O(1) reverse-triangle lower-bound queries. `KinodynamicAstar` gets a ~10-line hook: build the field only when every start corner's chord to the goal is blocked, and take `max(Euclid, field.query(p))` in `heuristic()`.

**Tech Stack:** numpy, scipy (`sparse.csgraph.dijkstra`), shapely 2.x (`buffer`, `contains_xy`) — all already in `requirements.txt`. pytest for tests.

**Spec:** `docs/superpowers/specs/2026-07-18-goal-distance-field-heuristic-design.md`

## Global Constraints

- Admissibility is the contract: every construction choice must only UNDER-estimate true remaining cost. Never interpolate grid values bilinearly; never skip the `/1.0824` stretch factor or the 2-cell slack.
- The heuristic must never fail a plan: the field build is wrapped in `try/except Exception` → fall back to `None` (pure Euclid).
- Do not touch successor generation, đoản-trình, collision checks, goal acceptance, or smoothing.
- `config.HEURISTIC_GRID_N = 256` is the only new config constant.
- The working tree contains unrelated WIP (valve/fan fixes and their tests). `git add` ONLY the files named in each task's commit step — never `git add -A` / `git add .`.
- Test files are named `*_test.py` (committed); `test_*.py` is gitignored scratch.
- Run tests with `python -m pytest -q ...` from the repo root.
- Suite baseline: 18 pre-existing failures (recorded in scratchpad `pytest_before.txt`-style list — `kinodynamic_arc_hop_test` reason-plumbing + `oracle_validity_test`). A task is green when it adds NO NEW failures.

---

### Task 1: `GoalDistanceField` — grid, blocking, Dijkstra, query (circle admissibility)

**Files:**
- Create: `core/heuristic_field.py`
- Modify: `config.py` (add `HEURISTIC_GRID_N`)
- Test: `tests/heuristic_field_test.py`

**Interfaces:**
- Consumes: the *preprocessed* scenario dict from `core.preprocessing.prepare_scenario` (keys used: `goal_state['waypoint']`, `circle_obstacles`, `polygon_obstacles`, `safezones` (optional), `map_bounds` (optional), `start_pos`).
- Produces: `GoalDistanceField(pre)` with method `query(p: (x, y)) -> float` returning an admissible lower bound on the remaining continuous distance from `p` to the goal waypoint, or `-inf` when the grid has no information (caller must `max` with Euclid). Constructor may raise; callers catch.

- [ ] **Step 1: Add the config constant**

In `config.py`, in the `# ====== A* SEARCH ======` section (after the `NUM_START_CORNERS` block), add:

```python
# Grid resolution (cells on the long side) for the admissible goal-distance
# field heuristic (core/heuristic_field.py). The field only ever tightens h
# via max(euclid, field), so a coarser grid degrades toward plain Euclid.
HEURISTIC_GRID_N = 256
```

- [ ] **Step 2: Write the failing admissibility test (single circle)**

Create `tests/heuristic_field_test.py`:

```python
"""Tests for the admissible goal-distance-field heuristic."""
import math

import pytest

import config
import core.preprocessing as prep
from core.heuristic_field import GoalDistanceField

CENTER = (250000.0, 250000.0)
RAW_R = 30000.0


def circle_scenario():
    """One raw circle dead-center between a west start and an east goal."""
    return {
        'start': (50000.0, 250000.0), 'start_heading': 0.0,
        'goal': (450000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [],
        'dynamic_obstacles': [(CENTER, RAW_R)],
        'obstacles': [{'type': 'circle', 'center': CENTER, 'radius': RAW_R}],
        'map_bounds': (500000.0, 500000.0),
    }


def shortest_around_circle(p, q, c, r):
    """Exact continuous shortest distance p->q avoiding one disc (c, r)."""
    d1, d2 = math.dist(p, c), math.dist(q, c)
    # segment p-q vs disc
    px, py = p; qx, qy = q; cx, cy = c
    dx, dy = qx - px, qy - py
    t = max(0.0, min(1.0, ((cx - px) * dx + (cy - py) * dy) / (dx * dx + dy * dy)))
    if math.dist((px + t * dx, py + t * dy), c) >= r:
        return math.dist(p, q)
    tang = math.sqrt(max(d1 * d1 - r * r, 0.0)) + math.sqrt(max(d2 * d2 - r * r, 0.0))
    ang = (math.acos(max(-1, min(1, ((px - cx) * (qx - cx) + (py - cy) * (qy - cy)) / (d1 * d2))))
           - math.acos(max(-1, min(1, r / d1))) - math.acos(max(-1, min(1, r / d2))))
    return tang + r * max(ang, 0.0)


def test_circle_field_is_admissible_lower_bound():
    pre = prep.prepare_scenario(circle_scenario())
    field = GoalDistanceField(pre)
    goal = pre['goal_state']['waypoint']
    (c, r_inf), = pre['circle_obstacles']
    import random
    rng = random.Random(7)
    checked = 0
    for _ in range(300):
        p = (rng.uniform(5000, 495000), rng.uniform(5000, 495000))
        if math.dist(p, c) < r_inf + 1.0:
            continue                       # inside the inflated disc: no admissibility claim
        true_d = shortest_around_circle(p, goal, c, r_inf)
        assert field.query(p) <= true_d + 1.0, f"h violates lower bound at {p}"
        checked += 1
    assert checked > 200


def test_query_outside_grid_returns_neg_inf():
    pre = prep.prepare_scenario(circle_scenario())
    field = GoalDistanceField(pre)
    assert field.query((-1e6, -1e6)) == -math.inf
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `python -m pytest -q tests/heuristic_field_test.py`
Expected: FAIL/ERROR with `ModuleNotFoundError: No module named 'core.heuristic_field'`

- [ ] **Step 4: Implement `core/heuristic_field.py`**

```python
"""Goal-rooted admissible distance field for the A* heuristic.

Precomputes, once per scenario, a coarse-grid lower bound on the continuous
obstacle-avoiding (and safezone-constrained) distance from any point to the
goal waypoint. Spec: docs/superpowers/specs/
2026-07-18-goal-distance-field-heuristic-design.md.

Every construction choice is biased so the result can only UNDER-estimate
the true remaining cost (admissibility):
- cells are blocked only when fully inside an obstacle (eroded by the cell
  half-diagonal) or fully outside the safezone (dilated by it);
- grid distances are divided by the 8-connectivity stretch factor and
  reduced by a 2-cell digitisation slack;
- the grid border is Euclid-seeded, so paths that leave the gridded area
  (unbounded worlds) are still lower-bounded;
- queries take the best reverse-triangle bound over the 4 surrounding cell
  centers (interpolated lower bounds are not lower bounds).
"""
import math

import numpy as np
import shapely
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from shapely.geometry import Polygon
from shapely.ops import unary_union

import config

# An 8-connected grid path exceeds the continuous shortest by <= this factor
# (away from cell-scale narrow passages; the query slack absorbs the rest).
_STRETCH = 1.0 / math.cos(math.pi / 8.0)


class GoalDistanceField:
    """Admissible lower-bound distance-to-goal field on a coarse grid."""

    def __init__(self, pre):
        gx, gy = pre['goal_state']['waypoint']
        self._goal = (float(gx), float(gy))
        x0, y0, w, h = self._extent(pre)
        cell = max(w, h) / int(config.HEURISTIC_GRID_N)
        self._x0, self._y0, self._cell = x0, y0, cell
        self._nx = max(4, int(math.ceil(w / cell)))
        self._ny = max(4, int(math.ceil(h / cell)))
        cx = x0 + (np.arange(self._nx) + 0.5) * cell
        cy = y0 + (np.arange(self._ny) + 0.5) * cell
        X, Y = np.meshgrid(cx, cy)                      # [iy, ix]
        self._blocked = self._block_cells(pre, X, Y, cell)
        self._d = self._solve(X, Y, self._blocked, cell)

    # ------------------------------------------------------------------
    def _extent(self, pre):
        """(x0, y0, width, height) of the gridded area. Safezone bbox wins;
        else the explicit map_bounds rectangle; else (permissive world) the
        obstacle/start/goal bbox padded 10% of its diagonal."""
        szs = pre.get('safezones')
        if szs:
            u = unary_union([Polygon(s) for s in szs])
            x0, y0, x1, y1 = u.bounds
            pad = max(x1 - x0, y1 - y0) / config.HEURISTIC_GRID_N
            return x0 - pad, y0 - pad, (x1 - x0) + 2 * pad, (y1 - y0) + 2 * pad
        mb = pre.get('map_bounds')
        if mb is not None:
            return 0.0, 0.0, float(mb[0]), float(mb[1])
        xs = [self._goal[0], pre['start_pos'][0]]
        ys = [self._goal[1], pre['start_pos'][1]]
        for (c, r) in pre['circle_obstacles']:
            xs += [c[0] - r, c[0] + r]
            ys += [c[1] - r, c[1] + r]
        for coords in pre['polygon_obstacles']:
            for (px, py) in coords:
                xs.append(px)
                ys.append(py)
        x0, x1 = min(xs), max(xs)
        y0, y1 = min(ys), max(ys)
        pad = 0.10 * math.hypot(x1 - x0, y1 - y0) + 1.0
        return x0 - pad, y0 - pad, (x1 - x0) + 2 * pad, (y1 - y0) + 2 * pad

    def _block_cells(self, pre, X, Y, cell):
        """Under-blocked occupancy: a cell is blocked only when its center is
        so deep inside an obstacle (or outside the safezone) that the WHOLE
        cell must be infeasible."""
        half = cell * math.sqrt(2.0) / 2.0
        blocked = np.zeros(X.shape, dtype=bool)
        for (c, r) in pre['circle_obstacles']:
            rr = r - half
            if rr > 0:
                blocked |= (X - c[0]) ** 2 + (Y - c[1]) ** 2 <= rr * rr
        for coords in pre['polygon_obstacles']:
            eroded = Polygon(coords).buffer(-half)
            if not eroded.is_empty:
                blocked |= shapely.contains_xy(eroded, X, Y)
        szs = pre.get('safezones')
        if szs:
            dilated = unary_union([Polygon(s) for s in szs]).buffer(half)
            blocked |= ~shapely.contains_xy(dilated, X, Y)
        return blocked

    def _solve(self, X, Y, blocked, cell):
        """One multi-source Dijkstra: a virtual super-source connects to the
        goal-neighborhood cells AND every border cell, each seeded with plain
        Euclid-to-goal (a universal lower bound, so leaving the grid stays
        sound). Returns d[iy, ix] (inf where unreachable/blocked)."""
        ny, nx = blocked.shape
        M = ny * nx
        idx = np.arange(M).reshape(ny, nx)
        free = ~blocked
        rows, cols, wts = [], [], []
        for dy, dx, wstep in ((0, 1, cell), (1, 0, cell),
                              (1, 1, cell * math.sqrt(2.0)),
                              (1, -1, cell * math.sqrt(2.0))):
            a_iy = slice(max(0, -dy), ny - max(0, dy))
            a_ix = slice(max(0, -dx), nx - max(0, dx))
            b_iy = slice(max(0, dy), ny - max(0, -dy))
            b_ix = slice(max(0, dx), nx - max(0, -dx))
            ok = free[a_iy, a_ix] & free[b_iy, b_ix]
            rows.append(idx[a_iy, a_ix][ok])
            cols.append(idx[b_iy, b_ix][ok])
            wts.append(np.full(rows[-1].shape, wstep))
        gx, gy = self._goal
        seeds = np.zeros(blocked.shape, dtype=bool)
        seeds[0, :] = seeds[-1, :] = True
        seeds[:, 0] = seeds[:, -1] = True
        gix = int((gx - self._x0) / cell)
        giy = int((gy - self._y0) / cell)
        seeds[max(0, giy - 2):giy + 3, max(0, gix - 2):gix + 3] = True
        seeds &= free
        siy, six = np.nonzero(seeds)
        rows.append(np.full(siy.shape, M))
        cols.append(idx[siy, six])
        wts.append(np.hypot(X[siy, six] - gx, Y[siy, six] - gy))
        graph = csr_matrix(
            (np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))),
            shape=(M + 1, M + 1))
        d = dijkstra(graph, directed=False, indices=M)
        return d[:M].reshape(ny, nx)

    # ------------------------------------------------------------------
    def query(self, p):
        """Admissible lower bound on the continuous distance p -> goal, or
        -inf when the grid has nothing sound to say (caller maxes with
        Euclid). Reverse-triangle over the 4 surrounding cell centers."""
        fx = (p[0] - self._x0) / self._cell - 0.5
        fy = (p[1] - self._y0) / self._cell - 0.5
        ix0 = int(math.floor(fx))
        iy0 = int(math.floor(fy))
        slack = 2.0 * self._cell
        best = -math.inf
        for iy in (iy0, iy0 + 1):
            if not 0 <= iy < self._ny:
                continue
            for ix in (ix0, ix0 + 1):
                if not 0 <= ix < self._nx:
                    continue
                if self._blocked[iy, ix]:
                    continue
                d = self._d[iy, ix]
                if not math.isfinite(d):
                    continue
                cx = self._x0 + (ix + 0.5) * self._cell
                cy = self._y0 + (iy + 0.5) * self._cell
                cand = d / _STRETCH - slack - math.hypot(p[0] - cx, p[1] - cy)
                if cand > best:
                    best = cand
        return best
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `python -m pytest -q tests/heuristic_field_test.py`
Expected: `2 passed` (runtime a few seconds — one field build + 300 queries).

- [ ] **Step 6: Commit**

```bash
git add core/heuristic_field.py config.py tests/heuristic_field_test.py
git commit -m "feat(core): admissible goal-distance-field (grid Dijkstra lower bound)"
```

---

### Task 2: Safezone corridor + permissive-world soundness

**Files:**
- Modify: `tests/heuristic_field_test.py` (append tests)
- (No production change expected — Task 1 already implements safezone/permissive paths; this task proves them.)

**Interfaces:**
- Consumes: `GoalDistanceField(pre)`, `query(p)` from Task 1.
- Produces: nothing new — locked behaviour for safezone and permissive modes.

- [ ] **Step 1: Write the failing/locking tests**

Append to `tests/heuristic_field_test.py`:

```python
def corridor_scenario():
    """L-shaped safezone; no map_bounds (production-style unbounded world).
    True paths must follow the L: ~490 km vs ~346 km Euclid."""
    r1 = [(0.0, 0.0), (300000.0, 0.0), (300000.0, 60000.0), (0.0, 60000.0)]
    r2 = [(240000.0, 0.0), (300000.0, 0.0), (300000.0, 300000.0), (240000.0, 300000.0)]
    return {
        'start': (30000.0, 30000.0), 'start_heading': 0.0,
        'goal': (270000.0, 280000.0), 'goal_heading': math.pi / 2,
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
        'safezones': [r1, r2],
    }


def test_corridor_field_prices_the_safezone_detour():
    pre = prep.prepare_scenario(corridor_scenario())
    field = GoalDistanceField(pre)
    goal = pre['goal_state']['waypoint']
    p = (30000.0, 30000.0)
    euclid = math.dist(p, goal)
    q = field.query(p)
    # adds value: the corridor forces a detour Euclid cannot see
    assert q >= 1.2 * euclid
    # stays admissible: generous hand bound on the true L-path length
    l_upper = (270000.0 - 30000.0) + (280000.0 - 30000.0) + 20000.0
    assert q <= l_upper


def test_permissive_world_border_seeding_is_sound():
    """No safezone, no map_bounds: a path may leave the gridded area, so the
    field must never exceed Euclid-through-the-border alternatives."""
    scn = circle_scenario()
    del scn['map_bounds']
    pre = prep.prepare_scenario(scn)
    field = GoalDistanceField(pre)
    goal = pre['goal_state']['waypoint']
    (c, r_inf), = pre['circle_obstacles']
    import random
    rng = random.Random(11)
    for _ in range(200):
        p = (rng.uniform(-100000, 600000), rng.uniform(-100000, 600000))
        if math.dist(p, c) < r_inf + 1.0:
            continue
        true_d = shortest_around_circle(p, goal, c, r_inf)
        assert field.query(p) <= true_d + 1.0
```

- [ ] **Step 2: Run the new tests**

Run: `python -m pytest -q tests/heuristic_field_test.py`
Expected: all pass (Task 1 code already handles both modes). If
`test_corridor_field_prices_the_safezone_detour` FAILS on the `safezones`
key: check with `grep -n "safezones" core/preprocessing.py` that
`prepare_scenario` carries the key into its output dict (the planner's own
`__init__` reads `preprocessed_scenario.get('safezones')`, so it should);
if it does not, pass the raw scenario's `safezones` through in
`prepare_scenario` (one line) and re-run.

- [ ] **Step 3: Commit**

```bash
git add tests/heuristic_field_test.py
git commit -m "test(core): corridor + permissive-world soundness for the goal field"
```

---

### Task 3: Planner hook — gating, `max(Euclid, field)`, fallback

**Files:**
- Modify: `core/kinodynamic_astar.py` (imports, `__init__`, `heuristic()`)
- Test: `tests/heuristic_field_test.py` (append)

**Interfaces:**
- Consumes: `GoalDistanceField` from Task 1.
- Produces: `KinodynamicAstar._goal_field` attribute (`GoalDistanceField | None`); `heuristic()` returns `max(euclid, field.query(wp))` when the field exists. The name `GoalDistanceField` must be importable AT MODULE LEVEL in `core.kinodynamic_astar` (tests monkeypatch `astar.GoalDistanceField`).

- [ ] **Step 1: Write the failing tests**

Append to `tests/heuristic_field_test.py`:

```python
import core.kinodynamic_astar as astar


def open_water_scenario():
    return {
        'start': (50000.0, 250000.0), 'start_heading': 0.0,
        'goal': (450000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
        'map_bounds': (500000.0, 500000.0),
    }


def test_gating_open_water_builds_no_field():
    planner = astar.KinodynamicAstar(prep.prepare_scenario(open_water_scenario()))
    assert planner._goal_field is None


def test_gating_occluded_goal_builds_field():
    # circle dead-center: every corner chord to the engage point is blocked
    planner = astar.KinodynamicAstar(prep.prepare_scenario(circle_scenario()))
    assert planner._goal_field is not None


def test_heuristic_is_max_of_euclid_and_field():
    planner = astar.KinodynamicAstar(prep.prepare_scenario(circle_scenario()))
    gs = planner.goal_state
    p = (150000.0, 250000.0)          # in the circle's shadow
    st = astar.State(p, 0.0)
    h = planner.heuristic(st, gs)
    assert h >= math.dist(p, gs.waypoint) - 1e-6
    assert h >= planner._goal_field.query(p) - 1e-6


def test_field_failure_falls_back_to_euclid(monkeypatch):
    class _Boom:
        def __init__(self, pre):
            raise RuntimeError("boom")
    monkeypatch.setattr(astar, 'GoalDistanceField', _Boom)
    pre = prep.prepare_scenario(circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    assert planner._goal_field is None
    res = astar.plan_trajectory(prep.prepare_scenario(circle_scenario()), verbose=False)
    assert res['success']
```

- [ ] **Step 2: Run to verify they fail**

Run: `python -m pytest -q tests/heuristic_field_test.py -k "gating or max_of or falls_back"`
Expected: FAIL with `AttributeError: ... no attribute '_goal_field'` (and no `GoalDistanceField` in `astar`).

- [ ] **Step 3: Implement the hook**

In `core/kinodynamic_astar.py`:

(a) Add to the imports block (after `import core.arc_geometry as ag`):

```python
from core.heuristic_field import GoalDistanceField
```

(b) In `KinodynamicAstar.__init__`, AFTER the start-corner seeding loop and
BEFORE the pre-computed-constants block, add:

```python
        # Admissible goal-distance field (heuristic tightening). Built only
        # when EVERY surviving corner's straight chord to the goal is
        # blocked — open scenarios keep zero overhead, and any build failure
        # degrades to the plain Euclid heuristic (the field must never be
        # able to fail a plan).
        self._goal_field = None
        if self.start_corners and all(
                not self._check_collision(c.waypoint, self.goal_state.waypoint)
                for c in self.start_corners):
            try:
                self._goal_field = GoalDistanceField(preprocessed_scenario)
            except Exception:
                self._goal_field = None
```

(c) Replace the body of `heuristic()` (keep the docstring, extend it):

```python
    def heuristic(self, state, goal_state):
        """Admissible lower-bound heuristic: straight-line distance to the
        goal waypoint, tightened by the goal-distance field (max of two
        lower bounds is a lower bound) when one was built."""
        dx = goal_state.waypoint[0] - state.waypoint[0]
        dy = goal_state.waypoint[1] - state.waypoint[1]
        h = math.sqrt(dx * dx + dy * dy)
        if self._goal_field is not None:
            hf = self._goal_field.query(state.waypoint)
            if hf > h:
                return hf
        return h
```

- [ ] **Step 4: Run the whole field test file**

Run: `python -m pytest -q tests/heuristic_field_test.py`
Expected: all pass.

- [ ] **Step 5: Quick no-new-failures sweep of the planner tests**

Run: `python -m pytest -q tests/kinodynamic_arc_hop_test.py tests/strategy_b_valve_test.py`
Expected: same failures as the recorded baseline for `kinodynamic_arc_hop_test` (reason-plumbing reds), `strategy_b_valve_test` all green.

- [ ] **Step 6: Commit**

```bash
git add core/kinodynamic_astar.py tests/heuristic_field_test.py
git commit -m "feat(core): hook goal-distance field into A* heuristic (gated, fallback-safe)"
```

---

### Task 4: Oracle witness + speed guard

**Files:**
- Test: `tests/heuristic_field_test.py` (append)

**Interfaces:**
- Consumes: `plan_trajectory`, `KinodynamicAstar`, `State` from `core.kinodynamic_astar`; `generate_random_scenario` from `batch_random_test`.
- Produces: locked regression baselines (the 11-seed table) and a relative speed guarantee on a synthetic wall map.

- [ ] **Step 1: Write the failing/locking tests**

Append to `tests/heuristic_field_test.py`:

```python
from batch_random_test import generate_random_scenario

# Mission lengths (km) with the pure-Euclid heuristic, recorded 2026-07-18
# (post valve/fan fixes). The field may only match or shorten, +5 km slack
# for tie-break noise (same threshold as the 1000-seed A/B protocol).
BASELINE_KM = {4: 446.9, 79: 503.1, 92: 521.5, 123: 447.2, 155: 477.8,
               187: 438.8, 242: 480.3, 272: 457.8, 496: 477.2, 612: 442.6,
               964: 481.2}


@pytest.fixture
def no_time_budget(monkeypatch):
    monkeypatch.setattr(config, 'TIME_BUDGET_S', None)


def _mission_km(pre, res):
    pts = [pre['start_pos']] + [p for p, _h in res['path']] + [pre['goal_pos']]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:])) / 1000.0


@pytest.mark.parametrize('seed', sorted(BASELINE_KM))
def test_oracle_witness_and_no_regression(seed, no_time_budget):
    pre = prep.prepare_scenario(generate_random_scenario(seed=seed))
    res = astar.plan_trajectory(pre, verbose=False)
    assert res['success']
    assert _mission_km(pre, res) <= BASELINE_KM[seed] + 5.0
    # admissibility witness: h at the path's first state must not exceed the
    # cost actually flown from there (body polyline + snap to goal waypoint)
    probe = astar.KinodynamicAstar(pre)          # fresh: builds field per gating
    first = astar.State(res['path'][0][0], res['path'][0][1])
    body = [p for p, _h in res['path']]
    flown = sum(math.dist(a, b) for a, b in zip(body, body[1:]))
    flown += math.dist(body[-1], probe.goal_state.waypoint)
    assert probe.heuristic(first, probe.goal_state) <= flown + 1.0


def wall_scenario():
    """Tall wall between start and goal: true detour ~+30%, so the field
    must prune the Euclid-optimistic basin behind the wall."""
    wall = [(240000.0, 80000.0), (260000.0, 80000.0),
            (260000.0, 420000.0), (240000.0, 420000.0)]
    return {
        'start': (50000.0, 250000.0), 'start_heading': 0.0,
        'goal': (450000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [wall], 'dynamic_obstacles': [],
        'obstacles': [{'type': 'polygon', 'polygon': wall}],
        'map_bounds': (500000.0, 500000.0),
    }


def test_field_cuts_expansions_on_wall_map(no_time_budget, monkeypatch):
    res_field = astar.plan_trajectory(prep.prepare_scenario(wall_scenario()),
                                      verbose=False)

    class _Boom:
        def __init__(self, pre):
            raise RuntimeError("disabled")
    monkeypatch.setattr(astar, 'GoalDistanceField', _Boom)
    res_euclid = astar.plan_trajectory(prep.prepare_scenario(wall_scenario()),
                                       verbose=False)

    assert res_field['success'] and res_euclid['success']
    it_f = res_field['stats']['iterations']
    it_e = res_euclid['stats']['iterations']
    assert it_f < 0.7 * it_e, f"field {it_f} vs euclid {it_e}: expected >30% cut"
    # equal-quality guard on the same map
    pre = prep.prepare_scenario(wall_scenario())
    assert abs(_mission_km(pre, res_field) - _mission_km(pre, res_euclid)) <= 5.0
```

- [ ] **Step 2: Run the new tests**

Run: `python -m pytest -q tests/heuristic_field_test.py -k "oracle or wall"`
Expected: all pass (~30–60 s: 11 unbudgeted plans + 2 wall plans). If
`test_field_cuts_expansions_on_wall_map` fails on the 0.7 ratio, do NOT
weaken the assertion blindly: print both iteration counts, check the field
was actually built (`_goal_field is not None` — the gate requires ALL
corner chords blocked), and check the wall's inflated polygon really blocks
the corner chords. A legitimately smaller-but-real cut (e.g. 0.8) may be
accepted ONLY with the observed numbers documented in the test comment.

- [ ] **Step 3: Commit**

```bash
git add tests/heuristic_field_test.py
git commit -m "test(core): oracle admissibility witness + wall-map speed guard"
```

---

### Task 5: Full suite + 1000-seed A/B acceptance

**Files:**
- Create: `<scratchpad>/hfield_sweep.py` (throwaway harness — scratchpad, NOT committed)
- No production changes unless the A/B uncovers a defect.

**Interfaces:**
- Consumes: everything above.
- Produces: the acceptance evidence demanded by the spec (success counts, distance distribution, h-witness, wall-time), reported to the user.

- [ ] **Step 1: Full pytest**

Run: `python -m pytest -q 2>&1 | tail -3`
Expected: `18 failed` (the pre-existing reds, byte-identical list) and all
new tests passing. Verify with:
`python -m pytest -q 2>&1 | grep FAILED | sort` — compare against the
recorded pre-existing-failure list; ANY new name = stop and fix before
proceeding.

- [ ] **Step 2: Write the sweep harness (scratchpad)**

`<scratchpad>/hfield_sweep.py` — same worker pattern as the earlier
`ab_sweep.py`, plus the admissibility witness:

```python
"""1000-seed sweep with admissibility witness. Usage:
   python hfield_sweep.py <out.json> <tree_path>"""
import json, math, sys, time
from multiprocessing import Pool

sys.path.insert(0, sys.argv[2])


def one(seed):
    import config
    config.TIME_BUDGET_S = 10
    import core.preprocessing as prep
    import core.kinodynamic_astar as astar
    from batch_random_test import generate_random_scenario
    try:
        pre = prep.prepare_scenario(generate_random_scenario(seed=seed))
        t0 = time.perf_counter()
        res = astar.plan_trajectory(pre, verbose=False)
        dt = time.perf_counter() - t0
        row = {'seed': seed, 'success': bool(res.get('success')),
               'time_s': round(dt, 3)}
        if res.get('stats'):
            row['iters'] = res['stats']['iterations']
        if res.get('success'):
            pts = [pre['start_pos']] + [p for p, h in res['path']] + [pre['goal_pos']]
            row['dist_km'] = round(sum(math.dist(a, b)
                                       for a, b in zip(pts, pts[1:])) / 1000, 2)
            probe = astar.KinodynamicAstar(pre)
            first = astar.State(res['path'][0][0], res['path'][0][1])
            body = [p for p, _h in res['path']]
            flown = sum(math.dist(a, b) for a, b in zip(body, body[1:]))
            flown += math.dist(body[-1], probe.goal_state.waypoint)
            hval = probe.heuristic(first, probe.goal_state)
            row['h_violation'] = bool(hval > flown + 1.0)
            row['field_built'] = probe._goal_field is not None
        return row
    except Exception as e:
        return {'seed': seed, 'success': False,
                'error': f'{type(e).__name__}: {e}', 'time_s': 0}


def main():
    t0 = time.time()
    with Pool(6) as pool:
        rows = list(pool.imap_unordered(one, range(1000), chunksize=10))
    rows.sort(key=lambda r: r['seed'])
    with open(sys.argv[1], 'w') as f:
        json.dump(rows, f)
    ok = sum(r['success'] for r in rows)
    viol = [r['seed'] for r in rows if r.get('h_violation')]
    built = sum(1 for r in rows if r.get('field_built'))
    print(f"DONE success={ok}/1000 h_violations={viol} fields_built={built} "
          f"wall={time.time() - t0:.0f}s")


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Baseline sweep (field disabled)**

Snapshot the tree and disable the field in the SNAPSHOT ONLY, so the
baseline is "same code, Euclid h":

```bash
SP=<scratchpad>
mkdir -p $SP/euclid_tree && cp config.py logger_config.py performance_eval.py \
    batch_random_test.py $SP/euclid_tree/ && cp -r core render $SP/euclid_tree/
python - <<'EOF'
import re, pathlib
p = pathlib.Path("<scratchpad>/euclid_tree/core/kinodynamic_astar.py")
src = p.read_text()
src = src.replace("self._goal_field = GoalDistanceField(preprocessed_scenario)",
                  "raise RuntimeError('field disabled for baseline')")
p.write_text(src)
EOF
cd $SP && python hfield_sweep.py euclid_1000.json $SP/euclid_tree
```

Expected: `DONE success=<S>/1000 h_violations=[] fields_built=0 ...`
(h can't violate when it IS the flown-cost lower bound Euclid).

- [ ] **Step 4: Field sweep (working tree)**

```bash
cd <scratchpad> && python hfield_sweep.py field_1000.json /mnt/d/Workspace/VTX/Path_Planning
```

Expected: `h_violations=[]` — **any non-empty list is an admissibility bug:
STOP, reproduce that seed, fix (larger slack / finer grid per spec) before
looking at anything else.**

- [ ] **Step 5: Compare and report**

```python
# <scratchpad>/hfield_compare.py
import json
A = {r['seed']: r for r in json.load(open('<scratchpad>/euclid_1000.json'))}
B = {r['seed']: r for r in json.load(open('<scratchpad>/field_1000.json'))}
okA = sum(A[s]['success'] for s in A); okB = sum(B[s]['success'] for s in B)
print(f"success: euclid {okA}  field {okB}")
print("flips e-ok->f-fail:", [s for s in A if A[s]['success'] and not B[s]['success']])
print("flips e-fail->f-ok:", [s for s in A if not A[s]['success'] and B[s]['success']])
both = [s for s in A if A[s]['success'] and B[s]['success']]
diffs = sorted(((B[s]['dist_km'] - A[s]['dist_km'], s) for s in both))
print("longer>0.5km:", [(s, round(d, 1)) for d, s in diffs if d > 0.5])
print("shorter<-0.5km:", len([1 for d, _ in diffs if d < -0.5]))
tA = sum(A[s]['time_s'] for s in A); tB = sum(B[s]['time_s'] for s in B)
iA = sum(A[s].get('iters', 0) for s in A); iB = sum(B[s].get('iters', 0) for s in B)
print(f"total time {tA:.0f}s -> {tB:.0f}s   total iters {iA} -> {iB}")
```

Acceptance gates (from the spec):
- success(field) ≥ success(euclid); zero euclid-OK → field-FAIL flips
  (a flip = the reordered search timed out: investigate that seed);
- `h_violations == []`;
- every seed with dist_km > baseline + 5 km gets the seed-92-style
  drill-down (raw route + probe) before acceptance;
- report total/hard-seed wall-time and iteration deltas to the user.

- [ ] **Step 6: Final report + (on user request) commit of any A/B-driven fixes**

Summarise: success table, distance distribution, h-witness result,
wall-time before/after, list of investigated seeds. Do not merge/push;
leave the branch state for the user.

## Self-review notes

- Spec coverage: module + extent modes (T1/T2), blocking/solve/query (T1),
  hook + gating + fallback (T3), all five spec test groups (T1 analytic
  circle, T2 corridor+permissive, T3 gating/fallback, T4 oracle witness +
  speed guard, T5 suite + A/B protocol with witness). Rollback needs no
  task (revert of T1/T3 commits).
- The wall-map speed guard uses a relative ratio (field vs disabled-field on
  the same code) instead of a brittle absolute iteration count.
- `GoalDistanceField` is imported at module level in `kinodynamic_astar`
  precisely so tests and the baseline snapshot can monkeypatch/disable it.
