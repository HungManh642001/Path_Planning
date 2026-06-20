# Sub-Second Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cut planner runtime to **under 1 second per query** (suboptimal paths acceptable) by removing the redundant turn-arc collision check, generating successors via on-the-fly tangent geometry, pruning collision tests with a spatial index, and capping search with a wall-clock budget — at the new operating point **α_max = 90°**.

**Architecture:** Three measured hotspots are eliminated. (1) `_arc_clear` (12 collision sub-checks per successor) is removed entirely — obstacle inflation `R(1/cos(α_max/2)−1)+SAFE_MARGIN` already guarantees the radius-R turn arc clears, *given every turn ≤ α_max* (which the search enforces). (2) The O(N²) static visibility-graph build is removed — successors are computed **dynamically per expanded position** as the tangent points to each circle, the convex-hull vertices of each polygon, and the goal; the 11-direction radial fan stays as a fallback when no graph successor is valid. (3) The remaining line-of-sight / collision test keeps the **exact** Shapely predicate but prunes obstacles with a `shapely.STRtree` (bounding-box prefilter → exact `intersects` on candidates only — no accuracy change). A wall-clock budget in the search loop guarantees the <1s bound even when a scenario can't be solved.

**Tech Stack:** Python 3.11, pytest 7.4, numpy, shapely 2.1 (`STRtree`, `STRtree.query(line, predicate='intersects')`).

## Global Constraints

- Operating point: **α_max = 90°** (`config.ALPHA_MAX = 90.0`). Set in Task 5.
- `SAFE_MARGIN` is a free configuration parameter; keep its current value **10000.0** for this plan (do not tune it here).
- **Accuracy must not change** for collision/LOS: use `STRtree` + the exact Shapely `intersects` predicate. Do **NOT** replace the geometric predicate with hand-rolled numpy (rejected by the user for boundary-case robustness risk). The circle test stays the existing exact point-to-segment distance.
- Suboptimal paths are acceptable; the success criterion is **runtime, not optimality**.
- Turn-arc collision is guaranteed by inflation, not by a runtime check (valid only because every turn ≤ α_max is enforced by `validate_kinodynamics` / radial construction).
- Test files MUST be named `*_test.py` (the repo `.gitignore` excludes `test_*.py`).
- Re-locking a characterization baseline = update `EXPECTED` in `characterization_test.py` to the ACTUAL printed values; never edit the algorithm to match old numbers. A `valid=True → valid=False` flip on a previously-solving scenario is a regression to REPORT, not silently lock.

---

## File Structure

**Modified:**
- `kinodynamic_astar.py` — `KinodynamicAstar.__init__` (build STRtree + precompute polygon hull vertices), `_check_collision` (STRtree prune), `get_next_states` (dynamic tangent successors + radial fallback, no arc), `search` (wall-clock budget), `smooth_path` (drop arc check), `plan_trajectory` (stop building the tangent graph); remove `_arc_clear`.
- `spatial_utils.py` — add `circle_tangent_points(point, center, radius)`.
- `config.py` — `ALPHA_MAX = 90.0`; add `TIME_BUDGET_S`.
- `characterization_test.py` — re-lock `EXPECTED` after each behavior-changing task.
- `kinodynamic_astar_test.py` — remove `test_arc_clear_detects_obstacle_in_turn`; add tests for STRtree-collision equivalence, dynamic tangent successors, time budget.
- `spatial_utils_test.py` (new) — tests for `circle_tangent_points`.

**Untouched but noted:** `graph_builder.py` stays in the repo (its own tests keep passing) but is **no longer called by the live path** after Task 4. `path_validation.py` is the independent validity oracle used by tests.

---

### Task 1: Spatial-index the polygon collision test (exact, behavior-preserving)

**Files:**
- Modify: `kinodynamic_astar.py` (`KinodynamicAstar.__init__`, `_check_collision`)
- Test: `kinodynamic_astar_test.py`

**Interfaces:**
- Consumes: `preprocessed_scenario['polygon_obstacles']` (list of coord lists), `['circle_obstacles']` (list of `(center,(x,y), radius)`).
- Produces: `planner._poly_tree` (a `shapely.STRtree` or `None`); `_check_collision(p1, p2) -> bool` (unchanged semantics: `True` = clear).

- [ ] **Step 1: Write the failing test** (STRtree result must equal brute-force for many random segments)

```python
# append to kinodynamic_astar_test.py
import random as _random
from shapely.geometry import Polygon as _Poly, LineString as _Line


def _brute_force_clear(p1, p2, circles, polys):
    import spatial_utils as su
    for c, r in circles:
        if su.point_to_line_distance(c, p1, p2) < r - 1e-6:
            return False
    line = _Line([p1, p2])
    return not any(line.intersects(_Poly(poly)) for poly in polys)


def test_check_collision_matches_bruteforce_with_spatial_index():
    polys = [[(10000, 10000), (30000, 10000), (30000, 30000), (10000, 30000)],
             [(60000, 5000), (90000, 5000), (75000, 40000)]]
    circles = [((50000.0, 50000.0), 8000.0)]
    pre = _simple_pre(circles=[((50000.0, 50000.0), 8000.0)],
                      polys=polys)
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    rng = _random.Random(1234)
    for _ in range(400):
        p1 = (rng.uniform(0, 100000), rng.uniform(0, 100000))
        p2 = (rng.uniform(0, 100000), rng.uniform(0, 100000))
        got = planner._check_collision(p1, p2)
        want = _brute_force_clear(p1, p2, pre['circle_obstacles'], pre['polygon_obstacles'])
        assert got == want, f"mismatch on {p1}->{p2}: got {got} want {want}"
    assert hasattr(planner, '_poly_tree')
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest kinodynamic_astar_test.py::test_check_collision_matches_bruteforce_with_spatial_index -v`
Expected: FAIL — `assert hasattr(planner, '_poly_tree')` (attribute does not exist yet). (The equivalence loop would pass against the current linear scan, but the attribute assertion fails.)

- [ ] **Step 3: Write minimal implementation**

In `kinodynamic_astar.py`, at the top with the other imports, add:
```python
from shapely import STRtree
```
In `KinodynamicAstar.__init__`, where `self._polygons` is built, add the tree right after it:
```python
        self._polygons = [Polygon(coords) for coords in preprocessed_scenario['polygon_obstacles']]
        self._poly_tree = STRtree(self._polygons) if self._polygons else None
```
Replace the polygon loop in `_check_collision` with an STRtree query using the EXACT `intersects` predicate (keep the circle loop exactly as is):
```python
        # Check against polygon obstacles via spatial index (exact predicate).
        if self._poly_tree is not None:
            line = LineString([p1, p2])
            if len(self._poly_tree.query(line, predicate='intersects')) > 0:
                return False
        return True
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest kinodynamic_astar_test.py::test_check_collision_matches_bruteforce_with_spatial_index -v`
Expected: PASS (result identical to brute force; `_poly_tree` present).

- [ ] **Step 5: Verify no behavior change on the baseline**

Run: `python -m pytest kinodynamic_astar_test.py characterization_test.py -v`
Expected: all PASS, characterization values UNCHANGED (exact pruning). If any characterization value changes, STOP — the index is not exact; report BLOCKED.

- [ ] **Step 6: Commit**

```bash
git add kinodynamic_astar.py kinodynamic_astar_test.py
git commit -m "perf: spatial-index polygon collision test (STRtree, exact)"
```

---

### Task 2: Remove the turn-arc collision check (`_arc_clear`)

**Files:**
- Modify: `kinodynamic_astar.py` (`get_next_states` Strategy 2 + goal-directed; `smooth_path`; delete `_arc_clear`)
- Test: `kinodynamic_astar_test.py`

**Interfaces:**
- Consumes: nothing new.
- Produces: `get_next_states` and `smooth_path` no longer call `_arc_clear`; the method `_arc_clear` no longer exists. Output paths remain valid because obstacle inflation covers the worst-case arc bulge for turns ≤ α_max.

- [ ] **Step 1: Update tests first**

In `kinodynamic_astar_test.py`: DELETE the test `test_arc_clear_detects_obstacle_in_turn` (it tests a method we are removing). KEEP `test_produced_path_is_fully_valid_around_circle` — it asserts `arcs_clear` on the produced path and now PROVES that inflation alone keeps arcs clear without the runtime check. Add an explicit regression test that the method is gone:
```python
def test_arc_clear_method_removed():
    import inspect
    assert not hasattr(astar.KinodynamicAstar, '_arc_clear'), \
        "arc clearance is guaranteed by inflation; the runtime check must be removed"
    src = inspect.getsource(astar.KinodynamicAstar.get_next_states)
    assert '_arc_clear' not in src
    src2 = inspect.getsource(astar.KinodynamicAstar.smooth_path)
    assert '_arc_clear' not in src2
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest kinodynamic_astar_test.py::test_arc_clear_method_removed -v`
Expected: FAIL — `_arc_clear` still present.

- [ ] **Step 3: Remove the arc check**

In `kinodynamic_astar.py`:
1. Delete the entire `_arc_clear` method.
2. In `get_next_states` Strategy 2 (radial fan), remove the `if not self._arc_clear(...): continue` line so a radial successor is accepted on `_check_collision` + `validate_kinodynamics` alone.
3. In the goal-directed successor block, remove the `and self._arc_clear(...)` conjunct from its accept condition.
4. In `smooth_path`, remove the arc-clearance conjuncts: the shortcut accept condition becomes `if is_valid and self._check_collision(prev_wp, next_wp):` (drop both the departure `_arc_clear` and the `landing_arc_ok` block). Keep the `validate_kinodynamics` / `is_valid` check and the geometric inbound-heading computation for `validate_kinodynamics`.

- [ ] **Step 4: Run to verify it passes and paths stay valid**

Run: `python -m pytest kinodynamic_astar_test.py -v`
Expected: PASS. Critically, `test_produced_path_is_fully_valid_around_circle` still passes (its `arcs_clear` assertion holds via inflation) and `test_finds_valid_path_around_single_circle` still solves.

- [ ] **Step 5: Re-lock characterization**

Run: `python -m pytest characterization_test.py -v -s` (use a 600000 ms timeout / background).
Removing the arc gate may let a previously-rejected successor through, changing some valid=True paths (and is much faster). Update `EXPECTED` for any scenario whose printed metrics changed; confirm every `valid=yes` scenario still prints `collision_free=True`. A `valid=True→False` flip is a regression → report DONE_WITH_CONCERNS. Record old→new.

- [ ] **Step 6: Commit**

```bash
git add kinodynamic_astar.py kinodynamic_astar_test.py characterization_test.py
git commit -m "perf: drop runtime arc-clearance check (guaranteed by inflation); re-lock baseline"
```

---

### Task 3: `circle_tangent_points` geometry helper

**Files:**
- Modify: `spatial_utils.py`
- Test: `spatial_utils_test.py` (new)

**Interfaces:**
- Produces: `spatial_utils.circle_tangent_points(point, center, radius) -> list[(x,y)]` — the two tangent points on the circle from an external `point`; `[]` if `point` is inside or on the circle.

- [ ] **Step 1: Write the failing test**

```python
# spatial_utils_test.py
import math
import spatial_utils as su


def test_circle_tangent_points_external():
    # point on +x axis, unit circle at origin: tangent points are symmetric about x-axis,
    # each at distance == radius from center, and the tangent line is perpendicular to the radius.
    pts = su.circle_tangent_points((10.0, 0.0), (0.0, 0.0), 6.0)
    assert len(pts) == 2
    for (tx, ty) in pts:
        assert abs(math.hypot(tx, ty) - 6.0) < 1e-9          # on the circle
        # radius vector (t-center) perpendicular to tangent direction (point - t)
        rx, ry = tx, ty
        dx, dy = 10.0 - tx, 0.0 - ty
        assert abs(rx * dx + ry * dy) < 1e-6                  # perpendicular
    # the two points are mirror images across the x-axis
    ys = sorted(p[1] for p in pts)
    assert abs(ys[0] + ys[1]) < 1e-9


def test_circle_tangent_points_inside_returns_empty():
    assert su.circle_tangent_points((1.0, 0.0), (0.0, 0.0), 6.0) == []
    assert su.circle_tangent_points((6.0, 0.0), (0.0, 0.0), 6.0) == []  # on the boundary
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest spatial_utils_test.py -v`
Expected: FAIL — `AttributeError: module 'spatial_utils' has no attribute 'circle_tangent_points'`.

- [ ] **Step 3: Implement**

Append to `spatial_utils.py`:
```python
def circle_tangent_points(point, center, radius):
    """Tangent points on a circle from an external point.

    Returns the two points where lines from `point` touch the circle, or []
    if `point` is inside or on the circle (no real tangent).
    """
    px, py = point
    cx, cy = center
    dx, dy = px - cx, py - cy
    d2 = dx * dx + dy * dy
    if d2 <= radius * radius + 1e-9:
        return []
    d = math.sqrt(d2)
    theta = math.atan2(dy, dx)          # center -> point direction
    alpha = math.acos(radius / d)       # half-angle of the tangent cone
    return [
        (cx + radius * math.cos(theta + alpha), cy + radius * math.sin(theta + alpha)),
        (cx + radius * math.cos(theta - alpha), cy + radius * math.sin(theta - alpha)),
    ]
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest spatial_utils_test.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add spatial_utils.py spatial_utils_test.py
git commit -m "feat: add circle_tangent_points geometry helper"
```

---

### Task 4: Dynamic tangent successors + retire the static graph

**Files:**
- Modify: `kinodynamic_astar.py` (`__init__`, `get_next_states`, `plan_trajectory`)
- Test: `kinodynamic_astar_test.py`

**Interfaces:**
- Consumes: `spatial_utils.circle_tangent_points` (Task 3); `_angle_diff` (module-level, exists); `prep.validate_kinodynamics`; `self._check_collision` (Task 1).
- Produces: `get_next_states(state)` returns successors generated from dynamic tangent geometry (circle tangent points + polygon hull vertices + goal), with the radial fan as a fallback; it no longer reads `self.tangent_graph`. `plan_trajectory` builds the planner with `tangent_graph=None` (no graph construction). New attribute `planner._poly_vertices` (list of `(x,y)` convex-hull vertices of all polygon obstacles, precomputed once).

- [ ] **Step 1a: Retire tests obsoleted by the rewrite**

The rewrite drops `tangent_graph` use and makes the radial fan a fallback. Two tests from the earlier plan become invalid and MUST be updated in this same task:
- DELETE `test_strategy1_uses_graph_adjacency` — it monkeypatches `tg.get_neighbors` and asserts it is called; `get_next_states` no longer reads the tangent graph.
- REPLACE the body of `test_turn_penalty_makes_turning_cost_more_than_straight` so it exercises the radial fallback (on an empty map the new Strategy A returns only the directly-reachable goal candidate, so the old test's "many radial successors" premise is gone). New body:
```python
def test_turn_penalty_makes_turning_cost_more_than_straight():
    # Goal directly behind the start heading (turn 180deg > alpha_max) => no graph
    # candidate is valid => radial fallback fires, giving the 11-way fan to compare.
    pre = _simple_pre(goal=(-50000.0, 0.0))
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    succ = planner.get_next_states(planner.start_state)
    assert len(succ) >= 2
    straight = min(succ, key=lambda s: abs(astar._angle_diff(s[0].heading, planner.start_state.heading)))
    turned = max(succ, key=lambda s: abs(astar._angle_diff(s[0].heading, planner.start_state.heading)))
    assert turned[1] > straight[1], "a sharper turn must cost more (turn penalty)"
```

- [ ] **Step 1: Write the failing tests**

```python
# append to kinodynamic_astar_test.py
import spatial_utils as su2  # alias to avoid clashing if su already imported


def test_dynamic_successors_include_circle_tangents():
    # Circle nearly straight ahead and small, so BOTH tangent directions are within
    # alpha_max of the start heading (this task runs at the default alpha_max=30 deg).
    # From start, successors must include the circle's tangent points (computed against
    # the INFLATED radius, which is what the planner stores) — no tangent_graph used.
    circle_raw = ((60000.0, 6000.0), 8000.0)
    pre = _simple_pre(circles=[circle_raw], goal=(120000.0, 0.0))
    inflated_center, inflated_radius = pre['circle_obstacles'][0]
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    succ = planner.get_next_states(planner.start_state)
    tang = su2.circle_tangent_points(planner.start_state.waypoint, inflated_center, inflated_radius)
    succ_wps = [s[0].waypoint for s in succ]
    assert tang, "expected two tangent points from start to the inflated circle"
    assert any(min(math.hypot(w[0]-t[0], w[1]-t[1]) for w in succ_wps) < 1.0 for t in tang), \
        "dynamic successors must include circle tangent points"


def test_radial_fallback_when_no_graph_candidate():
    # empty map: no obstacles, so tangent set is just the goal. If the goal is directly
    # reachable it is a successor; force the not-directly-reachable case by a goal behind
    # the start heading so the direct jump turn > alpha_max, exercising the radial fallback.
    pre = _simple_pre(goal=(-50000.0, 0.0))   # goal behind start (start_heading=0)
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    succ = planner.get_next_states(planner.start_state)
    assert len(succ) > 0, "radial fallback must produce successors when no graph candidate is valid"
```

- [ ] **Step 2: Run to verify they fail**

Run: `python -m pytest kinodynamic_astar_test.py -k "dynamic_successors or radial_fallback" -v`
Expected: FAIL — current `get_next_states` uses `self.tangent_graph` (None here) so Strategy 1 yields nothing structured around tangents; `test_dynamic_successors_include_circle_tangents` fails.

- [ ] **Step 3: Implement dynamic successors**

In `kinodynamic_astar.py` `__init__`, precompute polygon hull vertices once (after `self._polygons` is built):
```python
        self._poly_vertices = []
        for poly in self._polygons:
            self._poly_vertices.extend(poly.convex_hull.exterior.coords[:-1])
```
Replace the body of `get_next_states` with the dynamic-tangent generator + radial fallback (read the current method first for the exact radial-fan code to reuse):
```python
    def get_next_states(self, current_state):
        """Dynamic successors: tangent points to circles + polygon hull vertices +
        the goal; radial fan as a fallback when no graph candidate is valid."""
        successors = []
        P = current_state.waypoint
        h = current_state.heading

        # --- Strategy A: dynamic tangent / vertex / goal candidates ---
        candidates = []
        for center, radius in self.scenario['circle_obstacles']:
            candidates.extend(su.circle_tangent_points(P, center, radius))
        candidates.extend(self._poly_vertices)
        candidates.append(self.goal_state.waypoint)

        for node in candidates:
            dx = node[0] - P[0]
            dy = node[1] - P[1]
            if dx * dx + dy * dy < 10000:        # skip ~within 100 m
                continue
            heading_to_node = su.angle_to_heading(P, node)
            turn = abs(_angle_diff(heading_to_node, h))
            if turn > self.alpha_max_rad:
                continue
            is_valid, _ = prep.validate_kinodynamics(
                P, h, node, heading_to_node, R=self.R, alpha_max=self.alpha_max_rad)
            if not is_valid:
                continue
            if not self._check_collision(P, node):
                continue
            cost = math.hypot(dx, dy) + config.TURN_PENALTY_WEIGHT * turn
            successors.append((State(node, heading_to_node), cost))

        if successors:
            return successors

        # --- Strategy B: radial fan fallback (no graph candidate was valid) ---
        num_directions = 11
        distance = 2 * self.R * math.tan(self.alpha_max_rad / 2)
        for i in range(num_directions):
            heading_offset = -self.alpha_max_rad + 2 * self.alpha_max_rad * i / (num_directions - 1)
            next_heading = h + heading_offset
            nx = P[0] + distance * math.cos(next_heading)
            ny = P[1] + distance * math.sin(next_heading)
            next_waypoint = (nx, ny)
            if not self._in_bounds(next_waypoint):
                continue
            if not self._check_collision(P, next_waypoint):
                continue
            is_valid, _ = prep.validate_kinodynamics(
                P, h, next_waypoint, next_heading, R=self.R, alpha_max=self.alpha_max_rad)
            if not is_valid:
                continue
            turn = abs(_angle_diff(next_heading, h))
            cost = distance + config.TURN_PENALTY_WEIGHT * turn
            successors.append((State(next_waypoint, next_heading), cost))

        return successors
```
In `plan_trajectory`, stop building the tangent graph. Read the function; remove the `generate_bitangents(...)` and `extend_tangent_graph_with_start_goal(...)` calls (and the `gb` import use if now unused — leave the import line, harmless), and construct the planner with no graph:
```python
    planner = KinodynamicAstar(preprocessed_scenario, tangent_graph=None)
```
Keep the rest of `plan_trajectory` (search, smoothing, return dict). The returned dict's `'tangent_graph'` key — set it to `None`:
```python
        'tangent_graph': None,
```

- [ ] **Step 4: Run to verify the new tests pass and completeness holds**

Run: `python -m pytest kinodynamic_astar_test.py -v`
Expected: PASS, including `test_finds_valid_path_around_single_circle` (now solved via dynamic tangents), `test_produced_path_is_fully_valid_around_circle`, and the two new tests. If single-circle no longer solves, do NOT hack — report DONE_WITH_CONCERNS with the diagnosis (likely the goal/circle geometry; the radial fallback should still find it).

- [ ] **Step 5: Re-lock characterization**

Run: `python -m pytest characterization_test.py -v -s` (600000 ms / background). The successor model changed → valid=True paths likely change (often shorter / fewer waypoints) and runtimes drop sharply. Update `EXPECTED` to ACTUAL printed values; confirm `collision_free=True` on every `valid=yes`. A `valid=True→False` flip is a regression → report DONE_WITH_CONCERNS. Record old→new.

- [ ] **Step 6: Commit**

```bash
git add kinodynamic_astar.py kinodynamic_astar_test.py characterization_test.py
git commit -m "perf+feat: dynamic tangent successors, retire static graph build; re-lock baseline"
```

---

### Task 5: α_max = 90°, wall-clock budget, and the <1s benchmark

**Files:**
- Modify: `config.py` (`ALPHA_MAX = 90.0`, add `TIME_BUDGET_S`), `kinodynamic_astar.py` (`search` budget)
- Test: `kinodynamic_astar_test.py`, `characterization_test.py` (re-lock at 90°)
- Test: `benchmark_test.py` (new)

**Interfaces:**
- Consumes: `config.TIME_BUDGET_S` (float seconds or `None`).
- Produces: `search()` returns (best path or `None`) within roughly `TIME_BUDGET_S` wall-clock regardless of `MAX_ITERATIONS`.

- [ ] **Step 1: Set the operating point and budget in config**

In `config.py`: change `ALPHA_MAX = 30.0` to `ALPHA_MAX = 90.0` (the `ALPHA_MAX_RAD = deg_to_rad(ALPHA_MAX)` line below recomputes it). Add near the A* SEARCH section:
```python
# Wall-clock budget for a single search (seconds). None = no time limit.
TIME_BUDGET_S = 0.9
```

- [ ] **Step 2: Write the failing test** (budget actually bounds the loop)

```python
# append to kinodynamic_astar_test.py
import time as _time


def test_search_respects_time_budget(monkeypatch):
    # Force a tiny budget and a scenario that would otherwise run to the iteration cap;
    # search must return within a small multiple of the budget.
    monkeypatch.setattr(config, 'TIME_BUDGET_S', 0.05)
    # dense-ish: several polygons straddling the route so the graph can't trivially solve it
    polys = [[(40000+i*1000, 0), (60000+i*1000, 0), (50000+i*1000, 40000)] for i in range(6)]
    pre = _simple_pre(polys=polys, goal=(120000.0, 0.0))
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    t0 = _time.perf_counter()
    planner.search()
    dt = _time.perf_counter() - t0
    assert dt < 0.5, f"search ignored the 0.05s budget (took {dt:.3f}s)"
```

- [ ] **Step 3: Run to verify it fails**

Run: `python -m pytest kinodynamic_astar_test.py::test_search_respects_time_budget -v`
Expected: FAIL — no budget check; the search runs to the iteration cap, exceeding 0.5s.

- [ ] **Step 4: Implement the budget**

In `kinodynamic_astar.py` `search`, at the top of the method add a start time, and check it at the top of the while loop:
```python
        import time
        _start = time.perf_counter()
        _budget = config.TIME_BUDGET_S
```
Then inside `while self.open_set and self.iteration_count < self.max_iterations:`, as the first statement:
```python
            if _budget is not None and (time.perf_counter() - _start) > _budget:
                break
```
(The existing fall-through after the loop already sets `self.search_failed = True; return None` when no path was found — leave it.)

- [ ] **Step 5: Run to verify it passes**

Run: `python -m pytest kinodynamic_astar_test.py::test_search_respects_time_budget -v`
Expected: PASS (returns well under 0.5s).

- [ ] **Step 6: Write the benchmark test** (the <1s acceptance gate)

```python
# benchmark_test.py
import time
import math
import map_generator as mg
import preprocessing as prep
import kinodynamic_astar as astar

# Representative spread across difficulty (uses the predefined scenarios directly,
# independent of map_generator.get_all_scenarios which the user may have trimmed).
_SCENARIOS = [
    ('open', mg.scenario1_open_ocean),
    ('sparse', mg.scenario5_sparse_islands),
    ('archipelago', mg.scenario9_island_archipelago),
    ('dense_islands', mg.scenario13_dense_island_field),
    ('extreme', mg.scenario16_extreme_complexity),
]


def test_planning_runtime_under_one_second():
    slow = []
    for name, fn in _SCENARIOS:
        pre = prep.prepare_scenario(fn())
        t0 = time.perf_counter()
        res = astar.plan_trajectory(pre, verbose=False)
        dt = time.perf_counter() - t0
        print(f"[bench] {name:14} success={res['success']!s:5} time={dt*1000:7.1f} ms")
        slow.append((name, dt))
    worst = max(slow, key=lambda x: x[1])
    # Hard guard: the wall-clock budget (0.9s) + setup must keep every query under ~1.3s.
    assert worst[1] < 1.3, f"slowest scenario {worst[0]} took {worst[1]:.3f}s (budget regression)"
```

- [ ] **Step 7: Run the benchmark**

Run: `python -m pytest benchmark_test.py -v -s`
Expected: PASS — every scenario prints a per-query time, and the slowest is under 1.3s (the 0.9s budget plus preprocessing/smoothing overhead). Paste the printed times. If a scenario exceeds 1.3s, the budget check in Step 4 is not firing or preprocessing itself is the cost — investigate before proceeding (do NOT loosen the assert blindly).

- [ ] **Step 8: Re-lock characterization at α_max = 90°**

Run: `python -m pytest characterization_test.py -v -s` (600000 ms / background). α_max=90 changes turn limits and the radial step (16 km), so most valid=True scenarios change and some previously-no-path scenarios may now solve. Update every changed `EXPECTED` to ACTUAL printed values; confirm `collision_free=True` on every `valid=yes`. Record old→new. (No `valid=True→False` flip should occur from *loosening* α_max; if one does, report DONE_WITH_CONCERNS.)

- [ ] **Step 9: Full suite + commit**

Run: `python -m pytest -q` → all green.
```bash
git add config.py kinodynamic_astar.py kinodynamic_astar_test.py characterization_test.py benchmark_test.py
git commit -m "perf: alpha_max=90, wall-clock search budget, sub-second benchmark; re-lock baseline"
```

---

## Final verification

- [ ] `python -m pytest -q` → all pass.
- [ ] `python -m pytest benchmark_test.py -v -s` → every scenario under 1.3s; paste times. The target is **<1s** per query; 1.3s is the flaky-guard ceiling (budget 0.9s + overhead).
- [ ] Spot-check a solved scenario with `path_validation.path_is_valid(...)` to confirm produced paths remain collision-free and within α_max despite the removed runtime arc check.

## Notes on residual risk

- **Arc safety rests entirely on inflation now.** It is valid only while every turn ≤ α_max (enforced by `validate_kinodynamics` on graph successors and by construction on radial successors). If a future change emits a successor with a larger turn, arcs are no longer guaranteed — keep that invariant.
- **Single large circle straddling the route** still cannot be circumnavigated by the graph (no hugging edges, by design) and relies on the radial fallback; with α_max=90 and the time budget it either solves quickly or fails fast — both under 1s.
- **`SAFE_MARGIN` is unfixed.** All runtimes here use 10000 m. If it is later reduced, obstacles shrink, more tangents become valid, and runtimes only improve — re-run the benchmark to confirm.
- **`graph_builder.py` is now dead in the live path** but retained with its tests. A later cleanup task could delete it once nothing imports it.
- **`plan_trajectory` now returns `'tangent_graph': None`.** Tests don't use this key, but `visualizer.py` (run via `main.py`/GUI, not in the test suite) may plot bitangents from it — if so it needs a `None` guard. Out of scope for this plan; flag it if `main.py` is run.
