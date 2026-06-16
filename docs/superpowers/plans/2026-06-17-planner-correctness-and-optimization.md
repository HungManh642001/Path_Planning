# Kinodynamic A* Planner — Correctness & Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the missile path planner *find* kinodynamically-valid, obstacle-free trajectories (not just on empty fields), do so within seconds, and produce short paths with few turns — while every change is locked behind a regression test.

**Architecture:** Work bottom-up in 6 phases. Phase 0 builds reusable validators + extends the characterization baseline. Phase 1 restores completeness (a working navigation graph). Phase 2 makes the search tractable (state dedup, adjacency, polygon caching). Phase 3 enforces the kinodynamic constraints from the spec PDF. Phase 4 optimizes path quality. Phase 5 is cleanup. Each phase ends with the full test suite green.

**Tech Stack:** Python 3.11, pytest 7.4, numpy, scipy, shapely 2.x. No new dependencies.

---

## Conventions (read once before starting)

- **Test files MUST be named `*_test.py`** (e.g. `graph_builder_test.py`). The repo `.gitignore` excludes `test_*.py`, so `test_*.py` files will NOT be committed. `characterization_test.py` already follows this rule.
- Units are **meters & radians**. Angles stored in `config` as degrees are pre-converted (`config.ALPHA_MAX_RAD`).
- Run a single test: `python -m pytest <file>::<test> -v`. Run all: `python -m pytest -q`.
- The slow obstacle scenarios can take 20–130 s **today**. After Phase 2 they must be < 10 s. Phase-0/1 tests on small custom maps are designed to run fast (small `MAX_ITERATIONS` override where noted).
- A planner **path** is a list of `(waypoint, heading)` tuples, `waypoint=(x,y)`.
- Commit after every green step. Never commit red.
- **Do not** delete `characterization_test.py`'s locked values; when a behavior intentionally changes, update its `EXPECTED` table in the same commit and say so in the message.

---

## File Structure

**New files:**
- `path_validation.py` — pure functions to validate a produced path against the spec: segment clearance, turn-arc clearance, turn angles, straight-segment (đoản trình) lengths. Used by every later test. One responsibility: "is this path geometrically & kinodynamically valid?"
- `path_validation_test.py` — tests for the validators (the validators themselves are TDD'd).
- `graph_builder_test.py` — tests for the navigation graph.
- `kinodynamic_astar_test.py` — tests for search behavior (completeness, dedup, termination).
- `preprocessing_test.py` — tests for constraint formulas.

**Modified files:**
- `graph_builder.py` — correct tangents (Phase 1), visibility nodes (Phase 1), LOS factor (Phase 5).
- `kinodynamic_astar.py` — polygon cache (Phase 2), adjacency successors (Phase 2), state lattice (Phase 2), dead-code removal (Phase 2), validation re-enable + arc check (Phase 3), turn penalty + smoothing + heuristic (Phase 4).
- `spatial_utils.py` — state lattice quantization (Phase 2).
- `preprocessing.py` — constraint model (Phase 3), endpoint angles (Phase 3).
- `config.py` — new tunables (`STATE_POS_QUANTUM`, `TURN_PENALTY_WEIGHT`, `OBSTACLE_RING_SAMPLES`, `GOAL_THRESHOLD`).
- `characterization_test.py` — extend with multi-obstacle scenarios (Phase 0); update locked values when behavior changes (Phases 1, 3, 4).

---

# PHASE 0 — Safety net & validators

**Outcome:** Reusable, tested validators and an extended characterization baseline that locks today's (failing) behavior on multi-obstacle maps, so later phases prove progress.

### Task 0.1: Segment clearance validator

**Files:**
- Create: `path_validation.py`
- Test: `path_validation_test.py`

- [ ] **Step 1: Write the failing test**

```python
# path_validation_test.py
import math
import path_validation as pv


def test_segment_clear_returns_true_when_no_obstacle():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0)]
    assert pv.segments_clear(path, circle_obstacles=[], polygon_obstacles=[]) is True


def test_segment_blocked_by_circle_on_the_line():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0)]
    # circle centred on the segment, radius 10 -> blocks
    assert pv.segments_clear(path, circle_obstacles=[((50.0, 0.0), 10.0)],
                             polygon_obstacles=[]) is False


def test_segment_clear_when_circle_far_from_line():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0)]
    assert pv.segments_clear(path, circle_obstacles=[((50.0, 1000.0), 10.0)],
                             polygon_obstacles=[]) is True


def test_segment_blocked_by_polygon():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0)]
    poly = [(40.0, -10.0), (60.0, -10.0), (60.0, 10.0), (40.0, 10.0)]
    assert pv.segments_clear(path, circle_obstacles=[], polygon_obstacles=[poly]) is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest path_validation_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'path_validation'`.

- [ ] **Step 3: Write minimal implementation**

```python
# path_validation.py
"""Validators for produced planner paths (spec: Điều kiện ràng buộc đường bay).

A path is a list of (waypoint, heading) tuples with waypoint = (x, y).
These functions are deliberately independent of the planner internals so
tests can assert validity without trusting the code under review.
"""
import math
from shapely.geometry import Polygon, LineString


def _point_to_segment_distance(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def _segment_clear(a, b, circle_obstacles, polygon_obstacles, tol=1e-6):
    for center, radius in circle_obstacles:
        if _point_to_segment_distance(center, a, b) < radius - tol:
            return False
    line = LineString([a, b])
    for coords in polygon_obstacles:
        if line.intersects(Polygon(coords)):
            return False
    return True


def segments_clear(path, circle_obstacles, polygon_obstacles):
    """True iff every straight segment between consecutive waypoints is clear."""
    for i in range(len(path) - 1):
        a = path[i][0]
        b = path[i + 1][0]
        if not _segment_clear(a, b, circle_obstacles, polygon_obstacles):
            return False
    return True
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest path_validation_test.py -v`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit**

```bash
git add path_validation.py path_validation_test.py
git commit -m "test: add segment-clearance path validator"
```

---

### Task 0.2: Turn-angle and straight-segment (đoản trình) validators

**Files:**
- Modify: `path_validation.py`
- Test: `path_validation_test.py`

Spec formulas: turn αᵢ at each interior waypoint = angle between incoming and outgoing segment directions; αᵢ ≤ α_max. Straight portion of a middle segment `l_{i+1} = d_{i+1} − R·(tan(αᵢ/2) + tan(α_{i+1}/2))` must be > 0. First segment: `l₁ = d₁ − R·tan(α₁/2) ≥ L₀`. Last segment: `lₙ = dₙ − d_ss − R·tan(α_{n-1}/2) ≥ 0`.

- [ ] **Step 1: Write the failing test**

```python
# append to path_validation_test.py

def test_turn_angles_straight_line_is_zero():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0), ((200.0, 0.0), 0.0)]
    angles = pv.turn_angles(path)
    assert len(angles) == 1
    assert abs(angles[0]) < 1e-9


def test_turn_angles_right_angle():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0), ((100.0, 100.0), 0.0)]
    angles = pv.turn_angles(path)
    assert abs(angles[0] - math.pi / 2) < 1e-9


def test_turn_angle_limit_ok():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0), ((200.0, 50.0), 0.0)]
    # ~26.57 deg turn, under 30 deg
    assert pv.turn_angles_ok(path, alpha_max_rad=math.radians(30.0)) is True


def test_turn_angle_limit_violated():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0), ((150.0, 100.0), 0.0)]
    # ~63 deg turn, over 30 deg
    assert pv.turn_angles_ok(path, alpha_max_rad=math.radians(30.0)) is False


def test_straight_segment_lengths_positive_for_long_legs():
    # three 100 km legs, gentle turns -> middle straight portion must stay > 0
    path = [((0.0, 0.0), 0.0), ((100000.0, 0.0), 0.0),
            ((200000.0, 10000.0), 0.0), ((300000.0, 20000.0), 0.0)]
    ok, detail = pv.straight_segments_ok(path, R=8000.0, L0=4000.0, dss=23000.0)
    assert ok is True, detail
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest path_validation_test.py -k "turn or straight" -v`
Expected: FAIL — `AttributeError: module 'path_validation' has no attribute 'turn_angles'`.

- [ ] **Step 3: Write minimal implementation**

```python
# append to path_validation.py

def _seg_heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _norm(delta):
    return math.atan2(math.sin(delta), math.cos(delta))


def turn_angles(path):
    """Turn angle (rad, magnitude) at each interior waypoint, from segment geometry."""
    angles = []
    for i in range(1, len(path) - 1):
        h_in = _seg_heading(path[i - 1][0], path[i][0])
        h_out = _seg_heading(path[i][0], path[i + 1][0])
        angles.append(abs(_norm(h_out - h_in)))
    return angles


def turn_angles_ok(path, alpha_max_rad):
    return all(a <= alpha_max_rad + 1e-9 for a in turn_angles(path))


def _seg_len(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def straight_segments_ok(path, R, L0, dss):
    """Check đoản trình straight-portion constraints from the spec.

    Returns (ok, detail). alpha at interior waypoints comes from turn_angles();
    endpoints have no turn before/after them (alpha = 0 at O and at T).
    """
    n_seg = len(path) - 1
    if n_seg < 1:
        return True, "trivial"
    alphas = [0.0] + turn_angles(path) + [0.0]  # alpha at each waypoint index
    for i in range(n_seg):
        d = _seg_len(path[i][0], path[i + 1][0])
        a_i = alphas[i]
        a_next = alphas[i + 1]
        l = d - R * (math.tan(a_i / 2) + math.tan(a_next / 2))
        if i == 0:                       # first đoản trình: l1 >= L0
            if l < L0 - 1.0:
                return False, f"first segment l={l:.1f} < L0={L0}"
        elif i == n_seg - 1:             # last đoản trình: ln = l - dss >= 0
            if l - dss < -1.0:
                return False, f"last segment usable l={l - dss:.1f} < 0"
        else:                            # middle: l > 0
            if l < 1.0:
                return False, f"middle segment {i} l={l:.1f} <= 0"
    return True, "ok"
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest path_validation_test.py -v`
Expected: PASS (all).

- [ ] **Step 5: Commit**

```bash
git add path_validation.py path_validation_test.py
git commit -m "test: add turn-angle and doan-trinh validators per spec"
```

---

### Task 0.3: Turn-arc clearance validator

**Files:**
- Modify: `path_validation.py`
- Test: `path_validation_test.py`

The flown path replaces each corner with an arc of radius R tangent to both segments; the arc apex sits `R·(1/cos(α/2) − 1)` off the corner on the inside of the turn. We sample the arc and check clearance.

- [ ] **Step 1: Write the failing test**

```python
# append to path_validation_test.py

def test_arc_clear_when_no_obstacle():
    path = [((0.0, 0.0), 0.0), ((100000.0, 0.0), 0.0), ((200000.0, 50000.0), 0.0)]
    assert pv.arcs_clear(path, R=8000.0, circle_obstacles=[], polygon_obstacles=[]) is True


def test_arc_blocked_by_obstacle_on_inside_of_turn():
    # Corner at (100000,0) turning left; inside of the turn is +y.
    path = [((0.0, 0.0), 0.0), ((100000.0, 0.0), 0.0), ((100000.0, 100000.0), 0.0)]
    # An obstacle hugging the inside corner that the straight segments miss
    # but the radius-8000 arc cuts through.
    blocking = ((100000.0 - 3000.0, 3000.0), 1500.0)
    assert pv.arcs_clear(path, R=8000.0, circle_obstacles=[blocking],
                         polygon_obstacles=[]) is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest path_validation_test.py -k arc -v`
Expected: FAIL — `AttributeError: ... 'arcs_clear'`.

- [ ] **Step 3: Write minimal implementation**

```python
# append to path_validation.py

def _unit(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    return (dx / d, dy / d) if d > 0 else (0.0, 0.0)


def _arc_points(w_prev, w, w_next, R, n=24):
    """Sample the radius-R turn arc that replaces corner w."""
    u = _unit(w_prev, w)      # incoming direction
    v = _unit(w, w_next)      # outgoing direction
    alpha = abs(_norm(math.atan2(v[1], v[0]) - math.atan2(u[1], u[0])))
    if alpha < 1e-9:
        return []
    t = R * math.tan(alpha / 2)              # tangent length along each leg
    A = (w[0] - u[0] * t, w[1] - u[1] * t)   # tangent point on incoming leg
    s = 1.0 if (u[0] * v[1] - u[1] * v[0]) > 0 else -1.0   # left(+)/right(-) turn
    n_in = (-u[1] * s, u[0] * s)             # inward normal of incoming leg
    C = (A[0] + R * n_in[0], A[1] + R * n_in[1])   # arc centre
    start = math.atan2(A[1] - C[1], A[0] - C[0])
    pts = []
    for k in range(n + 1):
        ang = start + s * alpha * (k / n)
        pts.append((C[0] + R * math.cos(ang), C[1] + R * math.sin(ang)))
    return pts


def arcs_clear(path, R, circle_obstacles, polygon_obstacles):
    """True iff every turn arc clears all obstacles."""
    for i in range(1, len(path) - 1):
        pts = _arc_points(path[i - 1][0], path[i][0], path[i + 1][0], R)
        for j in range(len(pts) - 1):
            if not _segment_clear(pts[j], pts[j + 1], circle_obstacles, polygon_obstacles):
                return False
    return True


def path_is_valid(path, circle_obstacles, polygon_obstacles, R, alpha_max_rad, L0, dss):
    """One-call full validity gate used by later phases."""
    if not path or len(path) < 2:
        return False
    if not segments_clear(path, circle_obstacles, polygon_obstacles):
        return False
    if not arcs_clear(path, R, circle_obstacles, polygon_obstacles):
        return False
    if not turn_angles_ok(path, alpha_max_rad):
        return False
    ok, _ = straight_segments_ok(path, R, L0, dss)
    return ok
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest path_validation_test.py -v`
Expected: PASS (all). If `test_arc_blocked...` does not fail-as-designed, adjust the blocking circle in the test until red-then-green confirms the arc sampler detects an inside-corner hit (the geometry, not the threshold, is what we verify).

- [ ] **Step 5: Commit**

```bash
git add path_validation.py path_validation_test.py
git commit -m "test: add turn-arc clearance validator and full validity gate"
```

---

### Task 0.4: Extend the characterization baseline to multi-obstacle maps

**Files:**
- Modify: `characterization_test.py`

Lock today's behavior on two new deterministic maps so Phase 1 can prove the unlock. Today both are expected to FAIL (no path).

- [ ] **Step 1: Write the failing test (sentinel)**

Add to `characterization_test.py` two builders and entries, mirroring the existing style:

```python
# in characterization_test.py, near the other scenario builders
def scenario_two_circles_gap():
    """Two circles flanking the diagonal with a navigable gap between them."""
    return _build_scenario(_START, _H, _GOAL, _H, islands=[],
                           circles=[((180000, 240000), 35000),
                                    ((280000, 200000), 35000)])


def scenario_circle_and_island():
    """A circle then an island, both off-centre so a route exists around them."""
    return _build_scenario(_START, _H, _GOAL, _H,
                           islands=[_FAR_ISLAND],
                           circles=[((150000, 180000), 30000)])
```

```python
# extend the SCENARIOS dict
SCENARIOS = {
    'empty': scenario_empty,
    'one_circle': scenario_one_circle,
    'one_island': scenario_one_island,
    'mixed': scenario_mixed,
    'two_circles_gap': scenario_two_circles_gap,
    'circle_and_island': scenario_circle_and_island,
}
```

```python
# extend EXPECTED with sentinels to force a capture
EXPECTED['two_circles_gap'] = {'valid': None, 'waypoints': None, 'num_turns': None, 'total_length_m': None}
EXPECTED['circle_and_island'] = {'valid': None, 'waypoints': None, 'num_turns': None, 'total_length_m': None}
```

```python
# add the two test functions
def test_two_circles_gap():
    _check('two_circles_gap')


def test_circle_and_island():
    _check('circle_and_island')
```

- [ ] **Step 2: Run to capture actual behavior**

Run: `python -m pytest characterization_test.py -k "two_circles_gap or circle_and_island" -v -s`
Expected: FAIL — the printed `[name] valid=... waypoints=...` lines reveal the real values (today: `valid=no`, `waypoints=0`, `num_turns=0`, `length=0.0`). Note the runtimes.

- [ ] **Step 3: Lock the captured values**

Replace the two sentinel entries in `EXPECTED` with the actual captured values, e.g.:

```python
EXPECTED['two_circles_gap'] = {'valid': False, 'waypoints': 0, 'num_turns': 0, 'total_length_m': 0.0}
EXPECTED['circle_and_island'] = {'valid': False, 'waypoints': 0, 'num_turns': 0, 'total_length_m': 0.0}
```

(If a capture differs, lock what was actually printed.)

- [ ] **Step 4: Run to verify green**

Run: `python -m pytest characterization_test.py -k "two_circles_gap or circle_and_island" -v`
Expected: PASS (2 passed).

- [ ] **Step 5: Commit**

```bash
git add characterization_test.py
git commit -m "test: lock baseline on two new multi-obstacle scenarios (currently no-path)"
```

---

# PHASE 1 — Restore completeness (working navigation graph)

**Outcome:** The planner finds a *valid* path around a single obstacle and through multi-obstacle maps. Highest-leverage, lowest-risk first.

### Task 1.1: Cache Shapely polygons in collision checks (I3 — perf, enables fast Phase-1 tests)

**Files:**
- Modify: `kinodynamic_astar.py` (the `KinodynamicAstar.__init__` and `_check_collision`)
- Test: `kinodynamic_astar_test.py`

- [ ] **Step 1: Write the failing test**

```python
# kinodynamic_astar_test.py
import math
import preprocessing as prep
import kinodynamic_astar as astar


def _simple_pre(circles=(), polys=()):
    scenario = {
        'start': (2000, 2000), 'start_heading': 0.0,
        'goal': (100000, 0), 'goal_heading': 0.0,
        'obstacles': [{'type': 'circle', 'center': c, 'radius': r} for c, r in circles]
                     + [{'type': 'polygon', 'polygon': p} for p in polys],
        'islands': [], 'sam_sites': [],
    }
    return prep.prepare_scenario(scenario)


def test_polygons_are_prebuilt_shapely_objects():
    pre = _simple_pre(polys=[[(0, 0), (10, 0), (10, 10), (0, 10)]])
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    # New attribute caches one Shapely Polygon per obstacle, built once.
    from shapely.geometry import Polygon
    assert hasattr(planner, '_polygons')
    assert all(isinstance(p, Polygon) for p in planner._polygons)
    assert len(planner._polygons) == 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest kinodynamic_astar_test.py::test_polygons_are_prebuilt_shapely_objects -v`
Expected: FAIL — `assert hasattr(planner, '_polygons')` is False.

- [ ] **Step 3: Write minimal implementation**

In `kinodynamic_astar.py` `__init__`, after `self.scenario = preprocessed_scenario`, add:

```python
        from shapely.geometry import Polygon as _Poly
        self._polygons = [_Poly(coords) for coords in preprocessed_scenario['polygon_obstacles']]
```

Change `_check_collision` polygon loop from rebuilding to using the cache:

```python
        # Check against polygon obstacles (prebuilt)
        line = LineString([p1, p2])
        for polygon in self._polygons:
            if line.intersects(polygon):
                return False
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest kinodynamic_astar_test.py -v`
Expected: PASS. Then confirm no behavior change: `python -m pytest characterization_test.py -k "empty or one_circle" -v` → still PASS, and `one_circle` runtime printed (`-s`) should drop noticeably.

- [ ] **Step 5: Commit**

```bash
git add kinodynamic_astar.py kinodynamic_astar_test.py
git commit -m "perf: cache shapely polygons once per planner (I3)"
```

---

### Task 1.2: Use the correct circle-circle tangent routine (C1b)

**Files:**
- Modify: `graph_builder.py` (`generate_bitangents` line ~213)
- Test: `graph_builder_test.py`

The live `su.compute_tangent_lines` places tangent points 90° wrong (lines pass through circle centres). The repo already has a correct `_circle_circle_tangents` in `graph_builder.py`.

- [ ] **Step 1: Write the failing test**

```python
# graph_builder_test.py
import math
import graph_builder as gb
import spatial_utils as su


def _dist_point_to_line(p, a, b):
    return su.point_to_line_distance(p, a, b)


def test_generated_tangents_do_not_cross_circle_centres():
    # two equal circles; every generated edge must keep >= r clearance from
    # both centres (a true tangent touches at exactly r).
    circles = [((0.0, 0.0), 1000.0), ((10000.0, 0.0), 1000.0)]
    g = gb.generate_bitangents(circles, [], filter_los=True)
    assert len(g.edges) > 0
    for p1, p2 in g.edges:
        for center, radius in circles:
            assert _dist_point_to_line(center, p1, p2) >= radius - 1.0, \
                f"edge {p1}->{p2} cuts circle at {center}"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest graph_builder_test.py::test_generated_tangents_do_not_cross_circle_centres -v`
Expected: FAIL — edges run through the centres (distance ≈ 0).

- [ ] **Step 3: Write minimal implementation**

In `graph_builder.py` `generate_bitangents`, replace:

```python
            tangents = su.compute_tangent_lines(circle1, circle2)
```

with:

```python
            tangents = _circle_circle_tangents(circle1, circle2)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest graph_builder_test.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add graph_builder.py graph_builder_test.py
git commit -m "fix: use geometrically-correct circle tangents in graph (C1b)"
```

---

### Task 1.3: Visibility nodes so single & multi obstacle routes exist (C1a)

**Files:**
- Modify: `graph_builder.py` (`generate_bitangents`), `config.py`
- Test: `graph_builder_test.py`, `kinodynamic_astar_test.py`, `characterization_test.py`

A pairwise-tangent graph yields **zero** nodes for one obstacle. Add a ring of boundary "support" nodes per circle and the convex-hull vertices per polygon, then keep only nodes/edges with clear line-of-sight. Start/goal connect to these in `extend_tangent_graph_with_start_goal` (already iterates `graph.nodes`, so once nodes exist it works).

- [ ] **Step 1: Add the config knob**

In `config.py` add near the TANGENT GRAPH section:

```python
# Number of boundary support nodes sampled around each circular obstacle
OBSTACLE_RING_SAMPLES = 16
```

- [ ] **Step 2: Write the failing test**

```python
# append to graph_builder_test.py
import config


def test_single_circle_produces_navigation_nodes():
    circles = [((50000.0, 0.0), 20000.0)]
    g = gb.generate_bitangents(circles, [], filter_los=True)
    # ring of boundary nodes around the one obstacle
    assert len(g.nodes) >= config.OBSTACLE_RING_SAMPLES - 2


def test_single_polygon_produces_hull_nodes():
    poly = [(40000.0, -10000.0), (60000.0, -10000.0),
            (60000.0, 10000.0), (40000.0, 10000.0)]
    g = gb.generate_bitangents([], [poly], filter_los=True)
    assert len(g.nodes) >= 4
```

- [ ] **Step 3: Run to verify it fails**

Run: `python -m pytest graph_builder_test.py -k "produces" -v`
Expected: FAIL — `len(g.nodes) == 0` today for a single obstacle.

- [ ] **Step 4: Write minimal implementation**

In `graph_builder.py`, add a helper and call it at the end of `generate_bitangents` (just before `return graph`):

```python
def _ring_nodes(center, radius, n):
    pts = []
    for k in range(n):
        ang = 2.0 * math.pi * k / n
        pts.append((center[0] + radius * math.cos(ang),
                    center[1] + radius * math.sin(ang)))
    return pts


def _add_visibility_nodes(graph, circle_obstacles, polygon_obstacles):
    """Add boundary support nodes per obstacle and connect mutually-visible
    pairs whose connecting segment clears all obstacles."""
    nodes = []
    for center, radius in circle_obstacles:
        nodes.extend(_ring_nodes(center, radius, config.OBSTACLE_RING_SAMPLES))
    for poly in polygon_obstacles:
        nodes.extend(Polygon(poly).convex_hull.exterior.coords[:-1])

    def clear(a, b):
        for center, radius in circle_obstacles:
            if su.point_to_line_distance(center, a, b) < radius - 1.0:
                return False
        line = LineString([a, b])
        for poly in polygon_obstacles:
            if line.crosses(Polygon(poly)) or line.within(Polygon(poly)):
                return False
        return True

    for n in nodes:
        graph.add_node(n)
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if clear(nodes[i], nodes[j]):
                graph.add_edge(nodes[i], nodes[j])
```

At the end of `generate_bitangents`, before `return graph`:

```python
    _add_visibility_nodes(graph, circle_obstacles, polygon_obstacles)
    return graph
```

(Note: nodes sit exactly on the inflated boundary; `clear()` uses `< radius - 1.0` so a node's own circle does not reject its edges. `crosses`/`within` for polygons allow boundary-touching.)

- [ ] **Step 5: Run to verify graph tests pass**

Run: `python -m pytest graph_builder_test.py -v`
Expected: PASS.

- [ ] **Step 6: Write the completeness test (the real unlock)**

```python
# append to kinodynamic_astar_test.py
import path_validation as pv
import config


def test_finds_valid_path_around_single_circle():
    pre = _simple_pre(circles=[((50000.0, 0.0), 20000.0)])
    import graph_builder as gb
    tg = gb.generate_bitangents(pre['circle_obstacles'], pre['polygon_obstacles'])
    tg = gb.extend_tangent_graph_with_start_goal(
        tg, pre['start_state']['waypoint'], pre['start_state']['heading'],
        pre['goal_state']['waypoint'], pre['goal_state']['heading'],
        pre['circle_obstacles'], pre['polygon_obstacles'])
    planner = astar.KinodynamicAstar(pre, tg)
    path = planner.search()
    assert path is not None, "planner must find a route around one circle"
    assert pv.segments_clear(path, pre['circle_obstacles'], pre['polygon_obstacles'])
```

- [ ] **Step 7: Run to verify completeness**

Run: `python -m pytest kinodynamic_astar_test.py::test_finds_valid_path_around_single_circle -v`
Expected: PASS. If still None, the issue is search reachability (addressed in Phase 2); in that case temporarily raise nothing — instead verify the graph now has a start→…→goal chain via `tg.get_neighbors`. The expected outcome after this task is a found path on this *small* map (100 km) within the iteration cap.

- [ ] **Step 8: Re-capture the characterization baseline (behavior intentionally changed)**

Run: `python -m pytest characterization_test.py -v -s`
Several scenarios now find paths. Update each changed entry in `EXPECTED` to the newly printed `valid/waypoints/num_turns/total_length_m`. Keep `empty` unchanged. Re-run until green.

- [ ] **Step 9: Commit**

```bash
git add graph_builder.py config.py graph_builder_test.py kinodynamic_astar_test.py characterization_test.py
git commit -m "feat: visibility nodes enable routing around single/multi obstacles (C1a); re-lock baseline"
```

---

# PHASE 2 — Tractable search

**Outcome:** Searches terminate in seconds, not on the 50k cap. No path-validity regressions.

### Task 2.1: Quantize the state lattice so A* dedup works (C5)

**Files:**
- Modify: `spatial_utils.py` (`state_to_tuple`), `config.py`
- Test: `kinodynamic_astar_test.py`

Today positions round to 0.1 m and headings to 1e-4 rad — far finer than the ~4.3 km step, so no states ever merge. Quantize to a coarse lattice.

- [ ] **Step 1: Add config knobs**

In `config.py`:

```python
# State-lattice quantisation for A* de-duplication
STATE_POS_QUANTUM = 1000.0          # meters
STATE_HEADING_QUANTUM_DEG = 3.0     # degrees
```

- [ ] **Step 2: Write the failing test**

```python
# append to kinodynamic_astar_test.py
import spatial_utils as su
import math as _m


def test_state_tuple_buckets_nearby_states_together():
    a = su.state_to_tuple((123456.0, 7000.0), 0.10)
    b = su.state_to_tuple((123456.0 + 200.0, 7000.0 + 200.0), 0.10 + _m.radians(1.0))
    assert a == b, "states within one lattice cell must hash equal"


def test_state_tuple_distinguishes_far_states():
    a = su.state_to_tuple((0.0, 0.0), 0.0)
    b = su.state_to_tuple((5000.0, 0.0), 0.0)
    assert a != b
```

- [ ] **Step 3: Run to verify it fails**

Run: `python -m pytest kinodynamic_astar_test.py -k state_tuple -v`
Expected: FAIL — current fine rounding keeps `a != b` in the first test.

- [ ] **Step 4: Write minimal implementation**

Replace `state_to_tuple` in `spatial_utils.py`:

```python
def state_to_tuple(waypoint, heading):
    """Quantise (waypoint, heading) onto the search lattice for hashing/dedup."""
    q = config.STATE_POS_QUANTUM
    hq = math.radians(config.STATE_HEADING_QUANTUM_DEG)
    hx = round(waypoint[0] / q)
    hy = round(waypoint[1] / q)
    hh = round(math.atan2(math.sin(heading), math.cos(heading)) / hq)
    return (hx, hy, hh)
```

- [ ] **Step 5: Run to verify it passes**

Run: `python -m pytest kinodynamic_astar_test.py -k state_tuple -v`
Expected: PASS.

- [ ] **Step 6: Guard against over-coarse lattice merging goal**

Run the completeness test and characterization again:
`python -m pytest kinodynamic_astar_test.py::test_finds_valid_path_around_single_circle characterization_test.py -v -s`
Expected: PASS, with obstacle scenario runtimes now much lower (closed/open sets far smaller). If any scenario regresses to no-path, the lattice is too coarse relative to `GOAL_THRESHOLD` — reduce `STATE_POS_QUANTUM` (e.g. 500) and re-run. Re-lock any intentionally changed characterization values.

- [ ] **Step 7: Commit**

```bash
git add spatial_utils.py config.py kinodynamic_astar_test.py characterization_test.py
git commit -m "perf: quantise state lattice so A* de-dup actually prunes (C5)"
```

---

### Task 2.2: Expand only graph-adjacent nodes in Strategy 1 (I2)

**Files:**
- Modify: `kinodynamic_astar.py` (`get_next_states`)
- Test: `kinodynamic_astar_test.py`

Today Strategy 1 scans **every** graph node on every expansion (O(N·M)). Use the precomputed adjacency from the node the state sits on.

- [ ] **Step 1: Write the failing (characterization-of-cost) test**

```python
# append to kinodynamic_astar_test.py
def test_strategy1_uses_graph_adjacency(monkeypatch):
    import graph_builder as gb
    pre = _simple_pre(circles=[((50000.0, 0.0), 20000.0)])
    tg = gb.generate_bitangents(pre['circle_obstacles'], pre['polygon_obstacles'])
    tg = gb.extend_tangent_graph_with_start_goal(
        tg, pre['start_state']['waypoint'], pre['start_state']['heading'],
        pre['goal_state']['waypoint'], pre['goal_state']['heading'],
        pre['circle_obstacles'], pre['polygon_obstacles'])
    planner = astar.KinodynamicAstar(pre, tg)

    calls = {'n': 0}
    orig = tg.get_neighbors
    def counting(pos):
        calls['n'] += 1
        return orig(pos)
    monkeypatch.setattr(tg, 'get_neighbors', counting)

    planner.get_next_states(planner.start_state)
    assert calls['n'] >= 1, "Strategy 1 must consult graph adjacency, not scan all nodes"
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest kinodynamic_astar_test.py::test_strategy1_uses_graph_adjacency -v`
Expected: FAIL — `get_neighbors` is never called today (the code loops `self.tangent_graph.nodes`).

- [ ] **Step 3: Write minimal implementation**

In `kinodynamic_astar.py` `get_next_states`, replace the Strategy 1 block (`for node in self.tangent_graph.nodes:` …) with adjacency-based expansion plus a direct goal attempt:

```python
        if self.tangent_graph is not None:
            neighbors = self.tangent_graph.get_neighbors(current_state.waypoint)
            # Always also consider the goal node directly.
            goal_wp = self.goal_state.waypoint
            candidates = [wp for wp, _cost in neighbors] + [goal_wp]
            for node in candidates:
                dx = node[0] - current_state.waypoint[0]
                dy = node[1] - current_state.waypoint[1]
                if dx * dx + dy * dy < 10000:
                    continue
                heading_to_node = su.angle_to_heading(current_state.waypoint, node)
                is_valid, _ = prep.validate_kinodynamics(
                    current_state.waypoint, current_state.heading,
                    node, heading_to_node,
                    R=self.R, alpha_max=self.alpha_max_rad)
                if is_valid and self._check_collision(current_state.waypoint, node):
                    successors.append((State(node, heading_to_node), math.sqrt(dx * dx + dy * dy)))
```

(The start/goal nodes are wired into the graph by `extend_tangent_graph_with_start_goal`, so `get_neighbors(start)` returns the visible support nodes.)

- [ ] **Step 4: Run to verify it passes and completeness holds**

Run: `python -m pytest kinodynamic_astar_test.py characterization_test.py -v -s`
Expected: PASS. Completeness preserved; obstacle runtimes drop further.

- [ ] **Step 5: Commit**

```bash
git add kinodynamic_astar.py kinodynamic_astar_test.py
git commit -m "perf: Strategy-1 expands graph-adjacent nodes instead of scanning all (I2)"
```

---

### Task 2.3: Remove the dead early-exit guard and defaultdict bloat (C4, M4)

**Files:**
- Modify: `kinodynamic_astar.py` (`search`, `__init__`)
- Test: `kinodynamic_astar_test.py`

The `iterations_without_expansion` guard resets every iteration, so `> 10` never fires (dead code). Remove it. Use `.get` for `g_scores` reads so missing keys don't allocate.

- [ ] **Step 1: Write the failing test**

```python
# append to kinodynamic_astar_test.py
import inspect


def test_no_dead_stuck_counter():
    src = inspect.getsource(astar.KinodynamicAstar.search)
    assert 'iterations_without_expansion' not in src, \
        "dead early-exit counter must be removed (C4)"
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest kinodynamic_astar_test.py::test_no_dead_stuck_counter -v`
Expected: FAIL — the variable is still present.

- [ ] **Step 3: Write minimal implementation**

In `kinodynamic_astar.py` `search`, delete the three `iterations_without_expansion` lines (init, reset, increment) and the `if iterations_without_expansion > 10: break` block. The loop now ends only on success, empty open set, or `MAX_ITERATIONS`.

For M4, change the read in the relaxation step from `self.g_scores[next_state]` to a non-allocating read:

```python
                tentative_g = self.g_scores[current] + transition_cost
                if tentative_g < self.g_scores.get(next_state, float('inf')):
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest kinodynamic_astar_test.py characterization_test.py -v`
Expected: PASS. (Behavior identical; just removes dead code and reduces allocations.)

- [ ] **Step 5: Commit**

```bash
git add kinodynamic_astar.py kinodynamic_astar_test.py
git commit -m "perf: remove dead stuck-counter and avoid defaultdict bloat (C4, M4)"
```

---

# PHASE 3 — Enforce kinodynamic constraints (higher risk; do after search is stable)

**Outcome:** Produced paths provably satisfy the spec's turn-angle, đoản trình, and arc-clearance constraints. **I4 must precede C2.**

### Task 3.1: Faithful đoản trình constraint over a waypoint triple (I4)

**Files:**
- Modify: `preprocessing.py` (`validate_kinodynamics`)
- Test: `preprocessing_test.py`

Today `validate_kinodynamics` substitutes `alpha_next = alpha_max`. Make it use the *actual* next turn when the following waypoint is supplied; fall back to the conservative `alpha_max` only when it is unknown.

- [ ] **Step 1: Write the failing test**

```python
# preprocessing_test.py
import math
import preprocessing as prep


def test_validate_uses_actual_next_turn_when_triple_given():
    # Straight through three colinear points: next turn = 0, so required
    # straight length should NOT subtract a tan(alpha_max/2) term.
    w_i = (0.0, 0.0)
    w_next = (50000.0, 0.0)
    w_next_next = (100000.0, 0.0)
    ok, msg = prep.validate_kinodynamics(
        w_i, 0.0, w_next, 0.0,
        w_next_next=w_next_next, heading_next_next=0.0,
        R=8000.0, alpha_max=math.radians(30.0))
    assert ok is True, msg


def test_validate_rejects_too_short_leg():
    w_i = (0.0, 0.0)
    w_next = (100.0, 0.0)   # 100 m leg, far too short for R=8000 turns
    ok, _ = prep.validate_kinodynamics(
        w_i, 0.0, w_next, math.radians(20.0),
        R=8000.0, alpha_max=math.radians(30.0))
    assert ok is False
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest preprocessing_test.py -v`
Expected: FAIL — first test fails because `alpha_next` is hardcoded to `alpha_max`, over-subtracting on a straight run.

- [ ] **Step 3: Write minimal implementation**

In `preprocessing.py` `validate_kinodynamics`, replace the `if alpha_max is not None:` block with:

```python
    # Straight-segment (đoản trình) check.
    if w_next_next is not None and heading_next_next is not None:
        delta_next = heading_next_next - heading_next
        alpha_next = abs(math.atan2(math.sin(delta_next), math.cos(delta_next)))
    else:
        alpha_next = alpha_max if alpha_max is not None else 0.0

    d_segment = math.hypot(w_next[0] - w_i[0], w_next[1] - w_i[1])
    l_required = d_segment - R * (math.tan(alpha / 2) + math.tan(alpha_next / 2))
    if l_required < 10.0:
        return False, f"Straight segment length {l_required:.2f}m too small (need > 10m)"
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest preprocessing_test.py characterization_test.py -v`
Expected: PASS (characterization unchanged because callers still pass no triple yet).

- [ ] **Step 5: Commit**

```bash
git add preprocessing.py preprocessing_test.py
git commit -m "fix: validate_kinodynamics uses actual next turn over a triple (I4)"
```

---

### Task 3.2: Re-enable kinodynamic validation + arc collision on radial successors (C2, M5)

**Files:**
- Modify: `kinodynamic_astar.py` (`get_next_states`)
- Test: `kinodynamic_astar_test.py`

Radial successors currently skip validation. Re-enable the turn/straight check and reject any successor whose turn arc would clip an obstacle.

- [ ] **Step 1: Write the failing test**

```python
# append to kinodynamic_astar_test.py
import path_validation as pv


def test_produced_path_is_fully_valid_around_circle():
    pre = _simple_pre(circles=[((50000.0, 0.0), 20000.0)])
    import graph_builder as gb
    tg = gb.generate_bitangents(pre['circle_obstacles'], pre['polygon_obstacles'])
    tg = gb.extend_tangent_graph_with_start_goal(
        tg, pre['start_state']['waypoint'], pre['start_state']['heading'],
        pre['goal_state']['waypoint'], pre['goal_state']['heading'],
        pre['circle_obstacles'], pre['polygon_obstacles'])
    path = astar.KinodynamicAstar(pre, tg).search()
    assert path is not None
    assert pv.turn_angles_ok(path, pre['alpha_max_rad'])
    assert pv.arcs_clear(path, pre['turn_radius'],
                         pre['circle_obstacles'], pre['polygon_obstacles'])
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest kinodynamic_astar_test.py::test_produced_path_is_fully_valid_around_circle -v`
Expected: FAIL — `arcs_clear` (and possibly `turn_angles_ok`) fails because radial successors are unvalidated.

- [ ] **Step 3: Write minimal implementation**

In `get_next_states` Strategy 2, after the bounds and `_check_collision` checks, re-enable validation and add an arc-clearance gate before appending:

```python
            if not self._check_collision(current_state.waypoint, next_waypoint):
                continue
            is_valid, _ = prep.validate_kinodynamics(
                current_state.waypoint, current_state.heading,
                next_waypoint, next_heading,
                R=self.R, alpha_max=self.alpha_max_rad)
            if not is_valid:
                continue
            if not self._arc_clear(current_state.waypoint, current_state.heading, next_heading):
                continue
            successors.append((State(next_waypoint, next_heading), distance))
```

Add the `_arc_clear` method to `KinodynamicAstar` (reuses Phase-0 geometry):

```python
    def _arc_clear(self, w, h_in, h_out, n=12):
        import path_validation as pv
        alpha = abs(pv._norm(h_out - h_in))
        if alpha < 1e-9:
            return True
        t = self.R * math.tan(alpha / 2)
        u = (math.cos(h_in), math.sin(h_in))
        v = (math.cos(h_out), math.sin(h_out))
        A = (w[0] - u[0] * t, w[1] - u[1] * t)
        Bp = (w[0] + v[0] * t, w[1] + v[1] * t)
        pts = pv._arc_points(A, w, Bp, self.R, n)
        for j in range(len(pts) - 1):
            if not self._check_collision(pts[j], pts[j + 1]):
                return False
        return True
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest kinodynamic_astar_test.py characterization_test.py -v -s`
Expected: PASS. Re-lock any characterization values that shift (paths may get slightly longer/cleaner). If a scenario regresses to no-path, the validation is too strict combined with the coarse radial step — proceed to Phase 4 Task 4.4 (adaptive step) which is the intended remedy; meanwhile keep this task green on the single-circle and empty cases and note the regression in the commit body.

- [ ] **Step 5: Commit**

```bash
git add kinodynamic_astar.py kinodynamic_astar_test.py characterization_test.py
git commit -m "fix: enforce turn/straight constraints and arc clearance on radial successors (C2, M5)"
```

---

### Task 3.3: Correct endpoint turn-angle terms (M3)

**Files:**
- Modify: `preprocessing.py` (`calculate_start_state`, `calculate_end_state`)
- Test: `preprocessing_test.py`

Spec: `d₁ = l₁ + R·tan(|α₁|/2)` with `l₁ ≥ L₀`. With the true first turn unknown at preprocessing, the conservative choice `α₁ = α_max` is acceptable *and intentional*, but the start docstring claims `α₁ ≈ 0`. Lock the chosen convention with a test so it can't drift, and keep `l₁ = L₀` exactly.

- [ ] **Step 1: Write the failing test**

```python
# append to preprocessing_test.py
import config


def test_start_state_distance_matches_spec_with_alpha_max():
    st = prep.calculate_start_state((0.0, 0.0), 0.0,
                                    L0=config.L0, R=config.R,
                                    alpha_max_rad=config.ALPHA_MAX_RAD)
    expected_d = config.L0 + config.R * math.tan(config.ALPHA_MAX_RAD / 2)
    got_d = math.hypot(*st['waypoint'])
    assert abs(got_d - expected_d) < 1e-6
    assert abs(st['straight_length'] - config.L0) < 1e-6


def test_end_state_distance_matches_spec_with_alpha_max():
    end = prep.calculate_end_state((100000.0, 0.0), 0.0,
                                   dss=config.DSS, R=config.R,
                                   alpha_max_rad=config.ALPHA_MAX_RAD)
    expected_d = config.DSS + config.R * math.tan(config.ALPHA_MAX_RAD / 2)
    got_d = math.hypot(100000.0 - end['waypoint'][0], 0.0 - end['waypoint'][1])
    assert abs(got_d - expected_d) < 1e-6
```

- [ ] **Step 2: Run to verify it passes or fails**

Run: `python -m pytest preprocessing_test.py -k "start_state_distance or end_state_distance" -v`
Expected: PASS already for the math (the code uses `alpha_max`); if it fails, the formula drifted. The real change is the misleading docstring.

- [ ] **Step 3: Fix the docstring (no behavior change)**

In `calculate_start_state`, replace the comment `# With minimal turn angle α_1 ≈ 0, d_1 ≈ l_1` with:

```python
    # Conservative: reserve tangent length for the worst-case first turn α₁ = α_max,
    # so d₁ = L0 + R*tan(α_max/2) and l₁ = L0 exactly (l₁ ≥ L0 holds).
```

- [ ] **Step 4: Run to verify green**

Run: `python -m pytest preprocessing_test.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add preprocessing.py preprocessing_test.py
git commit -m "docs+test: lock conservative endpoint turn-angle convention (M3)"
```

---

# PHASE 4 — Path-quality optimization

**Outcome:** Shorter, smoother paths with fewer turns; optimality not silently broken.

### Task 4.1: Re-enable path smoothing (C3a)

**Files:**
- Modify: `kinodynamic_astar.py` (`plan_trajectory`)
- Test: `kinodynamic_astar_test.py`

- [ ] **Step 1: Write the failing test**

```python
# append to kinodynamic_astar_test.py
def test_smoothing_does_not_increase_waypoints_or_break_validity():
    pre = _simple_pre(circles=[((50000.0, 0.0), 20000.0)])
    result = astar.plan_trajectory(pre, verbose=False)
    assert result['success']
    raw = result['planner'].search()  # unsmoothed reference
    smoothed = result['path']
    assert len(smoothed) <= len(raw)
    import path_validation as pv
    assert pv.segments_clear(smoothed, pre['circle_obstacles'], pre['polygon_obstacles'])
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest kinodynamic_astar_test.py::test_smoothing_does_not_increase_waypoints_or_break_validity -v`
Expected: FAIL — `plan_trajectory` returns the raw path (smoothing commented out), so `len(smoothed) == len(raw)` may pass but validity/structure differs; more importantly the smoothing path is dead.

- [ ] **Step 3: Write minimal implementation**

In `plan_trajectory`, replace the commented block:

```python
    # Smooth path if found
    # if path:
    #     path = planner.smooth_path(path)
```

with:

```python
    # Smooth path if found
    if path:
        path = planner.smooth_path(path)
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest kinodynamic_astar_test.py characterization_test.py -v -s`
Expected: PASS. Re-lock characterization values that shrink (fewer waypoints/turns). Confirm `smooth_path` still validates shortcuts via `validate_kinodynamics` (it does, lines 312-319).

- [ ] **Step 5: Commit**

```bash
git add kinodynamic_astar.py kinodynamic_astar_test.py characterization_test.py
git commit -m "feat: re-enable path smoothing (C3a); re-lock baseline"
```

---

### Task 4.2: Penalize turns in the cost (C3b)

**Files:**
- Modify: `kinodynamic_astar.py` (`get_next_states` cost terms), `config.py`
- Test: `kinodynamic_astar_test.py`

Add a turn penalty to transition cost so equal-length paths prefer fewer/gentler turns.

- [ ] **Step 1: Add config knob**

```python
# config.py
# Cost added per radian of heading change at a transition (meters per radian)
TURN_PENALTY_WEIGHT = 4000.0
```

- [ ] **Step 2: Write the failing test**

```python
# append to kinodynamic_astar_test.py
def test_turn_penalty_in_transition_cost():
    pre = _simple_pre()
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    succ = planner.get_next_states(planner.start_state)
    # the straight-ahead successor must be strictly cheaper than a turning one
    straight = min(succ, key=lambda s: abs(s[0].heading - planner.start_state.heading))
    turned = max(succ, key=lambda s: abs(s[0].heading - planner.start_state.heading))
    # same step distance, so turned must cost more once a turn penalty exists
    assert turned[1] > straight[1]
```

- [ ] **Step 3: Run to verify it fails**

Run: `python -m pytest kinodynamic_astar_test.py::test_turn_penalty_in_transition_cost -v`
Expected: FAIL — radial successors all share the same `distance` cost today.

- [ ] **Step 4: Write minimal implementation**

In `get_next_states` Strategy 2, change the appended cost to include the turn penalty:

```python
            turn = abs(prep_norm(next_heading - current_state.heading))
            cost = distance + config.TURN_PENALTY_WEIGHT * turn
            successors.append((State(next_waypoint, next_heading), cost))
```

Add a small helper at module top of `kinodynamic_astar.py` (or reuse `math.atan2`):

```python
def prep_norm(delta):
    return math.atan2(math.sin(delta), math.cos(delta))
```

Apply the same penalty to Strategy 1's `cost` (turn = `heading_to_node - current_state.heading`).

- [ ] **Step 5: Run to verify it passes and quality improves**

Run: `python -m pytest kinodynamic_astar_test.py characterization_test.py -v -s`
Expected: PASS. Characterization `num_turns` should not increase; re-lock changed values. Note: cost no longer equals pure length, so the `total_length_m` characterization still measures geometric length (unchanged definition).

- [ ] **Step 6: Commit**

```bash
git add kinodynamic_astar.py config.py kinodynamic_astar_test.py characterization_test.py
git commit -m "feat: add turn penalty to transition cost for fewer turns (C3b)"
```

---

### Task 4.3: Admissible heuristic (I1)

**Files:**
- Modify: `kinodynamic_astar.py` (`heuristic`)
- Test: `kinodynamic_astar_test.py`

The current `dist + R·heading_diff` over-estimates (heading is corrected en route). Drop the additive heading term to restore admissibility (Euclidean distance is an admissible lower bound on remaining length; the turn penalty lives in g, not h).

- [ ] **Step 1: Write the failing test**

```python
# append to kinodynamic_astar_test.py
def test_heuristic_is_euclidean_lower_bound():
    pre = _simple_pre()
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    s = planner.start_state
    g = planner.goal_state
    import math as m
    euclid = m.hypot(g.waypoint[0] - s.waypoint[0], g.waypoint[1] - s.waypoint[1])
    h = planner.heuristic(s, g)
    assert abs(h - euclid) < 1e-6, "heuristic must equal Euclidean distance (admissible)"
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest kinodynamic_astar_test.py::test_heuristic_is_euclidean_lower_bound -v`
Expected: FAIL — current heuristic adds `R·heading_diff`.

- [ ] **Step 3: Write minimal implementation**

Replace `heuristic` body's return:

```python
    def heuristic(self, state, goal_state):
        dx = goal_state.waypoint[0] - state.waypoint[0]
        dy = goal_state.waypoint[1] - state.waypoint[1]
        return math.sqrt(dx * dx + dy * dy)
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest kinodynamic_astar_test.py characterization_test.py -v -s`
Expected: PASS. Re-lock changed characterization values (paths may get shorter). Confirm obstacle scenarios still terminate quickly.

- [ ] **Step 5: Commit**

```bash
git add kinodynamic_astar.py kinodynamic_astar_test.py characterization_test.py
git commit -m "fix: make A* heuristic admissible (Euclidean lower bound) (I1)"
```

---

### Task 4.4: Adaptive radial step toward the goal (I5, I6)

**Files:**
- Modify: `kinodynamic_astar.py` (`get_next_states`, goal handling), `config.py`
- Test: `kinodynamic_astar_test.py`

Add one extra "straight-toward-goal, distance = min(step, dist-to-goal)" successor so the search can land inside `GOAL_THRESHOLD` and avoid pure staircasing on the home stretch.

- [ ] **Step 1: Optionally widen the goal threshold**

In `config.py`, raise `GOAL_THRESHOLD` from 200.0 to a value the lattice can hit reliably:

```python
GOAL_THRESHOLD = 1000.0  # meters; reachable given STATE_POS_QUANTUM
```

- [ ] **Step 2: Write the failing test**

```python
# append to kinodynamic_astar_test.py
def test_goal_directed_successor_exists():
    pre = _simple_pre()  # empty 100 km map, no graph
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    succ = planner.get_next_states(planner.start_state)
    gh = __import__('spatial_utils').angle_to_heading(
        planner.start_state.waypoint, planner.goal_state.waypoint)
    # at least one successor heads (within a few degrees) straight at the goal
    assert any(abs(__import__('math').atan2(
        __import__('math').sin(s[0].heading - gh),
        __import__('math').cos(s[0].heading - gh))) < __import__('math').radians(7)
        for s in succ)
```

- [ ] **Step 3: Run to verify it fails**

Run: `python -m pytest kinodynamic_astar_test.py::test_goal_directed_successor_exists -v`
Expected: may already pass if a radial offset lands within 7°; if it fails, that confirms the staircase gap.

- [ ] **Step 4: Write minimal implementation**

At the end of `get_next_states`, before `return successors`, add a goal-directed successor when the goal is within line of sight and a valid turn:

```python
        goal_wp = self.goal_state.waypoint
        gh = su.angle_to_heading(current_state.waypoint, goal_wp)
        turn = abs(prep_norm(gh - current_state.heading))
        if turn <= self.alpha_max_rad:
            d = math.hypot(goal_wp[0] - current_state.waypoint[0],
                           goal_wp[1] - current_state.waypoint[1])
            step = min(d, 2 * self.R * math.tan(self.alpha_max_rad / 2))
            cand = (current_state.waypoint[0] + step * math.cos(gh),
                    current_state.waypoint[1] + step * math.sin(gh))
            if self._in_bounds(cand) and self._check_collision(current_state.waypoint, cand) \
                    and self._arc_clear(current_state.waypoint, current_state.heading, gh):
                successors.append((State(cand, gh),
                                   step + config.TURN_PENALTY_WEIGHT * turn))
```

- [ ] **Step 5: Run to verify it passes**

Run: `python -m pytest kinodynamic_astar_test.py characterization_test.py -v -s`
Expected: PASS; obstacle paths reach the goal more directly with fewer waypoints. Re-lock characterization.

- [ ] **Step 6: Commit**

```bash
git add kinodynamic_astar.py config.py kinodynamic_astar_test.py characterization_test.py
git commit -m "feat: goal-directed successor + reachable goal threshold (I5, I6)"
```

---

# PHASE 5 — Cleanups

### Task 5.1: Honest LOS factor + comment (M1)

**Files:**
- Modify: `graph_builder.py` (`extend_tangent_graph_with_start_goal`)
- Test: `graph_builder_test.py`

- [ ] **Step 1: Write the failing test**

```python
# append to graph_builder_test.py
def test_start_goal_los_uses_full_radius():
    # A circle exactly between start and goal must block the direct edge.
    circles = [((50000.0, 0.0), 20000.0)]
    g = gb.generate_bitangents(circles, [])
    g = gb.extend_tangent_graph_with_start_goal(
        g, (0.0, 0.0), 0.0, (100000.0, 0.0), 0.0, circles, [])
    # No direct start->goal edge should exist (line passes through the circle).
    assert g.find_node_index((0.0, 0.0)) is not None
    neighbors = [p for p, _ in g.get_neighbors((0.0, 0.0))]
    assert (100000.0, 0.0) not in neighbors
```

- [ ] **Step 2: Run to verify it fails or passes**

Run: `python -m pytest graph_builder_test.py::test_start_goal_los_uses_full_radius -v`
Expected: With the `0.8` factor the direct line is blocked anyway (dist 0 < r·0.8), so this likely PASSES; the change is to remove the misleading 20% slack and fix the comment.

- [ ] **Step 3: Implementation**

In `extend_tangent_graph_with_start_goal`, change all three `circle[1] * 0.8` comparisons to `circle[1]` and update the stale comment `# More lenient LOS check (0.9 factor...)` to `# LOS blocked if the segment enters the (already-inflated) obstacle`.

- [ ] **Step 4: Run to verify green**

Run: `python -m pytest graph_builder_test.py characterization_test.py -v`
Expected: PASS. Re-lock characterization if any edge change shifts a path.

- [ ] **Step 5: Commit**

```bash
git add graph_builder.py graph_builder_test.py characterization_test.py
git commit -m "fix: full-radius LOS for start/goal edges; fix stale comment (M1)"
```

---

### Task 5.2: Use convex-hull tangents for polygons instead of bounding circles (M2)

**Files:**
- Modify: `graph_builder.py` (`generate_bitangents` polygon-to-circle approximation)
- Test: `graph_builder_test.py`

The current `(centroid, max-vertex-radius)` circle is very coarse for elongated polygons. Since Task 1.3 already adds polygon **hull vertices** as visibility nodes, the bounding-circle bitangent approximation is now redundant and can be dropped to avoid over-inflated detours.

- [ ] **Step 1: Write the failing test**

```python
# append to graph_builder_test.py
def test_elongated_polygon_nodes_track_hull_not_bounding_circle():
    poly = [(0.0, 0.0), (100000.0, 0.0), (100000.0, 2000.0), (0.0, 2000.0)]
    g = gb.generate_bitangents([], [poly])
    # hull vertices (4) must be present as nodes
    for v in [(0.0, 0.0), (100000.0, 0.0), (100000.0, 2000.0), (0.0, 2000.0)]:
        assert g.find_node_index(v) is not None
    # no node should sit ~50 km away on a bounding-circle radius
    for n in g.nodes:
        assert abs(n[0]) <= 100000.0 + 1.0 and abs(n[1]) <= 2000.0 + 1.0
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest graph_builder_test.py::test_elongated_polygon_nodes_track_hull_not_bounding_circle -v`
Expected: FAIL — bounding-circle approximation may create far-off tangent nodes.

- [ ] **Step 3: Implementation**

In `generate_bitangents`, restrict the bounding-circle list to *circle* obstacles only (drop the polygon→circle conversion in `all_obstacles`); polygons are now represented purely by their hull vertices via `_add_visibility_nodes`:

```python
    all_obstacles = [(c, r) for c, r in circle_obstacles]
```

- [ ] **Step 4: Run to verify green**

Run: `python -m pytest graph_builder_test.py kinodynamic_astar_test.py characterization_test.py -v`
Expected: PASS. Re-lock characterization if polygon-scenario paths shorten.

- [ ] **Step 5: Commit**

```bash
git add graph_builder.py graph_builder_test.py characterization_test.py
git commit -m "fix: represent polygons by hull vertices, drop bounding-circle tangents (M2)"
```

---

## Final verification

- [ ] Run the whole suite: `python -m pytest -q`
- [ ] Run the full standalone report: `python characterization_test.py` and eyeball that obstacle scenarios now show `valid=yes`, sane lengths/turns, and runtimes in seconds.
- [ ] Run the original harness for a smoke test: `python main.py` (writes PNGs to `results/`), confirm success rate improved and no crashes.

---

## Notes on residual risk (read before Phase 3+)

- **C2 before C5/I2 would make things worse.** Enforcing constraints on an exploding, slow search amplifies failures. The ordering above (perf first, constraints after) is deliberate.
- **Lattice coarseness (C5) trades completeness for speed.** If a narrow channel needs sub-`STATE_POS_QUANTUM` precision, the search may miss it. The `two_circles_gap` scenario guards the common case; add a tighter scenario if real maps need it.
- **Turn penalty + admissible heuristic (Phase 4) interact.** Tune `TURN_PENALTY_WEIGHT` only with the characterization suite watching `num_turns` vs `total_length_m`; do not tune blind.
- **Arc clearance (`_arc_clear`) is sampled, not analytic.** 12 samples/arc is a balance; if a thin obstacle slips between samples, increase `n`. The Phase-0 validator (`arcs_clear`, 24 samples) is the stricter gate the tests assert against.
