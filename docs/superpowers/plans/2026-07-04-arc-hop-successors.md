# Arc-Hop Successors Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the `WRAP_STEP_M` straight-step circle-wrapping mechanism in the kinodynamic A\* planner with exact-geometry "arc-hop" successors, so path quality no longer depends on a wrap discretisation parameter.

**Architecture:** A new pure-geometry module `core/arc_geometry.py` provides bitangent/departure-point/arc-expansion math. `core/kinodynamic_astar.py` gains an `_arc_hop_successors` generator (riding states hop along a circle's boundary to tangent-continuous departure points, cost = arc length) and expands arcs into circumscribed-polygon waypoints only at path-reconstruction time. Strategy B becomes: pure fallback when no candidate exists, plus leave-the-boundary fan options at riding states. Smoothing is re-enabled. GUI is untouched in this plan (its wrap slider keeps writing the now-deprecated `config.WRAP_STEP_M`; a follow-up GUI task will swap it).

**Tech Stack:** Python 3, math/shapely (STRtree, LineString, Polygon), pytest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-07-04-arc-hop-successors-design.md`

## Global Constraints

- Units are meters and radians in all algorithm code; `config` angle constants are stored in degrees (`ARC_WAYPOINT_STEP_DEG`, `ARC_SAMPLE_STEP_DEG` follow this).
- Committed test files are named `tests/*_test.py` (never `test_*.py` — that pattern is for gitignored scratch).
- `ARC_WAYPOINT_STEP_DEG = 30.0` (max supported 45); `ARC_SAMPLE_STEP_DEG = 5.0`.
- Arc clearance is checked at the **fixed** 45°-bulge radius `r / cos(π/8)` so search connectivity is independent of `ARC_WAYPOINT_STEP_DEG`.
- Planning must finish in < 5 s per scenario (tests assert this with `TIME_BUDGET_S` disabled).
- Wrap sense convention: `s = +1` CCW, `s = -1` CW, `s = sign(cross(P − center, heading_vec))`.
- Pipeline contracts unchanged: scenario dict, preprocessed dict, path = list of `(waypoint, heading)` tuples; `path_validation` oracle stays untouched.
- Run tests from the repo root: `python -m pytest -q`.

---

### Task 1: Test scaffolding + `core/arc_geometry.py` basics

**Files:**
- Create: `pytest.ini`
- Create: `tests/arc_geometry_test.py`
- Create: `core/arc_geometry.py`

**Interfaces:**
- Consumes: nothing (pure math + `core.spatial_utils` later).
- Produces (used by Tasks 2–4):
  - `riding_sense(P, h, center, r, pos_tol=1.0, ang_tol=8.72e-3) -> int` (0 = not riding, ±1 = wrap sense)
  - `tangent_heading(P, center, s) -> float` (radians)
  - `arc_angle(P, Q, center, s) -> float` (Δφ in `[0, 2π)` measured from P to Q in direction s)

- [ ] **Step 1: Create pytest.ini** (the repo has none; CLAUDE.md describes this exact content)

```ini
[pytest]
pythonpath = .
testpaths = tests
python_files = *_test.py
```

- [ ] **Step 2: Write the failing tests**

Create `tests/arc_geometry_test.py`:

```python
"""Unit tests for core/arc_geometry.py (pure geometry, no planner)."""
import math

import core.arc_geometry as ag

C = (100000.0, 100000.0)
R_C = 20000.0


def test_riding_sense_ccw():
    # Point due east of center, heading north => CCW tangent
    P = (C[0] + R_C, C[1])
    assert ag.riding_sense(P, math.pi / 2, C, R_C) == 1


def test_riding_sense_cw():
    P = (C[0] + R_C, C[1])
    assert ag.riding_sense(P, -math.pi / 2, C, R_C) == -1


def test_riding_sense_rejects_off_boundary():
    P = (C[0] + R_C + 50.0, C[1])  # 50 m off the boundary; tol is 1 m
    assert ag.riding_sense(P, math.pi / 2, C, R_C) == 0


def test_riding_sense_rejects_non_tangent_heading():
    P = (C[0] + R_C, C[1])
    assert ag.riding_sense(P, math.pi / 4, C, R_C) == 0  # 45 deg off tangent


def test_tangent_heading_matches_sense():
    P = (C[0] + R_C, C[1])
    assert math.isclose(ag.tangent_heading(P, C, +1), math.pi / 2, abs_tol=1e-9)
    assert math.isclose(ag.tangent_heading(P, C, -1), -math.pi / 2, abs_tol=1e-9)


def test_arc_angle_quarter_turns():
    P = (C[0] + R_C, C[1])  # polar angle 0
    Q = (C[0], C[1] + R_C)  # polar angle pi/2
    assert math.isclose(ag.arc_angle(P, Q, C, +1), math.pi / 2, rel_tol=1e-9)
    assert math.isclose(ag.arc_angle(P, Q, C, -1), 3 * math.pi / 2, rel_tol=1e-9)


def test_arc_angle_same_point_is_zero():
    P = (C[0] + R_C, C[1])
    assert ag.arc_angle(P, P, C, +1) == 0.0
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `python -m pytest tests/arc_geometry_test.py -v`
Expected: errors — `ModuleNotFoundError: No module named 'core.arc_geometry'`

- [ ] **Step 4: Write the implementation**

Create `core/arc_geometry.py`:

```python
"""Pure geometry for arc-hop successors: riding detection, tangent headings,
bitangents between circles, departure points, and output-time arc expansion.

Wrap sense convention: s = +1 rides a circle counter-clockwise, s = -1
clockwise; s = sign(cross(P - center, heading_vector)).
No planner or config imports — all tolerances/steps are parameters.
"""
import math

import core.spatial_utils as su


def riding_sense(P, h, center, r, pos_tol=1.0, ang_tol=8.72e-3):
    """0 if (P, h) does not ride circle (center, r); else the wrap sense ±1.

    Riding means: P on the boundary (within pos_tol meters) AND heading h
    tangent to the circle at P (|dot(radial, heading)| < ang_tol ~ sin 0.5°).
    """
    dx, dy = P[0] - center[0], P[1] - center[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-9 or abs(dist - r) > pos_tol:
        return 0
    ux, uy = dx / dist, dy / dist
    hx, hy = math.cos(h), math.sin(h)
    if abs(ux * hx + uy * hy) > ang_tol:
        return 0
    return 1 if (ux * hy - uy * hx) > 0 else -1


def tangent_heading(P, center, s):
    """Heading (radians) of tangent travel at boundary point P, wrap sense s."""
    return math.atan2(s * (P[0] - center[0]), -s * (P[1] - center[1]))


def arc_angle(P, Q, center, s):
    """Angle in [0, 2π) swept from P to Q around center, in direction s."""
    a0 = math.atan2(P[1] - center[1], P[0] - center[0])
    a1 = math.atan2(Q[1] - center[1], Q[0] - center[0])
    return (s * (a1 - a0)) % (2.0 * math.pi)
```

(`su` is imported now so Task 2 can use `su.circle_tangent_points` without touching the import block; the unused-import interval is one commit.)

- [ ] **Step 5: Run tests to verify they pass**

Run: `python -m pytest tests/arc_geometry_test.py -v`
Expected: 7 passed

- [ ] **Step 6: Commit**

```bash
git add pytest.ini tests/arc_geometry_test.py core/arc_geometry.py
git commit -m "feat: arc geometry basics (riding sense, tangent heading, arc angle)"
```

---

### Task 2: Departure points and bitangents

**Files:**
- Modify: `core/arc_geometry.py`
- Modify: `tests/arc_geometry_test.py`

**Interfaces:**
- Consumes: `su.circle_tangent_points(point, center, radius)` (existing), Task 1 helpers.
- Produces (used by Task 4):
  - `departure_point(X, center, r, s) -> (x, y) | None` — the tangent point on the circle from which leaving toward external point X is consistent with wrap sense s; None if X is inside/on the circle.
  - `bitangent_departures(c1, r1, c2, r2, s) -> list[((depx, depy), (arrx, arry))]` — bitangent lines of the two circles whose departure from circle 1 is consistent with sense s.

- [ ] **Step 1: Write the failing tests** (append to `tests/arc_geometry_test.py`)

```python
import core.spatial_utils as su  # add at top of file with the other imports


def test_departure_point_picks_sense_consistent_tangent():
    X = (C[0] + 100000.0, C[1])  # far east of the circle
    dep_ccw = ag.departure_point(X, C, R_C, +1)
    dep_cw = ag.departure_point(X, C, R_C, -1)
    assert dep_ccw is not None and dep_cw is not None
    # CCW wrap leaves from below the center-line, CW from above.
    assert dep_ccw[1] < C[1]
    assert dep_cw[1] > C[1]
    for s, dep in ((+1, dep_ccw), (-1, dep_cw)):
        # On the boundary
        assert math.isclose(math.hypot(dep[0] - C[0], dep[1] - C[1]), R_C, rel_tol=1e-9)
        # Leave direction actually points toward X
        h = ag.tangent_heading(dep, C, s)
        to_x = math.atan2(X[1] - dep[1], X[0] - dep[0])
        assert abs(math.atan2(math.sin(h - to_x), math.cos(h - to_x))) < 1e-6


def test_departure_point_inside_returns_none():
    assert ag.departure_point((C[0] + 1000.0, C[1]), C, R_C, +1) is None


def test_bitangent_departures_disjoint_circles():
    c1, r1 = (0.0, 0.0), 10000.0
    c2, r2 = (100000.0, 0.0), 10000.0
    res = ag.bitangent_departures(c1, r1, c2, r2, +1)
    assert len(res) == 2  # one outer + one inner survive the sense filter
    for dep, arr in res:
        assert math.isclose(math.hypot(dep[0] - c1[0], dep[1] - c1[1]), r1, rel_tol=1e-9)
        assert math.isclose(math.hypot(arr[0] - c2[0], arr[1] - c2[1]), r2, rel_tol=1e-9)
        # The line dep->arr is tangent to both circles.
        assert math.isclose(su.point_to_line_distance(c1, dep, arr), r1, rel_tol=1e-6)
        assert math.isclose(su.point_to_line_distance(c2, dep, arr), r2, rel_tol=1e-6)
        # Departure is sense-consistent: tangent heading at dep points at arr.
        h = ag.tangent_heading(dep, c1, +1)
        to_arr = math.atan2(arr[1] - dep[1], arr[0] - dep[0])
        assert abs(math.atan2(math.sin(h - to_arr), math.cos(h - to_arr))) < 1e-6
    # CCW wrap on c1 must include the bottom outer tangent.
    assert any(dep[1] < 0 for dep, _ in res)


def test_bitangent_departures_overlapping_circles_outer_only():
    c1, r1 = (0.0, 0.0), 20000.0
    c2, r2 = (30000.0, 0.0), 20000.0  # overlapping: inner tangents impossible
    res = ag.bitangent_departures(c1, r1, c2, r2, +1)
    assert len(res) == 1


def test_bitangent_departures_concentric_returns_empty():
    assert ag.bitangent_departures((0.0, 0.0), 10000.0, (0.0, 0.0), 5000.0, +1) == []
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/arc_geometry_test.py -v`
Expected: 7 pass (Task 1), 5 FAIL with `AttributeError: ... has no attribute 'departure_point'`

- [ ] **Step 3: Write the implementation** (append to `core/arc_geometry.py`)

```python
def departure_point(X, center, r, s):
    """Tangent point on circle (center, r) from which leaving toward the
    external point X is tangent-continuous for wrap sense s. None if X is
    inside or on the circle."""
    for dep in su.circle_tangent_points(X, center, r):
        nx = (dep[0] - center[0]) / r
        ny = (dep[1] - center[1]) / r
        # velocity at dep for sense s is s * perp_ccw(n) = (-s*ny, s*nx)
        if (-s * ny) * (X[0] - dep[0]) + (s * nx) * (X[1] - dep[1]) > 0:
            return dep
    return None


def bitangent_departures(c1, r1, c2, r2, s):
    """Bitangent lines of circles (c1, r1) and (c2, r2), filtered to those
    departing circle 1 consistently with wrap sense s.

    Construction: a bitangent touches circle 1 at c1 + r1*n and circle 2 at
    c2 + sigma*r2*n for a unit normal n with n·D̂ = (r1 - sigma*r2)/d, where
    sigma=+1 gives the outer pair and sigma=-1 the inner (crossing) pair.
    Returns [(dep_on_c1, arr_on_c2), ...] (0..2 entries after filtering).
    """
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return []
    ux, uy = dx / d, dy / d
    out = []
    for sigma in (1.0, -1.0):
        k = (r1 - sigma * r2) / d
        if abs(k) > 1.0:
            continue
        root = math.sqrt(max(0.0, 1.0 - k * k))
        for pm in (1.0, -1.0):
            nx = k * ux - pm * root * uy
            ny = k * uy + pm * root * ux
            dep = (c1[0] + r1 * nx, c1[1] + r1 * ny)
            arr = (c2[0] + sigma * r2 * nx, c2[1] + sigma * r2 * ny)
            tx, ty = arr[0] - dep[0], arr[1] - dep[1]
            if math.hypot(tx, ty) < 1e-6:
                continue  # circles touch: degenerate tangent
            if (-s * ny) * tx + (s * nx) * ty > 0:  # sense-consistent at dep
                out.append((dep, arr))
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/arc_geometry_test.py -v`
Expected: 12 passed

- [ ] **Step 5: Commit**

```bash
git add core/arc_geometry.py tests/arc_geometry_test.py
git commit -m "feat: departure points and sense-filtered bitangents"
```

---

### Task 3: Arc → waypoint expansion (`arc_waypoints`)

**Files:**
- Modify: `core/arc_geometry.py`
- Modify: `tests/arc_geometry_test.py`

**Interfaces:**
- Consumes: `tangent_heading` (Task 1).
- Produces (used by Task 4's `_reconstruct_path`):
  - `arc_waypoints(center, r, start_pt, dphi, s, theta_max_rad) -> list[((x, y), heading)]` — circumscribed-polygon vertices replacing the boundary arc from `start_pt` sweeping `dphi` in direction `s`; excludes the arc's own endpoints; each vertex's heading is its outgoing tangent direction.

- [ ] **Step 1: Write the failing tests** (append to `tests/arc_geometry_test.py`)

```python
import core.path_validation as pv  # add at top of file with the other imports


def test_arc_waypoints_quarter_wrap_vertex_geometry():
    start = (C[0] + R_C, C[1])  # polar angle 0
    dphi = math.pi / 2
    theta = math.radians(30.0)
    wps = ag.arc_waypoints(C, R_C, start, dphi, +1, theta)
    assert len(wps) == 3  # ceil(90/30) = 3 vertices
    rv = R_C / math.cos((dphi / 3) / 2)
    for v, _h in wps:
        assert math.isclose(math.hypot(v[0] - C[0], v[1] - C[1]), rv, rel_tol=1e-9)


def test_arc_waypoint_chain_turns_and_clearance():
    """The full chain start -> vertices -> end must satisfy exactly what the
    oracle checks: every turn <= theta and no chord entering the circle."""
    start = (C[0] + R_C, C[1])
    dphi = 1.75 * math.pi  # long wrap
    theta = math.radians(30.0)
    wps = ag.arc_waypoints(C, R_C, start, dphi, +1, theta)
    end = (C[0] + R_C * math.cos(dphi), C[1] + R_C * math.sin(dphi))
    chain = [(start, 0.0)] + wps + [(end, 0.0)]
    for a in pv.turn_angles(chain):
        assert a <= theta + 1e-9
    for i in range(len(chain) - 1):
        d = su.point_to_line_distance(C, chain[i][0], chain[i + 1][0])
        assert d >= R_C - 1e-6  # chords are tangent, never inside


def test_arc_waypoints_cw_sense():
    start = (C[0] + R_C, C[1])
    theta = math.radians(30.0)
    wps = ag.arc_waypoints(C, R_C, start, math.pi / 2, -1, theta)
    assert len(wps) == 3
    assert all(v[1] < C[1] for v, _h in wps)  # CW from angle 0 goes below


def test_arc_waypoints_zero_angle_empty():
    start = (C[0] + R_C, C[1])
    assert ag.arc_waypoints(C, R_C, start, 0.0, +1, math.radians(30.0)) == []
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/arc_geometry_test.py -v`
Expected: 12 pass, 4 FAIL with `AttributeError: ... has no attribute 'arc_waypoints'`

- [ ] **Step 3: Write the implementation** (append to `core/arc_geometry.py`)

```python
def arc_waypoints(center, r, start_pt, dphi, s, theta_max_rad):
    """Expand a boundary arc into circumscribed-polygon vertices.

    The arc starts at start_pt (on the circle) and sweeps dphi (rad) in
    direction s. It is replaced by n = ceil(dphi / theta_max_rad) equal turns
    of theta = dphi/n each; vertex k is the intersection of consecutive
    tangent lines, at radius r / cos(theta/2) on the bisector. Headings are
    the outgoing tangent directions, so the turn angle at every vertex is
    exactly theta <= theta_max_rad, and every chord of the resulting chain is
    tangent to the circle (never inside it).

    Returns [(vertex, heading_out), ...] excluding the arc endpoints.
    """
    if dphi <= 1e-9:
        return []
    n = max(1, int(math.ceil(dphi / theta_max_rad)))
    step = dphi / n
    rv = r / math.cos(step / 2.0)
    phi0 = math.atan2(start_pt[1] - center[1], start_pt[0] - center[0])
    out = []
    for k in range(n):
        mid = phi0 + s * step * (k + 0.5)
        vertex = (center[0] + rv * math.cos(mid), center[1] + rv * math.sin(mid))
        nxt = phi0 + s * step * (k + 1)
        tangent_pt = (center[0] + r * math.cos(nxt), center[1] + r * math.sin(nxt))
        out.append((vertex, tangent_heading(tangent_pt, center, s)))
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/arc_geometry_test.py -v`
Expected: 16 passed

- [ ] **Step 5: Commit**

```bash
git add core/arc_geometry.py tests/arc_geometry_test.py
git commit -m "feat: arc-to-waypoint expansion (circumscribed polygon)"
```

---

### Task 4: Planner integration — arc-hop successors replace the wrap step

**Files:**
- Modify: `config.py` (add two constants; `WRAP_STEP_M` stays until Task 6 because `gui/params.py` still reads it)
- Modify: `core/kinodynamic_astar.py`
- Create: `tests/kinodynamic_arc_hop_test.py`

**Interfaces:**
- Consumes: `ag.riding_sense`, `ag.tangent_heading`, `ag.arc_angle`, `ag.departure_point`, `ag.bitangent_departures`, `ag.arc_waypoints`.
- Produces (used by Tasks 5, 7):
  - `State.arc_from`: `None` or `(center, radius, arc_start_pt, s)` on states reached via an arc hop.
  - `KinodynamicAstar._arc_hop_successors(state) -> list[(State, cost)]`
  - `KinodynamicAstar.raw_route`: list of `(waypoint, heading)` — the search route **before** arc expansion and smoothing (set by `_reconstruct_path`; `None` until a path is found).
  - `_check_collision(p1, p2, skip_circle=None)` — unchanged behavior when `skip_circle` is omitted.

- [ ] **Step 1: Write the failing tests**

Create `tests/kinodynamic_arc_hop_test.py`:

```python
"""Planner-level tests for arc-hop successor generation (synthetic maps)."""
import math

import config
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.arc_geometry as ag
import core.path_validation as pv
import render.trajectory as tr

CENTER = (250000.0, 250000.0)
RAW_R = 30000.0


def synthetic_circle_scenario():
    """One raw circle dead-center between a west start and an east goal."""
    return {
        'start': (50000.0, 250000.0), 'start_heading': 0.0,
        'goal': (450000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [],
        'dynamic_obstacles': [(CENTER, RAW_R)],
        'obstacles': [{'type': 'circle', 'center': CENTER, 'radius': RAW_R}],
    }


def test_arc_hop_successors_from_riding_state():
    pre = prep.prepare_scenario(synthetic_circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    (_, r_inf), = pre['circle_obstacles']
    P = (CENTER[0], CENTER[1] - r_inf)  # due south, heading east => CCW
    st = astar.State(P, 0.0)
    succ = planner._arc_hop_successors(st)
    assert succ, "a riding state must generate arc-hop successors"
    for nxt, cost in succ:
        center, radius, arc_start, s = nxt.arc_from
        assert (center, radius, s) == (CENTER, r_inf, 1)
        assert arc_start == P
        dphi = ag.arc_angle(P, nxt.waypoint, center, s)
        assert math.isclose(cost, radius * dphi, rel_tol=1e-9)
        assert math.isclose(
            math.hypot(nxt.waypoint[0] - center[0], nxt.waypoint[1] - center[1]),
            radius, rel_tol=1e-9)
    # The goal's departure point must be among the successors.
    dep_goal = ag.departure_point(pre['goal_state']['waypoint'], CENTER, r_inf, 1)
    assert any(math.dist(nxt.waypoint, dep_goal) < 1.0 for nxt, _ in succ)


def test_non_riding_state_has_no_arc_hops():
    pre = prep.prepare_scenario(synthetic_circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    st = astar.State((50000.0, 50000.0), 0.0)  # far from any boundary
    assert planner._arc_hop_successors(st) == []


def test_synthetic_circle_end_to_end_valid():
    scn = synthetic_circle_scenario()
    pre = prep.prepare_scenario(scn)
    result = astar.plan_trajectory(pre)
    assert result['success']
    full = tr.build_full_path(result['path'], pre)
    assert pv.path_is_valid(
        full, pre['circle_obstacles'], pre['polygon_obstacles'],
        config.R, config.ALPHA_MAX_RAD, config.L0, config.DSS,
        raw_circle_obstacles=[(CENTER, RAW_R)], raw_polygon_obstacles=[])
    # Straight line O->T is 400 km; the detour around one circle is small.
    dist = sum(math.dist(full[i][0], full[i + 1][0]) for i in range(len(full) - 1))
    assert dist < 430000.0
    # raw_route captured for the discretisation-invariance test (Task 7)
    assert result['planner'].raw_route is not None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/kinodynamic_arc_hop_test.py -v`
Expected: FAIL with `AttributeError: 'KinodynamicAstar' object has no attribute '_arc_hop_successors'`

- [ ] **Step 3: Add config constants**

In `config.py`, insert directly **after** the `WRAP_STEP_M = 10000.0  # 2000` line:

```python
# Angular step (deg) for expanding a circle-boundary arc into waypoint
# vertices (circumscribed polygon) at OUTPUT time. Max supported 45. Search
# connectivity does NOT depend on it: arc clearance is checked at the fixed
# 45-deg bulge radius r/cos(pi/8), which covers any expansion step <= 45 deg.
ARC_WAYPOINT_STEP_DEG = 30.0

# Angular step (deg) for sampling arc clearance during search.
ARC_SAMPLE_STEP_DEG = 5.0
```

- [ ] **Step 4: Planner changes in `core/kinodynamic_astar.py`**

4a. Add the import after `import core.preprocessing as prep`:

```python
import core.arc_geometry as ag
```

4b. Add a module constant after the imports:

```python
# Fixed clearance bulge for riding arcs: circumscribed-polygon vertices for
# any expansion step <= 45 deg stay within r / cos(pi/8) of the center.
_ARC_CLEAR_BULGE = 1.0 / math.cos(math.pi / 8.0)
```

4c. In `State.__init__`, after `self.h_cost = 0` add:

```python
        self.arc_from = None  # (center, radius, arc_start_pt, s) if reached via arc hop
```

4d. In `KinodynamicAstar.__init__`, after `self.search_failed = False` add:

```python
        # Search route before arc expansion/smoothing (set on success);
        # used to verify discretisation invariance.
        self.raw_route = None
```

4e. **Delete** the whole wrap-step block in `get_next_states` (the comment
paragraph starting `# --- Wrap step: straight continuation off a circle
boundary ---` down to and including the `successors.append((State(nx, h),
config.WRAP_STEP_M))` line and its `if`), and **delete** the
`_on_circle_boundary` method. Replace the wrap block with:

```python
        # --- Arc-hop: ride any circle boundary this state is tangent to ---
        successors.extend(self._arc_hop_successors(current_state))
```

4f. Add the two new methods to `KinodynamicAstar` (place them right after
`get_next_states`):

```python
    def _arc_hop_successors(self, current_state):
        """Successors that ride an inflated circle's boundary.

        For each target (bitangent departure toward another circle, tangent
        from a polygon hull vertex or the goal), hop along the boundary arc to
        the departure point where leaving is tangent-continuous. The emitted
        state is the departure point itself; the straight leg to the target is
        found by Strategy A on the next expansion (zero turn there). Cost is
        the true arc length. Replaces the old WRAP_STEP_M straight step; the
        search graph no longer depends on any wrap discretisation.
        """
        P = current_state.waypoint
        h = current_state.heading
        goal_wp = self.goal_state.waypoint
        successors = []
        for center, radius in self.scenario['circle_obstacles']:
            s = ag.riding_sense(P, h, center, radius)
            if s == 0:
                continue
            phi0 = math.atan2(P[1] - center[1], P[0] - center[0])
            max_wrap = self._max_clear_wrap(center, radius, phi0, s)
            if max_wrap <= 1e-6:
                continue
            deps = []
            for c2, r2 in self.scenario['circle_obstacles']:
                if c2 == center and r2 == radius:
                    continue
                deps.extend(dep for dep, _arr in
                            ag.bitangent_departures(center, radius, c2, r2, s))
            for vertex in self._poly_vertices:
                dep = ag.departure_point(vertex, center, radius, s)
                if dep is not None:
                    deps.append(dep)
            dep = ag.departure_point(goal_wp, center, radius, s)
            if dep is not None:
                deps.append(dep)
            for dep in deps:
                dphi = ag.arc_angle(P, dep, center, s)
                if dphi < 1e-3 or dphi > max_wrap:
                    continue
                nxt = State(dep, ag.tangent_heading(dep, center, s))
                nxt.arc_from = (center, radius, P, s)
                successors.append((nxt, radius * dphi))
        return successors

    def _max_clear_wrap(self, center, radius, phi0, s):
        """Maximal angle (rad) the aircraft can ride this boundary from phi0 in
        direction s before the bulged arc (circumscribed-vertex radius) hits
        another obstacle or leaves the map. One sweep bounds every arc-hop
        candidate on this circle, instead of checking each arc separately.
        Conservative: quantised down to ARC_SAMPLE_STEP_DEG."""
        r_check = radius * _ARC_CLEAR_BULGE
        step = math.radians(config.ARC_SAMPLE_STEP_DEG)
        n = int(round(2.0 * math.pi / step))
        prev = (center[0] + r_check * math.cos(phi0),
                center[1] + r_check * math.sin(phi0))
        for k in range(1, n + 1):
            a = phi0 + s * k * step
            p = (center[0] + r_check * math.cos(a),
                 center[1] + r_check * math.sin(a))
            if (not self._in_bounds(p)
                    or not self._check_collision(prev, p, skip_circle=(center, radius))):
                return (k - 1) * step
            prev = p
        return 2.0 * math.pi
```

4g. Extend `_check_collision` with the skip parameter — replace its signature
and the circle loop:

```python
    def _check_collision(self, p1, p2, skip_circle=None):
        """
        Check if line segment from p1 to p2 collides with any obstacle.
        Returns True if collision-free, False otherwise.
        skip_circle=(center, radius) exempts the circle being ridden by an
        arc-clearance sweep (its own boundary is not an obstacle to itself).
        """

        # Check against circle obstacles. A small grazing tolerance lets tangent /
        # wrap segments ride the inflated boundary (they dip a few metres inside the
        # ~13 km inflation band by discretisation but never approach the raw obstacle).
        for center, radius in self.scenario['circle_obstacles']:
            if skip_circle is not None and center == skip_circle[0] and radius == skip_circle[1]:
                continue
            dist = su.point_to_line_distance(center, p1, p2)
            if dist < radius - config.CIRCLE_GRAZE_TOL_M:
                return False
```

(the polygon half of the method is unchanged)

4h. Replace `_reconstruct_path` entirely:

```python
    def _reconstruct_path(self, state):
        """Reconstruct start->state, expanding arc-hop transitions into
        circumscribed-polygon waypoints (output-time discretisation only;
        the searched route itself is stored in self.raw_route)."""
        states = [state]
        current = state
        while current in self.came_from:
            current = self.came_from[current]
            states.append(current)
        states.reverse()

        self.raw_route = [(st.waypoint, st.heading) for st in states]

        theta_out = math.radians(config.ARC_WAYPOINT_STEP_DEG)
        path = []
        for st in states:
            if st.arc_from is not None and path:
                center, radius, arc_start, s = st.arc_from
                dphi = ag.arc_angle(arc_start, st.waypoint, center, s)
                path.extend(ag.arc_waypoints(center, radius, arc_start, dphi, s, theta_out))
            path.append((st.waypoint, st.heading))
        return path
```

- [ ] **Step 5: Run the new tests**

Run: `python -m pytest tests/kinodynamic_arc_hop_test.py -v`
Expected: 3 passed

- [ ] **Step 6: Run the whole suite**

Run: `python -m pytest -q`
Expected: all pass (19 tests)

- [ ] **Step 7: Commit**

```bash
git add config.py core/kinodynamic_astar.py tests/kinodynamic_arc_hop_test.py
git commit -m "feat: arc-hop successors replace WRAP_STEP_M wrap step"
```

---

### Task 5: Strategy B = fallback + leave-the-boundary fan; re-enable smoothing; drop dead budget config

**Files:**
- Modify: `core/kinodynamic_astar.py`
- Modify: `config.py`
- Modify: `tests/kinodynamic_arc_hop_test.py`

**Interfaces:**
- Consumes: Task 4 planner (`_arc_hop_successors`, `ag.riding_sense`).
- Produces: `get_next_states` returns Strategy A + arc-hop successors, **plus** radial-fan successors whenever the state rides a circle boundary (following the boundary to a tangent departure is not always optimal — the fan lets the search leave the boundary between departure points); the fan alone when nothing else exists. In open water with valid candidates there is NO fan. `plan_trajectory` smooths again. `config.NUM_STRATEGY_B` is gone; `config.WRAP_STEP_M` stays (deprecated — `gui/params.py` still reads it; the GUI update is deferred).

- [ ] **Step 1: Write the failing tests** (append to `tests/kinodynamic_arc_hop_test.py`)

```python
def open_water_scenario():
    return {
        'start': (100000.0, 250000.0), 'start_heading': 0.0,
        'goal': (400000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
    }


def test_no_radial_fan_in_open_water():
    """Not riding any boundary and the goal candidate is valid: the fan must
    NOT fire (it only adds branching noise there)."""
    pre = prep.prepare_scenario(open_water_scenario())
    planner = astar.KinodynamicAstar(pre)
    st = astar.State(pre['start_state']['waypoint'], pre['start_state']['heading'])
    succ = planner.get_next_states(st)
    assert len(succ) == 1
    assert math.dist(succ[0][0].waypoint, pre['goal_state']['waypoint']) < 1.0


def test_fan_added_while_riding_boundary():
    """Riding a circle boundary: fan successors appear IN ADDITION to
    arc-hops, so the search can leave the boundary between tangent
    departure points."""
    pre = prep.prepare_scenario(synthetic_circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    (_, r_inf), = pre['circle_obstacles']
    P = (CENTER[0], CENTER[1] - r_inf)  # due south, heading east => riding CCW
    st = astar.State(P, 0.0)
    succ = planner.get_next_states(st)
    assert any(s_.arc_from is not None for s_, _ in succ)  # arc-hops present
    fan_dist = 2 * config.R * math.tan(config.ALPHA_MAX_RAD / 2) + config.RADIAL_FAN_STEP_M
    assert any(s_.arc_from is None
               and math.isclose(math.dist(s_.waypoint, P), fan_dist, rel_tol=1e-9)
               for s_, _ in succ), "fan successors missing at a riding state"


def test_plan_trajectory_smooths_output():
    """Open water: the smoothed path is the minimal W1->goal route."""
    pre = prep.prepare_scenario(open_water_scenario())
    result = astar.plan_trajectory(pre)
    assert result['success']
    assert len(result['path']) <= 3
```

- [ ] **Step 2: Run tests to verify the new ones fail**

Run: `python -m pytest tests/kinodynamic_arc_hop_test.py -v`
Expected: `test_no_radial_fan_in_open_water` FAILS (the current inverted gating adds fan states whenever the goal is visible, so `len(succ)` is 4). `test_fan_added_while_riding_boundary` PASSES under the current gating (the goal happens to be visible there) — it exists to pin the riding-fan behavior so a naive "return when successors exist" fix cannot silently drop it. The smoothing test may pass already — keep it as a regression guard.

- [ ] **Step 3: Rework the Strategy B gating**

In `get_next_states`, replace the arc-hop insertion added in Task 4:

```python
        # --- Arc-hop: ride any circle boundary this state is tangent to ---
        successors.extend(self._arc_hop_successors(current_state))
```

with:

```python
        # --- Arc-hop: ride any circle boundary this state is tangent to ---
        successors.extend(self._arc_hop_successors(current_state))
        riding = any(ag.riding_sense(P, h, center, radius) != 0
                     for center, radius in self.scenario['circle_obstacles'])
```

and replace this block:

```python
        # --- Strategy B: radial fan fallback (no graph candidate was valid) ---
        strategy_b = False
        if not successors or self._check_collision(P, goal_wp):
            strategy_b = True
        elif not self._check_collision(P, goal_wp) and self.num_strategy_b > 0:
            self.num_strategy_b -= 1
            strategy_b = True

        if not strategy_b:
            return successors
```

with:

```python
        if successors and not riding:
            return successors

        # --- Strategy B: radial fan — pure fallback when no candidate is
        # valid, PLUS extra leave-the-boundary options while riding a circle:
        # following the boundary to a tangent departure point is not always
        # optimal, so the fan lets the search leave the boundary between
        # departure points. ---
```

In `search()`, delete the line:

```python
        self.num_strategy_b = config.NUM_STRATEGY_B  # Allow a few radial fan expansions even if goal is reachable
```

- [ ] **Step 4: Re-enable smoothing in `plan_trajectory`**

Replace:

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

- [ ] **Step 5: Config cleanup (GUI-safe subset)**

In `config.py`, delete the line:

```python
NUM_STRATEGY_B = 3  # number of radial fan attempts before giving up
```

(keep `RADIAL_FAN_DIRECTIONS` and `RADIAL_FAN_STEP_M`). Replace the
`WRAP_STEP_M` comment paragraph (starting `# Circle-wrap straight step (m).`)
and its assignment with:

```python
# DEPRECATED: the planner no longer reads this (arc-hop successors replaced
# the wrap step). Kept only because gui/params.py still exposes a slider that
# writes it; delete together with the GUI panel update.
WRAP_STEP_M = 10000.0
```

Verify: `grep -rn "NUM_STRATEGY_B\|num_strategy_b" --include="*.py" . | grep -v __pycache__` → no output.
Verify: `grep -rln "WRAP_STEP_M" --include="*.py" . | grep -v __pycache__` → exactly `./config.py` and `./gui/params.py`.

- [ ] **Step 6: Run the whole suite**

Run: `python -m pytest -q`
Expected: 22 passed

- [ ] **Step 7: Commit**

```bash
git add core/kinodynamic_astar.py config.py tests/kinodynamic_arc_hop_test.py
git commit -m "feat: fan augments boundary-riding states; re-enable smoothing"
```

---

### Task 6: Hard-seed integration + discretisation-invariance tests

**Files:**
- Create: `tests/hard_seeds_test.py`

**Interfaces:**
- Consumes: `batch_random_test.generate_random_scenario(seed)`, `KinodynamicAstar.raw_route` (Task 4), full pipeline.
- Produces: regression gates for the previously pathological seeds.

Baselines are the better of the two WRAP_STEP_M runs (planner-path distance
from `results1_fail_v1/2 batch_random_test_summary.json`, which measures
`result['path']` without O/T — compute the test distance the same way).

- [ ] **Step 1: Write the tests**

Create `tests/hard_seeds_test.py`:

```python
"""Integration gates on the seeds that exposed WRAP_STEP_M sensitivity.

Baseline = best planner-path distance of the two old configs
(results1_fail_v1: WRAP=10000, results1_fail_v2: WRAP=5000).
TIME_BUDGET_S is disabled so results are deterministic; wall time is
asserted separately against the 5 s budget.
"""
import math
import time

import pytest

import config
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.path_validation as pv
import render.trajectory as tr
from batch_random_test import generate_random_scenario

# (seed, distance ceiling in m). Ceilings: baseline * 1.05 for the two
# pathological seeds (huge headroom vs their bad runs: 762 km / 815 km),
# baseline * 1.10 for ordinary hard seeds.
CASES = [
    (125, 579700.0 * 1.05),   # scenario 126: v2 detoured to 762 km
    (981, 581200.0 * 1.05),   # scenario 982: v1 self-looped to 815 km
    (319, 681200.0 * 1.10),   # scenario 320
    (674, 531600.0 * 1.10),   # scenario 675
]


def _plan(seed):
    scn = generate_random_scenario(seed=seed)
    pre = prep.prepare_scenario(scn)
    t0 = time.perf_counter()
    result = astar.plan_trajectory(pre)
    elapsed = time.perf_counter() - t0
    return scn, pre, result, elapsed


@pytest.mark.parametrize("seed,max_dist", CASES)
def test_hard_seed_valid_fast_and_near_baseline(seed, max_dist, monkeypatch):
    monkeypatch.setattr(config, 'TIME_BUDGET_S', None)
    scn, pre, result, elapsed = _plan(seed)
    assert result['success'], f"seed {seed} failed to plan"
    assert elapsed < 5.0, f"seed {seed} took {elapsed:.2f}s"

    full = tr.build_full_path(result['path'], pre)
    raw_circles = [(o['center'], o['radius'])
                   for o in scn['obstacles'] if o['type'] == 'circle']
    raw_polys = [o['polygon'] for o in scn['obstacles'] if o['type'] == 'polygon']
    assert pv.path_is_valid(
        full, pre['circle_obstacles'], pre['polygon_obstacles'],
        config.R, config.ALPHA_MAX_RAD, config.L0, config.DSS,
        raw_circle_obstacles=raw_circles, raw_polygon_obstacles=raw_polys), \
        f"seed {seed}: oracle rejected the path"

    # Same basis as the recorded baselines: planner path, no O/T endpoints.
    path = result['path']
    dist = sum(math.dist(path[i][0], path[i + 1][0]) for i in range(len(path) - 1))
    assert dist <= max_dist, f"seed {seed}: {dist / 1000:.1f} km > ceiling"


def test_route_invariant_to_arc_waypoint_step(monkeypatch):
    """THE root-cause test: the searched route must not depend on the output
    discretisation step (it did depend on WRAP_STEP_M before)."""
    monkeypatch.setattr(config, 'TIME_BUDGET_S', None)
    routes, iterations = [], []
    for theta in (20.0, 30.0, 45.0):
        monkeypatch.setattr(config, 'ARC_WAYPOINT_STEP_DEG', theta)
        _scn, _pre, result, _elapsed = _plan(125)
        assert result['success']
        routes.append(result['planner'].raw_route)
        iterations.append(result['stats']['iterations'])
    assert routes[0] == routes[1] == routes[2]
    assert iterations[0] == iterations[1] == iterations[2]
```

- [ ] **Step 2: Run the tests**

Run: `python -m pytest tests/hard_seeds_test.py -v`
Expected: 5 passed (takes a few seconds per seed).

If a distance gate fails: inspect the plotted result before touching the
ceiling — `python -c` with `viz.plot_scenario` on that seed, compare with
`results1_fail_v1/2`. A regression here means arc-hop connectivity is missing
a corridor (check `_max_clear_wrap` conservatism first: try
`ARC_SAMPLE_STEP_DEG = 2.0`). Do not weaken a gate to make a bad path pass.

- [ ] **Step 3: Run the whole suite**

Run: `python -m pytest -q`
Expected: 27 passed

- [ ] **Step 4: Commit**

```bash
git add tests/hard_seeds_test.py
git commit -m "test: hard-seed regression gates + arc-step invariance"
```

---

### Task 7: Docs + 15-seed batch verification

**Files:**
- Modify: `CLAUDE.md`
- Modify: `README.md` (only if it mentions the wrap step — check)
- Batch output: `results_archop/` (gitignored via `results*/`)

- [ ] **Step 1: Update CLAUDE.md**

In the section "How the A\* search actually works", replace the wrap-step
sentence:

```
(1) **wrap-step** — a straight, heading-preserving step off a circle boundary so
the search can keep tangenting around it;
```

with:

```
(1) **arc-hop** — from a state riding an inflated circle's boundary (tangent
arrival), hop along the boundary arc to each tangent-continuous departure
point (bitangents to other circles, tangents from polygon vertices / the
goal) at true arc-length cost; the arc is expanded into circumscribed-polygon
waypoints (`config.ARC_WAYPOINT_STEP_DEG`) only at path reconstruction, so
search connectivity has no wrap discretisation parameter;
```

Also update the `config.py` description line in the architecture diagram if it
lists `WRAP_STEP_M`, and any other WRAP mentions:
`grep -n "WRAP\|wrap" CLAUDE.md README.md`

- [ ] **Step 2: Update README.md the same way** (only the lines the grep from Step 1 shows).

- [ ] **Step 3: Run the 15-hard-seed batch**

```bash
python - <<'EOF'
import matplotlib.pyplot as plt
plt.switch_backend('Agg')
import batch_random_test as brt
from logger_config import setup_logging
brt.logger = setup_logging("BatchRandomTest", log_file="logs/batch_random_test.log")
brt.run_batch_random_tests(output_dir="results_archop")
EOF
```

Expected: 15 scenarios, all SUCCESS, each planning under the 5 s budget.

- [ ] **Step 4: Compare against both old configs**

```bash
python - <<'EOF'
import json
new = json.load(open('results_archop/batch_random_test_summary.json'))
v1 = json.load(open('results1_fail_v1/batch_random_test_summary.json'))
v2 = json.load(open('results1_fail_v2/batch_random_test_summary.json'))
print(f"{'scenario':<24}{'new km':>9}{'v1 km':>9}{'v2 km':>9}{'vs best':>10}")
worse = 0
for n, a, b in zip(new, v1, v2):
    best = min(a['distance_m'], b['distance_m'])
    d = n['distance_m'] - best
    worse += d > best * 0.05
    print(f"{n['scenario_name']:<24}{n['distance_m']/1000:>9.1f}"
          f"{a['distance_m']/1000:>9.1f}{b['distance_m']/1000:>9.1f}{d/1000:>+10.1f}")
print(f"\nseeds >5% worse than best-of-two: {worse}")
EOF
```

Expected: `worse` is 0 (every seed at or near the better of the two old
configs). Eyeball `results_archop/01_scenario_random_scenario_126.png` and
`..._982.png`: no start zigzag, no self-loop.

- [ ] **Step 5: Full 1000-seed regression (spec testing item 4)**

`run_batch_random_tests` currently loops a hardcoded 15-seed list. Run the
full sweep without editing the committed file, and without PNG rendering
(planning only — the 15-seed run above already eyeballs the plots):

```bash
python - <<'EOF'
import math, time, json
import matplotlib.pyplot as plt
plt.switch_backend('Agg')
import core.preprocessing as prep
import core.kinodynamic_astar as astar
from batch_random_test import generate_random_scenario

rows, fails, slow = [], [], []
for seed in range(1000):
    scn = generate_random_scenario(seed=seed)
    pre = prep.prepare_scenario(scn)
    t0 = time.perf_counter()
    result = astar.plan_trajectory(pre)
    dt = time.perf_counter() - t0
    if not result['success']:
        fails.append(seed)
        continue
    if dt > 5.0:
        slow.append((seed, dt))
    p = result['path']
    dist = sum(math.dist(p[i][0], p[i+1][0]) for i in range(len(p)-1))
    rows.append({'seed': seed, 'distance_m': dist, 'time_s': dt})
json.dump({'results': rows, 'fails': fails, 'slow': slow},
          open('results_archop/full_sweep.json', 'w'), indent=2)
print(f"success {len(rows)}/1000, failed seeds: {fails}, over-budget: {slow}")
EOF
```

Expected: success count ≥ the old baseline (compare against the failure count
implied by `results1`; the 18 seeds originally flagged as problematic —
125, 319, 338, 426, 485, 532, 544, 581, 625, 641, 674, 686, 904, 923, 963,
981, 996, 998 — should all succeed), `slow` empty. Report any newly failing
seed to the user with its number so it can be triaged; do not hide it.

- [ ] **Step 6: Report the comparison table + sweep summary to the user, then commit docs**

```bash
git add CLAUDE.md README.md
git commit -m "docs: describe arc-hop successors (WRAP_STEP_M removed)"
```

---

## Verification checklist (after all tasks)

- `python -m pytest -q` → 27 passed
- `grep -rn "NUM_STRATEGY_B\|num_strategy_b\|_on_circle_boundary" --include="*.py" . | grep -v __pycache__` → empty
- `grep -rln "WRAP_STEP_M" --include="*.py" . | grep -v __pycache__` → only `./config.py` (deprecated constant) and `./gui/params.py` (GUI update deferred)
- `python main.py` still runs the 16-scenario harness without errors
- Batch table from Task 7 Step 4 shows no seed >5% worse than best-of-two baselines
