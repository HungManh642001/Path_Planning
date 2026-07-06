# Path Validity: Unified Graze Tolerance + Leg Validation — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every `success=True` plan return an actually-flyable path — unify the circle-graze tolerance between planner and oracle, and validate the fixed O→W₁ / W_{n-1}→T legs plus the final expanded/smoothed body, returning `success=False` with a specific `failure_reason` otherwise.

**Architecture:** Thread a `circle_tol` parameter through the independent oracle (`core/path_validation.py`) so callers validating planner output pass `config.CIRCLE_GRAZE_TOL_M`. Add a `_check_fixed_legs` planner method reusing `_check_collision`, then a final self-validation pass in `plan_trajectory` that checks both fixed legs and every final-path segment, setting `success`/`failure_reason`. Set the shared tolerance constant to a measured value before enabling the body check.

**Tech Stack:** Python 3, math/shapely, pytest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-07-05-path-validity-tolerance-and-legs-design.md`

## Global Constraints

- Units meters/radians; a planner path is a list of `(waypoint, heading)` tuples, `waypoint = (x, y)`.
- Committed test files are named `tests/*_test.py` (never `test_*.py`). Run from repo root: `python -m pytest -q` (`pytest.ini` sets `pythonpath = .`).
- The oracle `core/path_validation.py` stays *logically independent*: it may receive a tolerance number as a parameter but must not import `config` or planner code. Its default `circle_tol` stays `1e-6` (strict).
- Polygon-interior penetration is always a collision, tolerance-free (DE-9IM `relate_pattern('T********')`). `circle_tol` applies to circles only.
- `config.CIRCLE_GRAZE_TOL_M` is the single shared tolerance; it must be measured and raised (Task 3) **before** the body self-check is enabled (Task 4), else legitimate arc-expansion grazes flip to failure en masse.
- `failure_reason` values exactly: `None` (success), `'no_path'`, `'start_leg_blocked'`, `'goal_leg_blocked'`, `'path_self_collision'`.
- The preprocessed dict already carries `start_pos`, `goal_pos` (raw O and T); the planner stores it as `self.scenario`.

---

### Task 1: Thread `circle_tol` through the oracle

**Files:**
- Modify: `core/path_validation.py` (`_segment_clear`, `segments_clear`, `path_is_valid`)
- Create: `tests/path_validation_test.py`

**Interfaces:**
- Produces (used by Tasks 4, 5): `path_is_valid(path, circle_obstacles, polygon_obstacles, R, alpha_max_rad, L0, dss, raw_circle_obstacles=None, raw_polygon_obstacles=None, circle_tol=1e-6)`; `segments_clear(path, circle_obstacles, polygon_obstacles, circle_tol=1e-6)`; `_segment_clear(a, b, circle_obstacles, polygon_obstacles, circle_tol=1e-6)`.

- [ ] **Step 1: Write the failing tests**

Create `tests/path_validation_test.py`:

```python
"""Oracle tolerance behavior — circle_tol forgives sub-tolerance circle grazes
but never a polygon interior hit; default stays strict."""
import math
import core.path_validation as pv

# A single inflated circle at the origin, radius 1000; a horizontal segment
# whose closest approach to the center is (radius - depth).
C = (0.0, 0.0)
R = 1000.0


def _seg_at_depth(depth):
    y = R - depth              # closest-approach distance from center
    return (-5000.0, y), (5000.0, y)


def test_default_tol_is_strict():
    a, b = _seg_at_depth(0.5)   # 0.5 m inside the inflated boundary
    assert pv._segment_clear(a, b, [(C, R)], []) is False


def test_circle_tol_forgives_subtolerance_graze():
    a, b = _seg_at_depth(0.5)
    assert pv._segment_clear(a, b, [(C, R)], [], circle_tol=1.0) is True


def test_circle_tol_still_rejects_beyond_tolerance():
    a, b = _seg_at_depth(2.0)   # 2 m inside, tol only 1 m
    assert pv._segment_clear(a, b, [(C, R)], [], circle_tol=1.0) is False


def test_clear_segment_passes():
    a, b = _seg_at_depth(-50.0)  # 50 m OUTSIDE the boundary
    assert pv._segment_clear(a, b, [(C, R)], [], circle_tol=1.0) is True


def test_polygon_interior_fails_regardless_of_circle_tol():
    # Segment straight through a square's interior; circle_tol must not forgive it.
    square = [(-100.0, -100.0), (100.0, -100.0), (100.0, 100.0), (-100.0, 100.0)]
    a, b = (-500.0, 0.0), (500.0, 0.0)
    assert pv._segment_clear(a, b, [], [square], circle_tol=1000.0) is False


def test_path_is_valid_threads_circle_tol():
    # Two-waypoint path grazing the circle 0.5 m; strict default rejects,
    # circle_tol=1.0 accepts (turn/segment-length checks are trivially ok here).
    a, b = _seg_at_depth(0.5)
    path = [(a, 0.0), (b, 0.0)]
    common = dict(circle_obstacles=[(C, R)], polygon_obstacles=[],
                  R=8000.0, alpha_max_rad=math.radians(90), L0=4000.0, dss=23000.0,
                  raw_circle_obstacles=[(C, 1.0)], raw_polygon_obstacles=[])
    assert pv.path_is_valid(path, **common) is False
    assert pv.path_is_valid(path, **common, circle_tol=1.0) is True
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/path_validation_test.py -v`
Expected: FAIL — `_segment_clear`/`path_is_valid` do not accept `circle_tol` (TypeError), and the strict-default cases error too.

- [ ] **Step 3: Thread the parameter**

In `core/path_validation.py`, change `_segment_clear`'s signature and circle check:

```python
def _segment_clear(a, b, circle_obstacles, polygon_obstacles, circle_tol=1e-6):
    for center, radius in circle_obstacles:
        # Leniency: a segment grazing the inflated boundary to within
        # `circle_tol` meters is accepted (numerical/discretisation noise
        # within the ~13 km inflation band; the raw obstacle is far inside).
        # tol is subtracted, so only genuine penetration fails.
        if _point_to_segment_distance(center, a, b) < radius - circle_tol:
            return False
```

(polygon half of the function unchanged.)

Change `segments_clear` to thread it:

```python
def segments_clear(path, circle_obstacles, polygon_obstacles, circle_tol=1e-6):
    """True iff every straight segment between consecutive waypoints is clear."""
    for i in range(len(path) - 1):
        a = path[i][0]
        b = path[i + 1][0]
        if not _segment_clear(a, b, circle_obstacles, polygon_obstacles, circle_tol):
            return False
    return True
```

In `path_is_valid`, add `circle_tol=1e-6` as the last parameter and pass it to the `segments_clear` call only (arcs clear against RAW obstacles and keep their own logic):

```python
def path_is_valid(path, circle_obstacles, polygon_obstacles, R, alpha_max_rad, L0, dss,
                  raw_circle_obstacles=None, raw_polygon_obstacles=None, circle_tol=1e-6):
```

and the body's first check:

```python
    if not segments_clear(path, circle_obstacles, polygon_obstacles, circle_tol):
        return False
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/path_validation_test.py -v`
Expected: 6 passed.

- [ ] **Step 5: Run the whole suite (no regressions)**

Run: `python -m pytest -q`
Expected: all pass (existing hard-seed/arc tests call `path_is_valid` without `circle_tol`, so they keep the strict `1e-6` default — unchanged behavior).

- [ ] **Step 6: Commit**

```bash
git add core/path_validation.py tests/path_validation_test.py
git commit -m "feat: circle_tol parameter on the path-validity oracle"
```

---

### Task 2: `_check_fixed_legs` planner method

**Files:**
- Modify: `core/kinodynamic_astar.py` (add method to `KinodynamicAstar`)
- Modify: `tests/kinodynamic_arc_hop_test.py` (append a unit test)

**Interfaces:**
- Consumes: existing `self._check_collision(p1, p2)`, `self.scenario['start_pos']`, `self.scenario['goal_pos']`.
- Produces (used by Task 4): `KinodynamicAstar._check_fixed_legs(path) -> (ok: bool, reason: str | None)` where reason ∈ {`'start_leg_blocked'`, `'goal_leg_blocked'`, None}.

- [ ] **Step 1: Write the failing test** (append to `tests/kinodynamic_arc_hop_test.py`)

```python
def test_check_fixed_legs_detects_blocked_start_and_goal():
    """A circle straddling a fixed leg makes that leg's check fail with the
    matching reason; clear legs pass."""
    # Start O at (0,0), goal T at (400k,0); W1..W_{n-1} body sits mid-map.
    scn = {
        'start': (0.0, 0.0), 'start_heading': 0.0,
        'goal': (400000.0, 0.0), 'goal_heading': 0.0,
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
    }
    pre = prep.prepare_scenario(scn)
    planner = astar.KinodynamicAstar(pre)
    body = [((100000.0, 0.0), 0.0), ((300000.0, 0.0), 0.0)]

    # No obstacles -> both legs clear.
    assert planner._check_fixed_legs(body) == (True, None)

    # Put an inflated circle on the O->W1 leg (near O, off the body).
    Ocirc = (pre['start_pos'][0] + 50000.0, 0.0)
    planner.scenario['circle_obstacles'] = [(Ocirc, 20000.0)]
    ok, reason = planner._check_fixed_legs(body)
    assert ok is False and reason == 'start_leg_blocked'

    # Only a circle on the W_{n-1}->T leg (near T).
    Tcirc = (pre['goal_pos'][0] - 50000.0, 0.0)
    planner.scenario['circle_obstacles'] = [(Tcirc, 20000.0)]
    ok, reason = planner._check_fixed_legs(body)
    assert ok is False and reason == 'goal_leg_blocked'
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/kinodynamic_arc_hop_test.py::test_check_fixed_legs_detects_blocked_start_and_goal -v`
Expected: FAIL — `AttributeError: 'KinodynamicAstar' object has no attribute '_check_fixed_legs'`.

- [ ] **Step 3: Add the method**

In `core/kinodynamic_astar.py`, add to `KinodynamicAstar` (place right after `_check_collision`):

```python
    def _check_fixed_legs(self, path):
        """Validate the fixed takeoff/approach legs O->W1 and W_{n-1}->T.

        These legs are flown but lie outside the A* search (which runs
        W1..W_{n-1}); nothing else collision-checks them. They are determined
        by the mission spec (start/goal points, headings, L0/DSS) and cannot be
        rerouted, so a blocked leg means the mission is infeasible as posed.
        Returns (ok, reason) with reason in {'start_leg_blocked',
        'goal_leg_blocked', None}. Uses the same _check_collision (and thus the
        same CIRCLE_GRAZE_TOL_M / polygon-interior semantics) as the body.
        """
        if not path:
            return True, None
        O = self.scenario['start_pos']
        T = self.scenario['goal_pos']
        if not self._check_collision(O, path[0][0]):
            return False, 'start_leg_blocked'
        if not self._check_collision(path[-1][0], T):
            return False, 'goal_leg_blocked'
        return True, None
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/kinodynamic_arc_hop_test.py::test_check_fixed_legs_detects_blocked_start_and_goal -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add core/kinodynamic_astar.py tests/kinodynamic_arc_hop_test.py
git commit -m "feat: _check_fixed_legs validates the O->W1 and W_{n-1}->T legs"
```

---

### Task 3: Measure and set `CIRCLE_GRAZE_TOL_M`

**Files:**
- Modify: `config.py` (line 60, the `CIRCLE_GRAZE_TOL_M` value + comment)

**Interfaces:**
- Consumes: Tasks 1-2 (not strictly, but must land before Task 4).
- Produces: a `config.CIRCLE_GRAZE_TOL_M` large enough to cover the true numerical body graze of legitimate (raw-safe) paths, so Task 4's body self-check does not reject valid arc paths.

This task has no unit test — it calibrates a constant from measured data. It MUST land before Task 4.

- [ ] **Step 1: Measure the numerical body graze on raw-safe plans**

Run this measurement script (it does not modify anything):

```bash
python - <<'EOF'
import math, warnings; warnings.filterwarnings("ignore")
import matplotlib; matplotlib.use("Agg")
import config, core.preprocessing as prep, core.kinodynamic_astar as astar
import core.spatial_utils as su, render.trajectory as tr
from batch_random_test import generate_random_scenario

grazes = []
for seed in range(150):
    scn = generate_random_scenario(seed=seed); pre = prep.prepare_scenario(scn)
    r = astar.plan_trajectory(pre)
    if not r.get('success') or not r.get('path'):
        continue
    full = tr.build_full_path(r['path'], pre)
    inflated = pre['circle_obstacles']
    rawc = [(o['center'], o['radius']) for o in scn['obstacles'] if o['type'] == 'circle']
    # BODY segments only (exclude the two fixed legs); raw-safe paths only.
    n = len(full)
    raw_hit = any(
        rr - su.point_to_line_distance(cc, full[i][0], full[i+1][0]) > 0.001
        for i in range(n-1) for cc, rr in rawc)
    if raw_hit:
        continue
    for i in range(1, n-2):  # interior/body segments, excluding legs at 0 and n-2
        a, b = full[i][0], full[i+1][0]
        for c, rad in inflated:
            grazes.append(rad - su.point_to_line_distance(c, a, b))
grazes = [g for g in grazes if g > 0]
grazes.sort()
if grazes:
    print(f"body grazes on raw-safe paths: n={len(grazes)} "
          f"max={grazes[-1]:.2f} p99={grazes[int(len(grazes)*0.99)]:.2f} "
          f"median={grazes[len(grazes)//2]:.2f} m")
    import math as m
    rec = m.ceil(grazes[-1]) + 2.0
    print(f"RECOMMENDED CIRCLE_GRAZE_TOL_M = {rec}")
else:
    print("no positive body grazes found; keep CIRCLE_GRAZE_TOL_M small (e.g. 1.0)")
EOF
```

Record the printed `max` and `RECOMMENDED` value. The recommended value is `ceil(max) + 2.0` (headroom). Sanity gate: it must be `< 100.0` (well below `SAFE_MARGIN = 10000`); if it prints ≥ 100, STOP and report — that would indicate a real path-quality problem, not numerical noise, and the plan's assumption is wrong.

- [ ] **Step 2: Set the constant**

In `config.py`, replace the `CIRCLE_GRAZE_TOL_M` block (currently `CIRCLE_GRAZE_TOL_M = 1.0` at line 60, with its comment above) with the measured value and this comment (substitute `<REC>` with the recommended number from Step 1, and `<MAX>` with the observed max):

```python
# Tolerance (m) by which a straight segment may graze inside a circle's
# INFLATED boundary. Shared by the planner (_check_collision) and, when
# validating planner output, the oracle (path_is_valid(..., circle_tol=...)).
# Set from measurement: the max body graze of legitimate raw-safe paths was
# ~<MAX> m (arc-expansion chords / tangent segments dipping into the inflation
# band); this value adds headroom. It is a tiny fraction of the ~13.3 km
# inflation band and never approaches the raw obstacle. Polygon interior is
# checked tolerance-free.
CIRCLE_GRAZE_TOL_M = <REC>
```

- [ ] **Step 3: Verify the suite still passes**

Run: `python -m pytest -q`
Expected: all pass (hard-seed gates validate arcs against RAW obstacles and use the strict oracle default, so a larger `CIRCLE_GRAZE_TOL_M` does not affect them; the planner accepting slightly larger grazes does not change those seeds' routes materially — if any hard-seed distance gate fails, report it rather than adjusting the ceiling).

- [ ] **Step 4: Commit**

```bash
git add config.py
git commit -m "perf: set CIRCLE_GRAZE_TOL_M from measured numerical body graze"
```

---

### Task 4: Self-validation in `plan_trajectory` + `failure_reason`

**Files:**
- Modify: `core/kinodynamic_astar.py` (`plan_trajectory`, lines 563-572)
- Modify: `tests/kinodynamic_arc_hop_test.py` (append integration tests)

**Interfaces:**
- Consumes: `_check_fixed_legs` (Task 2), `_check_collision` (existing), `config.CIRCLE_GRAZE_TOL_M` (Task 3).
- Produces: `plan_trajectory(...)` result dict gains `'failure_reason'`; `'success'` is now "path found AND legs+body flyable".

- [ ] **Step 1: Write the failing integration tests** (append to `tests/kinodynamic_arc_hop_test.py`)

```python
def test_plan_maps_blocked_leg_to_failure_reason():
    """plan_trajectory must translate a blocked-leg verdict from
    _check_fixed_legs into success=False + the specific reason. Monkeypatched
    so the wiring is tested deterministically (real leg geometry is covered by
    test_check_fixed_legs_detects_blocked_start_and_goal and the Task-5 sweep,
    where the ~13 km inflation makes a hand-built blocking scenario fragile)."""
    scn = {
        'start': (100000.0, 250000.0), 'start_heading': 0.0,
        'goal': (400000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
    }
    pre = prep.prepare_scenario(scn)
    import core.kinodynamic_astar as k
    orig = k.KinodynamicAstar._check_fixed_legs
    k.KinodynamicAstar._check_fixed_legs = lambda self, path: (False, 'goal_leg_blocked')
    try:
        result = astar.plan_trajectory(pre)
    finally:
        k.KinodynamicAstar._check_fixed_legs = orig
    assert result['success'] is False
    assert result['failure_reason'] == 'goal_leg_blocked'


def test_plan_succeeds_open_water_reason_none():
    scn = {
        'start': (100000.0, 250000.0), 'start_heading': 0.0,
        'goal': (400000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
    }
    result = astar.plan_trajectory(prep.prepare_scenario(scn))
    assert result['success'] is True
    assert result['failure_reason'] is None


def test_plan_no_path_reason():
    """When search finds nothing, failure_reason is 'no_path'."""
    # Goal boxed so tightly the planner cannot reach an aligned arrival is hard
    # to guarantee; instead assert the key exists and is 'no_path' when path is None
    # by monkeypatching search to return None.
    scn = {
        'start': (100000.0, 250000.0), 'start_heading': 0.0,
        'goal': (400000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
    }
    pre = prep.prepare_scenario(scn)
    import core.kinodynamic_astar as k
    orig = k.KinodynamicAstar.search
    k.KinodynamicAstar.search = lambda self: None
    try:
        result = astar.plan_trajectory(pre)
    finally:
        k.KinodynamicAstar.search = orig
    assert result['success'] is False
    assert result['failure_reason'] == 'no_path'
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/kinodynamic_arc_hop_test.py -k "plan_maps_blocked_leg or plan_succeeds_open_water or plan_no_path" -v`
Expected: FAIL — `result` has no `'failure_reason'` key (KeyError/assertion).

- [ ] **Step 3: Rewrite the `plan_trajectory` tail**

In `core/kinodynamic_astar.py`, replace the smoothing + return block (currently lines 563-572):

```python
    # Smooth path if found
    if path:
        path = planner.smooth_path(path)

    return {
        'path': path,
        'success': path is not None,
        'stats': planner.get_search_stats(),
        'planner': planner,
    }
```

with:

```python
    # Smooth path if found
    if path:
        path = planner.smooth_path(path)

    # Final self-validation: a plan is only a success if the returned path is
    # actually flyable. Search checks segments as it goes, but arc expansion,
    # smoothing, and the fixed O->W1 / W_{n-1}->T legs (added outside the
    # search) can carry collisions that were never verified in final form.
    if not path:
        success, failure_reason = False, 'no_path'
    else:
        legs_ok, reason = planner._check_fixed_legs(path)
        body_ok = all(planner._check_collision(path[i][0], path[i + 1][0])
                      for i in range(len(path) - 1))
        if legs_ok and body_ok:
            success, failure_reason = True, None
        else:
            success, failure_reason = False, (reason or 'path_self_collision')

    if verbose and failure_reason:
        print(f"Plan rejected: {failure_reason}")

    return {
        'path': path,
        'success': success,
        'failure_reason': failure_reason,
        'stats': planner.get_search_stats(),
        'planner': planner,
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/kinodynamic_arc_hop_test.py -k "plan_maps_blocked_leg or plan_succeeds_open_water or plan_no_path" -v`
Expected: 3 passed.

- [ ] **Step 5: Run the whole suite**

Run: `python -m pytest -q`
Expected: all pass. In particular the 4 hard-seed gates must still succeed — they were oracle-valid before, so their legs+body are clear and `success` stays True. If a hard seed now reports `success=False`, its fixed leg is genuinely blocked; report it (do not weaken the check) — it means that seed was never a valid success.

- [ ] **Step 6: Commit**

```bash
git add core/kinodynamic_astar.py tests/kinodynamic_arc_hop_test.py
git commit -m "feat: plan_trajectory self-validates legs+body; adds failure_reason"
```

---

### Task 5: GUI call-site + oracle-validity regression + sweep

**Files:**
- Modify: `gui/summary.py` (line 42-45 `path_is_valid` call)
- Create: `tests/oracle_validity_test.py`

**Interfaces:**
- Consumes: `path_is_valid(..., circle_tol=...)` (Task 1), the self-validating `plan_trajectory` (Task 4), `config.CIRCLE_GRAZE_TOL_M` (Task 3).

- [ ] **Step 1: Update the GUI validity call**

In `gui/summary.py`, the `pv.path_is_valid(...)` call (lines 42-45) validates the planner body `path`; make it validate the full O..T path with the shared tolerance so the GUI "valid" flag agrees with the new invariant. Replace:

```python
    valid = pv.path_is_valid(
        path, preprocessed['circle_obstacles'], preprocessed['polygon_obstacles'],
        R, preprocessed['alpha_max_rad'], config.L0, config.DSS,
        raw_circle_obstacles=raw_circles, raw_polygon_obstacles=raw_polys)
```

with:

```python
    valid = pv.path_is_valid(
        full, preprocessed['circle_obstacles'], preprocessed['polygon_obstacles'],
        R, preprocessed['alpha_max_rad'], config.L0, config.DSS,
        raw_circle_obstacles=raw_circles, raw_polygon_obstacles=raw_polys,
        circle_tol=config.CIRCLE_GRAZE_TOL_M)
```

(`full` is already computed above at line 31.)

- [ ] **Step 2: Write the oracle-validity invariant test**

Create `tests/oracle_validity_test.py`:

```python
"""The invariant the old suite lacked: every plan the planner reports as a
success must be accepted by the independent oracle (with the shared circle
tolerance) over the FULL O..T path, including the fixed legs."""
import math
import pytest

import config
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.path_validation as pv
import render.trajectory as tr
from batch_random_test import generate_random_scenario

SEEDS = list(range(60))  # fast subset; the full 1000-seed sweep is Step 4


@pytest.mark.parametrize("seed", SEEDS)
def test_successful_plan_is_oracle_valid(seed):
    scn = generate_random_scenario(seed=seed)
    pre = prep.prepare_scenario(scn)
    result = astar.plan_trajectory(pre)
    if not result['success']:
        # A reported failure carries a reason and is not asserted for validity.
        assert result['failure_reason'] in (
            'no_path', 'start_leg_blocked', 'goal_leg_blocked', 'path_self_collision')
        return
    full = tr.build_full_path(result['path'], pre)
    rawc = [(o['center'], o['radius']) for o in scn['obstacles'] if o['type'] == 'circle']
    rawp = [o['polygon'] for o in scn['obstacles'] if o['type'] == 'polygon']
    assert pv.path_is_valid(
        full, pre['circle_obstacles'], pre['polygon_obstacles'],
        config.R, config.ALPHA_MAX_RAD, config.L0, config.DSS,
        raw_circle_obstacles=rawc, raw_polygon_obstacles=rawp,
        circle_tol=config.CIRCLE_GRAZE_TOL_M), \
        f"seed {seed}: reported success but oracle rejected the full path"
```

- [ ] **Step 3: Run the invariant test + full suite**

Run: `python -m pytest tests/oracle_validity_test.py -q`
Expected: 60 passed (every success is oracle-valid; failures carry a valid reason).
Run: `python -m pytest -q`
Expected: all pass.

If any seed asserts "reported success but oracle rejected": that is a real remaining gap between `_check_collision` and the oracle (e.g. a polygon-interior escape the body check missed). Investigate the specific segment before proceeding — do NOT loosen the test. Likely fix: ensure the body loop covers the same segments the oracle does (it should, since both walk the full path). Report findings if it is not a one-line reconciliation.

- [ ] **Step 4: Full 1000-seed sweep — record the new honest baseline**

```bash
python - <<'EOF'
import math, time, json
from collections import Counter
import matplotlib; matplotlib.use("Agg")
import core.preprocessing as prep, core.kinodynamic_astar as astar
from batch_random_test import generate_random_scenario

ok = 0; reasons = Counter(); rows = []
for seed in range(1000):
    pre = prep.prepare_scenario(generate_random_scenario(seed=seed))
    r = astar.plan_trajectory(pre)
    if r['success']:
        ok += 1
    else:
        reasons[r['failure_reason']] += 1
    rows.append({'seed': seed, 'success': r['success'],
                 'reason': r['failure_reason']})
import os; os.makedirs('results_archop', exist_ok=True)
json.dump({'success': ok, 'reasons': dict(reasons), 'rows': rows},
          open('results_archop/full_sweep_validity.json', 'w'), indent=2)
print(f"success {ok}/1000; failure reasons: {dict(reasons)}")
EOF
```

Record the success count and the `failure_reason` histogram. Expected: success rate drops from the prior ~778 (blocked-leg plans now honestly fail); the drop should be concentrated in `start_leg_blocked` / `goal_leg_blocked`, with few/zero `path_self_collision`. A large `path_self_collision` count means the body check and the planner's search collision model disagree in normal operation — report it for triage. `results_archop/` is gitignored.

- [ ] **Step 5: Commit**

```bash
git add gui/summary.py tests/oracle_validity_test.py
git commit -m "test: oracle-validity invariant on planner successes; GUI validates full path"
```

---

## Verification checklist (after all tasks)

- `python -m pytest -q` → all pass, including `tests/path_validation_test.py`, `tests/oracle_validity_test.py`, the new `_check_fixed_legs` and `plan_trajectory` tests.
- `grep -n "circle_tol" core/path_validation.py` shows the parameter threaded through `_segment_clear`, `segments_clear`, `path_is_valid`.
- `config.CIRCLE_GRAZE_TOL_M` is the measured value (< 100), documented.
- 1000-seed sweep `failure_reason` histogram recorded; `path_self_collision` count is small and any nonzero cases triaged.
- `python main.py` still runs the 16-scenario harness without error.
