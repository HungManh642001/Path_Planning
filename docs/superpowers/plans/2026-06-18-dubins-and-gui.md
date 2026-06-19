# Dubins Trajectory Rendering + GUI Redesign — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Render the planner's flight path with true Dubins curves (toggle straight/Dubins) and replace the monolithic GUI with a clean 3-column `gui/` package that configures all parameters, runs the planner, and shows a results summary.

**Architecture:** Phase 1 adds a headless rendering core — a correct Dubins solver (`dubins_curves.py`) and a `trajectory.py` facade (`sample_trajectory(path, R, mode)`) used by both `visualizer.py` and the GUI. Phase 2 builds a `gui/` package (pure helpers `params`/`summary`/`scenario_io` + Tk panels `map_canvas`/`config_panel`/`results_panel` + `app` orchestrator) wired into `launch_gui.py`.

**Tech Stack:** Python 3.11, pytest 7.4, numpy, shapely 2.1, matplotlib, tkinter (Tk backend for the GUI; Agg for headless tests).

## Global Constraints

- Test files MUST be named `*_test.py` (the repo `.gitignore` excludes `test_*.py`).
- Run all tests from the repo root `/mnt/d/Workspace/VTX/VCM_Path_Planning`.
- The Dubins solver is for **rendering only** — it does not feed planning or collision checking.
- True Dubins between consecutive `(pos, heading)` waypoints matches the planner's validated "turn-at-waypoint + straight" model (headings are arrival directions); rendering must never show a gap/jump (always fall back to a continuous polyline).
- `arc_line_trajectory(points, R, arc_samples=20)` in `spatial_utils.py` stays as the Dubins per-segment fallback — do not delete it.
- The planner reads some tunables directly from the `config` module; the GUI applies panel values by setting `config.*` attributes before each run (single-process tool).
- Do not change the search algorithm, obstacle model, or `path_validation.py` semantics.
- Angles in code are radians; `config.ALPHA_MAX`/launch/approach angles are stored in degrees.

---

## File Structure

**New:**
- `trajectory.py` — `sample_trajectory(path, R, mode, step)` facade.
- `gui/__init__.py` — empty package marker.
- `gui/params.py` — `PARAM_SPECS`, `default_values()`, `apply_overrides(values)`.
- `gui/summary.py` — `compute_summary(...)` pure metric computation.
- `gui/scenario_io.py` — `scenario_to_json(state)` / `scenario_from_json(text)`.
- `gui/map_canvas.py` — `MapCanvas` Tk+matplotlib widget.
- `gui/config_panel.py` — `ConfigPanel` Tk widget.
- `gui/results_panel.py` — `ResultsPanel` Tk widget.
- `gui/app.py` — `PlannerApp` orchestrator.
- Tests: `dubins_curves_test.py`, `trajectory_test.py`, `gui/params_test.py`, `gui/summary_test.py`, `gui/scenario_io_test.py`.

**Rewritten:** `dubins_curves.py`.
**Modified:** `visualizer.py` (`trajectory_mode` kwarg), `launch_gui.py` (import `gui.app`).
**Removed:** `gui_scenario_builder.py`.

---

# PHASE 1 — Dubins + trajectory rendering (headless)

## Task 1: Rewrite `dubins_curves.py` as a correct Dubins solver

**Files:**
- Rewrite: `dubins_curves.py`
- Test: `dubins_curves_test.py`

**Interfaces:**
- Produces:
  - `class DubinsPath(start_pos, start_heading, goal_pos, goal_heading, radius)`
  - `DubinsPath.shortest_path() -> dict | None` with keys `{'word','length','t','p','q'}` (`word` in `LSL,RSR,LSR,RSL,RLR,LRL`; `t,p,q` are normalized arc lengths).
  - `DubinsPath.sample_path(step) -> list[(x, y, heading)]` sampled along the real arcs/lines; first sample == start config, last sample == goal config.

- [ ] **Step 1: Write the failing tests**

```python
# dubins_curves_test.py
import math
import dubins_curves as dc


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def test_straight_line_when_aligned():
    # start and goal aligned on +x, both heading +x -> straight, length == distance
    p = dc.DubinsPath((0.0, 0.0), 0.0, (100.0, 0.0), 0.0, radius=10.0)
    sp = p.shortest_path()
    assert sp is not None
    assert abs(sp['length'] - 100.0) < 1e-6


def test_sample_reproduces_start_and_goal():
    # Several configurations: the sampled path must start at start and end at goal,
    # with matching headings -- this validates the whole solve+sample pipeline.
    cases = [
        ((0.0, 0.0), 0.0, (80.0, 40.0), math.pi / 2),
        ((0.0, 0.0), math.pi / 2, (-60.0, 30.0), math.pi),
        ((10.0, -5.0), -math.pi / 4, (50.0, 50.0), 0.0),
        ((0.0, 0.0), 0.0, (5.0, 0.0), math.pi),   # short hop, forces a CCC word
    ]
    for sp_pos, sp_h, gp_pos, gp_h in cases:
        path = dc.DubinsPath(sp_pos, sp_h, gp_pos, gp_h, radius=10.0)
        pts = path.sample_path(step=1.0)
        assert len(pts) >= 2
        assert math.hypot(pts[0][0] - sp_pos[0], pts[0][1] - sp_pos[1]) < 1e-6
        assert abs(_wrap(pts[0][2] - sp_h)) < 1e-6
        assert math.hypot(pts[-1][0] - gp_pos[0], pts[-1][1] - gp_pos[1]) < 1e-3
        assert abs(_wrap(pts[-1][2] - gp_h)) < 1e-3


def test_samples_are_continuous():
    path = dc.DubinsPath((0.0, 0.0), 0.0, (80.0, 40.0), math.pi / 2, radius=10.0)
    pts = path.sample_path(step=1.0)
    gaps = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            for i in range(len(pts) - 1)]
    assert max(gaps) < 2.0, "samples must be densely spaced (no jumps)"


def test_arc_points_lie_on_turning_circle():
    # A pure left turn: every sample must be exactly radius R from the turning centre,
    # proving arcs are sampled geometrically (not linearly interpolated).
    R = 10.0
    path = dc.DubinsPath((0.0, 0.0), 0.0, (0.0, 20.0), math.pi, radius=R)
    pts = path.sample_path(step=0.5)
    # left-turn centre for start (0,0,heading 0) is at (0, R)
    cx, cy = 0.0, R
    on_circle = sum(1 for (x, y, _) in pts if abs(math.hypot(x - cx, y - cy) - R) < 1e-6)
    assert on_circle >= len(pts) // 2, "arc samples must lie on the turning circle"
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest dubins_curves_test.py -v`
Expected: FAIL (current `DubinsPath.shortest_path` returns one of only 4 broken words; `sample_path` does linear interpolation and returns `(x,y,heading)` that does not reproduce start/goal). Most assertions fail.

- [ ] **Step 3: Replace the entire contents of `dubins_curves.py`**

```python
"""
Dubins Path Module
Shortest path of bounded curvature (radius R) between two configurations (x, y, heading).
Rendering only -- not used by planning or collision checking.
Standard normalized formulation (Walker / LaValle), six words: LSL RSR LSR RSL RLR LRL.
"""

import math

_WORDS = ('LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL')


def _mod2pi(theta):
    return theta % (2 * math.pi)


def _word_lengths(word, alpha, beta, d):
    """Normalized segment lengths (t, p, q) for `word`, or None if infeasible."""
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)

    if word == 'LSL':
        p_sq = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
        if p_sq < 0:
            return None
        tmp = math.atan2(cb - ca, d + sa - sb)
        return (_mod2pi(-alpha + tmp), math.sqrt(p_sq), _mod2pi(beta - tmp))

    if word == 'RSR':
        p_sq = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
        if p_sq < 0:
            return None
        tmp = math.atan2(ca - cb, d - sa + sb)
        return (_mod2pi(alpha - tmp), math.sqrt(p_sq), _mod2pi(-beta + tmp))

    if word == 'LSR':
        p_sq = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
        if p_sq < 0:
            return None
        p = math.sqrt(p_sq)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        return (_mod2pi(-alpha + tmp), p, _mod2pi(-_mod2pi(beta) + tmp))

    if word == 'RSL':
        p_sq = -2 + d * d + 2 * cab - 2 * d * (sa + sb)
        if p_sq < 0:
            return None
        p = math.sqrt(p_sq)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        return (_mod2pi(alpha - tmp), p, _mod2pi(beta - tmp))

    if word == 'RLR':
        tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8.0
        if abs(tmp) > 1:
            return None
        p = _mod2pi(2 * math.pi - math.acos(tmp))
        t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        return (t, p, _mod2pi(alpha - beta - t + p))

    if word == 'LRL':
        tmp = (6 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8.0
        if abs(tmp) > 1:
            return None
        p = _mod2pi(2 * math.pi - math.acos(tmp))
        t = _mod2pi(-alpha + math.atan2(-ca + cb, d + sa - sb) + p / 2.0)
        return (t, p, _mod2pi(_mod2pi(beta) - alpha - t + p))

    return None


def _step_config(x, y, theta, seg_len, kind, R):
    """Advance config (x, y, theta) by actual arc length seg_len along segment kind."""
    if kind == 'S':
        return (x + seg_len * math.cos(theta), y + seg_len * math.sin(theta), theta)
    dtheta = seg_len / R
    if kind == 'L':
        cx, cy = x - R * math.sin(theta), y + R * math.cos(theta)
        nt = theta + dtheta
        return (cx + R * math.sin(nt), cy - R * math.cos(nt), nt)
    # 'R'
    cx, cy = x + R * math.sin(theta), y - R * math.cos(theta)
    nt = theta - dtheta
    return (cx - R * math.sin(nt), cy + R * math.cos(nt), nt)


class DubinsPath:
    """Dubins shortest path between two configurations with turn radius `radius`."""

    def __init__(self, start_pos, start_heading, goal_pos, goal_heading, radius):
        self.start_pos = start_pos
        self.start_heading = start_heading
        self.goal_pos = goal_pos
        self.goal_heading = goal_heading
        self.radius = float(radius)

    def _normalized(self):
        dx = self.goal_pos[0] - self.start_pos[0]
        dy = self.goal_pos[1] - self.start_pos[1]
        D = math.hypot(dx, dy)
        d = D / self.radius
        theta = math.atan2(dy, dx) if D > 1e-12 else 0.0
        alpha = _mod2pi(self.start_heading - theta)
        beta = _mod2pi(self.goal_heading - theta)
        return alpha, beta, d

    def shortest_path(self):
        alpha, beta, d = self._normalized()
        best = None
        for word in _WORDS:
            res = _word_lengths(word, alpha, beta, d)
            if res is None:
                continue
            t, p, q = res
            length = (t + p + q) * self.radius
            if best is None or length < best['length']:
                best = {'word': word, 'length': length, 't': t, 'p': p, 'q': q}
        return best

    def sample_path(self, step):
        sp = self.shortest_path()
        if sp is None:
            return []
        kinds = list(sp['word'])
        seg_lengths = [sp['t'] * self.radius, sp['p'] * self.radius, sp['q'] * self.radius]
        x, y, theta = self.start_pos[0], self.start_pos[1], self.start_heading
        pts = [(x, y, theta)]
        for kind, seg_len in zip(kinds, seg_lengths):
            n = max(1, int(math.ceil(seg_len / step)))
            sub = seg_len / n
            for _ in range(n):
                x, y, theta = _step_config(x, y, theta, sub, kind, self.radius)
                pts.append((x, y, theta))
        return pts
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest dubins_curves_test.py -v`
Expected: PASS (all 5 tests).

- [ ] **Step 5: Commit**

```bash
git add dubins_curves.py dubins_curves_test.py
git commit -m "feat: correct Dubins solver (6 words, geometric sampling)"
```

---

## Task 2: `trajectory.py` facade + visualizer integration

**Files:**
- Create: `trajectory.py`
- Test: `trajectory_test.py`
- Modify: `visualizer.py` (trajectory rendering block + `plot_scenario` signature)

**Interfaces:**
- Consumes: `dubins_curves.DubinsPath` (Task 1); `spatial_utils.arc_line_trajectory(points, R, arc_samples=20)` (existing fallback).
- Produces: `trajectory.sample_trajectory(path, R, mode='dubins', step=None) -> list[(x, y)]` where `path` is the planner's `[(waypoint, heading), ...]`.

- [ ] **Step 1: Write the failing tests**

```python
# trajectory_test.py
import math
import trajectory as tr


def test_straight_mode_returns_waypoint_polyline():
    path = [((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0), ((100.0, 100.0), math.pi / 2)]
    pts = tr.sample_trajectory(path, R=8000.0, mode='straight')
    assert pts == [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]


def test_dubins_mode_is_continuous_and_hits_endpoints():
    path = [((0.0, 0.0), 0.0), ((40000.0, 0.0), 0.0), ((40000.0, 40000.0), math.pi / 2)]
    pts = tr.sample_trajectory(path, R=8000.0, mode='dubins')
    assert len(pts) >= 3
    assert math.hypot(pts[0][0] - 0.0, pts[0][1] - 0.0) < 1.0
    assert math.hypot(pts[-1][0] - 40000.0, pts[-1][1] - 40000.0) < 1.0
    gaps = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            for i in range(len(pts) - 1)]
    assert max(gaps) < 8000.0, "dubins trajectory must be continuous (no segment dropped)"


def test_single_waypoint_returns_itself():
    assert tr.sample_trajectory([((1.0, 2.0), 0.0)], R=8000.0, mode='dubins') == [(1.0, 2.0)]
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest trajectory_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'trajectory'`.

- [ ] **Step 3: Create `trajectory.py`**

```python
"""
Trajectory rendering facade.

Turns the planner's [(waypoint, heading), ...] output into a flat list of (x, y)
points for drawing. Two modes:
  - 'straight': the waypoint positions joined by straight lines.
  - 'dubins'  : the real flight path (radius-R Dubins curve between consecutive
                configurations); falls back to arc_line_trajectory then to the
                straight segment if a Dubins solve degenerates, so the output is
                always continuous.
"""

import math

import dubins_curves as dc
import spatial_utils as su


def sample_trajectory(path, R, mode='dubins', step=None):
    if not path:
        return []
    waypoints = [wp for wp, _ in path]
    if len(waypoints) == 1:
        return [waypoints[0]]
    if mode == 'straight':
        return list(waypoints)

    if step is None:
        step = R / 8.0
    pts = [waypoints[0]]
    for i in range(len(path) - 1):
        (p0, h0), (p1, h1) = path[i], path[i + 1]
        seg = _dubins_segment(p0, h0, p1, h1, R, step)
        if seg is None:
            seg = _fallback_segment(p0, p1, R)
        pts.extend(seg[1:])           # drop duplicated join point
    return pts


def _dubins_segment(p0, h0, p1, h1, R, step):
    try:
        dub = dc.DubinsPath(p0, h0, p1, h1, R)
        samples = dub.sample_path(step)
    except (ValueError, ZeroDivisionError):
        return None
    if len(samples) < 2:
        return None
    return [(x, y) for (x, y, _) in samples]


def _fallback_segment(p0, p1, R):
    # arc_line_trajectory needs >=3 points to insert an arc; for a 2-point leg it
    # returns the straight segment, which is exactly the safe fallback here.
    seg = su.arc_line_trajectory([p0, p1], R)
    if len(seg) < 2:
        return [p0, p1]
    return seg
```

- [ ] **Step 4: Run to verify the trajectory tests pass**

Run: `python -m pytest trajectory_test.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Wire `trajectory` into `visualizer.py`**

In `visualizer.py`, change the signature (line 16-17):

```python
def plot_scenario(scenario, preprocessed, result=None, title="Mission Scenario",
                 save_path=None, figsize=(14, 12), trajectory_mode='dubins'):
```

Add the import near the top with the other imports (after `import spatial_utils as su`):

```python
import trajectory as tr
```

Replace the trajectory-sampling call in the `if result and result.get('path'):` block. Find the line:

```python
            samples = su.arc_line_trajectory(waypoints, config.R)
```

and replace it with:

```python
            R = preprocessed.get('turn_radius', config.R)
            samples = [(x, y) for (x, y) in tr.sample_trajectory(path, R, mode=trajectory_mode)]
```

Update the legend label line:

```python
                ax.plot(traj_xs, traj_ys, 'b-', linewidth=3.0, label='Flight Trajectory',
```

to:

```python
                _label = 'Dubins Trajectory' if trajectory_mode == 'dubins' else 'Straight Segments'
                ax.plot(traj_xs, traj_ys, 'b-', linewidth=3.0, label=_label,
```

- [ ] **Step 6: Verify the full suite passes and regenerate one scenario**

Run: `python -m pytest -q`
Expected: all PASS (existing tests unaffected — `arc_line_trajectory` still present as fallback).

Run a render smoke check (no jumps; Dubins used):
```bash
python -c "
import matplotlib; matplotlib.use('Agg')
import map_generator as mg, preprocessing as prep, kinodynamic_astar as astar, visualizer as viz
sc=mg.scenario14_combined_threat(); pre=prep.prepare_scenario(sc); res=astar.plan_trajectory(pre,verbose=False)
viz.plot_scenario(sc, pre, res, title='S14 Dubins', save_path='results/01_scenario_scenario_14_combined_threat.png')
viz.plot_scenario(sc, pre, res, title='S14 Straight', save_path='/tmp/s14_straight.png', trajectory_mode='straight')
print('ok')
"
```
Expected: prints `ok`, no exception. (Dubins is the default; straight mode also renders.)

- [ ] **Step 7: Commit**

```bash
git add trajectory.py trajectory_test.py visualizer.py
git commit -m "feat: trajectory facade (straight/dubins) wired into visualizer"
```

---

# PHASE 2 — GUI redesign (`gui/` package)

## Task 3: `gui/params.py` — parameter specs + config overrides

**Files:**
- Create: `gui/__init__.py` (empty), `gui/params.py`
- Test: `gui/params_test.py`

**Interfaces:**
- Produces:
  - `PARAM_SPECS`: list of dicts `{'key','label','group','min','max','default'}`; `group` in `'tactical','run','advanced'`.
  - `default_values() -> dict[str, float]` mapping each `key` to its default.
  - `apply_overrides(values) -> dict` — writes the relevant `config.*` attributes from `values` and returns the keyword arguments for `preprocessing.prepare_scenario` (`R`, `L0`, `DSS`, `safe_margin`, `alpha_max_rad`).

- [ ] **Step 1: Write the failing tests**

```python
# gui/params_test.py
import math
import config
import gui.params as gp


def test_param_specs_cover_groups():
    groups = {s['group'] for s in gp.PARAM_SPECS}
    assert {'tactical', 'run', 'advanced'} <= groups
    keys = {s['key'] for s in gp.PARAM_SPECS}
    assert {'turn_radius', 'alpha_max_deg', 'safe_margin', 'time_budget_s',
            'turn_penalty_weight', 'wrap_step_m'} <= keys


def test_default_values_match_specs():
    dv = gp.default_values()
    for s in gp.PARAM_SPECS:
        assert dv[s['key']] == s['default']


def test_apply_overrides_sets_config_and_returns_prepare_kwargs():
    vals = gp.default_values()
    vals['turn_radius'] = 9000.0
    vals['alpha_max_deg'] = 60.0
    vals['safe_margin'] = 12000.0
    vals['turn_penalty_weight'] = 1234.0
    vals['wrap_step_m'] = 3000.0
    kwargs = gp.apply_overrides(vals)
    assert kwargs['R'] == 9000.0
    assert abs(kwargs['alpha_max_rad'] - math.radians(60.0)) < 1e-9
    assert kwargs['safe_margin'] == 12000.0
    assert config.TURN_PENALTY_WEIGHT == 1234.0
    assert config.WRAP_STEP_M == 3000.0
    assert config.ALPHA_MAX == 60.0
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest gui/params_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'gui'`.

- [ ] **Step 3: Create the package + `gui/params.py`**

Create empty `gui/__init__.py`. Create `gui/params.py`:

```python
"""Parameter specifications for the planner GUI and config-override application.

The planner reads several tunables directly from the `config` module, so the GUI
applies panel values by writing them onto `config` before each run (single process).
Physical params consumed by prepare_scenario are returned as keyword arguments.
"""

import math

import config

# key, label, group, min, max, default
PARAM_SPECS = [
    # --- tactical (physical) ---
    {'key': 'turn_radius',  'label': 'Turn Radius R (m)',   'group': 'tactical', 'min': 3000.0,  'max': 15000.0, 'default': config.R},
    {'key': 'alpha_max_deg','label': 'Max Turn Angle (deg)','group': 'tactical', 'min': 10.0,    'max': 90.0,    'default': config.ALPHA_MAX},
    {'key': 'l0',           'label': 'L0 stabilize (m)',    'group': 'tactical', 'min': 1000.0,  'max': 20000.0, 'default': config.L0},
    {'key': 'dss',          'label': 'Seeker DSS (m)',      'group': 'tactical', 'min': 5000.0,  'max': 60000.0, 'default': config.DSS},
    {'key': 'safe_margin',  'label': 'Safe Margin (m)',     'group': 'tactical', 'min': 0.0,     'max': 30000.0, 'default': config.SAFE_MARGIN},
    {'key': 'launch_angle', 'label': 'Launch Angle (deg)',  'group': 'tactical', 'min': config.LAUNCH_ANGLE_MIN,   'max': config.LAUNCH_ANGLE_MAX,   'default': config.LAUNCH_ANGLE_DEFAULT},
    {'key': 'approach_angle','label': 'Approach Angle (deg)','group': 'tactical','min': config.APPROACH_ANGLE_MIN, 'max': config.APPROACH_ANGLE_MAX, 'default': config.APPROACH_ANGLE_DEFAULT},
    # --- run ---
    {'key': 'time_budget_s','label': 'Time Budget (s)',     'group': 'run',      'min': 0.1,     'max': 10.0,    'default': config.TIME_BUDGET_S},
    {'key': 'max_iterations','label': 'Max Iterations',     'group': 'run',      'min': 1000.0,  'max': 200000.0,'default': float(config.MAX_ITERATIONS)},
    # --- advanced (search internals) ---
    {'key': 'state_pos_quantum',     'label': 'Pos Quantum (m)',     'group': 'advanced', 'min': 100.0,  'max': 5000.0,  'default': config.STATE_POS_QUANTUM},
    {'key': 'state_heading_quantum', 'label': 'Heading Quantum (deg)','group': 'advanced','min': 0.5,    'max': 15.0,    'default': config.STATE_HEADING_QUANTUM_DEG},
    {'key': 'heuristic_weight',      'label': 'Heuristic Weight',    'group': 'advanced', 'min': 1.0,    'max': 5.0,     'default': config.HEURISTIC_WEIGHT},
    {'key': 'turn_penalty_weight',   'label': 'Turn Penalty (m/rad)','group': 'advanced', 'min': 0.0,    'max': 20000.0, 'default': config.TURN_PENALTY_WEIGHT},
    {'key': 'goal_threshold',        'label': 'Goal Threshold (m)',  'group': 'advanced', 'min': 100.0,  'max': 5000.0,  'default': config.GOAL_THRESHOLD},
    {'key': 'polygon_mitre_limit',   'label': 'Polygon Mitre Limit', 'group': 'advanced', 'min': 1.0,    'max': 5.0,     'default': config.POLYGON_MITRE_LIMIT},
    {'key': 'wrap_step_m',           'label': 'Wrap Step (m)',       'group': 'advanced', 'min': 500.0,  'max': 20000.0, 'default': config.WRAP_STEP_M},
    {'key': 'circle_graze_tol_m',    'label': 'Circle Graze Tol (m)','group': 'advanced', 'min': 0.0,    'max': 500.0,   'default': config.CIRCLE_GRAZE_TOL_M},
    {'key': 'obstacle_ring_samples', 'label': 'Ring Samples',        'group': 'advanced', 'min': 6.0,    'max': 64.0,    'default': float(config.OBSTACLE_RING_SAMPLES)},
]


def default_values():
    return {s['key']: s['default'] for s in PARAM_SPECS}


def apply_overrides(values):
    """Write GUI values onto config.* and return prepare_scenario kwargs."""
    config.TIME_BUDGET_S = float(values['time_budget_s'])
    config.MAX_ITERATIONS = int(values['max_iterations'])
    config.STATE_POS_QUANTUM = float(values['state_pos_quantum'])
    config.STATE_HEADING_QUANTUM_DEG = float(values['state_heading_quantum'])
    config.HEURISTIC_WEIGHT = float(values['heuristic_weight'])
    config.TURN_PENALTY_WEIGHT = float(values['turn_penalty_weight'])
    config.GOAL_THRESHOLD = float(values['goal_threshold'])
    config.POLYGON_MITRE_LIMIT = float(values['polygon_mitre_limit'])
    config.WRAP_STEP_M = float(values['wrap_step_m'])
    config.CIRCLE_GRAZE_TOL_M = float(values['circle_graze_tol_m'])
    config.OBSTACLE_RING_SAMPLES = int(values['obstacle_ring_samples'])
    # ALPHA_MAX is stored in degrees in config; keep it consistent.
    config.ALPHA_MAX = float(values['alpha_max_deg'])
    return {
        'R': float(values['turn_radius']),
        'L0': float(values['l0']),
        'DSS': float(values['dss']),
        'safe_margin': float(values['safe_margin']),
        'alpha_max_rad': math.radians(float(values['alpha_max_deg'])),
    }
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest gui/params_test.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add gui/__init__.py gui/params.py gui/params_test.py
git commit -m "feat(gui): parameter specs and config-override application"
```

---

## Task 4: `gui/summary.py` — results metric computation

**Files:**
- Create: `gui/summary.py`
- Test: `gui/summary_test.py`

**Interfaces:**
- Consumes: `trajectory.sample_trajectory` (Task 2); `path_validation.path_is_valid`; `spatial_utils` (none directly).
- Produces: `compute_summary(result, preprocessed, raw_circles, raw_polys, render_mode, runtime_s) -> dict` with keys `success, distance_km, num_waypoints, num_turns, max_turn_deg, runtime_ms, iterations, valid`.

- [ ] **Step 1: Write the failing tests**

```python
# gui/summary_test.py
import math
import gui.summary as gs


def _fake_preprocessed():
    return {'turn_radius': 8000.0, 'alpha_max_rad': math.radians(90.0),
            'circle_obstacles': [], 'polygon_obstacles': []}


def test_summary_of_straight_two_point_path():
    path = [((0.0, 0.0), 0.0), ((100000.0, 0.0), 0.0)]
    result = {'success': True, 'path': path,
              'stats': {'iterations': 7}}
    s = gs.compute_summary(result, _fake_preprocessed(), raw_circles=[], raw_polys=[],
                           render_mode='straight', runtime_s=0.012)
    assert s['success'] is True
    assert abs(s['distance_km'] - 100.0) < 1e-6
    assert s['num_waypoints'] == 2
    assert s['num_turns'] == 0
    assert s['runtime_ms'] == 12.0
    assert s['iterations'] == 7


def test_summary_counts_turn_and_reports_max_angle():
    path = [((0.0, 0.0), 0.0), ((100000.0, 0.0), 0.0), ((100000.0, 100000.0), math.pi / 2)]
    result = {'success': True, 'path': path, 'stats': {'iterations': 3}}
    s = gs.compute_summary(result, _fake_preprocessed(), raw_circles=[], raw_polys=[],
                           render_mode='straight', runtime_s=0.0)
    assert s['num_turns'] == 1
    assert abs(s['max_turn_deg'] - 90.0) < 1e-6


def test_summary_handles_failure():
    result = {'success': False, 'path': None, 'stats': {'iterations': 50000}}
    s = gs.compute_summary(result, _fake_preprocessed(), raw_circles=[], raw_polys=[],
                           render_mode='dubins', runtime_s=0.9)
    assert s['success'] is False
    assert s['distance_km'] == 0.0
    assert s['num_waypoints'] == 0
    assert s['valid'] is False
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest gui/summary_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'gui.summary'`.

- [ ] **Step 3: Create `gui/summary.py`**

```python
"""Pure computation of the GUI results summary (no Tk)."""

import math

import config
import trajectory as tr
import path_validation as pv


def _angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def compute_summary(result, preprocessed, raw_circles, raw_polys, render_mode, runtime_s):
    R = preprocessed.get('turn_radius', config.R)
    base = {
        'success': bool(result.get('success')),
        'distance_km': 0.0,
        'num_waypoints': 0,
        'num_turns': 0,
        'max_turn_deg': 0.0,
        'runtime_ms': runtime_s * 1000.0,
        'iterations': result.get('stats', {}).get('iterations', 0),
        'valid': False,
    }
    path = result.get('path')
    if not (result.get('success') and path):
        return base

    pts = tr.sample_trajectory(path, R, mode=render_mode)
    dist = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))

    turns = []
    for i in range(1, len(path) - 1):
        h_in = math.atan2(path[i][0][1] - path[i - 1][0][1], path[i][0][0] - path[i - 1][0][0])
        h_out = math.atan2(path[i + 1][0][1] - path[i][0][1], path[i + 1][0][0] - path[i][0][0])
        turns.append(abs(_angle_diff(h_out, h_in)))

    valid = pv.path_is_valid(
        path, preprocessed['circle_obstacles'], preprocessed['polygon_obstacles'],
        R, preprocessed['alpha_max_rad'], config.L0, config.DSS,
        raw_circle_obstacles=raw_circles, raw_polygon_obstacles=raw_polys)

    base.update({
        'distance_km': dist / 1000.0,
        'num_waypoints': len(path),
        'num_turns': sum(1 for t in turns if t > math.radians(1.0)),
        'max_turn_deg': math.degrees(max(turns)) if turns else 0.0,
        'valid': bool(valid),
    })
    return base
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest gui/summary_test.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add gui/summary.py gui/summary_test.py
git commit -m "feat(gui): pure results-summary computation"
```

---

## Task 5: `gui/scenario_io.py` — JSON save/load

**Files:**
- Create: `gui/scenario_io.py`
- Test: `gui/scenario_io_test.py`

**Interfaces:**
- Produces:
  - `scenario_to_json(state) -> str` and `scenario_from_json(text) -> dict`.
  - The GUI **scenario state** dict shape: `{'start': (x,y)|None, 'start_heading': float, 'goal': (x,y)|None, 'goal_heading': float, 'obstacles': [ {'type':'circle','center':(x,y),'radius':float} | {'type':'polygon','polygon':[(x,y),...]} ]}`.

- [ ] **Step 1: Write the failing tests**

```python
# gui/scenario_io_test.py
import gui.scenario_io as sio


def test_roundtrip_preserves_state():
    state = {
        'start': (1000.0, 2000.0), 'start_heading': 0.5,
        'goal': (400000.0, 250000.0), 'goal_heading': 0.0,
        'obstacles': [
            {'type': 'circle', 'center': (100000.0, 50000.0), 'radius': 20000.0},
            {'type': 'polygon', 'polygon': [(10.0, 10.0), (20.0, 10.0), (15.0, 25.0)]},
        ],
    }
    text = sio.scenario_to_json(state)
    back = sio.scenario_from_json(text)
    assert back['start'] == (1000.0, 2000.0)
    assert back['goal_heading'] == 0.0
    assert back['obstacles'][0]['center'] == (100000.0, 50000.0)
    assert back['obstacles'][1]['polygon'][2] == (15.0, 25.0)


def test_from_json_handles_missing_points():
    state = {'start': None, 'start_heading': 0.0, 'goal': None, 'goal_heading': 0.0,
             'obstacles': []}
    back = sio.scenario_from_json(sio.scenario_to_json(state))
    assert back['start'] is None
    assert back['obstacles'] == []
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest gui/scenario_io_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'gui.scenario_io'`.

- [ ] **Step 3: Create `gui/scenario_io.py`**

```python
"""Serialize / deserialize the GUI scenario state to JSON.

Tuples become lists in JSON; deserialization restores tuples so downstream code
that unpacks (x, y) keeps working.
"""

import json


def _pt(p):
    return None if p is None else [float(p[0]), float(p[1])]


def scenario_to_json(state):
    obstacles = []
    for o in state['obstacles']:
        if o['type'] == 'circle':
            obstacles.append({'type': 'circle', 'center': _pt(o['center']),
                              'radius': float(o['radius'])})
        else:
            obstacles.append({'type': 'polygon',
                              'polygon': [_pt(v) for v in o['polygon']]})
    doc = {
        'start': _pt(state['start']),
        'start_heading': float(state['start_heading']),
        'goal': _pt(state['goal']),
        'goal_heading': float(state['goal_heading']),
        'obstacles': obstacles,
    }
    return json.dumps(doc, indent=2)


def _tup(p):
    return None if p is None else (float(p[0]), float(p[1]))


def scenario_from_json(text):
    doc = json.loads(text)
    obstacles = []
    for o in doc.get('obstacles', []):
        if o['type'] == 'circle':
            obstacles.append({'type': 'circle', 'center': _tup(o['center']),
                              'radius': float(o['radius'])})
        else:
            obstacles.append({'type': 'polygon',
                              'polygon': [_tup(v) for v in o['polygon']]})
    return {
        'start': _tup(doc.get('start')),
        'start_heading': float(doc.get('start_heading', 0.0)),
        'goal': _tup(doc.get('goal')),
        'goal_heading': float(doc.get('goal_heading', 0.0)),
        'obstacles': obstacles,
    }
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest gui/scenario_io_test.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add gui/scenario_io.py gui/scenario_io_test.py
git commit -m "feat(gui): scenario JSON save/load"
```

---

## Task 6: `gui/map_canvas.py` — map + drawing + trajectory render

**Files:**
- Create: `gui/map_canvas.py`

**Interfaces:**
- Consumes: `trajectory.sample_trajectory` (Task 2); `config.MAP_WIDTH/MAP_HEIGHT`.
- Produces: `class MapCanvas(parent, on_map_click)`:
  - `.widget()` — the Tk widget to pack.
  - `.render(state, result, preprocessed, render_mode)` — redraw obstacles, start/goal, and the trajectory (if `result` has a path). `preprocessed`/`result` may be `None` (scenario-only draw).
  - `on_map_click(x_m, y_m)` callback invoked with map-space coordinates on left click.

- [ ] **Step 1: Smoke test (headless construction + render)**

```python
# gui/map_canvas_test.py
import matplotlib
matplotlib.use('Agg')
import tkinter as tk
import pytest
import gui.map_canvas as mc


def test_canvas_constructs_and_renders_without_error():
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display available")
    clicks = []
    canvas = mc.MapCanvas(root, on_map_click=lambda x, y: clicks.append((x, y)))
    state = {'start': (10000.0, 10000.0), 'start_heading': 0.0,
             'goal': (400000.0, 400000.0), 'goal_heading': 0.0,
             'obstacles': [{'type': 'circle', 'center': (200000.0, 200000.0), 'radius': 30000.0}]}
    canvas.render(state, result=None, preprocessed=None, render_mode='dubins')
    root.destroy()
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest gui/map_canvas_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'gui.map_canvas'`.

- [ ] **Step 3: Create `gui/map_canvas.py`**

```python
"""Matplotlib map canvas for the planner GUI: draws obstacles, start/goal, and the
planned trajectory, and reports left-clicks in map coordinates."""

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle as MplCircle, Polygon as MplPolygon

import config
import trajectory as tr


class MapCanvas:
    def __init__(self, parent, on_map_click):
        self._on_map_click = on_map_click
        self.fig = Figure(figsize=(7, 7), dpi=100, layout='tight')
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.mpl_connect('button_press_event', self._handle_click)
        self._draw_empty()

    def widget(self):
        return self.canvas.get_tk_widget()

    def _handle_click(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        if event.button == 1:
            self._on_map_click(float(event.xdata), float(event.ydata))

    def _draw_empty(self):
        self.ax.clear()
        self.ax.set_xlim(0, config.MAP_WIDTH)
        self.ax.set_ylim(0, config.MAP_HEIGHT)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')
        self.ax.grid(True, alpha=0.3)

    def render(self, state, result, preprocessed, render_mode):
        self._draw_empty()
        for o in state['obstacles']:
            if o['type'] == 'circle':
                self.ax.add_patch(MplCircle(o['center'], o['radius'],
                                            color='salmon', alpha=0.5))
            else:
                self.ax.add_patch(MplPolygon(o['polygon'], color='saddlebrown', alpha=0.6))
        if state['start'] is not None:
            self.ax.plot(*state['start'], 'go', markersize=10, label='Launch O')
        if state['goal'] is not None:
            self.ax.plot(*state['goal'], 'r*', markersize=16, label='Target T')

        if result and result.get('path'):
            R = (preprocessed or {}).get('turn_radius', config.R)
            pts = tr.sample_trajectory(result['path'], R, mode=render_mode)
            if len(pts) >= 2:
                self.ax.plot([p[0] for p in pts], [p[1] for p in pts],
                             'b-', linewidth=2.5,
                             label='Dubins' if render_mode == 'dubins' else 'Straight')
            for wp, _ in result['path']:
                self.ax.plot(wp[0], wp[1], 'bo', markersize=5, alpha=0.7)

        handles, labels = self.ax.get_legend_handles_labels()
        if labels:
            self.ax.legend(loc='upper left', fontsize=8)
        self.canvas.draw()
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest gui/map_canvas_test.py -v`
Expected: PASS (or SKIP if no display; that is acceptable for the headless CI case).

- [ ] **Step 5: Commit**

```bash
git add gui/map_canvas.py gui/map_canvas_test.py
git commit -m "feat(gui): map canvas with obstacle/trajectory rendering"
```

---

## Task 7: `gui/config_panel.py` — left configuration column

**Files:**
- Create: `gui/config_panel.py`

**Interfaces:**
- Consumes: `gui.params.PARAM_SPECS`, `gui.params.default_values` (Task 3).
- Produces: `class ConfigPanel(parent, on_run, on_set_start, on_set_goal, on_draw_polygon, on_draw_circle, on_clear, on_load, on_save)`:
  - `.widget()` — the Tk frame to pack.
  - `.values() -> dict` — current parameter values keyed by `PARAM_SPECS` keys.
  - `.numeric_start_goal() -> dict` — `{'start':(x,y)|None,'start_heading_deg':float,'goal':(x,y)|None,'goal_heading_deg':float}` from the numeric entry fields.

- [ ] **Step 1: Smoke test**

```python
# gui/config_panel_test.py
import tkinter as tk
import pytest
import gui.config_panel as cp


def test_config_panel_constructs_and_reports_values():
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display available")
    noop = lambda *a, **k: None
    panel = cp.ConfigPanel(root, on_run=noop, on_set_start=noop, on_set_goal=noop,
                           on_draw_polygon=noop, on_draw_circle=noop, on_clear=noop,
                           on_load=noop, on_save=noop)
    vals = panel.values()
    assert 'turn_radius' in vals and 'wrap_step_m' in vals
    root.destroy()
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest gui/config_panel_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'gui.config_panel'`.

- [ ] **Step 3: Create `gui/config_panel.py`**

```python
"""Left configuration column: scenario inputs, tactical params, collapsible Advanced,
and the RUN button. Builds parameter widgets from gui.params.PARAM_SPECS."""

import tkinter as tk
from tkinter import ttk

import gui.params as gp


class ConfigPanel:
    def __init__(self, parent, on_run, on_set_start, on_set_goal,
                 on_draw_polygon, on_draw_circle, on_clear, on_load, on_save):
        self.frame = ttk.Frame(parent)
        self._vars = {}

        ttk.Label(self.frame, text='Scenario', font=('Arial', 11, 'bold')).pack(anchor=tk.W, pady=(4, 2))
        sc = ttk.Frame(self.frame); sc.pack(fill=tk.X)
        ttk.Button(sc, text='Set Launch', command=on_set_start).grid(row=0, column=0, sticky='ew', padx=2, pady=2)
        ttk.Button(sc, text='Set Target', command=on_set_goal).grid(row=0, column=1, sticky='ew', padx=2, pady=2)
        ttk.Button(sc, text='Draw Island', command=on_draw_polygon).grid(row=1, column=0, sticky='ew', padx=2, pady=2)
        ttk.Button(sc, text='Draw SAM', command=on_draw_circle).grid(row=1, column=1, sticky='ew', padx=2, pady=2)
        ttk.Button(sc, text='Clear All', command=on_clear).grid(row=2, column=0, sticky='ew', padx=2, pady=2)
        sc.columnconfigure(0, weight=1); sc.columnconfigure(1, weight=1)

        # Numeric start/goal entry
        ttk.Label(self.frame, text='Numeric (x, y, heading deg)', font=('Arial', 9)).pack(anchor=tk.W, pady=(6, 0))
        self._sg = {}
        for name in ('start_x', 'start_y', 'start_h', 'goal_x', 'goal_y', 'goal_h'):
            self._sg[name] = tk.StringVar(value='')
        grid = ttk.Frame(self.frame); grid.pack(fill=tk.X)
        ttk.Label(grid, text='Launch').grid(row=0, column=0)
        for c, n in enumerate(('start_x', 'start_y', 'start_h')):
            ttk.Entry(grid, textvariable=self._sg[n], width=8).grid(row=0, column=c + 1, padx=1)
        ttk.Label(grid, text='Target').grid(row=1, column=0)
        for c, n in enumerate(('goal_x', 'goal_y', 'goal_h')):
            ttk.Entry(grid, textvariable=self._sg[n], width=8).grid(row=1, column=c + 1, padx=1)

        io = ttk.Frame(self.frame); io.pack(fill=tk.X, pady=2)
        ttk.Button(io, text='Load JSON', command=on_load).grid(row=0, column=0, sticky='ew', padx=2)
        ttk.Button(io, text='Save JSON', command=on_save).grid(row=0, column=1, sticky='ew', padx=2)
        io.columnconfigure(0, weight=1); io.columnconfigure(1, weight=1)

        ttk.Separator(self.frame).pack(fill=tk.X, pady=4)
        ttk.Label(self.frame, text='Parameters', font=('Arial', 11, 'bold')).pack(anchor=tk.W)
        self._build_param_group(self.frame, ('tactical', 'run'))

        self._adv_shown = tk.BooleanVar(value=False)
        ttk.Checkbutton(self.frame, text='Advanced ▸', variable=self._adv_shown,
                        command=self._toggle_advanced).pack(anchor=tk.W, pady=(6, 0))
        self._adv_frame = ttk.Frame(self.frame)
        self._build_param_group(self._adv_frame, ('advanced',))

        ttk.Separator(self.frame).pack(fill=tk.X, pady=6)
        ttk.Button(self.frame, text='▶ RUN', command=on_run).pack(fill=tk.X, ipady=6)

    def _build_param_group(self, parent, groups):
        for spec in gp.PARAM_SPECS:
            if spec['group'] not in groups:
                continue
            var = tk.DoubleVar(value=spec['default'])
            self._vars[spec['key']] = var
            row = ttk.Frame(parent); row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=spec['label'], width=20, font=('Arial', 8)).pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=var, width=10).pack(side=tk.RIGHT)

    def _toggle_advanced(self):
        if self._adv_shown.get():
            self._adv_frame.pack(fill=tk.X)
        else:
            self._adv_frame.pack_forget()

    def widget(self):
        return self.frame

    def values(self):
        return {k: v.get() for k, v in self._vars.items()}

    def _opt_float(self, name):
        text = self._sg[name].get().strip()
        try:
            return float(text)
        except ValueError:
            return None

    def numeric_start_goal(self):
        sx, sy = self._opt_float('start_x'), self._opt_float('start_y')
        gx, gy = self._opt_float('goal_x'), self._opt_float('goal_y')
        return {
            'start': (sx, sy) if sx is not None and sy is not None else None,
            'start_heading_deg': self._opt_float('start_h') or 0.0,
            'goal': (gx, gy) if gx is not None and gy is not None else None,
            'goal_heading_deg': self._opt_float('goal_h') or 0.0,
        }
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest gui/config_panel_test.py -v`
Expected: PASS or SKIP (no display).

- [ ] **Step 5: Commit**

```bash
git add gui/config_panel.py gui/config_panel_test.py
git commit -m "feat(gui): configuration panel (params + scenario inputs + advanced)"
```

---

## Task 8: `gui/results_panel.py` — right results column

**Files:**
- Create: `gui/results_panel.py`

**Interfaces:**
- Consumes: nothing from prior tasks at construction (renders a summary dict shaped like `gui.summary.compute_summary` output).
- Produces: `class ResultsPanel(parent, on_render_mode_change)`:
  - `.widget()` — the Tk frame.
  - `.show_summary(summary)` — display the metrics dict.
  - `.log(message)` — append a line to the status log.
  - `.render_mode() -> str` — `'dubins'` or `'straight'` from the toggle.

- [ ] **Step 1: Smoke test**

```python
# gui/results_panel_test.py
import tkinter as tk
import pytest
import gui.results_panel as rp


def test_results_panel_shows_summary_and_reports_mode():
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display available")
    panel = rp.ResultsPanel(root, on_render_mode_change=lambda mode: None)
    panel.show_summary({'success': True, 'distance_km': 123.4, 'num_waypoints': 5,
                        'num_turns': 3, 'max_turn_deg': 42.0, 'runtime_ms': 88.0,
                        'iterations': 120, 'valid': True})
    panel.log('done')
    assert panel.render_mode() in ('dubins', 'straight')
    root.destroy()
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest gui/results_panel_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'gui.results_panel'`.

- [ ] **Step 3: Create `gui/results_panel.py`**

```python
"""Right results column: metric summary, render-mode toggle, and status log."""

import tkinter as tk
from tkinter import ttk

_FIELDS = [
    ('success', 'Success'),
    ('distance_km', 'Distance (km)'),
    ('num_waypoints', 'Waypoints'),
    ('num_turns', 'Turns'),
    ('max_turn_deg', 'Max turn (deg)'),
    ('runtime_ms', 'Runtime (ms)'),
    ('iterations', 'Iterations'),
    ('valid', 'Collision-free'),
]


class ResultsPanel:
    def __init__(self, parent, on_render_mode_change):
        self.frame = ttk.Frame(parent)
        self._on_mode = on_render_mode_change

        ttk.Label(self.frame, text='Results', font=('Arial', 11, 'bold')).pack(anchor=tk.W, pady=(4, 2))
        grid = ttk.Frame(self.frame); grid.pack(fill=tk.X)
        self._value_vars = {}
        for r, (key, label) in enumerate(_FIELDS):
            ttk.Label(grid, text=label, width=16, font=('Arial', 9)).grid(row=r, column=0, sticky='w')
            var = tk.StringVar(value='--')
            self._value_vars[key] = var
            ttk.Label(grid, textvariable=var, font=('Arial', 9, 'bold')).grid(row=r, column=1, sticky='w')

        ttk.Separator(self.frame).pack(fill=tk.X, pady=6)
        ttk.Label(self.frame, text='Render mode', font=('Arial', 10, 'bold')).pack(anchor=tk.W)
        self._mode = tk.StringVar(value='dubins')
        for val, text in (('dubins', 'Dubins (real)'), ('straight', 'Straight (waypoints)')):
            ttk.Radiobutton(self.frame, text=text, value=val, variable=self._mode,
                            command=lambda: self._on_mode(self._mode.get())).pack(anchor=tk.W)

        ttk.Separator(self.frame).pack(fill=tk.X, pady=6)
        ttk.Label(self.frame, text='Log', font=('Arial', 10, 'bold')).pack(anchor=tk.W)
        self._log = tk.Text(self.frame, height=14, width=30, font=('Courier', 8))
        self._log.pack(fill=tk.BOTH, expand=True)

    def widget(self):
        return self.frame

    def _fmt(self, key, value):
        if key in ('success', 'valid'):
            return 'yes' if value else 'no'
        if isinstance(value, float):
            return f'{value:.2f}'
        return str(value)

    def show_summary(self, summary):
        for key, _ in _FIELDS:
            self._value_vars[key].set(self._fmt(key, summary.get(key)))

    def log(self, message):
        self._log.insert(tk.END, message + '\n')
        self._log.see(tk.END)

    def render_mode(self):
        return self._mode.get()
```

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest gui/results_panel_test.py -v`
Expected: PASS or SKIP (no display).

- [ ] **Step 5: Commit**

```bash
git add gui/results_panel.py gui/results_panel_test.py
git commit -m "feat(gui): results panel (summary + render toggle + log)"
```

---

## Task 9: `gui/app.py` orchestrator + wiring + remove old GUI

**Files:**
- Create: `gui/app.py`
- Modify: `launch_gui.py`
- Remove: `gui_scenario_builder.py`

**Interfaces:**
- Consumes: `gui.config_panel.ConfigPanel`, `gui.map_canvas.MapCanvas`, `gui.results_panel.ResultsPanel`, `gui.params`, `gui.summary`, `gui.scenario_io`, `preprocessing`, `kinodynamic_astar`.
- Produces: `class PlannerApp(root)`; `gui/app.py` runnable as the GUI entry.

- [ ] **Step 1: Headless integration test (no Tk) for the run pipeline helper**

The planning pipeline used by the app is a module-level function so it can be tested without Tk.

```python
# gui/app_pipeline_test.py
import math
import gui.app as app


def test_run_pipeline_solves_simple_scenario():
    state = {'start': (10000.0, 250000.0), 'start_heading': 0.0,
             'goal': (400000.0, 250000.0), 'goal_heading': 0.0,
             'obstacles': []}
    import gui.params as gp
    result, preprocessed, raw_circles, raw_polys, runtime_s = app.run_pipeline(
        state, gp.default_values())
    assert result['success'] is True
    assert runtime_s >= 0.0
    assert preprocessed['turn_radius'] > 0
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m pytest gui/app_pipeline_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'gui.app'`.

- [ ] **Step 3: Create `gui/app.py`**

```python
"""Planner GUI orchestrator: 3-column layout (config | map | results) and the
scenario→plan pipeline. run_pipeline is Tk-free so it can be tested headlessly."""

import math
import time
import tkinter as tk
from tkinter import ttk, filedialog

import config
import preprocessing as prep
import kinodynamic_astar as astar
import gui.params as gp
import gui.summary as gsummary
import gui.scenario_io as sio
from gui.config_panel import ConfigPanel
from gui.map_canvas import MapCanvas
from gui.results_panel import ResultsPanel


def _build_scenario_dict(state):
    obstacles = []
    for o in state['obstacles']:
        if o['type'] == 'circle':
            obstacles.append({'type': 'circle', 'center': o['center'], 'radius': o['radius']})
        else:
            obstacles.append({'type': 'polygon', 'polygon': o['polygon']})
    islands = [o['polygon'] for o in state['obstacles'] if o['type'] == 'polygon']
    sams = [(o['center'], o['radius']) for o in state['obstacles'] if o['type'] == 'circle']
    return {
        'start': state['start'], 'start_heading': state['start_heading'],
        'goal': state['goal'], 'goal_heading': state['goal_heading'],
        'obstacles': obstacles, 'islands': islands, 'sam_sites': sams,
    }


def run_pipeline(state, values):
    """Apply params, preprocess, plan. Returns (result, preprocessed,
    raw_circles, raw_polys, runtime_s)."""
    kwargs = gp.apply_overrides(values)
    scenario = _build_scenario_dict(state)
    preprocessed = prep.prepare_scenario(scenario, **kwargs)
    raw_circles = [(o['center'], o['radius']) for o in scenario['obstacles'] if o['type'] == 'circle']
    raw_polys = [o['polygon'] for o in scenario['obstacles'] if o['type'] == 'polygon']
    t0 = time.perf_counter()
    result = astar.plan_trajectory(preprocessed, verbose=False)
    runtime_s = time.perf_counter() - t0
    return result, preprocessed, raw_circles, raw_polys, runtime_s


class PlannerApp:
    def __init__(self, root):
        self.root = root
        root.title('Missile Path Planner')
        self.state = {'start': None, 'start_heading': 0.0,
                      'goal': None, 'goal_heading': 0.0, 'obstacles': []}
        self.result = None
        self.preprocessed = None
        self.raw_circles = []
        self.raw_polys = []
        self.mode = 'idle'          # idle | start | goal | polygon | circle
        self._poly_pts = []
        self._circle_center = None

        container = ttk.Frame(root); container.pack(fill=tk.BOTH, expand=True)
        self.config_panel = ConfigPanel(
            container, on_run=self.on_run, on_set_start=lambda: self._set_mode('start'),
            on_set_goal=lambda: self._set_mode('goal'),
            on_draw_polygon=lambda: self._set_mode('polygon'),
            on_draw_circle=lambda: self._set_mode('circle'),
            on_clear=self.on_clear, on_load=self.on_load, on_save=self.on_save)
        self.config_panel.widget().pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=4)

        self.canvas = MapCanvas(container, on_map_click=self.on_map_click)
        self.canvas.widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.results = ResultsPanel(container, on_render_mode_change=self.on_mode_change)
        self.results.widget().pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=4)

        self._redraw()

    def _set_mode(self, mode):
        self.mode = mode
        self._poly_pts = []
        self._circle_center = None
        self.results.log(f'Mode: {mode} (click on the map)')

    def on_map_click(self, x, y):
        if self.mode == 'start':
            self.state['start'] = (x, y); self.mode = 'idle'
        elif self.mode == 'goal':
            self.state['goal'] = (x, y); self.mode = 'idle'
        elif self.mode == 'circle':
            if self._circle_center is None:
                self._circle_center = (x, y)
                self.results.log('Click again to set radius')
            else:
                r = math.hypot(x - self._circle_center[0], y - self._circle_center[1])
                self.state['obstacles'].append({'type': 'circle', 'center': self._circle_center, 'radius': r})
                self._circle_center = None; self.mode = 'idle'
        elif self.mode == 'polygon':
            close_tol = config.MAP_WIDTH * 0.02
            if (len(self._poly_pts) >= 3 and
                    math.hypot(x - self._poly_pts[0][0], y - self._poly_pts[0][1]) < close_tol):
                self.state['obstacles'].append({'type': 'polygon', 'polygon': list(self._poly_pts)})
                self._poly_pts = []
                self.mode = 'idle'
                self.results.log('Polygon closed')
            else:
                self._poly_pts.append((x, y))
                self.results.log(f'Polygon point {len(self._poly_pts)} (click near first point to close)')
        self._redraw()

    def on_clear(self):
        self.state['obstacles'] = []
        self._poly_pts = []
        self.result = None
        self._redraw()

    def on_save(self):
        path = filedialog.asksaveasfilename(defaultextension='.json')
        if path:
            with open(path, 'w') as f:
                f.write(sio.scenario_to_json(self.state))
            self.results.log(f'Saved {path}')

    def on_load(self):
        path = filedialog.askopenfilename(filetypes=[('JSON', '*.json')])
        if path:
            with open(path) as f:
                self.state = sio.scenario_from_json(f.read())
            self.result = None
            self._redraw()
            self.results.log(f'Loaded {path}')

    def _apply_numeric_entries(self):
        sg = self.config_panel.numeric_start_goal()
        if sg['start'] is not None:
            self.state['start'] = sg['start']
            self.state['start_heading'] = math.radians(sg['start_heading_deg'])
        if sg['goal'] is not None:
            self.state['goal'] = sg['goal']
            self.state['goal_heading'] = math.radians(sg['goal_heading_deg'])

    def on_run(self):
        self._apply_numeric_entries()
        if self.state['start'] is None or self.state['goal'] is None:
            self.results.log('ERROR: set launch and target first')
            return
        self.results.log('Planning...')
        try:
            (self.result, self.preprocessed, self.raw_circles,
             self.raw_polys, runtime_s) = run_pipeline(self.state, self.config_panel.values())
        except Exception as e:
            self.results.log(f'ERROR: {e}')
            return
        summary = gsummary.compute_summary(
            self.result, self.preprocessed, self.raw_circles, self.raw_polys,
            self.results.render_mode(), runtime_s)
        self.results.show_summary(summary)
        self.results.log('Done' if summary['success'] else 'No path found')
        self._redraw()

    def on_mode_change(self, _mode):
        self._redraw()              # re-render only; do NOT re-plan

    def _redraw(self):
        self.canvas.render(self.state, self.result, self.preprocessed,
                           self.results.render_mode())
```

- [ ] **Step 4: Run to verify the pipeline test passes**

Run: `python -m pytest gui/app_pipeline_test.py -v`
Expected: PASS.

- [ ] **Step 5: Replace `launch_gui.py` body**

Replace the `main()` function in `launch_gui.py` so it imports the new app:

```python
def main():
    """Launch the interactive GUI"""
    print("Missile Path Planning - Interactive Planner")
    try:
        import tkinter as tk
        from gui.app import PlannerApp
    except ImportError as e:
        print(f"Import Error: {e}")
        sys.exit(1)
    root = tk.Tk()
    PlannerApp(root)
    root.mainloop()
```

- [ ] **Step 6: Remove the old GUI and verify nothing imports it**

```bash
git rm gui_scenario_builder.py
grep -rn "gui_scenario_builder" --include=*.py . || echo "no references remain"
```
Expected: `no references remain`.

- [ ] **Step 7: Full suite + import smoke**

Run: `python -m pytest -q`
Expected: all PASS.

Run: `python -c "import launch_gui, gui.app; print('import ok')"`
Expected: prints `import ok` (no Tk window opened by import).

- [ ] **Step 8: Commit**

```bash
git add gui/app.py gui/app_pipeline_test.py launch_gui.py
git commit -m "feat(gui): app orchestrator, wire launch_gui, remove legacy GUI"
```

---

## Final verification

- [ ] `python -m pytest -q` → all pass.
- [ ] `python -c "import matplotlib; matplotlib.use('Agg'); import map_generator as mg, preprocessing as prep, kinodynamic_astar as astar, visualizer as viz; sc=mg.scenario14_combined_threat(); pre=prep.prepare_scenario(sc); r=astar.plan_trajectory(pre, verbose=False); viz.plot_scenario(sc,pre,r,save_path='/tmp/dub.png'); print('dubins render ok')"` → prints ok; the trajectory is continuous (no jumps).
- [ ] Manual GUI check (needs a display): `python launch_gui.py` → set launch/target, draw a SAM, RUN; results panel shows distance/waypoints/runtime; toggle Dubins/Straight re-renders without re-planning.

## Notes on residual risk

- The Dubins solver renders between the planner's `(pos, heading)` waypoints; with the planner's arrival-direction headings this matches the validated turn-at-waypoint maneuver. If a future change makes waypoint headings inconsistent with segment directions, Dubins curves may visibly diverge from the straight polyline — that is a rendering signal, not a planner bug.
- `apply_overrides` mutates module-level `config` attributes (single-process GUI). Tests that assert specific `config` values should set them explicitly, since the GUI can leave them changed after a run.
- Tk widget tests `pytest.skip` when no display is available; they protect construction/contract, not pixel output. The pure helpers (`params`, `summary`, `scenario_io`, `run_pipeline`) carry the real coverage.
