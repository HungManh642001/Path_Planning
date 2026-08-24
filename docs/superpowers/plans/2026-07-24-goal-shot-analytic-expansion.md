# Goal-Shot Analytic Expansion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Kill the adverse-heading search flood by adding a Hybrid-A*-style analytic terminal "shot": from each popped state, analytically construct a 2-corner vehicle-legal maneuver directly to the aligned goal; when one is collision-free and kinodynamically valid, return it immediately instead of flooding the goal region.

**Architecture:** A new pure-geometry module `core/goal_shot.py` enumerates 2-corner maneuver candidates (turn ≤ α_max at the current state → straight → intermediate corner C → turn ≤ α_max → arrive at `W_{n-1}` with a heading inside the ±α_max terminal cone). `KinodynamicAstar._try_goal_shot` collision-checks the two straight legs with the existing exact checker and, on the first valid candidate, links parent pointers and returns the terminal state. `search()` calls it once per popped state (fixed-goal mode only) and returns the reconstructed path on success. The shot is a pure ADDITION — every emitted edge passes the same `_check_collision` + đoản-trình reserves as any search edge, so validity is unchanged.

**Tech Stack:** Python 3, numpy/shapely (already used), pytest. No new dependencies.

## Global Constraints

- Units are **meters**; angles are **radians** throughout algorithm code (verbatim from CLAUDE.md).
- `core/goal_shot.py` is **pure geometry**: no `config` or planner imports; all tolerances/steps are parameters (mirrors `core/arc_geometry.py`).
- Collision checks are **exact** (zero tolerance): reuse `KinodynamicAstar._check_collision`; do not add tolerances.
- Đoản-trình (min straight) reserve at a corner turning by angle `α` is `R*tan(α/2)`; a straight leg is valid iff `length − near_reserve − far_reserve ≥ _MIN_STRAIGHT_M` (`10.0`).
- The shot applies to **fixed-goal mode only** (`goal_heading is not None`); free-goal mode is already fast and must be left untouched.
- Test files end in `*_test.py` (committed); `pytest.ini` sets `pythonpath = .` and `testpaths = tests`. Run tests from repo root with `python -m pytest -q`.
- A planner **state** is `(waypoint, heading)`; a successor's stored heading is the bearing of the leg INTO it (`su.angle_to_heading(prev, this)`), and the turn at `prev` is `angdiff(that bearing, prev.heading)`.

---

### Task 1: Pure-geometry 2-corner candidate enumerator

**Files:**
- Create: `core/goal_shot.py`
- Test: `tests/goal_shot_test.py`

**Interfaces:**
- Produces:
  - `goal_shot._angdiff(a, b) -> float` — signed difference `a−b` in `[-π, π]`.
  - `goal_shot.two_corner_candidates(P, h, goal_wp, goal_heading, R, alpha_max, min_straight, straight_budget_in, min_straight_in, num_dir=9, num_cone=9) -> list[tuple]` — each tuple is `(total_len, C, d1, phi, budget_C, budget_W)`:
    - `total_len` (float): `leg1_len + leg2_len`, list sorted ascending by it.
    - `C` (x, y): the intermediate corner.
    - `d1` (float): heading of leg1 `P→C` (= turn direction at `P`).
    - `phi` (float): heading of leg2 `C→goal_wp` (= arrival heading at the goal).
    - `budget_C` (float): leg1 length minus its near reserve at `P` (goes into the `C` state's `straight_budget`).
    - `budget_W` (float): leg2 length minus its near reserve at `C` (goes into the goal state's `straight_budget`).
  - Angle/length-feasible candidates ONLY (turns ≤ `alpha_max`, both legs satisfy đoản-trình, incoming deferred đoản-trình via `straight_budget_in`/`min_straight_in`). **No collision checking** here (the caller does that).

- [ ] **Step 1: Write the failing tests**

Create `tests/goal_shot_test.py`:

```python
"""Pure-geometry tests for the 2-corner goal-shot candidate enumerator."""
import math

import core.goal_shot as gs

# Non-degenerate 2-corner geometry: at the origin heading EAST, goal to the
# north-east, approach heading NORTH. The vehicle turns onto an intermediate
# leg and then onto the northern approach — a genuine 2-corner maneuver with a
# well-defined intermediate corner. (The exact-opposite-heading case at the
# origin is DEGENERATE: the ideal corner collapses onto the start point
# (leg1 length 0), so no candidate is returned and the search must first
# travel before a 2-corner shot becomes feasible — that is expected, correct
# behavior, not a bug.)
P = (0.0, 0.0)
H = 0.0              # heading east
GOAL = (100000.0, 50000.0)
GH = math.pi / 2     # approach heading north
R = 8000.0
AMAX = math.pi / 2   # 90 deg


def test_two_corner_candidates_are_feasible():
    cands = gs.two_corner_candidates(P, H, GOAL, GH, R, AMAX,
                                     10.0, 1e9, 10.0, num_dir=9, num_cone=9)
    assert cands, "adverse-launch open geometry must yield 2-corner candidates"
    for total_len, C, d1, phi, budget_C, budget_W in cands:
        a1 = abs(gs._angdiff(d1, H))          # turn at P
        a2 = abs(gs._angdiff(phi, d1))        # turn at C
        at = abs(gs._angdiff(GH, phi))        # terminal turn at the goal
        assert a1 <= AMAX + 1e-9
        assert a2 <= AMAX + 1e-9
        assert at <= AMAX + 1e-9
        # C is consistent with the reported leg headings.
        assert abs(gs._angdiff(math.atan2(C[1] - P[1], C[0] - P[0]), d1)) < 1e-6
        assert abs(gs._angdiff(math.atan2(GOAL[1] - C[1], GOAL[0] - C[0]), phi)) < 1e-6
        # Both legs keep the far-end reserve + min straight.
        assert budget_C - R * math.tan(a2 / 2.0) >= 10.0 - 1e-6
        assert budget_W - R * math.tan(at / 2.0) >= 10.0 - 1e-6


def test_candidates_sorted_shortest_first():
    cands = gs.two_corner_candidates(P, H, GOAL, GH, R, AMAX, 10.0, 1e9, 10.0)
    lengths = [c[0] for c in cands]
    assert lengths == sorted(lengths)


def test_reserves_reject_everything_when_min_straight_huge():
    cands = gs.two_corner_candidates(P, H, GOAL, GH, R, AMAX, 1e12, 1e9, 10.0)
    assert cands == []


def test_incoming_budget_gate_blocks_first_turn():
    # A tiny incoming straight budget cannot afford the near reserve of a large
    # first turn, so no candidate whose turn-at-P exceeds the budget survives.
    cands = gs.two_corner_candidates(P, H, GOAL, GH, R, AMAX,
                                     10.0, 100.0, 10.0)  # budget_in = 100 m
    for _tot, _C, d1, _phi, _bC, _bW in cands:
        a1 = abs(gs._angdiff(d1, H))
        assert 100.0 - R * math.tan(a1 / 2.0) >= 10.0 - 1e-9
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/goal_shot_test.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'core.goal_shot'`.

- [ ] **Step 3: Write the module**

Create `core/goal_shot.py`:

```python
"""Pure geometry for the analytic terminal "goal shot": enumerate 2-corner
vehicle maneuvers that connect a search state (P, h) to the goal waypoint,
arriving with a heading inside the +-alpha_max terminal cone.

A candidate is: turn <= alpha_max at P onto leg 1 (direction d1), fly straight
to an intermediate corner C, turn <= alpha_max at C onto leg 2 (direction phi),
fly straight to the goal, arriving heading phi (within alpha_max of the goal
heading so the terminal turn onto the approach is feasible). C is the
intersection of the ray from P along d1 and the back-ray into the goal along
phi. No planner/config imports; all tolerances are parameters.
"""
import math


def _angdiff(a, b):
    """Smallest signed difference a-b normalised to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def two_corner_candidates(P, h, goal_wp, goal_heading, R, alpha_max,
                          min_straight, straight_budget_in, min_straight_in,
                          num_dir=9, num_cone=9):
    """Feasible (angle + length) 2-corner maneuvers, shortest first.

    Args:
        P: current waypoint (x, y).
        h: current heading (rad).
        goal_wp: goal waypoint W_{n-1} (x, y).
        goal_heading: required approach heading at the goal (rad).
        R: turn radius (m).
        alpha_max: max turn angle (rad).
        min_straight: minimum usable straight length per leg (m).
        straight_budget_in: remaining straight budget of the leg INTO P
            (deferred đoản-trình: P's incoming leg must still keep
            min_straight_in after P's turn reserve).
        min_straight_in: đoản-trình threshold for P's incoming leg.
        num_dir: number of turn-at-P directions sampled across [h ± alpha_max].
        num_cone: number of arrival headings sampled across
            [goal_heading ± alpha_max].

    Returns:
        list of (total_len, C, d1, phi, budget_C, budget_W), sorted by
        total_len ascending. Empty if nothing is angle/length feasible.
    """
    Px, Py = P
    Dx, Dy = goal_wp[0] - Px, goal_wp[1] - Py
    out = []
    for i in range(num_dir):
        d1 = h - alpha_max + (2.0 * alpha_max) * i / (num_dir - 1)
        a1 = abs(_angdiff(d1, h))                      # turn at P
        # Deferred đoản-trình of P's incoming leg (near reserve = R*tan(a1/2)).
        if straight_budget_in - R * math.tan(a1 / 2.0) < min_straight_in:
            continue
        Ux, Uy = math.cos(d1), math.sin(d1)
        r1 = R * math.tan(a1 / 2.0)                     # leg1 near reserve (at P)
        for j in range(num_cone):
            phi = goal_heading - alpha_max + (2.0 * alpha_max) * j / (num_cone - 1)
            a2 = abs(_angdiff(phi, d1))                 # turn at C
            if a2 > alpha_max:
                continue
            at = abs(_angdiff(goal_heading, phi))       # terminal turn at goal
            if at > alpha_max:                          # (guard float on cone edge)
                continue
            Vx, Vy = math.cos(phi), math.sin(phi)
            det = Ux * Vy - Uy * Vx
            if abs(det) < 1e-9:
                continue                                # legs parallel: no corner
            t = (Dx * Vy - Dy * Vx) / det               # leg1 length P->C
            u = (Ux * Dy - Uy * Dx) / det               # leg2 length C->goal
            if t <= 0.0 or u <= 0.0:
                continue                                # corner behind an endpoint
            r2 = R * math.tan(a2 / 2.0)                 # reserve at C
            rt = R * math.tan(at / 2.0)                 # terminal reserve at goal
            budget_C = t - r1                           # leg1 minus its near reserve
            if budget_C - r2 < min_straight:            # leg1 far reserve + straight
                continue
            budget_W = u - r2                           # leg2 minus its near reserve
            if budget_W - rt < min_straight:            # leg2 far reserve + straight
                continue
            C = (Px + t * Ux, Py + t * Uy)
            out.append((t + u, C, d1, phi, budget_C, budget_W))
    out.sort(key=lambda c: c[0])
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/goal_shot_test.py -q`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit**

```bash
git add core/goal_shot.py tests/goal_shot_test.py
git commit -m "feat(core): 2-corner goal-shot candidate geometry"
```

---

### Task 2: Wire the goal shot into the planner

**Files:**
- Modify: `config.py` (append a "GOAL SHOT" block after the A* SEARCH section, near line 164)
- Modify: `core/kinodynamic_astar.py` (add `import core.goal_shot as gshot` near line 16; add `_try_goal_shot` method; call it in `search()`)
- Test: `tests/goal_shot_planner_test.py`

**Interfaces:**
- Consumes: `goal_shot.two_corner_candidates(...)` from Task 1; `KinodynamicAstar._check_collision`, `State`, `_MIN_STRAIGHT_M`, `_reconstruct_path` (existing).
- Produces:
  - `config.GOAL_SHOT_ENABLED: bool`, `config.GOAL_SHOT_EVERY_N: int`, `config.GOAL_SHOT_DIRS: int`, `config.GOAL_SHOT_CONE: int`.
  - `KinodynamicAstar._try_goal_shot(self, current) -> State | None` — the terminal goal `State` (with `parent` linked back through the intermediate corner to `current`) if a collision-free valid shot exists, else `None`.

- [ ] **Step 1: Write the failing tests**

Create `tests/goal_shot_planner_test.py`:

```python
"""End-to-end tests: the goal shot collapses the adverse-heading flood."""
import math

import config
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.path_validation as pv
import render.trajectory as tr


def _adverse_scenario(start_heading_deg, goal_heading_deg):
    """Open water, start->goal due east 200 km, with adverse headings."""
    return {
        'start': (100000.0, 100000.0),
        'start_heading': math.radians(start_heading_deg),
        'goal': (300000.0, 100000.0),
        'goal_heading': math.radians(goal_heading_deg),
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
    }


def _oracle_ok(result, pre):
    full = tr.build_full_path(result['path'], pre)
    return pv.path_is_valid(
        full, pre['circle_obstacles'], pre['polygon_obstacles'],
        config.R, config.ALPHA_MAX_RAD, config.L0, config.DSS)


def test_shot_solves_adverse_in_few_iterations(monkeypatch):
    monkeypatch.setattr(config, 'GOAL_SHOT_ENABLED', True)
    pre = prep.prepare_scenario(_adverse_scenario(45, 180))
    result = astar.plan_trajectory(pre)
    assert result['success']
    # Inject-into-open collapses the flood (baseline ~19000) but keeps A*
    # cost ordering, so it expands more than immediate-return would; ~760 here.
    assert result['stats']['iterations'] < 2000
    assert _oracle_ok(result, pre)


def test_shot_disabled_still_floods(monkeypatch):
    monkeypatch.setattr(config, 'GOAL_SHOT_ENABLED', False)
    pre = prep.prepare_scenario(_adverse_scenario(45, 180))
    result = astar.plan_trajectory(pre)
    assert result['success']
    assert result['stats']['iterations'] > 1000     # no shot => flood


def test_shot_valid_on_full_reversal(monkeypatch):
    monkeypatch.setattr(config, 'GOAL_SHOT_ENABLED', True)
    pre = prep.prepare_scenario(_adverse_scenario(180, 180))
    result = astar.plan_trajectory(pre)
    assert result['success']
    assert _oracle_ok(result, pre)


def test_free_goal_unaffected(monkeypatch):
    monkeypatch.setattr(config, 'GOAL_SHOT_ENABLED', True)
    scen = _adverse_scenario(180, 0)
    scen['goal_heading'] = None                      # free-goal mode
    pre = prep.prepare_scenario(scen)
    result = astar.plan_trajectory(pre)
    assert result['success']
    assert _oracle_ok(result, pre)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/goal_shot_planner_test.py -q`
Expected: FAIL — `test_shot_solves_adverse_in_few_iterations` fails on `iterations < 500` (no shot yet; also `AttributeError` on `config.GOAL_SHOT_ENABLED` until Step 3 adds it).

- [ ] **Step 3a: Add config knobs**

In `config.py`, immediately after the `NUM_STRATEGY_B = 5` block (around line 164), insert:

```python
# ====== GOAL SHOT (analytic terminal connect) ======
# Hybrid-A*-style analytic expansion. From each popped state the planner tries
# a 2-corner vehicle-legal maneuver straight to the goal, arriving within
# alpha_max of goal_heading; a valid one is INJECTED into OPEN with its true g
# and accepted only when it is the cheapest frontier node. This collapses the
# adverse-approach flood (the Euclid heuristic is blind to the terminal
# heading, so misaligned states pile up near the goal) WITHOUT regressing path
# quality. Fixed-goal mode only — free-goal is already fast.
GOAL_SHOT_ENABLED = True

# Attempt the shot every N popped states. The check is cheap (angle filter,
# then at most a few 2-segment collision checks) and the search returns on the
# first success, so 1 (every pop) is fine; raise it only to cap per-pop cost on
# ultra-dense maps where the shot rarely connects.
GOAL_SHOT_EVERY_N = 1

# Candidate scan resolution: turn-at-P directions across [h ± alpha_max] and
# arrival headings across [goal_heading ± alpha_max]. 9x9 measured sufficient.
GOAL_SHOT_DIRS = 9
GOAL_SHOT_CONE = 9
```

- [ ] **Step 3b: Import the geometry module**

In `core/kinodynamic_astar.py`, after `import core.arc_geometry as ag` (line 16), add:

```python
import core.goal_shot as gshot
```

- [ ] **Step 3c: Add the `_try_goal_shot` method**

In `core/kinodynamic_astar.py`, insert this method into `class KinodynamicAstar` immediately BEFORE `def _reconstruct_path` (around line 817):

```python
    def _try_goal_shot(self, current):
        """Analytic 2-corner connect from `current` to the aligned goal.

        Fixed-goal mode only. Scans 2-corner candidates (turn <= alpha_max at
        current -> straight -> corner C -> turn <= alpha_max -> arrive at the
        goal waypoint within alpha_max of goal_heading), exact-collision-checks
        the two straight legs, and on the first valid candidate builds the
        corner + goal States with parent pointers linked back to `current`.
        Returns the goal State (ready for _reconstruct_path) or None.

        The emitted maneuver is validated identically to any search edge:
        each leg passes _check_collision and the đoản-trình reserves are
        enforced inside two_corner_candidates, so the returned path is valid.
        """
        if self._free_goal:
            return None
        gw = self.goal_state.waypoint
        gh = self.goal_state.heading
        cands = gshot.two_corner_candidates(
            current.waypoint, current.heading, gw, gh,
            self.R, self.alpha_max_rad, _MIN_STRAIGHT_M,
            current.straight_budget, current.min_straight_in,
            num_dir=config.GOAL_SHOT_DIRS, num_cone=config.GOAL_SHOT_CONE)
        base_g = self.g_scores[current]
        for _total, C, d1, phi, budget_C, budget_W in cands:
            if not self._check_collision(current.waypoint, C):
                continue
            if not self._check_collision(C, gw):
                continue
            # Leg 1: current -> C (stored heading = leg bearing d1).
            c_state = State(C, d1)
            c_state.parent = current
            a1 = abs(_angle_diff(d1, current.heading))
            c_state.g_cost = (base_g + math.dist(current.waypoint, C)
                              + config.TURN_PENALTY_WEIGHT * a1)
            c_state.straight_budget = budget_C
            # Leg 2: C -> goal (stored heading = arrival bearing phi).
            w_state = State(gw, phi)
            w_state.parent = c_state
            a2 = abs(_angle_diff(phi, d1))
            w_state.g_cost = (c_state.g_cost + math.dist(C, gw)
                              + config.TURN_PENALTY_WEIGHT * a2)
            w_state.straight_budget = budget_W
            return w_state
        return None
```

- [ ] **Step 3d: Call the shot in `search()`**

In `core/kinodynamic_astar.py`, inside `search()`, find the escape-valve re-arm block (around line 750):

```python
            if len(self.open_set) <= 1 and self.num_strategy_b <= 0:
                self.num_strategy_b = config.NUM_STRATEGY_B
```

Immediately AFTER it (before the `dist_to_goal = ...` computation), insert:

```python
            # Analytic terminal shot: analytically construct a 2-corner
            # maneuver straight to the aligned goal and INJECT it into OPEN
            # with its true g (h = 0). A* accepts it via the normal goal-accept
            # block only when it is the cheapest frontier node, so the shot
            # prunes the adverse-heading flood WITHOUT sacrificing path quality
            # (immediate-return grabbed the first valid connect from whatever
            # state popped first, which regressed hard-seed 674 by ~40 km).
            # Fixed-goal mode only.
            if (config.GOAL_SHOT_ENABLED and not self._free_goal
                    and (self.iteration_count % config.GOAL_SHOT_EVERY_N) == 0):
                shot = self._try_goal_shot(current)
                if shot is not None:
                    tentative_g = shot.g_cost
                    if tentative_g < self.g_scores.get(shot, float('inf')):
                        self.g_scores[shot] = tentative_g
                        shot.h_cost = 0.0
                        heapq.heappush(self.open_set, (
                            shot.g_cost + config.HEURISTIC_WEIGHT * shot.h_cost,
                            self.iteration_count, shot))
```

The pushed `shot` is the goal `State`; when it is later popped as the cheapest
node, the existing goal-accept block (`dist_to_goal < GOAL_THRESHOLD` and
arrival heading within `alpha_max` of `goal_heading`) reconstructs and returns
it. No new return path is needed. In `_try_goal_shot`, base the corner costs on
`self.g_scores[current]` (not `current.g_cost`) to match the normal
successor-expansion path exactly.

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/goal_shot_planner_test.py -q`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit**

```bash
git add config.py core/kinodynamic_astar.py tests/goal_shot_planner_test.py
git commit -m "feat(core): analytic terminal goal-shot to collapse adverse-heading flood"
```

---

### Task 3: Regression sweep + adverse-seed A/B measurement

**Files:**
- Create: `scripts/goal_shot_ab.py` (measurement harness; not a pytest file)
- Test: run the full existing suite (no new test file)

**Interfaces:**
- Consumes: `config.GOAL_SHOT_ENABLED`, `batch_random_test.generate_random_scenario`, `plan_trajectory`, `path_is_valid` (all existing).

- [ ] **Step 1: Run the full existing suite with the shot ON (default)**

Run: `python -m pytest -q`
Expected: PASS — same green set as before this branch's work (the shot only adds edges; existing scenarios still succeed and stay oracle-valid). If any previously-green test now fails, STOP and treat it as a regression (do NOT adjust the test to pass).

- [ ] **Step 2: Write the A/B measurement harness**

Create `scripts/goal_shot_ab.py`:

```python
"""A/B: goal-shot ON vs OFF on the adverse-heading random seeds.

Reports, per seed: solved?, iterations, planning time, path length, and
oracle validity. Flood seeds should flip from FAIL/slow to solved-fast with no
length regression on the seeds that already passed.
"""
import math
import time

import config
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.path_validation as pv
import render.trajectory as tr
from batch_random_test import generate_random_scenario

# Seeds 5, 7, 8 are FEASIBILITY failures (start in obstacle / DSS leg blocked),
# out of scope for the shot; skip them here.
SEEDS = [0, 1, 2, 3, 4, 6, 9]


def _run(seed):
    scen = generate_random_scenario(seed=seed)
    pre = prep.prepare_scenario(scen)
    t0 = time.perf_counter()
    res = astar.plan_trajectory(pre)
    dt = time.perf_counter() - t0
    plen = 0.0
    valid = None
    if res['success'] and res['path']:
        for a, b in zip(res['path'][:-1], res['path'][1:]):
            plen += math.dist(a[0], b[0])
        full = tr.build_full_path(res['path'], pre)
        rawc = [(o['center'], o['radius']) for o in scen['obstacles'] if o['type'] == 'circle']
        rawp = [o['polygon'] for o in scen['obstacles'] if o['type'] == 'polygon']
        valid = pv.path_is_valid(
            full, pre['circle_obstacles'], pre['polygon_obstacles'],
            config.R, config.ALPHA_MAX_RAD, config.L0, config.DSS,
            raw_circle_obstacles=rawc, raw_polygon_obstacles=rawp)
    return res['success'], res['stats']['iterations'], dt, plen, valid


def main():
    for enabled in (False, True):
        config.GOAL_SHOT_ENABLED = enabled
        print(f"\n=== GOAL_SHOT_ENABLED = {enabled} ===")
        for seed in SEEDS:
            ok, it, dt, plen, valid = _run(seed)
            print(f"  seed {seed}: solved={ok!s:5s} iters={it:6d} "
                  f"t={dt:5.2f}s len={plen/1000:7.1f}km oracle={valid}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2b: Run the harness and record results**

Run: `PYTHONPATH=. python scripts/goal_shot_ab.py`
Expected observations (acceptance criteria):
- With the shot ON, every flood seed (0, 1, 3) that timed out is now `solved=True` with `oracle=True` and far fewer iterations.
- Seeds that already passed OFF (2, 4, 6, 9) stay `solved=True`, `oracle=True`, and their path length does **not** regress by more than a small margin (target: equal or shorter, matching the prototype where reversals came out shorter).
- If any seed regresses in length materially, do NOT tune blindly — record it and raise `GOAL_SHOT_EVERY_N` or narrow when the shot fires as a follow-up; the coarse dedup lattice makes quality non-monotone, so any tuning MUST be re-measured on this harness.

- [ ] **Step 3: Commit**

```bash
git add scripts/goal_shot_ab.py
git commit -m "test(core): A/B harness for goal-shot on adverse seeds"
```

---

## Self-Review

**Spec coverage:**
- 2-corner geometry (turn ≤ α_max, cone arrival, ray-intersection corner, đoản-trình reserves) → Task 1. ✓
- Collision-checked splice with parent linkage → Task 2 (`_try_goal_shot`). ✓
- Fire-per-pop gating, fixed-goal-only, config knobs → Task 2 (Step 3a/3d). ✓
- No-regression + adverse A/B (validated in prototype: 6–630× fewer iters, oracle-valid, equal-or-shorter) → Task 3. ✓
- Feasibility fails (seeds 5/7/8) explicitly out of scope → Task 3 SEEDS comment. ✓

**Placeholder scan:** No TBD/TODO/"handle edge cases" — all code is concrete. ✓

**Type consistency:** `two_corner_candidates` returns `(total_len, C, d1, phi, budget_C, budget_W)` in Task 1 and is unpacked with that exact shape in Task 2 `_try_goal_shot`. `_try_goal_shot` returns a `State` consumed by the existing `_reconstruct_path`. `config.GOAL_SHOT_*` names match between Task 2 Step 3a and their uses. ✓

**Design note (inject-into-open, chosen over immediate-return):** the shot's goal state is injected into OPEN with its true `g` and accepted only when cheapest, so path quality never regresses (immediate-return regressed hard-seed 674 by ~40 km over its ceiling; inject brings it to 552.9 km, under the 556.6 km shot-off baseline). Cost: inject expands ~2–4× more than immediate-return would (still a large cut vs the flooding baseline of 14k+ iters / timeout).

**Known residual risk (documented, not a plan defect):** on the densest flood map (seed 0) inject reaches ~8.5k iterations / ~11 s, which exceeds the current `TIME_BUDGET_S = 10`. Task 3's A/B harness must check dense-seed wall-clock under inject; if a genuinely-solvable dense seed times out, raise `config.TIME_BUDGET_S` (e.g. to 15–20 s) or `GOAL_SHOT_EVERY_N` — do NOT revert to immediate-return. The shot is never a correctness regression (pure added edge).
