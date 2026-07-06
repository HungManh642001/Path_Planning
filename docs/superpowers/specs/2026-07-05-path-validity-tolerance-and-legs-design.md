# Path Validity: Unified Graze Tolerance + Start/Goal Leg Validation

**Date:** 2026-07-05
**Status:** Approved

## Problem

On a broad random-seed sample, ~24% of plans reported `success=True` are
rejected by the independent oracle (`core/path_validation.path_is_valid`).
Measured breakdown (120 seeds, production `TIME_BUDGET_S`):

- **Circle grazes into the inflation band, median 1.29 m, 0 raw-obstacle hits.**
  15/22 are interior segments grazing ~1 m; the planner accepts them
  (`CIRCLE_GRAZE_TOL_M = 1.0`) but the oracle (`tol = 1e-6`) rejects them.
  Physically meaningless: the inflation band is
  `R*(1/cos(α_max/2)−1) + SAFE_MARGIN ≈ 13.3 km`, and the raw obstacle is
  never approached.
- **Start/goal legs O→W₁ and W_{n-1}→T, up to 11.5 km circle penetration and
  polygon-interior hits.** These fixed legs are added only in
  `render/trajectory.build_full_path` and are **never collision-checked
  anywhere** in the pipeline. The large penetrations and the polygon-interior
  goal/start hits live here.
- **1 polygon-interior hit on a non-leg interior segment** — a genuine escape
  from the planner's own checks (arc-expansion chord or smoothing shortcut).

Root cause of invisibility: `hard_seeds_test.py` runs the oracle on only 4
fixed seeds, and `batch_random_test.py` never calls `path_is_valid`. This is
**pre-existing** (confirmed identical rate at the pre-arc-hop base commit),
not introduced by the arc-hop work.

## Goals

- A plan reported `success=True` has a returned path (including the fixed
  O→W₁ and W_{n-1}→T legs) that is actually flyable — no straight segment
  enters the inflated circle beyond a single documented numerical tolerance,
  and none enters a polygon interior.
- Planner and oracle agree on circle-graze tolerance via one shared constant.
- The failure mode is honest: a plan whose fixed legs are blocked returns
  `success=False` with a specific reason, not a silently-invalid path.
- No change to the pipeline's public shape beyond an additive
  `result['failure_reason']`.

## Non-goals (out of scope)

- Searching over alternative approach/takeoff headings to avoid a blocked
  fixed leg. The legs are determined by the mission spec (start/goal points,
  headings, L0/DSS); blocked = infeasible for that spec. A heading search is
  separate future work.
- Loosening polygon-interior strictness. Polygon-interior penetration is
  always a real collision, tolerance-free.
- GUI panel/slider changes beyond the one validity call-site below.

## Design

### A. Unified circle-graze tolerance

`config.CIRCLE_GRAZE_TOL_M` is the single source of truth. The oracle stays
*logically* independent (it receives a number, imports no planner logic):

- `core/path_validation._segment_clear(a, b, circles, polys, circle_tol=1e-6)`
  — rename the internal `tol` to `circle_tol`, applied to circles only;
  polygons keep the strict interior predicate (no tolerance).
- `segments_clear(..., circle_tol=1e-6)` and
  `path_is_valid(..., circle_tol=1e-6)` thread the parameter through.
- The default stays `1e-6` (strict) so any independent use is unchanged.
  Callers validating *planner output* pass `config.CIRCLE_GRAZE_TOL_M`:
  `gui/summary.py` (the `path_is_valid` call at line 42), the new
  self-validation, and the new regression test.

**Value of the constant:** set empirically in the plan — sweep a clean sample
(plans whose path never touches a raw obstacle and whose legs are clear), take
the maximum *interior-segment* circle graze, round up with headroom (expected
a few metres). Document it as absorbing discretisation/float noise, `~0.0X%`
of the 13.3 km inflation band, never approaching the raw obstacle. Cap it far
below `SAFE_MARGIN`.

### B. Fixed-leg check (planner method)

`KinodynamicAstar._check_fixed_legs(path) -> (ok: bool, reason: str|None)`:

- Build `O = self.scenario['start_pos']`, `T = self.scenario['goal_pos']`.
- `_check_collision(O, path[0][0])` → if blocked, `(False, 'start_leg_blocked')`.
- `_check_collision(path[-1][0], T)` → if blocked, `(False, 'goal_leg_blocked')`.
- else `(True, None)`.

Reuses the existing `_check_collision` (same `CIRCLE_GRAZE_TOL_M` circle
tolerance + polygon-interior predicate), so leg and body checks share one
collision semantics — no duplicated geometry.

### C. Final self-validation in `plan_trajectory`

After search + smoothing produce `path` (the W₁..W_{n-1} waypoint list):

```
if not path or len(path) < 1:
    failure_reason = 'no_path'; success = False
else:
    legs_ok, reason = planner._check_fixed_legs(path)
    body_ok = all(planner._check_collision(path[i][0], path[i+1][0])
                  for i in range(len(path)-1))
    if legs_ok and body_ok:
        success = True; failure_reason = None
    else:
        success = False
        failure_reason = reason or 'path_self_collision'
```

`result` gains `failure_reason` (None on success; else one of
`'no_path'`, `'start_leg_blocked'`, `'goal_leg_blocked'`,
`'path_self_collision'`). Body check runs `_check_collision` over the
**final** expanded+smoothed waypoints, catching arc-expansion chords and
smoothing shortcuts that were never verified in that exact form during search.

Existing consumers read only `result['success']`/`result['path']` and remain
compatible; `failure_reason` is additive.

**Ordering dependency (critical):** the body self-check uses the same
`CIRCLE_GRAZE_TOL_M` as everything else, so the constant (A) must be measured
and raised to cover the true numerical graze of legitimate arc-expansion
chords **before** the body check is enabled — otherwise valid ~1.29 m arc
grazes flip to `success=False` en masse, far beyond the intended blocked-leg
cases. The plan must sequence A before C, and the regression sweep must
confirm the success drop is confined to genuinely blocked-leg / real-collision
plans, not legitimate arc paths.

**Consequence (accepted):** the 1000-seed success rate will *drop* — plans
with blocked fixed legs (~4-5%) flip from false-success to honest failure.
The regression step records the new rate and the `failure_reason`
distribution as the truthful baseline.

## Data flow (unchanged except the new step)

`search()` → `smooth_path()` → **self-validation (B+C)** →
set `success` / `failure_reason`. No change to search, successor generation,
smoothing, or the two pipeline dict shapes.

## Testing (`tests/*_test.py`, oracle = `core/path_validation.py`)

1. **Oracle tolerance (`tests/path_validation_test.py`, new):** a segment
   grazing a circle by `circle_tol − ε` passes, by `circle_tol + ε` fails;
   default `1e-6` still strict; passing `config.CIRCLE_GRAZE_TOL_M` forgives a
   sub-metre graze. Polygon-interior penetration fails for *any* `circle_tol`.
2. **Fixed legs (`tests/kinodynamic_arc_hop_test.py`, append):** a synthetic
   circle straddling the `W_{n-1}→T` leg → `plan_trajectory` returns
   `success=False`, `failure_reason='goal_leg_blocked'`; same for the start
   leg; open scenario → `success=True`, `failure_reason=None`. Direct
   `_check_fixed_legs` unit test with a constructed path + blocking circle.
3. **Self-validation body:** if the observed non-leg polygon-interior case is
   reproducible deterministically, assert `success=False`,
   `failure_reason='path_self_collision'`; otherwise cover via the regression
   sweep below.
4. **Oracle-validity invariant (regression, new):** sweep ~120 seeds; every
   `success=True` plan must be `path_is_valid(full_path, ...,
   circle_tol=config.CIRCLE_GRAZE_TOL_M)` where `full_path` includes O and T.
   This is the invariant the old suite lacked; it prevents regression.
5. **Full regression:** `python -m pytest -q` green; re-run the 1000-seed
   sweep, record the new success rate and `failure_reason` histogram.

## Files touched

- `core/path_validation.py` — thread `circle_tol` (A).
- `core/kinodynamic_astar.py` — `_check_fixed_legs`, self-validation in
  `plan_trajectory`, `failure_reason` (B, C).
- `config.py` — set `CIRCLE_GRAZE_TOL_M` to the measured value, document it.
- `gui/summary.py` — pass `circle_tol=config.CIRCLE_GRAZE_TOL_M` (line 42);
  optionally validate the full O..T path for consistency with the new invariant.
- `tests/path_validation_test.py` (new), `tests/kinodynamic_arc_hop_test.py`
  (append), one regression-sweep test.
