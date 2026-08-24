# Obstacle inflation: drop the turn term, keep only SAFE_MARGIN

Date: 2026-08-08

## Problem

`preprocessing.inflate_obstacles` inflates every obstacle by

```
R * (1/cos(alpha_max/2) - 1) + SAFE_MARGIN
```

At `R = 8000 m`, `alpha_max = 90 deg` the turn term alone is **3313.7 m**, applied
uniformly to every obstacle regardless of how sharply the path actually turns
near it — a straight transit past an island pays the same 3.3 km as a worst-case
90-degree corner.

Measured over 56 scenarios (16 presets + 40 seeds), counting **all** obstacle
pairs with a positive gap (circles *and* polygon islands, 1536 pairs): the median
gap is 6751 m and the inflation closes every gap below `2 x 3313.7 = 6627 m`, so

> **48.96 % of all corridors between obstacle pairs are closed.**

A closed pair is not a detour. Both inflated shapes overlap, the inner bitangent
stops existing (`k = (r1+r2)/d > 1` in `core/arc_geometry.py`), and that route
disappears from the search graph entirely. This also shows up as
`start_leg_blocked`, which rises from 4 to 8 scenarios as inflation grows.

## What the turn term is actually for

The search plans on a **polyline**; the aircraft flies **fillet arcs** that cut
each corner. The arc leaves the checked polyline, and inflation exists to cover
exactly that excursion. Two constants describe it, and they answer different
questions (both verified by brute force over 20 000 arc samples):

| question | constant | at alpha=90 |
|---|---|---|
| how far does the arc stray from the **leg polyline**? | `R(1-cos(a/2))` | 2343.1 m |
| how far must a **waypoint** sit from a corner it wraps? | `R(sec(a/2)-1)` | 3313.7 m |

They are linked by the mitre join: buffering a polygon by `d` displaces a vertex
by `d/sin(phi/2)`. The current code uses the *vertex* constant as the *buffer*
amount, so it over-inflates by `sec(alpha_max/2) = 1.414x`.

But the deeper finding is that **the turn term is not needed at all** once
`SAFE_MARGIN` is a real, non-zero number: a waypoint constructed `SAFE_MARGIN`
off the raw obstacle already gives the arc that much room to bulge into, and
`_corner_arc_clear` (shipped in 68f99af) rejects the corners where it is not
enough. Inflation was doing correctness work that an exact check already does.

## Evidence

Prototype in scratchpad, 25 maps (20 islands + 20 circles), free-goal,
goal shot and heuristic field off, 90 s budget. Every arm verifies its effective
inflation from the prepared scenario rather than assuming it.

| arm | SAFE_MARGIN | success | mean length | clearance (median) | wall clock |
|---|---|---|---|---|---|
| A — turn term (today) | 500 | 23/25 | 406.4 km | 3814.7 m | 146 s |
| B — SAFE_MARGIN only | 500 | **24/25** | **397.4 km** | 501.0 m | **77 s** |
| A — turn term (today) | 0 | 23/25 | 404.6 km | 3314.7 m | 163 s |
| B — SAFE_MARGIN only | 0 | 19/25 | 399.3 km | 1.0 m | 73 s |
| Bg — B + smoother guard | 0 | **24/25** | 397.8 km | 1.0 m | 82 s |

Dropping the turn term wins on every axis: one more scenario solved, ~9 km
shorter paths, and roughly **half the wall clock**. The clearance becomes the
number the operator chose instead of a 3.3 km accident of the turn geometry.

All five `oracle_reject` failures in arm B at `SAFE_MARGIN = 0` came from
`_smooth_greedy`, not from the geometry under test: the shortcutter invents a new
turn at a waypoint sitting on the obstacle boundary and never arc-checks it.
Arm A never sees this because 3.3 km of inflation proves those arcs for it.
Adding the guard removes all five.

## Design

### 1. `inflate_obstacles`: drop the turn term

```python
inflation = safe_margin
```

`SAFE_MARGIN` regains its documented meaning: the minimum distance the flight
path must keep from an obstacle. It is a mission parameter, not a geometric
correction.

`spatial_utils.inflate_polygon` must short-circuit when `inflation <= 0`:
shapely's `buffer(0)` is a *cleaning* operation, not a no-op — a self-touching
polygon splits into a MultiPolygon and the current code silently keeps the
largest piece, shrinking the obstacle.

### 2. Arcs must honour `SAFE_MARGIN` too

`_corner_arc_clear` and `path_validation.path_is_valid` currently check turn arcs
against the **raw** obstacles while straight legs get the inflated set. That
asymmetry existed only because arcs were designed to bulge into the turn-term
band. With no turn term there is no band, and the asymmetry lets an arc dip
inside the operator's minimum clearance — measured: with `SAFE_MARGIN = 500`, the
worst flown clearance was **97.9 m**.

Both must check arcs against the inflated set (`raw + SAFE_MARGIN`), i.e. stop
passing the `raw_*` override.

### 3. `smooth_path`: guard the shortcut's arcs

`_smooth_greedy` validates each shortcut chord with `_check_collision` and
`validate_kinodynamics`, but never `_corner_arc_clear`. Extend the existing
end-of-`smooth_path` guard — which already falls back to the un-smoothed search
path when the min-straight coupling is violated — to also reject on arc
clearance. The search path is valid edge by edge by construction, so the
fallback is always safe.

### 4. `_try_goal_shot`: arc-check the corners it synthesises

The analytic goal shot builds a two-corner manoeuvre and injects it straight into
OPEN, bypassing `get_next_states` — so nothing arc-checks the corner at
`current`, the corner at `C`, or the terminal turn at `gw` onto `goal_heading`.
Its docstring claimed the manoeuvre is "validated identically to any search
edge"; that stopped being true when 68f99af added the arc check to
`get_next_states` only.

Harmless at full inflation, which proves those fillets for free. With inflation
reduced to `SAFE_MARGIN` it is a real defect: seed 964 fails
`path_self_collision` without this fix and succeeds with it.

### 5. Documentation and display

- `inflation_offsets` returns two rings built on the turn term; with the term
  gone there is one ring. Update it and `gui/map_canvas.py`.
- The comments at `preprocessing.py` (`prepare_scenario`) and
  `path_validation.py` (`path_is_valid`) describe the asymmetric model; update.

## Explicitly out of scope

- **Per-edge waypoint repair** (offset `P` outward when its arc fails, and route
  the successor through `P'`). Prototyped and **measured to be net negative**:
  inert at `SAFE_MARGIN = 500` (899 repairs, 1 reaching a path) and actively
  harmful at 0 (22/25 with it vs 24/25 without), because `g_cost`,
  `straight_budget` and the successor's turn were all computed for `P`, leaving
  the path through `P'` unvalidated end to end.
- **A reduced `alpha_ref`** hybrid inflation. Unnecessary once `SAFE_MARGIN`
  carries a real value, and it would reintroduce a constant tuned to a test set.
- **The arc-expansion vertices** synthesised by `_reconstruct_path` when an
  arc-hop transition is expanded into circumscribed-polygon waypoints. They are
  built at reconstruction time and so never reach `_corner_arc_clear` — the same
  class of defect as the goal shot, still latent. No test in the current suite
  exercises it; tracked separately.

## Risks

1. **Search cost in the shipped configuration is unmeasured.** All numbers above
   come from free-goal runs with goal shot and heuristic field off. An earlier
   dense-map run with the shipped settings and a 15 s budget scored 28/56 at zero
   inflation versus 38/56 with it, `no_path` being `TIME_BUDGET_S` exhaustion
   rather than starvation. This must be re-measured before the change is trusted
   in production settings.
2. **Change 2 is stricter than anything measured.** Arcs moving from `raw` to
   `raw + SAFE_MARGIN` can only reject more. Verify it does not lose scenarios.
3. **Boundary-hugging waypoints.** Strategy A aims at hull vertices and tangent
   points, which sit on the constructed boundary by definition. With
   `SAFE_MARGIN = 0` they sit on the raw obstacle and 13 % of their corners get
   rejected by the arc check. This is a reason to run a positive `SAFE_MARGIN`,
   not a defect.

## Verification (results)

**Test suite, A/B on the identical working tree** (same WIP, same scenario
generator — only the seven core/gui files swapped):

| arm | failed | passed |
|---|---|---|
| HEAD core | 38 | 149 |
| this change | **37** | **150** |

No test fails under this change that passes at HEAD. One recovers
(`strategy_b_valve_test.py::test_corner_expansions_do_not_consume_valve_budget`).
The 37 remaining failures are pre-existing: gitignored tests for features absent
from this branch, plus stale `BASELINE_KM` values in `heuristic_field_test.py`.

**Shipped configuration** (fixed goal, goal shot ON, heuristic field ON, no time
budget), 12 hard seeds: **9/12 both before and after**. Seed 964 regresses to
`path_self_collision` without design item 4 and recovers with it. Seeds 4, 92 and
123 fail on both arms; two of them change reason (`start_leg_blocked` becomes
`goal_leg_blocked` / `no_path`), which is the expected consequence of a smaller
inflation making the takeoff corners feasible and pushing the failure downstream.

**Clearance semantics**: on `scenario4_complex_maze` the flown minimum clearance
to the raw obstacles is 501.0 m at `SAFE_MARGIN = 500` (97.9 m before design
item 2, i.e. arcs were dipping inside the operator's margin).

**Methodology note.** Two intermediate reports in this study wrongly claimed a
regression, both times because the comparison baseline differed from the test
tree in something other than the change under test — first a pristine worktree
missing the WIP, then a scenario generator (`batch_random_test`) whose WIP edits
change which map each seed produces, invalidating the hardcoded `BASELINE_KM`.
Any future A/B here must swap only the files under test inside one tree, and any
inflation experiment must verify the effective inflation from a prepared scenario
(`circle_obstacles[i].r - raw_circle_obstacles[i].r`) rather than assume it.
