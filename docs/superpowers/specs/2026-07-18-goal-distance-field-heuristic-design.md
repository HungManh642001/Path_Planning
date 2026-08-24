# Goal-Distance-Field Heuristic: Admissible Obstacle-Aware h()

**Date:** 2026-07-18
**Status:** Approved

## Problem

`KinodynamicAstar.heuristic()` is plain Euclidean distance to
`goal_state.waypoint`. It is admissible and consistent, but blind to
obstacles and to the safezone: every state "behind" an island or outside the
practical corridor is priced as if the goal were reachable in a straight
line. On occluded scenarios the search floods the Euclid-optimistic basin
before committing to the real detour — seed 4 spends 1739 iterations (~5 s)
where the true mission is 26 % longer than the straight line; seeds 92/964
show the same signature. Weighted A\* (`HEURISTIC_WEIGHT > 1`) buys speed
but was rejected: it forfeits the optimality guarantee.

Production constraint: there is no fixed map rectangle. Real missions supply
a `safezones` polygon (union) that start and goal lie inside; the world is
otherwise unbounded (`_in_bounds` is permissive when neither `safezones` nor
`map_bounds` is given).

## Goal

Replace h with a **provably admissible, tighter** lower bound that:

- prices obstacle detours AND the safezone corridor into h,
- costs O(1) per state and ~100 ms preprocessing only on scenarios that
  need it (zero overhead on open-water scenarios),
- degrades gracefully to exactly the current Euclid behaviour on any
  failure or out-of-coverage query,
- keeps the exact-optimality contract: acceptance is gated on a 1000-seed
  A/B (success count not lower, length distribution not worse, any seed
  > +5 km individually investigated) plus a runtime admissibility witness
  (h(start corner) ≤ found mission body cost on every solved seed).

Chosen after comparing three approaches (exact tangent-graph Dijkstra;
ring-sampled vertex graph; coarse grid field): **grid field** wins because
its preprocessing is independent of obstacle count, it prices the safezone
shape for free (often the dominant constraint in corridor missions), and it
needs no fixed world bound — the safezone bbox is the natural extent.

## Design

### New module `core/heuristic_field.py` — class `GoalDistanceField`

Built once per planner instance from the preprocessed scenario (inflated
`circle_obstacles`, inflated `polygon_obstacles`, raw `safezones`,
`map_bounds`, goal waypoint). Pure numpy + scipy (both already
dependencies); no new requirements.

**Grid extent** (first match wins):

1. bbox of the safezone union, padded by one cell;
2. explicit `map_bounds` rectangle `[0,w]×[0,h]`;
3. permissive mode: bbox of obstacles ∪ start ∪ goal, padded by 10 % of
   its diagonal (min 2 cells) — border seeding keeps the bound sound
   regardless of how much of the world the pad captures.

Resolution `config.HEURISTIC_GRID_N = 256` cells on the long side
(cell ≈ 2 km on the 500 km dev map; scales with mission size).

**Cell blocking — always under-blocked so distances can only shrink
(lower-bound-safe):**

- obstacle-blocked ⟺ cell center inside the obstacle *eroded* by the cell
  half-diagonal (circles: numpy distance test against `r − halfdiag`;
  polygons: `shapely.buffer(−halfdiag)` + vectorised `contains_xy`);
- safezone-blocked ⟺ cell center outside the safezone *dilated* by the
  half-diagonal (only when safezones exist).

**Distance solve:** 8-connected Dijkstra
(`scipy.sparse.csgraph.dijkstra`, adjacency assembled in numpy) with edge
weights `cell` / `cell·√2`. Sources:

- the goal cell at 0, and
- **every border cell of the grid at `Euclid(cell center, goal)`** — this
  keeps the bound sound in permissive/unbounded mode: a true path that
  leaves the gridded area re-enters the estimate through a border seed
  whose value is itself a universal lower bound.

**Query** (`query(p) -> float`, O(1)):

```
d̂(c)     = d_grid(c) / 1.0824  −  2·cell          # 8-connectivity stretch + digitisation slack
query(p) = max over the 4 surrounding unblocked cell centers c of ( d̂(c) − |p − c| )
```

No bilinear interpolation (interpolating lower bounds is not a lower
bound). Each term is reverse-triangle-sound; max of sound bounds is sound
and tight. All four cells blocked, or p outside the grid ⇒ `−inf` (the
caller's `max` with Euclid takes over).

### Hook in `core/kinodynamic_astar.py`

```python
def heuristic(self, state, goal_state):
    h = <euclid, unchanged>
    if self._goal_field is not None:
        h = max(h, self._goal_field.query(state.waypoint))
    return h
```

`__init__` builds the field **only when every surviving start corner's
straight chord to `goal_wp` is collision-blocked** (LOS test with the
existing `_check_collision`): open scenarios keep literally zero overhead.
The build is wrapped in `try/except Exception` → on any failure
`_goal_field = None` and planning proceeds on pure Euclid. The heuristic
must never be able to fail a plan.

Free-goal mode needs nothing special: the field is rooted at
`goal_state.waypoint`, which is `T` itself in that mode.

### What does NOT change

Successor generation, đoản-trình, collision checking, goal acceptance,
smoothing, cost function — untouched. Only the expansion order moves, so
path changes across seeds are tie-break effects among (near-)equal-cost
routes, bounded by the same admissibility guarantee as today.

## Admissibility argument (summary)

For any state p with true remaining mission body cost `C(p)`:
the flown path is a collision-free continuous curve inside the safezone, so
`C(p) ≥ d_cont(p)`, the safezone-constrained continuous shortest distance.
The grid free space is a superset of the true free space (under-blocking,
dilation) and border seeds are universal lower bounds, so the grid models a
relaxation; 8-connected grid paths overestimate continuous length by at
most ×1.0824 away from cell-scale narrow passages, absorbed together with
digitisation effects by the 2-cell slack. Finally
`d̂(c) − |p−c| ≤ d_cont(c) − |p−c| ≤ d_cont(p)`. The residual risk
(passages narrower than a cell) is monitored empirically: the A/B harness
asserts `h(corner) ≤ found cost` on all 1000 seeds; a violation triggers
larger slack / finer grid, data-driven.

## Testing

New `tests/heuristic_field_test.py` (TDD, red first):

1. **Analytic admissibility**: single mid-map circle — `query(p)` ≤ exact
   tangent-arc-tangent distance on a sample grid of p; diagonal-corridor
   safezone — `query(p)` respects the corridor (and ≥ Euclid where the
   corridor forces a detour, proving added value).
2. **Oracle witness**: for the 11 studied seeds, plan and assert
   `heuristic(corner) ≤ mission body cost` of the returned path; mission
   length within +5 km of the recorded Euclid baseline per seed (the same
   threshold the 1000-seed A/B uses for "investigate").
3. **Gating**: open-water scenario ⇒ `_goal_field is None`; occluded
   scenario (seed 4) ⇒ field built.
4. **Fallback**: monkeypatched build raising ⇒ plan still succeeds,
   `_goal_field is None`.
5. **Speed guard (loose)**: seed 4 iterations strictly below the Euclid
   count (1739) by a comfortable margin.

Acceptance: full pytest (no new failures vs the recorded 18 pre-existing
reds) + the 1000-seed A/B protocol above, with a wall-time report.

## Rollback

One new module + a ~10-line hook; revert restores byte-identical current
behaviour. No config knob is exposed beyond `HEURISTIC_GRID_N`.
