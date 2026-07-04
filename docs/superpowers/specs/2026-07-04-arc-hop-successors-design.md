# Arc-Hop Successors: Removing WRAP_STEP_M Sensitivity

**Date:** 2026-07-04
**Status:** Approved

## Problem

The planner wraps around circular obstacles by chaining straight `WRAP_STEP_M`
steps off the inflated boundary. The step size determines which lattice cells
(1000 m × 3°) the wrap points land in, which determines which states survive
de-duplication, which determines graph connectivity. Two step values
(10000 m vs 5000 m) therefore produce *different search graphs* and A\* commits
to different homotopy classes — observed path-length differences of 30–40 %
on the 15 hard seeds (e.g. seed 125: 580 km vs 762 km; seed 981: 815 km with a
full self-loop vs 581 km). Both runs finished under budget, so each result is
"optimal" over its own step-induced graph: the defect is in successor
generation, not in the search.

Two aggravating defects:

1. **Strategy B gating is inverted vs. its intent.** The radial fan fires at
   every expansion where the goal is line-of-sight visible (most open-water
   states), exploding the branching factor and injecting short zig-zag
   successors (`kinodynamic_astar.py`, `strategy_b` block). The
   `NUM_STRATEGY_B` budget is consumed in the *blocked* case instead.
2. **`smooth_path` is currently disabled** (debug-only change), so wrap chains
   and fan zig-zags survive into the final path.

## Goals

- Path quality independent of any wrap discretisation parameter
  (`WRAP_STEP_M` deleted, nothing replaces its role in *search*).
- Planning time within 2–5 s per scenario (expected to improve: smaller
  branching factor, no long wrap chains in the open set).
- Keep the existing architecture: dynamic successor generation in
  `KinodynamicAstar.get_next_states`, the two pipeline dict shapes, the
  waypoint-list output contract consumed by `render/` and `path_validation`.

## Design

### 1. Boundary-riding detection

A state (P, h) is "riding" circle C (center c, radius r) when
`| |P−c| − r | < tol` (tol = 1.0 m, as in the existing `_on_circle_boundary`)
**and** h is tangent to C at P (|dot(ĥ, û(P−c))| below a small angular
tolerance, e.g. sin 0.5°).
Wrap direction `s = sign( cross(P−c, ĥ) )` (+1 CCW, −1 CW). No new State
fields; s is derived. A point riding several circles generates arc-hops for
each.

### 2. Arc-hop successors (replaces the wrap step)

From a riding state (P, h) on C with direction s:

- **To another circle D:** compute the 4 bitangent lines C↔D (2 outer,
  2 inner; inner ones exist only for disjoint circles — closed-form). Keep
  those whose departure at C is direction-consistent with s. Each gives a
  departure point `dep` on C (and an arrival point on D, unused here).
- **To a polygon hull vertex or the goal X:** `circle_tangent_points(X, c, r)`
  gives two tangent points; keep the one where the leave direction at `dep`
  (wrap sense s) points from `dep` toward X, i.e. `dot(X − dep, v̂(dep, s)) > 0`.

The emitted successor is **`dep` itself**, heading = tangent direction at
`dep` (sense s), cost = arc length `r·Δφ` where Δφ is measured from P to `dep`
in direction s, Δφ ∈ (0, 2π). The straight segment `dep → X` is *not* part of
the transition: the next A\* expansion finds X through the existing Strategy A
with a zero turn at `dep`. This reuses all Strategy A validation and needs no
"via" metadata for the segment.

**Arc clearance:** sample the arc P→dep at radius `r / cos(θ_out/2)`
(covering the outward bulge of the later circumscribed-polygon expansion),
skip C itself, test samples against other circles (point-in-circle) and
polygons (STRtree + interior predicate on sample chords).

**Marking for reconstruction:** the `dep` State carries
`arc_from = (center, r, P_start, s)` so path reconstruction knows this
transition is an arc, not a chord (a chord would cut through C).

**Kinodynamic feasibility is automatic:** circumnavigating C as a
circumscribed polygon with per-vertex turn θ needs segment length
≥ `R·tan(θ/2)`; actual segments are `≥ r·tan(θ/2)` and inflation guarantees
r > R (r = raw + R(1/cos(α_max/2)−1) + SAFE_MARGIN), so the đoản-trình
constraint holds by construction.

### 3. Arc → waypoint expansion (output only, in `_reconstruct_path`)

When a state has `arc_from`, insert circumscribed-polygon vertices between
P_start and dep: split Δφ into `n = ceil(Δφ / θ_out)` equal steps,
`θ_out = config.ARC_WAYPOINT_STEP_DEG = 30°`. Vertex k sits at
`c + (r / cos(θ_step/2)) · û(φ_k + θ_step/2)`; headings advance by s·θ_step.
Every turn is θ_step ≤ 30° < α_max. The radius-R fillet at each vertex is
inscribed in the wedge between the vertex and C (two circles inscribed in the
same wedge, R < r ⇒ disjoint), so the flown path never touches C.

This constant affects *rendering granularity only* — search connectivity and
the chosen route are independent of it (verified by testing item 3 below).

### 4. Accompanying fixes

- **Strategy B:** `strategy_b = not successors` — pure fallback, as the
  original architecture intended. Delete the goal-visibility trigger and the
  `NUM_STRATEGY_B` budget.
- **Smoothing:** re-enable `planner.smooth_path(path)` in `plan_trajectory`.
  Safety: a shortcut across an arc's waypoint chain penetrates the inflated
  circle deeper than `CIRCLE_GRAZE_TOL_M`, so `_check_collision` rejects it —
  smoothing cannot destroy arc chains.
- **Config:** delete `WRAP_STEP_M`, `NUM_STRATEGY_B`; add
  `ARC_WAYPOINT_STEP_DEG = 30.0`. Keep `CIRCLE_GRAZE_TOL_M` (tangent segments
  still graze the boundary numerically).
- Delete the wrap-step successor block; keep boundary detection (now feeds
  arc-hop).

## Testing (`tests/*_test.py`, oracle = `core/path_validation.py`)

1. **Geometry units:** bitangent computation (outer/inner, degenerate
   overlap), direction filtering against wrap sense, departure-point selection
   from an external point, arc expansion (all turns ≤ θ_out, all waypoints
   outside C, đoản trình holds), arc-clearance sampling.
2. **Hard-seed integration (15 seeds):** `path_is_valid` true; planning time
   < 5 s; per-seed distance ≤ min(v1, v2 baseline) + tolerance; no
   self-intersecting loops (982) or start zig-zags (126).
3. **Discretisation invariance:** sweep `ARC_WAYPOINT_STEP_DEG` ∈
   {20, 30, 45} → identical search-state sequence and route, only waypoint
   density differs. This is the proof that the root cause is removed.
4. **Full regression:** 1000-seed batch; success rate ≥ `results1` baseline;
   distance distribution not worse.

## Out of scope (behavior unchanged)

- W₁ or W_{n−1} lying *inside* an inflated circle (no tangents exist; planner
  may fail today and still may).
- Polygon-hugging (already handled by hull vertices + boundary-touch
  predicate).
- GUI and rendering code besides what `_reconstruct_path` emits.
