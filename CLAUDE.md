# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single-process Python research codebase for planning autonomous aircraft trajectories. The planner combines a **Kinodynamic A\*** search over `(position, heading)` states with a **tangent (bitangent) graph** for long-range connectivity, then renders the result with matplotlib. Everything runs locally; there is no server, package, or external service.

## Commands

```bash
pip install -r requirements.txt          # numpy, scipy, shapely, matplotlib (+ tkinter for the GUI)

python main.py                           # Batch test harness: runs all 16 scenarios, writes PNGs to results/
python launch_gui.py                     # Interactive Tk GUI: click to place start/goal/obstacles, then plan
python performance_eval.py               # Performance metric helpers (also imported by main.py)
```

There **is** a pytest suite under `tests/` — run `python -m pytest -q` from the repo root (`pytest.ini` sets `pythonpath = .` so tests can `import config` / `import core.* / render.* / gui.*`, and `testpaths = tests`). Test files are named `*_test.py` (these ARE committed); `test_*.py` is gitignored, so name scratch tests that way to keep them out of git. `main.py` is a separate batch harness that runs all scenarios and writes PNGs.

To run/debug a single scenario instead of all 16, call the pieces directly (this is the canonical pipeline):

```python
import core.map_generator as mg, core.preprocessing as prep
import core.kinodynamic_astar as astar, render.visualizer as viz
scenario = mg.scenario4_complex_maze()        # any function in mg.get_all_scenarios()
pre = prep.prepare_scenario(scenario)
result = astar.plan_trajectory(pre, verbose=True)
if result['success']: viz.plot_scenario(scenario, pre, result, save_path="out.png")
```

`main.py` forces the matplotlib `Agg` backend for headless rendering; the GUI uses the interactive Tk backend.

## Architecture / data flow

The modules form a strict one-directional pipeline, grouped into packages.
`config.py` (top-level, imported everywhere as `import config`) holds the
constants; nothing imports `main.py`/`launch_gui.py`.

```
config.py                 tactical constants + deg/rad helpers (R, ALPHA_MAX, L0, DSS, SAFE_MARGIN, map bounds)
core/                     planning pipeline
  → map_generator         builds a "scenario" dict (start/goal + obstacles); 16 predefined + create_scenario()
  → preprocessing         prepare_scenario(): inflates obstacles, computes start/goal *waypoint* states
  → kinodynamic_astar     plan_trajectory(): A* over (waypoint, heading); returns path + stats
  → path_validation       independent oracle: segments/arcs clear, turn angles ok (used by tests + GUI summary)
  → spatial_utils         geometry helpers (distance, headings, polygon inflation, tangent points)
render/                   drawing (consumes the planner path)
  → trajectory            sample_trajectory(path, R, mode): straight or fillet-arc flight path + turn_markers()
  → visualizer            plot_scenario(...) for the batch harness
gui/                      interactive Tk app (config | map | results)
```

### Scenario generation (core/map_generator.py) — two rules that were missing

Every scenario, including the 16 named presets, builds its obstacles through
`generate_random_islands` / `generate_dynamic_obstacles` (seeded); only the
start/goal and counts are hand-set. Two constraints are enforced there and are
easy to reintroduce as bugs:

- **Same-type obstacles must not overlap.** Circles always had a separation
  test; islands had none, so they overlapped freely (measured over 200
  scenarios: 183 with overlapping pairs, median 7, max 62) and **21.2% of
  polygon hull vertices sat buried inside another polygon** — candidates the
  search re-tests and re-rejects on every expansion. Now gated by
  `ISLAND_MIN_SEPARATION_M`. Cross-type overlap (island vs circle) is still
  allowed and is common (100/100 scenarios).
- **Start and goal need real clearance.** The buffer was `config.EPS`, i.e.
  1e-6 m, which permitted an obstacle to touch the start point; 16% of scenarios
  put start or goal closer than `L0` to an obstacle, so the mandatory takeoff or
  seeker leg was born blocked. Now `SPAWN_CLEARANCE_M`. This is where most
  `start_leg_blocked` / `goal_leg_blocked` results came from — they were
  **generator artifacts, not hard scenarios**: fixing it took free-goal
  `start_leg_blocked` 2→0 and adverse `goal_leg_blocked` 10→7.

Consequence to remember: any change here **moves every random benchmark**, so
numbers are only comparable within one generator version (see also the note
that `batch_random_test` drifts).

### The two dict shapes that flow through the pipeline

- **scenario** (from `map_generator`): `start`, `start_heading`, `goal`, `goal_heading`, `islands` (list of polygons), `dynamic_obstacles` (list of `(center, radius)`), and a unified `obstacles` list where each item is `{'type': 'polygon', 'polygon': [...]}` or `{'type': 'circle', 'center', 'radius'}`.
- **preprocessed** (from `preprocessing.prepare_scenario`): adds `start_state`/`goal_state` (each a dict with `waypoint` + `heading`), `turn_radius`, `alpha_max_rad`, and the *inflated* obstacles split into `circle_obstacles` (list of `(center, radius)`) and `polygon_obstacles` (list of coord lists). The A\* planner consumes only the preprocessed dict.

### Conventions (important — easy to get wrong)

- **Units are meters; angles are radians** throughout the algorithm code. `config.ALPHA_MAX`/`START_ANGLE_*` are stored in **degrees** and converted (`config.ALPHA_MAX_RAD`, `config.deg_to_rad`). The map is 500 km × 500 km, `R = 8000 m`.
- A planner **state** is the tuple `(waypoint, heading)` where `waypoint = (x, y)`. Paths are lists of these tuples. `spatial_utils.state_to_tuple` is used for hashing/dedup.
- **Obstacle inflation** is not just `R + margin`. `preprocessing.inflate_obstacles` uses `R * (1/cos(α_max/2) - 1) + SAFE_MARGIN` so a turn at max angle still clears the obstacle. Start/goal waypoints (`W₁`, `W_{n-1}`) are offset from the raw start/goal points by `L0`/`DSS` plus a turn-radius term — the planner never searches to the literal goal.
- **Operating-area bounds** (`_in_bounds`): a `safezones` polygon ⇒ point must be `covers`-ed by it; else an **explicit** `map_bounds` ⇒ the `[0,w]×[0,h]` rectangle; else (neither given) ⇒ **permissive** (returns True). The old code always fell back to the `config.MAP_WIDTH/HEIGHT` 500 km box, which silently rejected every waypoint of scenarios living outside it (e.g. real missions at `y≈1.15e6` that pass no bounds) — surfaced once start corners started filtering on `_in_bounds`. NOTE the two easy scenario-key mistakes that make a scenario effectively unbounded: the planner reads `safezones` (plural) and `map_bounds` (a `(w,h)` tuple) — `safezone`/`map_width`/`map_height` are ignored.
- **Seeded start corners** (replaces the single worst-case `W₁`): the planner does NOT root the search at one `W₁ = O + (L0 + R·tan(α_max/2))·û`. Instead `KinodynamicAstar.__init__` seeds `config.NUM_START_CORNERS` (K=8) corner states on the takeoff ray at `d_i = L0 + R·tan(αᵢ/2)`, tan-uniform buckets `tan(αᵢ/2) = (i/K)·tan(α_max/2)`, i=1..K (bucket K == the legacy `W₁`, so `NUM_START_CORNERS=1` is exactly legacy — the A/B knob). A corner seeded for `αᵢ` affords any first turn `α ≤ αᵢ` while keeping the takeoff straight `l₁ ≥ L0` **exactly** (via `State.straight_budget` + `State.min_straight_in = L0`, checked in `_doan_trinh`). Corners outside the operating area or with a colliding `O→corner` leg are not seeded (feasibility recovery — the single legacy `W₁` could land inside an inflated obstacle / outside a safezone corridor and kill the whole plan; e.g. a diagonal-corridor safezone with an adverse takeoff heading). If none survive, the start is blocked (fast honest failure). `smooth_path` re-guards `l₁ ≥ L0` at the first anchor since a shortcut can change `α₁`. NOTE: the terminal `W_{n-1}` still uses the α_max reserve (fixed mode) — that's a planned Phase-2; free-goal mode already targets `T` with a real-angle usable run-in ≥ DSS.

- **`core/kinodynamic_astar_v0.py`** (gitignored, may not exist in a fresh
  clone) is a **readability-first** variant of the planner kept in sync by hand:
  it keeps the simpler `WRAP_STEP_M` straight-continuation instead of
  `_arc_hop_successors`, and reuses `path_validation` / `spatial_utils` helpers
  instead of the main file's hand-inlined hot loops. Nothing imports it. When
  porting a mechanism into it, **keep docstrings short** — one or two lines of
  *what*, not the main file's long *why* essays — that brevity is the point of
  the file.
- **Every tunable lives in `config.py`, never as a literal in the algorithm.**
  One constant per *meaning*, even when two happen to share a value: e.g. the
  đoản-trình floor `MIN_STRAIGHT_M` and the fan's float-noise pad
  `RADIAL_FAN_STEP_M` are separate knobs on purpose. `kinodynamic_astar.py`'s
  module-level `_MIN_STRAIGHT_M` is an alias of `config.MIN_STRAIGHT_M` (hot
  path), not a second definition — do not re-inline the number. Only degenerate
  -geometry guards (`< 1e-9` collinearity / right-angle tests) stay inline;
  they are not tunables.

### How the A\* search actually works (core/kinodynamic_astar.py)

`get_next_states` generates successors dynamically (no precomputed graph):
(1) **arc-hop** — from a state riding an inflated circle's boundary (tangent
arrival), hop along the boundary arc to each tangent-continuous departure
point (bitangents to other circles, tangents from polygon vertices / the
goal) at true arc-length cost; the arc is expanded into circumscribed-polygon
waypoints (`config.ARC_WAYPOINT_STEP_DEG`) only at path reconstruction, so
search connectivity has no wrap discretisation parameter; (2) **Strategy A**
— tangent points to each circle (`spatial_utils.circle_tangent_points`) +
polygon hull vertices + the goal, each accepted only with a valid turn and a
clear segment; (3) **Strategy B** — an `±α_max` radial fan that fires when
(a) no successor exists (pure fallback), (b) the state is riding a circle
boundary (leave-the-boundary options between departure points), or (c) the
goal is line-of-sight blocked, budgeted globally by `config.NUM_STRATEGY_B`
(an escape valve against long detours from an adverse initial heading). Note
`NUM_STRATEGY_B` gates only the **budget**, not whether the fan fires: gating
the firing itself on "the goal is already reachable" costs seed 4 88 km
(534.9 vs 446.9), because the dedup lattice (`STATE_POS_QUANTUM`,
`STATE_HEADING_QUANTUM_DEG`) makes the search only approximately optimal, so
the fan's redundant-looking pivots act as lattice diversity rather than noise.
Each fan direction emits `config.NUM_FAN_DISTANCES` **distance rungs**, not one
leg: rung `j` is the shortest leg still affording a next turn `β ≤ βⱼ`, with
tan-uniform buckets `tan(βⱼ/2) = (j/M)·tan(α_max/2)` — the same capability-bucket
idea as the seeded start corners (`M = 1` + the legacy 1000 m pad reproduces the
old single worst-case leg exactly). The old code hardcoded `β = α_max`, so every
fan leg paid the worst-case far reserve even when the pivot barely turns, which
bulged fan-routed paths in open water. M is **measured, not tuned by intuition**
— the relation is not monotone in M (see the note in `config.py`).
`validate_kinodynamics` enforces the max-turn-angle and minimum-straight-segment
(đoản trình) constraints. `search()` accepts the goal only when both within
`GOAL_THRESHOLD` **and** the arrival heading is within `α_max` of `goal_heading`
(so the terminal turn onto the approach is feasible). `smooth_path` shortcuts
each kept anchor to the **farthest** reachable waypoint whose direct chord is
exact-collision-free and kinodynamically valid (turn at the anchor **and** the
onward turn at the target; the terminal target uses `goal_pos`/`goal_heading`,
not the offset `goal_state.waypoint`).

**Along-ray pivot slide** (`_pivot_candidate` / `_slide_pivot`,
`config.NUM_PIVOT_SLIDES`): every Strategy-A candidate edge goes through
`_pivot_candidate(current, node, advance)`, which turns `advance` metres
further along the incoming ray (`advance = 0` is the plain corner and is
behaviour-identical to the pre-slide code — `NUM_PIVOT_SLIDES = 0` is the A/B
knob). When a candidate is rejected *by the fillet-arc gate* (`_corner_arc_clear`,
typically at a polygon hull vertex where the fillet folds into that polygon),
it is retried from pivots slid **forward**, `P' = P + d·ĥ_in`. Forward along
the ray — never along the outer bisector — because that keeps the incoming
**direction**, so the parent's corner, its turn reserve and every ancestor stay
valid and `_doan_trinh` only *gains* budget (hence its `advance` parameter);
sliding along the bisector rotates the incoming leg and forces ancestors to be
re-validated, which does not terminate. Nothing is mutated: the slide emits an
**additional** successor. The price is that with `ĥ_in` as x-axis and
`V − P = (a, b)` the resulting turn `|atan2(b, a − d)|` **grows** with `d`
(cap `d ≤ a − |b|/tan α_max`), so retry positions are parametrised by that
resulting turn in tan-uniform capability buckets — the same idiom as
`NUM_START_CORNERS` / `NUM_FAN_DISTANCES` — smallest slide first. Only the
`'arc'` rejection is retried (`self._last_reject` side-channel): sliding can
only *increase* the turn, so an already-over-α_max candidate is hopeless, and a
blocked chord is almost never unblocked by moving the pivot. The slide is
recorded in `State.via` and expanded back into a real waypoint by
`_reconstruct_path` (like `arc_from`, but `via` pivots also join `raw_route` —
they are searched waypoints, not output discretisation).

Note the smoother is the other half of that gate: a shortcut **re-forms** the
corner at the anchor, so it must be arc-checked there too. `smooth_path` is an
exact subsequence **DP** over `O..T` (state = the last two kept waypoints, plus
the straight budget left on the chord between them), because đoản trình couples
adjacent chords through the turn they share — a greedy one-chord-at-a-time scan
cannot see that dropping a waypoint retroactively steals straight length from
the chord *into* its neighbour.

**Both endpoints are constraints, not plain nodes.** No turn is available at `O`,
so the first kept chord must lie on the takeoff ray (`TAKEOFF_RAY_TOL_RAD`); and
in **fixed-goal** mode the seeker run-in must be flown along `goal_heading`, so
the last chord must lie on the approach ray (`APPROACH_RAY_TOL_RAD`). Omitting
the second one was a live bug in both planners until 2026-08-16: the DP drops
`W_{n-1}` whenever that shortens the path and arrives on the wrong heading —
measured 3/16 named scenarios (scenario_04 off by **45.5°**) and 16/28 on a
fixed-goal adverse suite (up to 61°). **The oracle cannot catch this**:
`path_validation` derives every angle from waypoint geometry and never compares
the arrival bearing against `goal_heading` — so it is asserted directly in
`tests/smooth_path_test.py`, alongside the takeoff-ray mirror. Note the
consequence for expectations: on the named presets smoothing now buys **node
reduction, not length** (14→7, 15→9 nodes, ~0 km); the old "scenario_04 saves
~10 km" figure *was* the illegal shortcut.

**The DP must always be able to reproduce its own input, and it must prefer
fewer waypoints.** Two independent defects broke that, and both surfaced as the
same symptom — a delivered path carrying waypoints the aircraft flies STRAIGHT
through (`batch_random_test` seed 34: W1, W2, W4 at turn = 0.00000°):

- **The turn gate used the construction reserve on measured geometry.** It
  compared against `self._alpha_build` (`α_max − GEOM_EPS_RAD`) while deriving
  the turn from waypoint coordinates. The search legitimately builds corners AT
  that limit, and re-deriving one from the emitted coordinates reads back as
  `_alpha_build + ~3e-15 rad` — over. That kills every continuation out of the
  corner, so `best` stays `None` and the DP hits its `return path` fallback:
  **smoothing silently does nothing at all** (measured 33 of 294 solved
  scenarios). The gate belongs at the **true** `alpha_max_rad`, because nothing
  in the DP is constructed from an angle: every corner it weighs is defined by
  waypoints that already exist, and it measures them with `path_validation`'s
  formula bit for bit, so the gate IS the oracle's check.
- **Length alone leaves the choice to iteration order.** A waypoint flown
  straight through adds exactly zero length (measured bit-identical: 28299.999971999972 m
  across three of them), so cost could not see it. `SMOOTH_NODE_PENALTY_M`
  charges each kept waypoint a metre, which makes the shortest subsequence also
  the one with the fewest waypoints, and bounds the preference at one metre per
  waypoint dropped. Measured effect on its own is small (−1 waypoint over 300
  scenarios) because insertion order already leaned that way — it converts an
  accident into a guarantee.

Both are asserted in `tests/smooth_path_dp_test.py` (a corner placed exactly on
`α_max`, followed by pass-through waypoints); both assertions fail on the code
before the fix.

**A planner stricter than its own oracle rejects flyable chords.** The third
cause of the same symptom was in `_check_collision`, not the smoother:
`relate_pattern(line, 'T********')` alone also matches a **zero-extent** touch,
so a chord grazing a hull VERTEX reads as a hit. `path_validation` has measured
the interior overlap against `POLYGON_TOUCH_TOL_M` since that was fixed for the
oracle; the planners had not. The consequence is worse than a lost chord — it
makes the collision test **non-monotone under splitting**: on seed 0, two
collinear chords are each clear while their union is "blocked" on an overlap of
**2.9e-11 m** at the shared vertex, which is exactly the chord the smoother
needs in order to drop the pass-through waypoint sitting on it. Both planners
now borrow the oracle's threshold (`_POLY_TOUCH_TOL_M`, a hot-path alias of
`pv.POLYGON_TOUCH_TOL_M` — the oracle owns the number, since it owns the
explanation, and stays free of `import config` on purpose).

Measuring the overlap costs 63 µs against 12 µs for the predicate, and 14.4% of
collision calls hit a polygon, so `config.POLYGON_DEEP_HIT_INSET_M` short-
circuits it: a chord whose interior reaches into a **shrunk** copy of the
polygon overlaps by more than the inset and is unambiguously blocked. That is a
performance gate, not a tolerance — it can only skip work on chords that are
already blocked, never forgive one — and it is behaviour-identical (bit-for-bit
equal results on 300 scenarios).

**Measure the INTERIOR overlap, not the closed-polygon one.**
`poly.intersection(line).length` is the overlap with interior **∪ boundary**, so
a chord that legitimately runs ALONG a hull edge — an explicitly allowed move —
scores its whole edge-following stretch. Measured on seed 194: **11533.475 m of
"overlap", every metre of it on the boundary and 0.0 m inside**; seed 257,
8597.245 m likewise. `path_validation.interior_overlap_length` subtracts the
boundary part, and both planners share that one function with the oracle so
there is a single answer to "how far inside is this chord".

This also exposes shapely disagreeing with itself: on seed 194 `'T********'`
reports a **dimension-1** interior overlap for a chord whose interior overlap
**measures 0.0 m**, because the chord runs along an edge and the predicate and
the overlay node it differently. The predicate stays as the cheap prefilter;
measuring settles what it flags.

**The fillet-arc gate resolves hits conservatively in the SEARCH, exactly in the
SMOOTHER** (`_corner_arc_clear(..., exact=)`), and that split is measured, not
principled. The smoother has ONE chord per waypoint pair, so a false hit there
costs a waypoint that marks no manoeuvre. The search has thousands of
alternatives and the dedup lattice makes quality non-monotone in successor
count: resolving hits exactly there too moved one route from 296.75 km to
**319.49 km (+7.7%)** and bought nothing on a second 300-scenario sample.
Conservative is also the safe direction — it only ever declines a candidate.

Measured over 300 random scenarios (v0, the shipped planner), everything above
together: waypoints **1506 → 1392** (−7.6%), waypoints flown straight through
**100 → 0**, silent DP fallbacks **33 → 7**, missions solved and oracle
rejections unchanged, path length −0.0097%, time **+6%** then **+2%** for the
interior-overlap round (paired, 3 repeats each — single runs drift 74→78 s on
identical code, so pair them). Confirmed on a second, disjoint sample (seeds
300–599): waypoints 1474 → 1399, straight-through 74 → 0.

Not covered, and deliberately: a 2-waypoint path returns early (`len(path) < 3`),
so a pure straight-line mission keeps its `L0` and `d_ss` waypoints even though
both read turn = 0 (18 of 44 waypoints across the named presets). The DP would
not remove them anyway — it would fold `O..T` to a single chord and produce an
EMPTY interior list, which `smooth_path` refuses. That refusal is load-bearing:
`straight_segments_ok` treats a one-segment path as the FIRST straight run and
checks it against `L0` only, so collapsing the mission would drop the `>= DSS`
run-in check altogether. The renderer does not need them (it places its `L₀` /
`d_ss` markers by arc length along the flown path), but the oracle does.

**Lift the navigation targets of EVERY obstacle type, not just circles — that
is the root fix, and everything above is the safety net.** Circle tangent points
were always built on `radius + _construct_delta`; polygon hull vertices were
handed to `get_next_states` raw, sitting EXACTLY on the boundary they have to
clear. That asymmetry is what put the boundary case in front of shapely on every
chord that ends at, passes through, or runs along a hull edge — the whole family
of interior-overlap artefacts above traces back to it. `_poly_vertices` is now
`convex_hull.buffer(_construct_delta, join_style=2)` (mitre: offsets every edge
perpendicular by delta, keeps the corner count).

Measured over 60 scenarios, the interior-overlap measurement fires **23 times,
9 of them forgiving an overlap** (0.0, 1.0e-10, 7.8e-10 m) without the lift, and
**0 times with it** — the predicate no longer flags anything to resolve. Over
600 scenarios in two disjoint halves: missions solved unchanged, waypoints flown
straight through still 0, path length −0.043% / +0.012% (median per case 0.000%,
p10 −0.0002%, p90 +0.0003% — a 1 m lift moves a 250 km mission by fractions of a
ppm), and **5% FASTER**, because the measuring path is never taken.

Keep the checking machinery anyway. `path_validation` is an independent oracle
that validates paths from any source and cannot assume the planner's lift, and a
fillet arc is constructed from the LEGS, not from the lifted vertices, so it can
still come tangent to some other polygon's edge. The machinery is now dormant,
not redundant — and "measured zero over 60 scenarios" is exactly the claim this
file has been wrong about three times.

**Rounding is absorbed when CONSTRUCTING, never forgiven when CHECKING.**
Geometry built to sit exactly on a limit gets rejected by the exact check that
follows — measured, **43% of tangent points fell inside their own circle** by
~1e-11 m (1 ULP). So every construction lifts by `CONSTRUCTION_CLEARANCE_M +
GEOM_EPS_M`: an operational stand-off (may be 0) **plus** a rounding guard
(never 0), added rather than merged. Pad **towards feasibility** — that means
`+` on an obstacle radius but `−` on a turn limit and *longer* on a straight
floor; `+EPS` on `α_max` would construct the violation it is meant to prevent.
Checks then carry no slack at all — **including the independent oracle**.
`path_validation` has no `circle_tol` parameter and compares `l1 >= L0`,
`l - dss >= 0`, `l > 0` and `turn <= α_max` exactly.

**Distinguish a forgiveness from a resolution limit.** Two numbers in
`path_validation` keep a tolerance on purpose, and neither is a check against a
limit: `TURN_RESERVE_TOL_M` *classifies* which waypoints split a straight run,
and `POLYGON_TOUCH_TOL_M` bounds how short an interior overlap the validator can
still tell apart from a tangency. Driving either to 0 makes the oracle reject
flyable missions because of *its own* rounding — measured for the polygon one:
3 of 300 v0 scenarios, rejected on overlaps of 8.1e-9 m, 4.4e-9 m and
**5.8e-11 m** (0.06 nanometres) where a fillet arc is tangent to a hull edge.
It is now 1e-6 m: 100× above that noise, still 6 orders below anything
operational. It was 1e-3 m, which genuinely *was* a forgiveness. Making it exact required padding construction FIRST;
measured worst margins on accepted paths, before → after that padding:

| quantity | before | after |
| --- | --- | --- |
| circle penetration | −0.112 m (already clear) | unchanged |
| polygon interior overlap | no occurrences | unchanged |
| turn vs α_max | **−1.1e-15 rad (over!)** | +1.0e-9 rad |
| l1 vs L0 | **−1.4e-11 m (short!)** | +9.96e-9 m |
| last leg vs DSS | +413 m | unchanged |
| middle straight | +96 m | unchanged |

Both planners therefore build with `self._alpha_build = α_max − GEOM_EPS_RAD`
and seed start corners at `L0 + GEOM_EPS_M + R·tan(αᵢ/2)`. **The one number in
`path_validation` that keeps a tolerance is `TURN_RESERVE_TOL_M`** — it
classifies which waypoints split a straight run rather than comparing a value
against a limit, and driving it to 0 would make float noise at a *deliberately*
collinear waypoint (arc-hop departure, pivot slide) manufacture zero-length
segments that then fail the exact `l > 0` test.

Two traps this exposed, both worth remembering:
- **`config.EPS` is dimensionless by accident.** `dx*dx + dy*dy < config.EPS`
  compares **squared** metres, so a "1 µm" constant was really a 1 mm cutoff.
  Use `GEOM_EPS_M`/`GEOM_EPS_RAD`, squared where the quantity is squared.
- **A classifier tolerance must track the construction lift.** v0's
  `_on_circle_boundary` used a fixed 1e-6 m; once tangents were built at
  `r + 1e-3` or more, *nothing* classified as boundary-riding and the wrap step
  silently switched off — which is what made a larger lift look like a 2.4%
  path-length cost. With the tolerance tracking the lift, the 1 m stand-off
  costs nothing.

Collision checks are **exact** (zero tolerance): a circle is hit iff the
(inlined) point-to-segment distance is `< radius` — `CIRCLE_GRAZE_TOL_M` is
deprecated (`0.0`). Feasibility of boundary-riding chords comes from the
CONSTRUCTION side instead: all tangent / bitangent / arc geometry is built on
`radius + config.CONSTRUCTION_CLEARANCE_M`, so legitimate chords keep a true
clearance margin rather than relying on a forgiven intrusion. Polygons use a
DE-9IM interior predicate (`relate_pattern('T********')`) that allows
boundary-following and endpoint-touch but blocks interior penetration. Arc-hop
riding clearance is checked by `_sector_clear` over the true annular sector
`[r_ride, r_ride/cos(π/8)]` (not just the outer ring — that closed the seed-155
gap). Note (perf): the circle collision loop is a hand-inlined scalar loop on
purpose — numpy-vectorising it or adding an STRtree over circles is SLOWER at
these obstacle counts (N≈6–16); see the memory note.

### Rendering model (render/trajectory.py)

`sample_trajectory(path, R, mode)` turns the planner waypoints into a drawable
polyline. `mode='straight'` joins waypoints directly; `mode='dubins'` rounds each
interior corner with a radius-`R` **fillet arc** tangent to both legs (symmetric
about the waypoint), which keeps the start/approach headings exact.
`build_full_path` prepends start `O` and appends goal `T` so the drawn path
spans the whole mission; `turn_markers` returns each arc's start/end/angle.
