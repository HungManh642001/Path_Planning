# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single-process Python research codebase for planning autonomous aircraft trajectories. The planner combines a **Kinodynamic A\*** search over `(position, heading)` states with a **tangent (bitangent) graph** for long-range connectivity, then renders the result with matplotlib. Everything runs locally; there is no server, package, or external service.

## Commands

```bash
pip install -r requirements.txt          # numpy, scipy, shapely, matplotlib (+ tkinter for the GUI)

python main.py                           # Batch test harness: runs all 18 scenarios, writes PNGs to results/
python launch_gui.py                     # Interactive Tk GUI: click to place start/goal/obstacles, then plan
python performance_eval.py               # Performance metric helpers (also imported by main.py)

# The gate for ANY change to either planner. Dumps every waypoint, so a claimed
# bit-identical optimisation can actually be checked. Read scripts/BENCHMARKS.md
# BEFORE trusting a number out of it — two traps there will otherwise fool you.
PYTHONPATH=. python scripts/ab_planners.py run --planner v0 --seeds 300 --out /tmp/new.json
PYTHONPATH=. python scripts/ab_planners.py compare docs/benchmarks/baseline-v0-free.json /tmp/new.json

# Static analysis. Config lives in pyproject.toml; `core/` and `render/` are
# clean under BOTH as of 2026-08-21 and are expected to stay that way.
ruff check core/ render/ && ruff format --check core/ render/
pyright                                  # strict mode, scoped to core/ + render/
```

There **is** a pytest suite under `tests/` — run `python -m pytest -q` from the repo root (`pytest.ini` sets `pythonpath = .` so tests can `import config` / `import core.* / render.* / gui.*`, and `testpaths = tests`). Test files are named `*_test.py`; `test_*.py` is a scratch-test convention. `main.py` is a separate batch harness that runs all scenarios and writes PNGs.

**`/tests/` is in `.gitignore`.** Individual test files are tracked only because they were force-added, and about a third are NOT — as of 2026-08-20 `goal_shot_align_gate_test.py`, `safezone_test.py`, `start_corner_test.py`, `strategy_b_ladder_test.py`, `strategy_b_valve_test.py`, `goal_heading_free_test.py` and `plot_extents_test.py` exist only on this machine. Check `git ls-files tests/` before assuming a test is shared — an earlier version of this file claimed all `*_test.py` "ARE committed", and acting on that claim means a deletion or an edit leaves no trace in git history at all.

**The environment needs `numpy 1.26.4`, and the whole suite runs.** Until 2026-08-20 numpy was 2.4.6 here, which broke every package this Anaconda base was built against — matplotlib 3.8, pandas 2.1.4, contourpy, numba, pywavelets, astropy, scikit-image, scikit-learn all pin `numpy<2` — so six test files could not even be COLLECTED (`numpy.core.multiarray failed to import`) and `pytest -q` silently reported a green 83/85 while covering barely half the suite. numpy 2.4.6 was the outlier, not the rest: `scipy 1.17.1` wants `numpy>=1.26.4,<2.7`, so 1.26.4 satisfies everything at once. Going the other way (upgrading matplotlib and pandas to numpy-2 builds) drags in **pandas 3.0**, a breaking release, plus half the stack.

If a `_ARRAY_API not found` / `numpy.core.multiarray failed to import` ever comes back, it is the same thing: `pip install "numpy==1.26.4"`.

**The real gate is `pytest -q tests/` = 188 passed, 6 failed** (as of 2026-08-21; it was 170/7 before the Strategy-B and goal-shot work added tests). The six all predate that work — the original seven were verified against `5400d9c` in a scratch worktree, and one of them has since gone green for a reason unrelated to its feature (see the table) — so treat them as the baseline, not as breakage:

| failing test | why |
| --- | --- |
| `goal_shot_align_gate_test::test_knob_off_restores_aligned_shot` | feature has tests but no implementation: `config.GOAL_SHOT_ALIGN_GATE` does not exist, and it must NOT be built — see the goal-shot note below. Its sibling `test_gate_skips_aligned_shot` went GREEN on 2026-08-21 **by accident**: `GOAL_SHOT_CONE = 3` means main finds no candidate from that particular aligned state, which is not the gate existing. Do not read it as the feature landing |
| `hard_seeds_test[674-584760.0]` | seed 674 fails to plan. Recorded as a real open regression, but treat that as UNCONFIRMED: this file's maps come from `batch_random_test.generate_random_scenario`, so a scratch edit there silently changes which mission "seed 674" is — see the note under this table |
| `kinodynamic_arc_hop_test::test_no_radial_fan_in_open_water` | asserts 1 successor, gets 7. It encodes a *rejected* design — suppressing the fan on every line-of-sight-clear expansion costs seed 51 +73.5%, see the Strategy-B note below |
| `kinodynamic_arc_hop_test::test_check_fixed_legs_detects_blocked_start_and_goal` | signature drift: calls `_check_fixed_legs(body)` expecting `(ok, reason)`; it now takes no arguments and returns a bool |
| `kinodynamic_arc_hop_test::test_plan_maps_blocked_leg_to_failure_reason` | same signature drift, via monkeypatch |
| `strategy_b_valve_test::test_non_corner_expansion_still_consumes_valve_budget` | asserts the global-budget branch decrements `num_strategy_b`; `STRATEGY_B_CONSECUTIVE = True` takes the per-path branch instead, leaving that counter untouched |

**That count is not stable, because `hard_seeds_test` imports
`generate_random_scenario` from `batch_random_test.py` — a file that is
routinely edited as scratch.** Its four seeds are then DIFFERENT MAPS from the
ones the recorded ceilings were measured on: with the working copy as of
2026-08-21, seed 674 is a 232 km free-goal mission where the committed
generator draws a 470 km fixed-goal one, and 125 / 319 / 981 all move too. The
test then passes or fails on luck rather than on the planner, so **check
`git diff batch_random_test.py` before reading anything into that file's
result** — a 674 that goes green is the likeliest false signal here.

To run/debug a single scenario instead of all 18, call the pieces directly (this is the canonical pipeline):

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
  → types                   shared TypedDicts/aliases for the two dict shapes below (typing only, no logic)
  → map_generator         builds a "scenario" dict (start/goal + obstacles); 16 predefined + create_scenario()
  → preprocessing         prepare_scenario(): inflates obstacles, computes start/goal *waypoint* states
  → kinodynamic_astar     plan_trajectory(): A* over (waypoint, heading); returns path + stats
  → path_validation       independent oracle: segments/arcs clear, turn angles ok (used by tests + GUI summary)
  → spatial_utils         geometry helpers (distance, headings, angle_diff, polygon inflation, tangent points)
  → mission               full_mission_path(): the flown path O..T (planners AND render share it)
  → arc_geometry          pure arc math for main's arc-hop (riding, bitangents, sector cover)
  → goal_shot             pure geometry for main's analytic 2-corner terminal shot
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
  allowed and is common (100/100 scenarios) — though see the next bullet for
  why "common" understates it.
- **Start and goal need real clearance.** The buffer was `config.EPS`, i.e.
  1e-6 m, which permitted an obstacle to touch the start point; 16% of scenarios
  put start or goal closer than `L0` to an obstacle, so the mandatory takeoff or
  seeker leg was born blocked. Now `SPAWN_CLEARANCE_M`. This is where most
  `start_leg_blocked` / `goal_leg_blocked` results came from — they were
  **generator artifacts, not hard scenarios**: fixing it took free-goal
  `start_leg_blocked` 2→0 and adverse `goal_leg_blocked` 10→7.

- **OPEN BUG (found 2026-08-20, not yet fixed): the two generators share one
  RNG stream.** `create_scenario` passes the same `seed` to both, and each calls
  `random.seed(seed)` on the global `random` module — so the circle generator
  restarts the identical sequence the island generator just used. With
  `topology='random'` the first circle's centre is therefore **bit-identical to
  the first island's centre**, every seed, verified: both draw
  `(179533.10593326495, 110339.66956980077)` at seed 7. That is where the
  "cross-type overlap is common (100/100 scenarios)" observation above comes
  from — it is an RNG artefact, not a property of the maps. The fix is a local
  `random.Random(seed)` per generator, and it will move **every** benchmark
  number in `docs/benchmarks/`, so it needs a full re-baseline in the same
  commit.

Consequence to remember: any change here **moves every random benchmark**, so
numbers are only comparable within one generator version (see also the note
that `batch_random_test` drifts).

### The two dict shapes that flow through the pipeline

- **scenario** (from `map_generator`): `start`, `start_heading`, `goal`, `goal_heading`, `islands` (list of polygons), `dynamic_obstacles` (list of `(center, radius)`), and a unified `obstacles` list where each item is `{'type': 'polygon', 'polygon': [...]}` or `{'type': 'circle', 'center', 'radius'}`.
- **preprocessed** (from `preprocessing.prepare_scenario`): adds `start_state`/`goal_state` (each a dict with `waypoint` + `heading`), `turn_radius`, `alpha_max_rad`, and the *inflated* obstacles split into `circle_obstacles` (list of `(center, radius)`) and `polygon_obstacles` (list of coord lists). The A\* planner consumes only the preprocessed dict.

### Conventions (important — easy to get wrong)

- **Units are meters; angles are radians** throughout the algorithm code. `config.ALPHA_MAX`/`START_ANGLE_*` are stored in **degrees** and converted (`config.ALPHA_MAX_RAD`, `config.deg_to_rad`). The map is 500 km × 500 km, `R = 8000 m`.
- A planner **state** is the tuple `(waypoint, heading)` where `waypoint = (x, y)`. Paths are lists of these tuples. `spatial_utils.state_to_tuple` is used for hashing/dedup.
- **Obstacle inflation is exactly `SAFE_MARGIN`** — the operator's stand-off, nothing else. It used to add a `R*(1/cos(α_max/2)-1)` worst-case fillet term; that is gone (the search checks each arc exactly, per corner, with the real turn angle), because sized for α_max and applied to every obstacle it closed 49% of all corridors between obstacle pairs. Start/goal waypoints (`W₁`, `W_{n-1}`) are offset from the raw start/goal points by `L0`/`DSS` plus a turn-radius term — the planner never searches to the literal goal.
- **Operating-area bounds** (`_in_bounds`): a `safezones` polygon ⇒ point must be `covers`-ed by it; else an **explicit** `map_bounds` ⇒ the `[0,w]×[0,h]` rectangle; else (neither given) ⇒ **permissive** (returns True). The old code always fell back to the `config.MAP_WIDTH/HEIGHT` 500 km box, which silently rejected every waypoint of scenarios living outside it (e.g. real missions at `y≈1.15e6` that pass no bounds) — surfaced once start corners started filtering on `_in_bounds`. NOTE the two easy scenario-key mistakes that make a scenario effectively unbounded: the planner reads `safezones` (plural) and `map_bounds` (a `(w,h)` tuple) — `safezone`/`map_width`/`map_height` are ignored.
- **Seeded start corners** (replaces the single worst-case `W₁`): the planner does NOT root the search at one `W₁ = O + (L0 + R·tan(α_max/2))·û`. Instead `KinodynamicAstar.__init__` seeds `config.NUM_START_CORNERS` (K=4) corner states on the takeoff ray at `d_i = L0 + R·tan(αᵢ/2)`, tan-uniform buckets `tan(αᵢ/2) = (i/K)·tan(α_max/2)`, i=1..K (bucket K == the legacy `W₁`, so `NUM_START_CORNERS=1` is exactly legacy — the A/B knob). A corner seeded for `αᵢ` affords any first turn `α ≤ αᵢ` while keeping the takeoff straight `l₁ ≥ L0` **exactly** (via `State.straight_budget` + `State.min_straight_in = L0`, checked in `_doan_trinh`). Corners outside the operating area or with a colliding `O→corner` leg are not seeded (feasibility recovery — the single legacy `W₁` could land inside an inflated obstacle / outside a safezone corridor and kill the whole plan; e.g. a diagonal-corridor safezone with an adverse takeoff heading). If none survive, the start is blocked (fast honest failure). `smooth_path` re-guards `l₁ ≥ L0` at the first anchor since a shortcut can change `α₁`. NOTE: the terminal `W_{n-1}` still uses the α_max reserve (fixed mode) — that's a planned Phase-2; free-goal mode already targets `T` with a real-angle usable run-in ≥ DSS.

- **`core/kinodynamic_astar_v0.py` is TRACKED, and it is the planner that
  actually ships.** `batch_random_test.py` imports it (line ~117, shadowing the
  `kinodynamic_astar` import at the top of that file); `main.py`, `run_test.py`
  and `gui/app.py` import the MAIN planner instead. (`ml_planner/` used to be the
  other big consumer of the main planner; it was deleted on 2026-08-21.) An earlier version of this file called v0 "gitignored, may not exist
  in a fresh clone" — it is in `git ls-files`.

  It began as a **readability-first** variant kept in sync by hand: simpler
  `WRAP_STEP_M` straight-continuation instead of `_arc_hop_successors`, and
  `path_validation` / `spatial_utils` helpers instead of the main file's
  hand-inlined hot loops. That framing is now in tension with its role — a file
  that ships is not a sketch — so when the two conflict, **v0 is the standard
  and main follows it** (owner's call, 2026-08-20). Keep its docstrings short
  regardless: one or two lines of *what*, not the main file's long *why* essays.

  Measured over 300 seeds, both goal modes: the two solve the SAME missions
  (294/300 free, 243/300 fixed). Main's paths are 0.53% shorter and cost 2.5x
  the time and 21% more waypoints. Neither dominates; do not "upgrade" one to
  the other without an A/B. Divergences that remain on purpose: arc-hop (main
  only), `STRATEGY_B_CONSECUTIVE` (main only), the interior-overlap machinery
  (main keeps it, v0 deleted it), and `FAN_SKIP_ON_SHORT_RUNIN` (v0 only, see
  the Strategy-B note below). **The analytic goal shot used to be on that list
  and is not any more** — it was ported to v0 on 2026-08-21, where it is also
  faster than main's copy. Note it still never runs on the `batch_random_test`
  path, which plans in free-goal mode where `_try_goal_shot` returns
  immediately; that is why v0 survived so long without it.

  **That 0.53% gap is Strategy B, and nothing else — measured 2026-08-21 by
  porting main's fan gate into v0 and then reverting it.** Two things move
  across: the boundary-riding exemption (`if successors and not riding and ...`,
  which v0 lacks entirely) and the `STRATEGY_B_CONSECUTIVE` hybrid
  (`State.consec_b` + `_sb_global`). With both, over 300 free seeds:

  | v0 variant | length vs shipped v0 | iterations | time |
  | --- | --- | --- | --- |
  | as shipped | — | 28,670 | 40.8s |
  | + riding exemption only | **−0.337%** | 75,152 | 73.9s |
  | + hybrid budget (= main's gate) | **−0.545%** | 113,304 | 110.6s |
  | *main, for reference* | *−0.531%* | *119,673* | *113.8s* |

  Fixed mode is the same shape (−0.534%, 72,040 → 148,654 iterations), and so is
  the adverse suite (−0.346%, 81,174 → 147,922, 141/144 either way). **After the
  port v0 equals or beats main** — 74,336.6 km at 113,304 iterations against
  main's 74,347.6 km at 119,673 — so arc-hop and the interior-overlap machinery
  are NOT what makes main's paths shorter. Each half of the port buys about half
  the quality for about half the time.

  It was reverted because v0 is what `batch_random_test` imports and being the
  fast one is its reason to exist: the port makes it 2.7x slower (free) / 2.1x
  (fixed) to buy 0.53%, i.e. it turns v0 into a second copy of main. Keep that
  trade in mind rather than re-deriving it — and note the middle option, riding
  exemption alone, is a real point on the curve.

  **The goal shot is not a luxury — v0 went without one until 2026-08-21, and
  that was its largest single weakness.** On a 144-case adverse-heading suite
  (obstacle-free and lightly cluttered, 24 start headings × `goal_heading` ∈
  {None, +90°, −90°}): main 141/144 solved at 106,563 iterations; main with
  `GOAL_SHOT_ENABLED = False` 131/144 at 1,175,528; v0 **before the port**
  131/144 at 1,177,550 with **10 cases dying on `MAX_ITERATIONS`**; v0 **after**
  141/144 at **78,979**, none at the cap. Turning the shot off in main
  reproduced v0 almost exactly, so the shot was the whole gap. It lived in one
  group — obstacle-free with an adverse `goal_heading`, where v0 solved 14/24 —
  costing 764,528 iterations against main's 36,623 (**20.9x**). Adverse *start*
  headings were never the problem: all 24 solve in **2–17 iterations** in both
  planners. On the standard 300-seed sweep the port is **−16.16% iterations and
  −0.2219% length** in fixed mode, bit-identical in free (the shot is fixed-goal
  only), and `GOAL_SHOT_ENABLED = False` reproduces the pre-port planner exactly.

  **The shot is INSURANCE, not a speedup, so v0 arms it per MISSION**
  (`config.GOAL_SHOT_MIN_REVERSAL_DEG`, default `ALPHA_MAX`, `0.0` = always, the
  A/B knob). The premium is steep: over 300 random fixed seeds it is attempted
  **55,184 times, connects 4,663 times, and 87 of those (0.16% of attempts) reach
  the delivered path** — buying −0.22% length for **+26% wall-clock** (paired
  repeats; +33% excluding the one budget-bound seed). On missions that actually
  reverse, the same code is worth 10 extra solved missions and −77%. So it now
  fires only when the angle between `goal_heading` and the start→goal bearing
  reaches α_max. That threshold is derived, not tuned: below α_max a straight run
  at the goal can still turn onto `goal_heading` in ONE corner — which the
  ordinary Strategy-A goal candidate already builds — and above it one corner
  cannot, so the shot's two are the only way to finish.
  **Until 2026-08-21 neither benchmark contained a reversed approach** (the named
  scenarios topped out at 45°, the 300-seed sweep still does at 89.5°), which is
  why the armed planner is
  bit-identical to the PRE-port planner on the fixed sweep — the sweep still has
  no turn-around mission, so read that as a gap in the sweep rather than as "the
  shot is dead". `scenario_17_reversed_approach_open` and
  `scenario_18_reversed_approach_cluttered` were added to close the gap on the
  named-scenario side: measured on v0, 17 takes 3,657 iterations with the shot
  and 10,528 without, and **18 FAILS outright without it** (15,435 iterations),
  while every pre-existing preset is bit-unchanged either way (scenario_01: 6
  iterations both, scenario_16: 73 both) because the shot never arms on them. On the
  adverse suite the reversed group is untouched at 76,686 iterations and all
  141/144 solves are kept, while the fixed sweep drops **−20.9%** wall-clock
  against the always-armed shot.

  **Do NOT add the alignment gate `tests/goal_shot_align_gate_test.py` asks for.**
  Its premise — "when the approach bearing is already within α_max of
  `goal_heading`, the ordinary Strategy-A goal leg can arrive, so the 625-grid is
  redundant" — does not hold. Measured over 300 fixed seeds: 49.4% of shot calls
  are on aligned states, and **3,788 of the 4,663 successful shots come from
  them**, because alignment says nothing about whether the Strategy-A goal leg is
  flyable — it worked in **156 of 52,595** calls. The gate would delete 81% of all
  working shots. Those two tests stay red for the same reason
  `test_no_radial_fan_in_open_water` does.

  **The shot is the planner's most expensive single component, so it is
  optimised twice over, both bit-identical.** `two_corner_candidates` hoists the
  arrival cone out of the nested loop (it depends only on `j`, so 625 evaluations
  of each of `cos`, `sin`, `atan2`, `tan` become 25), and
  `_ray_chord_clear` memoises both legs per ray: every corner sharing a
  `leg1_heading` lies on ONE ray out of the state and every corner sharing an
  `arrival_heading` on one back-ray into the goal, so a clear chord proves every
  shorter one and a blocked chord every longer one. Collision checks per shot
  **39.06 → 11.38** (2,037,170 → 627,367 over 300 seeds); together **−12.5%**
  wall-clock, median of 3 paired repeats. Hit rate is only **8.9%**, so cheap
  misses matter more than fast hits. Capping the candidate list is NOT the way:
  the mean winning rank is 39.8 of ~39 candidates and only 21.9% of hits are
  rank 1, so a cap throws away the hits it was meant to reach cheaply.

- **A dead knob is worse than no knob.** `config.CIRCLE_GRAZE_TOL_M` is
  deprecated and pinned at `0.0` — no planner reads it — yet `gui/params.py`
  still renders it as a 0-500 m slider, so an operator can move it and change
  nothing. Either wire it or drop it from the panel.
- **Every tunable lives in `config.py`, never as a literal in the algorithm.**
  The most recent offender was a bare `< 10000` (squared metres) in main's
  successor loop, which silently made the two planners disagree about which
  candidates exist; it is now `config.CANDIDATE_MIN_DIST_M`, shared and
  measured. Watch for the same shape: a literal that encodes a DECISION rather
  than a degenerate-geometry guard.
  One constant per *meaning*, even when two happen to share a value: e.g. the
  đoản-trình floor `MIN_STRAIGHT_M` and the fan's float-noise pad
  `RADIAL_FAN_STEP_M` are separate knobs on purpose. `kinodynamic_astar.py`'s
  module-level `_MIN_STRAIGHT_M` is an alias of `config.MIN_STRAIGHT_M` (hot
  path), not a second definition — do not re-inline the number. Only degenerate
  -geometry guards (`< 1e-9` collinearity / right-angle tests) stay inline;
  they are not tunables.

### Typing and style contract (added 2026-08-21)

`core/` and `render/` are **Ruff-clean and Pyright-strict-clean**; `pyproject.toml`
holds both configs. Everything below was landed as a pure restyle and proved
bit-identical (300 seeds x 2 planners x 2 goal modes, `bit-identical 300/300`,
identical iteration counts), so the numbers in this file still stand.

- **`core/types.py` is the single vocabulary** for the two dict shapes. Only the
  keys the search reads unconditionally are required; the rest are
  `NotRequired`, because GUI and render callers legitimately pass a partial
  mapping (`gui/map_canvas.py` passes `preprocessed or {}`). Marking them
  required would make the checker bless code that `KeyError`s on those inputs.
  The planners resolve `start_pos` / `goal_pos` / `straight_length` ONCE in
  `__init__` (`self._origin`, `self._target`, `self._l0`) and raise a clear
  `ValueError` when they are missing, instead of failing deep inside the search.
- **`plan_trajectory()` is now a thin wrapper over `KinodynamicAstar.plan()`**
  in both planners. It used to be a module-level function reaching into
  `planner._check_fixed_legs()`; the name, signature and result dict are
  unchanged. `_check_fixed_legs` kept its name only because `ml_planner/plan.py`
  called it — **`ml_planner/` was deleted on 2026-08-21**, so the last external
  caller is gone and it is now free to become public. The only remaining
  reference is `kinodynamic_arc_hop_test`, which is already red on signature
  drift.
- **`path_validation.arc_points` is public** (was `_arc_points`). It is a shared
  contract, not an internal: v0 calls it so the search weighs the SAME arc the
  oracle will.
- **`preprocessing` parameters are snake_case**: `prepare_scenario(scenario,
  turn_radius=, l0=, dss=, safe_margin=, alpha_max_rad=)` — was `R=, L0=, DSS=`.
  `path_is_valid` likewise takes `turn_radius=` / `l0=`.
- **Exactly two relaxations, both at a third-party boundary.** `render/` gets
  `reportUnknownMemberType` / `reportUnknownArgumentType` switched off via a
  per-directory `executionEnvironments` block, because matplotlib 3.8's stubs are
  partial and strict there reported ~300 unknowns that come from the LIBRARY.
  Everything that catches defects stays strict in `render/` too — verified with a
  canary: an unannotated parameter, an unused function and a `None + 1` are all
  still errors there. `core/spatial_utils.py` keeps a file-level suppression
  because shapely 2.1.2 ships no `py.typed`, so the checker infers `buffer()` as
  returning `Polygon` when it can return a `MultiPolygon` — the runtime branch is
  right and the checker is not. Nothing else is suppressed; `Any` appears nowhere.

  **Two pyright traps this cost, both measured, do not re-discover them:**
  `typeCheckingMode` inside `executionEnvironments` is silently IGNORED by
  pyright 1.1.411 (per-rule overrides work, the mode does not); and
  `executionEnvironments.root` also becomes the **import resolution root**, so
  without `extraPaths = ["."]` the package stops seeing `config` / `core.*` /
  its own `render.*` — that alone turned 0 errors into 174, of which 168 were
  downstream noise from 6 unresolved imports.
- **`pythonVersion` is pinned to 3.11**, matching the interpreter here;
  `typing.NotRequired` needs it.
- **Unicode in operator-facing text is deliberate** — `α_max`, `L₀`, `°`,
  `✓/✗`, and the domain term **đoản trình**. `RUF001-003` (the homoglyph hunt)
  are **off**: they exist to catch a Cyrillic `а` smuggled into an identifier,
  and here they flagged exactly one character — `α`, the domain's own notation.
  Following them silently changed operator-facing labels and mangled a
  Vietnamese technical term. Do not ASCII-fold these: `main.py`,
  `performance_eval.py` and `check_waypoints.py` use the same symbols.

- **`ruff`'s `D` rules only ever demand docstrings on PUBLIC functions** —
  verified. Docstrings on private helpers are a house choice, not a rule, so
  they are the first thing to trim if the file feels padded.

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
`NUM_STRATEGY_B` gates only the **budget**, not whether the fan fires, and
gating the FIRING is where this planner punishes intuition: the dedup lattice
(`STATE_POS_QUANTUM`, `STATE_HEADING_QUANTUM_DEG`) makes the search only
approximately optimal, so the fan's redundant-looking pivots act as lattice
diversity rather than noise. Suppressing the fan on every line-of-sight-clear
expansion costs **seed 51 +73.5%** (measured 2026-08-21, 300 seeds). The older
figure here — "gating on *the goal is already reachable* costs seed 4 88 km" —
**no longer reproduces**: that exact gate now costs +0.0376% (free) / +0.0483%
(fixed) and saves nothing, because it fires ~250 times in 300 seeds. The lesson
stands, the example is stale; re-measure before quoting a number from this file.
Each fan direction emits `config.NUM_FAN_DISTANCES` **distance rungs**, not one
leg: rung `j` is the shortest leg still affording a next turn `β ≤ βⱼ`, with
tan-uniform buckets `tan(βⱼ/2) = (j/M)·tan(α_max/2)` — the same capability-bucket
idea as the seeded start corners (`M = 1` + the legacy 1000 m pad reproduces the
old single worst-case leg exactly). The old code hardcoded `β = α_max`, so every
fan leg paid the worst-case far reserve even when the pivot barely turns, which
bulged fan-routed paths in open water. M is **measured, not tuned by intuition**
— the relation is not monotone in M (see the note in `config.py`).

**The fan's firing conditions do not line up with the jobs it was written for,
and the mismatch is measured** (2026-08-21, v0, 300 seeds per goal mode, plus a
144-case adverse-heading suite). Labelling each firing by its real situation —
did Strategy A accept the goal, and if not, which gate refused it — gives route
yield (fan waypoints that reach the delivered path, per 1000 firings), and the
ranking is the same in both modes: goal refused by the **arc** gate 610/596 ·
goal occluded + budget 106/107 · start corner 38/44 · no-successor fallback
13/4 · goal clear but misaligned 2.3/0.9 · **run-in too short 0/7.8** · **goal
already an accepted successor 0/0**. Two things follow.

- **`config.FAN_SKIP_ON_SHORT_RUNIN` (v0 only, default on, `False` = legacy).**
  In FREE-goal mode, when the goal is line-of-sight clear and the only thing
  wrong with the direct leg is that it cannot supply `DSS`, the fan is skipped.
  It cannot help — every leg departs at `±α_max` or straight ahead at a fixed
  rung, none is aimed at the goal — and it fired 1,108 times over 300 seeds for
  ZERO route waypoints. Gate: **bit-identical 300/300 with iterations −11.22%**
  (wall-clock −8.0%, median of 3 paired repeats); fixed mode untouched, +0.00%.
  **Do not widen the scope.** In FIXED mode the identical rejection means
  "cannot turn onto `goal_heading`" — a different problem, 43.6% of firings
  there, carrying 143 route waypoints; dropping it costs +0.426% with one seed
  at +40%. The right fix there is the goal shot, which v0 does not have.
- **Zero route yield is NOT zero value, and only an A/B tells the two apart.**
  The "goal already an accepted successor" trigger also yields 0 waypoints, but
  removing it measured **worse on both axes** (+0.0376% length, +0.93%
  iterations) and was reverted. Same for the near-zero-yield "clear but
  misaligned" branch: keeping the high-yield arc trigger and dropping that one
  still costs seed 51 **+73%**. Route yield finds candidates; it never justifies
  a removal.

**`NUM_STRATEGY_B` does not mean what its name says, and the obvious fix was
built, measured on four benchmarks and REVERTED (2026-08-21).** The name reads
as "the fan may open nodes for the first 3 steps out of the start corner". v0
keeps one GLOBAL tally instead, spent by whichever expansions reach it first,
with start corners EXEMPT and a re-arm when the frontier nearly dies. Two
consequences, both measured over 100 free seeds: it is an off switch rather
than an allowance — the gate is reached 8,967 times and SUPPRESSES 8,747
(97.5%), only 220 firings ever spend it, and 89% of the fan's 2,016 firings
never consult it at all (start corner 348, goal already clear, or the
no-successor fallback) — and it bounds nothing consecutive despite the name:
fan runs reach depth 6, with 16% of firings on a state already 3+ fan legs
deep. It also couples unrelated branches of the tree, since one branch
spending the tally silences every other.

The replacement was two gates keyed to the state itself. RULE 1: the fan fires
only within N steps of the start corner (a budget DERIVED from depth, so two
siblings get the same allowance whatever either did), with the no-successor
fallback exempt. RULE 2: with Strategy-A candidates in hand and the goal
occluded, the fan is not consulted — those candidates ARE the way round the
obstacle — start corners still exempt. Together, against the shipped planner,
**no mission lost on any benchmark**:

| benchmark | length | iterations | solved |
| --- | --- | --- | --- |
| free 300 | +0.449% | **−13.9%** | 294/294 |
| fixed 300 | +0.852% | **−25.4%** | 243/243 |
| adverse 144 | +0.134% | **−46.7%** | 141/141 |
| 18 named presets | +0.007% | **−41.0%** | 18/18 |

`scenario_18_reversed_approach_cluttered` went 3,078 → 258 iterations for
+0.13%, and 15 of the 18 presets kept their length to the last digit.

**It was reverted for the TAIL, which the summary rows hide.** Median 0.000%
and 204/294 free paths bit-unchanged, but 13 free seeds move >1% and 30 fixed
ones do — seed 51 **+73.53%** (239 → 414 km), fixed seed 182 **+40.03%**, seed
206 +35.64%. Nothing meaningful gets shorter (best case −0.76%), so it is a
one-way trade. For scale, lazy edge validation was rejected over 8 seeds >1%
at worst +11.9%.

**The two rules split cleanly, and the split is the reusable part.** Rule 2 is
the whole win on the random distribution and carries a mild tail; rule 1 is a
COST there and owns the entire tail, earning its keep only on adverse
missions:

| | free length / iters | fixed length / iters | adverse iters | worst free |
| --- | --- | --- | --- | --- |
| rule 2 alone | +0.148% / **−17.5%** | +0.116% / −7.3% | −7.5% | +16.21% |
| both, horizon 3 | +0.449% / −13.9% | +0.852% / −25.4% | **−46.7%** | **+73.53%** |
| rule 1 alone | +0.351% / **+4.1%** | +0.740% / −26.6% | **−45.6%** | +73.53% |

Relaxing the horizon removes the tail AND helps free-mode iterations (3 →
−13.9%, 6 → −17.3%, 10 → −17.3%, none → −17.5%), so on random maps the horizon
is pure cost; it buys adverse turn-arounds instead. **Rule 2 alone was never
rejected on its merits** — it went out with the rest — so it is the obvious
thing to re-measure before inventing something new here.

**And the first reading of that name was WRONG in a way the random sweep could
not see.** Read as "consecutive FAN legs rooted at a start corner" (resetting
on any Strategy-A step) rather than plain depth, a cap of 3 is a bit-identical
**no-op** 300/300, because start-rooted fan runs never get past depth 2.
Forcing it to bind by capping at 0 — i.e. no fan at start corners — looks
excellent on the sweep (**−19.1% iterations for +1.0% length, nothing lost**)
and destroys the adverse suite: **141 → 91 of 144, every loss a genuine
`no_path`, every one of them in OPEN WATER**, where Strategy A offers no
candidate but the goal and the fan at takeoff is the only way to turn around.
The 300-seed sweep cannot see this: its start headings are only ±90° of the
start→goal bearing. Same blind spot the goal shot had before scenarios 17/18.

The max-turn-angle and đoản-trình (minimum-straight) constraints are enforced
inline by `_pivot_candidate` and `_doan_trinh` (a `preprocessing.
validate_kinodynamics` once did this; it was dead code by 2026-08-20 and is
gone). `search()` accepts the goal only when both within
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

**The two planners now differ here, deliberately.** `path_validation` keeps the
machinery unconditionally — it is an independent oracle, it validates paths from
any source, and it cannot assume the planner's lift. The MAIN planner keeps it
too (`_POLY_TOUCH_TOL_M`, `_polygons_deep`, `_corner_arc_clear(..., exact=)`).
**v0 dropped all of it** and takes `'T********'` at face value: measured over
600 scenarios in two disjoint halves, every one of the 600 paths is BIT-IDENTICAL
with the machinery removed, and v0 is **~9% faster** (median of 7 paired repeats;
one repeat of the seven came out +17%, so read the median, not the mean).

That is a safe divergence in one direction only: the bare predicate also fires
on a chord merely GRAZING a hull edge, so v0 is now fractionally STRICTER than
its validator. Strict-planner/permissive-oracle can only cost an opportunity —
it can never emit a path the oracle then rejects. The reverse would be a bug.

What makes it safe is the CONSTRUCTION-side lift, not the measurement: with the
vertex lift there is nothing left to resolve (23 → 0 firings over 60 scenarios).
Reverting the lift without restoring the machinery would bring the artefacts
straight back. And keep reading "measured zero over N scenarios" with suspicion
— it is exactly the claim this file has been wrong about three times.

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

**The waste is never a redundant check — it is a check run against an obstacle
that cannot possibly be involved.** Profiled on v0 over 40 scenarios: 45% of
runtime was `point_to_line_distance`, and a bbox test showed **97.6%** of the
circle tests in `_corner_arc_clear` and **82.3%** of those in `_check_collision`
were against a circle whose bounding box cannot reach the query — 93% of ALL the
distance work in the planner. Every gate, meanwhile, earns its keep: the
rejection histogram over 985,623 Strategy-A candidates is turn 546,825 /
đoản trình 65,271 / line-of-sight 336,326 / arc 2,810, and even `_in_bounds`
fires 6 times in 23,493.

So the fixes are prefilters and ordering, not deletions:
- a plain-float bbox test before any circle distance and before any LineString
  is constructed (on open water none is built at all);
- Strategy B ran `_doan_trinh` — pure arithmetic — AFTER the fillet-arc gate,
  the most expensive check in the planner, so 31% of the legs that reached the
  arc gate were rejected by arithmetic straight afterwards. Strategy A already
  ordered it correctly;
- v0's `STRtree` over polygons then had no callers and is gone.

Measured: **2.05× faster** (76→37 s and 51→27 s on two disjoint 300-scenario
samples), `point_to_line_distance` calls 12.0M → 0.82M, and every one of the 590
paths **bit-identical** — same waypoints, same length to the last ULP. Held to
that by `tests/collision_prefilter_test.py`, which re-checks random chords
against a brute-force copy with every prefilter removed.

Two things measured and NOT worth doing: a prepared-geometry `intersects`
prefilter in front of `relate_pattern` (2× cheaper per call, but 47% of
bbox-overlapping pairs are genuine hits, so it saves ≈0), and testing the shrunk
`_polygons_deep` copy BEFORE the full polygon (saves a relate on every hit but
costs one on every miss, and misses are the majority: 57,820 vs 51,180).

Two smaller levers, both applied to BOTH planners and both bit-identical over
200 v0 / 100 main paths (every coordinate exactly equal):
- **The cheap turn gate.** 55% of `_pivot_candidate`'s candidates die on the
  turn limit, and the exact test costs two `atan2` plus a `sin` and a `cos` to
  find out. `cos(turn) = dot / seg_len` is one multiply-add, with the heading's
  unit vector cached on `State` (once per state, used ~120 times). It is NOT
  bit-identical near the limit, and turns land ON the limit routinely here, so
  it only rejects what is over by more than `TURN_PREFILTER_BAND_RAD` (1e-6) and
  anything inside the band still gets the exact test — the cheap form can never
  decide a borderline case. Worth −3.8% (v0) and −1.5% (main), consistent.
- **A circle bbox prefilter in the MAIN planner's `_check_collision`**, which
  inlined the distance but prefiltered nothing. Worth −2.2%, and noisy (one of
  three paired repeats came out +0.1%) — the per-pair cost was already low, so
  this is nothing like the 2× the same idea bought in v0.

Note the shape of the result: the same idea is worth 2× where each skipped pair
was a function call plus a `distance()` call, and 2% where it was already ten
inline arithmetic ops. Prefilter what is EXPENSIVE per pair, not what is
frequent.

### Measuring changes here (read before trusting any number)

The gate is `scripts/ab_planners.py` + `scripts/BENCHMARKS.md`; baselines live in
the gitignored `docs/benchmarks/` and regenerate in minutes. Decide the gate
BEFORE writing the change: *bit-identical* for anything claimed to be a pure
optimisation, *A/B* for anything that changes which successors exist. A matching
summary is not evidence — two different routes agree on total length to four
decimals — so compare the waypoint dumps.

Four traps, each of which produced a confident wrong conclusion in one session
on 2026-08-20:

- **`grep` here honours `.gitignore`.** It is a shell function wrapping
  `ugrep --ignore-files`, and `.gitignore` lists `/tests/` and `docs/`. Every
  "I grepped the whole repo" claim made with it silently excluded the entire
  test suite. Use `command grep`, and prefer `python -m pyflakes core/ render/
  gui/ scripts/` for "is this still referenced" questions — it does not skip
  what git happens to ignore.
- **cProfile inflates small hot functions.** It attributes per-call overhead to
  exactly the tiny functions worth optimising: it claimed 7.4% for
  `state_to_tuple`, and caching it away was worth 0.5-1.3%. Use cProfile to FIND
  candidates, never to size the win.
- **Wall-clock is not comparable across time on this box** (3 GB WSL2). Same
  commit, same config, same 300 seeds, two hours apart: 102.1 s vs 165.1 s,
  +62%, every path bit-identical. Only paired repeats run back to back mean
  anything, medians of 3+.
- **`config.TIME_BUDGET_S = 15` makes the search wall-clock dependent** — the
  same seed on a slower machine explores fewer nodes and can return a different
  answer. Exactly one seed is budget-bound on the current sweeps (**seed 39,
  fixed mode**), measured anywhere from 13,624 to 26,183 iterations on identical
  code, and it alone shifted a fixed-mode call-count total by 29%. Take
  instrumented counts in `free` mode, where nothing is budget-bound and they
  reproduce exactly. Note the wider implication, which is not just a measurement
  problem: **the planner is not deterministic across machines.**

Two mechanical notes. Most files here are **CRLF** (`git ls-files --eol` to
check, there is no `.gitattributes`); patching with `open(p, 'w').write(s)` in
Python rewrites the whole file to LF and buries the real change in a 1600-line
diff. Use `newline='\r\n'`, or the editing tools, which preserve it. And
`str.replace` with a pattern that does not match is a silent no-op — assert the
match count when scripting an edit. Note CRLF is not uniform even WITHIN a
file: resolving a `.gitignore` conflict on 2026-08-21 failed because git had
written one side of the conflict block with LF and the rest with CRLF, so
neither an all-LF nor an all-CRLF pattern matched. Match line-wise on
`rstrip('\r')` rather than on an exact multi-line string — and remember that
a failed script followed by `git commit` on the NEXT line still commits, which
is how conflict markers reached a commit that day.

### Rendering model (render/trajectory.py)

`sample_trajectory(path, R, mode)` turns the planner waypoints into a drawable
polyline. `mode='straight'` joins waypoints directly; `mode='dubins'` rounds each
interior corner with a radius-`R` **fillet arc** tangent to both legs (symmetric
about the waypoint), which keeps the start/approach headings exact.
`build_full_path` prepends start `O` and appends goal `T` so the drawn path
spans the whole mission; `turn_markers` returns each arc's start/end/angle.
