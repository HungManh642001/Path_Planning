# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single-process Python research codebase for planning sea-skimming cruise-missile trajectories. The planner combines a **Kinodynamic A\*** search over `(position, heading)` states with a **tangent (bitangent) graph** for long-range connectivity, then renders the result with matplotlib. Everything runs locally; there is no server, package, or external service.

## Commands

```bash
pip install -r requirements.txt          # numpy, scipy, shapely, matplotlib (+ tkinter for the GUI)

python main.py                           # Batch test harness: runs all 16 scenarios, writes PNGs to results/
python launch_gui.py                     # Interactive Tk GUI: click to place start/goal/obstacles, then plan
python performance_eval.py               # Performance metric helpers (also imported by main.py)
```

There **is** a pytest suite — run `python -m pytest -q` from the repo root. Test files are named `*_test.py` (these ARE committed); `test_*.py` is gitignored, so name scratch tests that way to keep them out of git. `main.py` is a separate batch harness that runs all scenarios and writes PNGs.

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

### The two dict shapes that flow through the pipeline

- **scenario** (from `map_generator`): `start`, `start_heading`, `goal`, `goal_heading`, `islands` (list of polygons), `sam_sites` (list of `(center, radius)`), and a unified `obstacles` list where each item is `{'type': 'polygon', 'polygon': [...]}` or `{'type': 'circle', 'center', 'radius'}`.
- **preprocessed** (from `preprocessing.prepare_scenario`): adds `start_state`/`goal_state` (each a dict with `waypoint` + `heading`), `turn_radius`, `alpha_max_rad`, and the *inflated* obstacles split into `circle_obstacles` (list of `(center, radius)`) and `polygon_obstacles` (list of coord lists). The A\* planner consumes only the preprocessed dict.

### Conventions (important — easy to get wrong)

- **Units are meters; angles are radians** throughout the algorithm code. `config.ALPHA_MAX`/`LAUNCH_ANGLE_*` are stored in **degrees** and converted (`config.ALPHA_MAX_RAD`, `config.deg_to_rad`). The map is 500 km × 500 km, `R = 8000 m`.
- A planner **state** is the tuple `(waypoint, heading)` where `waypoint = (x, y)`. Paths are lists of these tuples. `spatial_utils.state_to_tuple` is used for hashing/dedup.
- **Obstacle inflation** is not just `R + margin`. `preprocessing.inflate_obstacles` uses `R * (1/cos(α_max/2) - 1) + SAFE_MARGIN` so a turn at max angle still clears the obstacle. Start/goal waypoints (`W₁`, `W_{n-1}`) are offset from the raw launch/target points by `L0`/`DSS` plus a turn-radius term — the planner never searches to the literal target.

### How the A\* search actually works (core/kinodynamic_astar.py)

`get_next_states` generates successors dynamically (no precomputed graph):
(1) **wrap-step** — a straight, heading-preserving step off a circle boundary so
the search can keep tangenting around it; (2) **Strategy A** — tangent points to
each circle (`spatial_utils.circle_tangent_points`) + polygon hull vertices + the
goal, each accepted only with a valid turn and a clear segment; (3) **Strategy B**
— an `±α_max` radial fan fallback when no Strategy-A candidate is valid.
`validate_kinodynamics` enforces the max-turn-angle and minimum-straight-segment
(đoản trình) constraints. `search()` accepts the goal only when both within
`GOAL_THRESHOLD` **and** the arrival heading is within `α_max` of `goal_heading`
(so the terminal turn onto the approach is feasible). `smooth_path` re-validates
the turn at the *following* waypoint before skipping one. Collision checks use
point-to-line distance for circles (with `CIRCLE_GRAZE_TOL_M`) and a DE-9IM
interior predicate (`relate_pattern('T********')`) for polygons, which allows
boundary-following and endpoint-touch but blocks interior penetration.

### Rendering model (render/trajectory.py)

`sample_trajectory(path, R, mode)` turns the planner waypoints into a drawable
polyline. `mode='straight'` joins waypoints directly; `mode='dubins'` rounds each
interior corner with a radius-`R` **fillet arc** tangent to both legs (symmetric
about the waypoint), which keeps the launch/approach headings exact.
`build_full_path` prepends launch `O` and appends target `T` so the drawn path
spans the whole mission; `turn_markers` returns each arc's start/end/angle.

## Gotchas

- `README.md` is stale: it says R=500 m, α_max=30°, 4 scenarios, and describes a
  "Lazy Convex Hull fallback" and a tangent-graph stage. Reality: **R=8000 m**,
  **α_max=90°**, **16 scenarios** (`core.map_generator.get_all_scenarios`), no
  convex-hull fallback (search returns `None` on failure), and the tangent graph
  has been removed (the planner uses dynamic successors). Trust the code.
- The old `graph_builder.py` (tangent graph) and `dubins_curves.py` (6-word Dubins
  solver) were **removed** — they were unused. Rendering uses the fillet model in
  `render/trajectory.py`, not a Dubins solver.
