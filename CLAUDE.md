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

There is **no test framework** — `main.py` *is* the "test suite" (it runs scenarios and prints a pass/fail table based on whether a path was found). Note `test_*.py` is gitignored, so any scratch test files you create won't be committed.

To run/debug a single scenario instead of all 16, call the pieces directly (this is the canonical pipeline):

```python
import map_generator as mg, preprocessing as prep, kinodynamic_astar as astar, visualizer as viz
scenario = mg.scenario4_complex_maze()        # any function in mg.get_all_scenarios()
pre = prep.prepare_scenario(scenario)
result = astar.plan_trajectory(pre, verbose=True)
if result['success']: viz.plot_scenario(scenario, pre, result, save_path="out.png")
```

`main.py` forces the matplotlib `Agg` backend for headless rendering; the GUI uses the interactive Tk backend.

## Architecture / data flow

The modules form a strict one-directional pipeline. `config.py` is imported everywhere; nothing imports `main.py`/`launch_gui.py`.

```
config.py            tactical constants + deg/rad helpers (R, ALPHA_MAX, L0, DSS, SAFE_MARGIN, map bounds)
  → map_generator    builds a "scenario" dict (start/goal + obstacles); 16 predefined + create_scenario()
  → preprocessing    prepare_scenario(): inflates obstacles, computes start/goal *waypoint* states
  → graph_builder    generate_bitangents() → TangentGraph; extend_..._with_start_goal() wires in endpoints
  → kinodynamic_astar plan_trajectory(): A* over (waypoint, heading); returns path + stats + graph
  → dubins_curves    smooth (position,heading)→(position,heading) transitions into arc+line segments
  → visualizer       plot_scenario / plot_trajectory_details / plot_obstacles_comparison
```

### The two dict shapes that flow through the pipeline

- **scenario** (from `map_generator`): `start`, `start_heading`, `goal`, `goal_heading`, `islands` (list of polygons), `sam_sites` (list of `(center, radius)`), and a unified `obstacles` list where each item is `{'type': 'polygon', 'polygon': [...]}` or `{'type': 'circle', 'center', 'radius'}`.
- **preprocessed** (from `preprocessing.prepare_scenario`): adds `start_state`/`goal_state` (each a dict with `waypoint` + `heading`), `turn_radius`, `alpha_max_rad`, and the *inflated* obstacles split into `circle_obstacles` (list of `(center, radius)`) and `polygon_obstacles` (list of coord lists). The A\* planner consumes only the preprocessed dict.

### Conventions (important — easy to get wrong)

- **Units are meters; angles are radians** throughout the algorithm code. `config.ALPHA_MAX`/`LAUNCH_ANGLE_*` are stored in **degrees** and converted (`config.ALPHA_MAX_RAD`, `config.deg_to_rad`). The map is 500 km × 500 km, `R = 8000 m`.
- A planner **state** is the tuple `(waypoint, heading)` where `waypoint = (x, y)`. Paths are lists of these tuples. `spatial_utils.state_to_tuple` is used for hashing/dedup.
- **Obstacle inflation** is not just `R + margin`. `preprocessing.inflate_obstacles` uses `R * (1/cos(α_max/2) - 1) + SAFE_MARGIN` so a turn at max angle still clears the obstacle. Start/goal waypoints (`W₁`, `W_{n-1}`) are offset from the raw launch/target points by `L0`/`DSS` plus a turn-radius term — the planner never searches to the literal target.

### How the A\* search actually works (kinodynamic_astar.py)

`get_next_states` generates successors two ways: (1) every tangent-graph node reachable with a valid turn + clear line of sight, and (2) radial fan sampling within `±α_max`. `validate_kinodynamics` enforces the max-turn-angle and minimum-straight-segment constraints. Collision checks use point-to-line distance for circles and Shapely `LineString.intersects` for polygons.

## Gotchas — README and code have drifted

`README.md` is partly stale; trust the code over the README when they disagree:

- README says R=500 m, α_max=30°, 4 scenarios. Reality: **R=8000 m**, **16 scenarios** (`map_generator.get_all_scenarios`, grouped baseline/easy/medium/hard), Dubins smoothing added.
- The **"Lazy Convex Hull fallback"** described prominently in the README is **not implemented** — on failure the search just returns `None`. Don't assume it exists.
- `plan_trajectory` builds and returns the tangent graph but the **`smooth_path` call is commented out**; the radial-sampling kinodynamic validation block in `get_next_states` is also commented out. If editing search behavior, check which paths are actually live.
- `dubins_curves.py` `_sample_at_distance` is a **linear-interpolation placeholder**, not true Dubins arc sampling (noted in its own comments).
