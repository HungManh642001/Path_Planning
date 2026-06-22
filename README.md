# VCM Path Planning

Trajectory planner for autonomous aircraft. It searches a flyable route
from a launch point **O** to a target **T** through circular SAM threats and
polygonal islands, honouring the aircraft's turn radius and maximum turn angle,
then renders the real flight path (straight legs + radius-`R` turn arcs).

Everything runs locally — a single Python process, no server or external service.

## What it does

- **Kinodynamic A\*** search over `(position, heading)` states. Successors are
  generated dynamically (tangent points to circles, polygon corners, the goal,
  plus a radial fan fallback); there is no precomputed roadmap.
- **Obstacle inflation** that guarantees a max-angle turn still clears the
  obstacle, so a collision-free search path stays collision-free when flown.
- **Approach-heading feasibility**: the path is accepted only if the aircraft can
  turn onto the required approach heading at the target within the turn limit.
- **True flight rendering**: each corner is rounded by a radius-`R` arc tangent
  to both legs, so launch and approach headings are exact. The full path is drawn
  O → … → T with the turn-start / turn-end points marked.
- An **interactive Tk GUI** to build scenarios (click to place, drag to aim),
  run the planner, and read a results summary.

## Requirements

```bash
pip install -r requirements.txt      # numpy, scipy, shapely, matplotlib
```

The GUI also needs `tkinter` (bundled with most Python installs) and a display.

## Quick start

```bash
python main.py            # batch: run all 16 scenarios, write plots to results/
python launch_gui.py      # interactive GUI (build a scenario and plan it)
python -m pytest -q       # run the test suite (tests/)
```

## Planning a single scenario

The pipeline is just a few calls. `core.map_generator.get_all_scenarios()`
returns a dict of `name -> builder function`; call a builder to get the scenario
dict, or build your own with `create_scenario(...)`.

```python
import core.map_generator as mg
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import render.visualizer as viz

scenario = mg.scenario4_complex_maze()         # any builder in get_all_scenarios()
pre = prep.prepare_scenario(scenario)          # inflate obstacles, compute W1/W_{n-1}
result = astar.plan_trajectory(pre, verbose=True)
if result['success']:
    viz.plot_scenario(scenario, pre, result, save_path="out.png")
```

`main.py` forces the matplotlib `Agg` backend for headless rendering; the GUI
uses the interactive Tk backend.

## Project structure

```
config.py            shared tactical constants (R, ALPHA_MAX, L0, DSS, map bounds, …)
core/                planning pipeline
  map_generator.py     scenario builders (16 presets + create_scenario)
  preprocessing.py     prepare_scenario(): obstacle inflation + start/goal waypoint states
  kinodynamic_astar.py the A* search and path smoothing -> plan_trajectory()
  path_validation.py   independent oracle: segments/arcs clear, turn angles ok
  spatial_utils.py     geometry helpers (distance, headings, polygon inflation, tangents)
render/              drawing (consumes the planner path)
  trajectory.py        sample_trajectory(path, R, mode) + turn_markers() + build_full_path()
  visualizer.py        plot_scenario / plot_trajectory_details / plot_obstacles_comparison
gui/                 interactive Tk app (config | map | results)
tests/               pytest suite (see Testing)
main.py              batch harness   ·   launch_gui.py  GUI entry
```

The modules form a one-directional pipeline. `config.py` is imported everywhere;
nothing imports `main.py` / `launch_gui.py`.

### Data shapes

- **scenario** (from `map_generator`): `start`, `start_heading`, `goal`,
  `goal_heading`, `islands` (polygons), `sam_sites` (`(center, radius)`), and a
  unified `obstacles` list of `{'type':'polygon','polygon':[...]}` /
  `{'type':'circle','center','radius'}`.
- **preprocessed** (from `prepare_scenario`): adds `start_state` / `goal_state`
  (each `{waypoint, heading}`), `turn_radius`, `alpha_max_rad`, `safe_margin`, and
  the *inflated* obstacles split into `circle_obstacles` (`[(center, radius)]`) and
  `polygon_obstacles` (`[coords]`). The planner consumes only the preprocessed dict.

## How it works

**Search (`core/kinodynamic_astar.py`).** A state is `(waypoint, heading)`.
`get_next_states` proposes, in order: a straight *wrap-step* off a circle
boundary; tangent points to each circle + polygon corners + the goal (each
accepted only with a valid turn and a clear segment); and an `±α_max` radial fan
as a fallback. `validate_kinodynamics` enforces the max-turn-angle and the
minimum straight-segment (đoản trình) length. The goal is accepted only when the
state is within `GOAL_THRESHOLD` **and** its heading is within `α_max` of the
approach heading. `smooth_path` removes redundant waypoints, re-checking the turn
at the following waypoint so it never bends a turn past the limit.

**Collision checks.** Circles use point-to-segment distance (with a small graze
tolerance); polygons use a DE-9IM interior test that allows touching / boundary
-following but blocks interior penetration.

**Obstacle inflation (`core/preprocessing.py`).** Each obstacle is grown by
`R·(1/cos(α_max/2) − 1) + SAFE_MARGIN`, so a turn at the maximum angle still
clears it. Start/goal waypoints `W₁` / `W_{n-1}` are offset from the raw launch /
target points by `L0` / `DSS` plus a turn-radius term — the search never targets
the literal `O` / `T`.

**Rendering (`render/trajectory.py`).** `sample_trajectory(path, R, mode)` turns
the waypoints into a polyline. `mode='straight'` joins them directly;
`mode='dubins'` rounds each corner with a radius-`R` fillet arc tangent to both
legs (symmetric about the waypoint), which keeps the entry/exit headings exact.
`build_full_path` prepends `O` and appends `T`; `turn_markers` returns each arc's
start/end and signed turn angle.

> Note: a circular arc of radius `R` cannot both pass *through* the corner
> waypoint and preserve the leg headings, so the arc rounds the corner. Keeping
> the headings exact is what makes the approach to the target correct.

## Key parameters (`config.py`)

Units are **meters** and **radians** in the algorithms; angles in `config` are
stored in **degrees** and converted (`ALPHA_MAX_RAD`, `deg_to_rad`).

| Constant | Default | Meaning |
|---|---|---|
| `R` | 8000 m | turn radius (fixed) |
| `ALPHA_MAX` | 90° | maximum turn angle per waypoint |
| `L0` | 4000 m | post-launch stabilisation distance |
| `DSS` | 23000 m | seeker lock / terminal guidance distance |
| `SAFE_MARGIN` | 10000 m | safety buffer added to every obstacle |
| `MAP_WIDTH` / `MAP_HEIGHT` | 500000 m | map is 500 km × 500 km |
| `MAX_ITERATIONS` | 50000 | A* iteration cap |
| `TIME_BUDGET_S` | 0.9 | wall-clock search budget (`None` = unlimited) |
| `GOAL_THRESHOLD` | 1000 m | goal-reached distance |
| `TURN_PENALTY_WEIGHT` | 4000 m/rad | cost added per radian of turning |

The GUI can override these per run.

## Scenarios

`core.map_generator.get_all_scenarios()` returns the 16 presets, grouped by
difficulty: 01–04 baseline, 05–08 easy, 09–12 medium, 13–16 hard. Some of the
hardest maps have no feasible approach under the turn constraints and correctly
report no path rather than drawing an infeasible route.

## Testing

The pytest suite lives in `tests/`. `pytest.ini` sets `pythonpath = .` so tests
can import the top-level packages, and `testpaths = tests`.

```bash
python -m pytest -q
```

Test files are named `*_test.py` (committed). `test_*.py` is gitignored — use that
prefix for throwaway scratch tests. The GUI widget tests skip automatically when
no display is available.
