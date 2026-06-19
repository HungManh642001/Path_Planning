# Dubins Trajectory Rendering + Interactive GUI Redesign — Design Spec

**Date:** 2026-06-18
**Status:** Approved (brainstorming), pending implementation plan

## Goal

Two related features for the missile path planner:

1. **True Dubins trajectory rendering.** After the planner finds waypoints, render the *actual* flight path using proper Dubins curves (line + radius-R arcs), with a toggle between "connect waypoints with straight lines" and "Dubins curve (real trajectory)".
2. **Redesigned interactive GUI** (`launch_gui.py`) that lets a user configure all input parameters, build a scenario, run the planner, and read a results summary (distance, waypoints, runtime, …) — in a clean, scientific, easy-to-use layout.

Suboptimal-but-valid paths remain acceptable; this work is about *rendering fidelity* and *usability*, not changing the search.

## Background / current state

- The planner (`kinodynamic_astar.plan_trajectory`) returns a list of `(waypoint, heading)` tuples. Each waypoint's heading is the **arrival direction** (direction of the segment into it). The planner's validated flight model is **straight đoản trình segments + radius-R turn arcs at each waypoint** (`preprocessing.validate_kinodynamics`).
- `dubins_curves.py` exists but is a **broken placeholder**: only 4 path types (LSR/RSL/LRL/RRL, missing the common LSL/RSR/RLR), and `_sample_at_distance` does **linear interpolation** for LSR/RSL and returns `None` for the others — so the old renderer dropped whole segments, making the drawn line appear to "jump" between waypoints.
- `spatial_utils.arc_line_trajectory(points, R)` (added recently) renders straight legs + symmetric radius-R corner arcs. It is correct and continuous; it will be retained as an internal fallback.
- `visualizer.plot_scenario` currently calls `arc_line_trajectory`.
- `gui_scenario_builder.py` (687 lines) is the existing GUI: parameter sliders (R, α_max, safe margin, launch/approach angle, DSS, L0), click-to-place start/goal, click-to-draw polygons/circles, a "Run" button, a status text box, and a matplotlib canvas. It is monolithic and will be replaced.
- Equivalence: with the planner's consistent arrival-direction headings, the shortest Dubins path between consecutive `(pos, heading)` states is exactly the validated "turn-at-waypoint + straight" maneuver — so true Dubins rendering matches the validated model (no divergence).

## Part 1 — Dubins curves + trajectory rendering (headless, testable)

### `dubins_curves.py` (rewrite)

A correct Dubins shortest-path solver between two configurations `(x, y, θ)` with turn radius `R`.

- `class DubinsPath(start_pos, start_heading, goal_pos, goal_heading, radius)`.
- Compute all six Dubins words: **LSL, RSR, LSR, RSL, RLR, LRL** (CSC family: LSL, RSR, LSR, RSL; CCC family: RLR, LRL). Each returns `{'type', 'length', 't', 'p', 'q'}` (the three segment lengths in normalized form) or `None` if geometrically infeasible.
- `shortest_path()` → the minimum-length feasible word.
- `sample_path(step)` → list of `(x, y, heading)` sampled **along the actual arc/line geometry** at arc-length `step` (NOT linear interpolation). Arc segments use the turning-circle centre; straight segments interpolate linearly along the tangent.
- Use the standard normalized formulation (translate/rotate so start is at origin heading +x, scale by `1/R`, `d = dist/R`); reconstruct world points by inverting the transform.

### `trajectory.py` (new)

Single source of truth for "turn waypoints into a drawable polyline", used by both the visualizer and the GUI.

```
sample_trajectory(path, R, mode='dubins', step=None) -> list[(x, y)]
```

- `path`: list of `(waypoint, heading)` tuples (planner output).
- `mode='straight'`: return the waypoint positions as-is (straight polyline through the waypoints).
- `mode='dubins'`: for each consecutive pair, build the shortest Dubins path and concatenate its samples (dropping the duplicated join point). If a segment's Dubins solve fails or returns empty, **fall back to `spatial_utils.arc_line_trajectory` for that pair** (and ultimately to the straight segment), so the result is always continuous — never a gap/jump.
- `step` default derived from `R` (e.g. `R/8`) so arcs look smooth at map scale.
- Returns a flat continuous list of `(x, y)`.

### `visualizer.py`

- `plot_scenario(..., trajectory_mode='dubins')`: new keyword, default `'dubins'`. Render the trajectory by calling `trajectory.sample_trajectory(path, R, mode=trajectory_mode)`.
- Legend label reflects the mode ("Dubins Trajectory" / "Straight Segments").
- The `arc_line_trajectory` direct call is removed from the visualizer (the function stays in `spatial_utils` as the Dubins fallback).

## Part 2 — GUI redesign (`gui/` package)

### Layout (3 columns)

```
+----------+-------------------+----------+
| CONFIG   |   MAP 500x500km   | RESULTS  |
| scenario |   trajectory       | metrics  |
| params   |   canvas           | render   |
| advanced |                    | toggle   |
| [RUN]    |                    | log      |
+----------+-------------------+----------+
```

### Modules

- `gui/app.py` — `PlannerApp(root)` orchestrator. Owns application state: current scenario (start, goal, headings, obstacles), parameter values, last planning `result`, current render mode. Wires the three panels and the canvas together via callbacks. `launch_gui.py` imports and starts this.
- `gui/config_panel.py` — left column:
  - **Scenario input:** buttons for click-to-place start/goal and click-to-draw polygon/circle (delegates interaction to the canvas); numeric entry fields for precise start/goal/heading and circle centre/radius; **Load/Save scenario as JSON**; per-obstacle delete / clear-all.
  - **Tactical parameters** (sliders + numeric entry, kept in sync): R, α_max, L0, DSS, SAFE_MARGIN, launch angle, approach angle, map width/height.
  - **Run parameters:** TIME_BUDGET_S, MAX_ITERATIONS.
  - **Advanced** (collapsible, hidden by default): STATE_POS_QUANTUM, STATE_HEADING_QUANTUM_DEG, HEURISTIC_WEIGHT, TURN_PENALTY_WEIGHT, GOAL_THRESHOLD, POLYGON_MITRE_LIMIT, WRAP_STEP_M, CIRCLE_GRAZE_TOL_M, OBSTACLE_RING_SAMPLES.
  - **RUN** button.
- `gui/map_canvas.py` — centre column: matplotlib `FigureCanvasTkAgg`. Draws obstacles, start/goal, and the planned trajectory. Handles click-to-draw mouse interaction and obstacle editing. Re-renders the trajectory when the render mode changes (no re-plan).
- `gui/results_panel.py` — right column: results summary (see metrics below), the **render-mode toggle (Straight / Dubins)**, and a scrolling status log.

### Data flow

1. User edits params / builds scenario in the config panel.
2. **RUN** → `app` collects params, applies them as `config.*` overrides (single-process; see below), builds the scenario dict, calls `prep.prepare_scenario` then `astar.plan_trajectory`.
3. `app` stores the `result`, the results panel computes & shows the summary, the canvas renders obstacles + trajectory (current mode).
4. Toggling the render mode re-renders the canvas from the stored `result` only — **no re-plan**.

### Parameter override mechanism

The planner reads several tunables directly from the `config` module (e.g. `TURN_PENALTY_WEIGHT`, `WRAP_STEP_M`, `STATE_POS_QUANTUM`). The GUI runs in a single process, so before each run `app` writes the panel values onto the `config` module attributes, then calls the pipeline. Physical/tactical params that `prepare_scenario` already accepts as arguments (R, L0, DSS, safe_margin, alpha_max_rad) are passed as arguments; the rest are set on `config`. This is documented as the intended mechanism (acceptable for a single-process research tool).

### Results summary metrics

Computed by a **pure helper** (`gui/summary.py`, testable headlessly) from `(result, preprocessed, raw_obstacles, render_mode)`:

- Success (yes/no)
- Flight distance (km) — length of the rendered trajectory polyline (mode-dependent: Dubins length vs straight-segment length)
- Number of waypoints
- Number of turns (heading change > 1°)
- Max turn angle (deg)
- Planning runtime (ms)
- Search iterations
- Validity — collision-free vs **raw** obstacles (uses `path_validation.path_is_valid` with raw obstacle sets), yes/no

## Testing

- **`dubins_curves`** (`dubins_curves_test.py`): known-answer path lengths for canonical configurations; start/goal position and heading reproduced by `sample_path` endpoints; samples are continuous (no gaps); each of the 6 words is selected for an appropriate configuration; arc samples lie at radius `R` from the turning centre (proves arcs are sampled, not linearly interpolated).
- **`trajectory`** (`trajectory_test.py`): `mode='straight'` returns the waypoint polyline; `mode='dubins'` is continuous (max gap bounded), passes near each waypoint, and falls back gracefully (no exception, continuous output) on a degenerate/too-short segment.
- **`gui/summary`** (`gui/summary_test.py`): metric computation from a synthetic result/path is correct (distance, waypoints, turns, max angle, validity), independent of Tk.
- Tk widget construction is not unit-tested; GUI logic that matters is extracted into the pure `summary` helper and the param-collection logic, which are tested.

## File structure summary

**New:** `trajectory.py`, `gui/__init__.py`, `gui/app.py`, `gui/config_panel.py`, `gui/map_canvas.py`, `gui/results_panel.py`, `gui/summary.py`, plus `dubins_curves_test.py`, `trajectory_test.py`, `gui/summary_test.py`.
**Rewritten:** `dubins_curves.py`.
**Modified:** `visualizer.py` (trajectory_mode param), `launch_gui.py` (import `gui.app`).
**Removed:** `gui_scenario_builder.py` (superseded by the `gui/` package).
**Unchanged:** the planner/search (`kinodynamic_astar.py`, `preprocessing.py`, `spatial_utils.py` except `arc_line_trajectory` kept as fallback), `path_validation.py`.

## Out of scope

- No change to the search algorithm or obstacle model.
- No real-time / animated flight playback (static rendering only).
- No loading of the 16 predefined `map_generator` scenarios into the GUI (deferred; JSON load/save covers reuse).
- The `dubins_curves` solver is for *rendering*; it does not feed back into planning or collision checking.
