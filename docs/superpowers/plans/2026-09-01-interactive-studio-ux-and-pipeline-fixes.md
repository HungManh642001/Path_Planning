# Interactive Scenario Studio UX & Pipeline Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Resolve map click unresponsiveness, ModeBar polygon vertex bloat, ellipse drawing distortions, redundant line tools, and scenario state desynchronization during run planning in the QA Suite Interactive Scenario Studio.

**Architecture:** Add Ramer-Douglas-Peucker (RDP) polygon decimation and single-obstacle removal to `scenario_custom.py`. Inject a transparent interactive click grid in `visualizer_2d.py`. Refactor `tab_inspector.py` to support dual-input controls (map click + numeric coordinate inputs/buttons), granular obstacle inventory list with per-obstacle deletion, and robust single-source-of-truth scenario execution.

**Tech Stack:** Python 3.10/3.11, Plotly Graph Objects, Streamlit, Pytest, Pyright, Ruff.

**Spec:** `docs/superpowers/specs/2026-09-01-interactive-studio-ux-and-pipeline-fixes.md`

## Global Constraints
- Strict typing throughout (`pyright` compliant with 0 errors).
- All geometry transformations must be pure and immutable (using `_clone_scenario`).
- 100% passing tests on full test suite (248+ tests).
- 0 lint/formatting errors (`ruff format .` and `ruff check .`).

---

### Task 1: Core Geometry & Mutation Helpers (`scenario_custom.py`)

**Files:**
- Modify: `src/tools/qa_suite/core/scenario_custom.py`
- Modify: `src/tools/qa_suite/core/__init__.py`
- Test: `tests/tools/unit/test_scenario_custom.py`

**Interfaces:**
- Produces:
  - `simplify_polygon_rdp(vertices: list[tuple[float, float]], epsilon: float = 3000.0) -> list[tuple[float, float]]`
  - `remove_obstacle_by_index(scenario: Scenario, obstacle_index: int) -> Scenario`

- [ ] **Step 1: Write the failing tests in `test_scenario_custom.py`**

```python
def test_simplify_polygon_rdp_reduces_collinear_and_noisy_vertices() -> None:
    # A straight line with noisy intermediate points
    raw_poly = [
        (0.0, 0.0), (100.0, 50.0), (200.0, -30.0), (1000.0, 0.0),
        (1000.0, 1000.0), (0.0, 1000.0)
    ]
    simplified = simplify_polygon_rdp(raw_poly, epsilon=100.0)
    assert len(simplified) < len(raw_poly)
    assert len(simplified) >= 3

def test_remove_obstacle_by_index_removes_correct_obstacle() -> None:
    s = build_custom_scenario(
        start=(0.0, 0.0), start_heading=0.0, goal=(100.0, 100.0), goal_heading=None
    )
    s = add_circle_obstacle(s, (10.0, 10.0), 5.0)
    s = add_circle_obstacle(s, (20.0, 20.0), 6.0)
    s = add_polygon_obstacle(s, [(30.0, 30.0), (40.0, 30.0), (35.0, 40.0)])
    assert len(s["obstacles"]) == 3
    # Remove index 1 (the second circle)
    s_new = remove_obstacle_by_index(s, 1)
    assert len(s_new["obstacles"]) == 2
    assert s_new["obstacles"][0]["center"] == (10.0, 10.0)
    assert s_new["obstacles"][1]["type"] == "polygon"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/tools/unit/test_scenario_custom.py -k "test_simplify_polygon_rdp or test_remove_obstacle_by_index" -v`
Expected: FAIL with Import/NameError.

- [ ] **Step 3: Implement RDP simplification and `remove_obstacle_by_index` in `scenario_custom.py`**

Implement point-to-segment distance and recursive RDP splitting for 2D points, ensuring polygon closure and minimum 3 vertices.
Implement `remove_obstacle_by_index` to remove the item at index in `obstacles` and reconcile with `dynamic_obstacles` and `islands`.
Export both functions in `src/tools/qa_suite/core/__init__.py`.

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/tools/unit/test_scenario_custom.py -v`
Expected: PASS.

- [ ] **Step 5: Commit changes**

```bash
git add src/tools/qa_suite/core/ tests/tools/unit/test_scenario_custom.py
git commit -m "feat(qa): implement polygon RDP simplification and granular obstacle removal"
```

---

### Task 2: Plotly Visualizer 2D Click Surface & ModeBar Refinement (`visualizer_2d.py`)

**Files:**
- Modify: `src/tools/qa_suite/core/visualizer_2d.py`
- Test: `tests/tools/unit/test_visualizer_2d.py`

**Interfaces:**
- Modifies: `PlotlyVisualizer2D.create_scenario_figure(..., enable_click_grid: bool = True)`

- [ ] **Step 1: Write the failing tests in `test_visualizer_2d.py`**

```python
def test_create_scenario_figure_with_click_grid() -> None:
    scenario = get_all_scenarios()["scenario_01_open_ocean"]()
    fig = PlotlyVisualizer2D.create_scenario_figure(scenario, enable_click_grid=True)
    trace_names = [t.name for t in fig.data if hasattr(t, "name")]
    assert "Map Canvas Grid" in trace_names
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/tools/unit/test_visualizer_2d.py -k "test_create_scenario_figure_with_click_grid" -v`
Expected: FAIL.

- [ ] **Step 3: Implement click grid trace in `visualizer_2d.py`**

Generate a transparent regular grid of scatter points (spacing e.g. $25{,}000\text{ m}$) spanning `[0, map_w] x [0, map_h]` with `marker=dict(size=8, color="rgba(0,0,0,0.001)")` and informative hovertext showing $(X, Y)$ coords in km.
Add `enable_click_grid: bool = True` parameter to `create_scenario_figure`.

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/tools/unit/test_visualizer_2d.py -v`
Expected: PASS.

- [ ] **Step 5: Commit changes**

```bash
git add src/tools/qa_suite/core/visualizer_2d.py tests/tools/unit/test_visualizer_2d.py
git commit -m "feat(qa): add transparent interactive click mesh to Plotly 2D visualizer"
```

---

### Task 3: Interactive Scenario Studio Dual-Input Controls, Obstacle Manager & Pipeline Sync (`tab_inspector.py`)

**Files:**
- Modify: `src/tools/qa_suite/views/tab_inspector.py`
- Test: `tests/tools/unit/test_views.py`

**Interfaces:**
- Consumes: `simplify_polygon_rdp`, `remove_obstacle_by_index`, `add_circle_obstacle`, `add_polygon_obstacle`, `update_start_position`, `update_goal_position`.

- [ ] **Step 1: Write unit tests in `test_views.py`**

Test:
- Direct numeric input addition for circles (`Center X`, `Center Y`, `Radius`, `Add Circle Button`).
- Direct numeric input addition for polygon vertices (`Add Vertex Button`, `Complete Polygon Button`).
- Individual obstacle removal via `remove_obstacle_by_index`.
- ModeBar `drawclosedpath` RDP polygon simplification.
- Removal of `drawline` from `plotly_config["modeBarButtonsToAdd"]`.

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/tools/unit/test_views.py -k "test_render_tab_inspector" -v`

- [ ] **Step 3: Implement Dual-Input Controls, Obstacle Inventory List, and Sync in `tab_inspector.py`**

1. **ModeBar Config:** Set `modeBarButtonsToAdd=["drawcircle", "drawclosedpath", "eraseshape"]` (removing `drawline`).
2. **Dual-Input Mode in Studio Toolbar:**
   - For `⭕ Add Circle`: Add numeric inputs `Center X`, `Center Y`, `Radius` with `➕ Add Circle` button, alongside click handler.
   - For `📐 Add Polygon`: Add numeric inputs `Vertex X`, `Vertex Y` with `➕ Add Vertex` button, a draft vertex list, `🗑️ Remove Last Vertex`, and `✅ Complete Polygon` (which applies `simplify_polygon_rdp`).
   - For `🚀 Move Start` & `🎯 Move Goal`: Add numeric inputs for coordinates and headings with instant update.
3. **Obstacle Inventory Manager:** Render an expander `📋 Active Obstacles Inventory` with a list of all obstacles and an individual `❌ Delete` button for each obstacle index.
4. **Pipeline Synchronization:** Ensure `scenario_dict_repr` includes all vehicle parameters, ensuring `driver.run_scenario` runs on the live `active_scenario`.

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/tools/unit/test_views.py -v`
Expected: PASS.

- [ ] **Step 5: Commit changes**

```bash
git add src/tools/qa_suite/views/tab_inspector.py tests/tools/unit/test_views.py
git commit -m "feat(qa): implement dual-input studio controls, RDP smoothing and obstacle inventory manager"
```

---

### Task 4: Comprehensive Test Verification & CLI Smoke Test

**Files:**
- Test: Full repository test suite (`tests/`)

- [ ] **Step 1: Run static checks**

Run: `ruff format . && ruff check . && pyright`
Expected: 0 errors, 0 warnings.

- [ ] **Step 2: Run full test suite**

Run: `pytest tests/ -v`
Expected: 100% tests pass (248+ tests).

- [ ] **Step 3: CLI Presets Smoke Test**

Run: `python -m tools.qa_suite.cli run-presets --target local`
Expected: 18/18 PASS 100%.

- [ ] **Step 4: Commit any final polish**

```bash
git add -A
git commit -m "test(qa): verify full test suite with upgraded Interactive Studio UX"
```
