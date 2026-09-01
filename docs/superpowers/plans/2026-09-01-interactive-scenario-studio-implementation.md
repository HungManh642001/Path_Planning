# Interactive Scenario Studio Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Triển khai tính năng **Interactive Scenario Studio** cho phép đội kiểm thử tạo, thêm, xóa chướng ngại vật tròn & đảo đa giác, cũng như di chuyển điểm Start/Goal trực tiếp trên bản đồ tương tác 2D.

**Architecture:** Mở rộng `scenario_custom.py` với các hàm thao tác hình học bất biến; nâng cấp `visualizer_2d.py` hỗ trợ thanh công cụ vẽ hình của Plotly; tích hợp bộ bắt sự kiện click/selection và thanh công cụ 5 chế độ tương tác vào `tab_inspector.py` với quản lý trạng thái `st.session_state`.

**Tech Stack:** Python 3.10+, Streamlit (>=1.35 `st.plotly_chart` `on_select="rerun"`), Plotly Graph Objects, Pytest.

**Spec:** [docs/superpowers/specs/2026-09-01-interactive-scenario-studio-design.md](file:///mnt/d/Workspace/VTX/Path_Planning/docs/superpowers/specs/2026-09-01-interactive-scenario-studio-design.md)

## Global Constraints
- Hệ tọa độ metric Cartesian 2D ($X, Y$) tính bằng mét với tỷ lệ co giãn $1:1$ (`scaleratio=1`).
- Tất cả hàm helper thao tác trên `Scenario` phải đảm bảo tính bất biến (pure functions), không làm mutate input dict.
- Tất cả mã nguồn mới phải đạt chuẩn nghiêm ngặt: strict typing (`pyright`), docstrings tiếng Việt chuẩn Google, `ruff check` và `ruff format` 0 lỗi.
- Test suite đạt 100% pass trên toàn bộ hệ thống.

---

### Task 1: Scenario Mutation Helpers in `scenario_custom.py`

**Files:**
- Modify: `src/tools/qa_suite/core/scenario_custom.py`
- Modify: `src/tools/qa_suite/core/__init__.py`
- Modify: `tests/tools/unit/test_scenario_custom.py`

**Interfaces:**
- Produces:
  - `add_circle_obstacle(scenario: Scenario, center: tuple[float, float], radius: float) -> Scenario`
  - `add_polygon_obstacle(scenario: Scenario, vertices: list[tuple[float, float]]) -> Scenario`
  - `remove_last_obstacle(scenario: Scenario) -> Scenario`
  - `clear_all_obstacles(scenario: Scenario) -> Scenario`
  - `update_start_position(scenario: Scenario, start: tuple[float, float], heading_rad: float | None = None) -> Scenario`
  - `update_goal_position(scenario: Scenario, goal: tuple[float, float], heading_rad: float | None = None) -> Scenario`

- [ ] **Step 1: Write failing unit tests for new scenario manipulation helpers**

In `tests/tools/unit/test_scenario_custom.py`:
```python
def test_scenario_mutation_helpers() -> None:
    scenario = build_custom_scenario(
        start=(10000.0, 10000.0),
        start_heading=0.0,
        goal=(90000.0, 90000.0),
        goal_heading=None,
    )
    # Add circle
    s2 = add_circle_obstacle(scenario, center=(50000.0, 50000.0), radius=15000.0)
    assert len(s2["dynamic_obstacles"]) == 1
    assert len(s2["obstacles"]) == 1
    assert scenario["dynamic_obstacles"] == []  # Immutability check

    # Add polygon
    poly = [(20000.0, 20000.0), (30000.0, 20000.0), (25000.0, 30000.0)]
    s3 = add_polygon_obstacle(s2, vertices=poly)
    assert len(s3["islands"]) == 1
    assert len(s3["obstacles"]) == 2

    # Remove last obstacle
    s4 = remove_last_obstacle(s3)
    assert len(s4["obstacles"]) == 1

    # Clear all obstacles
    s5 = clear_all_obstacles(s3)
    assert len(s5["obstacles"]) == 0
    assert len(s5["dynamic_obstacles"]) == 0
    assert len(s5["islands"]) == 0

    # Update Start and Goal
    s6 = update_start_position(s5, start=(12000.0, 14000.0), heading_rad=1.57)
    assert s6["start"] == (12000.0, 14000.0)
    assert s6["start_heading"] == 1.57

    s7 = update_goal_position(s6, goal=(85000.0, 88000.0), heading_rad=0.0)
    assert s7["goal"] == (85000.0, 88000.0)
    assert s7["goal_heading"] == 0.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/tools/unit/test_scenario_custom.py -k test_scenario_mutation_helpers -v`
Expected: FAIL (ImportError / NameError)

- [ ] **Step 3: Implement helper functions in `scenario_custom.py` and export in `__init__.py`**

In `src/tools/qa_suite/core/scenario_custom.py`:
Implement `add_circle_obstacle`, `add_polygon_obstacle`, `remove_last_obstacle`, `clear_all_obstacles`, `update_start_position`, `update_goal_position`.

- [ ] **Step 4: Run tests and verify they pass**

Run: `pytest tests/tools/unit/test_scenario_custom.py -v`
Expected: 100% PASS

- [ ] **Step 5: Commit**

```bash
git add src/tools/qa_suite/core/ tests/tools/unit/test_scenario_custom.py
git commit -m "feat(qa): implement scenario mutation and geometry manipulation helpers"
```

---

### Task 2: Plotly Native Drawing ModeBar & Draft Polygon Rendering in `visualizer_2d.py`

**Files:**
- Modify: `src/tools/qa_suite/core/visualizer_2d.py`
- Modify: `tests/tools/unit/test_visualizer_2d.py`

**Interfaces:**
- Updates `PlotlyVisualizer2D.create_scenario_figure()`:
  - Accepts `draft_polygon_vertices: list[tuple[float, float]] | None = None`
  - Adds draft vertex scatter markers and dashed polyline trace when draft vertices exist.
  - Configures default shape styling for Plotly freehand draw: `layout.newshape = dict(line=dict(color="#f59e0b", width=2), fillcolor="rgba(245, 158, 11, 0.2)")`.

- [ ] **Step 1: Write unit test for draft polygon rendering in `test_visualizer_2d.py`**

```python
def test_create_scenario_figure_with_draft_polygon_vertices() -> None:
    scenario = get_all_scenarios()["scenario_01_open_ocean"]()
    draft = [(100000.0, 100000.0), (150000.0, 100000.0)]
    fig = PlotlyVisualizer2D.create_scenario_figure(
        scenario=scenario,
        draft_polygon_vertices=draft,
    )
    # Check that draft scatter trace is present
    trace_names = [t.name for t in fig.data if hasattr(t, "name")]
    assert any("Draft Polygon" in str(name) for name in trace_names)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/tools/unit/test_visualizer_2d.py -k test_create_scenario_figure_with_draft_polygon_vertices -v`
Expected: FAIL

- [ ] **Step 3: Implement draft polygon rendering and newshape layout in `visualizer_2d.py`**

In `src/tools/qa_suite/core/visualizer_2d.py`:
- Add `draft_polygon_vertices` parameter to `create_scenario_figure`.
- Add Scatter trace for draft vertices with dashed line and diamond markers.
- Configure `layout.newshape` for interactive Plotly drawings.

- [ ] **Step 4: Run tests and verify they pass**

Run: `pytest tests/tools/unit/test_visualizer_2d.py -v`
Expected: 100% PASS

- [ ] **Step 5: Commit**

```bash
git add src/tools/qa_suite/core/visualizer_2d.py tests/tools/unit/test_visualizer_2d.py
git commit -m "feat(qa): support draft polygon vertices and shape draw styling in PlotlyVisualizer2D"
```

---

### Task 3: Interactive Scenario Studio Toolbar & Selection Handler in `tab_inspector.py`

**Files:**
- Modify: `src/tools/qa_suite/views/tab_inspector.py`
- Modify: `tests/tools/unit/test_views.py`

**Interfaces:**
- In `tab_inspector.py`:
  - Renders 5 interactive modes:
    1. `🔍 Inspect (Pan/Zoom)`
    2. `⭕ Quick Add Circle` (with radius slider $R_{new}$)
    3. `📐 Draw Polygon` (with vertex accumulator and *"Finish Polygon"* / *"Discard Polygon"* buttons)
    4. `🟢 Relocate Start (O)`
    5. `🔴 Relocate Goal (T)`
  - Renders Action Bar:
    - `↩️ Undo Last Obstacle`
    - `🗑️ Clear All Obstacles`
    - `🔄 Reset to Preset`
  - Captures `selection = st.plotly_chart(fig, on_select="rerun", selection_mode=["points", "box"], key="inspector_map")`
  - Processes `selection["points"]` and `selection["shapes"]` to update `st.session_state["active_scenario"]`.

- [ ] **Step 1: Write smoke tests for interactive studio view in `test_views.py`**

In `tests/tools/unit/test_views.py`:
Test rendering `render_tab_inspector` with session state containing active scenario, draft vertices, and selection events.

- [ ] **Step 2: Implement Interactive Studio UI and event handler in `tab_inspector.py`**

- [ ] **Step 3: Run view tests and verify they pass**

Run: `pytest tests/tools/unit/test_views.py -v`
Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add src/tools/qa_suite/views/tab_inspector.py tests/tools/unit/test_views.py
git commit -m "feat(qa): implement interactive Studio toolbar, selection handler and obstacle manipulation in Tab 1"
```

---

### Task 4: Full Suite Verification & Final Polish

**Files:**
- All modified and existing files in `src/` and `tests/`.

- [ ] **Step 1: Run static analysis checks**

```bash
ruff format .
ruff check .
pyright
```
Expected: 0 errors, 0 warnings.

- [ ] **Step 2: Run complete pytest test suite**

```bash
pytest tests/ -v
```
Expected: 245+ tests pass (100% green).

- [ ] **Step 3: Smoke test CLI and Streamlit app**

```bash
python -m tools.qa_suite.cli run-presets --target local
```

- [ ] **Step 4: Commit any final polish**

```bash
git add -A
git commit -m "test(qa): verify full test suite with interactive scenario studio"
```
