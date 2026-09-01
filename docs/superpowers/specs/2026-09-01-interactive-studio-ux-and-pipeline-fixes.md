# Interactive Scenario Studio UX & Pipeline Fixes Design Specification

## 1. Context & Motivation
Testing and real-world usage of the QA Suite Interactive Scenario Studio revealed three operational friction points:
1. **Map Canvas Click Unresponsiveness:** Plotly in Streamlit only captures selection events when clicking existing scatter data points. Clicks on the empty canvas/ocean background do not emit events, freezing the 5 interaction modes.
2. **ModeBar Drawing Usability Issues:**
   - `drawcircle` produces freeform ellipses during dragging rather than perfect circles.
   - `drawclosedpath` produces jittery freehand paths with 50–100 micro-vertices, overwhelming the path planner and making vertex inspection impossible.
   - `drawline` is redundant as 1D open lines cannot act as 2D obstacles.
   - Lack of Start/Goal movement tools in ModeBar.
3. **Execution Pipeline Desynchronization:** Newly constructed/edited maps were not properly synchronized with the planner execution when clicking "🚀 Run Planning".

---

## 2. Architectural Design & Solutions

### A. Clickable Canvas Mesh & Dual-Input Interaction
To solve canvas click unresponsiveness while providing precise input:
1. **Interactive Click Surface in `visualizer_2d.py`:**
   - Inject a transparent scatter grid trace covering map bounds $([0, W] \times [0, H])$ with step spacing (e.g., $10{,}000\text{ m}$ to $25{,}000\text{ m}$ spacing) or click boundary vertices.
   - Any click on the canvas lands on a valid scatter point and invokes selection.
2. **Dual-Input Mode in `tab_inspector.py`:**
   - Each mode (`Add Circle`, `Add Polygon`, `Move Start`, `Move Goal`) provides explicit $(X, Y)$ coordinate inputs and action buttons in addition to map click events.
   - Users can either click the map or type exact coordinates and click `➕ Add`.

### B. Geometry Simplification & Granular Obstacle Management
1. **Ramer-Douglas-Peucker (RDP) Polygon Simplification (`scenario_custom.py`):**
   - Implement `simplify_polygon_rdp(vertices, epsilon=3000.0)` to reduce noisy 100-vertex freehand paths to clean 3–8 vertex geometric polygons.
2. **Granular Obstacle Deletion (`scenario_custom.py`):**
   - Implement `remove_obstacle_by_index(scenario, index)` to delete specific obstacles without wiping the entire map.
3. **Obstacle Inventory Manager in `tab_inspector.py`:**
   - Render a list of all active obstacles with coordinate summaries and individual `❌ Delete` buttons.
4. **ModeBar Cleanup:**
   - Remove `drawline` from `modeBarButtonsToAdd`, keeping only `["drawcircle", "drawclosedpath", "eraseshape"]`.
   - On `drawcircle`, force isotropic circle calculation $R = \max(\Delta X, \Delta Y) / 2$.

### C. State Synchronization & Run Planning Execution
1. Single source of truth: `st.session_state["active_scenario"]`.
2. Compute `scenario_fingerprint` (hash of endpoints, vehicle constraints, safe_margin, obstacles).
3. When `Run Planning` is clicked or scenario changes, guarantee `driver.run_scenario` executes against the exact `st.session_state["active_scenario"]`.

---

## 3. Interfaces & Function Signatures

### `src/tools/qa_suite/core/scenario_custom.py`:
```python
def simplify_polygon_rdp(
    vertices: list[tuple[float, float]],
    epsilon: float = 3000.0,
) -> list[tuple[float, float]]:
    """Rút gọn đỉnh đa giác bằng thuật toán Ramer-Douglas-Peucker (RDP)."""
    ...

def remove_obstacle_by_index(
    scenario: Scenario,
    obstacle_index: int,
) -> Scenario:
    """Xóa vật cản tại vị trí chỉ mục cụ thể trong kịch bản (immutable)."""
    ...
```

### `src/tools/qa_suite/core/visualizer_2d.py`:
```python
def create_scenario_figure(
    scenario: Scenario,
    result: QAResult | None = None,
    turn_radius: float = config.R,
    show_fillet_arcs: bool = True,
    safe_margin: float = config.SAFE_MARGIN,
    show_buffer: bool = True,
    draft_polygon_vertices: list[tuple[float, float]] | None = None,
    dragmode: str = "pan",
    enable_click_grid: bool = True,
) -> go.Figure: ...
```

---

## 4. Verification & Testing Strategy
- **Unit Tests (`test_scenario_custom.py`):** Test RDP simplification on noisy paths, test granular obstacle deletion.
- **Unit Tests (`test_visualizer_2d.py`):** Test click mesh trace generation, ModeBar configuration.
- **Unit Tests (`test_views.py`):** Test Dual-Input mode (numeric inputs and buttons for circle, polygon, start, goal), test obstacle deletion by index, test Run Planning with modified scenario.
- **Regression Suite:** Run all 248+ tests, ensuring 0 Ruff linting errors and 0 Pyright errors.
