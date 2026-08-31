# Kế hoạch Triển khai: Chuẩn hóa Import Alias, Full Path trong plan_trajectory, và Tính chiều dài Quỹ đạo theo Cung lượn Dubins

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Chuẩn hóa toàn bộ import aliases (`su`, `ag`, `pv`, `tr`, `mg`, `prep`), nâng cấp `plan_trajectory` trả về Full Path $[O, W_1, \dots, W_{n-1}, T]$, và hiện thực hàm tính chiều dài đường bay thực tế theo cung lượn Dubins.

**Architecture:**
- **Import Cleanup:** Thay thế các alias cũ bằng import module hoặc import hàm tường minh.
- **Dubins Path Length:** Thêm `calculate_dubins_path_length` và `calculate_polyline_length` vào `src/path_planning/geometry/spatial.py`.
- **Full Path Return:** `KinodynamicAstar.plan` trả về `result["path"] = full`, cập nhật các hàm sử dụng ở `src/service/vtx_service/planner.py`, `src/path_planning/render/`, và các file tests.

**Tech Stack:** Python 3.10+, Shapely, NumPy, Pytest, Ruff, Pyright.

**Spec:** `docs/superpowers/specs/2026-08-31-full-path-dubins-length-and-import-cleanup.md`

## Global Constraints
- Tuân thủ quy chuẩn kỹ thuật `docs/coding_standards_extracted.txt`: AAA pattern, Type annotations đầy đủ, Google-style docstrings tiếng Việt, 0 lỗi ruff, 0 cảnh báo pyright.
- Tỷ lệ vượt qua kiểm thử: 100% Passed.

---

### Task 1: Chuẩn hóa Import Alias trong `src/`, `tests/`, và `scripts/`

**Files:**
- Modify: `src/path_planning/planner.py`
- Modify: `src/path_planning/collision/detector.py`
- Modify: `src/path_planning/geometry/arc.py`
- Modify: `src/path_planning/render/visualizer.py`
- Modify: `src/path_planning/scenario/generator.py`
- Modify: `src/path_planning/scenario/preprocessing.py`
- Modify: `src/path_planning/search/astar.py`
- Modify: `src/path_planning/search/successors.py`
- Modify: `src/service/vtx_service/planner.py`
- Modify: `scripts/ab_planners.py`, `scripts/batch_random_test.py`, `scripts/goal_shot_ab.py`

**Interfaces:**
- Thay `from path_planning.geometry import spatial as su` thành `from path_planning.geometry import spatial`
- Thay `from path_planning.geometry import arc as ag` thành `from path_planning.geometry import arc`
- Thay `from path_planning.validation import oracle as pv` thành `from path_planning.validation import oracle`
- Thay `from path_planning.render import trajectory as tr` / `sampling as tr` thành `from path_planning.render import sampling`
- Thay `from path_planning.scenario import preprocessing as prep` thành `from path_planning.scenario import preprocessing`
- Thay `from path_planning.scenario import generator as mg` thành `from path_planning.scenario import generator`

- [ ] **Step 1: Cập nhật imports trong `src/path_planning/`**
- [ ] **Step 2: Cập nhật imports trong `src/service/` và `scripts/`**
- [ ] **Step 3: Chạy linter và pytest để kiểm tra**
  Run: `ruff check . && pytest tests/ -q`
- [ ] **Step 4: Commit Task 1**
  Run: `git add -u && git commit -m "refactor(imports): replace legacy abbreviation aliases with explicit module imports"`

---

### Task 2: Hiện thực Hàm Tính Chiều dài Quỹ đạo theo Cung lượn Dubins

**Files:**
- Modify: `src/path_planning/geometry/spatial.py`
- Test: `tests/path_planning/unit/geometry/test_spatial.py`
- Modify: `src/service/vtx_service/planner.py`
- Test: `tests/service/unit/` & `tests/service/integration/`

**Interfaces:**
- Produces:
  ```python
  def calculate_dubins_path_length(
      path: Sequence[PlannerState], turn_radius: float
  ) -> float: ...
  def calculate_polyline_length(path: Sequence[PlannerState]) -> float: ...
  ```

- [ ] **Step 1: Viết Unit Test cho `calculate_dubins_path_length` và `calculate_polyline_length`**
- [ ] **Step 2: Hiện thực `calculate_dubins_path_length` và `calculate_polyline_length` trong `spatial.py`**
- [ ] **Step 3: Cập nhật `src/service/vtx_service/planner.py` sử dụng `calculate_dubins_path_length`**
- [ ] **Step 4: Chạy pytest xác nhận các test tính chiều dài vượt qua**
  Run: `pytest tests/path_planning/unit/geometry/test_spatial.py tests/service/ -v`
- [ ] **Step 5: Commit Task 2**
  Run: `git add -u && git commit -m "feat(geometry): add dubins arc path length calculation and integrate into service"`

---

### Task 3: Cập nhật `plan_trajectory` Trả về Full Path $[O, \dots, T]$ & Đồng bộ Caller

**Files:**
- Modify: `src/path_planning/planner.py`
- Modify: `src/service/vtx_service/planner.py`
- Modify: `src/path_planning/render/sampling.py`
- Modify: `src/path_planning/render/visualizer.py`
- Modify: `tests/path_planning/integration/test_planner_pipeline.py`
- Modify: `tests/path_planning/integration/test_preset_benchmarks.py`
- Modify: `tests/path_planning/integration/test_time_budget.py`
- Modify: `tests/service/integration/test_equivalence.py`
- Modify: `tests/service/integration/test_planner_service.py`

**Interfaces:**
- `KinodynamicAstar.plan` -> returns `result["path"] = full` ($O \to T$) khi thành công.
- `_waypoints_out` trong `vtx_service/planner.py` -> ánh xạ trực tiếp `result["path"]`.
- `sample_trajectory` -> nhận `full` trực tiếp.

- [ ] **Step 1: Cập nhật `KinodynamicAstar.plan()` trong `planner.py` trả về `result["path"] = full`**
- [ ] **Step 2: Cập nhật `vtx_service/planner.py` và `render/` loại bỏ wrapper `build_full_path` dư thừa**
- [ ] **Step 3: Cập nhật các test integration trong `tests/` trực tiếp kiểm tra `result["path"]`**
- [ ] **Step 4: Chạy full pytest suite**
  Run: `pytest tests/ -v`
- [ ] **Step 5: Commit Task 3**
  Run: `git add -u && git commit -m "refactor(planner): return full mission path in plan_trajectory and unify callers"`

---

### Task 4: Kiểm tra Toàn diện, Định dạng & Xác thực Hệ thống

**Files:**
- Toàn bộ codebase

- [ ] **Step 1: Chạy `ruff format .`**
- [ ] **Step 2: Chạy `ruff check . --fix`**
- [ ] **Step 3: Chạy `pyright`**
- [ ] **Step 4: Chạy `python -m pytest tests/ -v`**
- [ ] **Step 5: Commit Task 4**
  Run: `git add -u && git commit -m "chore: final formatting, typing, and test suite verification"`
