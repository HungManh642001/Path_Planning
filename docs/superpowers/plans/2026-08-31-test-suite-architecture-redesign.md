# Test Suite Architecture Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Redesign, restructure, and rewrite the entire test suite (`tests/`) following the Multi-Algorithm Domain Layout, Agile Test Pyramid, AAA Pattern, and Technical Standards Document (`docs/coding_standards_extracted.txt`).

**Architecture:** 
- `tests/path_planning/`: Dedicated hierarchical test tree for the path planning algorithm (`unit/` across 7 subdomains and `integration/` for end-to-end pipeline & benchmarks).
- `tests/<future_algorithm>/`: Extensible slot for subsequent algorithms.
- `tests/service/`: Hierarchical test tree for service orchestration (`unit/` and `integration/`).
- Hierarchical fixtures via `conftest.py` at root, algorithm, and service levels.

**Tech Stack:** Python 3.11, Pytest, Shapely, Matplotlib, Ruff, Pyright.

**Spec:** [`docs/superpowers/specs/2026-08-31-test-suite-architecture-redesign.md`](file:///mnt/d/Workspace/VTX/Path_Planning/docs/superpowers/specs/2026-08-31-test-suite-architecture-redesign.md)

## Global Constraints
- Every test function must follow the standard naming convention: `test_<target_function>_<input_condition>_<expected_result>`.
- Every test function must be structured with the 3 distinct blocks: `# Arrange`, `# Act`, `# Assert`.
- Every test function must include a concise Vietnamese Google Style docstring and return type annotation `-> None`.
- Determinism is mandatory: All randomized generators must specify explicit seeds. No wall-clock reliance.
- Mocking boundary rule: Only mock external boundaries (network/DDS/file I/O in service layer). Never mock domain logic.
- Target: 100% Passed (0 xfailed, 0 failed, 0 errors, 0 flaky tests).
- Code formatting & linting: Must pass `ruff check`, `ruff format --check`, and `pyright`.

---

## Task 1: Hierarchical Fixtures Setup (`tests/conftest.py`, `tests/path_planning/conftest.py`, `tests/service/conftest.py`)

**Files:**
- Create: `tests/conftest.py`
- Create: `tests/path_planning/conftest.py`
- Create: `tests/service/conftest.py`

**Interfaces & Responsibilities:**
- `tests/conftest.py`: Global fixtures (fixed seed reset, temp directories, global constants).
- `tests/path_planning/conftest.py`: Algorithm fixtures (sample point pairs, circle obstacles, polygon obstacles, empty scenarios, standard preprocessed scenarios).
- `tests/service/conftest.py`: Service fixtures (sample request payloads, mock message transport, fake runner contexts).

### Steps:
- [ ] **Step 1.1**: Create `tests/conftest.py` defining global fixtures (`fixed_seed`, `global_test_env`).
- [ ] **Step 1.2**: Create `tests/path_planning/conftest.py` defining shared path planning fixtures (`sample_origin_and_target`, `sample_circle_obstacles`, `sample_polygon_obstacles`, `sample_clean_scenario`, `sample_preprocessed_scenario`).
- [ ] **Step 1.3**: Create `tests/service/conftest.py` defining shared service fixtures (`sample_request_payload`, `sample_map_data`).
- [ ] **Step 1.4**: Run pytest to ensure fixtures load without errors:
  ```bash
  python -m pytest tests/conftest.py tests/path_planning/conftest.py tests/service/conftest.py --collect-only
  ```
- [ ] **Step 1.5**: Commit changes:
  ```bash
  git add tests/ && git commit -m "test(fixtures): setup hierarchical conftest fixtures for multi-algorithm testing"
  ```

---

## Task 2: Unit Tests for Geometry & Collision (`tests/path_planning/unit/geometry/`, `tests/path_planning/unit/collision/`)

**Files:**
- Create: `tests/path_planning/unit/geometry/test_spatial.py`
- Create: `tests/path_planning/unit/geometry/test_arc.py`
- Create: `tests/path_planning/unit/geometry/test_goal_shot.py`
- Create: `tests/path_planning/unit/collision/test_detector.py`

**Scope & Coverage:**
- `test_spatial.py`: `distance`, `angle_to_heading`, `angle_diff`, `point_to_line_distance`, `inflate_polygon`, `circle_tangent_points`.
- `test_arc.py`: `riding_sense`, `tangent_heading`, `arc_angle`, `arc_waypoints`, `departure_point`, `bitangent_departures`, `is_point_on_circle_boundary`, `is_point_on_any_circle_boundary`, `sector_polygon`, `has_angular_overlap`.
- `test_goal_shot.py`: `two_corner_candidates` geometric math, clearance verification, budget preservation.
- `test_detector.py`: `CollisionDetector` line of sight `is_collision_free`, fillet arc `is_corner_arc_clear`, annular sector `is_sector_clear`, boundary `on_circle_boundary`, safezone `is_in_bounds`.

### Steps:
- [ ] **Step 2.1**: Write `tests/path_planning/unit/geometry/test_spatial.py` following AAA pattern and Vietnamese docstrings.
- [ ] **Step 2.2**: Write `tests/path_planning/unit/geometry/test_arc.py`.
- [ ] **Step 2.3**: Write `tests/path_planning/unit/geometry/test_goal_shot.py`.
- [ ] **Step 2.4**: Write `tests/path_planning/unit/collision/test_detector.py`.
- [ ] **Step 2.5**: Run pytest to verify all geometry and collision unit tests pass:
  ```bash
  python -m pytest tests/path_planning/unit/geometry/ tests/path_planning/unit/collision/ -v
  ```
- [ ] **Step 2.6**: Commit changes:
  ```bash
  git add tests/path_planning/unit/ && git commit -m "test(path_planning): add unit tests for geometry and collision modules"
  ```

---

## Task 3: Unit Tests for Search & Trajectory (`tests/path_planning/unit/search/`, `tests/path_planning/unit/trajectory/`)

**Files:**
- Create: `tests/path_planning/unit/search/test_state.py`
- Create: `tests/path_planning/unit/search/test_heuristic.py`
- Create: `tests/path_planning/unit/search/test_successors.py`
- Create: `tests/path_planning/unit/search/test_astar.py`
- Create: `tests/path_planning/unit/trajectory/test_mission_path.py`
- Create: `tests/path_planning/unit/trajectory/test_smoothing.py`

**Scope & Coverage:**
- `test_state.py`: `State` initialization, quantization `state_to_tuple`, hashing, equality, dominance.
- `test_heuristic.py`: `euclidean_heuristic` triangle inequality, zero at goal, non-negativity.
- `test_successors.py`: `SuccessorGenerator` methods (`seed_start_corners`, `get_next_states`, `pivot_candidate`, `slide_pivot`, `try_goal_shot`).
- `test_astar.py`: `AstarSearchEngine` queue ordering, termination `is_goal_reached`, `reconstruct_path`, time budget exhaustion and diagnostic stats.
- `test_mission_path.py`: `full_mission_path` prepending $O$, appending $T$, bearing calculation for free-goal, idempotence (no duplicate points).
- `test_smoothing.py`: `smooth_path` dynamic programming (DP) shortcuts, fillet arc reserve checks, straight segment budget constraints.

### Steps:
- [ ] **Step 3.1**: Write `tests/path_planning/unit/search/test_state.py` and `test_heuristic.py`.
- [ ] **Step 3.2**: Write `tests/path_planning/unit/search/test_successors.py` and `test_astar.py`.
- [ ] **Step 3.3**: Write `tests/path_planning/unit/trajectory/test_mission_path.py` and `test_smoothing.py`.
- [ ] **Step 3.4**: Run pytest to verify all search and trajectory unit tests pass:
  ```bash
  python -m pytest tests/path_planning/unit/search/ tests/path_planning/unit/trajectory/ -v
  ```
- [ ] **Step 3.5**: Commit changes:
  ```bash
  git add tests/path_planning/unit/ && git commit -m "test(path_planning): add unit tests for search and trajectory modules"
  ```

---

## Task 4: Unit Tests for Scenario, Validation & Render (`tests/path_planning/unit/scenario/`, `validation/`, `render/`)

**Files:**
- Create: `tests/path_planning/unit/scenario/test_preprocessing.py`
- Create: `tests/path_planning/unit/scenario/test_generator.py`
- Create: `tests/path_planning/unit/scenario/test_presets.py`
- Create: `tests/path_planning/unit/validation/test_oracle.py`
- Create: `tests/path_planning/unit/render/test_sampling.py`
- Create: `tests/path_planning/unit/render/test_visualizer.py`

**Scope & Coverage:**
- `test_preprocessing.py`: `prepare_scenario`, `inflate_obstacles`, `calculate_start_state`, `calculate_end_state`.
- `test_generator.py`: `generate_random_islands`, `generate_dynamic_obstacles`, `create_scenario`, seed reproducibility.
- `test_presets.py`: `get_all_scenarios`, structure integrity of 16 preset scenarios.
- `test_oracle.py`: `ValidationResult.ok()`, `segments_clear`, `turn_angles_ok`, `straight_segments_ok`, `arcs_clear`, `path_is_valid`.
- `test_sampling.py`: `sample_trajectory` (`straight` vs `dubins`), `turn_markers`, `build_full_path`.
- `test_visualizer.py`: `plot_scenario`, content view vs map view axis extents.

### Steps:
- [ ] **Step 4.1**: Write `tests/path_planning/unit/scenario/test_preprocessing.py`, `test_generator.py`, `test_presets.py`.
- [ ] **Step 4.2**: Write `tests/path_planning/unit/validation/test_oracle.py`.
- [ ] **Step 4.3**: Write `tests/path_planning/unit/render/test_sampling.py` and `test_visualizer.py`.
- [ ] **Step 4.4**: Run pytest across all path_planning unit tests:
  ```bash
  python -m pytest tests/path_planning/unit/ -v
  ```
- [ ] **Step 4.5**: Commit changes:
  ```bash
  git add tests/path_planning/unit/ && git commit -m "test(path_planning): add unit tests for scenario, validation, and render modules"
  ```

---

## Task 5: Integration Tests for Path Planning Pipeline (`tests/path_planning/integration/`)

**Files:**
- Create: `tests/path_planning/integration/test_planner_pipeline.py`
- Create: `tests/path_planning/integration/test_preset_benchmarks.py`
- Create: `tests/path_planning/integration/test_time_budget.py`

**Scope & Coverage:**
- `test_planner_pipeline.py`: `KinodynamicAstar` and `plan_trajectory` end-to-end execution for both fixed-goal and free-goal modes, blocked takeoff detection, blocked goal detection.
- `test_preset_benchmarks.py`: Execute all 16 benchmark preset scenarios and assert that every produced path is accepted by the independent validation Oracle.
- `test_time_budget.py`: Test wall-clock time budgeting, budget exhaustion behavior, fallback to default `TIME_BUDGET_S`, clamping and negative budget rejection in `resolve_time_budget_s`.

### Steps:
- [ ] **Step 5.1**: Write `tests/path_planning/integration/test_planner_pipeline.py`.
- [ ] **Step 5.2**: Write `tests/path_planning/integration/test_preset_benchmarks.py`.
- [ ] **Step 5.3**: Write `tests/path_planning/integration/test_time_budget.py`.
- [ ] **Step 5.4**: Run pytest across all path_planning tests:
  ```bash
  python -m pytest tests/path_planning/ -v
  ```
- [ ] **Step 5.5**: Commit changes:
  ```bash
  git add tests/path_planning/integration/ && git commit -m "test(path_planning): add integration tests for planner pipeline, benchmarks, and time budget"
  ```

---

## Task 6: Refactor & Standardize Service Tests (`tests/service/unit/` and `tests/service/integration/`)

**Files:**
- Create: `tests/service/unit/test_angles.py`
- Create: `tests/service/unit/test_messages.py`
- Create: `tests/service/unit/test_map_file.py`
- Create: `tests/service/unit/test_runtime.py`
- Create: `tests/service/unit/test_scenario_builder.py`
- Create: `tests/service/integration/test_boundary.py`
- Create: `tests/service/integration/test_equivalence.py`
- Create: `tests/service/integration/test_planner_service.py`
- Create: `tests/service/integration/test_runner.py`
- Create: `tests/service/integration/test_transport.py`

**Scope & Coverage:**
- `service/unit/`: Test angle conversions, message schemas, map file parsing, runtime git detection, scenario building from payload.
- `service/integration/`: Test package boundary isolation, equivalence between direct planner call and service call, planner service worker dispatch, runner process management, DDS mock transport.

### Steps:
- [ ] **Step 6.1**: Create `tests/service/unit/` and rewrite unit test files with AAA pattern and Vietnamese docstrings.
- [ ] **Step 6.2**: Create `tests/service/integration/` and rewrite integration test files with AAA pattern and Vietnamese docstrings.
- [ ] **Step 6.3**: Run pytest across all service tests:
  ```bash
  python -m pytest tests/service/ -v
  ```
- [ ] **Step 6.4**: Commit changes:
  ```bash
  git add tests/service/ && git commit -m "test(service): reorganize service tests into unit and integration hierarchies"
  ```

---

## Task 7: Cleanup Legacy Tests & Full Suite Verification

**Files:**
- Remove: `tests/core/` (legacy directory)
- Remove: Flat files under `tests/service/*.py` replaced by `tests/service/unit/` and `tests/service/integration/`

### Steps:
- [ ] **Step 7.1**: Remove legacy `tests/core/` and redundant flat service test files:
  ```bash
  rm -rf tests/core/
  rm -f tests/service/*.py (keeping tests/service/conftest.py, tests/service/unit/, tests/service/integration/)
  ```
- [ ] **Step 7.2**: Run Ruff formatting and linting:
  ```bash
  ruff format src/ tests/ scripts/ && ruff check src/ tests/ scripts/
  ```
- [ ] **Step 7.3**: Run Pyright static type checker:
  ```bash
  pyright src/ tests/
  ```
- [ ] **Step 7.4**: Run full pytest suite across entire project:
  ```bash
  python -m pytest tests/ -v
  ```
- [ ] **Step 7.5**: Confirm test result: 100% passed, 0 xfailed, 0 failed, 0 errors.
- [ ] **Step 7.6**: Final commit:
  ```bash
  git add tests/ && git commit -m "test: clean up legacy core tests and finalize modern multi-algorithm test architecture"
  ```
