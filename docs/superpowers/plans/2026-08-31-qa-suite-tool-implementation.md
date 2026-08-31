# Kế Hoạch Triển Khai Bộ Công Cụ Kiểm Thử VTX QA Suite

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Xây dựng bộ công cụ kiểm thử toàn diện VTX QA Suite (Web App Streamlit + CLI Tool) hỗ trợ trực quan hóa 2D, kiểm định độc lập bằng Validation Oracle, chạy kiểm thử hồi quy hàng loạt và đo tải đồng thời NATS Microservice.

**Architecture:** Kiến trúc mô-đun phân tầng:
- Lõi `src/tools/qa_suite/core/`: `ExecutionDriver` (Dual-mode Local/NATS), `PlotlyVisualizer2D`, `BatchRegressionEngine`, `NatsStressTester`, `ReportGenerator`.
- Giao diện `src/tools/qa_suite/views/` & `app.py`: Streamlit Web App 3 tabs (Visual Inspector, Batch Regression, NATS Stress Test).
- Giao diện dòng lệnh `src/tools/qa_suite/cli.py`: CLI runner cho CI/CD và headless servers.

**Tech Stack:** Python 3.10+, Streamlit, Plotly, NATS (nats-py), Protocol Buffers (protobuf), Shapely, Pytest.

**Spec:** `docs/superpowers/specs/2026-08-31-qa-suite-tool-design.md`

## Global Constraints
- Tất cả các module phải tuân thủ chuẩn định dạng và kiểu dữ liệu nghiêm ngặt của dự án (`ruff check`, `pyright`).
- Mọi kết quả kiểm thử phải được tự động thẩm định qua `src/path_planning/validation/oracle.py:path_is_valid`.
- Giao diện Streamlit sử dụng biểu đồ tương tác `plotly.graph_objects` để hiển thị bản đồ 2D.
- Không sửa đổi logic toán học cốt lõi trong `src/path_planning/` và `src/service/`.

---

### Task 1: Core QA Result Dataclass & Dual Execution Driver

**Files:**
- Create: `src/tools/__init__.py`
- Create: `src/tools/qa_suite/__init__.py`
- Create: `src/tools/qa_suite/core/__init__.py`
- Create: `src/tools/qa_suite/core/runner.py`
- Test: `tests/tools/__init__.py`
- Test: `tests/tools/unit/__init__.py`
- Test: `tests/tools/unit/test_qa_runner.py`

**Interfaces:**
- Produces:
  - `class ExecutionMode(str, Enum)`: `LOCAL = "local"`, `NATS = "nats"`
  - `class QAResult`: Chứa `scenario_name`, `status`, `is_success`, `waypoints`, `path_length_m`, `wall_time_s`, `applied_time_budget_s`, `iterations`, `oracle_verdict`, `error_detail`
  - `class ExecutionDriver`: `__init__(self, mode: ExecutionMode = ExecutionMode.LOCAL, nats_url: str = DEFAULT_NATS_SERVER, subject: str = DEFAULT_SUBJECT)`, `run_scenario(self, scenario: Scenario, name: str = "custom", time_budget_s: float = 15.0) -> QAResult`

- [ ] **Step 1: Write failing unit test for ExecutionDriver**

```python
# tests/tools/unit/test_qa_runner.py
from unittest.mock import AsyncMock, MagicMock
import pytest
from path_planning.scenario.presets import get_all_scenarios
from tools.qa_suite.core.runner import ExecutionDriver, ExecutionMode, QAResult

def test_local_execution_driver_solves_preset_scenario() -> None:
    scenario = get_all_scenarios()["scenario_01_open_space"]()
    driver = ExecutionDriver(mode=ExecutionMode.LOCAL)
    result = driver.run_scenario(scenario, name="scenario_01_open_space")
    assert isinstance(result, QAResult)
    assert result.is_success is True
    assert result.status == "OK"
    assert len(result.waypoints) >= 2
    assert result.oracle_verdict.is_ok is True
```

- [ ] **Step 2: Run test to verify failure**

Run: `pytest tests/tools/unit/test_qa_runner.py -v`
Expected: FAIL with "ModuleNotFoundError: No module named 'tools'"

- [ ] **Step 3: Implement ExecutionDriver in runner.py**

Create `src/tools/qa_suite/core/runner.py` implementing `QAResult`, `ExecutionMode`, `ExecutionDriver` (with `_run_local` and `_run_nats`).

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/tools/unit/test_qa_runner.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/tools/ tests/tools/
git commit -m "feat(qa): implement core QAResult dataclass and dual ExecutionDriver"
```

---

### Task 2: Plotly 2D Interactive Map Visualizer

**Files:**
- Create: `src/tools/qa_suite/core/visualizer_2d.py`
- Test: `tests/tools/unit/test_visualizer_2d.py`

**Interfaces:**
- Consumes: `Scenario` from `path_planning.types`, `QAResult` from `tools.qa_suite.core.runner`
- Produces:
  - `class PlotlyVisualizer2D`: `create_scenario_figure(scenario: Scenario, result: QAResult | None = None, show_fillet_arcs: bool = True) -> go.Figure`

- [ ] **Step 1: Write failing unit test for PlotlyVisualizer2D**

```python
# tests/tools/unit/test_visualizer_2d.py
import plotly.graph_objects as go
from path_planning.scenario.presets import get_all_scenarios
from tools.qa_suite.core.runner import ExecutionDriver, ExecutionMode
from tools.qa_suite.core.visualizer_2d import PlotlyVisualizer2D

def test_create_scenario_figure_returns_valid_plotly_figure() -> None:
    scenario = get_all_scenarios()["scenario_01_open_space"]()
    driver = ExecutionDriver(mode=ExecutionMode.LOCAL)
    result = driver.run_scenario(scenario, name="scenario_01_open_space")
    fig = PlotlyVisualizer2D.create_scenario_figure(scenario, result)
    assert isinstance(fig, go.Figure)
    assert len(fig.data) > 0
```

- [ ] **Step 2: Run test to verify failure**

Run: `pytest tests/tools/unit/test_visualizer_2d.py -v`
Expected: FAIL with "ModuleNotFoundError" or "ImportError"

- [ ] **Step 3: Implement PlotlyVisualizer2D**

Create `src/tools/qa_suite/core/visualizer_2d.py` with full rendering of Start/Goal vectors, Polygon obstacles with mitre buffer, Circle obstacles with radius buffer, Safezones, Waypoint polylines, and Dubins fillet arcs (`oracle.arc_points`).

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/tools/unit/test_visualizer_2d.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/tools/qa_suite/core/visualizer_2d.py tests/tools/unit/test_visualizer_2d.py
git commit -m "feat(qa): implement PlotlyVisualizer2D interactive scenario and trajectory map"
```

---

### Task 3: Batch Regression Engine & Report Generator

**Files:**
- Create: `src/tools/qa_suite/core/batch_runner.py`
- Create: `src/tools/qa_suite/core/report_generator.py`
- Test: `tests/tools/unit/test_batch_runner.py`

**Interfaces:**
- Consumes: `ExecutionDriver`, `QAResult`
- Produces:
  - `class BatchSummary`: `total_tests`, `success_count`, `fail_count`, `success_rate`, `wall_time_stats`, `path_length_stats`, `oracle_violation_count`, `results: list[QAResult]`
  - `class BatchRegressionEngine`: `run_presets(self, target_names: list[str] | None = None) -> BatchSummary`, `run_random_batch(self, count: int = 10, ...) -> BatchSummary`
  - `class ReportGenerator`: `export_json(summary: BatchSummary, file_path: Path) -> None`, `export_csv(summary: BatchSummary, file_path: Path) -> None`, `export_html(summary: BatchSummary, file_path: Path) -> None`

- [ ] **Step 1: Write failing unit test for BatchRegressionEngine and ReportGenerator**

```python
# tests/tools/unit/test_batch_runner.py
from pathlib import Path
from tools.qa_suite.core.batch_runner import BatchRegressionEngine
from tools.qa_suite.core.report_generator import ReportGenerator
from tools.qa_suite.core.runner import ExecutionDriver, ExecutionMode

def test_batch_regression_engine_runs_presets(tmp_path: Path) -> None:
    driver = ExecutionDriver(mode=ExecutionMode.LOCAL)
    engine = BatchRegressionEngine(driver=driver)
    summary = engine.run_presets(target_names=["scenario_01_open_space", "scenario_02_narrow_corridor"])
    assert summary.total_tests == 2
    assert summary.success_count == 2
    assert summary.success_rate == 100.0

    html_file = tmp_path / "report.html"
    ReportGenerator.export_html(summary, html_file)
    assert html_file.exists()
    assert len(html_file.read_text(encoding="utf-8")) > 100
```

- [ ] **Step 2: Run test to verify failure**

Run: `pytest tests/tools/unit/test_batch_runner.py -v`
Expected: FAIL

- [ ] **Step 3: Implement BatchRegressionEngine and ReportGenerator**

Implement `src/tools/qa_suite/core/batch_runner.py` and `src/tools/qa_suite/core/report_generator.py`.

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/tools/unit/test_batch_runner.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/tools/qa_suite/core/batch_runner.py src/tools/qa_suite/core/report_generator.py tests/tools/unit/test_batch_runner.py
git commit -m "feat(qa): implement BatchRegressionEngine and standalone ReportGenerator"
```

---

### Task 4: NATS Concurrency & Stress Tester

**Files:**
- Create: `src/tools/qa_suite/core/stress_tester.py`
- Test: `tests/tools/unit/test_stress_tester.py`

**Interfaces:**
- Produces:
  - `class StressTestSummary`: `total_requests`, `concurrency`, `success_count`, `error_count`, `timeout_count`, `throughput_rps`, `latency_p50_s`, `latency_p90_s`, `latency_p95_s`, `latency_p99_s`, `latencies: list[float]`
  - `class NatsStressTester`: `async run_stress_test(self, server_url: str, subject: str, scenario: Scenario, total_requests: int = 50, concurrency: int = 5, timeout_s: float = 6.0) -> StressTestSummary`

- [ ] **Step 1: Write failing unit test for NatsStressTester**

```python
# tests/tools/unit/test_stress_tester.py
import pytest
from unittest.mock import AsyncMock, MagicMock
from path_planning.scenario.presets import get_all_scenarios
from tools.qa_suite.core.stress_tester import NatsStressTester

@pytest.mark.asyncio
async def test_nats_stress_tester_collects_concurrency_metrics() -> None:
    scenario = get_all_scenarios()["scenario_01_open_space"]()
    tester = NatsStressTester()
    # Mock NATS request-reply
    ...
```

- [ ] **Step 2: Run test to verify failure**

Run: `pytest tests/tools/unit/test_stress_tester.py -v`
Expected: FAIL

- [ ] **Step 3: Implement NatsStressTester**

Create `src/tools/qa_suite/core/stress_tester.py` utilizing `asyncio.Semaphore` and `asyncio.gather` with percentile calculations (`numpy.percentile`).

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/tools/unit/test_stress_tester.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/tools/qa_suite/core/stress_tester.py tests/tools/unit/test_stress_tester.py
git commit -m "feat(qa): implement NatsStressTester for async microservice load profiling"
```

---

### Task 5: Streamlit Web UI Views & CLI Tool Entrypoint

**Files:**
- Create: `src/tools/qa_suite/views/__init__.py`
- Create: `src/tools/qa_suite/views/tab_inspector.py`
- Create: `src/tools/qa_suite/views/tab_batch.py`
- Create: `src/tools/qa_suite/views/tab_stress.py`
- Create: `src/tools/qa_suite/app.py`
- Create: `src/tools/qa_suite/cli.py`
- Test: `tests/tools/unit/test_cli.py`

**Interfaces:**
- Produces:
  - Streamlit UI (`streamlit run src/tools/qa_suite/app.py`)
  - CLI commands (`python -m tools.qa_suite.cli`)

- [ ] **Step 1: Write failing unit test for CLI parsing**

```python
# tests/tools/unit/test_cli.py
from tools.qa_suite.cli import build_parser

def test_cli_parser_recognizes_subcommands() -> None:
    parser = build_parser()
    args = parser.parse_args(["run-presets", "--target", "local"])
    assert args.command == "run-presets"
    assert args.target == "local"
```

- [ ] **Step 2: Run test to verify failure**

Run: `pytest tests/tools/unit/test_cli.py -v`
Expected: FAIL

- [ ] **Step 3: Implement Streamlit UI views and CLI entrypoint**

Create `views/tab_inspector.py`, `views/tab_batch.py`, `views/tab_stress.py`, `app.py`, and `cli.py`.

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/tools/unit/test_cli.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/tools/qa_suite/ tests/tools/unit/test_cli.py
git commit -m "feat(qa): implement Streamlit Web App views and unified CLI entrypoint"
```

---

### Task 6: Full Verification, Ruff/Pyright Check & End-to-End Run

**Files:**
- Verify all codebase files

- [ ] **Step 1: Run format and linters**
Run: `ruff format . && ruff check . && pyright`
Expected: All checks passed (0 errors, 0 warnings).

- [ ] **Step 2: Run complete pytest suite**
Run: `pytest tests/ -v`
Expected: All tests passed (100% green).

- [ ] **Step 3: Smoke test CLI run-presets command**
Run: `python -m tools.qa_suite.cli run-presets --target local`
Expected: Exit code 0, 18 presets solved with OK status.

- [ ] **Step 4: Final Commit**
```bash
git add -A
git commit -m "test(qa): verify full test suite and smoke test CLI with VTX QA Suite"
```
