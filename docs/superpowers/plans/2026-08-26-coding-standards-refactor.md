# Coding Standards Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor toàn bộ codebase Path_Planning để tuân thủ tài liệu Python Coding Standards, tái cấu trúc toàn bộ mã nguồn (bao gồm cả `service`) vào thư mục `src/`, và hạ Python target xuống 3.10.

**Architecture:** 9 task tuần tự. Task 1 dựng src layout mới (`src/path_planning` và `src/service`) + gộp chung tests + Python 3.10 compat. Task 2 cấu hình tooling. Task 3 auto-format. Task 4-9 sửa thủ công từng hạng mục. Mỗi task kết thúc bằng `pytest` pass + commit.

**Tech Stack:** Python 3.10+, Ruff (linter/formatter), Pyright (type checker), pytest, pre-commit, typing_extensions

**Spec:** [docs/Python_Coding_Standards.docx](file:///mnt/d/Workspace/VTX/Path_Planning/docs/Python_Coding_Standards.docx) — trích xuất text tại [docs/coding_standards_extracted.txt](file:///mnt/d/Workspace/VTX/Path_Planning/docs/coding_standards_extracted.txt)

## Global Constraints

- Python >= 3.10 (hạ từ 3.11 hiện tại)
- Line length: 88 ký tự (§2.2)
- Formatter/Linter: Ruff (§14.1)
- Type checker: Pyright strict mode (§14.2)
- Test runner: pytest >= 8.0 (§14.3)
- Docstring style: Google (§4)
- Naming: snake_case vars/funcs, PascalCase classes, SCREAMING_SNAKE_CASE constants, is_/has_/can_ booleans (§3)
- Project layout: src layout (§12.2)
- Không break test hiện tại — mỗi task phải `pytest` pass trước khi commit

---

## File Structure

### Current layout (flat):
```
Path_Planning/
├── config.py                    # Root-level config module
├── logger_config.py             # Root-level logging setup
├── batch_random_test.py         # Root-level test script
├── performance_eval.py          # Root-level eval script
├── core/                        # Core algorithm package
├── render/                      # Visualization package
├── service/                     # gRPC/DDS service (semi-independent)
│   ├── vtx_service/             # Service implementation
│   ├── tests/                   # Service tests
│   ├── deploy/, spike/, idl/
│   └── conftest.py              # sys.path hacks
├── scripts/                     # CLI scripts
├── tests/                       # Core tests (25 files)
├── cases/                       # Scenario test scripts
├── pyproject.toml               # line-length=100, py311
└── pytest.ini                   # Legacy config
```

### Target layout (src):
```
Path_Planning/
├── src/
│   ├── path_planning/           # Main core package
│   │   ├── __init__.py          # NEW
│   │   ├── config.py            # MOVED from root
│   │   ├── logger_config.py     # MOVED from root
│   │   ├── core/                # MOVED from root
│   │   └── render/              # MOVED from root
│   └── service/                 # Service package (MOVED from root)
│       ├── __init__.py          # NEW
│       ├── vtx_service/
│       ├── deploy/
│       ├── spike/
│       └── idl/
├── scripts/                     # STAYS
├── tests/                       # STRUCTURED
│   ├── core/                    # MOVED from tests root
│   └── service/                 # MOVED from service/tests
├── cases/                       # STAYS
├── batch_random_test.py         # STAYS (script, not package)
├── performance_eval.py          # STAYS (script, not package)
├── pyproject.toml               # UPDATED
├── .pre-commit-config.yaml      # NEW
└── requirements.txt             # STAYS
```

### Import migration map:

| Old import | New import |
|---|---|
| `import config` | `from path_planning import config` |
| `import core.X as Y` | `from path_planning.core import X as Y` |
| `from core.types import Z` | `from path_planning.core.types import Z` |
| `import render.X` | `from path_planning import render` |
| `from render import PathVisualizer` | `from path_planning.render import PathVisualizer` |
| `from logger_config import setup_logging` | `from path_planning.logger_config import setup_logging` |
| `from vtx_service import X` | `from service.vtx_service import X` |

---

### Task 1: Tái Cấu Trúc Thư Mục src Layout + Python 3.10 (§12.2)

**Files:**
- Create: `src/path_planning/__init__.py`, `src/service/__init__.py`
- Move: `config.py`, `logger_config.py`, `core/`, `render/` → `src/path_planning/`
- Move: `service/` → `src/service/`
- Move: `tests/*.py` → `tests/core/`
- Move: `src/service/tests/*` → `tests/service/`
- Modify: `src/path_planning/core/types.py` (NotRequired compat)
- Modify: ALL files with internal imports (~45 files)
- Modify: `pyproject.toml` (add [project], [build-system])

**Interfaces:**
- Produces: Packages `path_planning` và `service` installable via `pip install -e .`
- Produces: All imports use `path_planning.` hoặc `service.` prefix

- [ ] **Step 1: Tạo thư mục src layout và di chuyển files cho `path_planning`**

```bash
mkdir -p src/path_planning
git mv config.py src/path_planning/config.py
git mv logger_config.py src/path_planning/logger_config.py
git mv core/ src/path_planning/core/
git mv render/ src/path_planning/render/
```

- [ ] **Step 2: Di chuyển `service` vào `src` và gom `tests` lại**

```bash
# Move toàn bộ service vào src
git mv service src/service

# Cấu trúc lại thư mục tests
mkdir -p tests/core tests/service
find tests/ -maxdepth 1 -name "*.py" -exec git mv {} tests/core/ \;
git mv src/service/tests/* tests/service/
rmdir src/service/tests
```

- [ ] **Step 3: Tạo các `__init__.py` mới**

```python
# src/path_planning/__init__.py
"""Path Planning — Kinodynamic A* path planner cho robot."""

# src/service/__init__.py
"""Dịch vụ gRPC/DDS cho Path Planning."""
```

- [ ] **Step 4: Sửa Python 3.11 → 3.10 compatibility trong `core/types.py`**

File `src/path_planning/core/types.py` hiện dùng `NotRequired` từ `typing` (3.11+). Thay bằng:

```python
# Trước (Python 3.11+):
from typing import Literal, NotRequired, TypedDict

# Sau (Python 3.10+):
from __future__ import annotations

import sys
from typing import Literal, TypedDict

if sys.version_info >= (3, 11):
    from typing import NotRequired
else:
    from typing_extensions import NotRequired
```

- [ ] **Step 5: Cập nhật imports trong `src/path_planning/core/` và `render/`**

Sử dụng script bash (sed) hoặc sửa thủ công các pattern sau:
- `from core...` → `from path_planning.core...`
- `import core...` → `from path_planning.core import...` (hoặc `import path_planning.core...`)
- `from render...` → `from path_planning.render...`
- `import config` → `from path_planning import config`

- [ ] **Step 6: Cập nhật imports trong `tests/core/` và `tests/service/`**

```bash
# Chạy script replace trong tests/
find tests/ -name "*.py" -exec sed -i \
  -e 's/^import config$/from path_planning import config/' \
  -e 's/^import core\./from path_planning.core import /' \
  -e 's/^from core\./from path_planning.core./' \
  -e 's/^import render/from path_planning import render/' \
  -e 's/^from render /from path_planning.render /' \
  -e 's/from vtx_service/from service.vtx_service/g' \
  -e 's/import vtx_service/from service import vtx_service/g' \
  {} +
```

- [ ] **Step 7: Cập nhật imports trong `src/service/`**

Xoá `sys.path.insert` trong `src/service/conftest.py` (nếu còn dùng) hoặc `tests/service/conftest.py`. Môi trường sẽ load thông qua `pip install -e .`.

Cập nhật mọi import trong `src/service/vtx_service/`:
- `from vtx_service.messages import ...` → `from service.vtx_service.messages import ...`
- `import core...` → `from path_planning.core import...`
- `import config` → `from path_planning import config`

- [ ] **Step 8: Cập nhật imports trong root scripts**

Cập nhật `batch_random_test.py`, `performance_eval.py`, `scripts/*.py` theo map:
- `import config` → `from path_planning import config`
- `from logger_config...` → `from path_planning.logger_config...`
- `import core...` → `from path_planning.core import...`

- [ ] **Step 9: Thêm `[project]` và `[build-system]` vào `pyproject.toml`**

```toml
[project]
name = "path_planning"
version = "0.1.0"
description = "Kinodynamic A* path planner cho robot"
readme = "README.md"
requires-python = ">=3.10"
dependencies = [
    "numpy",
    "shapely",
    "matplotlib",
    "typing_extensions>=4.0; python_version < '3.11'",
]

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[tool.hatch.build.targets.wheel]
packages = ["src/path_planning", "src/service"]
```

- [ ] **Step 10: Cài đặt editable mode và chạy tests**

```bash
pip install -e .
pytest
```

- [ ] **Step 11: Commit**

```bash
git add -A
git commit -m "refactor: tái cấu trúc thư mục src layout cho cả core và service, gộp tests, hỗ trợ Python 3.10 (§12.2)"
```

---

### Task 2: Cấu Hình Tooling (§14)

**Files:**
- Modify: `pyproject.toml`
- Create: `.pre-commit-config.yaml`
- Delete: `pytest.ini`

**Interfaces:**
- Consumes: src layout từ Task 1
- Produces: Ruff, Pyright, pytest, bandit config chuẩn. `.pre-commit-config.yaml`.

- [ ] **Step 1: Cập nhật `[tool.ruff]` trong `pyproject.toml`**

```toml
[tool.ruff]
target-version = "py310"
line-length = 88
indent-width = 4
src = ["src", "tests"]

extend-exclude = [
    ".git",
    ".venv",
    "__pycache__",
    "build",
    "dist",
    "docs",
    "src/service/idl",
]

[tool.ruff.format]
quote-style = "double"
indent-style = "space"
line-ending = "auto"
skip-magic-trailing-comma = false
docstring-code-format = true
docstring-code-line-length = 80

[tool.ruff.lint]
select = [
    "E", "W", "F", "I", "N", "D", "UP", "B", "SIM", "S", "ASYNC", "RUF", "ANN", "C4"
]
ignore = [
    "D100", "D104", "D107", "B008", "RUF001", "RUF002", "RUF003", "D203", "D213"
]
fixable = ["ALL"]
unfixable = []

[tool.ruff.lint.pydocstyle]
convention = "google"

[tool.ruff.lint.isort]
known-first-party = ["path_planning", "service"]
combine-as-imports = true
lines-after-imports = 2

[tool.ruff.lint.per-file-ignores]
"tests/**/*" = ["S101", "D", "ANN", "ARG", "S105", "S106"]
"**/*_test.py" = ["S101", "D", "ANN", "ARG", "S105", "S106"]
"scripts/**" = ["D", "ANN"]
"batch_random_test.py" = ["D", "ANN"]
"performance_eval.py" = ["D", "ANN"]
"cases/**" = ["D", "ANN"]
```

- [ ] **Step 2: Cập nhật `[tool.pyright]`**

```toml
[tool.pyright]
include = ["src"]
exclude = [
    "**/__pycache__",
    ".venv",
    "dist",
    "build",
    "src/service/idl",
]
pythonVersion = "3.10"
pythonPlatform = "All"
typeCheckingMode = "strict"
useLibraryCodeForTypes = true
reportMissingTypeStubs = false
reportMissingTypeAnnotations = "error"
reportMissingParameterType = "error"
reportUnknownMemberType = "warning"
reportUntypedBaseClass = "warning"
reportUnnecessaryTypeIgnoreComment = "warning"

[[tool.pyright.executionEnvironments]]
root = "src/path_planning/render"
extraPaths = ["src"]
reportUnknownMemberType = false
reportUnknownArgumentType = false
```

- [ ] **Step 3: Migrate pytest.ini → pyproject.toml và xoá pytest.ini**

```toml
[tool.pytest.ini_options]
minversion = "8.0"
testpaths = ["tests"]
python_files = ["test_*.py", "*_test.py"]
python_classes = ["Test*"]
python_functions = ["test_*"]
markers = [
    "unit: Kiểm thử đơn vị",
    "integration: Kiểm thử tích hợp",
    "e2e: Kiểm thử End-to-End",
    "slow: Các bài test nặng",
]
addopts = [
    "-ra",
    "--strict-markers",
    "--strict-config",
]
filterwarnings = [
    "error",
    "ignore::DeprecationWarning",
    "ignore::PendingDeprecationWarning",
]
xfail_strict = true
```

- [ ] **Step 4: Thêm `[tool.bandit]`**

```toml
[tool.bandit]
exclude_dirs = ["tests", ".venv"]
skips = ["B101"]
```

- [ ] **Step 5: Tạo `.pre-commit-config.yaml`**

```yaml
repos:
  - repo: https://github.com/astral-sh/ruff-pre-commit
    rev: v0.6.0
    hooks:
      - id: ruff
        args: [ --fix ]
      - id: ruff-format
  - repo: https://github.com/PyCQA/bandit
    rev: 1.7.7
    hooks:
      - id: bandit
        args: ["-c", "pyproject.toml"]
        additional_dependencies: ["bandit[toml]"]
```

- [ ] **Step 6: Xoá `pytest.ini` và chạy tests**

```bash
git rm pytest.ini
pytest
```

- [ ] **Step 7: Commit**

```bash
git add pyproject.toml .pre-commit-config.yaml
git commit -m "build: chuẩn hoá tooling config theo coding standards (§14)"
```

---

### Task 3: Auto-Format & Modernize Syntax (§2, §5)

**Files:**
- Modify: ALL `.py` files (trừ `src/service/idl/`)

**Interfaces:**
- Consumes: Ruff config từ Task 2
- Produces: Code formatted đúng 88 chars, modern type hints (X|None), import ordering chuẩn

- [ ] **Step 1: Chạy Ruff format trên toàn dự án**

```bash
ruff format .
```

- [ ] **Step 2: Chạy Ruff auto-fix modernize syntax**

```bash
ruff check . --fix --select UP006,UP007
```

Lệnh này tự động chuyển:
- `Optional[X]` → `X | None`
- `Union[X, Y]` → `X | Y`
- `List[X]` → `list[X]`, `Dict[K,V]` → `dict[K,V]`, `Tuple[...]` → `tuple[...]`

- [ ] **Step 3: Chạy Ruff auto-fix import ordering**

```bash
ruff check . --fix --select I,F401
```

- [ ] **Step 4: Review diff thủ công**

```bash
git diff --stat
```

- [ ] **Step 5: Chạy tests**

```bash
pytest
```

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "style: ruff format 88 chars + modernize type hints (§2, §5)"
```

---

### Task 4: Thêm Google-Style Docstrings (§4)

**Files:**
- Modify: Tất cả module trong `src/path_planning/` và `src/service/vtx_service/`

**Interfaces:**
- Produces: Google-style docstrings cho tất cả public classes, functions, modules

- [ ] **Step 1: Thêm module-level docstrings cho `src/path_planning/core/`**

Mỗi file cần dòng đầu tiên là docstring mô tả module (đã được đề cập trong bản nháp).

- [ ] **Step 2: Thêm class docstrings cho dataclasses và classes chính**

Mẫu cho mỗi class trong `types.py`:

```python
class Pose(NamedTuple):
    """Vị trí và hướng của robot trong không gian 2D.

    Attributes:
        x: Toạ độ x (mét).
        y: Toạ độ y (mét).
        heading: Góc hướng (radian).
    """
    x: float
    y: float
    heading: float
```

- [ ] **Step 3: Thêm function docstrings cho public functions**

Mẫu chuẩn với Args/Returns/Raises:

```python
def prepare_scenario(
    scenario: Scenario,
    *,
    turn_radius: float = config.R,
    safe_margin: float = config.SAFE_MARGIN,
) -> PreprocessedScenario:
    """Tiền xử lý scenario: inflate obstacles và tính toán start/end state.

    Args:
        scenario: Dữ liệu scenario gốc chứa obstacles, start, goal.
        turn_radius: Bán kính cua tối thiểu của robot (mét).
        safe_margin: Khoảng cách an toàn inflate obstacles (mét).

    Returns:
        Scenario đã tiền xử lý, sẵn sàng cho A* search.

    Raises:
        ValueError: Nếu scenario thiếu trường bắt buộc.
    """
```

- [ ] **Step 4: Thêm docstrings cho `src/service/vtx_service/` modules**

```python
# src/service/vtx_service/messages.py
"""Domain messages: PlanRequest, PlanReply, và các value objects."""
# ...
```

- [ ] **Step 5: Kiểm tra Ruff docstring rules**

```bash
ruff check src/ --select D
```

- [ ] **Step 6: Chạy tests và Commit**

```bash
pytest
git add -A
git commit -m "docs: thêm Google-style docstrings (§4)"
```

---

### Task 5: Chuyển print() Sang Logger (§10)

**Files:**
- Modify: `src/path_planning/core/kinodynamic_astar.py`
- Modify: `src/path_planning/core/kinodynamic_astar_v0.py`
- Modify: `src/path_planning/render/visualizer.py`
- Modify: `batch_random_test.py`, `performance_eval.py`, `scripts/*.py`
- Modify: `src/service/spike/cyclone_probe.py`

**Interfaces:**
- Produces: Tất cả modules dùng `logger = logging.getLogger(__name__)`

- [ ] **Step 1: Thêm logger vào các module core thiếu**

Thêm sau block imports trong mỗi file:

```python
import logging

logger = logging.getLogger(__name__)
```

- [ ] **Step 2: Thay `print()` bằng `logger.info()` trong code thuật toán**

```python
# Trước:
print(f"Path found with {len(path)} waypoints")

# Sau:
logger.info("Path found with %d waypoints", len(path))
```

- [ ] **Step 3: Thay `print()` trong scripts**

- [ ] **Step 4: Chạy tests và Commit**

```bash
pytest
git commit -am "refactor: chuyển print() sang logging module (§10)"
```

---

### Task 6: Chuẩn Hoá Naming Conventions (§3)

**Files:**
- Modify: `src/path_planning/config.py` (camelCase fields)
- Modify: `src/path_planning/core/types.py` (boolean attrs)
- Modify: `src/path_planning/core/kinodynamic_astar.py` (boolean methods)
- Modify: `src/path_planning/core/kinodynamic_astar_v0.py` (boolean methods)
- Modify: `src/path_planning/core/path_validation.py` (boolean attr)
- Modify: `src/path_planning/core/arc_geometry.py` (boolean func)
- Modify: `src/service/vtx_service/messages.py` (boolean attrs)

**Interfaces:**
- Produces: All names follow §3 conventions

- [ ] **Step 1: Rename camelCase → snake_case trong `config.py`**

```python
goalTolerance       → goal_tolerance
maxSteerAngleDeg    → max_steer_angle_deg
reverseSteerAngleDeg → reverse_steer_angle_deg
reverseSpeedRatio   → reverse_speed_ratio
topLeftCorner       → top_left_corner
bottomRightCorner   → bottom_right_corner
gateWidth           → gate_width
gateHeading         → gate_heading
gateAngleFromBorder → gate_angle_from_border
rMin                → r_min
```

- [ ] **Step 2: Grep và cập nhật tất cả references cho config**

- [ ] **Step 3: Rename boolean attributes và methods**

```python
# types.py
success: bool        → is_success: bool
budget_bound: bool   → is_budget_bound: bool
search_failed: bool  → is_search_failed: bool

# kinodynamic_astar.py
_check_collision()   → _is_collision_free()
_in_bounds()         → _is_in_bounds()
_goal_reached()      → _is_goal_reached()
_sector_clear()      → _is_sector_clear()
_corner_arc_clear()  → _is_corner_arc_clear()

# path_validation.py
ok: bool             → is_ok: bool
```

- [ ] **Step 4: Grep toàn dự án và sửa tất cả call sites**

- [ ] **Step 5: Chạy tests và Commit**

```bash
pytest
git commit -am "refactor: chuẩn hoá naming conventions snake_case + boolean prefix (§3)"
```

---

### Task 7: Class Design & Function Design (§6, §7)

**Files:**
- Modify: `src/path_planning/core/preprocessing.py`
- Modify: `src/path_planning/core/kinodynamic_astar.py`
- Modify: `src/path_planning/core/kinodynamic_astar_v0.py`
- Modify: `src/path_planning/core/path_validation.py`
- Modify: `src/path_planning/core/map_generator.py`
- Modify: `src/path_planning/core/goal_shot.py`
- Modify: `src/path_planning/core/arc_geometry.py`

**Interfaces:**
- Produces: Keyword-only args trên optional params, frozen dataclasses thay NamedTuple

- [ ] **Step 1: Thêm `*` separator vào các hàm có tuỳ chọn (optional parameters)**

```python
# Sau khi sửa (Ví dụ):
def plan_trajectory(
    preprocessed_scenario,
    *,
    verbose: bool = False,
    time_budget_s: float | None = None,
) -> dict:
```

- [ ] **Step 2: Grep cho positional calls và sửa thành keyword calls**

- [ ] **Step 3: Chuyển `NamedTuple` → `@dataclass(frozen=True)`**

```python
# Sau khi sửa (Ví dụ):
from dataclasses import dataclass

@dataclass(frozen=True)
class ValidationResult:
    """Kết quả xác nhận tính hợp lệ của đường đi."""
    is_ok: bool
    reason: str
```

- [ ] **Step 4: Chuyển tuple unpacking sang attribute access**

```python
# Trước:
ok, reason = path_is_valid(...)
# Sau:
result = path_is_valid(...)
is_ok = result.is_ok
```

- [ ] **Step 5: Chạy tests và Commit**

```bash
pytest
git commit -am "refactor: keyword-only args + NamedTuple→dataclass (§6, §7)"
```

---

### Task 8: Imports Cleanup (§8)

**Files:**
- Modify: `src/service/vtx_service/runner.py` (in-function imports)
- Modify: `src/service/vtx_service/main.py` (in-function imports)
- Modify: `batch_random_test.py` (shadowed imports)

**Interfaces:**
- Produces: Clean imports: đúng order, không wildcard, không unused, không in-function (trừ lazy-load có lý do)

- [ ] **Step 1: Chạy Ruff fix imports toàn dự án**

```bash
ruff check . --fix --select I,F401,F811
```

- [ ] **Step 2: Review in-function imports trong `runner.py` và `main.py`**

Nếu có comment giải thích (e.g., fork safety) → giữ nguyên, thêm comment `# lazy import`. Nếu không → move lên đầu file.

- [ ] **Step 3: Sửa shadowed/dead import trong `batch_random_test.py`**

- [ ] **Step 4: Chạy tests và Commit**

```bash
pytest
git commit -am "refactor: cleanup imports — order, unused, in-function (§8)"
```

---

### Task 9: Error Handling & Config Cleanup (§9, §13)

**Files:**
- Modify: `src/path_planning/logger_config.py`
- Modify: `batch_random_test.py`
- Modify: `src/path_planning/render/visualizer.py`
- Modify: `src/service/vtx_service/transport.py`

**Interfaces:**
- Produces: Specific exception handling, no swallowed errors, logger_config bug fixed

- [ ] **Step 1: Sửa bug `os.makedirs("")` trong `logger_config.py`**

```python
# Sau:
log_dir = os.path.dirname(log_file)
if log_dir:
    os.makedirs(log_dir, exist_ok=True)
```

- [ ] **Step 2: Sửa broad except trong `batch_random_test.py`**

```python
# Sau:
except (ValueError, RuntimeError) as exc:
    logger.exception("Scenario %d failed", seed)
    result["status"] = "error"
    result["error"] = str(exc)
```

- [ ] **Step 3: Thêm logging cho broad except trong `render/visualizer.py`**

```python
# Sau:
except Exception:
    logger.debug("Arc interpolation failed, degrading to straight line.", exc_info=True)
```

- [ ] **Step 4: Review broad except trong `src/service/vtx_service/transport.py`**

Các `except Exception: # noqa: BLE001` đều có comment giải thích lý do (service resilience boundary). Giữ nguyên nhưng đảm bảo mỗi block có `logger.exception()`.

- [ ] **Step 5: Chạy tests và Commit**

```bash
pytest
git commit -am "refactor: sửa error handling + logger_config bug (§9, §13)"
```

---

## Self-Review Checklist

### 1. Spec coverage
Tất cả các phần của Spec đã được map thành các Task tương ứng (PEP 8, Naming, Docstrings, Layout src, v.v.).

### 2. Placeholder scan
Không có TBD, TODO, hay "implement later" trong plan.

### 3. Type consistency
- `path_planning.*` và `service.*` imports nhất quán qua tất cả tasks.
- Các interface thay đổi ở Task 6, 7 được tiêu thụ nhất quán.

---

## Execution Summary

| Task | Thời gian | Auto-fix |
|---|---|---|
| 1. src Layout (core + service) + Py3.10 | 2-3 giờ | Một phần (sed) |
| 2. Tooling Config | 30 phút | Không |
| 3. Auto-Format | 15 phút | ✅ Ruff |
| 4. Docstrings | 3-4 giờ | Không |
| 5. Logger | 1-2 giờ | Không |
| 6. Naming | 2-3 giờ | Một phần |
| 7. Class/Function Design | 2-3 giờ | Không |
| 8. Imports Cleanup | 30 phút | ✅ Ruff |
| 9. Error Handling | 1-2 giờ | Không |
| **Tổng** | **13-18 giờ** | |
