# VTX Path Planning Service — Part 1: Python worker

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Xây `vtx_planner` — một package Python thuần bọc thuật toán path planning hiện có, cộng một tiến trình worker phục vụ qua Unix domain socket, để phần DDS ở Part 2 chỉ còn là việc dịch dữ liệu.

**Architecture:** Ba lớp xếp chồng, mỗi lớp test được độc lập. Lớp trong cùng là các hàm thuần (đổi đơn vị góc, phép chiếu toạ độ). Lớp giữa là `plan(PlanRequest) -> PlanReply`, dựng `Scenario` dict rồi gọi `core.preprocessing` + `core.kinodynamic_astar_v0` mà không sửa gì trong đó. Lớp ngoài là vòng lặp socket đóng khung bằng msgpack. Không lớp nào import Fast DDS.

**Tech Stack:** Python 3.11, `shapely` (đã là phụ thuộc của `core/`), `msgpack`, `pyproj`, `pytest`.

**Spec:** `docs/superpowers/specs/2026-08-22-dds-path-planning-service-design.md`

## Global Constraints

- **Không sửa `core/`, `render/`, `config.py`.** `git diff --stat core/ render/ config.py` phải rỗng ở mọi commit. Có một test cưỡng chế điều này (Task 1).
- **Planner dùng là `core.kinodynamic_astar_v0`**, không phải `core.kinodynamic_astar`. v0 là bản đang ship.
- **Adapter chỉ gọi, tuyệt đối không sao chép.** Không copy công thức hình học, không copy TypedDict, không hardcode danh sách hằng số `config`.
- **Đơn vị trên interface:** khoảng cách mét; góc là **độ, phương vị thật, thuận chiều kim đồng hồ từ chính bắc**. Bên trong `core/` là radian, ngược chiều kim đồng hồ từ `+x`. Đổi đơn vị xảy ra tại đúng một module (`angles.py`).
- **Phụ thuộc của worker đúng ba gói:** `shapely`, `msgpack`, `pyproj`. Không numpy trực tiếp, không scipy, không matplotlib. `core/` không import numpy ở đâu cả; nó chỉ vào theo đường bắc cầu của shapely.
- **Baseline test phải giữ nguyên:** `python -m pytest -q tests/` = 188 passed, 6 failed. Sáu ca đỏ có từ trước, không liên quan. Công việc này không được thêm ca đỏ nào.
- **Test của service nằm ở `service/tests/`**, chạy bằng `python -m pytest -q service/tests/`. Không đụng `tests/` ở gốc (thư mục đó nằm trong `.gitignore` và chỉ được track bằng force-add).
- **Nhánh:** `feature/dds-service`.

---

## File Structure

```
service/
  worker/
    vtx_planner/
      __init__.py           export plan, PlanRequest, PlanReply, PlanStatus
      messages.py           dataclass request/reply + enum PlanStatus  (Task 1)
      angles.py             đổi phương vị <-> heading toán học        (Task 2)
      projection.py         AEQD + tịnh tiến góc phần tư dương        (Task 3)
      scenario_builder.py   PlanRequest -> Scenario dict              (Task 4)
      runtime.py            ngân sách, config_hash, planner_version   (Task 5)
      planner.py            plan(PlanRequest) -> PlanReply            (Task 6)
      codec.py              msgpack + đóng khung length-prefixed      (Task 8)
      preloaded_map.py      bản đồ nền tĩnh, gộp vào request           (Task 9)
    run_worker.py           vòng lặp Unix socket                      (Task 9)
  tests/
    boundary_test.py        (Task 1)
    angles_test.py          (Task 2)
    projection_test.py      (Task 3)
    scenario_builder_test.py(Task 4)
    runtime_test.py         (Task 5)
    planner_test.py         (Task 6)
    equivalence_test.py     (Task 7)
    codec_test.py           (Task 8)
    preloaded_map_test.py   (Task 9)
    worker_ipc_test.py      (Task 9)
  deploy/
    worker-requirements.txt (Task 9)
  conftest.py               đặt PYTHONPATH về gốc repo                (Task 1)
```

Mỗi file một trách nhiệm. `angles.py` và `projection.py` là hàm thuần, không biết gì về planner. `scenario_builder.py` biết về `core.types` nhưng không gọi planner. `planner.py` là chỗ duy nhất gọi `plan_trajectory`.

---

### Task 1: Bộ khung service, kiểu dữ liệu, và test ranh giới

**Files:**
- Create: `service/__init__.py`
- Create: `service/conftest.py`
- Create: `service/worker/__init__.py`
- Create: `service/worker/vtx_planner/__init__.py`
- Create: `service/worker/vtx_planner/messages.py`
- Test: `service/tests/boundary_test.py`
- Test: `service/tests/messages_test.py`

**Interfaces:**
- Consumes: không có (task đầu tiên).
- Produces: `PlanStatus` (IntEnum), `Point` (alias `tuple[float, float]`), `Circle`, `VehicleLimits`, `SearchBudget`, `PlanRequest`, `Waypoint`, `SearchStats`, `PlanReply` — tất cả là `@dataclass(frozen=True)` trừ chỗ ghi rõ. Task 4-9 dùng trực tiếp.

- [ ] **Step 1: Viết test ranh giới (sẽ đỏ vì thư mục chưa có)**

Create `service/tests/boundary_test.py`:

```python
"""Ràng buộc số 1 của dự án: service không được sửa thuật toán.

Test này là cơ chế cưỡng chế, không phải lời nhắc. Nó so cây làm việc với
nhánh gốc `main`, nên nó đỏ ngay cả khi thay đổi đã được commit.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTECTED = ["core/", "render/", "config.py"]


def _diff_against_main(paths: list[str]) -> str:
    proc = subprocess.run(
        ["git", "diff", "--stat", "main", "--", *paths],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    return proc.stdout.strip()


def test_service_work_does_not_touch_the_algorithm() -> None:
    diff = _diff_against_main(PROTECTED)
    assert diff == "", (
        "Nhánh service đã sửa thuật toán, điều bị cấm bởi ràng buộc 1 của spec.\n"
        f"Thay đổi:\n{diff}"
    )


def test_service_tree_exists_and_is_separate() -> None:
    service = REPO_ROOT / "service"
    assert service.is_dir()
    assert not (service / "core").exists(), "core/ không được sao chép vào service/"
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/boundary_test.py -v`
Expected: FAIL — `ERROR ... file or directory not found: service/tests/boundary_test.py` (thư mục chưa tồn tại).

- [ ] **Step 3: Dựng bộ khung và conftest**

Create `service/__init__.py` (rỗng), `service/worker/__init__.py` (rỗng).

Create `service/conftest.py`:

```python
"""Đặt gốc repo lên sys.path để service/tests import được core.* và config.

Đây là cùng cơ chế mà worker dùng lúc chạy thật (PYTHONPATH trỏ về gốc repo),
nên test chạy đúng cấu hình import của production.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
WORKER_ROOT = REPO_ROOT / "service" / "worker"

for path in (REPO_ROOT, WORKER_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
```

- [ ] **Step 4: Chạy lại test ranh giới**

Run: `python -m pytest service/tests/boundary_test.py -v`
Expected: PASS, 2 passed.

- [ ] **Step 5: Viết test cho kiểu dữ liệu**

Create `service/tests/messages_test.py`:

```python
from __future__ import annotations

import dataclasses

import pytest

from vtx_planner.messages import (
    Circle,
    PlanReply,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    SearchStats,
    VehicleLimits,
    Waypoint,
)

IDL_VERSION = 1


def _minimal_request() -> PlanRequest:
    return PlanRequest(
        request_id=b"\x00" * 16,
        idl_version=IDL_VERSION,
        frame="local_meters",
        start=(0.0, 0.0),
        start_heading_deg=0.0,
        goal=(100000.0, 0.0),
        goal_heading_deg=90.0,
        goal_heading_free=False,
        islands=(),
        dynamic_obstacles=(),
        safezones=(),
        use_preloaded_map=False,
        limits=VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0),
        budget=SearchBudget(15.0, 50000),
    )


def test_request_is_frozen() -> None:
    req = _minimal_request()
    with pytest.raises(dataclasses.FrozenInstanceError):
        req.start = (1.0, 1.0)  # type: ignore[misc]


def test_request_id_must_be_16_bytes() -> None:
    with pytest.raises(ValueError, match="16 byte"):
        dataclasses.replace(_minimal_request(), request_id=b"\x00" * 8)


def test_frame_must_be_known() -> None:
    with pytest.raises(ValueError, match="frame"):
        dataclasses.replace(_minimal_request(), frame="galactic")


def test_status_values_are_stable_across_the_idl_boundary() -> None:
    # Các giá trị này phải khớp thứ tự enum trong IDL ở Part 2. Đổi số ở đây
    # mà không đổi IDL là một thay đổi phá vỡ hợp đồng đi qua không tiếng động.
    assert PlanStatus.OK == 0
    assert PlanStatus.NO_PATH == 1
    assert PlanStatus.START_LEG_BLOCKED == 2
    assert PlanStatus.GOAL_LEG_BLOCKED == 3
    assert PlanStatus.ORACLE_REJECTED == 4
    assert PlanStatus.INVALID_REQUEST == 5
    assert PlanStatus.TIMEOUT == 6
    assert PlanStatus.INTERNAL_ERROR == 7
    assert PlanStatus.BUSY == 8


def test_reply_carries_everything_the_spec_promises() -> None:
    reply = PlanReply(
        request_id=b"\x00" * 16,
        idl_version=IDL_VERSION,
        status=PlanStatus.OK,
        detail="",
        waypoints=(Waypoint((0.0, 0.0), 12.5),),
        path_length_m=1.0,
        plan_wall_time_s=0.5,
        stats=SearchStats(3, 50000, 7, False, False),
        planner_version="abc1234",
        config_hash="0123456789abcdef",
    )
    assert {f.name for f in dataclasses.fields(reply)} == {
        "request_id",
        "idl_version",
        "status",
        "detail",
        "waypoints",
        "path_length_m",
        "plan_wall_time_s",
        "stats",
        "planner_version",
        "config_hash",
    }


def test_circle_rejects_negative_radius() -> None:
    with pytest.raises(ValueError, match="radius"):
        Circle(center=(0.0, 0.0), radius_m=-1.0)
```

- [ ] **Step 6: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/messages_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_planner'`.

- [ ] **Step 7: Viết `messages.py`**

Create `service/worker/vtx_planner/messages.py`:

```python
"""Kiểu dữ liệu request/reply của service, ánh xạ 1-1 sang IDL ở Part 2.

Đây là hợp đồng đối ngoại, tách hẳn khỏi hai dict shape nội bộ của pipeline
(`core.types.Scenario` / `PreprocessedScenario`). Giữ hai thứ tách nhau là cố ý:
hợp đồng đối ngoại đổi theo phiên bản IDL, còn dict nội bộ đổi theo thuật toán.

Đơn vị: khoảng cách mét, góc là ĐỘ và là phương vị thật, thuận chiều kim đồng hồ
từ chính bắc. Xem `vtx_planner.angles` cho phép đổi sang quy ước của `core/`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

Point = tuple[float, float]
"""Một vị trí phẳng ``(x, y)`` mét, hoặc ``(lon, lat)`` độ khi frame là wgs84."""

IDL_VERSION = 1
"""Tăng khi bố cục struct trong IDL đổi. Node từ chối request không khớp."""

FRAMES = ("local_meters", "wgs84")


class PlanStatus(IntEnum):
    """Kết cục của một lần lập kế hoạch.

    Giá trị số phải khớp thứ tự enum ``PlanStatus`` trong IDL.
    """

    OK = 0
    NO_PATH = 1
    START_LEG_BLOCKED = 2
    GOAL_LEG_BLOCKED = 3
    ORACLE_REJECTED = 4
    INVALID_REQUEST = 5
    TIMEOUT = 6
    INTERNAL_ERROR = 7
    BUSY = 8


@dataclass(frozen=True)
class Circle:
    """Chướng ngại vật tròn."""

    center: Point
    radius_m: float

    def __post_init__(self) -> None:
        if not self.radius_m > 0.0:
            raise ValueError(f"radius_m phải dương, nhận {self.radius_m}")


@dataclass(frozen=True)
class VehicleLimits:
    """Năm tham số duy nhất tới được planner qua đường tham số hàm.

    Chúng ánh xạ 1-1 sang tham số của ``core.preprocessing.prepare_scenario``.
    Mọi hằng số khác của planner là global và cố định lúc triển khai.
    """

    turn_radius_m: float
    l0_m: float
    dss_m: float
    safe_margin_m: float
    alpha_max_deg: float

    def __post_init__(self) -> None:
        for name in ("turn_radius_m", "l0_m", "dss_m", "alpha_max_deg"):
            value = getattr(self, name)
            if not value > 0.0:
                raise ValueError(f"{name} phải dương, nhận {value}")
        if self.safe_margin_m < 0.0:
            raise ValueError(f"safe_margin_m không được âm, nhận {self.safe_margin_m}")


@dataclass(frozen=True)
class SearchBudget:
    """Hai hằng số global được phép override theo request."""

    time_budget_s: float
    max_iterations: int

    def __post_init__(self) -> None:
        if not self.time_budget_s > 0.0:
            raise ValueError(f"time_budget_s phải dương, nhận {self.time_budget_s}")
        if not self.max_iterations > 0:
            raise ValueError(f"max_iterations phải dương, nhận {self.max_iterations}")


@dataclass(frozen=True)
class PlanRequest:
    """Một mission cần lập kế hoạch."""

    request_id: bytes
    idl_version: int
    frame: str
    start: Point
    start_heading_deg: float
    goal: Point
    goal_heading_deg: float
    goal_heading_free: bool
    islands: tuple[tuple[Point, ...], ...]
    dynamic_obstacles: tuple[Circle, ...]
    safezones: tuple[tuple[Point, ...], ...]
    use_preloaded_map: bool
    limits: VehicleLimits
    budget: SearchBudget

    def __post_init__(self) -> None:
        if len(self.request_id) != 16:
            raise ValueError(f"request_id phải đúng 16 byte, nhận {len(self.request_id)}")
        if self.frame not in FRAMES:
            raise ValueError(f"frame không hợp lệ: {self.frame!r}, chờ một trong {FRAMES}")


@dataclass(frozen=True)
class Waypoint:
    """Một điểm trên đường bay trả về."""

    position: Point
    heading_deg: float


@dataclass(frozen=True)
class SearchStats:
    """Bộ đếm mô tả một lần chạy search.

    ``budget_bound`` là trường hạng nhất chứ không phải chi tiết ẩn: planner
    cắt theo đồng hồ, nên cùng một request trên máy tải nặng có thể ra đường bay
    khác. Che giấu điều đó khiến client tin vào một sự đảm bảo không tồn tại.
    """

    iterations: int
    max_iterations: int
    open_set_size: int
    search_failed: bool
    budget_bound: bool


@dataclass(frozen=True)
class PlanReply:
    """Kết quả trả về client."""

    request_id: bytes
    idl_version: int
    status: PlanStatus
    detail: str
    waypoints: tuple[Waypoint, ...]
    path_length_m: float
    plan_wall_time_s: float
    stats: SearchStats
    planner_version: str
    config_hash: str

    @property
    def ok(self) -> bool:
        return self.status is PlanStatus.OK
```

Create `service/worker/vtx_planner/__init__.py`:

```python
"""Bọc thuật toán path planning thành một API thuần Python.

Package này không biết Fast DDS là gì và không được import bất kỳ thứ gì liên
quan tới transport. Xem docs/superpowers/specs/2026-08-22-dds-path-planning-service-design.md
"""

from __future__ import annotations

from vtx_planner.messages import (
    IDL_VERSION,
    Circle,
    PlanReply,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    SearchStats,
    VehicleLimits,
    Waypoint,
)

__all__ = [
    "IDL_VERSION",
    "Circle",
    "PlanReply",
    "PlanRequest",
    "PlanStatus",
    "SearchBudget",
    "SearchStats",
    "VehicleLimits",
    "Waypoint",
]
```

- [ ] **Step 8: Chạy test**

Run: `python -m pytest service/tests/ -v`
Expected: PASS, 7 passed.

- [ ] **Step 9: Xác nhận baseline gốc không đổi**

Run: `python -m pytest -q tests/ 2>&1 | tail -3`
Expected: `188 passed, 6 failed` (hoặc số tương đương đã ghi trong Global Constraints). Nếu khác, DỪNG và báo cáo — không sửa test để nó xanh.

- [ ] **Step 10: Commit**

```bash
git add service/
git commit -m "feat(service): scaffold vtx_planner with request/reply types and a boundary guard"
```

---

### Task 2: Đổi phương vị sang quy ước góc của thuật toán

**Files:**
- Create: `service/worker/vtx_planner/angles.py`
- Test: `service/tests/angles_test.py`

**Interfaces:**
- Consumes: không.
- Produces: `bearing_deg_to_math_rad(bearing_deg: float) -> float`, `math_rad_to_bearing_deg(theta_rad: float) -> float`. Task 3, 4, 6 dùng.

- [ ] **Step 1: Viết test**

Create `service/tests/angles_test.py`:

```python
"""Quy ước hướng là chỗ dễ sinh lỗi nhất của toàn service.

Một đường bay lệch 90 độ hoặc bị gương vẫn là đường bay hợp lệ về hình học, nên
mọi test hình học khác đều bỏ lọt loại lỗi này. Nó phải bị chặn ở đây.
"""

from __future__ import annotations

import math

import pytest

from vtx_planner.angles import bearing_deg_to_math_rad, math_rad_to_bearing_deg

# phương vị (độ, thuận kim đồng hồ từ bắc) -> heading toán học (rad, ngược kim
# đồng hồ từ +x). Quy ước: +y là bắc, +x là đông.
KNOWN = [
    (0.0, math.pi / 2),        # bắc  -> +y
    (90.0, 0.0),               # đông -> +x
    (180.0, -math.pi / 2),     # nam  -> -y
    (270.0, math.pi),          # tây  -> -x
    (45.0, math.pi / 4),       # đông bắc
]


@pytest.mark.parametrize("bearing_deg,expected_rad", KNOWN)
def test_cardinal_directions(bearing_deg: float, expected_rad: float) -> None:
    got = bearing_deg_to_math_rad(bearing_deg)
    assert math.isclose(math.cos(got), math.cos(expected_rad), abs_tol=1e-12)
    assert math.isclose(math.sin(got), math.sin(expected_rad), abs_tol=1e-12)


@pytest.mark.parametrize("bearing_deg", [0.0, 12.5, 90.0, 179.9, 180.0, 270.0, 359.99])
def test_round_trip_is_stable(bearing_deg: float) -> None:
    back = math_rad_to_bearing_deg(bearing_deg_to_math_rad(bearing_deg))
    assert math.isclose(back, bearing_deg, abs_tol=1e-9)


def test_result_is_normalised_to_a_single_turn() -> None:
    assert 0.0 <= math_rad_to_bearing_deg(bearing_deg_to_math_rad(730.0)) < 360.0


def test_negative_and_wrapped_bearings_agree() -> None:
    # So HƯỚNG, không so cách biểu diễn: -pi và +pi là cùng một hướng, và một
    # hàm chuẩn hoá đúng được phép trả về cái nào cũng được.
    a = bearing_deg_to_math_rad(-90.0)
    b = bearing_deg_to_math_rad(270.0)
    assert math.isclose(math.cos(a), math.cos(b), abs_tol=1e-12)
    assert math.isclose(math.sin(a), math.sin(b), abs_tol=1e-12)


def test_bearing_increases_clockwise_not_counterclockwise() -> None:
    """Phép thử phân biệt hai quy ước. Nếu ai đó lật dấu, test này đỏ."""
    north = bearing_deg_to_math_rad(0.0)
    slightly_east_of_north = bearing_deg_to_math_rad(10.0)
    # Quay theo chiều kim đồng hồ trên mặt đất = giảm góc toán học.
    assert slightly_east_of_north < north


def test_range_is_what_the_docstring_promises() -> None:
    """Chính khẳng định này đã bắt lỗi trong bản nháp đầu của module.

    Một cách viết bằng số học modulo trả về 4,712 rad ở phương vị 180 độ - đúng
    HƯỚNG, nên mọi test cos/sin ở trên vẫn xanh, nhưng ngoài dải mà docstring
    hứa. Sai lệch kiểu đó chỉ lộ ra ở downstream, nơi có ai đó so sánh hai góc
    trực tiếp thay vì so cos/sin.
    """
    for degrees in range(0, 360, 7):
        assert -math.pi <= bearing_deg_to_math_rad(float(degrees)) <= math.pi
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/angles_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_planner.angles'`.

- [ ] **Step 3: Viết `angles.py`**

Create `service/worker/vtx_planner/angles.py`:

```python
"""Đổi giữa phương vị đối ngoại và quy ước góc nội bộ của thuật toán.

Trên dây: ĐỘ, phương vị thật, thuận chiều kim đồng hồ từ chính bắc.
Trong `core/`: RADIAN, ngược chiều kim đồng hồ từ trục +x.
Frame phẳng quy ước +y là bắc, +x là đông.

Toàn bộ service đổi đơn vị góc ở đúng module này. Rải phép đổi ra nhiều chỗ là
cách chắc chắn nhất để có hai quy ước cùng tồn tại mà không ai nhận ra.
"""

from __future__ import annotations

import math


def bearing_deg_to_math_rad(bearing_deg: float) -> float:
    """Đổi phương vị (độ, thuận kim đồng hồ từ bắc) sang heading toán học (rad).

    Args:
        bearing_deg: Phương vị thật. Giá trị ngoài ``[0, 360)`` được chuẩn hoá.

    Returns:
        Góc radian ngược chiều kim đồng hồ từ ``+x``, trong ``[-pi, pi]``.
    """
    theta = math.radians(90.0 - bearing_deg)
    # atan2(sin, cos) chuẩn hoá về [-pi, pi]. Không dùng số học modulo: dạng
    # `radians((90 - b) % 360 - 180) + pi` trả về 4,712 rad ở phương vị 180 độ,
    # tức ra ngoài dải mà docstring hứa - đã đo, không phải phỏng đoán.
    return math.atan2(math.sin(theta), math.cos(theta))


def math_rad_to_bearing_deg(theta_rad: float) -> float:
    """Đổi heading toán học (rad) sang phương vị (độ, thuận kim đồng hồ từ bắc).

    Args:
        theta_rad: Góc ngược chiều kim đồng hồ từ ``+x``.

    Returns:
        Phương vị thật trong ``[0, 360)``.
    """
    return (90.0 - math.degrees(theta_rad)) % 360.0
```

- [ ] **Step 4: Chạy test**

Run: `python -m pytest service/tests/angles_test.py -v`
Expected: PASS, 16 passed.

- [ ] **Step 5: Commit**

```bash
git add service/worker/vtx_planner/angles.py service/tests/angles_test.py
git commit -m "feat(service): convert true bearings to the planner's angle convention"
```

---

### Task 3: Phép chiếu toạ độ và tịnh tiến về góc phần tư dương

**Files:**
- Create: `service/worker/vtx_planner/projection.py`
- Test: `service/tests/projection_test.py`

**Interfaces:**
- Consumes: `vtx_planner.angles.bearing_deg_to_math_rad`, `math_rad_to_bearing_deg` (Task 2).
- Produces: lớp `Projector` với `Projector.identity()`, `Projector.for_wgs84(anchor_lonlat, points, pad_m=10000.0)`, và các phương thức `forward(point)`, `inverse(point)`, `forward_bearing(bearing_deg, at_point)`, `inverse_bearing(theta_rad, at_projected_point)`. Task 4 và 6 dùng.

- [ ] **Step 1: Viết test**

Create `service/tests/projection_test.py`:

```python
"""Phép chiếu và phép tịnh tiến, kể cả cái bẫy toạ độ âm.

`core.kinodynamic_astar_v0._in_bounds` kiểm tra `0 < x < w and 0 < y < h`, neo
tại gốc và đòi hỏi dương ngặt. AEQD neo tại trung điểm sinh ra toạ độ quanh 0,
tức khoảng một nửa là số âm. Hôm nay không sao vì service không đặt bound, nhưng
đó là bẫy đang chờ, nên phép tịnh tiến là một bất biến được test.
"""

from __future__ import annotations

import math

import pytest

from vtx_planner.angles import bearing_deg_to_math_rad
from vtx_planner.projection import Projector

HANOI = (105.8342, 21.0278)
DA_NANG = (108.2022, 16.0544)


def test_identity_returns_the_very_same_values() -> None:
    """Bit-identical, không phải xấp xỉ: đây là điều test tương đương dựa vào."""
    proj = Projector.identity()
    point = (123456.789012345, -98765.43210987)
    assert proj.forward(point) == point
    assert proj.inverse(point) == point


def test_identity_only_converts_the_angle() -> None:
    proj = Projector.identity()
    assert proj.forward_bearing(90.0, (0.0, 0.0)) == bearing_deg_to_math_rad(90.0)


def _wgs84_projector() -> Projector:
    return Projector.for_wgs84(points=[HANOI, DA_NANG])


def test_round_trip_is_accurate_to_a_micrometre() -> None:
    proj = _wgs84_projector()
    for lonlat in (HANOI, DA_NANG, (106.5, 19.0)):
        back = proj.inverse(proj.forward(lonlat))
        # 1e-11 độ ~ 1e-6 m ở vĩ độ này.
        assert math.isclose(back[0], lonlat[0], abs_tol=1e-11)
        assert math.isclose(back[1], lonlat[1], abs_tol=1e-11)


def test_every_projected_coordinate_is_strictly_positive() -> None:
    proj = _wgs84_projector()
    for lonlat in (HANOI, DA_NANG):
        x, y = proj.forward(lonlat)
        assert x > 0.0 and y > 0.0


def test_projected_distance_matches_the_geodesic_within_the_documented_bound() -> None:
    """AEQD bảo toàn khoảng cách xuyên tâm; spec ghi cận sai số ~0,03%."""
    from pyproj import Geod

    proj = _wgs84_projector()
    geod = Geod(ellps="WGS84")
    _, _, geodesic_m = geod.inv(HANOI[0], HANOI[1], DA_NANG[0], DA_NANG[1])
    planar_m = math.dist(proj.forward(HANOI), proj.forward(DA_NANG))
    assert abs(planar_m - geodesic_m) / geodesic_m < 3e-4


def test_bearing_survives_the_projection() -> None:
    """Phương vị đo bằng trắc địa phải khớp góc đo trên mặt phẳng đã chiếu."""
    from pyproj import Geod

    proj = _wgs84_projector()
    geod = Geod(ellps="WGS84")
    azimuth_deg, _, _ = geod.inv(HANOI[0], HANOI[1], DA_NANG[0], DA_NANG[1])

    theta = proj.forward_bearing(azimuth_deg, HANOI)
    x0, y0 = proj.forward(HANOI)
    x1, y1 = proj.forward(DA_NANG)
    measured = math.atan2(y1 - y0, x1 - x0)
    assert math.isclose(math.cos(theta), math.cos(measured), abs_tol=1e-4)
    assert math.isclose(math.sin(theta), math.sin(measured), abs_tol=1e-4)


def test_bearing_round_trips_through_the_projection() -> None:
    proj = _wgs84_projector()
    for bearing in (0.0, 45.0, 137.5, 270.0):
        theta = proj.forward_bearing(bearing, HANOI)
        back = proj.inverse_bearing(theta, proj.forward(HANOI))
        assert math.isclose(back, bearing, abs_tol=1e-3)


def test_padding_keeps_geometry_clear_of_the_axes() -> None:
    proj = Projector.for_wgs84(points=[HANOI, DA_NANG], pad_m=25000.0)
    xs_ys = [proj.forward(p) for p in (HANOI, DA_NANG)]
    assert min(min(x, y) for x, y in xs_ys) >= 25000.0 - 1e-6


def test_wgs84_projector_needs_at_least_one_point() -> None:
    with pytest.raises(ValueError, match="ít nhất một điểm"):
        Projector.for_wgs84(points=[])
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/projection_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_planner.projection'`.

- [ ] **Step 3: Cài `pyproj` nếu chưa có**

Run: `python -c "import pyproj; print(pyproj.__version__)"`
Nếu lỗi: `pip install pyproj`.

- [ ] **Step 4: Viết `projection.py`**

Create `service/worker/vtx_planner/projection.py`:

```python
"""Chiếu toạ độ địa lý về mặt phẳng mét mà thuật toán làm việc trên đó.

Hai chế độ. ``identity`` dành cho ``FRAME_LOCAL_METERS``: client đã nói bằng
chính hệ toạ độ của thuật toán, nên không có phép toán nào chạm vào toạ độ -
điều này là cố ý, vì test tương đương dựa vào việc đường đi qua service giữ
nguyên từng bit.

``for_wgs84`` dùng phép chiếu phương vị cách đều (AEQD) neo tại tâm hình học của
mission. AEQD bảo toàn chính xác khoảng cách theo phương xuyên tâm từ tâm chiếu;
sai số theo phương tiếp tuyến xấp xỉ ``(c/R)^2 / 6``, tức ~0,03% ở mép một
mission 500 km. UTM bị loại vì múi rộng 6 độ, một mission 500 km có thể cắt qua
hai múi.

Sau khi chiếu, toàn bộ hình học được TỊNH TIẾN về góc phần tư dương.
``core.kinodynamic_astar_v0._in_bounds`` kiểm tra ``0 < x < w and 0 < y < h`` -
neo tại gốc và dương ngặt - nên toạ độ âm là một cái bẫy đang chờ ngày ai đó
thêm bound vào. Tịnh tiến không đổi khoảng cách hay phương vị, nên nó không ảnh
hưởng gì tới đường bay.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from vtx_planner.angles import bearing_deg_to_math_rad, math_rad_to_bearing_deg
from vtx_planner.messages import Point

_BEARING_PROBE_M = 1000.0
"""Bước phụ dùng để đo phương vị sau khi chiếu.

Phương vị tại một điểm bất kỳ không chiếu được bằng một công thức đóng, nên nó
được ĐO: đi một bước trắc địa ngắn theo phương vị đó, chiếu cả hai đầu, rồi lấy
atan2. Sai số là bậc hai theo bước, và 1 km là nhỏ so với mission 500 km.
"""

DEFAULT_PAD_M = 10000.0
"""Khoảng đệm giữa hình học đã chiếu và hai trục toạ độ."""


@dataclass(frozen=True)
class Projector:
    """Chiếu hai chiều giữa toạ độ client và mặt phẳng làm việc của thuật toán."""

    _crs: object | None
    _dx: float
    _dy: float

    @classmethod
    def identity(cls) -> Projector:
        """Không chiếu, không tịnh tiến. Dành cho ``FRAME_LOCAL_METERS``."""
        return cls(_crs=None, _dx=0.0, _dy=0.0)

    @classmethod
    def for_wgs84(
        cls, points: Sequence[Point], pad_m: float = DEFAULT_PAD_M
    ) -> Projector:
        """Dựng phép chiếu AEQD neo tại tâm của ``points``, đã tịnh tiến.

        Args:
            points: Mọi toạ độ ``(lon, lat)`` của mission. Tâm chiếu và phép
                tịnh tiến đều suy ra từ tập này, nên nó phải đủ để bao trọn
                mission - thiếu một đảo là đảo đó có thể rơi vào toạ độ âm.
            pad_m: Khoảng đệm tối thiểu tới hai trục.

        Returns:
            Một ``Projector`` đã sẵn sàng.

        Raises:
            ValueError: Khi ``points`` rỗng.
        """
        if not points:
            raise ValueError("for_wgs84 cần ít nhất một điểm để neo phép chiếu")

        lon0 = sum(p[0] for p in points) / len(points)
        lat0 = sum(p[1] for p in points) / len(points)

        from pyproj import CRS, Transformer

        crs = CRS.from_proj4(
            f"+proj=aeqd +lat_0={lat0} +lon_0={lon0} +datum=WGS84 +units=m +no_defs"
        )
        to_plane = Transformer.from_crs("EPSG:4326", crs, always_xy=True)

        raw = [to_plane.transform(lon, lat) for lon, lat in points]
        dx = pad_m - min(x for x, _ in raw)
        dy = pad_m - min(y for _, y in raw)
        return cls(_crs=crs, _dx=dx, _dy=dy)

    # --- toạ độ ---------------------------------------------------------------

    def forward(self, point: Point) -> Point:
        """Đưa một toạ độ client về mặt phẳng làm việc."""
        if self._crs is None:
            return point
        x, y = self._to_plane().transform(point[0], point[1])
        return (x + self._dx, y + self._dy)

    def inverse(self, point: Point) -> Point:
        """Đưa một toạ độ mặt phẳng làm việc trở lại hệ của client."""
        if self._crs is None:
            return point
        return self._to_geo().transform(point[0] - self._dx, point[1] - self._dy)

    # --- góc ------------------------------------------------------------------

    def forward_bearing(self, bearing_deg: float, at_point: Point) -> float:
        """Đổi một phương vị tại ``at_point`` thành heading toán học đã chiếu.

        Args:
            bearing_deg: Phương vị thật, độ, thuận chiều kim đồng hồ từ bắc.
            at_point: Vị trí trong hệ toạ độ CLIENT nơi phương vị được đo.

        Returns:
            Heading radian, ngược chiều kim đồng hồ từ ``+x`` của mặt phẳng.
        """
        if self._crs is None:
            return bearing_deg_to_math_rad(bearing_deg)

        from pyproj import Geod

        lon1, lat1, _ = Geod(ellps="WGS84").fwd(
            at_point[0], at_point[1], bearing_deg, _BEARING_PROBE_M
        )
        x0, y0 = self.forward(at_point)
        x1, y1 = self.forward((lon1, lat1))
        return math.atan2(y1 - y0, x1 - x0)

    def inverse_bearing(self, theta_rad: float, at_projected: Point) -> float:
        """Đổi một heading toán học tại ``at_projected`` trở lại phương vị thật.

        Args:
            theta_rad: Heading radian trên mặt phẳng làm việc.
            at_projected: Vị trí trên mặt phẳng làm việc nơi heading được đo.

        Returns:
            Phương vị thật trong ``[0, 360)``.
        """
        if self._crs is None:
            return math_rad_to_bearing_deg(theta_rad)

        from pyproj import Geod

        ahead = (
            at_projected[0] + _BEARING_PROBE_M * math.cos(theta_rad),
            at_projected[1] + _BEARING_PROBE_M * math.sin(theta_rad),
        )
        lon0, lat0 = self.inverse(at_projected)
        lon1, lat1 = self.inverse(ahead)
        azimuth_deg, _, _ = Geod(ellps="WGS84").inv(lon0, lat0, lon1, lat1)
        return azimuth_deg % 360.0

    # --- nội bộ ---------------------------------------------------------------

    def _to_plane(self):  # noqa: ANN202 - kiểu do pyproj cung cấp
        from pyproj import Transformer

        return Transformer.from_crs("EPSG:4326", self._crs, always_xy=True)

    def _to_geo(self):  # noqa: ANN202 - kiểu do pyproj cung cấp
        from pyproj import Transformer

        return Transformer.from_crs(self._crs, "EPSG:4326", always_xy=True)
```

- [ ] **Step 5: Chạy test**

Run: `python -m pytest service/tests/projection_test.py -v`
Expected: PASS, 11 passed.

- [ ] **Step 6: Commit**

```bash
git add service/worker/vtx_planner/projection.py service/tests/projection_test.py
git commit -m "feat(service): project WGS84 to the working plane, in the positive quadrant"
```

---

### Task 4: Dựng `Scenario` dict, và test hợp đồng khoá

**Files:**
- Create: `service/worker/vtx_planner/scenario_builder.py`
- Test: `service/tests/scenario_builder_test.py`

**Interfaces:**
- Consumes: `PlanRequest`, `Circle` (Task 1); `Projector` (Task 3).
- Produces: `build_scenario(request: PlanRequest, projector: Projector) -> dict[str, object]`, và `projector_for(request: PlanRequest) -> Projector`. Task 6 dùng cả hai.

- [ ] **Step 1: Viết test**

Create `service/tests/scenario_builder_test.py`:

```python
"""Cơ chế cưỡng chế số 1: adapter phải điền đủ mọi khoá `Scenario` khai báo.

Nếu `core.types.Scenario` mọc thêm một khoá bắt buộc, test này đỏ ngay thay vì
để một `KeyError` nổ ra giữa lúc chạy thật.
"""

from __future__ import annotations

import math
from typing import get_type_hints

from core.types import Scenario

from vtx_planner.messages import Circle, PlanRequest, SearchBudget, VehicleLimits
from vtx_planner.projection import Projector
from vtx_planner.scenario_builder import build_scenario, projector_for

LIMITS = VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0)
BUDGET = SearchBudget(15.0, 50000)


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x01" * 16,
        idl_version=1,
        frame="local_meters",
        start=(50000.0, 50000.0),
        start_heading_deg=90.0,
        goal=(300000.0, 200000.0),
        goal_heading_deg=45.0,
        goal_heading_free=False,
        islands=(((100000.0, 100000.0), (120000.0, 100000.0), (110000.0, 130000.0)),),
        dynamic_obstacles=(Circle(center=(200000.0, 150000.0), radius_m=12000.0),),
        safezones=(),
        use_preloaded_map=False,
        limits=LIMITS,
        budget=BUDGET,
    )
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def test_builder_fills_every_key_the_scenario_type_declares() -> None:
    built = build_scenario(_request(), Projector.identity())
    assert set(built) == set(get_type_hints(Scenario))


def test_identity_frame_keeps_coordinates_bit_identical() -> None:
    req = _request()
    built = build_scenario(req, Projector.identity())
    assert built["start"] == req.start
    assert built["goal"] == req.goal
    assert built["islands"][0][0] == req.islands[0][0]


def test_headings_are_converted_to_the_planner_convention() -> None:
    built = build_scenario(_request(start_heading_deg=90.0), Projector.identity())
    # phương vị 90 = đông = +x = 0 rad
    assert math.isclose(built["start_heading"], 0.0, abs_tol=1e-12)


def test_free_goal_becomes_none_not_a_sentinel_number() -> None:
    built = build_scenario(_request(goal_heading_free=True), Projector.identity())
    assert built["goal_heading"] is None


def test_map_bounds_is_deliberately_absent() -> None:
    """Spec bỏ map_bounds khỏi IDL: nó neo tại gốc toạ độ, thứ không có nghĩa
    ổn định sau phép chiếu. safezones biểu diễn được đúng vùng đó và bất biến
    với tịnh tiến."""
    built = build_scenario(_request(), Projector.identity())
    assert built["map_bounds"] is None


def test_empty_safezones_becomes_none_so_the_planner_stays_permissive() -> None:
    built = build_scenario(_request(safezones=()), Projector.identity())
    assert built["safezones"] is None


def test_obstacles_is_the_tagged_union_the_pipeline_consumes() -> None:
    built = build_scenario(_request(), Projector.identity())
    kinds = sorted(o["type"] for o in built["obstacles"])
    assert kinds == ["circle", "polygon"]
    circle = next(o for o in built["obstacles"] if o["type"] == "circle")
    assert circle["center"] == (200000.0, 150000.0)
    assert circle["radius"] == 12000.0


def test_the_built_scenario_actually_runs_through_the_pipeline() -> None:
    import core.kinodynamic_astar_v0 as astar
    import core.preprocessing as prep

    built = build_scenario(_request(), Projector.identity())
    pre = prep.prepare_scenario(
        built,
        turn_radius=LIMITS.turn_radius_m,
        l0=LIMITS.l0_m,
        dss=LIMITS.dss_m,
        safe_margin=LIMITS.safe_margin_m,
        alpha_max_rad=math.radians(LIMITS.alpha_max_deg),
    )
    result = astar.plan_trajectory(pre)
    assert result["success"] is True


def test_projector_for_local_frame_is_the_identity() -> None:
    assert projector_for(_request()).forward((1.5, -2.5)) == (1.5, -2.5)


def test_projector_for_wgs84_sees_every_obstacle_vertex() -> None:
    """Thiếu một đỉnh khi neo phép chiếu là để đảo đó rơi vào toạ độ âm."""
    req = _request(
        frame="wgs84",
        start=(105.8342, 21.0278),
        goal=(108.2022, 16.0544),
        islands=(((106.0, 18.0), (106.5, 18.0), (106.25, 18.5)),),
        dynamic_obstacles=(Circle(center=(107.0, 19.0), radius_m=12000.0),),
    )
    proj = projector_for(req)
    for lonlat in (req.start, req.goal, *req.islands[0], req.dynamic_obstacles[0].center):
        x, y = proj.forward(lonlat)
        assert x > 0.0 and y > 0.0
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/scenario_builder_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_planner.scenario_builder'`.

- [ ] **Step 3: Viết `scenario_builder.py`**

Create `service/worker/vtx_planner/scenario_builder.py`:

```python
"""Dịch một ``PlanRequest`` thành đúng dict ``Scenario`` mà pipeline tiêu thụ.

Đây là toàn bộ phần "dịch" của adapter. Nó không tính toán hình học và không
biết gì về search: mọi khoá đều lấy thẳng từ ``core.types.Scenario``, để một
khoá mới bên đó làm test hợp đồng đỏ chứ không làm production nổ.
"""

from __future__ import annotations

from typing import Any

from vtx_planner.messages import PlanRequest, Point
from vtx_planner.projection import Projector


def projector_for(request: PlanRequest) -> Projector:
    """Chọn phép chiếu phù hợp với frame của request.

    Args:
        request: Mission cần lập kế hoạch.

    Returns:
        ``Projector.identity()`` cho frame mét phẳng; với WGS84 là một phép
        chiếu AEQD neo trên TOÀN BỘ hình học của mission, không chỉ hai đầu
        mút - bỏ sót một đỉnh đảo là để đỉnh đó rơi ra ngoài vùng đã tịnh tiến.
    """
    if request.frame == "local_meters":
        return Projector.identity()

    points: list[Point] = [request.start, request.goal]
    for polygon in request.islands:
        points.extend(polygon)
    for zone in request.safezones:
        points.extend(zone)
    points.extend(circle.center for circle in request.dynamic_obstacles)
    return Projector.for_wgs84(points=points)


def build_scenario(request: PlanRequest, projector: Projector) -> dict[str, Any]:
    """Dựng dict ``Scenario`` từ một request đã được chiếu.

    Args:
        request: Mission cần lập kế hoạch.
        projector: Phép chiếu từ :func:`projector_for`.

    Returns:
        Một dict mang đúng tập khoá của ``core.types.Scenario``, sẵn sàng cho
        ``core.preprocessing.prepare_scenario``.
    """
    islands = [[projector.forward(v) for v in polygon] for polygon in request.islands]
    circles = [
        (projector.forward(c.center), c.radius_m) for c in request.dynamic_obstacles
    ]
    safezones = [[projector.forward(v) for v in zone] for zone in request.safezones]

    obstacles: list[dict[str, Any]] = [
        {"type": "polygon", "polygon": polygon} for polygon in islands
    ]
    obstacles.extend(
        {"type": "circle", "center": center, "radius": radius}
        for center, radius in circles
    )

    goal_heading = (
        None
        if request.goal_heading_free
        else projector.forward_bearing(request.goal_heading_deg, request.goal)
    )

    return {
        "start": projector.forward(request.start),
        "start_heading": projector.forward_bearing(
            request.start_heading_deg, request.start
        ),
        "goal": projector.forward(request.goal),
        "goal_heading": goal_heading,
        # Cố tình None: xem docstring của test map_bounds, và mục 4 của spec.
        "map_bounds": None,
        "safezones": safezones or None,
        "islands": islands,
        "dynamic_obstacles": circles,
        "obstacles": obstacles,
    }
```

- [ ] **Step 4: Chạy test**

Run: `python -m pytest service/tests/scenario_builder_test.py -v`
Expected: PASS, 10 passed.

- [ ] **Step 5: Commit**

```bash
git add service/worker/vtx_planner/scenario_builder.py service/tests/scenario_builder_test.py
git commit -m "feat(service): build the Scenario dict, with a key-contract guard"
```

---

### Task 5: Ngân sách chạy, `config_hash`, `planner_version`

**Files:**
- Create: `service/worker/vtx_planner/runtime.py`
- Test: `service/tests/runtime_test.py`

**Interfaces:**
- Consumes: `SearchBudget` (Task 1).
- Produces: `applied_budget(budget: SearchBudget)` (context manager), `planner_config_snapshot() -> dict[str, object]`, `config_hash() -> str`, `planner_version() -> str`. Task 6 dùng cả bốn.

- [ ] **Step 1: Viết test**

Create `service/tests/runtime_test.py`:

```python
"""Ngân sách chạy và siêu dữ liệu phiên bản.

Test quan trọng nhất ở đây là `test_budget_override_actually_binds`.
`scripts/ab_planners.py` ghi lại một cái bẫy: một số hằng số config được suy ra
ở MỨC MODULE lúc import, nên override sau đó bị bỏ qua trong im lặng và phép
A/B trông như đã chạy mà thực ra không đo gì. Với hai knob service dựa vào,
override lúc chạy CÓ hiệu lực - v0 đọc MAX_ITERATIONS trong __init__ và
TIME_BUDGET_S trong vòng lặp search - và test này ghim điều đó lại.
"""

from __future__ import annotations

import config
import core.kinodynamic_astar_v0 as astar
import core.map_generator as mg
import core.preprocessing as prep

from vtx_planner.messages import SearchBudget
from vtx_planner.runtime import (
    applied_budget,
    config_hash,
    planner_config_snapshot,
    planner_version,
)


def test_budget_override_actually_binds() -> None:
    scenario = mg.get_all_scenarios()["scenario_16_extreme_complexity"]()
    pre = prep.prepare_scenario(scenario)
    with applied_budget(SearchBudget(time_budget_s=15.0, max_iterations=5)):
        result = astar.plan_trajectory(pre)
    assert result["stats"]["max_iterations"] == 5
    assert result["success"] is False


def test_budget_is_restored_even_when_planning_raises() -> None:
    before = (config.TIME_BUDGET_S, config.MAX_ITERATIONS)
    try:
        with applied_budget(SearchBudget(1.0, 7)):
            raise RuntimeError("bùm")
    except RuntimeError:
        pass
    assert (config.TIME_BUDGET_S, config.MAX_ITERATIONS) == before


def test_snapshot_is_discovered_not_hardcoded() -> None:
    snapshot = planner_config_snapshot()
    # Vài knob chắc chắn v0 đọc. Không khẳng định TỔNG SỐ: con số đó phải được
    # phép thay đổi khi thuật toán đổi, đó chính là mục đích của cơ chế này.
    for name in ("TIME_BUDGET_S", "MAX_ITERATIONS", "NUM_START_CORNERS", "GOAL_THRESHOLD"):
        assert name in snapshot
    assert len(snapshot) > 20


def test_snapshot_excludes_constants_the_shipped_planner_never_reads() -> None:
    # CIRCLE_GRAZE_TOL_M bị khai tử và không planner nào đọc nó.
    assert "CIRCLE_GRAZE_TOL_M" not in planner_config_snapshot()


def test_hash_is_stable_and_sensitive() -> None:
    first = config_hash()
    assert first == config_hash()
    original = config.NUM_START_CORNERS
    try:
        config.NUM_START_CORNERS = original + 1
        assert config_hash() != first
    finally:
        config.NUM_START_CORNERS = original
    assert config_hash() == first


def test_version_is_a_non_empty_string() -> None:
    version = planner_version()
    assert isinstance(version, str) and version
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/runtime_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_planner.runtime'`.

- [ ] **Step 3: Viết `runtime.py`**

Create `service/worker/vtx_planner/runtime.py`:

```python
"""Ngân sách chạy và siêu dữ liệu phiên bản đi kèm mỗi reply.

Client phải phân biệt được hai đường bay khác nhau là do input khác hay do cấu
hình planner khác. Trên một codebase nghiên cứu nơi các hằng số được A/B liên
tục, đó không phải tiện nghi mà là điều kiện để reply có nghĩa.
"""

from __future__ import annotations

import hashlib
import inspect
import json
import re
import subprocess
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

import config
import core.kinodynamic_astar_v0 as astar

from vtx_planner.messages import SearchBudget

_CONFIG_REF = re.compile(r"\bconfig\.([A-Z][A-Z0-9_]*)\b")
_REPO_ROOT = Path(__file__).resolve().parents[3]


@contextmanager
def applied_budget(budget: SearchBudget) -> Iterator[None]:
    """Áp ngân sách của một request rồi khôi phục nguyên trạng.

    Hai hằng số này là global; đây là đường duy nhất để override chúng. Cách này
    an toàn vì service xử lý một request tại một thời điểm. Nếu về sau có xử lý
    song song, chỗ này phải đổi thành fork-per-request - xem mục 5 của spec.

    Args:
        budget: Ngân sách của request.

    Yields:
        Không có gì; ngữ cảnh chỉ để đánh dấu phạm vi.
    """
    saved_time = config.TIME_BUDGET_S
    saved_iterations = config.MAX_ITERATIONS
    config.TIME_BUDGET_S = budget.time_budget_s
    config.MAX_ITERATIONS = budget.max_iterations
    try:
        yield
    finally:
        config.TIME_BUDGET_S = saved_time
        config.MAX_ITERATIONS = saved_iterations


def planner_config_snapshot() -> dict[str, object]:
    """Liệt kê các hằng số ``config`` mà planner đang ship thực sự đọc.

    Danh sách được PHÁT HIỆN bằng cách quét mã nguồn của planner chứ không
    hardcode, nên một knob mới xuất hiện trong reply mà không ai phải nhớ cập
    nhật chỗ này.

    Returns:
        Ánh xạ tên hằng số sang giá trị hiện tại, sắp theo tên.
    """
    names = sorted(set(_CONFIG_REF.findall(inspect.getsource(astar))))
    return {name: getattr(config, name) for name in names if hasattr(config, name)}


def config_hash() -> str:
    """Băm rút gọn của cấu hình planner hiện hành.

    Returns:
        16 ký tự hex đầu của SHA-256 trên snapshot đã chuẩn hoá.
    """
    blob = json.dumps(planner_config_snapshot(), sort_keys=True, default=repr)
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:16]


def planner_version() -> str:
    """Mô tả phiên bản mã nguồn đang chạy.

    Returns:
        Kết quả ``git describe --always --dirty``, hoặc ``"unknown"`` khi không
        chạy được (bản triển khai không có git, hoặc không phải một repo).
    """
    try:
        proc = subprocess.run(
            ["git", "describe", "--always", "--dirty"],
            cwd=_REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=5,
            check=True,
        )
    except (OSError, subprocess.SubprocessError):
        return "unknown"
    return proc.stdout.strip() or "unknown"
```

- [ ] **Step 4: Chạy test**

Run: `python -m pytest service/tests/runtime_test.py -v`
Expected: PASS, 6 passed.

- [ ] **Step 5: Commit**

```bash
git add service/worker/vtx_planner/runtime.py service/tests/runtime_test.py
git commit -m "feat(service): per-request budget, discovered config hash, version stamp"
```

---

### Task 6: `plan()` và ánh xạ trạng thái

**Files:**
- Create: `service/worker/vtx_planner/planner.py`
- Modify: `service/worker/vtx_planner/__init__.py`
- Test: `service/tests/planner_test.py`

**Interfaces:**
- Consumes: mọi thứ từ Task 1-5.
- Produces: `plan(request: PlanRequest, preloaded: PreloadedMap | None = None) -> PlanReply`. Task 7 và 9 dùng. Tham số `preloaded` được thêm ở Task 9; tới lúc đó `plan(request)` là đủ.

- [ ] **Step 1: Viết test**

Create `service/tests/planner_test.py`:

```python
from __future__ import annotations

import math

from vtx_planner import plan
from vtx_planner.messages import (
    Circle,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    VehicleLimits,
)

LIMITS = VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0)


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x02" * 16,
        idl_version=1,
        frame="local_meters",
        start=(50000.0, 50000.0),
        start_heading_deg=45.0,
        goal=(300000.0, 250000.0),
        goal_heading_deg=45.0,
        goal_heading_free=True,
        islands=(),
        dynamic_obstacles=(),
        safezones=(),
        use_preloaded_map=False,
        limits=LIMITS,
        budget=SearchBudget(15.0, 50000),
    )
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def test_open_water_mission_succeeds() -> None:
    reply = plan(_request())
    assert reply.status is PlanStatus.OK
    assert reply.detail == ""
    assert len(reply.waypoints) >= 2
    assert reply.path_length_m > 0.0


def test_reply_echoes_the_request_id_and_idl_version() -> None:
    req = _request(request_id=b"\x07" * 16)
    reply = plan(req)
    assert reply.request_id == req.request_id
    assert reply.idl_version == req.idl_version


def test_path_starts_at_takeoff_and_ends_at_the_target() -> None:
    req = _request()
    reply = plan(req)
    assert reply.waypoints[0].position == req.start
    assert reply.waypoints[-1].position == req.goal


def test_first_waypoint_keeps_the_requested_takeoff_bearing() -> None:
    req = _request(start_heading_deg=45.0)
    reply = plan(req)
    assert math.isclose(reply.waypoints[0].heading_deg, 45.0, abs_tol=1e-6)


def test_reply_carries_version_and_config_identity() -> None:
    reply = plan(_request())
    assert reply.planner_version
    assert len(reply.config_hash) == 16


def test_a_goal_buried_in_an_obstacle_fails_honestly() -> None:
    reply = plan(
        _request(
            goal=(300000.0, 250000.0),
            dynamic_obstacles=(Circle(center=(300000.0, 250000.0), radius_m=40000.0),),
        )
    )
    assert reply.status is not PlanStatus.OK
    assert reply.detail != ""


def test_a_tiny_iteration_budget_reports_budget_bound() -> None:
    reply = plan(
        _request(
            islands=(
                ((150000.0, 120000.0), (200000.0, 120000.0), (175000.0, 200000.0)),
                ((220000.0, 60000.0), (280000.0, 60000.0), (250000.0, 190000.0)),
            ),
            budget=SearchBudget(15.0, 3),
        )
    )
    assert reply.stats.budget_bound is True
    assert reply.stats.max_iterations == 3


def test_wall_time_is_measured_and_positive() -> None:
    reply = plan(_request())
    assert reply.plan_wall_time_s > 0.0


def test_wgs84_mission_returns_waypoints_in_wgs84() -> None:
    reply = plan(
        _request(
            frame="wgs84",
            start=(105.8342, 21.0278),
            goal=(108.2022, 16.0544),
        )
    )
    assert reply.status is PlanStatus.OK
    for waypoint in reply.waypoints:
        assert 100.0 < waypoint.position[0] < 115.0
        assert 10.0 < waypoint.position[1] < 25.0


def test_wgs84_endpoints_round_trip_to_the_requested_coordinates() -> None:
    req = _request(frame="wgs84", start=(105.8342, 21.0278), goal=(108.2022, 16.0544))
    reply = plan(req)
    assert math.isclose(reply.waypoints[0].position[0], req.start[0], abs_tol=1e-9)
    assert math.isclose(reply.waypoints[0].position[1], req.start[1], abs_tol=1e-9)
    assert math.isclose(reply.waypoints[-1].position[0], req.goal[0], abs_tol=1e-9)
    assert math.isclose(reply.waypoints[-1].position[1], req.goal[1], abs_tol=1e-9)
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/planner_test.py -v`
Expected: FAIL — `ImportError: cannot import name 'plan' from 'vtx_planner'`.

- [ ] **Step 3: Viết `planner.py`**

Create `service/worker/vtx_planner/planner.py`:

```python
"""Điểm vào của service: một mission vào, một đường bay ra.

Đây là chỗ DUY NHẤT trong service gọi tới planner. Mọi thứ nó dùng từ ``core/``
đều là hàm công khai đang tồn tại, không có bản sao nào: ``prepare_scenario``
chuẩn bị bài toán, ``plan_trajectory`` giải nó, ``full_mission_path`` ghép hai
đầu mút vào. Thuật toán đổi thì đường đi này đổi theo mà không cần sửa gì.
"""

from __future__ import annotations

import math
import time
from typing import Any

import core.kinodynamic_astar_v0 as astar
import core.mission as mission
import core.preprocessing as prep

from vtx_planner.messages import (
    IDL_VERSION,
    PlanReply,
    PlanRequest,
    PlanStatus,
    SearchStats,
    Waypoint,
)
from vtx_planner.projection import Projector
from vtx_planner.runtime import applied_budget, config_hash, planner_version
from vtx_planner.scenario_builder import build_scenario, projector_for

_REASON_TO_STATUS = {
    "no_path": PlanStatus.NO_PATH,
    "start_leg_blocked": PlanStatus.START_LEG_BLOCKED,
    "goal_leg_blocked": PlanStatus.GOAL_LEG_BLOCKED,
}
"""Tập lý do có mã riêng. Mọi chuỗi khác đến từ oracle và mang tham số, nên nó
đi nguyên văn vào ``detail`` thay vì bị ép vào một enum làm mất thông tin."""


def plan(request: PlanRequest) -> PlanReply:
    """Lập kế hoạch cho một mission.

    Args:
        request: Mission cần giải, ở frame mét phẳng hoặc WGS84.

    Returns:
        Đường bay đầy đủ ``O..T`` trong hệ toạ độ của request, kèm trạng thái,
        bộ đếm search và nhận dạng phiên bản/cấu hình.
    """
    started = time.perf_counter()
    projector = projector_for(request)
    scenario = build_scenario(request, projector)

    preprocessed = prep.prepare_scenario(
        scenario,
        turn_radius=request.limits.turn_radius_m,
        l0=request.limits.l0_m,
        dss=request.limits.dss_m,
        safe_margin=request.limits.safe_margin_m,
        alpha_max_rad=math.radians(request.limits.alpha_max_deg),
    )

    search_started = time.perf_counter()
    with applied_budget(request.budget):
        result = astar.plan_trajectory(preprocessed)
    search_elapsed = time.perf_counter() - search_started

    status, detail = _classify(result)
    waypoints = _waypoints_out(result, preprocessed, projector)
    length_m = _planar_length(result, preprocessed)

    return PlanReply(
        request_id=request.request_id,
        idl_version=IDL_VERSION,
        status=status,
        detail=detail,
        waypoints=waypoints,
        path_length_m=length_m,
        plan_wall_time_s=time.perf_counter() - started,
        stats=_stats_out(result, request, search_elapsed),
        planner_version=planner_version(),
        config_hash=config_hash(),
    )


def _classify(result: dict[str, Any]) -> tuple[PlanStatus, str]:
    """Ánh xạ kết quả planner sang trạng thái đối ngoại và phần diễn giải."""
    if result["success"]:
        return PlanStatus.OK, ""
    reason = result["failure_reason"] or ""
    return _REASON_TO_STATUS.get(reason, PlanStatus.ORACLE_REJECTED), reason


def _full_path(result: dict[str, Any], preprocessed: dict[str, Any]) -> list[Any]:
    """Đường bay đầy đủ ``O..T``, hoặc rỗng khi không có đường nào."""
    if not result["path"]:
        return []
    return mission.full_mission_path(result["path"], preprocessed)


def _waypoints_out(
    result: dict[str, Any], preprocessed: dict[str, Any], projector: Projector
) -> tuple[Waypoint, ...]:
    """Đưa đường bay trở lại hệ toạ độ và quy ước góc của client."""
    return tuple(
        Waypoint(
            position=projector.inverse(position),
            heading_deg=projector.inverse_bearing(heading, position),
        )
        for position, heading in _full_path(result, preprocessed)
    )


def _planar_length(result: dict[str, Any], preprocessed: dict[str, Any]) -> float:
    """Tổng chiều dài các dây cung trên mặt phẳng làm việc.

    Đo trên mặt phẳng đã chiếu chứ không trên hệ của client, vì đó chính là đại
    lượng mà search tối ưu - và là cùng công thức ``scripts/ab_planners.py``
    dùng, nên số liệu so sánh được với các benchmark đã ghi.
    """
    full = _full_path(result, preprocessed)
    return sum(math.dist(full[i][0], full[i + 1][0]) for i in range(len(full) - 1))


def _stats_out(
    result: dict[str, Any], request: PlanRequest, search_elapsed: float
) -> SearchStats:
    """Đóng gói bộ đếm search, kèm cờ cho biết ngân sách có chạm trần không."""
    stats = result["stats"]
    budget_bound = (
        stats["iterations"] >= stats["max_iterations"]
        or search_elapsed >= request.budget.time_budget_s
    )
    return SearchStats(
        iterations=stats["iterations"],
        max_iterations=stats["max_iterations"],
        open_set_size=stats["open_set_size"],
        search_failed=stats["search_failed"],
        budget_bound=budget_bound,
    )
```

- [ ] **Step 4: Export `plan` từ package**

Modify `service/worker/vtx_planner/__init__.py` — thêm import và mục `__all__`:

```python
from vtx_planner.planner import plan
```

và thêm `"plan"` vào danh sách `__all__`.

- [ ] **Step 5: Chạy test**

Run: `python -m pytest service/tests/planner_test.py -v`
Expected: PASS, 10 passed.

- [ ] **Step 6: Chạy toàn bộ test service**

Run: `python -m pytest service/tests/ -q`
Expected: tất cả PASS.

- [ ] **Step 7: Commit**

```bash
git add service/worker/vtx_planner/planner.py service/worker/vtx_planner/__init__.py service/tests/planner_test.py
git commit -m "feat(service): plan(PlanRequest) -> PlanReply over the shipped v0 planner"
```

---

### Task 7: Test tương đương — cơ chế cưỡng chế "thuật toán đổi thì tự cập nhật"

**Files:**
- Test: `service/tests/equivalence_test.py`

**Interfaces:**
- Consumes: `plan` (Task 6), `build_scenario` / `projector_for` (Task 4).
- Produces: không có mã production. Đây là bộ gác.

Task này chỉ có test — đó là chủ đích. Nó là cơ chế số 2 trong spec, và là thứ khiến service tự đi theo thuật toán mà không cần sửa tay.

- [ ] **Step 1: Viết test tương đương**

Create `service/tests/equivalence_test.py`:

```python
"""Cơ chế cưỡng chế số 2: adapter không được làm sai lệch bất cứ điều gì.

Hai khẳng định tách bạch, và việc tách chúng ra là có lý do.

`test_adapter_is_transparent` so đường bay đi qua service với đường bay khi gọi
thẳng thuật toán TRÊN CÙNG MỘT dict Scenario. Yêu cầu là bit-identical. Cả hai
vế đều gọi thuật toán HIỆN HÀNH, nên test không bao giờ lỗi thời: thuật toán
đổi thì hai vế đổi cùng nhau và test vẫn xanh; adapter lệch đi thì đỏ ngay.

`test_every_preset_still_solves_through_the_service` chạy 18 preset qua service.
Nó KHÔNG đòi bit-identical, vì có một khác biệt ngữ nghĩa cố ý: preset mang
`map_bounds = (500000, 500000)` còn IDL bỏ trường đó, nên service chạy ở chế độ
không giới hạn. Đòi bit-identical ở đây sẽ là ép hai thứ khác nhau phải giống
nhau. Cái nó gác là điều thực sự quan trọng: service không được làm mất mission,
và không được làm đường bay dài ra.
"""

from __future__ import annotations

import math

import core.kinodynamic_astar_v0 as astar
import core.map_generator as mg
import core.mission as mission
import core.preprocessing as prep
import pytest

import config
from vtx_planner import plan
from vtx_planner.angles import math_rad_to_bearing_deg
from vtx_planner.messages import (
    Circle,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    VehicleLimits,
)
from vtx_planner.projection import Projector
from vtx_planner.scenario_builder import build_scenario

LIMITS = VehicleLimits(
    turn_radius_m=config.R,
    l0_m=config.L0,
    dss_m=config.DSS,
    safe_margin_m=config.SAFE_MARGIN,
    alpha_max_deg=config.ALPHA_MAX,
)
# config.TIME_BUDGET_S có kiểu `float | None`; None nghĩa là không giới hạn, mà
# SearchBudget lại đòi một số dương. 15.0 là giá trị đang cấu hình.
BUDGET = SearchBudget(
    time_budget_s=float(config.TIME_BUDGET_S if config.TIME_BUDGET_S else 15.0),
    max_iterations=config.MAX_ITERATIONS,
)
SCENARIOS = sorted(mg.get_all_scenarios())


def _request_from_scenario(name: str) -> PlanRequest:
    """Dựng một request tương đương với một preset, ở frame mét phẳng."""
    scenario = mg.get_all_scenarios()[name]()
    goal_heading = scenario["goal_heading"]
    return PlanRequest(
        request_id=name.encode("utf-8")[:16].ljust(16, b"\x00"),
        idl_version=1,
        frame="local_meters",
        start=scenario["start"],
        start_heading_deg=math_rad_to_bearing_deg(scenario["start_heading"]),
        goal=scenario["goal"],
        goal_heading_deg=0.0 if goal_heading is None else math_rad_to_bearing_deg(goal_heading),
        goal_heading_free=goal_heading is None,
        islands=tuple(tuple(tuple(v) for v in poly) for poly in scenario["islands"]),
        dynamic_obstacles=tuple(
            Circle(center=tuple(center), radius_m=radius)
            for center, radius in scenario["dynamic_obstacles"]
        ),
        safezones=(),
        use_preloaded_map=False,
        limits=LIMITS,
        budget=BUDGET,
    )


def _direct_plan(request: PlanRequest):
    """Gọi thẳng thuật toán trên CHÍNH dict Scenario mà adapter dựng ra."""
    scenario = build_scenario(request, Projector.identity())
    preprocessed = prep.prepare_scenario(
        scenario,
        turn_radius=request.limits.turn_radius_m,
        l0=request.limits.l0_m,
        dss=request.limits.dss_m,
        safe_margin=request.limits.safe_margin_m,
        alpha_max_rad=math.radians(request.limits.alpha_max_deg),
    )
    result = astar.plan_trajectory(preprocessed)
    full = mission.full_mission_path(result["path"], preprocessed) if result["path"] else []
    return result, full


@pytest.mark.parametrize("name", SCENARIOS)
def test_adapter_is_transparent(name: str) -> None:
    request = _request_from_scenario(name)
    result, expected_full = _direct_plan(request)
    reply = plan(request)

    assert (reply.status is PlanStatus.OK) == result["success"]
    assert len(reply.waypoints) == len(expected_full)
    for got, (position, heading) in zip(reply.waypoints, expected_full, strict=True):
        # Bit-identical: projector identity không thực hiện phép toán nào lên toạ độ.
        assert got.position == position
        assert got.heading_deg == pytest.approx(
            math_rad_to_bearing_deg(heading), abs=1e-9
        )

    expected_length = sum(
        math.dist(expected_full[i][0], expected_full[i + 1][0])
        for i in range(len(expected_full) - 1)
    )
    assert reply.path_length_m == pytest.approx(expected_length, rel=0.0, abs=1e-9)
    assert reply.stats.iterations == result["stats"]["iterations"]


def test_every_preset_still_solves_through_the_service() -> None:
    failures = [
        name for name in SCENARIOS if plan(_request_from_scenario(name)).status is not PlanStatus.OK
    ]
    assert failures == [], f"service làm mất mission: {failures}"


@pytest.mark.parametrize("name", SCENARIOS)
def test_service_does_not_lengthen_the_route_against_the_preset(name: str) -> None:
    """So với preset NGUYÊN BẢN (còn map_bounds), không phải với dict của adapter."""
    scenario = mg.get_all_scenarios()[name]()
    preprocessed = prep.prepare_scenario(scenario)
    result = astar.plan_trajectory(preprocessed)
    full = mission.full_mission_path(result["path"], preprocessed)
    baseline = sum(math.dist(full[i][0], full[i + 1][0]) for i in range(len(full) - 1))

    reply = plan(_request_from_scenario(name))
    assert reply.status is PlanStatus.OK
    assert reply.path_length_m <= baseline * 1.005
```

- [ ] **Step 2: Chạy test**

Run: `python -m pytest service/tests/equivalence_test.py -q`
Expected: PASS, 37 passed (18 + 1 + 18). Chạy mất vài chục giây vì `scenario_18` tốn ~4 s và chạy ba lần.

Nếu `test_adapter_is_transparent` đỏ ở bất kỳ preset nào: đó là một khiếm khuyết THẬT của adapter, không phải test quá khắt khe. Dừng lại và tìm nguyên nhân; hai vế đang chạy trên cùng một dict Scenario nên chúng phải trùng khớp.

Nếu `test_service_does_not_lengthen_the_route_against_the_preset` đỏ: nghĩa là việc bỏ `map_bounds` có hệ quả đo được. Ghi lại preset nào, chênh bao nhiêu, và BÁO CÁO — đó là dữ liệu để quyết định có đưa `map_bounds` (hoặc một safezone hình chữ nhật) vào IDL hay không, chứ không phải cái để nới ngưỡng cho qua.

- [ ] **Step 3: Commit**

```bash
git add service/tests/equivalence_test.py
git commit -m "test(service): pin the adapter to the algorithm, bit for bit"
```

---

### Task 8: Codec msgpack và đóng khung

**Files:**
- Create: `service/worker/vtx_planner/codec.py`
- Test: `service/tests/codec_test.py`

**Interfaces:**
- Consumes: các dataclass từ Task 1.
- Produces: `encode_request(req) -> bytes`, `decode_request(bytes) -> PlanRequest`, `encode_reply(reply) -> bytes`, `decode_reply(bytes) -> PlanReply`, `frame(payload: bytes) -> bytes`, `read_frame(sock) -> bytes | None`. Task 9 và Part 2 dùng.

- [ ] **Step 1: Viết test**

Create `service/tests/codec_test.py`:

```python
from __future__ import annotations

import socket

import pytest

from vtx_planner.codec import (
    decode_reply,
    decode_request,
    encode_reply,
    encode_request,
    frame,
    read_frame,
)
from vtx_planner.messages import (
    Circle,
    PlanReply,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    SearchStats,
    VehicleLimits,
    Waypoint,
)


def _request() -> PlanRequest:
    return PlanRequest(
        request_id=bytes(range(16)),
        idl_version=1,
        frame="wgs84",
        start=(105.8342, 21.0278),
        start_heading_deg=137.5,
        goal=(108.2022, 16.0544),
        goal_heading_deg=42.0,
        goal_heading_free=False,
        islands=(((106.0, 18.0), (106.5, 18.0), (106.25, 18.5)),),
        dynamic_obstacles=(Circle(center=(107.0, 19.0), radius_m=12000.0),),
        safezones=(((105.0, 15.0), (109.0, 15.0), (109.0, 22.0), (105.0, 22.0)),),
        use_preloaded_map=True,
        limits=VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0),
        budget=SearchBudget(15.0, 50000),
    )


def _reply() -> PlanReply:
    return PlanReply(
        request_id=bytes(range(16)),
        idl_version=1,
        status=PlanStatus.ORACLE_REJECTED,
        detail="first W1..W2 l=7421.3 < L0=8000",
        waypoints=(Waypoint((105.8342, 21.0278), 137.5), Waypoint((108.2022, 16.0544), 42.0)),
        path_length_m=612345.678,
        plan_wall_time_s=0.0421,
        stats=SearchStats(1234, 50000, 56, True, False),
        planner_version="v1.0-3-gabc1234-dirty",
        config_hash="0123456789abcdef",
    )


def test_request_round_trips_exactly() -> None:
    original = _request()
    assert decode_request(encode_request(original)) == original


def test_reply_round_trips_exactly() -> None:
    original = _reply()
    assert decode_reply(encode_reply(original)) == original


def test_floats_survive_bit_for_bit() -> None:
    """msgpack phải mang double, không phải float32. Sai chỗ này là mất mét."""
    tricky = 123456.78901234567
    original = _reply()
    replaced = PlanReply(**{**original.__dict__, "path_length_m": tricky})
    assert decode_reply(encode_reply(replaced)).path_length_m == tricky


def test_status_survives_as_an_enum_not_a_bare_int() -> None:
    decoded = decode_reply(encode_reply(_reply()))
    assert isinstance(decoded.status, PlanStatus)


def test_frame_prefixes_a_big_endian_length() -> None:
    framed = frame(b"abcd")
    assert framed == b"\x00\x00\x00\x04abcd"


def test_read_frame_reassembles_a_split_payload() -> None:
    left, right = socket.socketpair()
    try:
        payload = b"x" * 100000
        blob = frame(payload)
        left.sendall(blob[:7])
        left.sendall(blob[7:])
        left.shutdown(socket.SHUT_WR)
        assert read_frame(right) == payload
    finally:
        left.close()
        right.close()


def test_read_frame_returns_none_on_a_clean_close() -> None:
    left, right = socket.socketpair()
    try:
        left.close()
        assert read_frame(right) is None
    finally:
        right.close()


def test_read_frame_rejects_a_truncated_body() -> None:
    left, right = socket.socketpair()
    try:
        left.sendall(frame(b"abcdefgh")[:8])
        left.shutdown(socket.SHUT_WR)
        with pytest.raises(ConnectionError):
            read_frame(right)
    finally:
        left.close()
        right.close()
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/codec_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_planner.codec'`.

- [ ] **Step 3: Cài `msgpack` nếu chưa có**

Run: `python -c "import msgpack; print(msgpack.version)"`
Nếu lỗi: `pip install msgpack`.

- [ ] **Step 4: Viết `codec.py`**

Create `service/worker/vtx_planner/codec.py`:

```python
"""Giao thức nội bộ giữa node DDS và worker.

Khung là một tiền tố độ dài 4 byte big-endian, thân là msgpack. Đơn giản đủ để
viết lại bằng C++ trong vài chục dòng, và không có phần nào tự mô tả kiểu - hai
đầu đã thống nhất bố cục qua `IDL_VERSION`.

Đây là giao thức nội bộ trên một Unix socket, không phải bề mặt mạng. Bề mặt
mạng duy nhất của hệ thống là DDS.
"""

from __future__ import annotations

import socket
import struct
from typing import Any

import msgpack

from vtx_planner.messages import (
    Circle,
    PlanReply,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    SearchStats,
    VehicleLimits,
    Waypoint,
)

_HEADER = struct.Struct("!I")
MAX_FRAME_BYTES = 64 * 1024 * 1024
"""Trần kích thước khung, để một tiền tố hỏng không xin cấp phát vô hạn."""


def frame(payload: bytes) -> bytes:
    """Bọc một thân tin thành khung có tiền tố độ dài."""
    return _HEADER.pack(len(payload)) + payload


def read_frame(sock: socket.socket) -> bytes | None:
    """Đọc trọn một khung từ socket.

    Args:
        sock: Socket dòng đã kết nối.

    Returns:
        Thân tin, hoặc ``None`` khi đối phương đóng kết nối sạch sẽ trước khi
        gửi byte nào.

    Raises:
        ConnectionError: Khi kết nối đứt giữa chừng một khung, hoặc tiền tố độ
            dài vượt :data:`MAX_FRAME_BYTES`.
    """
    header = _read_exactly(sock, _HEADER.size, allow_empty=True)
    if header is None:
        return None
    (length,) = _HEADER.unpack(header)
    if length > MAX_FRAME_BYTES:
        raise ConnectionError(f"khung {length} byte vượt trần {MAX_FRAME_BYTES}")
    body = _read_exactly(sock, length, allow_empty=False)
    assert body is not None  # allow_empty=False đã loại trường hợp None
    return body


def _read_exactly(sock: socket.socket, count: int, *, allow_empty: bool) -> bytes | None:
    chunks: list[bytes] = []
    remaining = count
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            if not chunks and allow_empty:
                return None
            raise ConnectionError(f"kết nối đứt sau {count - remaining}/{count} byte")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


# --- request ------------------------------------------------------------------


def encode_request(request: PlanRequest) -> bytes:
    """Tuần tự hoá một request."""
    return msgpack.packb(
        {
            "request_id": request.request_id,
            "idl_version": request.idl_version,
            "frame": request.frame,
            "start": list(request.start),
            "start_heading_deg": request.start_heading_deg,
            "goal": list(request.goal),
            "goal_heading_deg": request.goal_heading_deg,
            "goal_heading_free": request.goal_heading_free,
            "islands": [[list(v) for v in poly] for poly in request.islands],
            "dynamic_obstacles": [
                {"center": list(c.center), "radius_m": c.radius_m}
                for c in request.dynamic_obstacles
            ],
            "safezones": [[list(v) for v in zone] for zone in request.safezones],
            "use_preloaded_map": request.use_preloaded_map,
            "limits": [
                request.limits.turn_radius_m,
                request.limits.l0_m,
                request.limits.dss_m,
                request.limits.safe_margin_m,
                request.limits.alpha_max_deg,
            ],
            "budget": [request.budget.time_budget_s, request.budget.max_iterations],
        },
        use_bin_type=True,
    )


def decode_request(blob: bytes) -> PlanRequest:
    """Giải tuần tự một request."""
    raw: dict[str, Any] = msgpack.unpackb(blob, raw=False)
    return PlanRequest(
        request_id=raw["request_id"],
        idl_version=raw["idl_version"],
        frame=raw["frame"],
        start=tuple(raw["start"]),
        start_heading_deg=raw["start_heading_deg"],
        goal=tuple(raw["goal"]),
        goal_heading_deg=raw["goal_heading_deg"],
        goal_heading_free=raw["goal_heading_free"],
        islands=tuple(tuple(tuple(v) for v in poly) for poly in raw["islands"]),
        dynamic_obstacles=tuple(
            Circle(center=tuple(c["center"]), radius_m=c["radius_m"])
            for c in raw["dynamic_obstacles"]
        ),
        safezones=tuple(tuple(tuple(v) for v in zone) for zone in raw["safezones"]),
        use_preloaded_map=raw["use_preloaded_map"],
        limits=VehicleLimits(*raw["limits"]),
        budget=SearchBudget(raw["budget"][0], raw["budget"][1]),
    )


# --- reply --------------------------------------------------------------------


def encode_reply(reply: PlanReply) -> bytes:
    """Tuần tự hoá một reply."""
    return msgpack.packb(
        {
            "request_id": reply.request_id,
            "idl_version": reply.idl_version,
            "status": int(reply.status),
            "detail": reply.detail,
            "waypoints": [[w.position[0], w.position[1], w.heading_deg] for w in reply.waypoints],
            "path_length_m": reply.path_length_m,
            "plan_wall_time_s": reply.plan_wall_time_s,
            "stats": [
                reply.stats.iterations,
                reply.stats.max_iterations,
                reply.stats.open_set_size,
                reply.stats.search_failed,
                reply.stats.budget_bound,
            ],
            "planner_version": reply.planner_version,
            "config_hash": reply.config_hash,
        },
        use_bin_type=True,
    )


def decode_reply(blob: bytes) -> PlanReply:
    """Giải tuần tự một reply."""
    raw: dict[str, Any] = msgpack.unpackb(blob, raw=False)
    return PlanReply(
        request_id=raw["request_id"],
        idl_version=raw["idl_version"],
        status=PlanStatus(raw["status"]),
        detail=raw["detail"],
        waypoints=tuple(Waypoint((x, y), heading) for x, y, heading in raw["waypoints"]),
        path_length_m=raw["path_length_m"],
        plan_wall_time_s=raw["plan_wall_time_s"],
        stats=SearchStats(*raw["stats"]),
        planner_version=raw["planner_version"],
        config_hash=raw["config_hash"],
    )
```

- [ ] **Step 5: Chạy test**

Run: `python -m pytest service/tests/codec_test.py -v`
Expected: PASS, 8 passed.

- [ ] **Step 6: Commit**

```bash
git add service/worker/vtx_planner/codec.py service/tests/codec_test.py
git commit -m "feat(service): length-prefixed msgpack codec for the internal socket"
```

---

### Task 9: Tiến trình worker

**Files:**
- Create: `service/worker/run_worker.py`
- Create: `service/deploy/worker-requirements.txt`
- Test: `service/tests/worker_ipc_test.py`

**Interfaces:**
- Consumes: `plan` (Task 6), codec (Task 8).
- Produces: một tiến trình phục vụ trên Unix socket. Part 2 là client của nó.

- [ ] **Step 1: Viết test cho bản đồ nền tĩnh**

Create `service/tests/preloaded_map_test.py`:

```python
"""Bản đồ nền tĩnh: nạp một lần lúc khởi động, gộp vào request khi được yêu cầu.

Mặc định triển khai là KHÔNG nạp bản đồ nào - request tự chứa thì dễ replay và
dễ chẩn đoán hơn nhiều. Bản đồ nền là tuỳ chọn, và khi client yêu cầu một thứ
service không có thì nó bị TỪ CHỐI rõ ràng chứ không âm thầm bỏ qua cờ.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from vtx_planner import plan
from vtx_planner.messages import Circle, PlanRequest, PlanStatus, SearchBudget, VehicleLimits
from vtx_planner.preloaded_map import PreloadedMap

LIMITS = VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0)


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x04" * 16,
        idl_version=1,
        frame="local_meters",
        start=(50000.0, 50000.0),
        start_heading_deg=45.0,
        goal=(300000.0, 250000.0),
        goal_heading_deg=45.0,
        goal_heading_free=True,
        islands=(),
        dynamic_obstacles=(),
        safezones=(),
        use_preloaded_map=False,
        limits=LIMITS,
        budget=SearchBudget(15.0, 50000),
    )
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def _write_map(tmp_path: Path, frame: str = "local_meters") -> Path:
    path = tmp_path / "basemap.json"
    path.write_text(
        json.dumps(
            {
                "frame": frame,
                "islands": [[[150000.0, 120000.0], [200000.0, 120000.0], [175000.0, 200000.0]]],
                "dynamic_obstacles": [{"center": [220000.0, 180000.0], "radius_m": 15000.0}],
            }
        ),
        encoding="utf-8",
    )
    return path


def test_load_reads_both_obstacle_kinds(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(_write_map(tmp_path))
    assert loaded.frame == "local_meters"
    assert len(loaded.islands) == 1
    assert loaded.dynamic_obstacles == (Circle(center=(220000.0, 180000.0), radius_m=15000.0),)


def test_merge_appends_to_whatever_the_request_carries(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(_write_map(tmp_path))
    request = _request(
        use_preloaded_map=True,
        dynamic_obstacles=(Circle(center=(100000.0, 100000.0), radius_m=5000.0),),
    )
    merged = loaded.merged_into(request)
    assert len(merged.islands) == 1
    assert len(merged.dynamic_obstacles) == 2
    # Chướng ngại vật của request đứng trước, để đọc log dễ đối chiếu.
    assert merged.dynamic_obstacles[0].radius_m == 5000.0


def test_flag_off_leaves_the_request_untouched(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(_write_map(tmp_path))
    request = _request(use_preloaded_map=False)
    assert loaded.merged_into(request) is request


def test_asking_for_a_map_the_service_does_not_have_is_refused() -> None:
    reply = plan(_request(use_preloaded_map=True), preloaded=None)
    assert reply.status is PlanStatus.INVALID_REQUEST
    assert "preloaded" in reply.detail


def test_a_frame_mismatch_is_refused_not_silently_reinterpreted(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(_write_map(tmp_path, frame="wgs84"))
    reply = plan(_request(use_preloaded_map=True), preloaded=loaded)
    assert reply.status is PlanStatus.INVALID_REQUEST
    assert "frame" in reply.detail


def test_the_merged_map_actually_changes_the_route(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(_write_map(tmp_path))
    open_water = plan(_request())
    with_basemap = plan(_request(use_preloaded_map=True), preloaded=loaded)
    assert open_water.status is PlanStatus.OK
    assert with_basemap.status is PlanStatus.OK
    assert with_basemap.path_length_m > open_water.path_length_m


def test_load_rejects_a_file_without_a_frame(tmp_path: Path) -> None:
    path = tmp_path / "bad.json"
    path.write_text(json.dumps({"islands": [], "dynamic_obstacles": []}), encoding="utf-8")
    with pytest.raises(ValueError, match="frame"):
        PreloadedMap.load(path)
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/preloaded_map_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_planner.preloaded_map'`.

- [ ] **Step 3: Viết `preloaded_map.py`**

Create `service/worker/vtx_planner/preloaded_map.py`:

```python
"""Bản đồ nền tĩnh, nạp một lần lúc worker khởi động.

Mặc định triển khai là không có bản đồ nào: một request tự chứa thì replay được
và chẩn đoán được, còn state ẩn trong service thì không. Bản đồ nền tồn tại cho
trường hợp bản đồ nền quá lớn để gửi kèm mỗi request.

File khai báo frame CỦA CHÍNH NÓ, và frame đó phải khớp request. Diễn giải lại
toạ độ mét thành độ, hay ngược lại, sẽ tạo ra một bản đồ hợp lệ về cú pháp và
vô nghĩa về vị trí - loại lỗi không có test hình học nào bắt được.
"""

from __future__ import annotations

import dataclasses
import json
from dataclasses import dataclass
from pathlib import Path

from vtx_planner.messages import FRAMES, Circle, PlanRequest, Point


@dataclass(frozen=True)
class PreloadedMap:
    """Chướng ngại vật nền, trong một frame đã khai báo."""

    frame: str
    islands: tuple[tuple[Point, ...], ...]
    dynamic_obstacles: tuple[Circle, ...]

    @classmethod
    def load(cls, path: Path) -> PreloadedMap:
        """Đọc một bản đồ nền từ file JSON.

        Args:
            path: File JSON có các khoá ``frame``, ``islands``,
                ``dynamic_obstacles``.

        Returns:
            Bản đồ đã nạp.

        Raises:
            ValueError: Khi thiếu ``frame`` hoặc frame không hợp lệ.
        """
        raw = json.loads(path.read_text(encoding="utf-8"))
        frame = raw.get("frame")
        if frame not in FRAMES:
            raise ValueError(
                f"bản đồ nền phải khai báo frame thuộc {FRAMES}, nhận {frame!r}"
            )
        return cls(
            frame=frame,
            islands=tuple(
                tuple((float(v[0]), float(v[1])) for v in polygon)
                for polygon in raw.get("islands", [])
            ),
            dynamic_obstacles=tuple(
                Circle(
                    center=(float(c["center"][0]), float(c["center"][1])),
                    radius_m=float(c["radius_m"]),
                )
                for c in raw.get("dynamic_obstacles", [])
            ),
        )

    def merged_into(self, request: PlanRequest) -> PlanRequest:
        """Gộp bản đồ nền vào một request, nếu request yêu cầu.

        Args:
            request: Request gốc.

        Returns:
            Request đã gộp, hoặc CHÍNH ``request`` khi cờ tắt - trả về cùng đối
            tượng để chỗ gọi phân biệt được "không gộp" với "gộp rỗng".

        Raises:
            ValueError: Khi frame của bản đồ khác frame của request.
        """
        if not request.use_preloaded_map:
            return request
        if self.frame != request.frame:
            raise ValueError(
                f"frame của bản đồ nền ({self.frame}) khác request ({request.frame})"
            )
        return dataclasses.replace(
            request,
            islands=request.islands + self.islands,
            dynamic_obstacles=request.dynamic_obstacles + self.dynamic_obstacles,
        )
```

- [ ] **Step 4: Nối vào `plan()`**

Modify `service/worker/vtx_planner/planner.py`. Thêm import:

```python
from vtx_planner.preloaded_map import PreloadedMap
```

Đổi chữ ký và xử lý cờ ngay đầu hàm, TRƯỚC khi dựng phép chiếu:

```python
def plan(request: PlanRequest, preloaded: PreloadedMap | None = None) -> PlanReply:
    """Lập kế hoạch cho một mission.

    Args:
        request: Mission cần giải, ở frame mét phẳng hoặc WGS84.
        preloaded: Bản đồ nền tĩnh, hoặc ``None`` khi service không nạp bản đồ
            nào. Chỉ được dùng khi ``request.use_preloaded_map`` bật.

    Returns:
        Đường bay đầy đủ ``O..T`` trong hệ toạ độ của request, kèm trạng thái,
        bộ đếm search và nhận dạng phiên bản/cấu hình.
    """
    started = time.perf_counter()

    if request.use_preloaded_map:
        if preloaded is None:
            return _refusal(
                request, "yêu cầu preloaded map nhưng service không nạp bản đồ nào"
            )
        try:
            request = preloaded.merged_into(request)
        except ValueError as exc:
            return _refusal(request, str(exc))

    projector = projector_for(request)
    # ... phần còn lại giữ nguyên
```

Thêm hàm phụ ở cuối module:

```python
def _refusal(request: PlanRequest, detail: str) -> PlanReply:
    """Từ chối một request không hợp lệ, không chạy search."""
    return PlanReply(
        request_id=request.request_id,
        idl_version=IDL_VERSION,
        status=PlanStatus.INVALID_REQUEST,
        detail=detail,
        waypoints=(),
        path_length_m=0.0,
        plan_wall_time_s=0.0,
        stats=SearchStats(0, 0, 0, True, False),
        planner_version=planner_version(),
        config_hash=config_hash(),
    )
```

- [ ] **Step 5: Chạy test**

Run: `python -m pytest service/tests/preloaded_map_test.py service/tests/planner_test.py service/tests/equivalence_test.py -q`
Expected: tất cả PASS. Test tương đương phải VẪN xanh: `plan(request)` không truyền `preloaded` giữ nguyên hành vi cũ từng bit.

- [ ] **Step 6: Commit**

```bash
git add service/worker/vtx_planner/preloaded_map.py service/worker/vtx_planner/planner.py service/tests/preloaded_map_test.py
git commit -m "feat(service): optional static base map, refused loudly when it is missing"
```

- [ ] **Step 7: Viết test cho worker**

Create `service/tests/worker_ipc_test.py`:

```python
"""Worker phục vụ tuần tự trên một Unix socket, và không chết vì một request xấu."""

from __future__ import annotations

import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

from vtx_planner.codec import decode_reply, encode_request, frame, read_frame
from vtx_planner.messages import (
    PlanRequest,
    PlanStatus,
    SearchBudget,
    VehicleLimits,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
WORKER = REPO_ROOT / "service" / "worker" / "run_worker.py"


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x03" * 16,
        idl_version=1,
        frame="local_meters",
        start=(50000.0, 50000.0),
        start_heading_deg=45.0,
        goal=(300000.0, 250000.0),
        goal_heading_deg=45.0,
        goal_heading_free=True,
        islands=(),
        dynamic_obstacles=(),
        safezones=(),
        use_preloaded_map=False,
        limits=VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0),
        budget=SearchBudget(15.0, 50000),
    )
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


@pytest.fixture()
def worker(tmp_path: Path):
    sock_path = tmp_path / "planner.sock"
    proc = subprocess.Popen(
        [sys.executable, str(WORKER), "--socket", str(sock_path), "--repo-root", str(REPO_ROOT)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    deadline = time.monotonic() + 30.0
    while not sock_path.exists():
        if proc.poll() is not None:
            raise RuntimeError(f"worker chết khi khởi động: {proc.communicate()[1].decode()}")
        if time.monotonic() > deadline:
            proc.kill()
            raise TimeoutError("worker không tạo socket trong 30 s")
        time.sleep(0.05)
    try:
        yield sock_path
    finally:
        proc.terminate()
        proc.wait(timeout=10)


def _round_trip(sock_path: Path, request: PlanRequest):
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.connect(str(sock_path))
    try:
        client.sendall(frame(encode_request(request)))
        blob = read_frame(client)
        assert blob is not None
        return decode_reply(blob)
    finally:
        client.close()


def test_worker_plans_a_mission_over_the_socket(worker: Path) -> None:
    reply = _round_trip(worker, _request())
    assert reply.status is PlanStatus.OK
    assert len(reply.waypoints) >= 2


def test_worker_echoes_the_request_id(worker: Path) -> None:
    reply = _round_trip(worker, _request(request_id=b"\x09" * 16))
    assert reply.request_id == b"\x09" * 16


def test_worker_handles_two_requests_on_one_connection(worker: Path) -> None:
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.connect(str(worker))
    try:
        for tag in (b"\x0a", b"\x0b"):
            client.sendall(frame(encode_request(_request(request_id=tag * 16))))
            blob = read_frame(client)
            assert blob is not None
            assert decode_reply(blob).request_id == tag * 16
    finally:
        client.close()


def test_a_malformed_request_gets_invalid_request_not_a_dead_worker(worker: Path) -> None:
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.connect(str(worker))
    try:
        client.sendall(frame(b"\xc1\xc1 rác, không phải msgpack hợp lệ"))
        blob = read_frame(client)
        assert blob is not None
        assert decode_reply(blob).status is PlanStatus.INVALID_REQUEST
    finally:
        client.close()
    # Worker vẫn sống và vẫn phục vụ được.
    assert _round_trip(worker, _request()).status is PlanStatus.OK


def test_a_wrong_idl_version_is_refused(worker: Path) -> None:
    reply = _round_trip(worker, _request(idl_version=999))
    assert reply.status is PlanStatus.INVALID_REQUEST
    assert "idl_version" in reply.detail


def test_worker_survives_a_client_that_disconnects_mid_request(worker: Path) -> None:
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.connect(str(worker))
    client.sendall(frame(encode_request(_request()))[:6])
    client.close()
    assert _round_trip(worker, _request()).status is PlanStatus.OK
```

- [ ] **Step 8: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/worker_ipc_test.py -v`
Expected: FAIL — worker chết khi khởi động, `can't open file .../run_worker.py`.

- [ ] **Step 9: Viết `run_worker.py`**

Create `service/worker/run_worker.py`:

```python
#!/usr/bin/env python3
"""Tiến trình worker: nhận request trên Unix socket, trả đường bay.

Phục vụ TUẦN TỰ, một request tại một thời điểm. Đó là chủ đích: planner là
Python thuần CPU-bound, nên xử lý song song trong một tiến trình không mua được
gì, và hai knob ngân sách được override qua biến global module - an toàn đúng
khi chỉ có một request đang chạy.

Worker không tự đặt thời hạn cho chính mình. Planner tự dừng theo
`config.TIME_BUDGET_S`, còn thời hạn CỨNG do node DDS giữ: nó `SIGKILL` worker
rồi dựng lại. Đó là cách trung thực duy nhất, vì một vòng lặp search Python
không hủy được từ bên ngoài một cách lịch sự.
"""

from __future__ import annotations

import argparse
import contextlib
import logging
import os
import socket
import sys
from pathlib import Path


def _bootstrap_import_path(repo_root: Path) -> None:
    """Đưa gốc repo và thư mục worker lên sys.path.

    Worker import `core.*` THẲNG từ cây mã nguồn, không qua wheel. Đó là cơ chế
    cập nhật tự động ở mức triển khai: `git pull` là đã đổi thuật toán.
    """
    for path in (repo_root, repo_root / "service" / "worker"):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))


def _serve(sock_path: Path, preloaded_map_path: Path | None, log: logging.Logger) -> None:
    from vtx_planner import plan
    from vtx_planner.preloaded_map import PreloadedMap
    from vtx_planner.codec import (
        MAX_FRAME_BYTES,
        decode_request,
        encode_reply,
        frame,
        read_frame,
    )
    from vtx_planner.messages import (
        IDL_VERSION,
        PlanReply,
        PlanStatus,
        SearchStats,
    )
    from vtx_planner.runtime import config_hash, planner_version

    def _refuse(request_id: bytes, status: PlanStatus, detail: str) -> bytes:
        return encode_reply(
            PlanReply(
                request_id=request_id,
                idl_version=IDL_VERSION,
                status=status,
                detail=detail,
                waypoints=(),
                path_length_m=0.0,
                plan_wall_time_s=0.0,
                stats=SearchStats(0, 0, 0, True, False),
                planner_version=planner_version(),
                config_hash=config_hash(),
            )
        )

    preloaded = PreloadedMap.load(preloaded_map_path) if preloaded_map_path else None
    if preloaded is not None:
        log.info(
            "bản đồ nền: %d đảo, %d vòng tròn, frame %s",
            len(preloaded.islands),
            len(preloaded.dynamic_obstacles),
            preloaded.frame,
        )

    with contextlib.suppress(FileNotFoundError):
        sock_path.unlink()
    sock_path.parent.mkdir(parents=True, exist_ok=True)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(sock_path))
    os.chmod(sock_path, 0o660)
    server.listen(1)
    log.info("worker sẵn sàng trên %s (planner %s, config %s)", sock_path, planner_version(), config_hash())

    while True:
        connection, _ = server.accept()
        log.info("client đã kết nối")
        try:
            while True:
                try:
                    blob = read_frame(connection)
                except ConnectionError as exc:
                    log.warning("khung hỏng, đóng kết nối: %s", exc)
                    break
                if blob is None:
                    break

                unknown_id = b"\x00" * 16
                try:
                    request = decode_request(blob)
                except Exception as exc:  # noqa: BLE001 - biên giải mã, mọi lỗi đều là request xấu
                    log.warning("request không giải mã được: %s", exc)
                    connection.sendall(
                        frame(_refuse(unknown_id, PlanStatus.INVALID_REQUEST, f"giải mã lỗi: {exc}"))
                    )
                    continue

                if request.idl_version != IDL_VERSION:
                    connection.sendall(
                        frame(
                            _refuse(
                                request.request_id,
                                PlanStatus.INVALID_REQUEST,
                                f"idl_version {request.idl_version} != {IDL_VERSION}",
                            )
                        )
                    )
                    continue

                try:
                    reply = plan(request, preloaded=preloaded)
                except Exception as exc:  # noqa: BLE001 - báo lỗi thay vì chết
                    log.exception("lập kế hoạch thất bại")
                    connection.sendall(
                        frame(_refuse(request.request_id, PlanStatus.INTERNAL_ERROR, str(exc)))
                    )
                    continue

                log.info(
                    "request %s -> %s, %d waypoint, %.3f s",
                    request.request_id.hex()[:8],
                    reply.status.name,
                    len(reply.waypoints),
                    reply.plan_wall_time_s,
                )
                connection.sendall(frame(encode_reply(reply)))
        finally:
            connection.close()
            log.info("client đã ngắt")


def main() -> int:
    parser = argparse.ArgumentParser(description="VTX path planning worker")
    parser.add_argument("--socket", required=True, type=Path, help="đường dẫn Unix socket")
    parser.add_argument("--repo-root", required=True, type=Path, help="gốc repo chứa core/")
    parser.add_argument(
        "--preloaded-map",
        type=Path,
        default=None,
        help="file JSON bản đồ nền tĩnh; bỏ trống thì mọi request phải tự chứa",
    )
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()

    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)s vtx-worker %(message)s",
        stream=sys.stderr,
    )
    log = logging.getLogger("vtx-worker")

    _bootstrap_import_path(args.repo_root.resolve())
    try:
        _serve(args.socket, args.preloaded_map, log)
    except KeyboardInterrupt:
        log.info("nhận tín hiệu dừng")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 10: Chạy test**

Run: `python -m pytest service/tests/worker_ipc_test.py -v`
Expected: PASS, 6 passed.

- [ ] **Step 11: Ghi lại phụ thuộc**

Create `service/deploy/worker-requirements.txt`:

```
# Phụ thuộc của worker. BA gói, và không gói nào trong số đó là numpy.
#
# core/ không import numpy ở đâu cả (0 lần), cũng không scipy, cũng không
# matplotlib. Cái pin numpy==1.26.4 trong requirements.txt ở gốc repo là ràng
# buộc của matplotlib 3.8 / pandas 2.1.4 trong stack test-benchmark-GUI, và nó
# KHÔNG áp dụng ở đây. Đã kiểm chứng: venv sạch chỉ có shapely + msgpack kéo
# theo numpy 2.4.6 và cả 18 preset vẫn giải được.
shapely==2.1.2
msgpack==1.2.1
pyproj==3.7.2
```

- [ ] **Step 12: Xác minh trong một venv sạch**

```bash
python3.11 -m venv /tmp/vtx-worker-venv
/tmp/vtx-worker-venv/bin/pip install -q -r service/deploy/worker-requirements.txt pytest
PYTHONPATH=. /tmp/vtx-worker-venv/bin/python -m pytest -q service/tests/
```
Expected: mọi test PASS. Nếu có `ModuleNotFoundError`, nghĩa là service đã lỡ phụ thuộc vào một gói ngoài danh sách — thêm nó vào file requirements HOẶC bỏ chỗ dùng nó, đừng cài lén vào venv.

- [ ] **Step 13: Chạy toàn bộ và xác nhận baseline**

Run: `python -m pytest -q service/tests/ && python -m pytest -q tests/ 2>&1 | tail -3`
Expected: service toàn PASS; `tests/` vẫn `188 passed, 6 failed`.

- [ ] **Step 14: Kiểm tra lại ranh giới**

Run: `git diff --stat main -- core/ render/ config.py`
Expected: không có dòng nào.

- [ ] **Step 15: Commit**

```bash
git add service/worker/run_worker.py service/deploy/worker-requirements.txt service/tests/worker_ipc_test.py
git commit -m "feat(service): serve plans over a Unix socket, three dependencies deep"
```

---

## Sau Part 1

Kết thúc Part 1, service đã chạy được và test được đầy đủ mà chưa cần một dòng DDS nào: một tiến trình worker nhận request msgpack trên Unix socket và trả về đường bay, cộng một bộ test gác adapter vào thuật toán từng bit.

Part 2 (`docs/superpowers/plans/2026-08-22-dds-service-part2-cpp-node.md`) xây IDL, node C++ Fast DDS, ba tầng thời hạn, và phần triển khai systemd.
