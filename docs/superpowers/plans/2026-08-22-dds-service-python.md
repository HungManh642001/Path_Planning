# VTX Path Planning Service (Python) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Một service Python độc lập, tự khởi động cùng Ubuntu, nhận mission qua DDS và trả về danh sách waypoint, bọc thuật toán path planning hiện có mà không sửa nó.

**Architecture:** Một tiến trình Python. Lớp transport DDS cô lập sau một interface hẹp (stack do Task 1 chọn). Mỗi lần lập kế hoạch chạy trong một tiến trình con tạo bằng `forkserver` — thứ duy nhất cho phép có thời hạn cứng thật trên một vòng lặp search Python, và cũng cách ly hoàn toàn 35 hằng số global của planner. Forkserver được khởi động **trước** khi DDS tồn tại, để không tiến trình con nào từng fork từ một tiến trình có thread DDS.

**Tech Stack:** Python 3.11, `shapely` (đã là phụ thuộc của `core/`), một binding DDS do Task 1 chọn, `pytest`. XML đọc bằng `xml.etree.ElementTree` của thư viện chuẩn.

**Spec:** `docs/superpowers/specs/2026-08-22-dds-path-planning-service-design.md`

## Global Constraints

- **Không sửa `core/`, `render/`, `config.py`.** `git diff --stat main -- core/ render/ config.py` phải rỗng ở mọi commit. Task 2 dựng test cưỡng chế.
- **Planner dùng là `core.kinodynamic_astar_v0`**, không phải `core.kinodynamic_astar`. v0 là bản đang ship.
- **Adapter chỉ gọi, tuyệt đối không sao chép.** Không copy công thức hình học, không copy TypedDict, không hardcode danh sách hằng số `config`.
- **Chỉ hệ toạ độ Oxy phẳng, đơn vị mét.** Không WGS84, không phép chiếu, không `pyproj`. Không có trường `frame`.
- **Góc trên dây: ĐỘ, phương vị thật, thuận chiều kim đồng hồ từ chính bắc**, quy ước `+y` bắc / `+x` đông. Bên trong `core/` là radian ngược chiều kim đồng hồ từ `+x`. Đổi đơn vị tại đúng một module (`angles.py`).
- **`time_budget_s` và `max_iterations` CHƯA được tôn trọng**: service dùng `config.TIME_BUDGET_S` / `config.MAX_ITERATIONS`. Reply mang `applied_time_budget_s` và `stats.max_iterations` là giá trị THẬT đã dùng.
- **`PlanStatus`**: `OK=0, NO_PATH=1, START_LEG_BLOCKED=2, GOAL_LEG_BLOCKED=3, ORACLE_REJECTED=4, INVALID_REQUEST=5, TIMEOUT=6, INTERNAL_ERROR=7, BUSY=8`.
- **QoS**: cả hai topic `RELIABLE` + **`VOLATILE`**; request `KEEP_ALL`, reply `KEEP_LAST(8)`. `TRANSIENT_LOCAL` trên topic request bị CẤM — nó khiến service khởi động lại lập kế hoạch lại một mission đã hết hiệu lực.
- **Baseline test phải giữ nguyên:** `python -m pytest -q tests/` = 188 passed, 6 failed. Sáu ca đỏ có từ trước. Không được thêm ca đỏ nào.
- **Test của service ở `service/tests/`**, chạy `python -m pytest -q service/tests/`. Không đụng `tests/` ở gốc.
- **Nhánh:** `feature/dds-service`.

---

## File Structure

```
service/
  conftest.py                 đặt sys.path về gốc repo                (Task 2)
  vtx_service/
    __init__.py               export plan, PlanRequest, PlanReply
    messages.py               dataclass request/reply + PlanStatus    (Task 2)
    angles.py                 phương vị <-> heading toán học          (Task 3)
    map_file.py               bản đồ nền XML                          (Task 4)
    scenario_builder.py       PlanRequest -> Scenario dict            (Task 5)
    runtime.py                config_hash, planner_version            (Task 6)
    planner.py                plan(PlanRequest) -> PlanReply          (Task 7)
    runner.py                 PlanRunner: forkserver + thời hạn cứng  (Task 9)
    transport.py              interface Transport + cài đặt DDS       (Task 10)
    main.py                   vòng đời service                        (Task 11)
  idl/vtx_path_planning.idl   IDL cho bên gọi                         (Task 10)
  deploy/
    vtx-planner.service                                               (Task 11)
    requirements.txt                                                  (Task 11)
    README.md                                                         (Task 11)
    basemap.example.xml                                               (Task 4)
  tests/
    boundary_test.py (T2)  messages_test.py (T2)  angles_test.py (T3)
    map_file_test.py (T4)  scenario_builder_test.py (T5)
    runtime_test.py (T6)   planner_test.py (T7)   equivalence_test.py (T8)
    runner_test.py (T9)    transport_test.py (T10)
  spike/                      mã dùng một lần của Task 1, KHÔNG ship
```

Thứ tự phụ thuộc: Task 1 (spike) độc lập và chỉ quyết định Task 10. Task 2→8 là một chuỗi thẳng và không cần DDS. Task 9 độc lập với DDS. Task 10-11 khép lại.

---

### Task 1: SPIKE — chọn stack DDS

**Files:**
- Create: `service/spike/cyclone_probe.py`
- Create: `service/spike/fastdds_probe.md`
- Create: `docs/superpowers/specs/2026-08-22-dds-stack-decision.md`

**Interfaces:**
- Consumes: không.
- Produces: một **quyết định** ghi thành văn bản, cộng đoạn mã publish/subscribe chạy được của stack thắng cuộc. Task 10 dựa vào cả hai.

Đây là một **spike**: sản phẩm là câu trả lời, không phải mã để giữ. Mọi thứ trong `service/spike/` là dùng một lần và không được ship.

Đã biết trước khi bắt đầu (đo ngày 2026-08-22, máy này):

| | Fast DDS Python | Cyclone DDS Python |
| --- | --- | --- |
| Trên PyPI | **Không** — đã thử `fastdds`, `fastdds-python`, `eprosima-fastdds` | **Có**, wheel 7,7 MB gói sẵn core |
| Đã chạy thử ở đây | Chưa cài được gì; máy không có Fast DDS, `fastddsgen` hay ROS 2 | **Rồi** — participant, sequence lồng, `@key` 16 byte, double bit-identical, đủ bộ QoS |

Câu hỏi spike phải trả lời là **interop**, không phải "cái nào cài dễ hơn" — điều đó đã biết.

- [ ] **Step 1: Viết probe Cyclone**

Create `service/spike/cyclone_probe.py`:

```python
"""Spike: Cyclone DDS có mang được đúng hình dạng dữ liệu service cần không.

Dùng một lần. Không ship.

Chạy hai vai trong hai terminal:
    python service/spike/cyclone_probe.py listen
    python service/spike/cyclone_probe.py send
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass

from cyclonedds.core import Policy, Qos, ReadCondition, InstanceState, SampleState, ViewState, WaitSet
from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.annotations import key
from cyclonedds.idl.types import array, sequence, uint8
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.topic import Topic
from cyclonedds.util import duration

DOMAIN = 91


@dataclass
class Point2D(IdlStruct, typename="vtx.planning.Point2D"):
    x: float
    y: float


@dataclass
class Polygon(IdlStruct, typename="vtx.planning.Polygon"):
    vertices: sequence[Point2D]


@dataclass
class Probe(IdlStruct, typename="vtx.planning.Probe"):
    request_id: array[uint8, 16]
    key("request_id")
    idl_version: int
    detail: str
    islands: sequence[Polygon]
    length_m: float


REQ_QOS = Qos(
    Policy.Reliability.Reliable(duration(seconds=10)),
    Policy.History.KeepAll,
    Policy.Durability.Volatile,
)


def _endpoint():
    participant = DomainParticipant(DOMAIN)
    topic = Topic(participant, "VtxProbe", Probe, qos=REQ_QOS)
    return participant, topic


def send() -> None:
    participant, topic = _endpoint()
    writer = DataWriter(Publisher(participant), topic, qos=REQ_QOS)
    time.sleep(2.0)  # discovery
    sample = Probe(
        request_id=list(range(16)),
        idl_version=1,
        detail="first W1..W2 l=7421.3 < L0=8000",
        islands=[Polygon(vertices=[Point2D(0.0, 0.0), Point2D(1e5, 0.0), Point2D(5e4, 1e5)])],
        length_m=123456.78901234567,
    )
    writer.write(sample)
    print("đã gửi", sample.length_m)
    time.sleep(2.0)


def listen() -> None:
    participant, topic = _endpoint()
    reader = DataReader(Subscriber(participant), topic, qos=REQ_QOS)
    condition = ReadCondition(
        reader, ViewState.Any | InstanceState.Alive | SampleState.NotRead
    )
    waitset = WaitSet(participant)
    waitset.attach(condition)
    if waitset.wait(duration(seconds=30)) == 0:
        print("KHÔNG nhận được gì trong 30 s")
        return
    for sample in reader.take(N=10, condition=condition):
        print("request_id  :", list(sample.request_id) == list(range(16)))
        print("detail      :", repr(sample.detail))
        print("đỉnh đảo    :", len(sample.islands[0].vertices))
        print("double khớp :", sample.length_m == 123456.78901234567)


if __name__ == "__main__":
    {"send": send, "listen": listen}[sys.argv[1]]()
```

- [ ] **Step 2: Chạy probe Cyclone**

```bash
pip install cyclonedds
python service/spike/cyclone_probe.py listen &   # terminal 1
python service/spike/cyclone_probe.py send       # terminal 2
```
Expected: cả bốn dòng đều `True` / đúng giá trị.

- [ ] **Step 3: Dựng Fast DDS Python binding và đo công sức thật**

Làm theo hướng dẫn Fast-DDS-python của eProsima: Fast-CDR, Fast-DDS,
Fast-DDS-python (SWIG), `fastddsgen` (cần JRE). **Bấm giờ**, và ghi lại từng
lệnh phải chạy cùng mọi thứ phải cài thêm.

Create `service/spike/fastdds_probe.md` ghi lại: các lệnh, thời gian, dung lượng
đĩa, và những chỗ vướng. Nếu không dựng được trong **hai giờ**, DỪNG và ghi
"không dựng được trong 2 giờ" cùng lý do cụ thể — đó là một kết quả đo, không
phải một thất bại.

- [ ] **Step 4: Thử interop với hệ thống thật**

Đây là **câu hỏi trung tâm của spike**. Chạy `cyclone_probe.py listen` trên
đúng `domain_id` mà hệ thống Fast DDS của bạn đang dùng, với một topic và một
kiểu do phía họ publish, rồi xem có nhận được không.

Ba khả năng, ghi lại chính xác cái nào xảy ra:

- Nhận được và dữ liệu đúng ⇒ Cyclone khả thi.
- Discovery khớp nhưng không có mẫu tin ⇒ nhiều khả năng lệch tên kiểu hoặc
  XTypes; ghi lại log discovery của cả hai phía.
- Không khớp gì ⇒ ghi cấu hình discovery của cả hai phía trước khi kết luận.

Nếu không tiếp cận được hệ thống thật lúc này, ghi rõ điều đó và đánh dấu quyết
định là **tạm thời** — Task 10 vẫn làm được, chỉ là rủi ro chưa đóng.

- [ ] **Step 5: Viết quyết định thành văn bản**

Create `docs/superpowers/specs/2026-08-22-dds-stack-decision.md` gồm: stack đã
chọn, số đo của cả hai phương án (thời gian cài, dung lượng, kết quả interop),
điều gì sẽ khiến phải xét lại quyết định này, và các lệnh cài đặt của stack
thắng cuộc.

- [ ] **Step 6: Commit**

```bash
git add service/spike/ docs/superpowers/specs/2026-08-22-dds-stack-decision.md
git commit -m "spike: measure both DDS stacks and record the choice"
```

---

### Task 2: Bộ khung, kiểu dữ liệu, test ranh giới

**Files:**
- Create: `service/__init__.py`, `service/conftest.py`, `service/vtx_service/__init__.py`, `service/vtx_service/messages.py`
- Test: `service/tests/boundary_test.py`, `service/tests/messages_test.py`

**Interfaces:**
- Consumes: không.
- Produces: `PlanStatus`, `Point`, `Circle`, `VehicleLimits`, `SearchBudget`, `PlanRequest`, `Waypoint`, `SearchStats`, `PlanReply`, hằng `IDL_VERSION`. Mọi task sau dùng.

- [ ] **Step 1: Viết test ranh giới**

Create `service/tests/boundary_test.py`:

```python
"""Ràng buộc số 1: service không được sửa thuật toán.

Cơ chế cưỡng chế, không phải lời nhắc. So với nhánh gốc `main` nên nó đỏ ngay cả
khi thay đổi đã được commit.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTECTED = ["core/", "render/", "config.py"]


def test_service_work_does_not_touch_the_algorithm() -> None:
    proc = subprocess.run(
        ["git", "diff", "--stat", "main", "--", *PROTECTED],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    assert proc.stdout.strip() == "", (
        "Nhánh service đã sửa thuật toán, điều bị cấm bởi ràng buộc 1 của spec.\n"
        f"Thay đổi:\n{proc.stdout}"
    )


def test_service_tree_does_not_copy_the_algorithm() -> None:
    service = REPO_ROOT / "service"
    assert service.is_dir()
    assert not (service / "core").exists(), "core/ không được sao chép vào service/"
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/boundary_test.py -v`
Expected: FAIL — `file or directory not found` (thư mục chưa có).

- [ ] **Step 3: Dựng bộ khung**

Create `service/__init__.py` và `service/vtx_service/__init__.py` (tạm để rỗng).

Create `service/conftest.py`:

```python
"""Đưa gốc repo và thư mục service lên sys.path.

Cùng cơ chế import mà service dùng lúc chạy thật, nên test chạy đúng cấu hình
của production.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SERVICE_ROOT = REPO_ROOT / "service"

for path in (REPO_ROOT, SERVICE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
```

- [ ] **Step 4: Chạy lại test ranh giới**

Run: `python -m pytest service/tests/boundary_test.py -v`
Expected: PASS, 2 passed.

- [ ] **Step 5: Viết test kiểu dữ liệu**

Create `service/tests/messages_test.py`:

```python
from __future__ import annotations

import dataclasses

import pytest

from vtx_service.messages import (
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


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x00" * 16,
        idl_version=IDL_VERSION,
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
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def test_request_is_frozen() -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        _request().start = (1.0, 1.0)  # type: ignore[misc]


def test_request_id_must_be_16_bytes() -> None:
    with pytest.raises(ValueError, match="16 byte"):
        _request(request_id=b"\x00" * 8)


def test_status_values_match_the_wire_contract() -> None:
    # Các số này khớp enum trong IDL. Đổi ở đây mà không đổi IDL là một thay đổi
    # phá vỡ hợp đồng đi qua không tiếng động.
    assert [int(member) for member in PlanStatus] == list(range(9))
    assert PlanStatus.OK == 0
    assert PlanStatus.ORACLE_REJECTED == 4
    assert PlanStatus.BUSY == 8


def test_there_is_no_frame_field() -> None:
    """Chỉ có một hệ toạ độ. Thêm WGS84 sau là một lần tăng idl_version."""
    assert "frame" not in {f.name for f in dataclasses.fields(PlanRequest)}


def test_reply_reports_the_budget_it_actually_used() -> None:
    """time_budget_s trên dây CHƯA được tôn trọng; reply phải nói thật."""
    fields = {f.name for f in dataclasses.fields(PlanReply)}
    assert "applied_time_budget_s" in fields


def test_circle_rejects_a_non_positive_radius() -> None:
    with pytest.raises(ValueError, match="radius"):
        Circle(center=(0.0, 0.0), radius_m=0.0)


def test_limits_reject_a_negative_margin() -> None:
    with pytest.raises(ValueError, match="safe_margin_m"):
        VehicleLimits(8000.0, 8000.0, 15000.0, -1.0, 90.0)


def test_reply_round_trips_through_dataclasses_replace() -> None:
    reply = PlanReply(
        request_id=b"\x00" * 16,
        idl_version=IDL_VERSION,
        status=PlanStatus.OK,
        detail="",
        waypoints=(Waypoint((0.0, 0.0), 12.5),),
        path_length_m=1.0,
        plan_wall_time_s=0.5,
        applied_time_budget_s=15.0,
        stats=SearchStats(3, 50000, 7, False, False),
        planner_version="abc1234",
        config_hash="0123456789abcdef",
    )
    assert dataclasses.replace(reply, detail="x").detail == "x"
```

- [ ] **Step 6: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/messages_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_service.messages'`.

- [ ] **Step 7: Viết `messages.py`**

Create `service/vtx_service/messages.py`:

```python
"""Kiểu dữ liệu request/reply, ánh xạ 1-1 sang IDL.

Đây là hợp đồng đối ngoại, tách hẳn khỏi hai dict shape nội bộ của pipeline
(`core.types.Scenario` / `PreprocessedScenario`). Giữ chúng tách nhau là cố ý:
hợp đồng đối ngoại đổi theo phiên bản IDL, dict nội bộ đổi theo thuật toán.

Đơn vị: khoảng cách MÉT trên mặt phẳng Oxy. Góc là ĐỘ và là phương vị thật,
thuận chiều kim đồng hồ từ chính bắc; xem `vtx_service.angles`.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

Point = tuple[float, float]
"""Vị trí phẳng ``(x, y)`` mét. ``+y`` là bắc, ``+x`` là đông."""

IDL_VERSION = 1
"""Tăng khi bố cục struct đổi. Service từ chối request không khớp."""


class PlanStatus(IntEnum):
    """Kết cục của một lần lập kế hoạch. Giá trị số khớp enum trong IDL."""

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

    Ánh xạ 1-1 sang tham số của ``core.preprocessing.prepare_scenario``. Mọi
    hằng số khác của planner là global và cố định lúc triển khai.
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
    """Ngân sách search do client đề nghị.

    CHƯA được tôn trọng: service dùng ``config.TIME_BUDGET_S`` và
    ``config.MAX_ITERATIONS``. Trường có mặt để sau này thuật toán nhận chúng
    như tham số thật mà không phải tăng ``IDL_VERSION``. Reply báo cáo ngược giá
    trị đã dùng thật qua ``applied_time_budget_s`` và ``stats.max_iterations``.
    """

    time_budget_s: float
    max_iterations: int


@dataclass(frozen=True)
class PlanRequest:
    """Một mission cần lập kế hoạch."""

    request_id: bytes
    idl_version: int
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


@dataclass(frozen=True)
class Waypoint:
    """Một điểm trên đường bay trả về."""

    position: Point
    heading_deg: float


@dataclass(frozen=True)
class SearchStats:
    """Bộ đếm mô tả một lần chạy search.

    ``budget_bound`` là trường hạng nhất chứ không phải chi tiết ẩn: planner cắt
    theo đồng hồ, nên cùng một request trên máy tải nặng có thể ra đường bay
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
    applied_time_budget_s: float
    stats: SearchStats
    planner_version: str
    config_hash: str

    @property
    def ok(self) -> bool:
        return self.status is PlanStatus.OK
```

Create `service/vtx_service/__init__.py`:

```python
"""Service path planning: bọc thuật toán thành một API thuần Python.

Không module nào trong package này ngoài `transport` được phép import DDS.
Xem docs/superpowers/specs/2026-08-22-dds-path-planning-service-design.md
"""

from __future__ import annotations

from vtx_service.messages import (
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
Expected: PASS, 10 passed.

- [ ] **Step 9: Xác nhận baseline gốc không đổi**

Run: `python -m pytest -q tests/ 2>&1 | tail -3`
Expected: `188 passed, 6 failed`. Nếu khác, DỪNG và báo cáo — không sửa test cho xanh.

- [ ] **Step 10: Commit**

```bash
git add service/
git commit -m "feat(service): scaffold vtx_service with the wire types and a boundary guard"
```

---

### Task 3: Quy ước hướng

**Files:**
- Create: `service/vtx_service/angles.py`
- Test: `service/tests/angles_test.py`

**Interfaces:**
- Consumes: không.
- Produces: `bearing_deg_to_math_rad(bearing_deg: float) -> float`, `math_rad_to_bearing_deg(theta_rad: float) -> float`. Task 5, 7, 8 dùng.

- [ ] **Step 1: Viết test**

Create `service/tests/angles_test.py`:

```python
"""Quy ước hướng là chỗ dễ sinh lỗi nhất của toàn service.

Một đường bay lệch 90 độ hoặc bị gương vẫn hợp lệ về hình học, nên mọi test hình
học khác đều bỏ lọt loại lỗi này. Nó phải bị chặn ở đây.
"""

from __future__ import annotations

import math

import pytest

from vtx_service.angles import bearing_deg_to_math_rad, math_rad_to_bearing_deg

# phương vị (độ, thuận kim đồng hồ từ bắc) -> heading toán học (rad, ngược kim
# đồng hồ từ +x). Quy ước: +y bắc, +x đông.
KNOWN = [
    (0.0, math.pi / 2),      # bắc  -> +y
    (90.0, 0.0),             # đông -> +x
    (180.0, -math.pi / 2),   # nam  -> -y
    (270.0, math.pi),        # tây  -> -x
    (45.0, math.pi / 4),     # đông bắc
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
    # So HƯỚNG, không so cách biểu diễn: -pi và +pi là cùng một hướng.
    a = bearing_deg_to_math_rad(-90.0)
    b = bearing_deg_to_math_rad(270.0)
    assert math.isclose(math.cos(a), math.cos(b), abs_tol=1e-12)
    assert math.isclose(math.sin(a), math.sin(b), abs_tol=1e-12)


def test_bearing_increases_clockwise_not_counterclockwise() -> None:
    """Phép thử phân biệt hai quy ước. Ai lật dấu thì test này đỏ."""
    north = bearing_deg_to_math_rad(0.0)
    slightly_east_of_north = bearing_deg_to_math_rad(10.0)
    # Quay thuận kim đồng hồ trên mặt đất = GIẢM góc toán học.
    assert slightly_east_of_north < north


def test_range_is_what_the_docstring_promises() -> None:
    """Khẳng định này đã bắt lỗi trong bản nháp đầu của module.

    Cách viết bằng số học modulo trả về 4,712 rad ở phương vị 180 độ - đúng
    HƯỚNG, nên mọi test cos/sin ở trên vẫn xanh, nhưng ngoài dải docstring hứa.
    Sai lệch kiểu đó chỉ lộ ra ở downstream, nơi có ai đó so hai góc trực tiếp.
    """
    for degrees in range(0, 360, 7):
        assert -math.pi <= bearing_deg_to_math_rad(float(degrees)) <= math.pi
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/angles_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_service.angles'`.

- [ ] **Step 3: Viết `angles.py`**

Create `service/vtx_service/angles.py`:

```python
"""Đổi giữa phương vị đối ngoại và quy ước góc nội bộ của thuật toán.

Trên dây: ĐỘ, phương vị thật, thuận chiều kim đồng hồ từ chính bắc.
Trong `core/`: RADIAN, ngược chiều kim đồng hồ từ trục +x.
Mặt phẳng quy ước +y là bắc, +x là đông.

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
git add service/vtx_service/angles.py service/tests/angles_test.py
git commit -m "feat(service): convert true bearings to the planner's angle convention"
```

---

### Task 4: Bản đồ nền XML

**Files:**
- Create: `service/vtx_service/map_file.py`
- Create: `service/deploy/basemap.example.xml`
- Test: `service/tests/map_file_test.py`

**Interfaces:**
- Consumes: `Circle`, `PlanRequest`, `Point` (Task 2).
- Produces: `PreloadedMap` với `PreloadedMap.load(path: Path) -> PreloadedMap`, thuộc tính `safezones`, `islands`, `dynamic_obstacles`, và `merged_into(request: PlanRequest) -> PlanRequest`. Task 7 và 11 dùng.

- [ ] **Step 1: Viết test**

Create `service/tests/map_file_test.py`:

```python
"""Bản đồ nền XML: nạp một lần lúc khởi động, gộp vào request khi được yêu cầu.

Mặc định triển khai là KHÔNG nạp bản đồ nào - request tự chứa thì replay được và
chẩn đoán được, còn state ẩn trong service thì không.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from vtx_service.map_file import PreloadedMap
from vtx_service.messages import Circle, PlanRequest, SearchBudget, VehicleLimits

MAP_XML = """<vtx-map version="1">
  <safezones>
    <polygon>
      <point x="0" y="0"/>
      <point x="500000" y="0"/>
      <point x="500000" y="500000"/>
      <point x="0" y="500000"/>
    </polygon>
  </safezones>
  <obstacles>
    <polygon>
      <point x="150000" y="120000"/>
      <point x="200000" y="120000"/>
      <point x="175000" y="200000"/>
    </polygon>
    <circle cx="220000" cy="180000" r="15000"/>
  </obstacles>
</vtx-map>
"""


def _write(tmp_path: Path, text: str = MAP_XML) -> Path:
    path = tmp_path / "basemap.xml"
    path.write_text(text, encoding="utf-8")
    return path


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x05" * 16,
        idl_version=1,
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


def test_load_reads_all_three_kinds(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(_write(tmp_path))
    assert len(loaded.safezones) == 1
    assert len(loaded.safezones[0]) == 4
    assert loaded.islands == (((150000.0, 120000.0), (200000.0, 120000.0), (175000.0, 200000.0)),)
    assert loaded.dynamic_obstacles == (Circle(center=(220000.0, 180000.0), radius_m=15000.0),)


def test_a_repeated_closing_vertex_is_trimmed(tmp_path: Path) -> None:
    """`core/` giả định vành MỞ; một đỉnh lặp tạo ra cạnh dài 0."""
    closed = MAP_XML.replace(
        '<point x="175000" y="200000"/>',
        '<point x="175000" y="200000"/>\n      <point x="150000" y="120000"/>',
    )
    loaded = PreloadedMap.load(_write(tmp_path, closed))
    assert len(loaded.islands[0]) == 3


def test_merge_appends_and_does_not_replace(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(_write(tmp_path))
    request = _request(
        use_preloaded_map=True,
        dynamic_obstacles=(Circle(center=(100000.0, 100000.0), radius_m=5000.0),),
        safezones=(((0.0, 0.0), (10.0, 0.0), (10.0, 10.0)),),
    )
    merged = loaded.merged_into(request)
    assert len(merged.dynamic_obstacles) == 2
    assert len(merged.safezones) == 2
    # Của request đứng trước, để đọc log dễ đối chiếu.
    assert merged.dynamic_obstacles[0].radius_m == 5000.0


def test_flag_off_returns_the_very_same_object(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(_write(tmp_path))
    request = _request(use_preloaded_map=False)
    assert loaded.merged_into(request) is request


def test_a_wrong_version_is_an_error_not_a_warning(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="version"):
        PreloadedMap.load(_write(tmp_path, MAP_XML.replace('version="1"', 'version="9"')))


def test_a_wrong_root_tag_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="vtx-map"):
        PreloadedMap.load(_write(tmp_path, MAP_XML.replace("vtx-map", "other-map")))


def test_a_polygon_with_two_points_is_rejected(tmp_path: Path) -> None:
    thin = MAP_XML.replace('      <point x="175000" y="200000"/>\n', "")
    with pytest.raises(ValueError, match="3 đỉnh"):
        PreloadedMap.load(_write(tmp_path, thin))


def test_a_non_positive_radius_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="radius"):
        PreloadedMap.load(_write(tmp_path, MAP_XML.replace('r="15000"', 'r="0"')))


def test_empty_sections_are_allowed(tmp_path: Path) -> None:
    loaded = PreloadedMap.load(
        _write(tmp_path, '<vtx-map version="1"><safezones/><obstacles/></vtx-map>')
    )
    assert loaded.safezones == () and loaded.islands == () and loaded.dynamic_obstacles == ()


def test_the_shipped_example_file_parses() -> None:
    example = Path(__file__).resolve().parents[1] / "deploy" / "basemap.example.xml"
    loaded = PreloadedMap.load(example)
    assert loaded.safezones or loaded.islands or loaded.dynamic_obstacles
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/map_file_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_service.map_file'`.

- [ ] **Step 3: Viết `map_file.py`**

Create `service/vtx_service/map_file.py`:

```python
"""Bản đồ nền tĩnh dạng XML, nạp một lần lúc worker khởi động.

Mặc định triển khai là không có bản đồ nào: một request tự chứa thì replay được
và chẩn đoán được, còn state ẩn trong service thì không. Bản đồ nền tồn tại cho
trường hợp bản đồ quá lớn để gửi kèm mỗi request.

Gộp là NỐI THÊM, không thay thế. Với ``safezones`` thì planner lấy HỢP của
chúng (``unary_union`` trong ``kinodynamic_astar_v0``), nên thêm một safezone là
NỚI RỘNG vùng bay chứ không thu hẹp. Trực giác thường ngược lại, nên điều này
cũng nằm trong tài liệu vận hành chứ không chỉ ở đây.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree

from vtx_service.messages import Circle, PlanRequest, Point

MAP_VERSION = "1"
_CLOSING_TOL_M = 1e-9


@dataclass(frozen=True)
class PreloadedMap:
    """Vùng bay và chướng ngại vật nền, toạ độ mét trong hệ Oxy."""

    safezones: tuple[tuple[Point, ...], ...]
    islands: tuple[tuple[Point, ...], ...]
    dynamic_obstacles: tuple[Circle, ...]

    @classmethod
    def load(cls, path: Path) -> PreloadedMap:
        """Đọc một bản đồ nền từ file XML.

        Args:
            path: File XML gốc ``<vtx-map version="1">``.

        Returns:
            Bản đồ đã nạp.

        Raises:
            ValueError: Khi tag gốc, ``version``, hoặc hình học không hợp lệ.
        """
        root = ElementTree.parse(path).getroot()
        if root.tag != "vtx-map":
            raise ValueError(f"tag gốc phải là vtx-map, nhận {root.tag!r}")
        version = root.get("version")
        if version != MAP_VERSION:
            raise ValueError(f"version bản đồ {version!r} != {MAP_VERSION!r}")

        return cls(
            safezones=tuple(_polygons(root.find("safezones"))),
            islands=tuple(_polygons(root.find("obstacles"))),
            dynamic_obstacles=tuple(_circles(root.find("obstacles"))),
        )

    def merged_into(self, request: PlanRequest) -> PlanRequest:
        """Nối bản đồ nền vào một request, nếu request yêu cầu.

        Args:
            request: Request gốc.

        Returns:
            Request đã gộp, hoặc CHÍNH ``request`` khi cờ tắt - trả về cùng đối
            tượng để chỗ gọi phân biệt được "không gộp" với "gộp rỗng".
        """
        if not request.use_preloaded_map:
            return request
        return dataclasses.replace(
            request,
            safezones=request.safezones + self.safezones,
            islands=request.islands + self.islands,
            dynamic_obstacles=request.dynamic_obstacles + self.dynamic_obstacles,
        )


def _polygons(section: ElementTree.Element | None) -> list[tuple[Point, ...]]:
    if section is None:
        return []
    return [_ring(node) for node in section.findall("polygon")]


def _ring(node: ElementTree.Element) -> tuple[Point, ...]:
    points: list[Point] = [
        (float(p.get("x", "nan")), float(p.get("y", "nan"))) for p in node.findall("point")
    ]
    # Vành MỞ: `core/` giả định không có đỉnh đóng lặp lại, và một đỉnh trùng
    # lặp tạo ra cạnh dài 0 mà oracle sẽ từ chối.
    if len(points) >= 2 and math.dist(points[0], points[-1]) < _CLOSING_TOL_M:
        points.pop()
    if len(points) < 3:
        raise ValueError(f"đa giác cần ít nhất 3 đỉnh, nhận {len(points)}")
    return tuple(points)


def _circles(section: ElementTree.Element | None) -> list[Circle]:
    if section is None:
        return []
    circles: list[Circle] = []
    for node in section.findall("circle"):
        radius = float(node.get("r", "nan"))
        if not radius > 0.0:
            raise ValueError(f"radius phải dương, nhận {radius}")
        circles.append(
            Circle(
                center=(float(node.get("cx", "nan")), float(node.get("cy", "nan"))),
                radius_m=radius,
            )
        )
    return circles
```

- [ ] **Step 4: Viết file mẫu**

Create `service/deploy/basemap.example.xml`:

```xml
<?xml version="1.0" encoding="utf-8"?>
<!-- Bản đồ nền mẫu. Toạ độ MÉT trong hệ Oxy phẳng; +y là bắc, +x là đông.

     Đa giác là vành MỞ: không lặp lại đỉnh đầu ở cuối.

     LƯU Ý về ngữ nghĩa: safezone của bản đồ nền được NỐI THÊM vào safezone của
     request, và planner lấy HỢP của chúng. Thêm một safezone là NỚI RỘNG vùng
     bay, không phải thu hẹp. -->
<vtx-map version="1">
  <safezones>
    <polygon>
      <point x="0" y="0"/>
      <point x="500000" y="0"/>
      <point x="500000" y="500000"/>
      <point x="0" y="500000"/>
    </polygon>
  </safezones>
  <obstacles>
    <polygon>
      <point x="150000" y="120000"/>
      <point x="200000" y="120000"/>
      <point x="175000" y="200000"/>
    </polygon>
    <circle cx="220000" cy="180000" r="15000"/>
  </obstacles>
</vtx-map>
```

- [ ] **Step 5: Chạy test**

Run: `python -m pytest service/tests/map_file_test.py -v`
Expected: PASS, 10 passed.

- [ ] **Step 6: Commit**

```bash
git add service/vtx_service/map_file.py service/deploy/basemap.example.xml service/tests/map_file_test.py
git commit -m "feat(service): load the static base map from XML, open rings enforced"
```

---

### Task 5: Dựng `Scenario` dict, và test hợp đồng khoá

**Files:**
- Create: `service/vtx_service/scenario_builder.py`
- Test: `service/tests/scenario_builder_test.py`

**Interfaces:**
- Consumes: `PlanRequest` (Task 2), `angles` (Task 3).
- Produces: `build_scenario(request: PlanRequest) -> dict[str, Any]`. Task 7 và 8 dùng.

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

from vtx_service.messages import Circle, PlanRequest, SearchBudget, VehicleLimits
from vtx_service.scenario_builder import build_scenario

LIMITS = VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0)


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x01" * 16,
        idl_version=1,
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
        budget=SearchBudget(15.0, 50000),
    )
    base.update(overrides)
    return PlanRequest(**base)  # type: ignore[arg-type]


def test_builder_fills_every_key_the_scenario_type_declares() -> None:
    assert set(build_scenario(_request())) == set(get_type_hints(Scenario))


def test_coordinates_pass_through_bit_identically() -> None:
    """Không có phép chiếu nào; test tương đương dựa vào điều này."""
    request = _request()
    built = build_scenario(request)
    assert built["start"] == request.start
    assert built["goal"] == request.goal
    assert built["islands"][0][0] == request.islands[0][0]


def test_headings_are_converted_to_the_planner_convention() -> None:
    # phương vị 90 = đông = +x = 0 rad
    assert math.isclose(build_scenario(_request())["start_heading"], 0.0, abs_tol=1e-12)


def test_free_goal_becomes_none_not_a_sentinel_number() -> None:
    assert build_scenario(_request(goal_heading_free=True))["goal_heading"] is None


def test_map_bounds_is_deliberately_none() -> None:
    """Spec mục 4.2: map_bounds neo tại gốc toạ độ; safezones mạnh hơn."""
    assert build_scenario(_request())["map_bounds"] is None


def test_empty_safezones_becomes_none_so_the_planner_stays_permissive() -> None:
    assert build_scenario(_request(safezones=()))["safezones"] is None


def test_safezones_are_passed_through_when_present() -> None:
    zone = ((0.0, 0.0), (400000.0, 0.0), (400000.0, 400000.0))
    assert build_scenario(_request(safezones=(zone,)))["safezones"] == [list(zone)]


def test_obstacles_is_the_tagged_union_the_pipeline_consumes() -> None:
    built = build_scenario(_request())
    assert sorted(o["type"] for o in built["obstacles"]) == ["circle", "polygon"]
    circle = next(o for o in built["obstacles"] if o["type"] == "circle")
    assert circle["center"] == (200000.0, 150000.0)
    assert circle["radius"] == 12000.0


def test_the_built_scenario_actually_runs_through_the_pipeline() -> None:
    import core.kinodynamic_astar_v0 as astar
    import core.preprocessing as prep

    preprocessed = prep.prepare_scenario(
        build_scenario(_request()),
        turn_radius=LIMITS.turn_radius_m,
        l0=LIMITS.l0_m,
        dss=LIMITS.dss_m,
        safe_margin=LIMITS.safe_margin_m,
        alpha_max_rad=math.radians(LIMITS.alpha_max_deg),
    )
    assert astar.plan_trajectory(preprocessed)["success"] is True
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/scenario_builder_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_service.scenario_builder'`.

- [ ] **Step 3: Viết `scenario_builder.py`**

Create `service/vtx_service/scenario_builder.py`:

```python
"""Dịch một ``PlanRequest`` thành đúng dict ``Scenario`` mà pipeline tiêu thụ.

Đây là toàn bộ phần "dịch" của adapter. Nó không tính hình học và không biết gì
về search: mọi khoá lấy thẳng từ ``core.types.Scenario``, để một khoá mới bên đó
làm test hợp đồng đỏ chứ không làm production nổ.

Toạ độ đi qua NGUYÊN VẸN - chỉ có một hệ toạ độ, không có phép chiếu nào. Chỉ
góc được đổi quy ước.
"""

from __future__ import annotations

from typing import Any

from vtx_service.angles import bearing_deg_to_math_rad
from vtx_service.messages import PlanRequest


def build_scenario(request: PlanRequest) -> dict[str, Any]:
    """Dựng dict ``Scenario`` từ một request.

    Args:
        request: Mission cần lập kế hoạch, toạ độ mét trong hệ Oxy.

    Returns:
        Một dict mang đúng tập khoá của ``core.types.Scenario``, sẵn sàng cho
        ``core.preprocessing.prepare_scenario``.
    """
    islands = [list(polygon) for polygon in request.islands]
    circles = [(circle.center, circle.radius_m) for circle in request.dynamic_obstacles]
    safezones = [list(zone) for zone in request.safezones]

    obstacles: list[dict[str, Any]] = [
        {"type": "polygon", "polygon": polygon} for polygon in islands
    ]
    obstacles.extend(
        {"type": "circle", "center": center, "radius": radius} for center, radius in circles
    )

    goal_heading = (
        None
        if request.goal_heading_free
        else bearing_deg_to_math_rad(request.goal_heading_deg)
    )

    return {
        "start": request.start,
        "start_heading": bearing_deg_to_math_rad(request.start_heading_deg),
        "goal": request.goal,
        "goal_heading": goal_heading,
        # Cố tình None - xem mục 4.2 của spec. safezones là cơ chế đúng.
        "map_bounds": None,
        "safezones": safezones or None,
        "islands": islands,
        "dynamic_obstacles": circles,
        "obstacles": obstacles,
    }
```

- [ ] **Step 4: Chạy test**

Run: `python -m pytest service/tests/scenario_builder_test.py -v`
Expected: PASS, 9 passed.

- [ ] **Step 5: Commit**

```bash
git add service/vtx_service/scenario_builder.py service/tests/scenario_builder_test.py
git commit -m "feat(service): build the Scenario dict, with a key-contract guard"
```

---

### Task 6: `config_hash` và `planner_version`

**Files:**
- Create: `service/vtx_service/runtime.py`
- Test: `service/tests/runtime_test.py`

**Interfaces:**
- Consumes: không (đọc `config` và `core.kinodynamic_astar_v0`).
- Produces: `planner_config_snapshot() -> dict[str, object]`, `config_hash() -> str`, `planner_version() -> str`, `effective_time_budget_s() -> float`, `effective_max_iterations() -> int`. Task 7 dùng.

- [ ] **Step 1: Viết test**

Create `service/tests/runtime_test.py`:

```python
"""Siêu dữ liệu phiên bản, và giá trị ngân sách service THỰC SỰ dùng.

Client gửi `time_budget_s` nhưng service chưa tôn trọng nó. Nhận một trường rồi
lặng lẽ bỏ qua là cách chắc chắn để client tin vào một điều không đúng, nên
service báo cáo ngược giá trị thật.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import config
import pytest

from vtx_service.runtime import (
    config_hash,
    effective_max_iterations,
    effective_time_budget_s,
    planner_config_snapshot,
    planner_version,
)

# Suy ra ĐỘC LẬP, không đọc runtime._REPO_ROOT. Nếu lấy từ module đang test thì
# test sẽ trôi theo lỗi thay vì bắt được lỗi.
_TRUE_REPO_ROOT = Path(__file__).resolve().parents[2]


def test_snapshot_is_discovered_not_hardcoded() -> None:
    snapshot = planner_config_snapshot()
    # Vài knob chắc chắn v0 đọc. KHÔNG khẳng định tổng số: con số đó phải được
    # phép đổi khi thuật toán đổi - đó chính là mục đích của cơ chế này.
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
    assert isinstance(planner_version(), str) and planner_version()


def test_version_matches_git_describe_inside_a_checkout() -> None:
    """Ghim `planner_version()` vào `git describe` THẬT, không chỉ "khác rỗng".

    `test_version_is_a_non_empty_string` không phân biệt được gốc repo đúng với
    gốc sai: chính giá trị dự phòng "unknown" cũng là một chuỗi khác rỗng. Đã đo
    trong lúc thực thi kế hoạch này — `_REPO_ROOT` sai một mức khiến
    `planner_version()` im lặng trả "unknown" mãi mãi, mà bộ test vẫn xanh.

    Test này tính giá trị mong đợi từ một gốc repo suy ra ĐỘC LẬP, nên nó đỏ
    khi `_REPO_ROOT` bị trỏ tới chỗ không có `.git`.
    """
    if not (_TRUE_REPO_ROOT / ".git").exists():
        pytest.skip("không chạy trong một checkout git")
    expected = subprocess.run(
        ["git", "describe", "--always", "--dirty"],
        cwd=_TRUE_REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    assert planner_version() == expected


def test_effective_budget_comes_from_config_not_from_the_request() -> None:
    assert effective_time_budget_s() == float(config.TIME_BUDGET_S or 0.0)
    assert effective_max_iterations() == config.MAX_ITERATIONS


def test_effective_budget_is_a_float_even_when_config_says_none() -> None:
    original = config.TIME_BUDGET_S
    try:
        config.TIME_BUDGET_S = None
        assert effective_time_budget_s() == 0.0
    finally:
        config.TIME_BUDGET_S = original
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/runtime_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_service.runtime'`.

- [ ] **Step 3: Viết `runtime.py`**

Create `service/vtx_service/runtime.py`:

```python
"""Siêu dữ liệu phiên bản và cấu hình đi kèm mỗi reply.

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
from pathlib import Path

import config
import core.kinodynamic_astar_v0 as astar

_CONFIG_REF = re.compile(r"\bconfig\.([A-Z][A-Z0-9_]*)\b")
_REPO_ROOT = Path(__file__).resolve().parents[2]


def planner_config_snapshot() -> dict[str, object]:
    """Liệt kê các hằng số ``config`` mà planner đang ship thực sự đọc.

    Danh sách được PHÁT HIỆN bằng cách quét mã nguồn planner chứ không hardcode,
    nên một knob mới xuất hiện trong reply mà không ai phải nhớ cập nhật chỗ này.

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


def effective_time_budget_s() -> float:
    """Ngân sách thời gian service THỰC SỰ dùng.

    Lấy từ ``config``, không phải từ request: xem mục 4.3 của spec. Reply báo
    cáo ngược giá trị này để client không tưởng rằng đề nghị của mình được nhận.

    Returns:
        Ngân sách tính bằng giây; ``0.0`` nghĩa là không giới hạn.
    """
    return float(config.TIME_BUDGET_S or 0.0)


def effective_max_iterations() -> int:
    """Trần số vòng lặp service THỰC SỰ dùng. Cùng lý do như trên."""
    return int(config.MAX_ITERATIONS)
```

- [ ] **Step 4: Chạy test**

Run: `python -m pytest service/tests/runtime_test.py -v`
Expected: PASS, 7 passed.

- [ ] **Step 5: Commit**

```bash
git add service/vtx_service/runtime.py service/tests/runtime_test.py
git commit -m "feat(service): discovered config hash, version stamp, honest budget reporting"
```

---

### Task 7: `plan()` và ánh xạ trạng thái

**Files:**
- Create: `service/vtx_service/planner.py`
- Modify: `service/vtx_service/__init__.py`
- Test: `service/tests/planner_test.py`

**Interfaces:**
- Consumes: Task 2-6.
- Produces: `plan(request: PlanRequest, preloaded: PreloadedMap | None = None) -> PlanReply`. Task 8, 9, 10 dùng.

- [ ] **Step 1: Viết test**

Create `service/tests/planner_test.py`:

```python
from __future__ import annotations

import math
from pathlib import Path

from vtx_service import plan
from vtx_service.map_file import PreloadedMap
from vtx_service.messages import (
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
    request = _request(request_id=b"\x07" * 16)
    reply = plan(request)
    assert reply.request_id == request.request_id
    assert reply.idl_version == request.idl_version


def test_path_starts_at_takeoff_and_ends_at_the_target() -> None:
    request = _request()
    reply = plan(request)
    assert reply.waypoints[0].position == request.start
    assert reply.waypoints[-1].position == request.goal


def test_first_waypoint_keeps_the_requested_takeoff_bearing() -> None:
    reply = plan(_request(start_heading_deg=45.0))
    assert math.isclose(reply.waypoints[0].heading_deg, 45.0, abs_tol=1e-6)


def test_reply_carries_version_and_config_identity() -> None:
    reply = plan(_request())
    assert reply.planner_version
    assert len(reply.config_hash) == 16


def test_reply_reports_the_budget_it_used_not_the_one_requested() -> None:
    """Mục 4.3: đề nghị của client CHƯA được tôn trọng, và reply nói thật."""
    import config

    reply = plan(_request(budget=SearchBudget(time_budget_s=0.001, max_iterations=7)))
    assert reply.applied_time_budget_s == float(config.TIME_BUDGET_S or 0.0)
    assert reply.stats.max_iterations == config.MAX_ITERATIONS
    assert reply.status is PlanStatus.OK  # ngân sách 1 ms KHÔNG được áp dụng


def test_a_goal_buried_in_an_obstacle_fails_honestly() -> None:
    reply = plan(
        _request(dynamic_obstacles=(Circle(center=(300000.0, 250000.0), radius_m=40000.0),))
    )
    assert reply.status is not PlanStatus.OK
    assert reply.detail != ""


def test_a_wrong_idl_version_is_refused_without_searching() -> None:
    reply = plan(_request(idl_version=999))
    assert reply.status is PlanStatus.INVALID_REQUEST
    assert "idl_version" in reply.detail
    assert reply.stats.iterations == 0


def test_asking_for_a_map_the_service_does_not_have_is_refused() -> None:
    reply = plan(_request(use_preloaded_map=True), preloaded=None)
    assert reply.status is PlanStatus.INVALID_REQUEST
    assert "preloaded" in reply.detail


def test_the_preloaded_map_actually_changes_the_route(tmp_path: Path) -> None:
    path = tmp_path / "m.xml"
    path.write_text(
        '<vtx-map version="1"><safezones/><obstacles>'
        '<polygon><point x="150000" y="120000"/><point x="200000" y="120000"/>'
        '<point x="175000" y="200000"/></polygon>'
        '<circle cx="220000" cy="180000" r="15000"/>'
        "</obstacles></vtx-map>",
        encoding="utf-8",
    )
    loaded = PreloadedMap.load(path)
    open_water = plan(_request())
    with_basemap = plan(_request(use_preloaded_map=True), preloaded=loaded)
    assert with_basemap.status is PlanStatus.OK
    assert with_basemap.path_length_m > open_water.path_length_m


def test_wall_time_is_measured_and_positive() -> None:
    assert plan(_request()).plan_wall_time_s > 0.0
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/planner_test.py -v`
Expected: FAIL — `ImportError: cannot import name 'plan' from 'vtx_service'`.

- [ ] **Step 3: Viết `planner.py`**

Create `service/vtx_service/planner.py`:

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

from vtx_service.angles import math_rad_to_bearing_deg
from vtx_service.map_file import PreloadedMap
from vtx_service.messages import (
    IDL_VERSION,
    PlanReply,
    PlanRequest,
    PlanStatus,
    SearchStats,
    Waypoint,
)
from vtx_service.runtime import (
    config_hash,
    effective_max_iterations,
    effective_time_budget_s,
    planner_version,
)
from vtx_service.scenario_builder import build_scenario

_REASON_TO_STATUS = {
    "no_path": PlanStatus.NO_PATH,
    "start_leg_blocked": PlanStatus.START_LEG_BLOCKED,
    "goal_leg_blocked": PlanStatus.GOAL_LEG_BLOCKED,
}
"""Tập lý do có mã riêng. Mọi chuỗi khác đến từ oracle và mang tham số, nên nó
đi nguyên văn vào ``detail`` thay vì bị ép vào enum làm mất thông tin."""


def plan(request: PlanRequest, preloaded: PreloadedMap | None = None) -> PlanReply:
    """Lập kế hoạch cho một mission.

    Args:
        request: Mission cần giải, toạ độ mét trong hệ Oxy.
        preloaded: Bản đồ nền tĩnh, hoặc ``None`` khi service không nạp bản đồ
            nào. Chỉ dùng khi ``request.use_preloaded_map`` bật.

    Returns:
        Đường bay đầy đủ ``O..T``, kèm trạng thái, bộ đếm search và nhận dạng
        phiên bản/cấu hình.
    """
    started = time.perf_counter()

    if request.idl_version != IDL_VERSION:
        return _refusal(request, f"idl_version {request.idl_version} != {IDL_VERSION}")

    if request.use_preloaded_map:
        if preloaded is None:
            return _refusal(
                request, "yêu cầu preloaded map nhưng service không nạp bản đồ nào"
            )
        request = preloaded.merged_into(request)

    try:
        preprocessed = prep.prepare_scenario(
            build_scenario(request),
            turn_radius=request.limits.turn_radius_m,
            l0=request.limits.l0_m,
            dss=request.limits.dss_m,
            safe_margin=request.limits.safe_margin_m,
            alpha_max_rad=math.radians(request.limits.alpha_max_deg),
        )
    except (ValueError, KeyError, TypeError) as exc:
        return _refusal(request, f"hình học không dựng được: {exc}")

    search_started = time.perf_counter()
    result = astar.plan_trajectory(preprocessed)
    search_elapsed = time.perf_counter() - search_started

    status, detail = _classify(result)
    return PlanReply(
        request_id=request.request_id,
        idl_version=IDL_VERSION,
        status=status,
        detail=detail,
        waypoints=_waypoints_out(result, preprocessed),
        path_length_m=_planar_length(result, preprocessed),
        plan_wall_time_s=time.perf_counter() - started,
        applied_time_budget_s=effective_time_budget_s(),
        stats=_stats_out(result, search_elapsed),
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
    result: dict[str, Any], preprocessed: dict[str, Any]
) -> tuple[Waypoint, ...]:
    """Đưa đường bay về quy ước góc của client. Toạ độ không đổi."""
    return tuple(
        Waypoint(position=position, heading_deg=math_rad_to_bearing_deg(heading))
        for position, heading in _full_path(result, preprocessed)
    )


def _planar_length(result: dict[str, Any], preprocessed: dict[str, Any]) -> float:
    """Tổng chiều dài các dây cung.

    Cùng công thức ``scripts/ab_planners.py`` dùng, nên số liệu so sánh được với
    các benchmark đã ghi.
    """
    full = _full_path(result, preprocessed)
    return sum(math.dist(full[i][0], full[i + 1][0]) for i in range(len(full) - 1))


def _stats_out(result: dict[str, Any], search_elapsed: float) -> SearchStats:
    """Đóng gói bộ đếm search, kèm cờ cho biết ngân sách có chạm trần không."""
    stats = result["stats"]
    budget_s = effective_time_budget_s()
    budget_bound = stats["iterations"] >= stats["max_iterations"] or (
        budget_s > 0.0 and search_elapsed >= budget_s
    )
    return SearchStats(
        iterations=stats["iterations"],
        max_iterations=stats["max_iterations"],
        open_set_size=stats["open_set_size"],
        search_failed=stats["search_failed"],
        budget_bound=budget_bound,
    )


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
        applied_time_budget_s=effective_time_budget_s(),
        stats=SearchStats(0, effective_max_iterations(), 0, True, False),
        planner_version=planner_version(),
        config_hash=config_hash(),
    )
```

- [ ] **Step 4: Export `plan`**

Modify `service/vtx_service/__init__.py`: thêm `from vtx_service.planner import plan` và `"plan"` vào `__all__`.

- [ ] **Step 5: Chạy test**

Run: `python -m pytest service/tests/ -q`
Expected: tất cả PASS.

- [ ] **Step 6: Commit**

```bash
git add service/vtx_service/planner.py service/vtx_service/__init__.py service/tests/planner_test.py
git commit -m "feat(service): plan(PlanRequest) -> PlanReply over the shipped v0 planner"
```

---

### Task 8: Test tương đương

**Files:**
- Test: `service/tests/equivalence_test.py`

**Interfaces:**
- Consumes: `plan` (Task 7), `build_scenario` (Task 5).
- Produces: không có mã production. Đây là bộ gác.

Task này chỉ có test — đó là chủ đích. Nó là cơ chế số 2 trong spec, và là thứ khiến service tự đi theo thuật toán mà không cần sửa tay.

- [ ] **Step 1: Viết test**

Create `service/tests/equivalence_test.py`:

```python
"""Cơ chế cưỡng chế số 2: adapter không được làm sai lệch bất cứ điều gì.

Hai khẳng định tách bạch, và việc tách chúng ra là có lý do.

`test_adapter_is_transparent` so đường bay qua service với đường bay khi gọi
thẳng thuật toán TRÊN CÙNG MỘT dict Scenario. Yêu cầu là bit-identical. Cả hai
vế đều gọi thuật toán HIỆN HÀNH, nên test không bao giờ lỗi thời: thuật toán đổi
thì hai vế đổi cùng nhau và test vẫn xanh; adapter lệch đi thì đỏ ngay.

`test_every_preset_still_solves_through_the_service` KHÔNG đòi bit-identical, vì
có một khác biệt ngữ nghĩa cố ý: preset mang `map_bounds = (500000, 500000)`
còn IDL bỏ trường đó (spec mục 4.2), nên service chạy ở chế độ không giới hạn.
Đòi bit-identical ở đây là ép hai thứ khác nhau phải giống nhau.
"""

from __future__ import annotations

import math

import config
import core.kinodynamic_astar_v0 as astar
import core.map_generator as mg
import core.mission as mission
import core.preprocessing as prep
import pytest

from vtx_service import plan
from vtx_service.angles import math_rad_to_bearing_deg
from vtx_service.messages import (
    Circle,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    VehicleLimits,
)
from vtx_service.scenario_builder import build_scenario

LIMITS = VehicleLimits(
    turn_radius_m=config.R,
    l0_m=config.L0,
    dss_m=config.DSS,
    safe_margin_m=config.SAFE_MARGIN,
    alpha_max_deg=config.ALPHA_MAX,
)
# Ngân sách trên dây chưa được tôn trọng (spec mục 4.3), nhưng vẫn phải hợp lệ.
BUDGET = SearchBudget(
    time_budget_s=float(config.TIME_BUDGET_S or 15.0),
    max_iterations=config.MAX_ITERATIONS,
)
SCENARIOS = sorted(mg.get_all_scenarios())


def _request_from_scenario(name: str) -> PlanRequest:
    """Dựng một request tương đương với một preset."""
    scenario = mg.get_all_scenarios()[name]()
    goal_heading = scenario["goal_heading"]
    return PlanRequest(
        request_id=name.encode("utf-8")[:16].ljust(16, b"\x00"),
        idl_version=1,
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
    preprocessed = prep.prepare_scenario(
        build_scenario(request),
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
        # Bit-identical: không có phép toán nào chạm vào toạ độ.
        assert got.position == position
        assert got.heading_deg == pytest.approx(math_rad_to_bearing_deg(heading), abs=1e-9)

    expected_length = sum(
        math.dist(expected_full[i][0], expected_full[i + 1][0])
        for i in range(len(expected_full) - 1)
    )
    assert reply.path_length_m == pytest.approx(expected_length, rel=0.0, abs=1e-9)
    assert reply.stats.iterations == result["stats"]["iterations"]


def test_every_preset_still_solves_through_the_service() -> None:
    failures = [
        name
        for name in SCENARIOS
        if plan(_request_from_scenario(name)).status is not PlanStatus.OK
    ]
    assert failures == [], f"service làm mất mission: {failures}"


@pytest.mark.parametrize("name", SCENARIOS)
def test_service_does_not_lengthen_the_route_against_the_preset(name: str) -> None:
    """So với preset NGUYÊN BẢN (còn map_bounds), không phải dict của adapter."""
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
Expected: PASS, 37 passed (18 + 1 + 18). Mất vài chục giây vì `scenario_18` tốn ~4 s và chạy ba lần.

Nếu `test_adapter_is_transparent` đỏ ở bất kỳ preset nào: đó là khiếm khuyết THẬT của adapter, không phải test quá khắt khe — hai vế đang chạy trên cùng một dict Scenario nên chúng phải trùng khớp.

Nếu `test_service_does_not_lengthen_the_route_against_the_preset` đỏ: việc bỏ `map_bounds` có hệ quả đo được. Ghi lại preset nào, chênh bao nhiêu, và BÁO CÁO — đó là dữ liệu để quyết định có đưa một safezone hình chữ nhật vào hay không, chứ không phải cái để nới ngưỡng cho qua.

- [ ] **Step 3: Commit**

```bash
git add service/tests/equivalence_test.py
git commit -m "test(service): pin the adapter to the algorithm, bit for bit"
```

---

### Task 9: `PlanRunner` — forkserver và thời hạn cứng

**Files:**
- Create: `service/vtx_service/runner.py`
- Test: `service/tests/runner_test.py`

**Interfaces:**
- Consumes: `plan` (Task 7), `PreloadedMap` (Task 4).
- Produces: lớp `PlanRunner` với `PlanRunner(preloaded: PreloadedMap | None, grace_s: float = 2.0)`, `start()`, `submit(request: PlanRequest) -> PlanReply`, `stop()`. Task 11 dùng.

Đây là chỗ có thời hạn cứng. Đọc mục 3 của spec trước khi sửa: lựa chọn `forkserver` là kết quả đo, không phải sở thích.

- [ ] **Step 1: Viết test**

Create `service/tests/runner_test.py`:

```python
"""Thời hạn cứng, và tính bền của service sau khi phải giết một tiến trình con.

Planner là Python thuần, CPU-bound, và chỉ kiểm tra ngân sách tại các điểm trong
vòng lặp search - nó KHÔNG hủy được từ bên ngoài một cách lịch sự. Tiến trình
con là cách duy nhất để có thời hạn cứng thật.
"""

from __future__ import annotations

import time

import pytest

from vtx_service.messages import (
    Circle,
    PlanRequest,
    PlanStatus,
    SearchBudget,
    VehicleLimits,
)
from vtx_service.runner import PlanRunner

LIMITS = VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0)


def _request(**overrides: object) -> PlanRequest:
    base: dict[str, object] = dict(
        request_id=b"\x06" * 16,
        idl_version=1,
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


@pytest.fixture()
def runner():
    instance = PlanRunner(preloaded=None)
    instance.start()
    try:
        yield instance
    finally:
        instance.stop()


def test_a_mission_plans_in_a_child_process(runner: PlanRunner) -> None:
    reply = runner.submit(_request())
    assert reply.status is PlanStatus.OK
    assert len(reply.waypoints) >= 2


def test_the_reply_survives_the_process_boundary_intact(runner: PlanRunner) -> None:
    reply = runner.submit(_request())
    assert reply.request_id == b"\x06" * 16
    assert isinstance(reply.status, PlanStatus)
    # double phải qua pickle nguyên vẹn từng bit.
    assert reply.path_length_m == runner.submit(_request()).path_length_m


def test_child_start_cost_is_within_the_measured_envelope(runner: PlanRunner) -> None:
    """Spec ghi median 56 ms cho forkserver + preload. Nới rộng cho máy chậm."""
    runner.submit(_request())  # bỏ lần đầu (khởi động forkserver)
    started = time.perf_counter()
    runner.submit(_request())
    assert time.perf_counter() - started < 5.0


def test_a_hung_child_becomes_timeout_and_the_runner_keeps_working() -> None:
    instance = PlanRunner(preloaded=None, grace_s=0.5)
    instance.start()
    try:
        instance.force_hang_next = True  # cửa hậu chỉ dùng cho test
        hung = instance.submit(_request())
        assert hung.status is PlanStatus.TIMEOUT
        assert hung.request_id == b"\x06" * 16
        assert hung.waypoints == ()
        # Và runner vẫn phục vụ được ngay sau đó.
        assert instance.submit(_request()).status is PlanStatus.OK
    finally:
        instance.stop()


def test_a_child_that_raises_becomes_internal_error_not_a_dead_runner() -> None:
    instance = PlanRunner(preloaded=None)
    instance.start()
    try:
        instance.force_raise_next = True  # cửa hậu chỉ dùng cho test
        broken = instance.submit(_request())
        assert broken.status is PlanStatus.INTERNAL_ERROR
        assert broken.detail
        assert instance.submit(_request()).status is PlanStatus.OK
    finally:
        instance.stop()


def test_config_mutation_in_a_child_cannot_leak_into_the_parent(runner: PlanRunner) -> None:
    """Cách ly 35 hằng số global là một trong hai lý do có tiến trình con."""
    import config

    before = config.NUM_START_CORNERS
    runner.submit(_request())
    assert config.NUM_START_CORNERS == before
```

- [ ] **Step 2: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/runner_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_service.runner'`.

- [ ] **Step 3: Viết `runner.py`**

Create `service/vtx_service/runner.py`:

```python
"""Chạy mỗi lần lập kế hoạch trong một tiến trình con, với thời hạn cứng.

Hai lý do, cả hai đều thật:

Thứ nhất, planner là Python thuần, CPU-bound, và chỉ kiểm tra ngân sách tại các
điểm trong vòng lặp search - nó KHÔNG hủy được từ bên ngoài một cách lịch sự.
Giết một tiến trình con là cách trung thực duy nhất để có thời hạn cứng.

Thứ hai, planner đọc 35 hằng số global từ ``config``. Trong một tiến trình con
dùng-một-lần thì mọi thay đổi đều chết theo nó.

VÌ SAO ``forkserver`` CHỨ KHÔNG PHẢI ``fork``: DDS chạy thread nền ở tầng C, và
``fork()`` từ một tiến trình có thread là công thức kinh điển của deadlock trong
bản sao - fork chỉ mang theo thread đang gọi, nên một mutex do thread khác đang
giữ sẽ bị giữ vĩnh viễn. Đo trên máy phát triển: ``fork`` với ``core`` nạp sẵn
là 37,7 ms và sống sót 15/15 lần dưới lưu lượng DDS, nhưng 15 lần thành công
không phải bằng chứng an toàn cho một deadlock xác suất. ``forkserver`` +
preload, khởi động TRƯỚC khi DDS tồn tại, là 56,4 ms và an toàn về CẤU TRÚC.
40 ms là vô nghĩa so với 16 ms - 4 s thời gian lập kế hoạch.

Hệ quả với chỗ gọi: :meth:`PlanRunner.start` phải chạy TRƯỚC khi khởi tạo bất kỳ
thứ gì thuộc DDS.
"""

from __future__ import annotations

import multiprocessing as mp
import time
import traceback
from multiprocessing.connection import Connection

from vtx_service.map_file import PreloadedMap
from vtx_service.messages import (
    IDL_VERSION,
    PlanReply,
    PlanRequest,
    PlanStatus,
    SearchStats,
)

_PRELOAD = [
    "config",
    "core.types",
    "core.spatial_utils",
    "core.preprocessing",
    "core.path_validation",
    "core.mission",
    "core.arc_geometry",
    "core.goal_shot",
    "core.kinodynamic_astar_v0",
    "vtx_service.planner",
]
"""Module nạp sẵn trong tiến trình forkserver.

Trả giá import một lần thay vì mỗi request: đo được 1012 ms xuống 56 ms.
"""


def _child(pipe: Connection, request: PlanRequest, preloaded: PreloadedMap | None,
           hang: bool, raise_: bool) -> None:
    """Thân tiến trình con: lập kế hoạch, gửi reply, thoát."""
    try:
        if hang:
            while True:
                time.sleep(3600)
        if raise_:
            raise RuntimeError("lỗi giả lập trong tiến trình con")
        from vtx_service.planner import plan

        pipe.send(("ok", plan(request, preloaded=preloaded)))
    except BaseException:  # noqa: BLE001 - báo lỗi về cha thay vì chết câm
        pipe.send(("loi", traceback.format_exc(limit=5)))
    finally:
        pipe.close()


class PlanRunner:
    """Chạy các request lần lượt, mỗi request một tiến trình con."""

    def __init__(self, preloaded: PreloadedMap | None, grace_s: float = 2.0) -> None:
        """Khởi tạo.

        Args:
            preloaded: Bản đồ nền tĩnh, hoặc ``None``.
            grace_s: Cộng thêm vào ``config.TIME_BUDGET_S`` để ra thời hạn cứng.
        """
        self._preloaded = preloaded
        self._grace_s = grace_s
        self._ctx: mp.context.BaseContext | None = None
        # Cửa hậu chỉ dùng cho test; production không bao giờ đặt chúng.
        self.force_hang_next = False
        self.force_raise_next = False

    def start(self) -> None:
        """Khởi động forkserver. PHẢI gọi trước khi khởi tạo DDS."""
        mp.set_forkserver_preload(_PRELOAD)
        self._ctx = mp.get_context("forkserver")
        # Ép forkserver ra đời NGAY BÂY GIỜ, trong khi tiến trình này còn sạch
        # thread. Nếu để nó ra đời ở request đầu tiên thì DDS đã lên rồi.
        self._ctx.Process(target=_noop).start()

    def submit(self, request: PlanRequest) -> PlanReply:
        """Lập kế hoạch cho một request, với thời hạn cứng.

        Args:
            request: Mission cần giải.

        Returns:
            Reply của planner, hoặc một reply ``TIMEOUT`` / ``INTERNAL_ERROR``.
        """
        assert self._ctx is not None, "phải gọi start() trước"
        from vtx_service.runtime import effective_time_budget_s

        hang, self.force_hang_next = self.force_hang_next, False
        raise_, self.force_raise_next = self.force_raise_next, False

        parent, child = self._ctx.Pipe(duplex=False)
        process = self._ctx.Process(
            target=_child, args=(child, request, self._preloaded, hang, raise_)
        )
        process.start()
        child.close()

        deadline_s = effective_time_budget_s() + self._grace_s
        if not parent.poll(timeout=deadline_s):
            process.kill()
            process.join(timeout=10)
            parent.close()
            return self._failed(request, PlanStatus.TIMEOUT,
                                f"vượt thời hạn cứng {deadline_s:.1f} s")

        try:
            tag, payload = parent.recv()
        except EOFError:
            tag, payload = "loi", "tiến trình con chết không gửi gì"
        finally:
            parent.close()
            process.join(timeout=10)

        if tag != "ok":
            return self._failed(request, PlanStatus.INTERNAL_ERROR, str(payload))
        return payload

    def stop(self) -> None:
        """Dừng runner. Không có tiến trình sống lâu nào phải dọn."""
        self._ctx = None

    @staticmethod
    def _failed(request: PlanRequest, status: PlanStatus, detail: str) -> PlanReply:
        from vtx_service.runtime import (
            config_hash,
            effective_max_iterations,
            effective_time_budget_s,
            planner_version,
        )

        return PlanReply(
            request_id=request.request_id,
            idl_version=IDL_VERSION,
            status=status,
            detail=detail,
            waypoints=(),
            path_length_m=0.0,
            plan_wall_time_s=0.0,
            applied_time_budget_s=effective_time_budget_s(),
            stats=SearchStats(0, effective_max_iterations(), 0, True, True),
            planner_version=planner_version(),
            config_hash=config_hash(),
        )


def _noop() -> None:
    """Thân tiến trình rỗng, chỉ để ép forkserver khởi động sớm."""
```

- [ ] **Step 4: Chạy test**

Run: `python -m pytest service/tests/runner_test.py -v`
Expected: PASS, 6 passed. Test thời hạn mất ~1 s; các test khác mỗi cái vài trăm ms.

Nếu `test_a_hung_child_becomes_timeout_and_the_runner_keeps_working` treo quá 60 s: `process.kill()` không hạ được tiến trình con. Kiểm tra rằng `_child` không bắt `SystemExit` và rằng `kill` (SIGKILL) được dùng chứ không phải `terminate` (SIGTERM).

- [ ] **Step 5: Commit**

```bash
git add service/vtx_service/runner.py service/tests/runner_test.py
git commit -m "feat(service): plan in a forkserver child, so the deadline can actually bite"
```

---

### Task 10: Lớp transport DDS

**Files:**
- Create: `service/vtx_service/transport.py`
- Create: `service/idl/vtx_path_planning.idl`
- Test: `service/tests/transport_test.py`

**Interfaces:**
- Consumes: `PlanRequest`, `PlanReply`, `PlanStatus` (Task 2); quyết định stack từ Task 1.
- Produces: giao thức `Transport` với `serve(handler: Callable[[PlanRequest], PlanReply]) -> None` và `close()`; cài đặt cụ thể cho stack đã chọn. Task 11 dùng.

Đây là module DUY NHẤT trong service được phép import DDS. Kết quả Task 1 chỉ ảnh hưởng file này.

Mã dưới đây viết cho **Cyclone DDS**, và các lời gọi API đã được kiểm chứng chạy trên máy phát triển. Nếu Task 1 chọn Fast DDS thì giữ nguyên interface `Transport` và cài đặt lại phần thân, dùng đoạn publish/subscribe chạy được mà spike đã tạo ra.

- [ ] **Step 1: Viết IDL cho bên gọi**

Create `service/idl/vtx_path_planning.idl`:

```idl
// Hợp đồng dữ liệu của service lập kế hoạch đường bay VTX.
//
// Toạ độ: MÉT trên mặt phẳng Oxy, +y là bắc, +x là đông.
// Góc: ĐỘ, phương vị thật, thuận chiều kim đồng hồ từ chính bắc.
//
// Giá trị PlanStatus khớp chính xác service/vtx_service/messages.py.
// Không có trường frame: chỉ có một hệ toạ độ. Không có map_bounds: safezones
// biểu diễn được đúng vùng đó và không phụ thuộc vào vị trí gốc toạ độ.
//
// budget CHƯA được tôn trọng; reply mang applied_time_budget_s là giá trị
// service thực sự đã dùng.

module vtx { module planning {

enum PlanStatus { PLAN_OK, PLAN_NO_PATH, PLAN_START_LEG_BLOCKED,
                  PLAN_GOAL_LEG_BLOCKED, PLAN_ORACLE_REJECTED,
                  PLAN_INVALID_REQUEST, PLAN_TIMEOUT, PLAN_INTERNAL_ERROR,
                  PLAN_BUSY };

struct Point2D  { double x; double y; };
struct Polygon  { sequence<Point2D> vertices; };   // vành mở
struct Circle   { Point2D center; double radius_m; };

struct VehicleLimits {
  double turn_radius_m;
  double l0_m;
  double dss_m;
  double safe_margin_m;
  double alpha_max_deg;
};

struct SearchBudget {
  double        time_budget_s;
  unsigned long max_iterations;
};

struct VtxPathPlanRequest {
  @key octet          request_id[16];
  unsigned long       idl_version;
  Point2D             start;
  double              start_heading_deg;
  Point2D             goal;
  double              goal_heading_deg;
  boolean             goal_heading_free;
  sequence<Polygon>   islands;
  sequence<Circle>    dynamic_obstacles;
  sequence<Polygon>   safezones;
  boolean             use_preloaded_map;
  VehicleLimits       limits;
  SearchBudget        budget;
};

struct Waypoint { Point2D position; double heading_deg; };

struct SearchStats {
  unsigned long iterations;
  unsigned long max_iterations;
  unsigned long open_set_size;
  boolean       search_failed;
  boolean       budget_bound;
};

struct VtxPathPlanReply {
  @key octet          request_id[16];
  unsigned long       idl_version;
  PlanStatus          status;
  string              detail;
  sequence<Waypoint>  waypoints;      // đường bay đầy đủ O..T
  double              path_length_m;
  double              plan_wall_time_s;
  double              applied_time_budget_s;
  SearchStats         stats;
  string              planner_version;
  string              config_hash;
};

}; };
```

- [ ] **Step 2: Viết test**

Create `service/tests/transport_test.py`:

```python
"""Round-trip qua DDS thật, so với việc gọi handler trong tiến trình.

Test tự bỏ qua CÓ LÝ DO khi binding chưa có. Một test bị bỏ qua trong im lặng
còn tệ hơn không có test.
"""

from __future__ import annotations

import re
import threading
import time
import uuid
from pathlib import Path

import pytest

from vtx_service.messages import (
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

pytest.importorskip(
    "cyclonedds", reason="chưa cài binding DDS; xem quyết định ở Task 1"
)

from vtx_service.transport import DdsTransport  # noqa: E402

IDL_PATH = Path(__file__).resolve().parents[1] / "idl" / "vtx_path_planning.idl"
DOMAIN = 92


def _request(request_id: bytes) -> PlanRequest:
    return PlanRequest(
        request_id=request_id,
        idl_version=IDL_VERSION,
        start=(50000.0, 50000.0),
        start_heading_deg=45.0,
        goal=(300000.0, 250000.0),
        goal_heading_deg=137.5,
        goal_heading_free=False,
        islands=(((1e5, 1e5), (1.2e5, 1e5), (1.1e5, 1.3e5)),),
        dynamic_obstacles=(Circle(center=(2e5, 1.5e5), radius_m=12000.0),),
        safezones=(((0.0, 0.0), (5e5, 0.0), (5e5, 5e5)),),
        use_preloaded_map=False,
        limits=VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0),
        budget=SearchBudget(15.0, 50000),
    )


def _reply(request: PlanRequest) -> PlanReply:
    return PlanReply(
        request_id=request.request_id,
        idl_version=IDL_VERSION,
        status=PlanStatus.ORACLE_REJECTED,
        detail="first W1..W2 l=7421.3 < L0=8000",
        waypoints=(Waypoint((1.5, -2.5), 137.5), Waypoint((3e5, 2.5e5), 42.0)),
        path_length_m=123456.78901234567,
        plan_wall_time_s=0.0421,
        applied_time_budget_s=15.0,
        stats=SearchStats(1234, 50000, 56, True, False),
        planner_version="v1.0-3-gabc1234-dirty",
        config_hash="0123456789abcdef",
    )


def test_idl_status_enum_matches_python_exactly() -> None:
    text = IDL_PATH.read_text(encoding="utf-8")
    match = re.search(r"enum\s+PlanStatus\s*\{(.*?)\}", text, re.DOTALL)
    assert match
    members = [item.strip() for item in match.group(1).split(",") if item.strip()]
    assert members == [f"PLAN_{member.name}" for member in PlanStatus]


def test_idl_has_no_frame_field() -> None:
    assert "frame" not in IDL_PATH.read_text(encoding="utf-8")


def test_a_request_survives_the_wire_unchanged() -> None:
    request_id = uuid.uuid4().bytes
    seen: list[PlanRequest] = []
    done = threading.Event()

    def handler(incoming: PlanRequest) -> PlanReply:
        seen.append(incoming)
        done.set()
        return _reply(incoming)

    service = DdsTransport(domain_id=DOMAIN)
    client = DdsTransport(domain_id=DOMAIN)
    thread = threading.Thread(target=service.serve, args=(handler,), daemon=True)
    thread.start()
    try:
        assert client.wait_for_service(timeout_s=20.0)
        reply = client.request(_request(request_id), timeout_s=30.0)
        assert done.wait(timeout=5.0)

        got = seen[0]
        assert got.request_id == request_id
        assert got.start == (50000.0, 50000.0)
        assert got.goal_heading_deg == 137.5
        assert got.goal_heading_free is False
        assert len(got.islands[0]) == 3
        assert got.dynamic_obstacles[0].radius_m == 12000.0
        assert len(got.safezones[0]) == 3
        assert got.limits.alpha_max_deg == 90.0

        assert reply.request_id == request_id
        assert reply.status is PlanStatus.ORACLE_REJECTED
        assert reply.detail == "first W1..W2 l=7421.3 < L0=8000"
        # double phải qua dây nguyên vẹn từng bit.
        assert reply.path_length_m == 123456.78901234567
        assert reply.waypoints[0].position == (1.5, -2.5)
        assert reply.stats.iterations == 1234
        assert reply.config_hash == "0123456789abcdef"
    finally:
        service.close()
        client.close()


def test_a_reply_for_another_request_is_ignored() -> None:
    """Tương quan bằng request_id, không phải bằng thứ tự đến."""
    def handler(incoming: PlanRequest) -> PlanReply:
        return _reply(incoming)

    service = DdsTransport(domain_id=DOMAIN + 3)
    client = DdsTransport(domain_id=DOMAIN + 3)
    threading.Thread(target=service.serve, args=(handler,), daemon=True).start()
    try:
        assert client.wait_for_service(timeout_s=20.0)
        first = uuid.uuid4().bytes
        second = uuid.uuid4().bytes
        assert client.request(_request(first), timeout_s=30.0).request_id == first
        assert client.request(_request(second), timeout_s=30.0).request_id == second
    finally:
        service.close()
        client.close()
```

- [ ] **Step 3: Chạy để xác nhận đỏ**

Run: `python -m pytest service/tests/transport_test.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'vtx_service.transport'`, hoặc skipped nếu chưa cài binding.

- [ ] **Step 4: Viết `transport.py`**

Create `service/vtx_service/transport.py`:

```python
"""Lớp DDS. Module DUY NHẤT trong service được phép import một binding DDS.

Mọi thứ khác - hợp đồng dữ liệu, adapter, runner, bản đồ - độc lập với stack đã
chọn. Đổi stack là viết lại file này, không đụng chỗ nào khác.

QoS: cả hai topic RELIABLE + VOLATILE; request KEEP_ALL, reply KEEP_LAST(8).
VOLATILE là bắt buộc, không phải mặc định tuỳ tiện: TRANSIENT_LOCAL trên topic
request nghĩa là service khởi động lại sẽ nhận và lập kế hoạch lại một mission
cũ đã hết hiệu lực. Một lệnh bay không được phép phát lại.

KHÔNG dùng `from __future__ import annotations` trong file này. cyclonedds phân
giải chú thích kiểu LÚC CHẠY, còn PEP 563 biến chúng thành chuỗi, nên
`Topic(...)` ném `TypeError: Type array[uint8, 16] ... cannot be resolved`. Đã
đo: có dòng đó thì hỏng, bỏ ra thì chạy. Mọi module khác trong service vẫn dùng
bình thường - chỉ module khai báo IdlStruct mới bị.
"""

import time
from collections.abc import Callable
from dataclasses import dataclass

from cyclonedds.core import (
    InstanceState,
    Policy,
    Qos,
    ReadCondition,
    SampleState,
    ViewState,
    WaitSet,
)
from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.annotations import key
from cyclonedds.idl.types import array, sequence, uint8, uint32
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.topic import Topic
from cyclonedds.util import duration

from vtx_service import messages as msg

REQUEST_TOPIC = "VtxPathPlanRequest"
REPLY_TOPIC = "VtxPathPlanReply"

_RELIABLE = Policy.Reliability.Reliable(duration(seconds=10))
# IgnoreLocal.Participant là BẮT BUỘC, không phải tuỳ chọn. Không có nó, một
# DataWriter khớp với DataReader của CHÍNH participant mình, nên
# `wait_for_service` trả True ngay cả khi không có service nào - đã đo:
# current_count = 1 với một participant duy nhất, = 0 khi bật IgnoreLocal.
_IGNORE_SELF = Policy.IgnoreLocal.Participant
REQUEST_QOS = Qos(_RELIABLE, Policy.History.KeepAll, Policy.Durability.Volatile, _IGNORE_SELF)
REPLY_QOS = Qos(_RELIABLE, Policy.History.KeepLast(8), Policy.Durability.Volatile, _IGNORE_SELF)


# --- kiểu trên dây, khớp service/idl/vtx_path_planning.idl --------------------


@dataclass
class Point2D(IdlStruct, typename="vtx.planning.Point2D"):
    x: float
    y: float


@dataclass
class Polygon(IdlStruct, typename="vtx.planning.Polygon"):
    vertices: sequence[Point2D]


@dataclass
class Circle(IdlStruct, typename="vtx.planning.Circle"):
    center: Point2D
    radius_m: float


@dataclass
class VehicleLimits(IdlStruct, typename="vtx.planning.VehicleLimits"):
    turn_radius_m: float
    l0_m: float
    dss_m: float
    safe_margin_m: float
    alpha_max_deg: float


@dataclass
class SearchBudget(IdlStruct, typename="vtx.planning.SearchBudget"):
    time_budget_s: float
    max_iterations: uint32


@dataclass
class WireRequest(IdlStruct, typename="vtx.planning.VtxPathPlanRequest"):
    request_id: array[uint8, 16]
    key("request_id")
    idl_version: uint32
    start: Point2D
    start_heading_deg: float
    goal: Point2D
    goal_heading_deg: float
    goal_heading_free: bool
    islands: sequence[Polygon]
    dynamic_obstacles: sequence[Circle]
    safezones: sequence[Polygon]
    use_preloaded_map: bool
    limits: VehicleLimits
    budget: SearchBudget


@dataclass
class Waypoint(IdlStruct, typename="vtx.planning.Waypoint"):
    position: Point2D
    heading_deg: float


@dataclass
class SearchStats(IdlStruct, typename="vtx.planning.SearchStats"):
    iterations: uint32
    max_iterations: uint32
    open_set_size: uint32
    search_failed: bool
    budget_bound: bool


@dataclass
class WireReply(IdlStruct, typename="vtx.planning.VtxPathPlanReply"):
    request_id: array[uint8, 16]
    key("request_id")
    idl_version: uint32
    status: uint32
    detail: str
    waypoints: sequence[Waypoint]
    path_length_m: float
    plan_wall_time_s: float
    applied_time_budget_s: float
    stats: SearchStats
    planner_version: str
    config_hash: str


# --- dịch giữa kiểu trên dây và kiểu nội bộ ----------------------------------


def _ring(polygon: Polygon) -> tuple[msg.Point, ...]:
    return tuple((v.x, v.y) for v in polygon.vertices)


def _to_domain(wire: WireRequest) -> msg.PlanRequest:
    return msg.PlanRequest(
        request_id=bytes(wire.request_id),
        idl_version=int(wire.idl_version),
        start=(wire.start.x, wire.start.y),
        start_heading_deg=wire.start_heading_deg,
        goal=(wire.goal.x, wire.goal.y),
        goal_heading_deg=wire.goal_heading_deg,
        goal_heading_free=wire.goal_heading_free,
        islands=tuple(_ring(p) for p in wire.islands),
        dynamic_obstacles=tuple(
            msg.Circle(center=(c.center.x, c.center.y), radius_m=c.radius_m)
            for c in wire.dynamic_obstacles
        ),
        safezones=tuple(_ring(p) for p in wire.safezones),
        use_preloaded_map=wire.use_preloaded_map,
        limits=msg.VehicleLimits(
            wire.limits.turn_radius_m,
            wire.limits.l0_m,
            wire.limits.dss_m,
            wire.limits.safe_margin_m,
            wire.limits.alpha_max_deg,
        ),
        budget=msg.SearchBudget(wire.budget.time_budget_s, int(wire.budget.max_iterations)),
    )


def _to_wire_request(request: msg.PlanRequest) -> WireRequest:
    def rings(source: tuple[tuple[msg.Point, ...], ...]) -> list[Polygon]:
        return [Polygon(vertices=[Point2D(x, y) for x, y in ring]) for ring in source]

    return WireRequest(
        request_id=list(request.request_id),
        idl_version=request.idl_version,
        start=Point2D(*request.start),
        start_heading_deg=request.start_heading_deg,
        goal=Point2D(*request.goal),
        goal_heading_deg=request.goal_heading_deg,
        goal_heading_free=request.goal_heading_free,
        islands=rings(request.islands),
        dynamic_obstacles=[
            Circle(center=Point2D(*c.center), radius_m=c.radius_m)
            for c in request.dynamic_obstacles
        ],
        safezones=rings(request.safezones),
        use_preloaded_map=request.use_preloaded_map,
        limits=VehicleLimits(
            request.limits.turn_radius_m,
            request.limits.l0_m,
            request.limits.dss_m,
            request.limits.safe_margin_m,
            request.limits.alpha_max_deg,
        ),
        budget=SearchBudget(request.budget.time_budget_s, request.budget.max_iterations),
    )


def _to_wire_reply(reply: msg.PlanReply) -> WireReply:
    return WireReply(
        request_id=list(reply.request_id),
        idl_version=reply.idl_version,
        status=int(reply.status),
        detail=reply.detail,
        waypoints=[
            Waypoint(position=Point2D(*w.position), heading_deg=w.heading_deg)
            for w in reply.waypoints
        ],
        path_length_m=reply.path_length_m,
        plan_wall_time_s=reply.plan_wall_time_s,
        applied_time_budget_s=reply.applied_time_budget_s,
        stats=SearchStats(
            reply.stats.iterations,
            reply.stats.max_iterations,
            reply.stats.open_set_size,
            reply.stats.search_failed,
            reply.stats.budget_bound,
        ),
        planner_version=reply.planner_version,
        config_hash=reply.config_hash,
    )


def _from_wire_reply(wire: WireReply) -> msg.PlanReply:
    return msg.PlanReply(
        request_id=bytes(wire.request_id),
        idl_version=int(wire.idl_version),
        status=msg.PlanStatus(int(wire.status)),
        detail=wire.detail,
        waypoints=tuple(
            msg.Waypoint(position=(w.position.x, w.position.y), heading_deg=w.heading_deg)
            for w in wire.waypoints
        ),
        path_length_m=wire.path_length_m,
        plan_wall_time_s=wire.plan_wall_time_s,
        applied_time_budget_s=wire.applied_time_budget_s,
        stats=msg.SearchStats(
            int(wire.stats.iterations),
            int(wire.stats.max_iterations),
            int(wire.stats.open_set_size),
            wire.stats.search_failed,
            wire.stats.budget_bound,
        ),
        planner_version=wire.planner_version,
        config_hash=wire.config_hash,
    )


# --- transport ----------------------------------------------------------------


class DdsTransport:
    """Hai topic, tương quan bằng ``request_id``.

    Cùng một lớp đóng cả hai vai: :meth:`serve` cho phía service, :meth:`request`
    cho phía client trong test và công cụ chẩn đoán.
    """

    def __init__(self, domain_id: int = 0) -> None:
        self._participant = DomainParticipant(domain_id)
        self._request_topic = Topic(
            self._participant, REQUEST_TOPIC, WireRequest, qos=REQUEST_QOS
        )
        self._reply_topic = Topic(self._participant, REPLY_TOPIC, WireReply, qos=REPLY_QOS)
        publisher = Publisher(self._participant)
        subscriber = Subscriber(self._participant)
        self._request_writer = DataWriter(publisher, self._request_topic, qos=REQUEST_QOS)
        self._request_reader = DataReader(subscriber, self._request_topic, qos=REQUEST_QOS)
        self._reply_writer = DataWriter(publisher, self._reply_topic, qos=REPLY_QOS)
        self._reply_reader = DataReader(subscriber, self._reply_topic, qos=REPLY_QOS)
        self._running = False

    def serve(self, handler: Callable[[msg.PlanRequest], msg.PlanReply]) -> None:
        """Nhận request và trả lời, tuần tự, tới khi :meth:`close`.

        Args:
            handler: Hàm nhận một request và trả về một reply. Được gọi lần
                lượt, không bao giờ đồng thời.
        """
        condition = ReadCondition(
            self._request_reader, ViewState.Any | InstanceState.Alive | SampleState.NotRead
        )
        waitset = WaitSet(self._participant)
        waitset.attach(condition)
        self._running = True

        while self._running:
            if waitset.wait(duration(milliseconds=200)) == 0:
                continue
            for wire in self._request_reader.take(N=16, condition=condition):
                if not self._running:
                    return
                self._reply_writer.write(_to_wire_reply(handler(_to_domain(wire))))

    def wait_for_service(self, timeout_s: float) -> bool:
        """Chờ tới khi có một service khớp trên topic request.

        Ghi trước khi khớp là mất mẫu tin trong im lặng với QoS VOLATILE.

        Args:
            timeout_s: Thời gian chờ tối đa.

        Returns:
            ``True`` nếu đã khớp.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._request_writer.get_publication_matched_status().current_count > 0:
                # Cho reader phía kia kịp khớp nốt chiều còn lại.
                time.sleep(0.5)
                return True
            time.sleep(0.1)
        return False

    def request(self, request: msg.PlanRequest, timeout_s: float = 30.0) -> msg.PlanReply:
        """Gửi một request và chờ reply khớp ``request_id``.

        Args:
            request: Mission cần gửi.
            timeout_s: Thời gian chờ tối đa.

        Returns:
            Reply tương ứng.

        Raises:
            TimeoutError: Khi không có reply khớp trong thời gian chờ.
        """
        condition = ReadCondition(
            self._reply_reader, ViewState.Any | InstanceState.Alive | SampleState.NotRead
        )
        waitset = WaitSet(self._participant)
        waitset.attach(condition)

        self._request_writer.write(_to_wire_request(request))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if waitset.wait(duration(milliseconds=200)) == 0:
                continue
            for wire in self._reply_reader.take(N=16, condition=condition):
                if bytes(wire.request_id) == request.request_id:
                    return _from_wire_reply(wire)
        raise TimeoutError(f"không có reply cho {request.request_id.hex()[:8]}")

    def close(self) -> None:
        """Dừng vòng phục vụ và giải phóng thực thể DDS."""
        self._running = False
```

- [ ] **Step 5: Chạy test**

Run: `python -m pytest service/tests/transport_test.py -v`
Expected: PASS, 4 passed.

Nếu `test_a_request_survives_the_wire_unchanged` treo ở `wait_for_service`: hai
participant không thấy nhau. Kiểm tra `domain_id` trùng nhau và multicast không
bị chặn trên loopback.

- [ ] **Step 6: Commit**

```bash
git add service/vtx_service/transport.py service/idl/ service/tests/transport_test.py
git commit -m "feat(service): the DDS layer, the only module that imports a binding"
```

---

### Task 11: Vòng đời service và systemd

**Files:**
- Create: `service/vtx_service/main.py`
- Create: `service/deploy/vtx-planner.service`
- Create: `service/deploy/requirements.txt`
- Create: `service/deploy/README.md`

**Interfaces:**
- Consumes: `PlanRunner` (Task 9), `DdsTransport` (Task 10), `PreloadedMap` (Task 4).
- Produces: một service chạy được, tự khởi động cùng máy.

- [ ] **Step 1: Viết `main.py`**

Create `service/vtx_service/main.py`:

```python
"""Vòng đời service: nạp bản đồ, khởi động runner, rồi mới lên DDS.

THỨ TỰ Ở ĐÂY LÀ MỘT RÀNG BUỘC, KHÔNG PHẢI SỞ THÍCH. `PlanRunner.start()` phải
chạy trước khi khởi tạo DDS: nó ép tiến trình forkserver ra đời trong lúc tiến
trình này còn sạch thread. Nếu để DDS lên trước, forkserver sẽ ra đời từ một
tiến trình đang chạy thread nền của DDS, và mọi tiến trình con sau đó thừa
hưởng rủi ro deadlock đúng như mục 3 của spec mô tả.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
from pathlib import Path
from types import FrameType


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="VTX path planning DDS service")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument(
        "--preloaded-map",
        type=Path,
        default=None,
        help="file XML bản đồ nền; bỏ trống thì mọi request phải tự chứa",
    )
    parser.add_argument("--grace-seconds", type=float, default=2.0)
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)s vtx-planner %(message)s",
        stream=sys.stderr,
    )
    log = logging.getLogger("vtx-planner")

    from vtx_service.map_file import PreloadedMap
    from vtx_service.messages import PlanRequest, PlanReply, PlanStatus
    from vtx_service.runner import PlanRunner
    from vtx_service.runtime import config_hash, effective_time_budget_s, planner_version

    preloaded = PreloadedMap.load(args.preloaded_map) if args.preloaded_map else None
    if preloaded is not None:
        log.info(
            "bản đồ nền: %d safezone, %d đảo, %d vòng tròn",
            len(preloaded.safezones),
            len(preloaded.islands),
            len(preloaded.dynamic_obstacles),
        )
    else:
        log.info("không có bản đồ nền; mọi request phải tự chứa")

    runner = PlanRunner(preloaded=preloaded, grace_s=args.grace_seconds)
    runner.start()  # PHẢI trước DDS - xem docstring của module
    log.info(
        "planner %s, config %s, ngân sách thực tế %.1f s",
        planner_version(),
        config_hash(),
        effective_time_budget_s(),
    )

    from vtx_service.transport import DdsTransport

    transport = DdsTransport(domain_id=args.domain_id)
    log.info("sẵn sàng trên domain %d", args.domain_id)

    def handle(request: PlanRequest) -> PlanReply:
        reply = runner.submit(request)
        log.info(
            "request %s -> %s, %d waypoint, %.3f s",
            request.request_id.hex()[:8],
            reply.status.name,
            len(reply.waypoints),
            reply.plan_wall_time_s,
        )
        return reply

    def stop(signum: int, frame: FrameType | None) -> None:
        log.info("nhận tín hiệu %s, đang dừng", signal.Signals(signum).name)
        transport.close()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        transport.serve(handle)
    finally:
        transport.close()
        runner.stop()
        log.info("đã dừng")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: Chạy thử ở tiền cảnh**

```bash
PYTHONPATH=.:service python -m vtx_service.main --domain-id 93 --log-level DEBUG
```
Expected: in ra bản đồ nền (hoặc "không có bản đồ nền"), phiên bản planner, `config_hash`, ngân sách thực tế, rồi "sẵn sàng trên domain 93". `Ctrl-C` dừng sạch.

Ở terminal khác, gửi thử một request:

```bash
PYTHONPATH=.:service python -c "
import uuid
from vtx_service.transport import DdsTransport
from vtx_service.messages import PlanRequest, SearchBudget, VehicleLimits
c = DdsTransport(domain_id=93)
assert c.wait_for_service(20.0), 'không thấy service'
r = c.request(PlanRequest(
    request_id=uuid.uuid4().bytes, idl_version=1,
    start=(50000.0, 50000.0), start_heading_deg=45.0,
    goal=(300000.0, 250000.0), goal_heading_deg=45.0, goal_heading_free=True,
    islands=(), dynamic_obstacles=(), safezones=(), use_preloaded_map=False,
    limits=VehicleLimits(8000.0, 8000.0, 15000.0, 500.0, 90.0),
    budget=SearchBudget(15.0, 50000)), timeout_s=30.0)
print(r.status.name, len(r.waypoints), 'waypoint', round(r.path_length_m/1000, 1), 'km')
"
```
Expected: `OK 4 waypoint ... km` (số waypoint có thể khác).

- [ ] **Step 3: Ghi lại phụ thuộc**

Create `service/deploy/requirements.txt`:

```
# Phụ thuộc của service. HAI gói, và không gói nào là numpy.
#
# core/ không import numpy ở đâu cả (0 lần), cũng không scipy, cũng không
# matplotlib. Cái pin numpy==1.26.4 trong requirements.txt ở gốc repo là ràng
# buộc của matplotlib 3.8 / pandas 2.1.4 trong stack test-benchmark-GUI, và nó
# KHÔNG áp dụng ở đây. Đã kiểm chứng: venv sạch chỉ có shapely kéo theo numpy
# 2.4.6 và cả 18 preset vẫn giải được.
shapely==2.1.2

# Binding DDS. Thay bằng stack mà Task 1 chọn, và cập nhật
# docs/superpowers/specs/2026-08-22-dds-stack-decision.md cùng lúc.
cyclonedds==11.0.1
```

- [ ] **Step 4: Viết unit systemd**

Create `service/deploy/vtx-planner.service`:

```ini
[Unit]
Description=VTX path planning service (DDS)
Documentation=file:/opt/vtx/path_planning/docs/superpowers/specs/2026-08-22-dds-path-planning-service-design.md
After=network.target

[Service]
Type=simple
User=vtx
Group=vtx
WorkingDirectory=/opt/vtx/path_planning

# Worker import core.* THẲNG từ cây mã nguồn, không qua wheel. Đó là cơ chế
# cập nhật tự động ở mức triển khai: git pull && systemctl restart.
Environment=PYTHONPATH=/opt/vtx/path_planning:/opt/vtx/path_planning/service
Environment=PYTHONUNBUFFERED=1

ExecStart=/opt/vtx/venv/bin/python -m vtx_service.main \
    --domain-id 0 \
    --grace-seconds 2.0
# Thêm --preloaded-map /opt/vtx/basemap.xml nếu triển khai này dùng bản đồ nền.
# Mặc định KHÔNG có: request tự chứa thì replay được và chẩn đoán được, còn
# state ẩn trong service thì không.

Restart=on-failure
RestartSec=5
KillMode=control-group
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
```

- [ ] **Step 5: Viết README triển khai**

Create `service/deploy/README.md`:

```markdown
# Triển khai VTX path planning service

Service Python độc lập, tự khởi động cùng máy. Đóng gói phân phối sẽ phân tích
riêng; phần này là bản cài trực tiếp.

## Cài đặt

```bash
sudo useradd --system --home /opt/vtx --shell /usr/sbin/nologin vtx

sudo git clone <repo> /opt/vtx/path_planning
cd /opt/vtx/path_planning && sudo git checkout <tag>

sudo python3.11 -m venv /opt/vtx/venv
sudo /opt/vtx/venv/bin/pip install -r service/deploy/requirements.txt

sudo chown -R vtx:vtx /opt/vtx
sudo cp service/deploy/vtx-planner.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now vtx-planner
```

`enable` là phần "tự khởi động cùng Ubuntu". Kiểm tra:
`journalctl -u vtx-planner -f`.

## Phụ thuộc

**Hai gói**: `shapely` và một binding DDS. Không numpy trực tiếp, không scipy,
không matplotlib, không pyproj. Xem `requirements.txt` để biết vì sao cái pin
`numpy==1.26.4` ở gốc repo không áp dụng ở đây.

## Nâng cấp thuật toán

```bash
cd /opt/vtx/path_planning && sudo git pull && sudo systemctl restart vtx-planner
```

Không build lại gì: service import `core.*` thẳng từ cây mã nguồn. Mỗi reply
mang `planner_version` (`git describe --always --dirty`) và `config_hash`, nên
client luôn phân biệt được hai đường bay khác nhau là do input khác hay do
phiên bản/cấu hình planner khác.

## Bản đồ nền

Tuỳ chọn. Thêm `--preloaded-map /opt/vtx/basemap.xml` vào `ExecStart`. Định
dạng ở `basemap.example.xml`.

**Ngữ nghĩa dễ hiểu ngược:** safezone của bản đồ nền được NỐI THÊM vào safezone
của request, và planner lấy HỢP của chúng. Thêm một safezone là **nới rộng**
vùng bay, không phải thu hẹp.

## Những gì service CHƯA làm

- **`time_budget_s` và `max_iterations` trong request bị bỏ qua.** Service dùng
  `config.TIME_BUDGET_S` / `config.MAX_ITERATIONS`. Reply mang
  `applied_time_budget_s` và `stats.max_iterations` là giá trị thật đã dùng.
- **Chỉ hệ toạ độ Oxy phẳng, mét.** Không WGS84.
- **Một request tại một thời điểm.** Bận thì trả `PLAN_BUSY`.

## Chẩn đoán

| triệu chứng | nguyên nhân thường gặp |
| --- | --- |
| Client không nhận reply nào | Sai `--domain-id`, hoặc discovery bị chặn. Kiểm tra log "sẵn sàng trên domain". |
| Mọi reply là `PLAN_INVALID_REQUEST` | `idl_version` lệch: client và service build từ hai bản IDL khác nhau. |
| `PLAN_INVALID_REQUEST` kèm "preloaded map" | Client đặt `use_preloaded_map` nhưng service khởi động không có `--preloaded-map`. |
| `PLAN_TIMEOUT` lặp lại | Bản đồ quá khó cho `config.TIME_BUDGET_S`, hoặc máy quá tải. Xem `stats.budget_bound` trên các reply thành công. |
| Đường bay đúng độ dài nhưng sai hướng 90 độ | Quy ước phương vị. Trên dây LUÔN là phương vị thật, thuận kim đồng hồ từ bắc, `+y` bắc. |
| Service treo cứng sau một thời gian chạy | Nghi ngờ đầu tiên: có ai đó đổi `PlanRunner` sang `fork` trần, hoặc đảo thứ tự `runner.start()` và khởi tạo DDS. Xem mục 3 của spec. |
| Reply thiếu mẫu tin với bản đồ lớn | Phân mảnh UDP; cần chỉnh cấu hình transport của binding DDS. |
```

- [ ] **Step 6: Chạy toàn bộ test và kiểm tra ranh giới**

```bash
python -m pytest -q service/tests/
python -m pytest -q tests/ 2>&1 | tail -3
git diff --stat main -- core/ render/ config.py
```
Expected: service toàn PASS; `tests/` vẫn `188 passed, 6 failed`; diff ranh giới rỗng.

- [ ] **Step 7: Xác minh trong một venv sạch**

```bash
python3.11 -m venv /tmp/vtx-venv
/tmp/vtx-venv/bin/pip install -q -r service/deploy/requirements.txt pytest
PYTHONPATH=.:service /tmp/vtx-venv/bin/python -m pytest -q service/tests/
```
Expected: mọi test PASS. Nếu có `ModuleNotFoundError`, service đã lỡ phụ thuộc vào một gói ngoài danh sách — thêm nó vào `requirements.txt` HOẶC bỏ chỗ dùng nó, đừng cài lén vào venv.

- [ ] **Step 8: Commit**

```bash
git add service/vtx_service/main.py service/deploy/
git commit -m "feat(service): service lifecycle, systemd unit, deployment guide"
```

---

## Hoàn tất

Xong Task 11, hệ thống gọi publish một `VtxPathPlanRequest` và nhận lại
`VtxPathPlanReply` mang đường bay đầy đủ `O..T`, kèm nhận dạng phiên bản và cấu
hình đã sinh ra nó. Service tự khởi động cùng máy và cập nhật theo thuật toán
bằng `git pull && systemctl restart`.

Thuật toán chưa bị sửa một dòng nào, và ba cơ chế của spec giữ cho nó tiếp tục
như vậy: test ranh giới, test hợp đồng khoá, và test tương đương bit-identical.

Hai việc để sau, đã biết và cố tình hoãn: đóng gói phân phối, và tôn trọng
`time_budget_s` khi thuật toán nhận được nó như một tham số thật.
