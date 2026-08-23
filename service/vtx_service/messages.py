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
    # RESERVED - không đường mã nào sinh ra giá trị này (R23, xem spec mục 3
    # và 6). Vòng phục vụ tuần tự trên một reader KEEP_ALL, nên một request
    # đến khi service đang bận được DDS xếp hàng và trả lời sau, không bị từ
    # chối. Giữ số 8 để không đánh số lại các giá trị đứng trước.
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
