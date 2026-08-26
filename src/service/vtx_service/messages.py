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

IDL_VERSION = 2
"""Tăng khi bố cục struct đổi. Service từ chối request không khớp.

2: bỏ ``max_iterations`` khỏi ``SearchBudget`` và ``SearchStats``; đồng thời
``budget.time_budget_s`` bắt đầu được tôn trọng thật. Hai thay đổi này đi cùng
một lần tăng vì chúng là cùng một quyết định: thuật toán chỉ còn MỘT điều kiện
dừng, và nó là con số client gửi.
"""


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
    """Chướng ngại vật tròn.

    Attributes:
        center: The center.
        radius_m: The radius_m.
    """

    center: Point
    radius_m: float

    def __post_init__(self) -> None:
        """Validate values."""
        if not self.radius_m > 0.0:
            raise ValueError(f"radius_m phải dương, nhận {self.radius_m}")


@dataclass(frozen=True)
class VehicleLimits:
    """Năm tham số duy nhất tới được planner qua đường tham số hàm.

    Ánh xạ 1-1 sang tham số của ``core.preprocessing.prepare_scenario``. Mọi
    hằng số khác của planner là global và cố định lúc triển khai.

    Attributes:
        turn_radius_m: The turn_radius_m.
        l0_m: The l0_m.
        dss_m: The dss_m.
        safe_margin_m: The safe_margin_m.
        alpha_max_deg: The alpha_max_deg.
    """

    turn_radius_m: float
    l0_m: float
    dss_m: float
    safe_margin_m: float
    alpha_max_deg: float

    def __post_init__(self) -> None:
        """Validate limits."""
        for name in ("turn_radius_m", "l0_m", "dss_m", "alpha_max_deg"):
            value = getattr(self, name)
            if not value > 0.0:
                raise ValueError(f"{name} phải dương, nhận {value}")
        if self.safe_margin_m < 0.0:
            raise ValueError(f"safe_margin_m không được âm, nhận {self.safe_margin_m}")


@dataclass(frozen=True)
class SearchBudget:
    """Ngân sách search do client đề nghị.

    ``time_budget_s`` ĐƯỢC tôn trọng: nó đi thẳng vào thuật toán làm điều kiện
    dừng DUY NHẤT. ``<= 0`` (hoặc rác) nghĩa là "không đề nghị gì" và service
    dùng mặc định của mình; giá trị quá lớn bị kẹp xuống
    ``runtime.MAX_REQUEST_TIME_BUDGET_S``. Reply luôn mang
    ``applied_time_budget_s`` là giá trị THẬT đã dùng, nên client biết đề nghị
    của mình được nhận nguyên vẹn hay đã bị thay.

    Không còn trần theo số vòng lặp: thuật toán bỏ ``MAX_ITERATIONS`` vì một
    con số vòng lặp không phải đại lượng người vận hành suy luận được, và hai
    điều kiện dừng độc lập khiến một lần search có thể kết thúc vì lý do mà
    reply không hề nói ra.

    Attributes:
        time_budget_s: The time_budget_s.
    """

    time_budget_s: float


@dataclass(frozen=True)
class PlanRequest:
    """Một mission cần lập kế hoạch.

    Attributes:
        request_id: The request_id.
        idl_version: The idl_version.
        start: The start.
        start_heading_deg: The start_heading_deg.
        goal: The goal.
        goal_heading_deg: The goal_heading_deg.
        goal_heading_free: The goal_heading_free.
        islands: The islands.
        dynamic_obstacles: The dynamic_obstacles.
        safezones: The safezones.
        use_preloaded_map: The use_preloaded_map.
        limits: The limits.
        budget: The budget.
    """

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
        """Validate request ID."""
        if len(self.request_id) != 16:
            raise ValueError(
                f"request_id phải đúng 16 byte, nhận {len(self.request_id)}"
            )


@dataclass(frozen=True)
class Waypoint:
    """Một điểm trên đường bay trả về.

    Attributes:
        position: The position.
        heading_deg: The heading_deg.
    """

    position: Point
    heading_deg: float


@dataclass(frozen=True)
class SearchStats:
    """Bộ đếm mô tả một lần chạy search.

    ``budget_bound`` là trường hạng nhất chứ không phải chi tiết ẩn: planner cắt
    theo đồng hồ, nên cùng một request trên máy tải nặng có thể ra đường bay
    khác. Che giấu điều đó khiến client tin vào một sự đảm bảo không tồn tại.

    Attributes:
        iterations: The iterations.
        open_set_size: The open_set_size.
        search_failed: The search_failed.
        budget_bound: The budget_bound.
    """

    iterations: int
    open_set_size: int
    search_failed: bool
    budget_bound: bool


@dataclass(frozen=True)
class PlanReply:
    """Kết quả trả về client.

    Attributes:
        request_id: The request_id.
        idl_version: The idl_version.
        status: The status.
        detail: The detail.
        waypoints: The waypoints.
        path_length_m: The path_length_m.
        plan_wall_time_s: The plan_wall_time_s.
        applied_time_budget_s: The applied_time_budget_s.
        stats: The stats.
        planner_version: The planner_version.
        config_hash: The config_hash.
    """

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
        """Check if status is OK.

        Returns:
            bool: True if OK, False otherwise.
        """
        return self.status is PlanStatus.OK
