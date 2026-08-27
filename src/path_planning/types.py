"""Hệ thống định nghĩa kiểu dữ liệu (Types) dùng chung trong toàn bộ pipeline.

Bao gồm các cấu trúc hình học cơ bản (Point, PolygonCoords, CircleGeometry),
các cấu trúc dữ liệu TypedDict cho kịch bản (Scenario, PreprocessedScenario)
và kết quả lập kế hoạch đường bay (PlanResult, SearchStats).
Quy ước đơn vị đo: Khoảng cách tính bằng mét (m), góc tính bằng radian (rad).
"""

from __future__ import annotations

import sys
from typing import Literal, TypedDict


if sys.version_info >= (3, 11):
    from typing import NotRequired
else:
    from typing_extensions import NotRequired

# --- Kiểu hình học nguyên thủy ------------------------------------------------

Point = tuple[float, float]
"""Tọa độ 2D phẳng (x, y) tính bằng mét."""

PolygonCoords = list[Point]
"""Danh sách các đỉnh (x, y) liên tiếp tạo thành đa giác."""

CircleGeometry = tuple[Point, float]
"""Hình học đường tròn dạng ((cx, cy), r) tính bằng mét."""

MapBounds = tuple[float, float]
"""Kích thước khung hình chữ nhật bản đồ (chiều_rộng, chiều_cao)."""

PlannerState = tuple[Point, float]
"""Trạng thái tìm kiếm: ``(tọa_độ, hướng_bay)``. Đường bay là danh sách trạng thái."""

LatticeKey = tuple[int, int, int]
"""Khóa rời rạc hóa trạng thái (x_grid, y_grid, heading_grid) để băm và loại trùng."""

Topology = Literal["random", "center_cluster", "wall_block"]
"""Chiến lược phân bố chướng ngại vật: ngẫu nhiên, cụm trung tâm hoặc chắn ngang."""

WrapSense = Literal[-1, 1]
"""Chiều di chuyển bám cung tròn: +1 là ngược chiều (CCW), -1 là thuận chiều (CW)."""

RidingSense = Literal[-1, 0, 1]
"""Chiều bám cung tròn: +1 (CCW), -1 (CW), hoặc 0 nếu không bám đường tròn."""


# --- Chướng ngại vật ----------------------------------------------------------


class CircleObstacle(TypedDict):
    """Bản ghi dữ liệu chướng ngại vật hình tròn.

    Attributes:
        type: Định danh kiểu chướng ngại vật, luôn là 'circle'.
        center: Tọa độ tâm hình tròn (x, y) tính bằng mét.
        radius: Bán kính hình tròn tính bằng mét.
    """

    type: Literal["circle"]
    center: Point
    radius: float


class PolygonObstacle(TypedDict):
    """Bản ghi dữ liệu chướng ngại vật hình đa giác (đảo).

    Attributes:
        type: Định danh kiểu chướng ngại vật, luôn là 'polygon'.
        polygon: Danh sách tọa độ các đỉnh (x, y) của đa giác.
    """

    type: Literal["polygon"]
    polygon: PolygonCoords


Obstacle = CircleObstacle | PolygonObstacle
"""Kiểu dữ liệu hợp nhất (Union) của các loại chướng ngại vật."""


# --- Kịch bản nhiệm vụ --------------------------------------------------------


class ScenarioConfig(TypedDict, total=False):
    """Cấu hình đầu vào để sinh kịch bản ngẫu nhiên.

    Attributes:
        start: Tọa độ điểm cất cánh (x, y) tính bằng mét.
        start_heading: Hướng cất cánh ban đầu (rad).
        goal: Tọa độ điểm mục tiêu đích (x, y) tính bằng mét.
        goal_heading: Hướng tiếp cận mục tiêu yêu cầu (rad), hoặc None nếu tự do.
        num_islands: Số lượng đảo đa giác cần tạo.
        num_dynamic_obstacles: Số lượng chướng ngại vật tròn cần tạo.
        map_bounds: Kích thước khung bản đồ (rộng, cao) tính bằng mét.
        safezones: Danh sách các vùng bay an toàn (đa giác).
        topology: Kiểu phân bố chướng ngại vật ('random', 'center_cluster', ...).
        seed: Hạt giống ngẫu nhiên để tái lập kết quả.
    """

    start: Point
    start_heading: float
    goal: Point
    goal_heading: float | None
    num_islands: int
    num_dynamic_obstacles: int
    map_bounds: MapBounds
    safezones: list[PolygonCoords] | None
    topology: Topology
    seed: int | None


class Scenario(TypedDict):
    """Kịch bản nhiệm vụ hoàn chỉnh đầu vào cho bộ tiền xử lý.

    Attributes:
        start: Tọa độ điểm xuất phát (x, y) tính bằng mét.
        start_heading: Hướng xuất phát ban đầu (rad).
        goal: Tọa độ điểm mục tiêu đích (x, y) tính bằng mét.
        goal_heading: Hướng tiếp cận mục tiêu yêu cầu (rad), hoặc None nếu tự do.
        map_bounds: Kích thước khung bản đồ (chiều rộng, chiều cao).
        safezones: Danh sách các vùng an toàn cho phép bay, hoặc None nếu toàn map.
        islands: Danh sách tọa độ các đảo đa giác.
        dynamic_obstacles: Danh sách các chướng ngại vật tròn dạng ((cx, cy), r).
        obstacles: Danh sách hợp nhất tất cả các chướng ngại vật trong kịch bản.
    """

    start: Point
    start_heading: float
    goal: Point
    goal_heading: float | None
    map_bounds: MapBounds
    safezones: list[PolygonCoords] | None
    islands: list[PolygonCoords]
    dynamic_obstacles: list[CircleGeometry]
    obstacles: list[Obstacle]


# --- Kết quả tìm kiếm ---------------------------------------------------------


class SearchStats(TypedDict):
    """Thống kê chi tiết về hiệu năng quá trình tìm kiếm A*.

    Attributes:
        iterations: Số bước lặp mở rộng nút trong vòng lặp tìm kiếm A*.
        time_budget_s: Hạn mức thời gian đã áp dụng cho lần tìm kiếm này (giây).
        is_budget_bound: True nếu thuật toán bị dừng do chạm hạn mức thời gian.
        open_set_size: Số lượng nút trạng thái còn lại trong hàng đợi ưu tiên.
        is_search_failed: True nếu thuật toán kết thúc mà không tìm thấy đường bay.
        closed_set_size: Tổng số lượng nút trạng thái đã khám phá trong Closed set.
    """

    iterations: int
    time_budget_s: float
    is_budget_bound: bool
    open_set_size: int
    is_search_failed: bool
    closed_set_size: NotRequired[int]


class PlanResultView(TypedDict):
    """Giao diện đọc kết quả tối giản cho các module trực quan hóa.

    Attributes:
        path: Danh sách các waypoint (tọa_độ, hướng_bay), hoặc None nếu thất bại.
        is_success: True nếu tìm thấy đường bay hợp lệ thỏa mãn mọi ràng buộc.
        failure_reason: Mô tả nguyên nhân thất bại nếu is_success là False.
        stats: Thống kê hiệu năng quá trình tìm kiếm.
    """

    path: list[PlannerState] | None
    is_success: bool
    failure_reason: str | None
    stats: SearchStats


class PlanResult(TypedDict):
    """Kết quả hoàn chỉnh trả về từ thuật toán lập kế hoạch đường bay.

    Attributes:
        path: Danh sách các waypoint (tọa_độ, hướng_bay), hoặc None nếu thất bại.
        is_success: True nếu tìm thấy đường bay hợp lệ thỏa mãn mọi ràng buộc.
        failure_reason: Mô tả nguyên nhân thất bại nếu is_success là False.
        stats: Thống kê hiệu năng quá trình tìm kiếm.
        planner: Thể hiện đối tượng planner thực thi.
    """

    path: list[PlannerState] | None
    is_success: bool
    failure_reason: str | None
    stats: SearchStats
    planner: object


# --- Kịch bản tiền xử lý ------------------------------------------------------


class StartState(TypedDict):
    """Trạng thái xuất phát đầu tiên W_1 sau khi bay thẳng ổn định sau cất cánh.

    Attributes:
        waypoint: Tọa độ điểm W_1 (x, y) tính bằng mét.
        heading: Hướng cất cánh ban đầu (rad).
        straight_length: Chiều dài đoạn bay thẳng cất cánh l_1 (m, >= L0).
        distance_from_origin: Khoảng cách từ điểm cất cánh O tới W_1 (m).
    """

    waypoint: Point
    heading: float
    straight_length: NotRequired[float]
    distance_from_origin: NotRequired[float]


class GoalState(TypedDict):
    """Trạng thái trước đích W_{n-1} phục vụ tiếp cận khoá mục tiêu.

    Attributes:
        waypoint: Tọa độ điểm W_{n-1} (x, y) tính bằng mét.
        heading: Hướng tiếp cận mục tiêu (rad), hoặc None nếu không ràng buộc.
        engagement_distance: Chiều dài đoạn thẳng khoá mục tiêu (m, >= DSS).
        distance_to_target: Khoảng cách từ W_{n-1} tới điểm mục tiêu đích T (m).
    """

    waypoint: Point
    heading: float | None
    engagement_distance: NotRequired[float]
    distance_to_target: NotRequired[float]


class InflatedObstacleSets(TypedDict):
    """Tập hợp các chướng ngại vật đã được giãn nở theo khoảng an toàn safe_margin.

    Attributes:
        inflated_obstacles: Danh sách bản ghi chướng ngại vật đã giãn nở.
        circle_obstacles: Danh sách chướng ngại vật tròn đã giãn nở.
        polygon_obstacles: Danh sách đa giác chướng ngại vật đã giãn nở.
    """

    inflated_obstacles: list[Obstacle]
    circle_obstacles: list[CircleGeometry]
    polygon_obstacles: list[PolygonCoords]


class PreprocessedScenario(TypedDict):
    """Kịch bản đã được tiền xử lý hoàn chỉnh cho thuật toán tìm kiếm A*.

    Attributes:
        start_state: Trạng thái xuất phát tính toán W_1.
        goal_state: Trạng thái trước đích tính toán W_{n-1}.
        turn_radius: Bán kính quay vòng tối thiểu R (m).
        alpha_max_rad: Góc ngoặt tối đa cho phép tại mỗi góc rẽ (rad).
        circle_obstacles: Danh sách chướng ngại vật tròn đã giãn nở.
        polygon_obstacles: Danh sách đa giác chướng ngại vật đã giãn nở.
        safezones: Danh sách vùng an toàn cho phép bay (nếu có).
        map_bounds: Kích thước giới hạn bản đồ (nếu có).
        start_pos: Tọa độ điểm cất cánh gốc O.
        goal_pos: Tọa độ điểm mục tiêu đích gốc T.
        start_heading: Hướng cất cánh ban đầu (rad).
        goal_heading: Hướng tiếp cận mục tiêu (rad), hoặc None nếu tự do.
        safe_margin: Khoảng đệm an toàn mở rộng chướng ngại vật (m).
        obstacles: Danh sách toàn bộ chướng ngại vật đã giãn nở.
        raw_circle_obstacles: Danh sách chướng ngại vật tròn gốc chưa giãn nở.
        raw_polygon_obstacles: Danh sách đa giác chướng ngại vật gốc chưa giãn nở.
        islands: Danh sách các đảo đa giác gốc.
        dynamic_obstacles: Danh sách các chướng ngại vật tròn gốc.
    """

    start_state: StartState
    goal_state: GoalState
    turn_radius: float
    alpha_max_rad: float
    circle_obstacles: list[CircleGeometry]
    polygon_obstacles: list[PolygonCoords]
    safezones: list[PolygonCoords] | None
    map_bounds: MapBounds | None
    start_pos: NotRequired[Point]
    goal_pos: NotRequired[Point]
    start_heading: NotRequired[float]
    goal_heading: NotRequired[float | None]
    safe_margin: NotRequired[float]
    obstacles: NotRequired[list[Obstacle]]
    raw_circle_obstacles: NotRequired[list[CircleGeometry]]
    raw_polygon_obstacles: NotRequired[list[PolygonCoords]]
    islands: NotRequired[list[PolygonCoords]]
    dynamic_obstacles: NotRequired[list[CircleGeometry]]
