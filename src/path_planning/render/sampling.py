"""Bộ kết xuất quỹ đạo bay dạng chuỗi điểm tọa độ 2D.

Chuyển đổi danh sách trạng thái ``[(waypoint, heading), ...]`` của planner
thành danh sách phẳng các điểm ``(x, y)`` để vẽ đồ thị:
- ``'straight'``: Nối các waypoint bằng đoạn thẳng.
- ``'dubins'``: Mô phỏng đường bay thực tế với các cung lượn fillet arc bán kính R.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Literal, TypedDict

from path_planning.trajectory import mission_path as mission_path
from path_planning.types import PlannerState, Point, PreprocessedScenario


# Số chẵn -> một điểm mẫu rơi chính xác vào waypoint (trung điểm cung lượn)
_ARC_SAMPLES = 24

RenderMode = Literal["straight", "dubins"]
"""Chế độ kết xuất quỹ đạo bay của hàm :func:`sample_trajectory`."""


class TurnMarker(TypedDict):
    """Thông tin vị trí tiếp điểm bắt đầu, đỉnh rẽ và kết thúc của cung lượn.

    Attributes:
        start: Tiếp điểm vào, nơi cung lượn rời khỏi đoạn bay đến.
        mid: Waypoint đỉnh rẽ mà cung lượn đối xứng qua.
        end: Tiếp điểm ra, nơi cung lượn nhập lại vào đoạn bay đi.
        angle_deg: Góc đổi hướng có dấu; giá trị dương là rẽ trái / ngược chiều
            kim đồng hồ.
    """

    start: Point
    mid: Point
    end: Point
    angle_deg: float


def sample_trajectory(
    path: Sequence[PlannerState],
    turn_radius: float,
    mode: RenderMode = "dubins",
    step: float | None = None,
) -> list[Point]:
    """Lấy mẫu quỹ đạo bay thành chuỗi điểm dày đặc để vẽ đồ thị.

    Args:
        path: Danh sách waypoint dạng các cặp ``(waypoint, heading)``.
        turn_radius: Bán kính cung lượn fillet (m).
        mode: ``'straight'`` để nối thẳng các waypoint, ``'dubins'`` để bo tròn
            từng góc cua bên trong bằng một cung tiếp tuyến.
        step: Bước lấy mẫu dọc theo các đoạn thẳng (m); mặc định là
            ``turn_radius / 8``.

    Returns:
        Danh sách các điểm đường gấp khúc (polyline); rỗng nếu đường bay rỗng.
    """
    if not path:
        return []
    waypoints = [wp for wp, _ in path]
    if len(waypoints) == 1:
        return [waypoints[0]]
    if mode == "straight":
        return list(waypoints)
    if step is None:
        step = turn_radius / 8.0
    points, _ = _dubins_arc_path(waypoints, turn_radius, step)
    return points


def turn_markers(path: Sequence[PlannerState], turn_radius: float) -> list[TurnMarker]:
    """Xác định vị trí các điểm tiếp xúc của từng cung lượn dọc theo đường bay.

    Args:
        path: Danh sách waypoint dạng các cặp ``(waypoint, heading)``.
        turn_radius: Bán kính cung lượn fillet (m).

    Returns:
        Một marker cho mỗi góc rẽ, theo thứ tự đường bay. Các đoạn thẳng không
        sinh marker, và đường bay có ít hơn 3 waypoint sẽ không có góc rẽ nào.
    """
    waypoints = [wp for wp, _ in path]
    if len(waypoints) < 3:
        return []
    _, turns = _dubins_arc_path(waypoints, turn_radius, turn_radius / 8.0)
    return turns


def build_full_path(
    result_path: Sequence[PlannerState], preprocessed: PreprocessedScenario | None
) -> list[PlannerState]:
    """Thêm điểm cất cánh O và đích T để đường bay bao phủ toàn bộ hành trình.

    Bí danh chuyển tiếp gọn cho
    :func:`path_planning.trajectory.mission_path.full_mission_path`, được giữ
    lại vì tầng hiển thị render, GUI và các bài test đã gọi tên này. Bộ lập kế
    hoạch kiểm định đường bay phát ra bằng CHÍNH hàm này: quỹ đạo vẽ ra và phán
    quyết của oracle phải cùng dựa trên một danh sách waypoint duy nhất.

    Args:
        result_path: Các waypoint bên trong do bộ lập kế hoạch tìm được.
        preprocessed: Kịch bản đã tiền xử lý cung cấp các điểm đầu cuối.

    Returns:
        Đường bay nhiệm vụ đầy đủ, bao gồm cả điểm xuất phát và đích.
    """
    return mission_path.full_mission_path(result_path, preprocessed)


# --------------------------------------------------------------------------
# Hình học nội bộ
# --------------------------------------------------------------------------


def _extend_straight(points: list[Point], target: Point, step: float) -> None:
    """Nối thêm các điểm mẫu dọc theo đoạn thẳng ``points[-1] -> target``.

    Điểm bắt đầu không bị lặp lại. Đoạn thẳng suy biến (< 1e-9 m) sẽ không thêm gì.
    """
    x0, y0 = points[-1]
    x1, y1 = target
    d = math.hypot(x1 - x0, y1 - y0)
    if d < 1e-9:
        return
    nseg = max(1, math.ceil(d / step))
    for k in range(1, nseg + 1):
        t = k / nseg
        points.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))


def _unit(a: Point, b: Point) -> Point:
    """Tính vector đơn vị từ a đến b, hoặc (0, 0) nếu hai điểm trùng nhau."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    return (dx / d, dy / d) if d > 0 else (0.0, 0.0)


def _dubins_arc_path(
    waypoints: Sequence[Point],
    turn_radius: float,
    step: float,
    arc_samples: int = _ARC_SAMPLES,
) -> tuple[list[Point], list[TurnMarker]]:
    """Tạo các đoạn thẳng và cung lượn tròn bo góc.

    Mỗi cung tiếp tuyến với cả hai chặng bay và đối xứng qua waypoint,
    đảm bảo góc hướng bay vào và ra được bảo toàn chính xác.

    Args:
        waypoints: Danh sách tọa độ góc rẽ theo thứ tự đường bay.
        turn_radius: Bán kính cung lượn fillet arc (m).
        step: Bước lấy mẫu dọc theo đoạn thẳng (m).
        arc_samples: Số mẫu sinh ra trên mỗi cung lượn.

    Returns:
        Cặp ``(points, turns)``: danh sách điểm tọa độ liên tục dày đặc và danh
        sách :class:`TurnMarker` ứng với từng góc rẽ.
    """
    points: list[Point] = [waypoints[0]]
    turns: list[TurnMarker] = []
    for i in range(1, len(waypoints) - 1):
        wp_prev, wp, wp_next = waypoints[i - 1], waypoints[i], waypoints[i + 1]
        u = _unit(wp_prev, wp)  # hướng chặng bay đến
        v = _unit(wp, wp_next)  # hướng chặng bay đi
        h_in = math.atan2(u[1], u[0])
        h_out = math.atan2(v[1], v[0])
        alpha = math.atan2(math.sin(h_out - h_in), math.cos(h_out - h_in))
        a_abs = abs(alpha)
        if a_abs < 1e-9:
            # không đổi hướng: bay thẳng tới waypoint
            _extend_straight(points, wp, step)
            continue
        # Độ lùi tiếp điểm t = R*tan(alpha/2) với bán kính R không đổi
        t = turn_radius * math.tan(a_abs / 2.0)
        s = 1.0 if alpha > 0 else -1.0
        start = (wp[0] - u[0] * t, wp[1] - u[1] * t)  # tiếp điểm vào cung lượn
        end = (wp[0] + v[0] * t, wp[1] + v[1] * t)  # tiếp điểm ra khỏi cung lượn
        n_in = (-u[1] * s, u[0] * s)  # pháp tuyến hướng vào tâm quay
        cx = start[0] + turn_radius * n_in[0]
        cy = start[1] + turn_radius * n_in[1]
        ang0 = math.atan2(start[1] - cy, start[0] - cx)
        _extend_straight(points, start, step)  # chặng bay thẳng dẫn vào góc rẽ
        for k in range(1, arc_samples + 1):
            a = ang0 + s * a_abs * (k / arc_samples)
            points.append(
                (cx + turn_radius * math.cos(a), cy + turn_radius * math.sin(a))
            )
        turns.append(
            {"start": start, "mid": wp, "end": end, "angle_deg": math.degrees(alpha)}
        )
    _extend_straight(points, waypoints[-1], step)  # chặng bay thẳng cuối cùng
    return points, turns
