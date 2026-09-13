"""Tạo đường bay nhiệm vụ hoàn chỉnh: ``O -> W_1 ... W_{n-1} -> T``.

Thuật toán A* chỉ tìm chuỗi waypoint bên trong. Điểm cất cánh O và đích T
là ràng buộc cố định: W_1 cách O đoạn thẳng L0, W_{n-1} cách T đoạn thẳng DSS.
Hàm này hoàn thiện chuỗi waypoint đầy đủ phục vụ kiểm định và vẽ đồ thị.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from path_planning.types import PlannerState, PreprocessedScenario


def full_mission_path(
    path: Sequence[PlannerState], preprocessed: PreprocessedScenario | None
) -> list[PlannerState]:
    """Thêm điểm cất cánh O vào đầu và mục tiêu T vào cuối chuỗi waypoint.

    Các điểm đầu cuối nếu đã hiện diện (trong phạm vi 1 m) sẽ không bị lặp lại,
    do đó gọi hàm này nhiều lần vẫn an toàn.

    Args:
        path: Chuỗi waypoint bên trong tìm được dạng các cặp (waypoint, heading).
        preprocessed: Kịch bản tiền xử lý cung cấp ``start_pos``/``goal_pos``
            và hướng bay tại các đầu mút. Nếu ``None`` hoặc thiếu các khóa này,
            hàm trả về chuỗi đường bay ban đầu.

    Returns:
        Đường bay nhiệm vụ hoàn chỉnh bao gồm cả hai điểm đầu cuối O và T.
    """
    waypoints = list(path)
    if preprocessed is None:
        return waypoints

    takeoff = preprocessed.get("start_pos")
    target = preprocessed.get("goal_pos")
    start_heading = preprocessed.get("start_heading", 0.0)
    goal_heading = preprocessed.get("goal_heading", 0.0)

    if takeoff is not None and (
        not waypoints or math.dist(takeoff, waypoints[0][0]) > 1.0
    ):
        waypoints.insert(0, ((takeoff[0], takeoff[1]), start_heading))
    if target is not None and (
        not waypoints or math.dist(target, waypoints[-1][0]) > 1.0
    ):
        if goal_heading is None:
            # Chế độ đích tự do (free-goal) để goal_heading là None; hướng tiếp cận
            # khi đó là phương vị của chặng bay cuối cùng tiến vào T.
            last = waypoints[-1][0] if waypoints else None
            goal_heading = (
                math.atan2(target[1] - last[1], target[0] - last[0]) if last else 0.0
            )
        waypoints.append(((target[0], target[1]), goal_heading))
    return waypoints
