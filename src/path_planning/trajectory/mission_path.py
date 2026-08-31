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

    Endpoints already present (within 1 m) are not duplicated, so calling this
    twice is harmless.

    Args:
        path: The searched interior waypoints as ``(waypoint, heading)`` pairs.
        preprocessed: The prepared scenario supplying ``start_pos``/``goal_pos``
            and the endpoint headings. ``None``, or a dict without those keys,
            returns the path unchanged.

    Returns:
        The full mission path, endpoints included.
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
            # Free-goal mode leaves goal_heading None; the arrival heading is
            # then the bearing of the final leg into T.
            last = waypoints[-1][0] if waypoints else None
            goal_heading = (
                math.atan2(target[1] - last[1], target[0] - last[0]) if last else 0.0
            )
        waypoints.append(((target[0], target[1]), goal_heading))
    return waypoints
