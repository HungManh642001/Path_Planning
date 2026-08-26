"""Dịch một ``PlanRequest`` thành đúng dict ``Scenario`` mà pipeline tiêu thụ.

Đây là toàn bộ phần "dịch" của adapter. Nó không tính hình học và không biết gì
về search: mọi khoá lấy thẳng từ ``core.types.Scenario``, để một khoá mới bên đó
làm test hợp đồng đỏ chứ không làm production nổ.

Toạ độ đi qua NGUYÊN VẸN - chỉ có một hệ toạ độ, không có phép chiếu nào. Chỉ
góc được đổi quy ước.
"""

from __future__ import annotations

from typing import Any

from service.vtx_service.angles import bearing_deg_to_math_rad
from service.vtx_service.messages import PlanRequest


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
        {"type": "circle", "center": center, "radius": radius}
        for center, radius in circles
    )

    goal_heading = (
        None
        if request.is_goal_heading_free
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
