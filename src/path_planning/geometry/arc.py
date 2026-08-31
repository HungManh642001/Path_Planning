"""Hình học kiểm tra trạng thái tiếp xúc biên đường tròn.

Module cung cấp các hàm kiểm tra điểm nằm trên biên đường tròn vật cản
trong ngưỡng dung sai hình học.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from path_planning.types import (
    CircleGeometry,
    Point,
)


def is_point_on_circle_boundary(
    point: Point, center: Point, radius: float, tol: float
) -> bool:
    """Kiểm tra một điểm có nằm trên biên đường tròn trong ngưỡng dung sai không.

    Args:
        point: Tọa độ điểm cần kiểm tra (x, y).
        center: Tọa độ tâm đường tròn (cx, cy).
        radius: Bán kính đường tròn (m).
        tol: Dung sai khoảng cách (m).

    Returns:
        True nếu khoảng cách từ điểm tới tâm sai lệch so với bán kính nhỏ hơn tol.
    """
    return abs(math.hypot(point[0] - center[0], point[1] - center[1]) - radius) < tol


def is_point_on_any_circle_boundary(
    point: Point, circles: Sequence[CircleGeometry], tol: float
) -> bool:
    """Kiểm tra điểm có nằm trên biên của bất kỳ đường tròn nào trong danh sách không.

    Args:
        point: Tọa độ điểm cần kiểm tra (x, y).
        circles: Danh sách các đường tròn dạng ((cx, cy), radius).
        tol: Dung sai khoảng cách (m).

    Returns:
        True nếu điểm nằm trên biên của ít nhất một đường tròn; ngược lại False.
    """
    return any(
        is_point_on_circle_boundary(point, center, radius, tol)
        for center, radius in circles
    )
