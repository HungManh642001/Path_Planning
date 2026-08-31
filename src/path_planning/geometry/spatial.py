"""Các hàm tiện ích hình học không gian 2D cho bài toán lập kế hoạch đường bay.

Bao gồm tính khoảng cách Euclid, góc phương vị, khoảng cách điểm - đoạn thẳng,
giãn nở đa giác, rời rạc hóa trạng thái ô lưới và tìm tiếp điểm đường tròn.
Đơn vị tính: khoảng cách bằng mét (m), góc bằng radian (rad).
"""
# pyright: reportUnnecessaryIsInstance=false, reportUnknownVariableType=false
# pyright: reportUnknownArgumentType=false, reportUnknownMemberType=false
# pyright: reportUnknownLambdaType=false

from __future__ import annotations

import math
from collections.abc import Sequence

from shapely.geometry import MultiPolygon, Polygon

from path_planning import config
from path_planning.search.state import state_to_tuple as state_to_tuple
from path_planning.types import PlannerState, Point, PolygonCoords


def distance(p1: Point, p2: Point) -> float:
    """Tính khoảng cách Euclid giữa hai điểm 2D.

    Args:
        p1: Tọa độ điểm thứ nhất (x, y).
        p2: Tọa độ điểm thứ hai (x, y).

    Returns:
        Khoảng cách tính bằng mét.
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def angle_to_heading(p1: Point, p2: Point) -> float:
    """Tính góc hướng bay (phương vị) từ điểm p1 đến điểm p2.

    Args:
        p1: Điểm xuất phát (gốc).
        p2: Điểm đích đến.

    Returns:
        Góc phương vị tính bằng radian so với trục Ox dương.
    """
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


def angle_diff(a: float, b: float) -> float:
    """Tính độ lệch góc có dấu nhỏ nhất giữa hai góc a và b.

    Args:
        a: Góc bị trừ (rad).
        b: Góc trừ (rad).

    Returns:
        Độ lệch góc chuẩn hóa trong khoảng [-pi, pi].
    """
    return math.atan2(math.sin(a - b), math.cos(a - b))


def point_to_line_distance(point: Point, line_start: Point, line_end: Point) -> float:
    """Tính khoảng cách vuông góc ngắn nhất từ một điểm đến đoạn thẳng.

    Args:
        point: Tọa độ điểm cần tính (x, y).
        line_start: Tọa độ đầu mút thứ nhất của đoạn thẳng.
        line_end: Tọa độ đầu mút thứ hai của đoạn thẳng.

    Returns:
        Khoảng cách ngắn nhất tính bằng mét.
    """
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return distance(point, line_start)
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    return distance(point, (x1 + t * dx, y1 + t * dy))


def _exterior_coords(polygon: Polygon) -> PolygonCoords:
    """Trích xuất các đỉnh của vòng đa giác ngoài (loại bỏ đỉnh trùng cuối)."""
    return [(float(x), float(y)) for x, y in polygon.exterior.coords[:-1]]


def inflate_polygon(polygon_coords: PolygonCoords, inflation: float) -> PolygonCoords:
    """Giãn nở đa giác ra ngoài một khoảng inflation mét.

    Sử dụng kiểu nối mitre để giữ góc sắc nhọn, tạo ít đỉnh dẫn đường hơn.

    Args:
        polygon_coords: Danh sách các đỉnh của đa giác ban đầu.
        inflation: Khoảng cách giãn nở tính bằng mét (<= 0 sẽ giữ nguyên).

    Returns:
        Danh sách các đỉnh của đa giác mới sau khi giãn nở.
    """
    if inflation <= 0.0:
        return list(polygon_coords)
    expanded = Polygon(polygon_coords).buffer(
        inflation, join_style="mitre", mitre_limit=config.POLYGON_MITRE_LIMIT
    )
    if isinstance(expanded, Polygon):
        return _exterior_coords(expanded)
    if isinstance(expanded, MultiPolygon):
        largest = max(expanded.geoms, key=lambda p: p.area)
        return _exterior_coords(largest)
    return polygon_coords


def circle_tangent_points(point: Point, center: Point, radius: float) -> list[Point]:
    """Tìm 2 tiếp điểm trên đường tròn kẻ từ một điểm bên ngoài.

    Args:
        point: Tọa độ điểm bên ngoài (x, y).
        center: Tọa độ tâm đường tròn (cx, cy).
        radius: Bán kính đường tròn tính bằng mét.

    Returns:
        Danh sách 2 tiếp điểm (x, y), hoặc rỗng nếu điểm nằm bên trong đường tròn.
    """
    px, py = point
    cx, cy = center
    dx, dy = px - cx, py - cy
    d2 = dx * dx + dy * dy
    if d2 <= radius * radius + 1e-9:
        return []
    d = math.sqrt(d2)
    theta = math.atan2(dy, dx)
    alpha = math.acos(radius / d)
    return [
        (cx + radius * math.cos(theta + alpha), cy + radius * math.sin(theta + alpha)),
        (cx + radius * math.cos(theta - alpha), cy + radius * math.sin(theta - alpha)),
    ]


def calculate_polyline_length(
    path: Sequence[PlannerState] | Sequence[Point],
) -> float:
    """Tính tổng chiều dài đoạn thẳng đa giác qua các waypoints.

    Args:
        path: Danh sách điểm dạng [(x, y), ...] hoặc [((x, y), heading), ...].

    Returns:
        Tổng khoảng cách Euclid giữa các waypoint liên tiếp (m).
    """
    if len(path) < 2:
        return 0.0
    pts: list[Point] = [p[0] if isinstance(p[0], tuple) else p for p in path]  # type: ignore[misc]
    return sum(distance(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def calculate_dubins_path_length(
    path: Sequence[PlannerState] | Sequence[Point], turn_radius: float
) -> float:
    """Tính tổng chiều dài quỹ đạo bay thực tế gồm các đoạn thẳng và cung lượn Dubins.

    Tại mỗi góc rẽ W_i (i=1..n-1) giữa 2 đoạn thẳng, đoạn thẳng đi qua đỉnh được thay
    bằng cung tròn bán kính R, làm chiều dài tại góc rẽ rút ngắn đi một khoảng:
    Delta L_i = 2 * R * tan(|alpha_i| / 2) - R * |alpha_i|.

    Args:
        path: Danh sách điểm dạng [(x, y), ...] hoặc [((x, y), heading), ...].
        turn_radius: Bán kính quay tối thiểu R (m).

    Returns:
        Tổng chiều dài quỹ đạo bay thực tế (m).
    """
    if len(path) < 2:
        return 0.0
    pts: list[Point] = [p[0] if isinstance(p[0], tuple) else p for p in path]  # type: ignore[misc]
    if len(pts) == 2:
        return distance(pts[0], pts[1])

    poly_len = sum(distance(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    shortening = 0.0
    for i in range(1, len(pts) - 1):
        p_prev, p_curr, p_next = pts[i - 1], pts[i], pts[i + 1]
        h_in = angle_to_heading(p_prev, p_curr)
        h_out = angle_to_heading(p_curr, p_next)
        alpha = abs(angle_diff(h_out, h_in))
        if alpha > 1e-9:
            shortening += (
                2.0 * turn_radius * math.tan(alpha / 2.0) - turn_radius * alpha
            )

    return max(0.0, poly_len - shortening)
