"""Hình học thuần túy cho các trạng thái bám cung tròn và cơ động quanh đường tròn.

Bao gồm nhận diện trạng thái bám biên, tính góc tiếp tuyến, tiếp tuyến chung bitangent,
điểm rời tiếp tuyến, mở rộng cung tròn thành đa giác ngoại tiếp và hình học hình quạt.
Quy ước chiều quay: +1 là ngược chiều (CCW), -1 là thuận chiều (CW).
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from path_planning.geometry import spatial
from path_planning.types import (
    CircleGeometry,
    PlannerState,
    Point,
    PolygonCoords,
    RidingSense,
    WrapSense,
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


def riding_sense(
    point: Point,
    heading: float,
    center: Point,
    radius: float,
    *,
    pos_tol: float = 1.0,
    ang_tol: float = 8.72e-3,
) -> RidingSense:
    """Xác định trạng thái có bay bám theo biên đường tròn hay không và chiều bám.

    Args:
        point: Tọa độ vị trí trạng thái (x, y).
        heading: Hướng bay của trạng thái (rad).
        center: Tọa độ tâm đường tròn (cx, cy).
        radius: Bán kính đường tròn (m).
        pos_tol: Dung sai khoảng cách tới biên đường tròn (m).
        ang_tol: Dung sai góc tiếp tuyến với phương pháp tuyến.

    Returns:
        Chiều bám +1 (CCW) hoặc -1 (CW), hoặc 0 nếu không bám biên.
    """
    dx, dy = point[0] - center[0], point[1] - center[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-9 or abs(dist - radius) > pos_tol:
        return 0
    ux, uy = dx / dist, dy / dist
    hx, hy = math.cos(heading), math.sin(heading)
    if abs(ux * hx + uy * hy) > ang_tol:
        return 0
    return 1 if (ux * hy - uy * hx) > 0 else -1


def tangent_heading(point: Point, center: Point, sense: WrapSense) -> float:
    """Tính góc hướng bay tiếp tuyến tại một điểm trên biên đường tròn.

    Args:
        point: Tọa độ điểm trên biên đường tròn (x, y).
        center: Tọa độ tâm đường tròn (cx, cy).
        sense: Chiều di chuyển quanh đường tròn (+1 hoặc -1).

    Returns:
        Góc hướng bay tiếp tuyến tính bằng radian.
    """
    return math.atan2(sense * (point[0] - center[0]), -sense * (point[1] - center[1]))


def arc_angle(start: Point, end: Point, center: Point, sense: WrapSense) -> float:
    """Tính góc quét trên cung tròn từ điểm bắt đầu tới kết thúc.

    Args:
        start: Điểm bắt đầu trên biên đường tròn.
        end: Điểm kết thúc trên biên đường tròn.
        center: Tọa độ tâm đường tròn.
        sense: Chiều di chuyển trên cung tròn.

    Returns:
        Góc quét tính bằng radian trong khoảng [0, 2*pi).
    """
    a0 = math.atan2(start[1] - center[1], start[0] - center[0])
    a1 = math.atan2(end[1] - center[1], end[0] - center[0])
    return (sense * (a1 - a0)) % (2.0 * math.pi)


def departure_point(
    target: Point, center: Point, radius: float, sense: WrapSense
) -> Point | None:
    """Tìm điểm rời trên biên đường tròn để bay thẳng tới target tiếp tuyến mượt mà.

    Args:
        target: Tọa độ mục tiêu đích đến bên ngoài (x, y).
        center: Tọa độ tâm đường tròn (cx, cy).
        radius: Bán kính đường tròn (m).
        sense: Chiều bám cung tròn (+1 hoặc -1).

    Returns:
        Tọa độ điểm rời (x, y), hoặc None nếu target nằm bên trong đường tròn.
    """
    for dep in spatial.circle_tangent_points(target, center, radius):
        nx = (dep[0] - center[0]) / radius
        ny = (dep[1] - center[1]) / radius
        if (-sense * ny) * (target[0] - dep[0]) + (sense * nx) * (
            target[1] - dep[1]
        ) > 0:
            return dep
    return None


def bitangent_departures(
    c1: Point, r1: float, c2: Point, r2: float, sense: WrapSense
) -> list[tuple[Point, Point]]:
    """Tìm các đoạn thẳng tiếp tuyến chung giữa hai đường tròn theo chiều bám.

    Args:
        c1: Tâm đường tròn thứ nhất.
        r1: Bán kính đường tròn thứ nhất (m).
        c2: Tâm đường tròn thứ hai.
        r2: Bán kính đường tròn thứ hai (m).
        sense: Chiều bám trên đường tròn 1.

    Returns:
        Danh sách các cặp (điểm_rời_c1, điểm_đến_c2).
    """
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return []
    ux, uy = dx / d, dy / d
    out: list[tuple[Point, Point]] = []
    for sigma in (1.0, -1.0):
        k = (r1 - sigma * r2) / d
        if abs(k) > 1.0:
            continue
        root = math.sqrt(max(0.0, 1.0 - k * k))
        for pm in (1.0, -1.0):
            nx = k * ux - pm * root * uy
            ny = k * uy + pm * root * ux
            dep = (c1[0] + r1 * nx, c1[1] + r1 * ny)
            arr = (c2[0] + sigma * r2 * nx, c2[1] + sigma * r2 * ny)
            tx, ty = arr[0] - dep[0], arr[1] - dep[1]
            if math.hypot(tx, ty) < 1e-6:
                continue
            if (-sense * ny) * tx + (sense * nx) * ty > 0:
                out.append((dep, arr))
    return out


def arc_waypoints(
    center: Point,
    radius: float,
    start_pt: Point,
    dphi: float,
    sense: WrapSense,
    theta_max_rad: float,
) -> list[PlannerState]:
    """Mở rộng cung tròn biên thành các đỉnh đa giác ngoại tiếp.

    Args:
        center: Tọa độ tâm đường tròn.
        radius: Bán kính đường tròn (m).
        start_pt: Điểm bắt đầu trên biên.
        dphi: Góc quét (rad).
        sense: Chiều di chuyển (+1 CCW, -1 CW).
        theta_max_rad: Góc chuyển hướng tối đa tại mỗi đỉnh (rad).

    Returns:
        Danh sách các trạng thái (đỉnh, hướng_bay) ngoại tiếp cung tròn.
    """
    if dphi <= 1e-9:
        return []
    n = max(1, math.ceil(dphi / theta_max_rad))
    step = dphi / n
    rv = radius / math.cos(step / 2.0)
    phi0 = math.atan2(start_pt[1] - center[1], start_pt[0] - center[0])
    out: list[PlannerState] = []
    for k in range(n):
        mid = phi0 + sense * step * (k + 0.5)
        vertex = (center[0] + rv * math.cos(mid), center[1] + rv * math.sin(mid))
        nxt = phi0 + sense * step * (k + 1)
        tangent_pt = (
            center[0] + radius * math.cos(nxt),
            center[1] + radius * math.sin(nxt),
        )
        out.append((vertex, tangent_heading(tangent_pt, center, sense)))
    return out


def has_angular_overlap(a0: float, a1: float, b0: float, b1: float) -> bool:
    """Kiểm tra hai khoảng góc có chồng lấn nhau trên đường tròn không (modulo 2*pi).

    Args:
        a0: Cận dưới khoảng thứ nhất (rad).
        a1: Cận trên khoảng thứ nhất (rad), >= a0.
        b0: Cận dưới khoảng thứ hai (rad).
        b1: Cận trên khoảng thứ hai (rad), >= b0.

    Returns:
        True nếu hai khoảng góc có phần giao nhau; ngược lại False.
    """
    two_pi = 2.0 * math.pi
    wa = a1 - a0
    wb = b1 - b0
    if wa >= two_pi or wb >= two_pi:
        return True
    a = a0 % two_pi
    b = b0 % two_pi
    return ((b - a) % two_pi) < wa or ((a - b) % two_pi) < wb


def sector_polygon(
    center: Point, r_in: float, r_out: float, phi_a: float, phi_b: float
) -> PolygonCoords:
    """Tạo đa giác tứ giác bao phủ một hình quạt vành khuyên hẹp.

    Args:
        center: Tọa độ tâm đường tròn.
        r_in: Bán kính trong của hình quạt (m).
        r_out: Bán kính ngoài của hình quạt (m).
        phi_a: Cạnh góc thứ nhất của hình quạt (rad).
        phi_b: Cạnh góc thứ hai của hình quạt (rad).

    Returns:
        Danh sách 4 đỉnh tứ giác không lặp lại đỉnh đóng.
    """
    width = abs(phi_b - phi_a)
    r_out_pad = r_out / math.cos(min(width, math.pi / 2) / 2.0)
    ca, sa = math.cos(phi_a), math.sin(phi_a)
    cb, sb = math.cos(phi_b), math.sin(phi_b)
    return [
        (center[0] + r_in * ca, center[1] + r_in * sa),
        (center[0] + r_in * cb, center[1] + r_in * sb),
        (center[0] + r_out_pad * cb, center[1] + r_out_pad * sb),
        (center[0] + r_out_pad * ca, center[1] + r_out_pad * sa),
    ]
