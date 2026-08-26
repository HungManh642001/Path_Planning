"""Đổi giữa phương vị đối ngoại và quy ước góc nội bộ của thuật toán.

Trên dây: ĐỘ, phương vị thật, thuận chiều kim đồng hồ từ chính bắc.
Trong `core/`: RADIAN, ngược chiều kim đồng hồ từ trục +x.
Mặt phẳng quy ước +y là bắc, +x là đông.

Toàn bộ service đổi đơn vị góc ở đúng module này. Rải phép đổi ra nhiều chỗ là
cách chắc chắn nhất để có hai quy ước cùng tồn tại mà không ai nhận ra.
"""

from __future__ import annotations

import math


def bearing_deg_to_math_rad(bearing_deg: float) -> float:
    """Đổi phương vị (độ, thuận kim đồng hồ từ bắc) sang heading toán học (rad).

    Args:
        bearing_deg: Phương vị thật. Giá trị ngoài ``[0, 360)`` được chuẩn hoá.

    Returns:
        Góc radian ngược chiều kim đồng hồ từ ``+x``, trong ``[-pi, pi]``.
    """
    theta = math.radians(90.0 - bearing_deg)
    # atan2(sin, cos) chuẩn hoá về [-pi, pi]. Không dùng số học modulo: dạng
    # `radians((90 - b) % 360 - 180) + pi` trả về 4,712 rad ở phương vị 180 độ,
    # tức ra ngoài dải mà docstring hứa - đã đo, không phải phỏng đoán.
    return math.atan2(math.sin(theta), math.cos(theta))


def math_rad_to_bearing_deg(theta_rad: float) -> float:
    """Đổi heading toán học (rad) sang phương vị (độ, thuận kim đồng hồ từ bắc).

    Args:
        theta_rad: Góc ngược chiều kim đồng hồ từ ``+x``.

    Returns:
        Phương vị thật trong ``[0, 360)``.
    """
    return (90.0 - math.degrees(theta_rad)) % 360.0
