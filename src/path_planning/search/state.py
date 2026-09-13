"""Biểu diễn nút trạng thái ô lưới tìm kiếm và băm trạng thái."""

from __future__ import annotations

import math

from path_planning import config
from path_planning.types import LatticeKey, Point


def state_to_tuple(waypoint: Point, heading: float) -> LatticeKey:
    """Rời rạc hóa trạng thái (tọa_độ, hướng_bay) thành khóa ô lưới.

    Args:
        waypoint: Tọa độ điểm (x, y) tính bằng mét.
        heading: Góc hướng bay tính bằng radian.

    Returns:
        Khóa ô lưới dạng (x_index, y_index, heading_index).
    """
    q = config.STATE_POS_QUANTUM
    hq = math.radians(config.STATE_HEADING_QUANTUM_DEG)
    hx = int(waypoint[0] // q)
    hy = int(waypoint[1] // q)
    hh = round(math.atan2(math.sin(heading), math.cos(heading)) / hq)
    return (hx, hy, hh)


class State:
    """Một nút trạng thái tìm kiếm đại diện cho vị trí 2D và hướng bay.

    Attributes:
        waypoint: Tọa độ 2D mặt phẳng (x, y) tính bằng mét.
        heading: Góc hướng bay tính bằng radian, hoặc None nếu đích tự do.
        cos_h: Cosine góc hướng bay dùng để lọc nhanh tích vô hướng.
        sin_h: Sine góc hướng bay dùng để lọc nhanh tích vô hướng.
        parent: Con trỏ tới trạng thái tiền nhiệm trên đường tìm kiếm.
        g_cost: Chi phí tích lũy từ điểm xuất phát tới trạng thái này (m).
        h_cost: Chi phí heuristic ước lượng từ trạng thái này tới đích (m).
        straight_budget: Chiều dài đoạn bay thẳng còn lại trên chặng bay vào (m).
        min_straight_in: Ngưỡng chiều dài bay thẳng tối thiểu bắt buộc (m).
        is_start_corner: True nếu là một trong các điểm rẽ xuất phát được gieo mầm.
        via: Waypoint trung gian tùy chọn được chèn khi trượt điểm pivot.
    """

    def __init__(self, waypoint: Point, heading: float | None) -> None:
        """Khởi tạo một nút trạng thái trên lưới tìm kiếm.

        Args:
            waypoint: Tọa độ 2D mặt phẳng (x, y) tính bằng mét.
            heading: Góc hướng bay (rad), hoặc None nếu đích không ràng buộc.
        """
        self.waypoint: Point = waypoint
        self.heading: float | None = heading
        self.cos_h: float | None = math.cos(heading) if heading is not None else None
        self.sin_h: float | None = math.sin(heading) if heading is not None else None
        self.parent: State | None = None
        self.g_cost: float = float("inf")
        self.h_cost: float = 0.0
        self.straight_budget: float = float("inf")
        self.min_straight_in: float = config.MIN_STRAIGHT_M
        self.is_start_corner: bool = False
        self.via: tuple[Point, float] | None = None
        self._key: LatticeKey | None = None

    def _compute_key(self) -> LatticeKey:
        """Lượng tử hóa tọa độ liên tục thành các ô lưới rời rạc.

        Returns:
            Tuple (x_bin, y_bin, heading_bin).

        Raises:
            TypeError: Nếu hướng bay heading là None.
        """
        if self.heading is None:
            raise TypeError("a headingless goal target has no lattice key")
        return state_to_tuple(self.waypoint, self.heading)

    def __hash__(self) -> int:
        """Băm nút trạng thái theo ô lưới lượng tử hóa.

        Returns:
            Giá trị băm nguyên.
        """
        key = self._key
        if key is None:
            key = self._key = self._compute_key()
        return hash(key)

    def __eq__(self, other: object) -> bool:
        """So sánh bằng nhau dựa trên khóa ô lưới lượng tử hóa.

        Args:
            other: Đối tượng mục tiêu cần so sánh.

        Returns:
            True nếu other là State và rơi vào cùng ô lưới rời rạc; False ngược lại.
        """
        if not isinstance(other, State):
            return NotImplemented
        key = self._key
        if key is None:
            key = self._key = self._compute_key()
        other_key = other._key
        if other_key is None:
            other_key = other._key = other._compute_key()
        return key == other_key

    def __lt__(self, other: State) -> bool:
        """So sánh thứ tự ưu tiên theo tổng chi phí ước lượng f = g + w*h.

        Args:
            other: Nút State khác cần so sánh.

        Returns:
            True nếu trạng thái này có chi phí f thấp hơn rõ rệt.
        """
        return (self.g_cost + config.HEURISTIC_WEIGHT * self.h_cost) < (
            other.g_cost + config.HEURISTIC_WEIGHT * other.h_cost
        )

    def __repr__(self) -> str:
        """Chuỗi biểu diễn trạng thái phục vụ gỡ lỗi."""
        heading = (
            "none" if self.heading is None else f"{math.degrees(self.heading):.1f}°"
        )
        return f"State(wp={self.waypoint}, h={heading})"
