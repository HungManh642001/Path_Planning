"""Biểu diễn nút trạng thái ô lưới tìm kiếm và băm trạng thái."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from path_planning import config
from path_planning.geometry import spatial as su


if TYPE_CHECKING:
    from path_planning.types import LatticeKey, Point


class State:
    """Một nút trạng thái tìm kiếm đại diện cho vị trí 2D và hướng bay.

    Attributes:
        waypoint: Planar 2D coordinate (x, y) in metres.
        heading: Vehicle orientation angle in radians, or None for free-goal.
        cos_h: Cosine of heading angle for fast dot-product prefiltering.
        sin_h: Sine of heading angle for fast dot-product prefiltering.
        parent: Pointer to preceding state on the search path.
        g_cost: Accumulated cost from takeoff to this state in metres.
        h_cost: Estimated heuristic cost from this state to the goal in metres.
        straight_budget: Remaining straight flight length on the inbound leg (m).
        min_straight_in: Minimum required straight flight threshold (m).
        is_start_corner: True if state is one of the initial seeded takeoff corners.
        via: Optional intermediate waypoint inserted during pivot sliding.
    """

    def __init__(self, waypoint: Point, heading: float | None) -> None:
        """Khởi tạo một nút trạng thái trên lưới tìm kiếm.

        Args:
            waypoint: 2D planar coordinates (x, y) in metres.
            heading: Heading angle in radians, or None for headingless goal target.
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
            TypeError: If heading is None.
        """
        if self.heading is None:
            raise TypeError("a headingless goal target has no lattice key")
        return su.state_to_tuple(self.waypoint, self.heading)

    def __hash__(self) -> int:
        """Băm nút trạng thái theo ô lưới lượng tử hóa.

        Returns:
            Integer hash value.
        """
        key = self._key
        if key is None:
            key = self._key = self._compute_key()
        return hash(key)

    def __eq__(self, other: object) -> bool:
        """So sánh bằng nhau dựa trên khóa ô lưới lượng tử hóa.

        Args:
            other: Comparison target object.

        Returns:
            True if other is State and falls in the same quantized lattice cell.
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
            other: Another State to compare with.

        Returns:
            True if this state has strictly lower f-cost than other.
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
