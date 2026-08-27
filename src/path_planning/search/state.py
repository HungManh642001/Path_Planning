"""Search lattice node representation."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from path_planning import config
from path_planning.geometry import spatial as su


if TYPE_CHECKING:
    from path_planning.types import LatticeKey, Point


class State:
    """One search node: a waypoint plus the heading the vehicle holds there."""

    def __init__(self, waypoint: Point, heading: float | None) -> None:
        """Initialize search lattice state.

        Args:
            waypoint: 2D coordinates of the state.
            heading: Heading angle in radians (or None for free goal target).
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
        if self.heading is None:
            raise TypeError("a headingless goal target has no lattice key")
        return su.state_to_tuple(self.waypoint, self.heading)

    def __hash__(self) -> int:
        """Hash on the quantised search lattice, caching the key."""
        key = self._key
        if key is None:
            key = self._key = self._compute_key()
        return hash(key)

    def __eq__(self, other: object) -> bool:
        """Compare on the quantised search lattice."""
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
        """Order by f = g + w*h for the priority queue."""
        return (self.g_cost + config.HEURISTIC_WEIGHT * self.h_cost) < (
            other.g_cost + config.HEURISTIC_WEIGHT * other.h_cost
        )

    def __repr__(self) -> str:
        """Return debug representation."""
        heading = (
            "none" if self.heading is None else f"{math.degrees(self.heading):.1f}°"
        )
        return f"State(wp={self.waypoint}, h={heading})"
