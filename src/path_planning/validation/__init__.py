"""Bộ kiểm định độc lập tính hợp lệ và an toàn của quỹ đạo đường bay."""

from path_planning.validation.oracle import (
    ValidationResult,
    arc_points,
    arcs_clear,
    path_is_valid,
    segments_clear,
    straight_segments_ok,
    turn_angles_ok,
)


__all__ = [
    "ValidationResult",
    "arc_points",
    "arcs_clear",
    "path_is_valid",
    "segments_clear",
    "straight_segments_ok",
    "turn_angles_ok",
]
