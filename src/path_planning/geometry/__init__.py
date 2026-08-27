"""Các hàm toán học và hình học không gian 2D cho bài toán lập kế hoạch đường bay."""

from path_planning.geometry.arc import (
    arc_angle,
    arc_waypoints,
    bitangent_departures,
    departure_point,
    has_angular_overlap,
    riding_sense,
    sector_polygon,
    tangent_heading,
)
from path_planning.geometry.goal_shot import TwoCornerCandidate, two_corner_candidates
from path_planning.geometry.spatial import (
    angle_diff,
    angle_to_heading,
    circle_tangent_points,
    distance,
    inflate_polygon,
    point_to_line_distance,
    state_to_tuple,
)


__all__ = [
    "TwoCornerCandidate",
    "angle_diff",
    "angle_to_heading",
    "arc_angle",
    "arc_waypoints",
    "bitangent_departures",
    "circle_tangent_points",
    "departure_point",
    "distance",
    "has_angular_overlap",
    "inflate_polygon",
    "point_to_line_distance",
    "riding_sense",
    "sector_polygon",
    "state_to_tuple",
    "tangent_heading",
    "two_corner_candidates",
]
