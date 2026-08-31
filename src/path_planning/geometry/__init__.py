"""Các hàm toán học và hình học không gian 2D cho bài toán lập kế hoạch đường bay."""

from path_planning.geometry.arc import (
    is_point_on_any_circle_boundary,
    is_point_on_circle_boundary,
)
from path_planning.geometry.goal_shot import (
    TwoCornerCandidate,
    build_goal_cone,
    two_corner_candidates,
)
from path_planning.geometry.spatial import (
    angle_diff,
    angle_to_heading,
    calculate_dubins_path_length,
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
    "build_goal_cone",
    "calculate_dubins_path_length",
    "circle_tangent_points",
    "distance",
    "inflate_polygon",
    "is_point_on_any_circle_boundary",
    "is_point_on_circle_boundary",
    "point_to_line_distance",
    "state_to_tuple",
    "two_corner_candidates",
]
