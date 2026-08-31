"""Hình học giải tích nghiệm bắn thẳng về đích bằng 2 góc rẽ (Goal Shot).

Tìm kiếm các phương án cơ động 2 góc rẽ nối trạng thái hiện tại (vị trí, hướng)
tới điểm mục tiêu đích W_{n-1} và thỏa mãn góc tiếp cận trong nón +-alpha_max.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import TypeAlias

from path_planning.types import Point

GoalConeEntry: TypeAlias = tuple[float, float, float, float]
"""(arrival_heading, cos(arrival_heading), sin(arrival_heading), reserve_terminal)"""


@dataclass(frozen=True)
class TwoCornerCandidate:
    """Một phương án cơ động 2 góc rẽ khả thi về đích.

    Attributes:
        total_length: Tổng chiều dài hai đoạn bay thẳng (m).
        corner: Điểm rẽ trung gian C nối giữa hai chặng bay thẳng (x, y).
        leg1_heading: Hướng bay rời khỏi vị trí hiện tại (rad).
        arrival_heading: Hướng bay tiếp cận tới mục tiêu đích (rad).
        budget_corner: Ngân sách đoạn thẳng còn lại trên chặng 1 sau góc rẽ (m).
        budget_goal: Ngân sách đoạn thẳng còn lại trên chặng 2 trước khi tới đích (m).
        leg1_len: Chiều dài đoạn bay thẳng thứ nhất từ vị trí tới điểm rẽ (m).
        leg2_len: Chiều dài đoạn bay thẳng thứ hai từ điểm rẽ tới mục tiêu (m).
    """

    total_length: float
    corner: Point
    leg1_heading: float
    arrival_heading: float
    budget_corner: float
    budget_goal: float
    leg1_len: float = 0.0
    leg2_len: float = 0.0


def _angdiff(a: float, b: float) -> float:
    """Tính hiệu hai góc chuẩn hóa về khoảng [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def build_goal_cone(
    goal_heading: float,
    turn_radius: float,
    alpha_max: float,
    num_cone: int = 9,
) -> list[GoalConeEntry]:
    """Tính toán trước nón các hướng tiếp cận mục tiêu hợp lệ.

    Args:
        goal_heading: Hướng bay yêu cầu khi tới đích (rad).
        turn_radius: Bán kính quay vòng tối thiểu R (m).
        alpha_max: Góc lượn tối đa cho phép alpha_max (rad).
        num_cone: Số hướng lấy mẫu góc tiếp cận mục tiêu trong nón hợp lệ (>= 2).

    Returns:
        Danh sách các phần tử GoalConeEntry (arrival_heading, cos, sin, reserve_terminal).

    Raises:
        ValueError: Nếu num_cone < 2.
    """
    if num_cone < 2:
        raise ValueError(f"num_cone must be >= 2; got {num_cone}")

    cone: list[GoalConeEntry] = []
    for j in range(num_cone):
        arrival_heading = (
            goal_heading - alpha_max + (2.0 * alpha_max) * j / (num_cone - 1)
        )
        turn_at_goal = abs(_angdiff(goal_heading, arrival_heading))
        if turn_at_goal > alpha_max:
            continue
        cone.append(
            (
                arrival_heading,
                math.cos(arrival_heading),
                math.sin(arrival_heading),
                turn_radius * math.tan(turn_at_goal / 2.0),
            )
        )
    return cone


def two_corner_candidates(
    position: Point,
    heading: float,
    goal_waypoint: Point,
    goal_heading: float,
    turn_radius: float,
    alpha_max: float,
    min_straight: float,
    straight_budget_in: float,
    min_straight_in: float,
    *,
    num_dir: int = 9,
    num_cone: int = 9,
    cone: Sequence[GoalConeEntry] | None = None,
) -> list[TwoCornerCandidate]:
    """Tìm các nghiệm cơ động 2 góc rẽ khả thi về đích, sắp xếp theo chiều dài tăng dần.

    Args:
        position: Tọa độ điểm hiện tại (x, y).
        heading: Hướng bay hiện tại (rad).
        goal_waypoint: Tọa độ điểm mục tiêu đích W_{n-1} (x, y).
        goal_heading: Hướng bay yêu cầu khi tới đích (rad).
        turn_radius: Bán kính quay vòng tối thiểu R (m).
        alpha_max: Góc lượn tối đa cho phép alpha_max (rad).
        min_straight: Chiều dài đoạn thẳng tối thiểu giữa hai góc lượn (m).
        straight_budget_in: Ngân sách đoạn thẳng còn lại của chặng bay hiện tại (m).
        min_straight_in: Ngưỡng đoạn thẳng tối thiểu của chặng bay trước đó (m).
        num_dir: Số hướng lấy mẫu góc rẽ tại vị trí hiện tại (>= 2).
        num_cone: Số hướng lấy mẫu góc tiếp cận mục tiêu trong nón hợp lệ (>= 2).
        cone: Nón tiếp cận mục tiêu đã được tính toán trước (tùy chọn).

    Returns:
        Danh sách ứng viên khả thi được sắp xếp theo tổng chiều dài từ ngắn đến dài.

    Raises:
        ValueError: Nếu num_dir hoặc num_cone nhỏ hơn 2.
    """
    if num_dir < 2 or (cone is None and num_cone < 2):
        raise ValueError(
            f"num_dir and num_cone must be >= 2; got num_dir={num_dir}, num_cone={num_cone}"
        )

    px, py = position
    delta_x, delta_y = goal_waypoint[0] - px, goal_waypoint[1] - py

    actual_cone = (
        cone
        if cone is not None
        else build_goal_cone(goal_heading, turn_radius, alpha_max, num_cone)
    )

    out: list[TwoCornerCandidate] = []
    for i in range(num_dir):
        offset = -alpha_max + (2.0 * alpha_max) * i / (num_dir - 1)
        leg1_heading = heading + offset
        turn_at_position = abs(offset)
        if (
            straight_budget_in - turn_radius * math.tan(turn_at_position / 2.0)
            < min_straight_in
        ):
            continue
        ux, uy = math.cos(leg1_heading), math.sin(leg1_heading)
        reserve_1 = turn_radius * math.tan(turn_at_position / 2.0)
        for arrival_heading, vx, vy, reserve_terminal in actual_cone:
            turn_at_corner = abs(_angdiff(arrival_heading, leg1_heading))
            if turn_at_corner > alpha_max:
                continue
            det = ux * vy - uy * vx
            if abs(det) < 1e-9:
                continue
            leg1_len = (delta_x * vy - delta_y * vx) / det
            leg2_len = (ux * delta_y - uy * delta_x) / det
            if leg1_len <= 0.0 or leg2_len <= 0.0:
                continue
            reserve_2 = turn_radius * math.tan(turn_at_corner / 2.0)
            budget_corner = leg1_len - reserve_1
            if budget_corner - reserve_2 < min_straight:
                continue
            budget_goal = leg2_len - reserve_2
            if budget_goal - reserve_terminal < min_straight:
                continue
            corner = (px + leg1_len * ux, py + leg1_len * uy)
            out.append(
                TwoCornerCandidate(
                    total_length=leg1_len + leg2_len,
                    corner=corner,
                    leg1_heading=leg1_heading,
                    arrival_heading=arrival_heading,
                    budget_corner=budget_corner,
                    budget_goal=budget_goal,
                    leg1_len=leg1_len,
                    leg2_len=leg2_len,
                )
            )
    out.sort(key=lambda c: c.total_length)
    return out
