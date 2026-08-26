"""Pure geometry for the analytic terminal "goal shot".

Enumerates 2-corner vehicle manoeuvres that connect a search state
``(position, heading)`` to the goal waypoint, arriving with a heading inside
the +-alpha_max terminal cone.

A candidate is: turn <= alpha_max at ``position`` onto leg 1 (direction
``leg1_heading``), fly straight to an intermediate corner ``corner``, turn
<= alpha_max there onto leg 2 (direction ``arrival_heading``), fly straight to
the goal. ``corner`` is the intersection of the ray from ``position`` along
``leg1_heading`` and the back-ray into the goal along ``arrival_heading``.

No planner or config imports: every tolerance is a parameter, so this module
stays a pure-geometry leaf that can be tested in isolation.
"""

from __future__ import annotations

import math
from typing import NamedTuple

from path_planning.core.types import Point


class TwoCornerCandidate(NamedTuple):
    """One feasible 2-corner manoeuvre to the goal.

    Attributes:
        total_length: Combined length of both straight legs (m).
        corner: The intermediate corner ``C`` joining the two legs.
        leg1_heading: Heading of the leg leaving the current position (rad).
        arrival_heading: Heading of the leg arriving at the goal (rad).
        budget_corner: Straight length left on leg 1 after its near reserve (m).
        budget_goal: Straight length left on leg 2 after its near reserve (m).
    """

    total_length: float
    corner: Point
    leg1_heading: float
    arrival_heading: float
    budget_corner: float
    budget_goal: float


def _angdiff(a: float, b: float) -> float:
    """Return the smallest signed difference ``a - b`` normalised to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


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
    num_dir: int = 9,
    num_cone: int = 9,
) -> list[TwoCornerCandidate]:
    """Enumerate feasible 2-corner manoeuvres to the goal, shortest first.

    Args:
        position: Current waypoint ``(x, y)``.
        heading: Current heading (rad).
        goal_waypoint: Goal waypoint ``W_{n-1}`` ``(x, y)``.
        goal_heading: Required approach heading at the goal (rad).
        turn_radius: Vehicle turn radius (m).
        alpha_max: Maximum turn angle per corner (rad).
        min_straight: Minimum usable straight length per leg (m).
        straight_budget_in: Remaining straight budget of the leg INTO
            ``position`` (deferred đoản-trình: that leg must still keep
            ``min_straight_in`` after the turn reserve at ``position``).
        min_straight_in: Đoản-trình threshold for the incoming leg (m).
        num_dir: Number of turn-at-position directions sampled across
            ``[heading +- alpha_max]``. Must be >= 2.
        num_cone: Number of arrival headings sampled across
            ``[goal_heading +- alpha_max]``. Must be >= 2.

    Returns:
        Feasible candidates sorted by ``total_length`` ascending; empty if
        nothing is angle- or length-feasible.

    Raises:
        ValueError: If ``num_dir`` or ``num_cone`` is below 2, which would make
            the sampler divide by zero.
    """
    if num_dir < 2 or num_cone < 2:
        raise ValueError(
            f"num_dir and num_cone must be >= 2; got {num_dir}, {num_cone}"
        )

    px, py = position
    delta_x, delta_y = goal_waypoint[0] - px, goal_waypoint[1] - py

    # The arrival cone depends only on j, so its trigonometry is hoisted out of
    # the nested loop: at the default 25x25 it turned 625 evaluations of each of
    # cos, sin, atan2 and tan into 25. Cone entries whose terminal turn exceeds
    # alpha_max (float noise on the cone edge) are dropped here rather than
    # re-tested num_dir times.
    cone: list[tuple[float, float, float, float]] = []
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

    out: list[TwoCornerCandidate] = []
    for i in range(num_dir):
        leg1_heading = heading - alpha_max + (2.0 * alpha_max) * i / (num_dir - 1)
        turn_at_position = abs(_angdiff(leg1_heading, heading))
        # Deferred đoản-trình of the incoming leg (near reserve = R*tan(a/2)).
        if (
            straight_budget_in - turn_radius * math.tan(turn_at_position / 2.0)
            < min_straight_in
        ):
            continue
        ux, uy = math.cos(leg1_heading), math.sin(leg1_heading)
        reserve_1 = turn_radius * math.tan(turn_at_position / 2.0)
        for arrival_heading, vx, vy, reserve_terminal in cone:
            turn_at_corner = abs(_angdiff(arrival_heading, leg1_heading))
            if turn_at_corner > alpha_max:
                continue
            det = ux * vy - uy * vx
            if abs(det) < 1e-9:
                continue  # legs parallel: no corner
            leg1_len = (delta_x * vy - delta_y * vx) / det
            leg2_len = (ux * delta_y - uy * delta_x) / det
            if leg1_len <= 0.0 or leg2_len <= 0.0:
                continue  # corner behind an endpoint
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
                    leg1_len + leg2_len,
                    corner,
                    leg1_heading,
                    arrival_heading,
                    budget_corner,
                    budget_goal,
                )
            )
    out.sort(key=lambda c: c.total_length)
    return out
