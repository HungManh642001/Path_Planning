"""The flown mission path: ``O -> W_1 ... W_{n-1} -> T``.

The planner searches only the INTERIOR waypoints. The takeoff point ``O`` and
the target ``T`` are constraints, not searched nodes -- ``W_1`` is offset from
``O`` by the takeoff leg and ``W_{n-1}`` from ``T`` by the seeker run-in -- but
the aircraft still flies ``O -> W_1 ... W_{n-1} -> T``, so anything that
measures, validates or draws the real trajectory has to put them back.

This lived in three places at once: both planners (for the final oracle call in
``plan_trajectory``) and ``render.trajectory.build_full_path`` (for drawing),
each a byte-for-byte copy of the others, each carrying a comment promising it
mirrored the other two. The oracle's verdict and the drawn path MUST come from
the same list of waypoints -- that is the invariant
``tests/oracle_validity_test.py`` asserts -- and three copies is a strange way
to guarantee it. It lives in ``core/`` rather than ``render/`` so the dependency
runs render -> core, never the reverse.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from core.types import PlannerState, PreprocessedScenario


def full_mission_path(
    path: Sequence[PlannerState], preprocessed: PreprocessedScenario | None
) -> list[PlannerState]:
    """Prepend takeoff ``O`` and append goal ``T`` around the searched waypoints.

    Endpoints already present (within 1 m) are not duplicated, so calling this
    twice is harmless.

    Args:
        path: The searched interior waypoints as ``(waypoint, heading)`` pairs.
        preprocessed: The prepared scenario supplying ``start_pos``/``goal_pos``
            and the endpoint headings. ``None``, or a dict without those keys,
            returns the path unchanged.

    Returns:
        The full mission path, endpoints included.
    """
    waypoints = list(path)
    if preprocessed is None:
        return waypoints

    takeoff = preprocessed.get("start_pos")
    target = preprocessed.get("goal_pos")
    start_heading = preprocessed.get("start_heading", 0.0)
    goal_heading = preprocessed.get("goal_heading", 0.0)

    if takeoff is not None and (not waypoints or math.dist(takeoff, waypoints[0][0]) > 1.0):
        waypoints.insert(0, ((takeoff[0], takeoff[1]), start_heading))
    if target is not None and (not waypoints or math.dist(target, waypoints[-1][0]) > 1.0):
        if goal_heading is None:
            # Free-goal mode leaves goal_heading None; the arrival heading is
            # then the bearing of the final leg into T.
            last = waypoints[-1][0] if waypoints else None
            goal_heading = math.atan2(target[1] - last[1], target[0] - last[0]) if last else 0.0
        waypoints.append(((target[0], target[1]), goal_heading))
    return waypoints
