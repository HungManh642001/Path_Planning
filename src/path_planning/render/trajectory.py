"""Trajectory rendering facade.

Turns the planner's ``[(waypoint, heading), ...]`` output into a flat list of
``(x, y)`` points for drawing. Two modes:

- ``'straight'``: the waypoint positions joined by straight lines.
- ``'dubins'``: the real flight path -- straight legs joined by radius-R turn
  arcs. At each interior WAYPOINT the arc is tangent to both legs (the fillet
  model): symmetric about the waypoint, radius R, and subtending the heading
  change. Because the arc is tangent, entry and exit headings are preserved
  exactly, so the takeoff leaves ``O`` along the takeoff heading and the
  approach reaches ``T`` along the required approach heading. (A circular arc of
  radius R cannot both pass through the corner waypoint AND keep those headings,
  so the arc rounds the corner rather than passing through it; this matches the
  planner's validated kinodynamic model: straight trajectory plus
  ``R*tan(alpha/2)`` tangents.)

:func:`turn_markers` returns, per turn, the start-of-turn and end-of-turn points
(the two tangent points) so the caller can mark where each arc begins and ends.

:func:`build_full_path` prepends the takeoff point ``O`` and appends the goal
``T`` (taken from the preprocessed scenario) so the drawn flight path spans
``O..T`` -- the planner's path only covers the interior waypoints
``W_1..W_{n-1}``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Literal, TypedDict

from path_planning.core import mission as mission
from path_planning.core.types import PlannerState, Point, PreprocessedScenario

_ARC_SAMPLES = 24  # even -> a sample lands exactly on the waypoint (arc midpoint)

RenderMode = Literal["straight", "dubins"]
"""How :func:`sample_trajectory` joins consecutive waypoints."""


class TurnMarker(TypedDict):
    """Where one fillet arc begins, pivots and ends.

    Attributes:
        start: Entry tangent point, where the arc leaves the incoming leg.
        mid: The waypoint the arc is symmetric about.
        end: Exit tangent point, where the arc rejoins the outgoing leg.
        angle_deg: Signed heading change; positive is left/CCW.
    """

    start: Point
    mid: Point
    end: Point
    angle_deg: float


def sample_trajectory(
    path: Sequence[PlannerState],
    turn_radius: float,
    mode: RenderMode = "dubins",
    step: float | None = None,
) -> list[Point]:
    """Sample the flown trajectory as a dense polyline.

    Args:
        path: Waypoints as ``(waypoint, heading)`` pairs.
        turn_radius: Fillet arc radius (m).
        mode: ``'straight'`` to join waypoints directly, ``'dubins'`` to round
            each interior corner with a tangent arc.
        step: Sample spacing along straight legs (m); defaults to
            ``turn_radius / 8``.

    Returns:
        The polyline points; empty for an empty path.
    """
    if not path:
        return []
    waypoints = [wp for wp, _ in path]
    if len(waypoints) == 1:
        return [waypoints[0]]
    if mode == "straight":
        return list(waypoints)
    if step is None:
        step = turn_radius / 8.0
    points, _ = _dubins_arc_path(waypoints, turn_radius, step)
    return points


def turn_markers(path: Sequence[PlannerState], turn_radius: float) -> list[TurnMarker]:
    """Locate every fillet arc along the path.

    Args:
        path: Waypoints as ``(waypoint, heading)`` pairs.
        turn_radius: Fillet arc radius (m).

    Returns:
        One marker per turn, in path order. Straight legs produce no markers,
        and a path of fewer than three waypoints has no interior corner at all.
    """
    waypoints = [wp for wp, _ in path]
    if len(waypoints) < 3:
        return []
    _, turns = _dubins_arc_path(waypoints, turn_radius, turn_radius / 8.0)
    return turns


def build_full_path(
    result_path: Sequence[PlannerState], preprocessed: PreprocessedScenario | None
) -> list[PlannerState]:
    """Prepend takeoff ``O`` and append goal ``T`` so the drawn path spans the mission.

    Thin alias for :func:`core.mission.full_mission_path`, kept because this is
    the name the render layer, the GUI and the tests already call. The planners
    validate the path they emit with the SAME function, which is the point: the
    drawn trajectory and the oracle's verdict must be about one list of
    waypoints.

    Args:
        result_path: The planner's interior waypoints.
        preprocessed: The prepared scenario supplying the endpoints.

    Returns:
        The full mission path, endpoints included.
    """
    return mission.full_mission_path(result_path, preprocessed)


# --------------------------------------------------------------------------
# Internal geometry
# --------------------------------------------------------------------------


def _extend_straight(points: list[Point], target: Point, step: float) -> None:
    """Append samples along the straight segment ``points[-1] -> target``.

    The starting point is not repeated. A degenerate (sub-nanometre) segment
    appends nothing.
    """
    x0, y0 = points[-1]
    x1, y1 = target
    d = math.hypot(x1 - x0, y1 - y0)
    if d < 1e-9:
        return
    nseg = max(1, math.ceil(d / step))
    for k in range(1, nseg + 1):
        t = k / nseg
        points.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))


def _unit(a: Point, b: Point) -> Point:
    """Return the unit vector from ``a`` to ``b``, or ``(0, 0)`` if they coincide."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    return (dx / d, dy / d) if d > 0 else (0.0, 0.0)


def _dubins_arc_path(
    waypoints: Sequence[Point],
    turn_radius: float,
    step: float,
    arc_samples: int = _ARC_SAMPLES,
) -> tuple[list[Point], list[TurnMarker]]:
    """Build straight legs joined by radius-R fillet arcs at each interior waypoint.

    Each arc is tangent to both legs and symmetric about the waypoint, so entry
    and exit headings are preserved exactly.

    Args:
        waypoints: Corner positions in path order.
        turn_radius: Fillet arc radius (m).
        step: Sample spacing along straight legs (m).
        arc_samples: Samples emitted per arc.

    Returns:
        A ``(points, turns)`` pair: a dense continuous polyline, and one
        :class:`TurnMarker` per arc.
    """
    points: list[Point] = [waypoints[0]]
    turns: list[TurnMarker] = []
    for i in range(1, len(waypoints) - 1):
        wp_prev, wp, wp_next = waypoints[i - 1], waypoints[i], waypoints[i + 1]
        u = _unit(wp_prev, wp)  # incoming leg direction
        v = _unit(wp, wp_next)  # outgoing leg direction
        h_in = math.atan2(u[1], u[0])
        h_out = math.atan2(v[1], v[0])
        alpha = math.atan2(math.sin(h_out - h_in), math.cos(h_out - h_in))
        a_abs = abs(alpha)
        if a_abs < 1e-9:
            _extend_straight(points, wp, step)  # no turn: straight to wp
            continue
        # Tangent inset t = R*tan(alpha/2) - unchanged radius R.
        t = turn_radius * math.tan(a_abs / 2.0)
        s = 1.0 if alpha > 0 else -1.0
        start = (wp[0] - u[0] * t, wp[1] - u[1] * t)  # entry tangent point
        end = (wp[0] + v[0] * t, wp[1] + v[1] * t)  # exit tangent point
        n_in = (-u[1] * s, u[0] * s)  # inward normal
        cx = start[0] + turn_radius * n_in[0]
        cy = start[1] + turn_radius * n_in[1]
        ang0 = math.atan2(start[1] - cy, start[0] - cx)
        _extend_straight(points, start, step)  # straight leg into the turn
        for k in range(1, arc_samples + 1):
            a = ang0 + s * a_abs * (k / arc_samples)
            points.append((cx + turn_radius * math.cos(a), cy + turn_radius * math.sin(a)))
        turns.append({"start": start, "mid": wp, "end": end, "angle_deg": math.degrees(alpha)})
    _extend_straight(points, waypoints[-1], step)  # final straight leg
    return points, turns
