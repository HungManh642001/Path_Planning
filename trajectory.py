"""
Trajectory rendering facade.

Turns the planner's [(waypoint, heading), ...] output into a flat list of (x, y)
points for drawing. Two modes:
  - 'straight': the waypoint positions joined by straight lines.
  - 'dubins'  : the real flight path (radius-R Dubins curve between consecutive
                configurations); falls back to arc_line_trajectory then to the
                straight segment if a Dubins solve degenerates, so the output is
                always continuous.
"""

import dubins_curves as dc
import spatial_utils as su


def sample_trajectory(path, R, mode='dubins', step=None):
    if not path:
        return []
    waypoints = [wp for wp, _ in path]
    if len(waypoints) == 1:
        return [waypoints[0]]
    if mode == 'straight':
        return list(waypoints)

    if step is None:
        step = R / 8.0
    pts = [waypoints[0]]
    for i in range(len(path) - 1):
        (p0, h0), (p1, h1) = path[i], path[i + 1]
        seg = _dubins_segment(p0, h0, p1, h1, R, step)
        if seg is None:
            seg = _fallback_segment(p0, p1, R)
        pts.extend(seg[1:])           # drop duplicated join point
    return pts


def _dubins_segment(p0, h0, p1, h1, R, step):
    try:
        dub = dc.DubinsPath(p0, h0, p1, h1, R)
        samples = dub.sample_path(step)
    except (ValueError, ZeroDivisionError):
        return None
    if len(samples) < 2:
        return None
    return [(x, y) for (x, y, _) in samples]


def _fallback_segment(p0, p1, R):
    # arc_line_trajectory needs >=3 points to insert an arc; for a 2-point leg it
    # returns the straight segment, which is exactly the safe fallback here.
    seg = su.arc_line_trajectory([p0, p1], R)
    if len(seg) < 2:
        return [p0, p1]
    return seg
