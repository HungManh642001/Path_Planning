"""
Trajectory rendering facade.

Turns the planner's [(waypoint, heading), ...] output into a flat list of (x, y)
points for drawing. Two modes:
  - 'straight': the waypoint positions joined by straight lines.
  - 'dubins'  : the real flight path -- straight legs joined by radius-R turn
                arcs. At each interior WAYPOINT the arc is tangent to both legs
                (the fillet model): symmetric about the waypoint, radius R, and
                subtending the heading change (turn angle). Because the arc is
                tangent, the entry and exit headings are preserved exactly -- so
                the launch leaves O along the launch heading and the approach
                reaches T along the required approach heading. (A circular arc of
                radius R cannot both pass through the corner waypoint AND keep
                those headings, so the arc rounds the corner rather than passing
                through it; this matches the planner's validated kinodynamic
                model: straight đoản trình + R*tan(alpha/2) tangents.)

`turn_markers()` returns, per turn, the start-of-turn and end-of-turn points
(the two tangent points) so the caller can mark where each arc begins and ends.

`build_full_path()` prepends the launch point O and appends the target T (taken
from the preprocessed scenario) so the drawn flight path spans O..T -- the
planner's path only covers the interior waypoints W_1..W_{n-1}.
"""

import math

import spatial_utils as su  # noqa: F401  (kept available as a geometry helper)

_ARC_SAMPLES = 24  # even -> a sample lands exactly on the waypoint (arc midpoint)


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
    pts, _ = _dubins_arc_path(waypoints, R, step)
    return pts


def turn_markers(path, R):
    """List of {'start', 'mid', 'end'} for each turn arc (dubins model).

    'mid' is the waypoint (arc midpoint); 'start'/'end' are where the arc begins
    and ends. Straight legs in between produce no markers.
    """
    waypoints = [wp for wp, _ in path]
    if len(waypoints) < 3:
        return []
    _, turns = _dubins_arc_path(waypoints, R, R / 8.0)
    return turns


def build_full_path(result_path, preprocessed):
    """Prepend launch O and append target T (from `preprocessed`) so the drawn
    path spans the whole mission, not just the interior waypoints.

    The planner searches between W_1 (offset from O) and W_{n-1} (offset from T);
    the missile still flies O -> W_1 ... W_{n-1} -> T.
    """
    wps = list(result_path)
    O = preprocessed.get('start_pos') if preprocessed else None
    T = preprocessed.get('goal_pos') if preprocessed else None
    sh = preprocessed.get('start_heading', 0.0) if preprocessed else 0.0
    gh = preprocessed.get('goal_heading', 0.0) if preprocessed else 0.0
    if O is not None and (not wps or math.dist(O, wps[0][0]) > 1.0):
        wps = [(tuple(O), sh)] + wps
    if T is not None and (not wps or math.dist(T, wps[-1][0]) > 1.0):
        wps = wps + [(tuple(T), gh)]
    return wps


# --------------------------------------------------------------------------
# Internal geometry
# --------------------------------------------------------------------------
def _extend_straight(pts, target, step):
    """Append samples along the straight segment pts[-1] -> target (excl. start)."""
    x0, y0 = pts[-1]
    x1, y1 = target
    d = math.hypot(x1 - x0, y1 - y0)
    if d < 1e-9:
        return
    nseg = max(1, int(math.ceil(d / step)))
    for k in range(1, nseg + 1):
        t = k / nseg
        pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))


def _unit(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    return (dx / d, dy / d) if d > 0 else (0.0, 0.0)


def _dubins_arc_path(waypoints, R, step, arc_samples=_ARC_SAMPLES):
    """Straight legs + radius-R fillet arcs tangent to both legs at each interior
    waypoint (symmetric about the waypoint; entry/exit headings preserved).

    Returns (points, turns) where points is a dense continuous polyline and
    turns is a list of {'start', 'mid', 'end'} (start/end are the tangent points).
    """
    pts = [tuple(waypoints[0])]
    turns = []
    n = len(waypoints)
    for i in range(1, n - 1):
        wp_prev, wp, wp_next = waypoints[i - 1], waypoints[i], waypoints[i + 1]
        u = _unit(wp_prev, wp)        # incoming leg direction
        v = _unit(wp, wp_next)        # outgoing leg direction
        h_in = math.atan2(u[1], u[0])
        h_out = math.atan2(v[1], v[0])
        alpha = math.atan2(math.sin(h_out - h_in), math.cos(h_out - h_in))
        a_abs = abs(alpha)
        if a_abs < 1e-9:
            _extend_straight(pts, tuple(wp), step)       # no turn: straight to wp
            continue
        # Tangent inset t = R*tan(alpha/2), clamped so adjacent arcs fit the legs.
        t = R * math.tan(a_abs / 2.0)
        leg_in = math.hypot(wp[0] - wp_prev[0], wp[1] - wp_prev[1])
        leg_out = math.hypot(wp_next[0] - wp[0], wp_next[1] - wp[1])
        t = min(t, leg_in * 0.5, leg_out * 0.5)
        r = t / math.tan(a_abs / 2.0)                    # effective radius after clamp
        s = 1.0 if alpha > 0 else -1.0
        start = (wp[0] - u[0] * t, wp[1] - u[1] * t)     # entry tangent point
        end = (wp[0] + v[0] * t, wp[1] + v[1] * t)       # exit tangent point
        n_in = (-u[1] * s, u[0] * s)                     # inward normal
        cx, cy = start[0] + r * n_in[0], start[1] + r * n_in[1]   # arc centre
        ang0 = math.atan2(start[1] - cy, start[0] - cx)
        _extend_straight(pts, start, step)               # straight leg into the turn
        for k in range(1, arc_samples + 1):
            a = ang0 + s * a_abs * (k / arc_samples)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
        turns.append({'start': start, 'mid': tuple(wp), 'end': end})
    _extend_straight(pts, tuple(waypoints[-1]), step)     # final straight leg
    return pts, turns
