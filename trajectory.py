"""
Trajectory rendering facade.

Turns the planner's [(waypoint, heading), ...] output into a flat list of (x, y)
points for drawing. Two modes:
  - 'straight': the waypoint positions joined by straight lines.
  - 'dubins'  : the real flight path -- straight legs joined by radius-R turn
                arcs. Each interior WAYPOINT sits at the MIDDLE of its turn arc,
                and the arc subtends the heading change (turn angle) at radius R.
                The path therefore passes through every waypoint.

`turn_markers()` returns, per turn, the start-of-turn and end-of-turn points so
the caller can mark where each arc begins and ends.

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


def _dubins_arc_path(waypoints, R, step, arc_samples=_ARC_SAMPLES):
    """Straight legs + radius-R arcs, each interior waypoint at its arc midpoint.

    Returns (points, turns) where points is a dense continuous polyline and
    turns is a list of {'start', 'mid', 'end'}.
    """
    pts = [tuple(waypoints[0])]
    turns = []
    n = len(waypoints)
    for i in range(1, n - 1):
        wp_prev, wp, wp_next = waypoints[i - 1], waypoints[i], waypoints[i + 1]
        h_in = math.atan2(wp[1] - wp_prev[1], wp[0] - wp_prev[0])
        h_out = math.atan2(wp_next[1] - wp[1], wp_next[0] - wp[0])
        alpha = math.atan2(math.sin(h_out - h_in), math.cos(h_out - h_in))
        if abs(alpha) < 1e-9:
            _extend_straight(pts, tuple(wp), step)       # no turn: straight to wp
            continue
        s = 1.0 if alpha > 0 else -1.0
        bisector = h_in + alpha / 2.0
        # Turning-circle centre: perpendicular to the bisector, on the turn side,
        # at distance R. The waypoint lies on this circle (it is the arc midpoint).
        cx = wp[0] + R * math.cos(bisector + s * math.pi / 2.0)
        cy = wp[1] + R * math.sin(bisector + s * math.pi / 2.0)
        phi_w = math.atan2(wp[1] - cy, wp[0] - cx)
        phi_s = phi_w - alpha / 2.0
        phi_e = phi_w + alpha / 2.0
        start = (cx + R * math.cos(phi_s), cy + R * math.sin(phi_s))
        end = (cx + R * math.cos(phi_e), cy + R * math.sin(phi_e))
        _extend_straight(pts, start, step)               # straight leg into the turn
        for k in range(1, arc_samples + 1):
            a = phi_s + (phi_e - phi_s) * (k / arc_samples)
            pts.append((cx + R * math.cos(a), cy + R * math.sin(a)))
        turns.append({'start': start, 'mid': tuple(wp), 'end': end})
    _extend_straight(pts, tuple(waypoints[-1]), step)     # final straight leg
    return pts, turns
