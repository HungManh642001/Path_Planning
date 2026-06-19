"""
Dubins Path Module
Shortest path of bounded curvature (radius R) between two configurations (x, y, heading).
Rendering only -- not used by planning or collision checking.
Standard normalized formulation (Walker / LaValle), six words: LSL RSR LSR RSL RLR LRL.
"""

import math

_WORDS = ('LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL')


def _mod2pi(theta):
    return theta % (2 * math.pi)


def _word_lengths(word, alpha, beta, d):
    """Normalized segment lengths (t, p, q) for `word`, or None if infeasible."""
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)

    if word == 'LSL':
        p_sq = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
        if p_sq < 0:
            return None
        tmp = math.atan2(cb - ca, d + sa - sb)
        return (_mod2pi(-alpha + tmp), math.sqrt(p_sq), _mod2pi(beta - tmp))

    if word == 'RSR':
        p_sq = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
        if p_sq < 0:
            return None
        tmp = math.atan2(ca - cb, d - sa + sb)
        return (_mod2pi(alpha - tmp), math.sqrt(p_sq), _mod2pi(-beta + tmp))

    if word == 'LSR':
        p_sq = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
        if p_sq < 0:
            return None
        p = math.sqrt(p_sq)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        return (_mod2pi(-alpha + tmp), p, _mod2pi(-_mod2pi(beta) + tmp))

    if word == 'RSL':
        p_sq = -2 + d * d + 2 * cab - 2 * d * (sa + sb)
        if p_sq < 0:
            return None
        p = math.sqrt(p_sq)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        return (_mod2pi(alpha - tmp), p, _mod2pi(beta - tmp))

    if word == 'RLR':
        tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8.0
        if abs(tmp) > 1:
            return None
        p = _mod2pi(2 * math.pi - math.acos(tmp))
        t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        return (t, p, _mod2pi(alpha - beta - t + p))

    if word == 'LRL':
        tmp = (6 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8.0
        if abs(tmp) > 1:
            return None
        p = _mod2pi(2 * math.pi - math.acos(tmp))
        t = _mod2pi(-alpha + math.atan2(-ca + cb, d + sa - sb) + p / 2.0)
        return (t, p, _mod2pi(_mod2pi(beta) - alpha - t + p))

    return None


def _step_config(x, y, theta, seg_len, kind, R):
    """Advance config (x, y, theta) by actual arc length seg_len along segment kind."""
    if kind == 'S':
        return (x + seg_len * math.cos(theta), y + seg_len * math.sin(theta), theta)
    dtheta = seg_len / R
    if kind == 'L':
        cx, cy = x - R * math.sin(theta), y + R * math.cos(theta)
        nt = theta + dtheta
        return (cx + R * math.sin(nt), cy - R * math.cos(nt), nt)
    # 'R'
    cx, cy = x + R * math.sin(theta), y - R * math.cos(theta)
    nt = theta - dtheta
    return (cx - R * math.sin(nt), cy + R * math.cos(nt), nt)


class DubinsPath:
    """Dubins shortest path between two configurations with turn radius `radius`."""

    def __init__(self, start_pos, start_heading, goal_pos, goal_heading, radius):
        self.start_pos = start_pos
        self.start_heading = start_heading
        self.goal_pos = goal_pos
        self.goal_heading = goal_heading
        self.radius = float(radius)

    def _normalized(self):
        dx = self.goal_pos[0] - self.start_pos[0]
        dy = self.goal_pos[1] - self.start_pos[1]
        D = math.hypot(dx, dy)
        d = D / self.radius
        theta = math.atan2(dy, dx) if D > 1e-12 else 0.0
        alpha = _mod2pi(self.start_heading - theta)
        beta = _mod2pi(self.goal_heading - theta)
        return alpha, beta, d

    def shortest_path(self):
        alpha, beta, d = self._normalized()
        best = None
        for word in _WORDS:
            res = _word_lengths(word, alpha, beta, d)
            if res is None:
                continue
            t, p, q = res
            length = (t + p + q) * self.radius
            if best is None or length < best['length']:
                best = {'word': word, 'length': length, 't': t, 'p': p, 'q': q}
        return best

    def sample_path(self, step):
        sp = self.shortest_path()
        if sp is None:
            return []
        kinds = list(sp['word'])
        seg_lengths = [sp['t'] * self.radius, sp['p'] * self.radius, sp['q'] * self.radius]
        x, y, theta = self.start_pos[0], self.start_pos[1], self.start_heading
        pts = [(x, y, theta)]
        for kind, seg_len in zip(kinds, seg_lengths):
            n = max(1, int(math.ceil(seg_len / step)))
            sub = seg_len / n
            for _ in range(n):
                x, y, theta = _step_config(x, y, theta, sub, kind, self.radius)
                pts.append((x, y, theta))
        return pts


def compute_dubins_path_between_waypoints(waypoints, turn_radius):
    """
    Compute Dubins paths between all consecutive waypoints.

    Args:
        waypoints: List of (pos, heading) tuples
        turn_radius: Missile turn radius

    Returns:
        List of DubinsPath objects
    """
    dubins_paths = []

    for i in range(len(waypoints) - 1):
        start_wp, start_heading = waypoints[i]
        goal_wp, goal_heading = waypoints[i + 1]

        dubins = DubinsPath(start_wp, start_heading, goal_wp, goal_heading, turn_radius)
        dubins_paths.append(dubins)

    return dubins_paths


def sample_all_dubins_paths(dubins_paths, step=50):
    """
    Sample all Dubins paths to get smooth trajectory points.

    Args:
        dubins_paths: List of DubinsPath objects
        step: Step size in metres for sampling along path segments

    Returns:
        List of (x, y, heading) tuples representing smooth trajectory
    """
    all_samples = []

    for i, dubins_path in enumerate(dubins_paths):
        samples = dubins_path.sample_path(step=step)

        # Skip first sample of subsequent segments (avoid duplicates)
        if i > 0 and samples:
            samples = samples[1:]

        all_samples.extend(samples)

    return all_samples
