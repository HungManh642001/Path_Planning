"""Pure computation of the GUI results summary (no Tk)."""

import math

import config
import trajectory as tr
import path_validation as pv


def _angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def compute_summary(result, preprocessed, raw_circles, raw_polys, render_mode, runtime_s):
    R = preprocessed.get('turn_radius', config.R)
    base = {
        'success': bool(result.get('success')),
        'distance_km': 0.0,
        'num_waypoints': 0,
        'num_turns': 0,
        'max_turn_deg': 0.0,
        'runtime_ms': runtime_s * 1000.0,
        'iterations': result.get('stats', {}).get('iterations', 0),
        'valid': False,
    }
    path = result.get('path')
    if not (result.get('success') and path):
        return base

    # Distance over the FULL flown path O..T (matches what the canvas draws).
    full = tr.build_full_path(path, preprocessed)
    pts = tr.sample_trajectory(full, R, mode=render_mode)
    dist = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))

    turns = []
    for i in range(1, len(path) - 1):
        h_in = math.atan2(path[i][0][1] - path[i - 1][0][1], path[i][0][0] - path[i - 1][0][0])
        h_out = math.atan2(path[i + 1][0][1] - path[i][0][1], path[i + 1][0][0] - path[i][0][0])
        turns.append(abs(_angle_diff(h_out, h_in)))

    valid = pv.path_is_valid(
        path, preprocessed['circle_obstacles'], preprocessed['polygon_obstacles'],
        R, preprocessed['alpha_max_rad'], config.L0, config.DSS,
        raw_circle_obstacles=raw_circles, raw_polygon_obstacles=raw_polys)

    base.update({
        'distance_km': dist / 1000.0,
        'num_waypoints': len(path),
        'num_turns': sum(1 for t in turns if t > math.radians(1.0)),
        'max_turn_deg': math.degrees(max(turns)) if turns else 0.0,
        'valid': bool(valid),
    })
    return base
