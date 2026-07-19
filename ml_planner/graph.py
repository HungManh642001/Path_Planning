"""Explicit tangent/bitangent graph over a preprocessed scenario.

Nodes are the waypoint candidates the kinodynamic search navigates between
(bitangent touch points on inflated circles, tangent points from start/goal,
polygon hull vertices, start, goal); edges are collision-free chords plus
boundary arcs. The GNN guidance consumes this graph; nothing in core/ does.

All circle geometry is built on radius r + config.CONSTRUCTION_CLEARANCE_M,
mirroring the planner's construction-side clearance convention.
"""
import math

import config


def bitangent_points(c1, r1, c2, r2):
    """Touch-point pairs of the up-to-4 bitangent segments between two circles.

    Returns [(p_on_circle1, p_on_circle2), ...] — external bitangents first
    (exist unless one circle contains the other), then internal ones (exist
    only when the circles are disjoint). [] for concentric circles.

    Construction: a bitangent touches circle1 at polar angle phi where
    phi = theta ± acos((r1 - s·r2)/d), theta = angle(c1→c2), s = +1 external /
    -1 internal; it touches circle2 at phi (external) or phi + pi (internal).
    """
    (x1, y1), (x2, y2) = c1, c2
    d = math.hypot(x2 - x1, y2 - y1)
    if d < 1e-9:
        return []
    theta = math.atan2(y2 - y1, x2 - x1)
    pairs = []
    for s in (+1.0, -1.0):              # +1 external, -1 internal
        cosval = (r1 - s * r2) / d
        if abs(cosval) > 1.0:
            continue                    # this bitangent family does not exist
        alpha = math.acos(cosval)
        for side in (+1.0, -1.0):
            phi1 = theta + side * alpha
            phi2 = phi1 if s > 0 else phi1 + math.pi
            p1 = (x1 + r1 * math.cos(phi1), y1 + r1 * math.sin(phi1))
            p2 = (x2 + r2 * math.cos(phi2), y2 + r2 * math.sin(phi2))
            pairs.append((p1, p2))
    return pairs
