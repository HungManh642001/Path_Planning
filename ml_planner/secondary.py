"""Hand-crafted secondary heuristic for focal search (Phase 1).

The secondary heuristic ranks nodes inside the FOCAL band; it need NOT be
admissible (the admissible Euclid on OPEN keeps the epsilon bound). This
one is Euclid-to-goal inflated when the straight waypoint->goal line is
blocked by an inflated circle, biasing expansion toward states that have a
clear shot at the goal. O(N) in the number of circles.
"""

import math


def _seg_point_dist_sq(px, py, ax, ay, bx, by):
    """Squared distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    sx, sy = bx - ax, by - ay
    dd = sx * sx + sy * sy
    if dd == 0.0:
        rx, ry = px - ax, py - ay
        return rx * rx + ry * ry
    t = ((px - ax) * sx + (py - ay) * sy) / dd
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    ex = (ax + t * sx) - px
    ey = (ay + t * sy) - py
    return ex * ex + ey * ey


def handcrafted_secondary(waypoint, goal_wp, circle_obstacles, block_penalty=1.5):
    """Cost-to-go estimate (meters): Euclid to goal plus a rough detour
    penalty for each inflated circle the straight line to the goal crosses."""
    px, py = waypoint
    gx, gy = goal_wp
    base = math.hypot(gx - px, gy - py)
    blocked = 0.0
    for (cx, cy), r in circle_obstacles:
        if _seg_point_dist_sq(cx, cy, px, py, gx, gy) < r * r:
            blocked += r
    return base + block_penalty * blocked
