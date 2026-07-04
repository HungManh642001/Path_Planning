"""Pure geometry for arc-hop successors: riding detection, tangent headings,
bitangents between circles, departure points, and output-time arc expansion.

Wrap sense convention: s = +1 rides a circle counter-clockwise, s = -1
clockwise; s = sign(cross(P - center, heading_vector)).
No planner or config imports — all tolerances/steps are parameters.
"""
import math

import core.spatial_utils as su


def riding_sense(P, h, center, r, pos_tol=1.0, ang_tol=8.72e-3):
    """0 if (P, h) does not ride circle (center, r); else the wrap sense ±1.

    Riding means: P on the boundary (within pos_tol meters) AND heading h
    tangent to the circle at P (|dot(radial, heading)| < ang_tol ~ sin 0.5°).
    """
    dx, dy = P[0] - center[0], P[1] - center[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-9 or abs(dist - r) > pos_tol:
        return 0
    ux, uy = dx / dist, dy / dist
    hx, hy = math.cos(h), math.sin(h)
    if abs(ux * hx + uy * hy) > ang_tol:
        return 0
    return 1 if (ux * hy - uy * hx) > 0 else -1


def tangent_heading(P, center, s):
    """Heading (radians) of tangent travel at boundary point P, wrap sense s."""
    return math.atan2(s * (P[0] - center[0]), -s * (P[1] - center[1]))


def arc_angle(P, Q, center, s):
    """Angle in [0, 2π) swept from P to Q around center, in direction s."""
    a0 = math.atan2(P[1] - center[1], P[0] - center[0])
    a1 = math.atan2(Q[1] - center[1], Q[0] - center[0])
    return (s * (a1 - a0)) % (2.0 * math.pi)
