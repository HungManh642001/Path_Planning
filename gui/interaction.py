"""Pure (Tk-free) interaction helpers for the planner GUI.

Kept separate from the Tk panels so the logic that matters -- parsing user
input and turning a mouse drag into a heading -- is unit-testable headlessly.
"""

import math


def parse_param(text, default):
    """Parse a parameter entry's text to float, falling back to `default`.

    GUI entry fields are free text: a user may clear a field to retype it or
    fat-finger a non-number. Returning the default (instead of raising) keeps
    RUN from crashing on a transiently-invalid field.
    """
    try:
        return float(str(text).strip())
    except (TypeError, ValueError):
        return float(default)


def heading_from_drag(p0, p1, min_dist):
    """Heading (radians) of the vector p0->p1, or None if the drag is shorter
    than `min_dist` (i.e. the user clicked rather than aimed)."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    if math.hypot(dx, dy) < min_dist:
        return None
    return math.atan2(dy, dx)
