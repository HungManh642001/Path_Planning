"""Pure geometry for the loiter (turn-around) macro.

A state (P, h) has two minimum-radius turning circles tangent to it: one to the
left of the heading (ridden counter-clockwise, sense s = +1) and one to the
right (clockwise, s = -1). Riding such a circle and departing tangentially
reverses the heading in place — the compact "loiter" the discrete radial fan
cannot express without a branching blow-up.

The circle is VIRTUAL (not an obstacle): the aircraft flies ON it. Its radius
is factor*R with factor > 1 so that, when the arc is expanded into
circumscribed-polygon corners at output time (core.arc_geometry.arc_waypoints),
every corner's straight side exceeds its đoản-trình turn reserve.

Sense convention matches core.arc_geometry: s = sign(cross(P - center, h)).
No planner or config imports — all parameters are passed in.
"""
import math


def virtual_center(P, h, r_v, s):
    """Centre of the radius-r_v turning circle tangent to (P, h) with wrap
    sense s (+1 = left / CCW, -1 = right / CW).

    Left of heading h is the unit normal (-sin h, cos h); the CCW circle sits
    there. Chosen so arc_geometry.riding_sense(P, h, center, r_v) == s and
    arc_geometry.tangent_heading(P, center, s) == h.
    """
    if s > 0:
        return (P[0] - r_v * math.sin(h), P[1] + r_v * math.cos(h))
    return (P[0] + r_v * math.sin(h), P[1] - r_v * math.cos(h))


def entry_turn(dphi, theta_step):
    """Turn angle (rad) the circumscribed-polygon expansion introduces AT the
    arc-start point P: the arc is split into n = ceil(dphi/theta_step) equal
    turns and the first chord leaves the tangent by half a step. Used to guard
    P's incoming đoản-trình reserve before emitting a loiter successor."""
    if dphi <= 1e-9:
        return 0.0
    n = max(1, int(math.ceil(dphi / theta_step)))
    return (dphi / n) / 2.0
