"""Pure geometry for the analytic terminal "goal shot": enumerate 2-corner
vehicle maneuvers that connect a search state (P, h) to the goal waypoint,
arriving with a heading inside the +-alpha_max terminal cone.

A candidate is: turn <= alpha_max at P onto leg 1 (direction d1), fly straight
to an intermediate corner C, turn <= alpha_max at C onto leg 2 (direction phi),
fly straight to the goal, arriving heading phi (within alpha_max of the goal
heading so the terminal turn onto the approach is feasible). C is the
intersection of the ray from P along d1 and the back-ray into the goal along
phi. No planner/config imports; all tolerances are parameters.
"""
import math


def _angdiff(a, b):
    """Smallest signed difference a-b normalised to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def two_corner_candidates(P, h, goal_wp, goal_heading, R, alpha_max,
                          min_straight, straight_budget_in, min_straight_in,
                          num_dir=9, num_cone=9):
    """Feasible (angle + length) 2-corner maneuvers, shortest first.

    Args:
        P: current waypoint (x, y).
        h: current heading (rad).
        goal_wp: goal waypoint W_{n-1} (x, y).
        goal_heading: required approach heading at the goal (rad).
        R: turn radius (m).
        alpha_max: max turn angle (rad).
        min_straight: minimum usable straight length per leg (m).
        straight_budget_in: remaining straight budget of the leg INTO P
            (deferred đoản-trình: P's incoming leg must still keep
            min_straight_in after P's turn reserve).
        min_straight_in: đoản-trình threshold for P's incoming leg.
        num_dir: number of turn-at-P directions sampled across [h ± alpha_max].
            Must be >= 2 (the sampler divides by num_dir - 1).
        num_cone: number of arrival headings sampled across
            [goal_heading ± alpha_max]. Must be >= 2 (divides by num_cone - 1).

    Returns:
        list of (total_len, C, d1, phi, budget_C, budget_W), sorted by
        total_len ascending. Empty if nothing is angle/length feasible.
    """
    Px, Py = P
    Dx, Dy = goal_wp[0] - Px, goal_wp[1] - Py
    out = []
    for i in range(num_dir):
        d1 = h - alpha_max + (2.0 * alpha_max) * i / (num_dir - 1)
        a1 = abs(_angdiff(d1, h))                      # turn at P
        # Deferred đoản-trình of P's incoming leg (near reserve = R*tan(a1/2)).
        if straight_budget_in - R * math.tan(a1 / 2.0) < min_straight_in:
            continue
        Ux, Uy = math.cos(d1), math.sin(d1)
        r1 = R * math.tan(a1 / 2.0)                     # leg1 near reserve (at P)
        for j in range(num_cone):
            phi = goal_heading - alpha_max + (2.0 * alpha_max) * j / (num_cone - 1)
            a2 = abs(_angdiff(phi, d1))                 # turn at C
            if a2 > alpha_max:
                continue
            at = abs(_angdiff(goal_heading, phi))       # terminal turn at goal
            if at > alpha_max:                          # (guard float on cone edge)
                continue
            Vx, Vy = math.cos(phi), math.sin(phi)
            det = Ux * Vy - Uy * Vx
            if abs(det) < 1e-9:
                continue                                # legs parallel: no corner
            t = (Dx * Vy - Dy * Vx) / det               # leg1 length P->C
            u = (Ux * Dy - Uy * Dx) / det               # leg2 length C->goal
            if t <= 0.0 or u <= 0.0:
                continue                                # corner behind an endpoint
            r2 = R * math.tan(a2 / 2.0)                 # reserve at C
            rt = R * math.tan(at / 2.0)                 # terminal reserve at goal
            budget_C = t - r1                           # leg1 minus its near reserve
            if budget_C - r2 < min_straight:            # leg1 far reserve + straight
                continue
            budget_W = u - r2                           # leg2 minus its near reserve
            if budget_W - rt < min_straight:            # leg2 far reserve + straight
                continue
            C = (Px + t * Ux, Py + t * Uy)
            out.append((t + u, C, d1, phi, budget_C, budget_W))
    out.sort(key=lambda c: c[0])
    return out
