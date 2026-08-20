"""The flown mission path: O -> W_1 ... W_{n-1} -> T.

The planner searches only the INTERIOR waypoints. The takeoff point O and the
target T are constraints, not searched nodes -- W_1 is offset from O by the
takeoff leg and W_{n-1} from T by the seeker run-in -- but the aircraft still
flies O -> W_1 ... W_{n-1} -> T, so anything that measures, validates or draws
the real trajectory has to put them back.

This lived in three places at once: both planners (for the final oracle call in
plan_trajectory) and render.trajectory.build_full_path (for drawing), each a
byte-for-byte copy of the others, each carrying a comment promising it mirrored
the other two. The oracle's verdict and the drawn path MUST come from the same
list of waypoints -- that is the invariant tests/oracle_validity_test.py asserts
-- and three copies is a strange way to guarantee it. It lives in core/ rather
than render/ so the dependency runs render -> core, never the reverse.
"""
import math


def full_mission_path(path, preprocessed):
    """Prepend takeoff O and append goal T around the searched waypoints.

    `path` is a list of (waypoint, heading). Endpoints already present (within
    1 m) are not duplicated, so calling this twice is harmless. `preprocessed`
    may be None or empty, in which case the path is returned unchanged.
    """
    wps = list(path)
    O = preprocessed.get('start_pos') if preprocessed else None
    T = preprocessed.get('goal_pos') if preprocessed else None
    sh = preprocessed.get('start_heading', 0.0) if preprocessed else 0.0
    gh = preprocessed.get('goal_heading', 0.0) if preprocessed else 0.0
    if O is not None and (not wps or math.dist(O, wps[0][0]) > 1.0):
        wps = [(tuple(O), sh)] + wps
    if T is not None and (not wps or math.dist(T, wps[-1][0]) > 1.0):
        # Free-goal mode leaves goal_heading None; the arrival heading is then
        # the bearing of the final leg into T.
        if gh is None:
            gh = math.atan2(T[1] - wps[-1][0][1], T[0] - wps[-1][0][0]) if wps else 0.0
        wps = wps + [(tuple(T), gh)]
    return wps
