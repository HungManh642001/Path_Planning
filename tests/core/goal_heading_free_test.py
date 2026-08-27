"""Tests for the free terminal approach mode (goal_heading is None).

When a scenario omits goal_heading (None), the planner chooses the approach
direction itself: the search targets T directly and the final edge into T is a
straight seeker run-in of length >= DSS in a search-chosen direction.
"""

import math

from path_planning import config, planner as astar
from path_planning.scenario import preprocessing as prep, presets as mg


# Comparison slack for a length in metres. Was config.EPS, which is gone: it
# was a dimensionless catch-all and nothing in the planner reads it any more.
_LEN_TOL_M = 1e-6


def _leg_len(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _usable_runin(path, R):
    """USABLE straight seeker run-in: the last-leg length minus the turn fillet
    bite (R*tan(turn/2)) at the previous waypoint. This is what must be >= DSS —
    there has to be room both to bank onto the run-in and for the full DSS
    straight seeker leg."""
    a, b = path[-2][0], path[-1][0]
    bearing = math.atan2(b[1] - a[1], b[0] - a[0])
    turn = abs(
        math.atan2(math.sin(bearing - path[-2][1]), math.cos(bearing - path[-2][1]))
    )
    return _leg_len(a, b) - R * math.tan(turn / 2.0)


def _total_len(path, start_pos, goal_pos):
    """Full O..T length. In fixed mode result['path'] ends at W_{n-1} (short of
    T by the fixed run-in leg), so append T when it is not already the endpoint."""
    pts = [start_pos] + [wp for wp, _h in path]
    if _leg_len(pts[-1], goal_pos) > 1.0:
        pts.append(goal_pos)
    return sum(_leg_len(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


# --------------------------------------------------------------------------- #
# preprocessing
# --------------------------------------------------------------------------- #


def test_prepare_scenario_free_goal_state():
    scenario = mg.scenario1_open_ocean()
    scenario["goal_heading"] = None
    pre = prep.prepare_scenario(scenario)

    assert pre["goal_heading"] is None
    assert pre["goal_state"]["heading"] is None
    assert pre["goal_state"]["waypoint"] == scenario["goal"]
    assert pre["goal_state"]["engagement_distance"] == config.DSS


def test_create_scenario_defaults_goal_heading_to_none():
    # Omitting goal_heading => free mode.
    scenario = mg.create_scenario({"start": (2000, 2000), "goal": (400000, 400000)})
    assert scenario["goal_heading"] is None


# --------------------------------------------------------------------------- #
# end-to-end free-mode planning
# --------------------------------------------------------------------------- #


def test_open_water_free_goal_arrives_straight_at_target():
    scenario = mg.scenario1_open_ocean()
    scenario["goal_heading"] = None
    pre = prep.prepare_scenario(scenario)

    result = astar.plan_trajectory(pre)
    assert result["is_success"], "free-goal open-water plan should succeed"

    path = result["path"]
    T = scenario["goal"]
    # Path ends exactly at T.
    assert path[-1][0] == T
    assert len(path) >= 2

    # Final leg is a straight run-in whose USABLE length (after the turn fillet
    # at the previous waypoint) is >= DSS, heading pointing at T.
    usable = _usable_runin(path, pre["turn_radius"])
    assert usable >= config.DSS - _LEN_TOL_M, (
        f"usable run-in {usable} shorter than DSS {config.DSS}"
    )

    bearing_to_T = math.atan2(T[1] - path[-2][0][1], T[0] - path[-2][0][0])
    dh = abs(
        math.atan2(
            math.sin(path[-1][1] - bearing_to_T), math.cos(path[-1][1] - bearing_to_T)
        )
    )
    assert dh < 1e-3, "arrival heading should point straight at T"


def test_free_goal_with_obstacles_has_clear_run_in():
    scenario = mg.scenario2_single_obstacle()
    scenario["goal_heading"] = None
    pre = prep.prepare_scenario(scenario)

    result = astar.plan_trajectory(pre)
    assert result["is_success"]

    path = result["path"]
    assert path[-1][0] == scenario["goal"]
    # Usable straight run-in (after the turn fillet at the previous waypoint)
    # must be >= DSS: room to bank onto the run-in AND the full seeker leg.
    assert _usable_runin(path, pre["turn_radius"]) >= config.DSS - _LEN_TOL_M
    # The run-in edge is collision-free per the planner's exact check.
    assert result["planner"]._is_collision_free(path[-2][0], path[-1][0])


def test_free_goal_not_worse_than_fixed():
    # Same geometry, once with an explicit approach heading, once free. The free
    # planner has at least as much freedom, so its total path is no longer.
    base = mg.scenario1_open_ocean()

    fixed = dict(base)
    fixed["goal_heading"] = math.pi / 4
    r_fixed = astar.plan_trajectory(prep.prepare_scenario(fixed))

    free = dict(base)
    free["goal_heading"] = None
    r_free = astar.plan_trajectory(prep.prepare_scenario(free))

    assert r_fixed["is_success"] and r_free["is_success"]
    len_fixed = _total_len(r_fixed["path"], base["start"], base["goal"])
    len_free = _total_len(r_free["path"], base["start"], base["goal"])
    assert len_free <= len_fixed * 1.02 + _LEN_TOL_M


# --------------------------------------------------------------------------- #
# fixed mode is unaffected
# --------------------------------------------------------------------------- #


def test_fixed_mode_still_requires_alignment():
    scenario = mg.scenario1_open_ocean()  # explicit goal_heading = pi/4
    assert scenario["goal_heading"] is not None
    pre = prep.prepare_scenario(scenario)
    result = astar.plan_trajectory(pre)
    assert result["is_success"]
    assert result["planner"]._free_goal is False
