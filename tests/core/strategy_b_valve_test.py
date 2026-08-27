import pytest


"""Escape-valve budget and fan-reach regressions (results_fail vs results_v2).

The K seeded start corners are all expanded while the goal is still occluded,
so under a global NUM_STRATEGY_B budget of 3 the K=4 corners drained the valve
at takeoff and no mid-course reorientation fan could ever fire. Separately,
the fan reach was halved (2R·tan(α/2)+step → R·tan(α/2)+step) which left fan
points too short to bridge constrained goal-approach slots (seed 4: +88 km).
"""
import math

from path_planning import config, planner as astar
from path_planning.scenario import preprocessing as prep
from path_planning.scenario.generator import generate_random_scenario


@pytest.fixture
def no_time_budget(monkeypatch):
    """A budget far above what these seeds need, so the clock never decides."""
    monkeypatch.setattr(config, "TIME_BUDGET_S", 600.0)


def _mission_km(pre, res):
    pts = [pre["start_pos"]] + [p for p, _h in res["path"]] + [pre["goal_pos"]]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:])) / 1000.0


def _planner_for_seed(seed):
    pre = prep.prepare_scenario(generate_random_scenario(seed=seed))
    return astar.KinodynamicAstar(pre), pre


def test_corner_expansions_do_not_consume_valve_budget():
    # Seed 4: the goal is occluded from every start corner (the engage point
    # sits in a slot behind the goal island), so each corner expansion hits
    # the valve gate. Corners must not drain the global budget.
    planner, _pre = _planner_for_seed(4)
    assert len(planner.start_corners) > 1
    for corner in planner.start_corners:
        succ = planner.get_next_states(corner)
        assert succ, "corner expansion should produce successors"
    assert planner.num_strategy_b == config.NUM_STRATEGY_B, (
        "start-corner expansions drained the Strategy-B valve budget"
    )


def test_non_corner_expansion_still_consumes_valve_budget():
    # A regular state at a corner's position (same occluded-goal situation)
    # must keep decrementing the budget — the exemption is corners-only.
    planner, _pre = _planner_for_seed(4)
    corner = planner.start_corners[-1]
    st = astar.State(corner.waypoint, corner.heading)
    before = planner.num_strategy_b
    succ = planner.get_next_states(st)
    assert succ
    assert planner.num_strategy_b == before - 1


def test_fan_reach_covers_worst_case_far_reserve():
    """The LONGEST fan leg must still reserve a full alpha_max turn at both
    ends, so a pivot can bridge a constrained goal-approach slot (seed 4).

    Asserted on the distance the fan actually emits, not on an internal
    variable: the near reserve moved out of the precomputed constant and into
    a per-direction term, and the ladder later split the far reserve into
    NUM_FAN_DISTANCES buckets — both refactors are invisible here, while a
    genuine loss of reach is not.
    """
    planner, _pre = _planner_for_seed(4)
    offset = planner.alpha_max_rad
    near = planner.R * math.tan(offset / 2.0)
    far = planner.R * math.tan(planner.alpha_max_rad / 2.0)
    assert near + planner._fan_rungs[-1] == pytest.approx(
        near + far + config.RADIAL_FAN_STEP_M
    )


def test_seed4_goal_slot_no_long_detour(no_time_budget):
    # Historic: 534.9 km (fan reach too short to pivot into the goal slot);
    # with the fan restored the planner finds ~446.9 km. Bound is loose.
    planner, pre = _planner_for_seed(4)
    res = astar.plan_trajectory(pre, verbose=False)
    assert res["is_success"]
    assert _mission_km(pre, res) < 480.0


def test_seed964_valve_starvation_no_long_detour(no_time_budget):
    # Historic: 546.9 km (valve drained by the 4 corners at takeoff);
    # with corners exempt the planner finds ~481.2 km. Bound is loose.
    planner, pre = _planner_for_seed(964)
    res = astar.plan_trajectory(pre, verbose=False)
    assert res["is_success"]
    assert _mission_km(pre, res) < 510.0
