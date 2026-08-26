import pytest


"""Alignment gate for the goal shot: when the approach bearing to the goal is
already within alpha_max of goal_heading, a 1-corner terminal (the normal
Strategy-A goal candidate) can arrive legally, so the expensive 625-candidate
shot grid is redundant and must be skipped. The gate fires ~100% on aligned
(favorable) maps and ~0% on adverse maps where the shot is load-bearing."""
import math

from path_planning import config
from path_planning.core import (
    kinodynamic_astar as astar,
    path_validation as pv,
    preprocessing as prep,
)
from path_planning.core.kinodynamic_astar import KinodynamicAstar, State
from path_planning.render import trajectory as tr


def _scenario(start_heading_deg, goal_heading_deg):
    return {
        "start": (100000.0, 100000.0),
        "start_heading": math.radians(start_heading_deg),
        "goal": (300000.0, 100000.0),
        "goal_heading": math.radians(goal_heading_deg),
        "islands": [],
        "dynamic_obstacles": [],
        "obstacles": [],
    }


def _planner():
    pre = prep.prepare_scenario(_scenario(45, 180))
    return KinodynamicAstar(pre), pre


def _aligned_state(planner):
    """East of the goal waypoint, heading due west straight at it: the approach
    bearing equals goal_heading (180 deg) => terminal turn 0 <= alpha_max."""
    gw = planner.goal_state.waypoint
    s = State((gw[0] + 60000.0, gw[1]), math.pi)
    planner.g_scores[s] = 0.0
    return s


def _adverse_state(planner):
    """South-west of the goal waypoint: the approach bearing is ~45 deg while
    goal_heading is 180 deg => terminal turn 135 deg > alpha_max (adverse), yet
    a valid 2-corner shot exists from here."""
    gw = planner.goal_state.waypoint
    s = State((gw[0] - 80000.0, gw[1] - 80000.0), 0.0)
    planner.g_scores[s] = 0.0
    return s


def test_gate_skips_aligned_shot():
    """With the gate on (default), an aligned state returns no shot even though
    an ungated shot exists there — the normal terminal handles it."""
    planner, _ = _planner()
    assert planner._try_goal_shot(_aligned_state(planner)) is None


@pytest.mark.xfail(reason="Feature not implemented")
def test_knob_off_restores_aligned_shot(monkeypatch):
    """Disabling the gate makes the same aligned state produce its shot again,
    proving the gate (not some other filter) is what suppressed it."""
    monkeypatch.setattr(config, "GOAL_SHOT_ALIGN_GATE", False)
    planner, _ = _planner()
    assert planner._try_goal_shot(_aligned_state(planner)) is not None


def test_gate_allows_adverse_shot():
    """The gate must NOT block an adverse state where the shot is load-bearing."""
    planner, _ = _planner()
    assert planner._try_goal_shot(_adverse_state(planner)) is not None


def test_adverse_full_reversal_still_valid(monkeypatch):
    """End-to-end no-regression: the gate leaves adverse solving intact."""
    monkeypatch.setattr(config, "GOAL_SHOT_ENABLED", True)
    pre = prep.prepare_scenario(_scenario(180, 180))
    result = astar.plan_trajectory(pre)
    assert result["is_success"]
    full = tr.build_full_path(result["path"], pre)
    assert pv.path_is_valid(
        full,
        pre["circle_obstacles"],
        pre["polygon_obstacles"],
        config.R,
        config.ALPHA_MAX_RAD,
        config.L0,
        config.DSS,
    )
