"""End-to-end tests: the goal shot collapses the adverse-heading flood."""
import math

from path_planning import config
from path_planning.core import preprocessing as prep
from path_planning.core import kinodynamic_astar as astar
from path_planning.core import path_validation as pv
from path_planning.render import trajectory as tr


def _adverse_scenario(start_heading_deg, goal_heading_deg):
    """Open water, start->goal due east 200 km, with adverse headings."""
    return {
        'start': (100000.0, 100000.0),
        'start_heading': math.radians(start_heading_deg),
        'goal': (300000.0, 100000.0),
        'goal_heading': math.radians(goal_heading_deg),
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
    }


def _oracle_ok(result, pre):
    full = tr.build_full_path(result['path'], pre)
    return pv.path_is_valid(
        full, pre['circle_obstacles'], pre['polygon_obstacles'],
        config.R, config.ALPHA_MAX_RAD, config.L0, config.DSS)


def test_shot_solves_adverse_in_few_iterations(monkeypatch):
    monkeypatch.setattr(config, 'GOAL_SHOT_ENABLED', True)
    pre = prep.prepare_scenario(_adverse_scenario(45, 180))
    result = astar.plan_trajectory(pre)
    assert result['success']
    # inject-into-open keeps normal A* ordering (quality-safe), so the shot
    # is accepted only once it surfaces as cheapest rather than short-
    # circuiting the search immediately; it still collapses the flood far
    # below the ~19000-iteration baseline (45/180 measures ~760).
    assert result['stats']['iterations'] < 2000
    assert _oracle_ok(result, pre)


def test_shot_disabled_still_floods(monkeypatch):
    monkeypatch.setattr(config, 'GOAL_SHOT_ENABLED', False)
    pre = prep.prepare_scenario(_adverse_scenario(45, 180))
    result = astar.plan_trajectory(pre)
    assert result['success']
    assert result['stats']['iterations'] > 1000     # no shot => flood


def test_shot_valid_on_full_reversal(monkeypatch):
    monkeypatch.setattr(config, 'GOAL_SHOT_ENABLED', True)
    pre = prep.prepare_scenario(_adverse_scenario(180, 180))
    result = astar.plan_trajectory(pre)
    assert result['success']
    assert _oracle_ok(result, pre)


def test_free_goal_unaffected(monkeypatch):
    monkeypatch.setattr(config, 'GOAL_SHOT_ENABLED', True)
    scen = _adverse_scenario(180, 0)
    scen['goal_heading'] = None                      # free-goal mode
    pre = prep.prepare_scenario(scen)
    result = astar.plan_trajectory(pre)
    assert result['success']
    assert _oracle_ok(result, pre)
