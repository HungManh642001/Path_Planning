"""Tests for seeded start-corner states — real-angle first-turn placement
replacing the alpha_max worst-case W1.

Instead of a single start waypoint W1 placed at the alpha_max worst-case
offset `L0 + R*tan(alpha_max/2)` along start_heading, the planner seeds
`config.NUM_START_CORNERS` (K) corner states at `d_i = L0 + R*tan(a_i/2)`,
tan-uniform buckets `tan(a_i/2) = (i/K)*tan(alpha_max/2)` for i = 1..K.
Bucket K reproduces the legacy W1 exactly, so K=1 is a strict A/B knob
against historical behaviour. Corners outside the safezone bounds or whose
takeoff leg O->corner collides are not seeded; if none survive, the plan
fails.
"""

import math

from shapely.geometry import Polygon, Point

from path_planning import config
from path_planning.core import map_generator as mg
from path_planning.core import preprocessing as prep
from path_planning.core import kinodynamic_astar as astar
from path_planning.core import path_validation as pv
from path_planning.render import trajectory as tr


def _plan(scenario, k, monkeypatch):
    """A/B helper: seed K start corners, prepare + plan, return (pre, result).

    NUM_START_CORNERS is read in the planner constructor, so it must be
    monkeypatched before prepare_scenario/plan_trajectory run.
    """
    monkeypatch.setattr(config, 'NUM_START_CORNERS', k)
    pre = prep.prepare_scenario(scenario)
    result = astar.plan_trajectory(pre)
    return pre, result


def _path_length(start_pos, path):
    """Total length of [start_pos] + waypoints of path (consecutive chords)."""
    pts = [start_pos] + [wp for wp, _ in path]
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def _diagonal_corridor_scenario():
    # Diagonal (45deg) safezone corridor from the start; start heads EAST, so the
    # first turn onto the corridor is ~45deg. The legacy single W1 (12 km east)
    # pokes OUTSIDE the band -> legacy seeds 0 corners and fails; nearer corners
    # stay inside and afford the moderate turn.
    sz = [(44343, 55657), (55657, 44343), (205657, 194343), (194343, 205657)]
    return {
        'start': (50000.0, 50000.0), 'start_heading': 0.0,
        'goal': (150000.0, 150000.0), 'goal_heading': None,
        'obstacles': [], 'islands': [], 'dynamic_obstacles': [],
        'safezones': [sz],
    }


def test_feasibility_diagonal_corridor(monkeypatch):
    scenario = _diagonal_corridor_scenario()

    # A: legacy single-corner W1 pokes outside the diagonal band -> no corners
    # survive seeding, and the plan fails.
    _, result_k1 = _plan(scenario, 1, monkeypatch)
    assert result_k1['success'] is False
    assert len(result_k1['planner'].start_corners) == 0

    # B: 8 tan-uniform corners include at least one that stays inside the band
    # and affords the ~45deg turn -> succeed.
    pre_k8, result_k8 = _plan(scenario, 8, monkeypatch)
    assert result_k8['success'] is True
    assert len(result_k8['planner'].start_corners) >= 1

    full_path = tr.build_full_path(result_k8['path'], pre_k8)
    ok, detail = pv.straight_segments_ok(
        full_path, config.R, config.L0, pre_k8['goal_state']['engagement_distance']
    )
    assert ok, detail

    poly = Polygon(scenario['safezones'][0])
    for waypoint in [wp for wp, _ in result_k8['path']]:
        assert poly.covers(Point(waypoint)), f"waypoint {waypoint} left the safezone"


def test_feasibility_needs_multiple_corners(monkeypatch):
    # Same diagonal scenario: seeding multiple corners (not a single anchor)
    # is what recovers feasibility.
    scenario = _diagonal_corridor_scenario()

    _, result_k1 = _plan(scenario, 1, monkeypatch)
    k1_corners = len(result_k1['planner'].start_corners)
    assert k1_corners == 0
    assert result_k1['success'] is False

    _, result_k8 = _plan(scenario, 8, monkeypatch)
    k8_corners = len(result_k8['planner'].start_corners)
    assert k8_corners > k1_corners
    assert result_k8['success'] is True


def test_smoothed_path_keeps_l1_at_least_l0(monkeypatch):
    scenario = _diagonal_corridor_scenario()
    pre, result = _plan(scenario, 8, monkeypatch)
    assert result['success'] is True

    path = result['path']
    p0 = path[0][0]
    p1 = path[1][0]

    bearing = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    alpha1 = abs(math.atan2(
        math.sin(bearing - scenario['start_heading']),
        math.cos(bearing - scenario['start_heading']),
    ))

    l1 = math.dist(scenario['start'], p0) - config.R * math.tan(alpha1 / 2.0)
    assert l1 >= config.L0 - 1.0

    # Belt-and-braces: the independent oracle agrees.
    full_path = tr.build_full_path(path, pre)
    ok, detail = pv.straight_segments_ok(
        full_path, config.R, config.L0, pre['goal_state']['engagement_distance']
    )
    assert ok, detail


def test_backward_compat_k1_equals_legacy(monkeypatch):
    scenario = mg.scenario1_open_ocean()
    pre, result = _plan(scenario, 1, monkeypatch)

    planner = result['planner']
    assert len(planner.start_corners) == 1

    legacy_w1 = pre['start_state']['waypoint']
    corner_wp = planner.start_corners[0].waypoint
    # 1e-3 m, not 1e-6: construction now pads towards feasibility, so bucket K
    # sits GEOM_EPS_M + R*(tan(amax/2) - tan((amax - GEOM_EPS_RAD)/2)) ~ 8 um
    # short of the legacy W1. Still tight enough to catch a real formula change,
    # which would move it by metres.
    assert abs(corner_wp[0] - legacy_w1[0]) < 1e-3
    assert abs(corner_wp[1] - legacy_w1[1]) < 1e-3

    assert result['success'] is True
    # The search roots exactly at the (single) seeded corner.
    assert abs(result['path'][0][0][0] - legacy_w1[0]) < 1e-3
    assert abs(result['path'][0][0][1] - legacy_w1[1]) < 1e-3


def test_gentle_adverse_heading_not_longer(monkeypatch):
    # Real first turn is only ~60 deg (< alpha_max = 90 deg): K=8's finer
    # angular buckets should not force a longer route than the legacy K=1
    # worst-case placement.
    scenario = {
        'start': (100000.0, 100000.0),
        'start_heading': math.radians(60),
        'goal': (300000.0, 100000.0),  # due east of start
        'goal_heading': None,
        'obstacles': [],
        'islands': [],
        'dynamic_obstacles': [],
    }

    pre_k1, result_k1 = _plan(scenario, 1, monkeypatch)
    pre_k8, result_k8 = _plan(scenario, 8, monkeypatch)

    assert result_k1['success'] is True
    assert result_k8['success'] is True

    len_k1 = _path_length(scenario['start'], result_k1['path'])
    len_k8 = _path_length(scenario['start'], result_k8['path'])
    assert len_k8 <= len_k1 + 1.0
