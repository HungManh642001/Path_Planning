"""smooth_path is an exact DP over subsequences of the reconstructed path.

The properties that matter are structural, not cosmetic: it must never return a
path the independent oracle rejects, never lengthen one, and never break the
takeoff leg — the O->W1 straight is not in the input path at all, which is what
made the previous greedy shortcutter need an all-or-nothing guard behind it.
"""
import math

import config
import core.map_generator as mg
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.path_validation as pv


def _plan(scenario, smooth):
    pre = prep.prepare_scenario(scenario)
    orig = astar.KinodynamicAstar.smooth_path
    if not smooth:
        astar.KinodynamicAstar.smooth_path = lambda self, path: path
    try:
        return pre, astar.plan_trajectory(pre)
    finally:
        astar.KinodynamicAstar.smooth_path = orig


def _length(full):
    return sum(math.dist(full[i][0], full[i + 1][0])
               for i in range(len(full) - 1))


def _validate(pre, result):
    full = astar._full_mission_path(result['path'], pre)
    return full, pv.path_is_valid(
        full, pre['circle_obstacles'], pre['polygon_obstacles'],
        pre['turn_radius'], pre['alpha_max_rad'],
        pre['start_state']['straight_length'],
        pre['goal_state']['engagement_distance'],
        circle_tol=config.CIRCLE_GRAZE_TOL_M)


# scenario_04 is the preset where smoothing actually bites: the DP drops the
# route to a single interior waypoint and saves ~10 km.
MAZE = 'scenario_04_complex_maze'


def test_smoothing_shortens_the_maze_and_stays_oracle_valid():
    scen = mg.get_all_scenarios()[MAZE]
    pre_off, off = _plan(scen(), smooth=False)
    pre_on, on = _plan(scen(), smooth=True)
    assert off['success'] and on['success']

    full_off = astar._full_mission_path(off['path'], pre_off)
    full_on, (ok, why) = _validate(pre_on, on)
    assert ok, why
    assert _length(full_on) < _length(full_off) - 1.0, 'expected a real shortcut'
    assert len(on['path']) < len(off['path'])


def test_smoothing_never_lengthens_and_stays_valid_across_presets():
    for name, fn in mg.get_all_scenarios().items():
        pre_off, off = _plan(fn(), smooth=False)
        if not off['success']:
            continue
        pre_on, on = _plan(fn(), smooth=True)
        assert on['success'], f'{name}: smoothing lost a solution ({on["failure_reason"]})'
        _full, (ok, why) = _validate(pre_on, on)
        assert ok, f'{name}: {why}'
        l_off = _length(astar._full_mission_path(off['path'], pre_off))
        l_on = _length(_full)
        assert l_on <= l_off + 1.0, f'{name}: smoothing lengthened {l_off} -> {l_on}'


def test_smoothed_path_keeps_the_takeoff_leg_on_its_ray_and_above_L0():
    """The O->W1 leg is not in the path smooth_path receives. The DP puts O in
    the graph precisely so this cannot be broken: the first chord must lie along
    start_heading (no turn is available at O) and the first straight run must
    still clear L0."""
    for name, fn in mg.get_all_scenarios().items():
        pre, res = _plan(fn(), smooth=True)
        if not res['success']:
            continue
        full = astar._full_mission_path(res['path'], pre)
        O = pre['start_pos']
        assert math.dist(O, full[0][0]) < 1.0

        bearing = math.atan2(full[1][0][1] - O[1], full[1][0][0] - O[0])
        drift = abs(math.atan2(math.sin(bearing - pre['start_state']['heading']),
                               math.cos(bearing - pre['start_state']['heading'])))
        assert drift < 1e-6, f'{name}: first chord left the takeoff ray by {drift} rad'

        R = pre['turn_radius']
        alphas = [0.0] + pv.turn_angles(full) + [0.0]
        l1 = math.dist(full[0][0], full[1][0]) - R * math.tan(alphas[1] / 2.0)
        L0 = pre['start_state']['straight_length']
        assert l1 >= L0 - 1.0, f'{name}: takeoff straight {l1:.1f} < L0 {L0}'
