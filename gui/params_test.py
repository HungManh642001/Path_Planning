import math
import config
import gui.params as gp


def test_param_specs_cover_groups():
    groups = {s['group'] for s in gp.PARAM_SPECS}
    assert {'tactical', 'run', 'advanced'} <= groups
    keys = {s['key'] for s in gp.PARAM_SPECS}
    assert {'turn_radius', 'alpha_max_deg', 'safe_margin', 'time_budget_s',
            'turn_penalty_weight', 'wrap_step_m'} <= keys


def test_default_values_match_specs():
    dv = gp.default_values()
    for s in gp.PARAM_SPECS:
        assert dv[s['key']] == s['default']


def test_apply_overrides_sets_config_and_returns_prepare_kwargs():
    vals = gp.default_values()
    vals['turn_radius'] = 9000.0
    vals['alpha_max_deg'] = 60.0
    vals['safe_margin'] = 12000.0
    vals['turn_penalty_weight'] = 1234.0
    vals['wrap_step_m'] = 3000.0
    kwargs = gp.apply_overrides(vals)
    assert kwargs['R'] == 9000.0
    assert abs(kwargs['alpha_max_rad'] - math.radians(60.0)) < 1e-9
    assert kwargs['safe_margin'] == 12000.0
    assert config.TURN_PENALTY_WEIGHT == 1234.0
    assert config.WRAP_STEP_M == 3000.0
    assert config.ALPHA_MAX == 60.0
