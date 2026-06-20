import math
import core.preprocessing as prep
import config


def test_validate_uses_actual_next_turn_when_triple_given():
    # Short colinear triple: legs ~2000 m, all turns zero. With the OLD code
    # (which assumed the next turn = alpha_max), the required straight length
    # would be 2000 - 8000*tan(15 deg) ~= -143 m and the leg would be WRONGLY
    # rejected. With the actual next turn (0), it is correctly accepted.
    w_i = (0.0, 0.0)
    w_next = (2000.0, 0.0)
    w_next_next = (4000.0, 0.0)
    ok, msg = prep.validate_kinodynamics(
        w_i, 0.0, w_next, 0.0,
        w_next_next=w_next_next, heading_next_next=0.0,
        R=8000.0, alpha_max=math.radians(30.0))
    assert ok is True, msg


def test_validate_rejects_too_short_leg():
    w_i = (0.0, 0.0)
    w_next = (100.0, 0.0)   # 100 m leg, far too short for R=8000 turns
    ok, _ = prep.validate_kinodynamics(
        w_i, 0.0, w_next, math.radians(20.0),
        R=8000.0, alpha_max=math.radians(30.0))
    assert ok is False


def test_start_state_distance_matches_spec_with_alpha_max():
    st = prep.calculate_start_state((0.0, 0.0), 0.0,
                                    L0=config.L0, R=config.R,
                                    alpha_max_rad=config.ALPHA_MAX_RAD)
    expected_d = config.L0 + config.R * math.tan(config.ALPHA_MAX_RAD / 2)
    got_d = math.hypot(*st['waypoint'])
    assert abs(got_d - expected_d) < 1e-6
    assert abs(st['straight_length'] - config.L0) < 1e-6


def test_end_state_distance_matches_spec_with_alpha_max():
    end = prep.calculate_end_state((100000.0, 0.0), 0.0,
                                   dss=config.DSS, R=config.R,
                                   alpha_max_rad=config.ALPHA_MAX_RAD)
    expected_d = config.DSS + config.R * math.tan(config.ALPHA_MAX_RAD / 2)
    got_d = math.hypot(100000.0 - end['waypoint'][0], 0.0 - end['waypoint'][1])
    assert abs(got_d - expected_d) < 1e-6


def test_inflation_offsets_two_rings():
    # Ring 1 = safe_margin; ring 2 = safe_margin + turn term R*(1/cos(a/2)-1).
    R, alpha, sm = 8000.0, math.radians(90.0), 10000.0
    ring1, ring2 = prep.inflation_offsets(R, alpha, sm)
    assert abs(ring1 - sm) < 1e-9
    turn_term = R * (1.0 / math.cos(alpha / 2.0) - 1.0)
    assert abs(ring2 - (sm + turn_term)) < 1e-6
    assert ring2 > ring1


def test_inflation_offsets_matches_total_inflation_used_by_inflate():
    # Ring 2 must equal the single total inflation that inflate_obstacles applies,
    # so the outer ring coincides with the planner's actual inflated obstacle.
    R, alpha, sm = 8000.0, math.radians(60.0), 5000.0
    _, ring2 = prep.inflation_offsets(R, alpha, sm)
    raw = [{'type': 'circle', 'center': (0.0, 0.0), 'radius': 20000.0}]
    inflated = prep.inflate_obstacles(raw, R=R, safe_margin=sm, alpha_max_rad=alpha)
    assert abs((inflated[0]['radius'] - 20000.0) - ring2) < 1e-6


def test_prepare_scenario_exposes_safe_margin():
    scenario = {'start': (0.0, 0.0), 'start_heading': 0.0,
                'goal': (100000.0, 0.0), 'goal_heading': 0.0, 'obstacles': []}
    pre = prep.prepare_scenario(scenario, safe_margin=12345.0)
    assert pre['safe_margin'] == 12345.0
