import math
import preprocessing as prep
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
