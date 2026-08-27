"""The cheap turn gate may only reject what the exact one would.

_pivot_candidate rejects 55% of its candidates on the turn limit, and used to
spend two atan2, a sin and a cos to find that out. cos(turn) = dot / seg_len
costs one multiply-add — but it is not BIT-identical to the atan2 form near the
limit, and turns land on the limit routinely here. So the cheap form only
rejects what is over by more than config.TURN_PREFILTER_BAND_RAD; anything
inside the band falls through to the exact test and the cheap form never decides
a borderline case. These tests hold it to exactly that.
"""

import math

from path_planning import config, planner as astar
from path_planning.scenario import preprocessing as prep, presets as mg


PLANNERS = (astar.KinodynamicAstar,)


def _planners():
    scen = mg.get_all_scenarios()["scenario_01_open_ocean"]()
    pre = prep.prepare_scenario(scen)
    return [cls(pre) for cls in PLANNERS]


def _passes_prefilter(planner, heading, turn, seg_len=50000.0):
    """Reproduce the gate: reject iff dot < guard * seg_len."""
    ux, uy = math.cos(heading), math.sin(heading)
    d = heading + turn
    dx, dy = seg_len * math.cos(d), seg_len * math.sin(d)
    return dx * ux + dy * uy >= planner._turn_cos_guard * seg_len


def test_no_legal_turn_is_ever_rejected_by_the_cheap_gate():
    for planner in _planners():
        amax = planner._alpha_build
        for i in range(201):
            turn = amax * i / 200.0
            for sign in (1.0, -1.0):
                for heading in (0.0, 1.1, -2.7, math.pi):
                    assert _passes_prefilter(planner, heading, sign * turn), (
                        f"cheap gate rejected a legal turn of {math.degrees(turn)} deg"
                    )


def test_a_turn_exactly_on_the_limit_survives_to_the_exact_test():
    """The case that has bitten this codebase repeatedly: geometry built to sit
    ON the limit, then re-measured."""
    for planner in _planners():
        for turn in (
            planner._alpha_build,
            planner._alpha_build + 1e-15,
            planner.alpha_max_rad,
        ):
            assert _passes_prefilter(planner, 0.7, turn), (
                f"{math.degrees(turn)} deg was decided by the cheap gate"
            )


def test_a_turn_well_over_the_limit_is_rejected_cheaply():
    """Otherwise the prefilter would buy nothing."""
    for planner in _planners():
        over = planner._alpha_build + 10 * config.TURN_PREFILTER_BAND_RAD
        assert not _passes_prefilter(planner, 0.7, over)
        assert not _passes_prefilter(planner, 0.7, -over)


def test_the_band_is_far_above_the_dot_products_own_error():
    assert config.TURN_PREFILTER_BAND_RAD >= 1e-9
    assert config.TURN_PREFILTER_BAND_RAD <= 1e-3
