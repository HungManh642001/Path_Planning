"""Strategy-B fan distance ladder (config.NUM_FAN_DISTANCES).

A fan leg must cover near reserve + far reserve + the đoản-trình minimum:

    d_j = R*tan(theta/2) + R*tan(beta_j/2) + RADIAL_FAN_STEP_M

theta is the fan's own turn at P (known at creation); beta_j is the NEXT turn
at the pivot, which _doan_trinh defers. The legacy fan hardcoded
beta = alpha_max, so every leg paid the worst-case far reserve even when the
pivot barely turns — an unconditional bulge on fan-routed paths in open water.

Rung j is now "the shortest leg that still affords a next turn beta <= beta_j",
tan-uniform exactly like the seeded start corners.
"""
import math

import pytest

from path_planning import config
from path_planning.core import preprocessing as prep
from path_planning.core import kinodynamic_astar as astar


def open_water_scenario():
    """No obstacles: the fan is the only source of off-chord successors."""
    return {
        'start': (100000.0, 250000.0), 'start_heading': 0.0,
        'goal': (400000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
    }


def _planner():
    pre = prep.prepare_scenario(open_water_scenario())
    return astar.KinodynamicAstar(pre), pre


def _fan_legs(planner, st, heading_offset):
    """Distances of the fan successors emitted along one fan direction.

    Fan successors are identified by their heading: the fan is the only
    producer of states whose heading is exactly h + heading_offset, since
    Strategy-A candidates take their heading from the bearing to a tangent
    point / vertex / the goal. The goal itself is excluded explicitly — on the
    straight-ahead branch it shares the fan's heading.
    """
    want = st.heading + heading_offset
    goal_wp = planner.goal_state.waypoint
    out = []
    for nxt, _cost in planner.get_next_states(st):
        if nxt.waypoint == goal_wp:
            continue
        if abs(nxt.heading - want) < 1e-9:
            out.append(math.dist(st.waypoint, nxt.waypoint))
    return sorted(out)


def _expected_ladder(planner, heading_offset, m=None):
    m = config.NUM_FAN_DISTANCES if m is None else m
    near = planner.R * math.tan(abs(heading_offset) / 2.0)
    tan_half_max = math.tan(planner._alpha_build / 2)
    return [near + planner.R * (j / m) * tan_half_max + config.RADIAL_FAN_STEP_M
            for j in range(1, m + 1)]


def test_ladder_distances_match_the_reserve_formula():
    planner, pre = _planner()
    st = astar.State(pre['start_state']['waypoint'], pre['start_state']['heading'])
    for offset in (-planner._alpha_build, 0.0, planner._alpha_build):
        legs = _fan_legs(planner, st, offset)
        assert legs == pytest.approx(_expected_ladder(planner, offset))


def test_every_rung_affords_exactly_its_own_turn_bucket():
    """The capability invariant: rung j leaves room for a next turn beta_j but
    NOT for beta_{j+1}. This is what makes each rung earn its place — without
    it the ladder is just an arbitrary set of lengths."""
    planner, pre = _planner()
    st = astar.State(pre['start_state']['waypoint'], pre['start_state']['heading'])
    m = config.NUM_FAN_DISTANCES
    tan_half_max = math.tan(planner._alpha_build / 2)
    offset = planner._alpha_build

    succ = [(nxt, math.dist(st.waypoint, nxt.waypoint))
            for nxt, _c in planner.get_next_states(st)
            if abs(nxt.heading - (st.heading + offset)) < 1e-9]
    succ.sort(key=lambda t: t[1])
    assert len(succ) == m

    for j, (nxt, _d) in enumerate(succ, start=1):
        afforded = planner.R * (j / m) * tan_half_max
        # Its own bucket fits (this is the float-boundary guard too: the rung
        # is built to land on the threshold, so RADIAL_FAN_STEP_M must keep it
        # clear of round-trip noise in _doan_trinh's tan recomputation).
        assert nxt.straight_budget - afforded >= astar._MIN_STRAIGHT_M
        if j < m:
            nxt_bucket = planner.R * ((j + 1) / m) * tan_half_max
            assert nxt.straight_budget - nxt_bucket < astar._MIN_STRAIGHT_M


def test_shortest_rung_is_shorter_than_the_legacy_worst_case():
    """The whole point: the search gets a tight pivot option instead of always
    paying the full alpha_max far reserve."""
    planner, pre = _planner()
    st = astar.State(pre['start_state']['waypoint'], pre['start_state']['heading'])
    legs = _fan_legs(planner, st, planner._alpha_build)
    legacy = (2 * planner.R * math.tan(planner._alpha_build / 2) + 1000.0)
    assert legs[0] < legacy
    assert legs[-1] == pytest.approx(_expected_ladder(planner, planner._alpha_build)[-1])


def test_rung_spacing_stays_above_the_dedup_quantum():
    """Rungs closer together than STATE_POS_QUANTUM collapse onto the same
    lattice cell, so the extra successors would be wasted branching."""
    planner, _pre = _planner()
    spacing = planner.R * math.tan(planner._alpha_build / 2) / config.NUM_FAN_DISTANCES
    assert spacing > config.STATE_POS_QUANTUM
    # M = 8 is the documented ceiling; M = 16 must violate it.
    assert planner.R * math.tan(planner._alpha_build / 2) / 16 < config.STATE_POS_QUANTUM


def test_single_rung_with_legacy_pad_reproduces_legacy_distance(monkeypatch):
    """A/B knob, mirroring NUM_START_CORNERS = 1: M = 1 plus the old 1000 m pad
    is exactly the legacy single worst-case leg."""
    monkeypatch.setattr(config, 'NUM_FAN_DISTANCES', 1)
    monkeypatch.setattr(config, 'RADIAL_FAN_STEP_M', 1000.0)
    planner, pre = _planner()
    st = astar.State(pre['start_state']['waypoint'], pre['start_state']['heading'])
    legs = _fan_legs(planner, st, planner._alpha_build)
    assert legs == pytest.approx(
        [2 * planner.R * math.tan(planner._alpha_build / 2) + 1000.0])
