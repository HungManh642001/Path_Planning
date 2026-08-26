"""The fan must not fire for a problem it cannot solve: a short d_ss run-in.

In FREE-goal mode `_pivot_candidate` turns the goal candidate away when the
direct leg cannot supply the d_ss run-in after its fillet reserve. The goal is
in the clear; the leg is simply too short. The fan does not help with that --
every fan leg departs at +-alpha_max or straight ahead at a fixed rung, and none
is aimed at the goal -- so firing here only floods the lattice near the target.
Measured over 300 free-goal seeds: 1,108 firings, ZERO waypoints on any
delivered route.

`config.FAN_SKIP_ON_SHORT_RUNIN` drops exactly that case. The boundary matters
more than the case, so most of what is asserted here is what must STILL fire:

* FIXED-goal mode, where the identical rejection means "cannot turn onto
  goal_heading" -- a different problem, 43.6% of firings there, and it carries
  143 route waypoints. Dropping it costs +0.426% with one seed at +40%.
* the occluded-reorientation valve, the no-successor fallback, start corners.

Widening this to every goal-line-of-sight-clear expansion is a different change
and a bad one: it costs seed 51 +73.5%.
"""
import contextlib
import math

from path_planning import config
from path_planning.core import kinodynamic_astar_v0 as astar_v0
from path_planning.core import map_generator as mg
from path_planning.core import preprocessing as prep

START = (0.0, 250000.0)
GOAL = (400000.0, 250000.0)
# Off the goal chord but REACHABLE from the test state: its hull vertices are a
# legal turn away, so Strategy A has successors and the fan gate is the one
# under test rather than the no-successor fallback.
SIDE = [(420000.0, 300000.0), (460000.0, 300000.0), (460000.0, 340000.0), (420000.0, 340000.0)]
# Squarely on the goal chord, for the occluded case.
BLOCKER = [(200000.0, 210000.0), (240000.0, 210000.0), (240000.0, 290000.0), (200000.0, 290000.0)]


def _planner(goal_heading=None, obstacles=(SIDE,)):
    scenario = mg.create_scenario({
        'map_bounds': (config.MAP_WIDTH, config.MAP_HEIGHT),
        'start': START, 'start_heading': 0.0,
        'goal': GOAL, 'goal_heading': goal_heading,
        'num_islands': 0, 'num_dynamic_obstacles': 0, 'seed': 1,
    })
    scenario['islands'] = list(obstacles)
    scenario['obstacles'] = [{'type': 'polygon', 'polygon': p} for p in obstacles]
    return astar_v0.KinodynamicAstar(prep.prepare_scenario(scenario))


@contextlib.contextmanager
def _skip(enabled):
    """The knob is read per expansion, so it must be held across the CALL."""
    previous = config.FAN_SKIP_ON_SHORT_RUNIN
    config.FAN_SKIP_ON_SHORT_RUNIN = enabled
    try:
        yield
    finally:
        config.FAN_SKIP_ON_SHORT_RUNIN = previous


def _state(planner, waypoint, heading=0.0):
    state = astar_v0.State(waypoint, heading)
    state.g_cost = 0.0
    planner.g_scores[state] = 0.0
    return state


def _short_runin_state(planner):
    """A state on the goal chord, too close to fly the d_ss run-in."""
    goal_wp = planner.goal_state.waypoint
    return _state(planner, (goal_wp[0] - 0.5 * planner._dss, goal_wp[1]))


def _fan_legs(planner, state):
    """Successors whose (heading offset, distance) match a fan direction/rung."""
    successors = planner.get_next_states(state)
    offsets = [
        -planner._alpha_build + 2 * planner._alpha_build * i / (config.RADIAL_FAN_DIRECTIONS - 1)
        for i in range(config.RADIAL_FAN_DIRECTIONS)
    ]
    legs = []
    for successor, _ in successors:
        delta = successor.heading - state.heading
        if not any(abs(delta - off) < 1e-9 for off in offsets):
            continue
        distance = math.dist(state.waypoint, successor.waypoint)
        near = math.tan(abs(delta) / 2.0) * planner.R
        if any(abs(distance - (near + rung)) < 1e-6 for rung in planner._fan_rungs):
            legs.append(successor)
    return successors, legs


def test_premise_goal_is_clear_but_the_run_in_is_short():
    """Guard the setup: without this the other assertions prove nothing.

    In particular Strategy A must have a successor here. If it has none, the
    UNCONDITIONAL no-successor fallback fires instead and the gate under test is
    never reached -- which is exactly how the first draft of this test fooled
    itself.
    """
    planner = _planner()
    state = _short_runin_state(planner)
    goal_wp = planner.goal_state.waypoint
    assert planner._check_collision(state.waypoint, goal_wp), 'goal must be in the clear'
    assert planner._pivot_candidate(state, goal_wp, 0.0) is None
    assert planner._last_reject == 'goal', 'must be the d_ss gate, not turn/los/arc'
    with _skip(False):
        successors, legs = _fan_legs(planner, state)
    assert len(successors) > len(legs), 'Strategy A must contribute a successor here'
    assert legs, 'and the legacy fan must fire, or there is nothing to skip'


def test_fan_is_skipped_on_a_short_run_in():
    planner = _planner()
    with _skip(True):
        successors, legs = _fan_legs(planner, _short_runin_state(planner))
    assert successors
    assert not legs, f'fan fired for a d_ss problem it cannot solve: {legs}'


def test_knob_off_restores_the_legacy_fan():
    planner = _planner()
    with _skip(False):
        _, legs = _fan_legs(planner, _short_runin_state(planner))
    assert legs, 'FAN_SKIP_ON_SHORT_RUNIN=False must reproduce legacy firing'


def test_fixed_goal_mode_is_untouched():
    """The same rejection means the terminal-heading problem there, and it pays."""
    planner = _planner(goal_heading=math.radians(90.0))
    assert not planner._free_goal
    state = _state(planner, (planner.goal_state.waypoint[0] - 0.5 * planner._dss,
                             planner.goal_state.waypoint[1]))
    with _skip(True):
        _, legs = _fan_legs(planner, state)
    assert legs, 'fixed-goal mode must keep the fan: it carries 143 route waypoints'


def test_fan_still_fires_when_the_goal_is_occluded():
    planner = _planner(obstacles=(SIDE, BLOCKER))
    state = _state(planner, (100000.0, 250000.0))
    assert not planner._check_collision(state.waypoint, planner.goal_state.waypoint)
    with _skip(True):
        _, legs = _fan_legs(planner, state)
    assert legs, 'the occluded-reorientation valve must be untouched'


def test_fan_still_fires_from_a_start_corner():
    planner = _planner()
    corner = planner.start_corners[-1]
    assert corner.is_start_corner
    planner.g_scores[corner] = corner.g_cost
    with _skip(True):
        _, legs = _fan_legs(planner, corner)
    assert legs, 'start corners are exempt: their fan is takeoff reorientation'


def test_fan_still_fires_when_there_is_no_other_successor():
    """The fallback is unconditional; on open water it is the only generator."""
    planner = _planner(obstacles=())
    state = _state(planner, (200000.0, 250000.0), heading=math.pi)
    with _skip(True):
        successors, legs = _fan_legs(planner, state)
    assert legs, 'the no-successor fallback must never be gated'
    assert len(successors) == len(legs), 'this state should have no Strategy-A successor'
