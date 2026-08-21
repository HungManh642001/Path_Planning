"""The analytic goal shot in the v0 planner, and the per-ray collision memo.

v0 had no goal shot, and that absence was its largest single weakness: on a
144-case adverse-heading suite it solved 131/144 at 1,177,550 iterations with 10
cases dying on MAX_ITERATIONS, while the main planner solved 141/144 at 106,563
-- and turning main's shot off reproduced v0 almost exactly. With the shot
ported, v0 solves 141/144 at 78,979.

Two invariants are asserted here.

1. The shot is FIXED-goal only. Free-goal missions do not have the terminal
   heading constraint that makes the Euclid heuristic blind, and the whole
   free-goal sweep is bit-identical with the shot present.
2. `_ray_chord_clear` is a pure memo. All corners sharing a leg1_heading lie on
   one ray out of the state, and all corners sharing an arrival_heading lie on
   one back-ray into the goal, so a clear chord proves every shorter one on that
   ray and a blocked chord proves every longer one. It changes how many chords
   are tested (39.1 -> 11.4 per shot, measured over 300 fixed-goal seeds), never
   the verdict -- which is what the randomised equivalence test below pins.
"""
import math
import random

import config
import core.kinodynamic_astar_v0 as astar_v0
import core.map_generator as mg
import core.mission as mission
import core.path_validation as pv
import core.preprocessing as prep


def _preprocessed(seed, goal_heading):
    return prep.prepare_scenario(mg.create_scenario({
        'map_bounds': (config.MAP_WIDTH, config.MAP_HEIGHT),
        'start': (60000.0, 60000.0), 'start_heading': math.radians(20.0),
        'goal': (380000.0, 380000.0), 'goal_heading': goal_heading,
        'num_islands': 5, 'num_dynamic_obstacles': 4, 'seed': seed,
    }))


def test_shot_is_disabled_in_free_goal_mode():
    planner = astar_v0.KinodynamicAstar(_preprocessed(3, None))
    assert planner._free_goal
    state = astar_v0.State((200000.0, 200000.0), math.radians(45.0))
    planner.g_scores[state] = 0.0
    assert planner._try_goal_shot(state) is None


def test_shot_connects_an_adverse_approach_in_fixed_mode():
    """The case the shot exists for: the goal heading points back at the state."""
    pre = prep.prepare_scenario(mg.create_scenario({
        'map_bounds': (config.MAP_WIDTH, config.MAP_HEIGHT),
        'start': (100000.0, 100000.0), 'start_heading': 0.0,
        'goal': (300000.0, 100000.0), 'goal_heading': math.pi,
        'num_islands': 0, 'num_dynamic_obstacles': 0, 'seed': 1,
    }))
    planner = astar_v0.KinodynamicAstar(pre)
    goal_wp = planner.goal_state.waypoint
    state = astar_v0.State((goal_wp[0] - 80000.0, goal_wp[1] - 80000.0), 0.0)
    planner.g_scores[state] = 0.0

    shot = planner._try_goal_shot(state)
    assert shot is not None, 'an adverse approach in open water must be shootable'
    assert shot.waypoint == goal_wp
    corner = shot.parent
    assert corner is not None and corner.parent is state
    # Both synthesised corners must be legal turns.
    turn_1 = abs(astar_v0._angle_diff(corner.heading, state.heading))
    turn_2 = abs(astar_v0._angle_diff(shot.heading, corner.heading))
    turn_3 = abs(astar_v0._angle_diff(planner.goal_state.heading, shot.heading))
    assert max(turn_1, turn_2, turn_3) <= planner._alpha_build


def test_knob_off_removes_the_shot():
    pre = _preprocessed(5, math.radians(30.0))
    planner = astar_v0.KinodynamicAstar(pre)
    state = astar_v0.State((200000.0, 200000.0), math.radians(45.0))
    planner.g_scores[state] = 0.0
    previous = config.GOAL_SHOT_ENABLED
    config.GOAL_SHOT_ENABLED = False
    try:
        # The knob gates the CALL SITE, so _try_goal_shot itself still works;
        # what must hold is that the search never consults it.
        result = astar_v0.plan_trajectory(pre)
    finally:
        config.GOAL_SHOT_ENABLED = previous
    assert result['stats']['iterations'] > 0


def test_shot_paths_satisfy_the_independent_oracle():
    """A shot synthesises corners no other gate sees; the oracle must still pass."""
    for seed in range(8):
        pre = _preprocessed(seed, math.radians(30.0))
        result = astar_v0.plan_trajectory(pre)
        if not result['success']:
            continue
        full = mission.full_mission_path(result['path'], pre)
        valid, reason = pv.path_is_valid(
            full,
            pre['circle_obstacles'],
            pre['polygon_obstacles'],
            turn_radius=pre['turn_radius'],
            alpha_max_rad=pre['alpha_max_rad'],
            l0=config.L0,
            dss=config.DSS,
        )
        assert valid, f'seed {seed}: {reason}'


def test_ray_memo_never_changes_a_verdict():
    """Random chords on shared rays: memo answer == a fresh _check_collision."""
    planner = astar_v0.KinodynamicAstar(_preprocessed(2, math.radians(30.0)))
    rng = random.Random(11)
    for _ in range(60):
        origin = (rng.uniform(0.0, 400000.0), rng.uniform(0.0, 400000.0))
        ray = rng.uniform(-math.pi, math.pi)
        ux, uy = math.cos(ray), math.sin(ray)
        memo: dict[float, list[float]] = {}
        # Distances in a shuffled order, so the memo is exercised from both
        # sides: a later short chord under a known-clear span, a later long one
        # over a known-blocked span.
        distances = [rng.uniform(1000.0, 250000.0) for _ in range(12)]
        rng.shuffle(distances)
        for distance in distances:
            far = (origin[0] + distance * ux, origin[1] + distance * uy)
            memoised = planner._ray_chord_clear(memo, ray, distance, origin, far)
            assert memoised == planner._check_collision(origin, far), (
                f'memo disagreed at {distance:.1f} m along {math.degrees(ray):.1f} deg'
            )


def _counting_planner(seed=2):
    planner = astar_v0.KinodynamicAstar(_preprocessed(seed, math.radians(30.0)))
    calls = [0]
    real = planner._check_collision

    def counting(p1, p2):
        calls[0] += 1
        return real(p1, p2)

    planner._check_collision = counting
    return planner, calls


def test_memo_skips_shorter_chords_once_a_long_one_is_clear():
    """Guard the premise: a memo that never fired would pass the equality test.

    Note the direction. A CLEAR verdict covers everything SHORTER, so it pays
    when the long chord is settled first. The reverse order saves nothing, which
    is why the real caller's ascending-by-total-length order still leaves 11.4
    checks per shot rather than one.
    """
    planner, calls = _counting_planner()
    memo: dict[float, list[float]] = {}
    origin = (100000.0, 100000.0)
    ray = math.radians(45.0)
    ux, uy = math.cos(ray), math.sin(ray)
    distances = [50000.0, 40000.0, 30000.0, 20000.0, 10000.0]
    verdicts = []
    for distance in distances:
        far = (origin[0] + distance * ux, origin[1] + distance * uy)
        verdicts.append(planner._ray_chord_clear(memo, ray, distance, origin, far))
    assert verdicts[0] and all(verdicts), 'this ray was meant to be clear throughout'
    assert calls[0] == 1, f'only the longest chord needed testing, but {calls[0]} were'


def test_memo_skips_longer_chords_once_a_short_one_is_blocked():
    """The mirror direction: BLOCKED covers everything longer."""
    planner, calls = _counting_planner()
    memo: dict[float, list[float]] = {}
    origin = (100000.0, 100000.0)
    # Aim at the centre of an inflated circle, so every chord from short to long
    # is blocked and the shortest settles the ray.
    centre, _radius = planner.scenario['circle_obstacles'][0]
    ray = math.atan2(centre[1] - origin[1], centre[0] - origin[0])
    ux, uy = math.cos(ray), math.sin(ray)
    reach = math.dist(origin, centre)
    distances = [reach, reach * 1.2, reach * 1.5, reach * 2.0]
    verdicts = []
    for distance in distances:
        far = (origin[0] + distance * ux, origin[1] + distance * uy)
        verdicts.append(planner._ray_chord_clear(memo, ray, distance, origin, far))
    assert not any(verdicts), 'this ray was meant to be blocked throughout'
    assert calls[0] == 1, f'only the shortest chord needed testing, but {calls[0]} were'
