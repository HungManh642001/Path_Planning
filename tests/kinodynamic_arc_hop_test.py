"""Planner-level tests for arc-hop successor generation (synthetic maps)."""
import math

import config
import core.preprocessing as prep
import core.kinodynamic_astar as astar
import core.arc_geometry as ag
import core.path_validation as pv
import render.trajectory as tr

CENTER = (250000.0, 250000.0)
RAW_R = 30000.0


def synthetic_circle_scenario():
    """One raw circle dead-center between a west start and an east goal."""
    return {
        'start': (50000.0, 250000.0), 'start_heading': 0.0,
        'goal': (450000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [],
        'dynamic_obstacles': [(CENTER, RAW_R)],
        'obstacles': [{'type': 'circle', 'center': CENTER, 'radius': RAW_R}],
    }


def test_arc_hop_successors_from_riding_state():
    pre = prep.prepare_scenario(synthetic_circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    (_, r_inf), = pre['circle_obstacles']
    P = (CENTER[0], CENTER[1] - r_inf)  # due south, heading east => CCW
    st = astar.State(P, 0.0)
    succ = planner._arc_hop_successors(st)
    assert succ, "a riding state must generate arc-hop successors"
    for nxt, cost in succ:
        center, radius, arc_start, s = nxt.arc_from
        assert (center, radius, s) == (CENTER, r_inf, 1)
        assert arc_start == P
        dphi = ag.arc_angle(P, nxt.waypoint, center, s)
        assert math.isclose(cost, radius * dphi, rel_tol=1e-9)
        assert math.isclose(
            math.hypot(nxt.waypoint[0] - center[0], nxt.waypoint[1] - center[1]),
            radius, rel_tol=1e-9)
    # The goal's departure point must be among the successors.
    dep_goal = ag.departure_point(pre['goal_state']['waypoint'], CENTER, r_inf, 1)
    assert any(math.dist(nxt.waypoint, dep_goal) < 1.0 for nxt, _ in succ)


def test_non_riding_state_has_no_arc_hops():
    pre = prep.prepare_scenario(synthetic_circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    st = astar.State((50000.0, 50000.0), 0.0)  # far from any boundary
    assert planner._arc_hop_successors(st) == []


def test_synthetic_circle_end_to_end_valid():
    scn = synthetic_circle_scenario()
    pre = prep.prepare_scenario(scn)
    result = astar.plan_trajectory(pre)
    assert result['success']
    full = tr.build_full_path(result['path'], pre)
    assert pv.path_is_valid(
        full, pre['circle_obstacles'], pre['polygon_obstacles'],
        config.R, config.ALPHA_MAX_RAD, config.L0, config.DSS,
        raw_circle_obstacles=[(CENTER, RAW_R)], raw_polygon_obstacles=[])
    # Straight line O->T is 400 km; the detour around one circle is small.
    dist = sum(math.dist(full[i][0], full[i + 1][0]) for i in range(len(full) - 1))
    assert dist < 430000.0
    # raw_route captured for the discretisation-invariance test (Task 7)
    assert result['planner'].raw_route is not None


def open_water_scenario():
    return {
        'start': (100000.0, 250000.0), 'start_heading': 0.0,
        'goal': (400000.0, 250000.0), 'goal_heading': 0.0,
        'islands': [], 'dynamic_obstacles': [], 'obstacles': [],
    }


def test_no_radial_fan_in_open_water():
    """Not riding any boundary and the goal candidate is valid: the fan must
    NOT fire (it only adds branching noise there)."""
    pre = prep.prepare_scenario(open_water_scenario())
    planner = astar.KinodynamicAstar(pre)
    st = astar.State(pre['start_state']['waypoint'], pre['start_state']['heading'])
    succ = planner.get_next_states(st)
    assert len(succ) == 1
    assert math.dist(succ[0][0].waypoint, pre['goal_state']['waypoint']) < 1.0


def test_fan_added_while_riding_boundary():
    """Riding a circle boundary: fan successors appear IN ADDITION to
    arc-hops, so the search can leave the boundary between tangent
    departure points."""
    pre = prep.prepare_scenario(synthetic_circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    (_, r_inf), = pre['circle_obstacles']
    P = (CENTER[0], CENTER[1] - r_inf)  # due south, heading east => riding CCW
    st = astar.State(P, 0.0)
    succ = planner.get_next_states(st)
    assert any(s_.arc_from is not None for s_, _ in succ)  # arc-hops present
    fan_dist = 2 * config.R * math.tan(config.ALPHA_MAX_RAD / 2) + config.RADIAL_FAN_STEP_M
    assert any(s_.arc_from is None
               and math.isclose(math.dist(s_.waypoint, P), fan_dist, rel_tol=1e-9)
               for s_, _ in succ), "fan successors missing at a riding state"


def test_plan_trajectory_smooths_output():
    """Open water: the smoothed path is the minimal W1->goal route."""
    pre = prep.prepare_scenario(open_water_scenario())
    result = astar.plan_trajectory(pre)
    assert result['success']
    assert len(result['path']) <= 3


def test_departure_state_does_not_refire_same_ride():
    """A state that IS an arc-hop departure point of a circle must not
    regenerate ride candidates for that same circle+sense (they were all
    enumerated from the ride-start; duplicates collide on the dedup lattice).

    Needs a second circle: with only ONE circle (synthetic_circle_scenario),
    the ride's sole candidate is the goal's departure point, and recomputing
    it from that exact point always yields dphi == 0.0 (departure_point()
    depends only on the target, not on the current position), so the bug
    can't be observed there even pre-fix. A second circle gives the ride an
    additional bitangent-departure candidate distinct from the goal
    departure, so re-firing from a departure point demonstrably regenerates
    that other candidate with a shorter residual arc.
    """
    scn = synthetic_circle_scenario()
    center2, radius2 = (400000.0, 400000.0), 20000.0
    scn['dynamic_obstacles'].append((center2, radius2))
    scn['obstacles'].append({'type': 'circle', 'center': center2, 'radius': radius2})
    pre = prep.prepare_scenario(scn)
    planner = astar.KinodynamicAstar(pre)
    (c1, r1) = pre['circle_obstacles'][0]
    P = (c1[0], c1[1] - r1)
    ride_start = astar.State(P, 0.0)
    hops = planner._arc_hop_successors(ride_start)
    assert len(hops) > 1, "ride-start needs >1 distinct departure candidate"
    dep_state, _cost = hops[0]
    assert dep_state.arc_from is not None
    assert planner._arc_hop_successors(dep_state) == []
    # but the same point reached WITHOUT arc_from is a fresh ride-start
    fresh = astar.State(dep_state.waypoint, dep_state.heading)
    assert planner._arc_hop_successors(fresh) != []


def test_escape_valve_fan_when_goal_occluded():
    """With the goal LOS-blocked and budget remaining, the fan augments
    Strategy A successors; once the budget is exhausted it does not."""
    pre = prep.prepare_scenario(synthetic_circle_scenario())
    planner = astar.KinodynamicAstar(pre)
    st = astar.State(pre['start_state']['waypoint'], pre['start_state']['heading'])
    fan_dist = 2 * config.R * math.tan(config.ALPHA_MAX_RAD / 2) + config.RADIAL_FAN_STEP_M

    succ = planner.get_next_states(st)
    assert any(s_.arc_from is None
               and math.isclose(math.dist(s_.waypoint, st.waypoint), fan_dist, rel_tol=1e-9)
               for s_, _ in succ), "budgeted fan missing at goal-occluded state"
    assert planner.num_strategy_b == config.NUM_STRATEGY_B - 1

    planner.num_strategy_b = 0
    succ2 = planner.get_next_states(st)
    assert not any(s_.arc_from is None
                   and math.isclose(math.dist(s_.waypoint, st.waypoint), fan_dist, rel_tol=1e-9)
                   for s_, _ in succ2), "fan fired with exhausted budget"
