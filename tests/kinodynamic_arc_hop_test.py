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
