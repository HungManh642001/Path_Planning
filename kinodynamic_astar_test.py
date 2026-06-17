import math
import pytest
import preprocessing as prep
import kinodynamic_astar as astar
import path_validation as pv
import config
import spatial_utils as su


def _simple_pre(circles=(), polys=(), start=(2000, 2000), goal=(100000, 0)):
    scenario = {
        'start': start, 'start_heading': 0.0,
        'goal': goal, 'goal_heading': 0.0,
        'obstacles': [{'type': 'circle', 'center': c, 'radius': r} for c, r in circles]
                     + [{'type': 'polygon', 'polygon': p} for p in polys],
        'islands': [], 'sam_sites': [],
    }
    return prep.prepare_scenario(scenario)


def test_polygons_are_prebuilt_shapely_objects():
    pre = _simple_pre(polys=[[(0, 0), (10, 0), (10, 10), (0, 10)]])
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    from shapely.geometry import Polygon
    assert hasattr(planner, '_polygons')
    assert all(isinstance(p, Polygon) for p in planner._polygons)
    assert len(planner._polygons) == 1


def test_state_tuple_buckets_nearby_states_together():
    a = su.state_to_tuple((123456.0, 7000.0), 0.10)
    b = su.state_to_tuple((123456.0 + 200.0, 7000.0 + 200.0), 0.10 + math.radians(1.0))
    assert a == b, "states within one lattice cell must hash equal"


def test_state_tuple_distinguishes_far_states():
    a = su.state_to_tuple((0.0, 0.0), 0.0)
    b = su.state_to_tuple((5000.0, 0.0), 0.0)
    assert a != b


def test_finds_valid_path_around_single_circle():
    # Circle at (150000,0) r=20000 -> inflated ~30.3 km. Goal far enough right
    # that the goal waypoint (offset back by DSS) clears the inflated circle,
    # but the circle still blocks the direct start->goal line, forcing a detour.
    pre = _simple_pre(circles=[((150000.0, 0.0), 20000.0)], goal=(300000.0, 0.0))
    import graph_builder as gb
    tg = gb.generate_bitangents(pre['circle_obstacles'], pre['polygon_obstacles'])
    tg = gb.extend_tangent_graph_with_start_goal(
        tg, pre['start_state']['waypoint'], pre['start_state']['heading'],
        pre['goal_state']['waypoint'], pre['goal_state']['heading'],
        pre['circle_obstacles'], pre['polygon_obstacles'])
    planner = astar.KinodynamicAstar(pre, tg)
    path = planner.search()
    assert path is not None, "planner must find a route around one circle"
    assert pv.segments_clear(path, pre['circle_obstacles'], pre['polygon_obstacles'])


def test_strategy1_uses_graph_adjacency(monkeypatch):
    import graph_builder as gb
    pre = _simple_pre(circles=[((50000.0, 0.0), 20000.0)])
    tg = gb.generate_bitangents(pre['circle_obstacles'], pre['polygon_obstacles'])
    tg = gb.extend_tangent_graph_with_start_goal(
        tg, pre['start_state']['waypoint'], pre['start_state']['heading'],
        pre['goal_state']['waypoint'], pre['goal_state']['heading'],
        pre['circle_obstacles'], pre['polygon_obstacles'])
    planner = astar.KinodynamicAstar(pre, tg)

    calls = {'n': 0}
    orig = tg.get_neighbors
    def counting(pos):
        calls['n'] += 1
        return orig(pos)
    monkeypatch.setattr(tg, 'get_neighbors', counting)

    planner.get_next_states(planner.start_state)
    assert calls['n'] >= 1, "Strategy 1 must consult graph adjacency, not scan all nodes"


def test_no_dead_stuck_counter():
    import inspect
    src = inspect.getsource(astar.KinodynamicAstar.search)
    assert 'iterations_without_expansion' not in src, \
        "dead early-exit counter must be removed (C4)"


def test_produced_path_is_fully_valid_around_circle():
    pre = _simple_pre(circles=[((150000.0, 0.0), 20000.0)], goal=(300000.0, 0.0))
    import graph_builder as gb
    tg = gb.generate_bitangents(pre['circle_obstacles'], pre['polygon_obstacles'])
    tg = gb.extend_tangent_graph_with_start_goal(
        tg, pre['start_state']['waypoint'], pre['start_state']['heading'],
        pre['goal_state']['waypoint'], pre['goal_state']['heading'],
        pre['circle_obstacles'], pre['polygon_obstacles'])
    path = astar.KinodynamicAstar(pre, tg).search()
    assert path is not None
    assert pv.turn_angles_ok(path, pre['alpha_max_rad'])
    assert pv.arcs_clear(path, pre['turn_radius'],
                         pre['circle_obstacles'], pre['polygon_obstacles'])


def test_arc_clear_detects_obstacle_in_turn():
    base = {
        'start_state': {'waypoint': (0.0, 0.0), 'heading': 0.0},
        'goal_state': {'waypoint': (200000.0, 0.0), 'heading': 0.0},
        'turn_radius': 8000.0,
        'alpha_max_rad': math.radians(30.0),
        'polygon_obstacles': [],
    }
    clear_pre = dict(base, circle_obstacles=[])
    blocked_pre = dict(base, circle_obstacles=[((97000.0, 3000.0), 1500.0)])
    planner_clear = astar.KinodynamicAstar(clear_pre, tangent_graph=None)
    planner_blocked = astar.KinodynamicAstar(blocked_pre, tangent_graph=None)
    corner = (100000.0, 0.0)
    # No obstacle -> the turn arc is clear.
    assert planner_clear._arc_clear(corner, 0.0, math.pi / 2) is True
    # Obstacle sitting on the inside of the 90-deg turn arc -> blocked.
    assert planner_blocked._arc_clear(corner, 0.0, math.pi / 2) is False
    # A straight (no-turn) transition is always clear, even with the obstacle.
    assert planner_blocked._arc_clear(corner, 0.0, 0.0) is True


def test_smoothing_reduces_or_keeps_waypoints_and_stays_valid():
    pre = _simple_pre(circles=[((150000.0, 0.0), 20000.0)], goal=(300000.0, 0.0))
    import graph_builder as gb
    tg = gb.generate_bitangents(pre['circle_obstacles'], pre['polygon_obstacles'])
    tg = gb.extend_tangent_graph_with_start_goal(
        tg, pre['start_state']['waypoint'], pre['start_state']['heading'],
        pre['goal_state']['waypoint'], pre['goal_state']['heading'],
        pre['circle_obstacles'], pre['polygon_obstacles'])
    raw = astar.KinodynamicAstar(pre, tg).search()           # fresh planner
    result = astar.plan_trajectory(pre, verbose=False)        # builds its own graph + smooths
    smoothed = result['path']
    print(f"\nraw waypoints={len(raw) if raw else 0}, smoothed waypoints={len(smoothed) if smoothed else 0}")
    assert raw is not None and smoothed is not None
    assert len(smoothed) <= len(raw), f"smoothed {len(smoothed)} > raw {len(raw)}"
    # smoothed path must remain fully valid
    assert pv.segments_clear(smoothed, pre['circle_obstacles'], pre['polygon_obstacles'])
    assert pv.turn_angles_ok(smoothed, pre['alpha_max_rad'])
    assert pv.arcs_clear(smoothed, pre['turn_radius'],
                         pre['circle_obstacles'], pre['polygon_obstacles'])


def test_turn_penalty_makes_turning_cost_more_than_straight():
    pre = _simple_pre()  # empty map, no tangent graph
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    succ = planner.get_next_states(planner.start_state)
    # all radial successors share the same step distance; the straight-ahead one
    # (smallest heading change) must cost strictly less than the sharpest-turn one.
    straight = min(succ, key=lambda s: abs(astar._angle_diff(s[0].heading, planner.start_state.heading)))
    turned = max(succ, key=lambda s: abs(astar._angle_diff(s[0].heading, planner.start_state.heading)))
    assert turned[1] > straight[1], "a sharper turn must cost more once a turn penalty exists"


def test_heuristic_is_euclidean_lower_bound():
    pre = _simple_pre()
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    s = planner.start_state
    g = planner.goal_state
    euclid = math.hypot(g.waypoint[0] - s.waypoint[0], g.waypoint[1] - s.waypoint[1])
    h = planner.heuristic(s, g)
    assert abs(h - euclid) < 1e-6, "heuristic must equal Euclidean distance (admissible)"

    # Strengthen: build a state with a heading DIFFERENT from goal_state.heading so
    # the old `dist + R * heading_diff` formula would return dist + R * pi/2, not
    # just dist.  The heuristic must still equal pure Euclidean distance regardless
    # of heading, so the old formula is guaranteed to fail this assertion.
    differing_heading_state = astar.State(s.waypoint, g.heading + math.pi / 2)
    h2 = planner.heuristic(differing_heading_state, g)
    # euclid distance is identical (same waypoints); heading must NOT add anything
    assert abs(h2 - euclid) < 1e-6, \
        f"heuristic with heading diff pi/2 returned {h2:.2f}, expected Euclidean {euclid:.2f}; " \
        f"heading penalty R*pi/2 = {planner.R * math.pi / 2:.2f} must be removed"


def test_goal_directed_successor_heads_straight_at_goal():
    pre = _simple_pre()  # empty map; Strategy 2 + goal-directed only
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    succ = planner.get_next_states(planner.start_state)
    gh = su.angle_to_heading(planner.start_state.waypoint, planner.goal_state.waypoint)
    # The goal-directed successor heads EXACTLY at the goal; the radial fan headings
    # are offsets of the current heading and won't match gh exactly. Use a tight bound.
    assert any(abs(astar._angle_diff(s[0].heading, gh)) < math.radians(0.5) for s in succ), \
        "a goal-directed successor pointing straight at the goal must exist"


import random as _random
from shapely.geometry import Polygon as _Poly, LineString as _Line


def _brute_force_clear(p1, p2, circles, polys):
    import spatial_utils as su
    for c, r in circles:
        if su.point_to_line_distance(c, p1, p2) < r - 1e-6:
            return False
    line = _Line([p1, p2])
    return not any(line.intersects(_Poly(poly)) for poly in polys)


def test_check_collision_matches_bruteforce_with_spatial_index():
    polys = [[(10000, 10000), (30000, 10000), (30000, 30000), (10000, 30000)],
             [(60000, 5000), (90000, 5000), (75000, 40000)]]
    circles = [((50000.0, 50000.0), 8000.0)]
    pre = _simple_pre(circles=[((50000.0, 50000.0), 8000.0)],
                      polys=polys)
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    rng = _random.Random(1234)
    for _ in range(400):
        p1 = (rng.uniform(0, 100000), rng.uniform(0, 100000))
        p2 = (rng.uniform(0, 100000), rng.uniform(0, 100000))
        got = planner._check_collision(p1, p2)
        want = _brute_force_clear(p1, p2, pre['circle_obstacles'], pre['polygon_obstacles'])
        assert got == want, f"mismatch on {p1}->{p2}: got {got} want {want}"
    assert hasattr(planner, '_poly_tree')
