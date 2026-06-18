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


def test_arc_clear_method_removed():
    import inspect
    assert not hasattr(astar.KinodynamicAstar, '_arc_clear'), \
        "arc clearance is guaranteed by inflation; the runtime check must be removed"
    src = inspect.getsource(astar.KinodynamicAstar.get_next_states)
    assert '_arc_clear' not in src
    src2 = inspect.getsource(astar.KinodynamicAstar.smooth_path)
    assert '_arc_clear' not in src2


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
    # Goal directly behind the start heading (turn 180deg > alpha_max) => no graph
    # candidate is valid => radial fallback fires, giving the 11-way fan to compare.
    pre = _simple_pre(goal=(-50000.0, 0.0))
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    succ = planner.get_next_states(planner.start_state)
    assert len(succ) >= 2
    straight = min(succ, key=lambda s: abs(astar._angle_diff(s[0].heading, planner.start_state.heading)))
    turned = max(succ, key=lambda s: abs(astar._angle_diff(s[0].heading, planner.start_state.heading)))
    assert turned[1] > straight[1], "a sharper turn must cost more (turn penalty)"


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


import spatial_utils as su2  # alias to avoid clashing if su already imported


def test_dynamic_successors_include_circle_tangents():
    # Circle nearly straight ahead and small, so BOTH tangent directions are within
    # alpha_max of the start heading (this task runs at the default alpha_max=30 deg).
    # From start, successors must include the circle's tangent points (computed against
    # the INFLATED radius, which is what the planner stores) — no tangent_graph used.
    circle_raw = ((60000.0, 6000.0), 8000.0)
    pre = _simple_pre(circles=[circle_raw], goal=(120000.0, 0.0))
    inflated_center, inflated_radius = pre['circle_obstacles'][0]
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    succ = planner.get_next_states(planner.start_state)
    tang = su2.circle_tangent_points(planner.start_state.waypoint, inflated_center, inflated_radius)
    succ_wps = [s[0].waypoint for s in succ]
    assert tang, "expected two tangent points from start to the inflated circle"
    assert any(min(math.hypot(w[0]-t[0], w[1]-t[1]) for w in succ_wps) < 1.0 for t in tang), \
        "dynamic successors must include circle tangent points"


def test_radial_fallback_when_no_graph_candidate():
    # empty map: no obstacles, so tangent set is just the goal. If the goal is directly
    # reachable it is a successor; force the not-directly-reachable case by a goal behind
    # the start heading so the direct jump turn > alpha_max, exercising the radial fallback.
    pre = _simple_pre(goal=(-50000.0, 0.0))   # goal behind start (start_heading=0)
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    succ = planner.get_next_states(planner.start_state)
    assert len(succ) > 0, "radial fallback must produce successors when no graph candidate is valid"


import time as _time


def test_search_respects_time_budget(monkeypatch):
    # Force a tiny budget and a scenario that would otherwise run to the iteration cap;
    # search must return within a small multiple of the budget.
    monkeypatch.setattr(config, 'TIME_BUDGET_S', 0.05)
    # dense-ish: several polygons straddling the route so the graph can't trivially solve it
    polys = [[(40000+i*1000, 0), (60000+i*1000, 0), (50000+i*1000, 40000)] for i in range(6)]
    pre = _simple_pre(polys=polys, goal=(120000.0, 0.0))
    planner = astar.KinodynamicAstar(pre, tangent_graph=None)
    t0 = _time.perf_counter()
    planner.search()
    dt = _time.perf_counter() - t0
    assert dt < 0.5, f"search ignored the 0.05s budget (took {dt:.3f}s)"
