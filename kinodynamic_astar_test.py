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
