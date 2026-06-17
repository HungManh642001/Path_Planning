import math
import pytest
import preprocessing as prep
import kinodynamic_astar as astar
import path_validation as pv
import config


def _simple_pre(circles=(), polys=()):
    scenario = {
        'start': (2000, 2000), 'start_heading': 0.0,
        'goal': (100000, 0), 'goal_heading': 0.0,
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


@pytest.mark.skip(reason="completeness expected after Phase 2 (lattice/adjacency); re-enable in Task 2.2")
def test_finds_valid_path_around_single_circle():
    pre = _simple_pre(circles=[((50000.0, 0.0), 20000.0)])
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
