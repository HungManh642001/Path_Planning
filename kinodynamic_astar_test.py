import math
import preprocessing as prep
import kinodynamic_astar as astar


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
