"""Polygon vertex candidates are LIFTED off the hull, like circle tangents.

Polygons used to be the one obstacle type whose navigation targets sat EXACTLY
on the boundary they have to clear, while circles were built on
`radius + CONSTRUCTION_CLEARANCE_M + GEOM_EPS_M`. That asymmetry is what put the
boundary case in front of shapely on every chord that ends at, passes through,
or runs along a hull edge, and it is where the whole family of interior-overlap
artefacts came from. Absorbing it during CONSTRUCTION is cheaper and more robust
than resolving it during checking.
"""

from shapely.geometry import Point, Polygon

from path_planning import config, planner as astar
from path_planning.scenario import preprocessing as prep, presets as mg


PLANNERS = (astar.KinodynamicAstar,)


def _planners_for(scenario):
    pre = prep.prepare_scenario(scenario)
    return pre, [cls(pre) for cls in PLANNERS]


def test_every_vertex_candidate_clears_its_polygon():
    scen = mg.get_all_scenarios()["scenario_13_dense_island_field"]()
    pre, planners = _planners_for(scen)
    polys = [Polygon(c) for c in pre["polygon_obstacles"]]
    assert polys, "scenario has no polygons to test against"

    for planner in planners:
        assert planner._poly_vertices
        for v in planner._poly_vertices:
            P = Point(*v)
            for poly in polys:
                assert not poly.contains(P), f"{v} is inside a polygon"
                if poly.distance(P) < 1.0:  # the polygon this vertex came from
                    assert poly.distance(P) >= config.CONSTRUCTION_CLEARANCE_M * 0.99, (
                        f"{v} sits on the boundary ({poly.distance(P)} m)"
                    )


def test_the_lift_is_the_same_one_circles_get():
    scen = mg.get_all_scenarios()["scenario_13_dense_island_field"]()
    _pre, planners = _planners_for(scen)
    for planner in planners:
        assert planner._construct_delta == (
            config.CONSTRUCTION_CLEARANCE_M + config.GEOM_EPS_M
        )


def test_lifted_hull_still_encloses_the_polygon():
    """The lift may only grow the hull — a candidate set that cut a corner would
    route the aircraft through the obstacle it is meant to go around."""
    scen = mg.get_all_scenarios()["scenario_13_dense_island_field"]()
    pre, planners = _planners_for(scen)
    planner = planners[0]
    for coords in pre["polygon_obstacles"]:
        poly = Polygon(coords)
        hull = poly.convex_hull
        grown = hull.buffer(planner._construct_delta, join_style=2)
        assert grown.contains(hull)
