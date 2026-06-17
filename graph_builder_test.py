import math
import graph_builder as gb
import spatial_utils as su


def _dist_point_to_line(p, a, b):
    return su.point_to_line_distance(p, a, b)


def test_generated_tangents_do_not_cross_circle_centres():
    # two equal circles; every generated edge must keep >= r clearance from
    # both centres (a true tangent touches at exactly r).
    circles = [((0.0, 0.0), 1000.0), ((10000.0, 0.0), 1000.0)]
    g = gb.generate_bitangents(circles, [], filter_los=True)
    assert len(g.edges) > 0
    for p1, p2 in g.edges:
        for center, radius in circles:
            assert _dist_point_to_line(center, p1, p2) >= radius - 1.0, \
                f"edge {p1}->{p2} cuts circle at {center}"
