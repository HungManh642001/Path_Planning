import math
import graph_builder as gb
import spatial_utils as su
import config


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


def test_single_circle_produces_navigation_nodes():
    circles = [((50000.0, 0.0), 20000.0)]
    g = gb.generate_bitangents(circles, [], filter_los=True)
    assert len(g.nodes) == config.OBSTACLE_RING_SAMPLES


def test_single_polygon_produces_hull_nodes():
    poly = [(40000.0, -10000.0), (60000.0, -10000.0),
            (60000.0, 10000.0), (40000.0, 10000.0)]
    g = gb.generate_bitangents([], [poly], filter_los=True)
    assert len(g.nodes) >= 4


def test_start_goal_los_uses_full_radius():
    # A circle on the direct start->goal line must block the direct edge.
    circles = [((50000.0, 0.0), 20000.0)]
    g = gb.generate_bitangents(circles, [])
    g = gb.extend_tangent_graph_with_start_goal(
        g, (0.0, 0.0), 0.0, (100000.0, 0.0), 0.0, circles, [])
    assert g.find_node_index((0.0, 0.0)) is not None
    neighbors = [p for p, _ in g.get_neighbors((0.0, 0.0))]
    assert (100000.0, 0.0) not in neighbors, "direct start->goal edge must be blocked by the circle"
