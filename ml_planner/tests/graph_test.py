import math

import numpy as np

from ml_planner.graph import bitangent_points


def _tangency_ok(p_on_1, p_on_2, c1, c2):
    """The bitangent chord must be perpendicular to both touch radii."""
    vx, vy = p_on_2[0] - p_on_1[0], p_on_2[1] - p_on_1[1]
    r1x, r1y = p_on_1[0] - c1[0], p_on_1[1] - c1[1]
    r2x, r2y = p_on_2[0] - c2[0], p_on_2[1] - c2[1]
    L = math.hypot(vx, vy)
    return (abs(vx * r1x + vy * r1y) / L < 1e-9
            and abs(vx * r2x + vy * r2y) / L < 1e-9)


def test_bitangent_two_unit_circles_exact():
    c1, c2 = (0.0, 0.0), (4.0, 0.0)
    pairs = bitangent_points(c1, 1.0, c2, 1.0)
    assert len(pairs) == 4
    for p1, p2 in pairs:
        assert abs(math.hypot(p1[0] - c1[0], p1[1] - c1[1]) - 1.0) < 1e-9
        assert abs(math.hypot(p2[0] - c2[0], p2[1] - c2[1]) - 1.0) < 1e-9
        assert _tangency_ok(p1, p2, c1, c2)
    # External bitangents of equal circles are horizontal lines y=±1.
    ext = sorted(pairs[:2], key=lambda pr: pr[0][1])
    assert np.allclose(ext[0][0], (0.0, -1.0)) and np.allclose(ext[0][1], (4.0, -1.0))
    assert np.allclose(ext[1][0], (0.0, 1.0)) and np.allclose(ext[1][1], (4.0, 1.0))


def test_bitangent_overlapping_circles_drop_internal():
    # d=3 < r1+r2=4: internal bitangents vanish, external survive.
    pairs = bitangent_points((0.0, 0.0), 2.0, (3.0, 0.0), 2.0)
    assert len(pairs) == 2
    for p1, p2 in pairs:
        assert _tangency_ok(p1, p2, (0.0, 0.0), (3.0, 0.0))


def test_bitangent_concentric_returns_empty():
    assert bitangent_points((5.0, 5.0), 2.0, (5.0, 5.0), 1.0) == []


import core.preprocessing as prep
from ml_planner.graph import build_graph, EDGE_CHORD, EDGE_ARC


def _circle_scenario():
    circles = [((250_000.0, 250_000.0), 20_000.0), ((150_000.0, 300_000.0), 15_000.0)]
    return {
        'start': (20_000.0, 250_000.0), 'start_heading': 0.0,
        'goal': (480_000.0, 250_000.0), 'goal_heading': 0.0,
        'islands': [], 'dynamic_obstacles': list(circles),
        'obstacles': [{'type': 'circle', 'center': c, 'radius': r} for c, r in circles],
    }


def _seg_center_dist(p, q, c):
    px, py = p; qx, qy = q; cx, cy = c
    sx, sy = qx - px, qy - py
    dd = sx * sx + sy * sy
    if dd == 0.0:
        return math.hypot(cx - px, cy - py)
    t = max(0.0, min(1.0, ((cx - px) * sx + (cy - py) * sy) / dd))
    return math.hypot(px + t * sx - cx, py + t * sy - cy)


def test_build_graph_node_census_and_determinism():
    pre = prep.prepare_scenario(_circle_scenario())
    g1, g2 = build_graph(pre), build_graph(pre)
    assert np.array_equal(g1.nodes, g2.nodes)
    assert np.array_equal(g1.edges, g2.edges)
    assert g1.start_idx == 0 and g1.goal_idx == 1
    # 2 disjoint circles: start(2/circle=4) + goal(4) tangent points
    # + 4 bitangents x 2 touch points = 8  ->  2 + 4 + 4 + 8 = 18 nodes.
    assert len(g1.nodes) == 18
    assert g1.node_feat.shape == (18, 7)
    assert g1.edge_feat.shape[1] == 2


def test_chord_edges_clear_of_inflated_circles():
    pre = prep.prepare_scenario(_circle_scenario())
    g = build_graph(pre)
    chords = g.edges[g.edge_feat[:, 1] == EDGE_CHORD]
    assert len(chords) > 0
    for a, b in chords:
        for c, r in pre['circle_obstacles']:
            assert _seg_center_dist(g.nodes[a], g.nodes[b], c) >= r - 1e-6


def test_arc_edges_connect_same_circle_nodes():
    pre = prep.prepare_scenario(_circle_scenario())
    g = build_graph(pre)
    delta = 1.0  # config.CONSTRUCTION_CLEARANCE_M
    arcs = g.edges[g.edge_feat[:, 1] == EDGE_ARC]
    assert len(arcs) > 0
    for a, b in arcs:
        on_same = False
        for c, r in pre['circle_obstacles']:
            rc = r + delta
            da = abs(math.hypot(g.nodes[a][0] - c[0], g.nodes[a][1] - c[1]) - rc)
            db = abs(math.hypot(g.nodes[b][0] - c[0], g.nodes[b][1] - c[1]) - rc)
            if da < 1e-6 and db < 1e-6:
                on_same = True
        assert on_same


def test_node_features_normalized_and_flagged():
    pre = prep.prepare_scenario(_circle_scenario())
    g = build_graph(pre)
    D = g.scale
    # feat 0: dist-to-goal/D — exactly 0 at the goal node, 1 at the start node.
    assert abs(g.node_feat[g.goal_idx, 0]) < 1e-6
    assert abs(g.node_feat[g.start_idx, 0] - 1.0) < 1e-6
    # flags
    assert g.node_feat[g.goal_idx, 4] == 1.0 and g.node_feat[g.start_idx, 5] == 1.0
    assert np.all(g.node_feat[:, 6] == 1.0)          # safezone slot, this phase
    # sin/cos are unit
    assert np.allclose(g.node_feat[:, 1] ** 2 + g.node_feat[:, 2] ** 2, 1.0, atol=1e-5)


def test_empty_map_graph_is_start_goal_edge():
    scen = {'start': (0.0, 0.0), 'start_heading': 0.0,
            'goal': (100_000.0, 0.0), 'goal_heading': 0.0,
            'islands': [], 'dynamic_obstacles': [], 'obstacles': []}
    g = build_graph(prep.prepare_scenario(scen))
    assert len(g.nodes) == 2
    assert len(g.edges) == 1 and tuple(g.edges[0]) == (0, 1)


def test_two_node_ring_keeps_both_arcs():
    # Goal at the circle center -> no goal tangents, so the ring has exactly
    # the 2 start tangent nodes; both complementary arcs must survive dedup.
    circles = [((250_000.0, 250_000.0), 20_000.0)]
    scen = {'start': (20_000.0, 250_000.0), 'start_heading': 0.0,
            'goal': (250_000.0, 250_000.0), 'goal_heading': 0.0,
            'islands': [], 'dynamic_obstacles': list(circles),
            'obstacles': [{'type': 'circle', 'center': c, 'radius': r}
                          for c, r in circles]}
    pre = prep.prepare_scenario(scen)
    g = build_graph(pre)
    arc_lengths = g.edge_feat[g.edge_feat[:, 1] == EDGE_ARC][:, 0]
    assert len(arc_lengths) == 2
    (c0, r0) = pre['circle_obstacles'][0]
    rc = r0 + 1.0                      # config.CONSTRUCTION_CLEARANCE_M
    full_circle = 2.0 * math.pi * rc / g.scale
    assert abs(float(arc_lengths.sum()) - full_circle) < 1e-6
