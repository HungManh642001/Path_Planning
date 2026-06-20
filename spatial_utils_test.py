import math
import spatial_utils as su


def test_circle_tangent_points_external():
    # point on +x axis, unit circle at origin: tangent points are symmetric about x-axis,
    # each at distance == radius from center, and the tangent line is perpendicular to the radius.
    pts = su.circle_tangent_points((10.0, 0.0), (0.0, 0.0), 6.0)
    assert len(pts) == 2
    for (tx, ty) in pts:
        assert abs(math.hypot(tx, ty) - 6.0) < 1e-9          # on the circle
        # radius vector (t-center) perpendicular to tangent direction (point - t)
        rx, ry = tx, ty
        dx, dy = 10.0 - tx, 0.0 - ty
        assert abs(rx * dx + ry * dy) < 1e-6                  # perpendicular
    # the two points are mirror images across the x-axis
    ys = sorted(p[1] for p in pts)
    assert abs(ys[0] + ys[1]) < 1e-9


def test_circle_tangent_points_inside_returns_empty():
    assert su.circle_tangent_points((1.0, 0.0), (0.0, 0.0), 6.0) == []
    assert su.circle_tangent_points((6.0, 0.0), (0.0, 0.0), 6.0) == []  # on the boundary


def test_inflate_polygon_keeps_sharp_corners_few_vertices():
    # A square inflated must keep ~4 sharp corners (mitre join), NOT ~70 rounded
    # arc points. The vertex count must stay small so the planner's candidate
    # branching factor does not explode.
    square = [(0.0, 0.0), (20000.0, 0.0), (20000.0, 20000.0), (0.0, 20000.0)]
    inflated = su.inflate_polygon(square, 13000.0)
    assert len(inflated) <= 12, f"mitre inflation should keep few vertices, got {len(inflated)}"


def test_inflate_polygon_contains_round_buffer_is_safe():
    # Safety invariant: the inflated (mitre) polygon must CONTAIN the exact
    # round Minkowski buffer, so the arc-clearance guarantee is preserved.
    from shapely.geometry import Polygon
    square = [(0.0, 0.0), (20000.0, 0.0), (20000.0, 20000.0), (0.0, 20000.0)]
    inflation = 13000.0
    mitre = Polygon(su.inflate_polygon(square, inflation))
    round_buffer = Polygon(square).buffer(inflation)
    assert mitre.contains(round_buffer.buffer(-1e-6)), \
        "mitre inflation must contain the round buffer (arc-clearance safety)"
