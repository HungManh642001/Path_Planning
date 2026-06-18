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
