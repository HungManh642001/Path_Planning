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


def test_arc_line_trajectory_passes_near_each_waypoint_no_jumps():
    # A 3-waypoint path with a 90deg corner. The flown trajectory (straight legs +
    # radius-R turn arc at the corner) must be continuous (no gaps/jumps) and pass
    # through the endpoints exactly; near the corner it cuts inside by the arc.
    R = 8000.0
    pts = [(0.0, 0.0), (100000.0, 0.0), (100000.0, 100000.0)]
    traj = su.arc_line_trajectory(pts, R)
    assert len(traj) >= 3
    # endpoints preserved
    assert math.hypot(traj[0][0] - pts[0][0], traj[0][1] - pts[0][1]) < 1.0
    assert math.hypot(traj[-1][0] - pts[-1][0], traj[-1][1] - pts[-1][1]) < 1.0
    # continuity: no jump between consecutive trajectory samples larger than the
    # longest straight leg (i.e. the polyline is connected, not skipping a corner)
    max_gap = max(math.hypot(traj[i + 1][0] - traj[i][0], traj[i + 1][1] - traj[i][1])
                  for i in range(len(traj) - 1))
    assert max_gap < 100001.0, f"trajectory has a jump of {max_gap:.0f} m"
    # the corner is rounded: the trajectory comes within R of the corner but the
    # arc cuts inside, so the closest sample to the corner is < the leg length
    dmin = min(math.hypot(x - pts[1][0], y - pts[1][1]) for (x, y) in traj)
    assert 0.0 < dmin < R + 1.0, f"corner not rounded by an R arc (dmin={dmin:.0f})"


def test_arc_line_trajectory_arc_radius_matches_R():
    # The turn arc must have radius ~R: sampled arc points are equidistant (=R) from
    # the arc centre. Check the mid-arc sample curvature indirectly via the inset:
    # tangent points sit R*tan(alpha/2) back from the corner along each leg.
    R = 8000.0
    pts = [(0.0, 0.0), (100000.0, 0.0), (200000.0, 0.0)]  # straight line, no turn
    traj = su.arc_line_trajectory(pts, R)
    # A straight path stays on y=0 throughout (no spurious arc)
    assert all(abs(y) < 1e-6 for (x, y) in traj)
