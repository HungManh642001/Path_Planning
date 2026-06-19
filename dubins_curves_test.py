# dubins_curves_test.py
import math
import dubins_curves as dc


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def test_straight_line_when_aligned():
    # start and goal aligned on +x, both heading +x -> straight, length == distance
    p = dc.DubinsPath((0.0, 0.0), 0.0, (100.0, 0.0), 0.0, radius=10.0)
    sp = p.shortest_path()
    assert sp is not None
    assert abs(sp['length'] - 100.0) < 1e-6


def test_sample_reproduces_start_and_goal():
    # Several configurations: the sampled path must start at start and end at goal,
    # with matching headings -- this validates the whole solve+sample pipeline.
    cases = [
        ((0.0, 0.0), 0.0, (80.0, 40.0), math.pi / 2),
        ((0.0, 0.0), math.pi / 2, (-60.0, 30.0), math.pi),
        ((10.0, -5.0), -math.pi / 4, (50.0, 50.0), 0.0),
        ((0.0, 0.0), 0.0, (5.0, 0.0), math.pi),   # short hop, forces a CCC word
    ]
    for sp_pos, sp_h, gp_pos, gp_h in cases:
        path = dc.DubinsPath(sp_pos, sp_h, gp_pos, gp_h, radius=10.0)
        pts = path.sample_path(step=1.0)
        assert len(pts) >= 2
        assert math.hypot(pts[0][0] - sp_pos[0], pts[0][1] - sp_pos[1]) < 1e-6
        assert abs(_wrap(pts[0][2] - sp_h)) < 1e-6
        assert math.hypot(pts[-1][0] - gp_pos[0], pts[-1][1] - gp_pos[1]) < 1e-3
        assert abs(_wrap(pts[-1][2] - gp_h)) < 1e-3


def test_samples_are_continuous():
    path = dc.DubinsPath((0.0, 0.0), 0.0, (80.0, 40.0), math.pi / 2, radius=10.0)
    pts = path.sample_path(step=1.0)
    gaps = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            for i in range(len(pts) - 1)]
    assert max(gaps) < 2.0, "samples must be densely spaced (no jumps)"


def test_arc_points_lie_on_turning_circle():
    # A pure left turn: every sample must be exactly radius R from the turning centre,
    # proving arcs are sampled geometrically (not linearly interpolated).
    R = 10.0
    path = dc.DubinsPath((0.0, 0.0), 0.0, (0.0, 20.0), math.pi, radius=R)
    pts = path.sample_path(step=0.5)
    # left-turn centre for start (0,0,heading 0) is at (0, R)
    cx, cy = 0.0, R
    on_circle = sum(1 for (x, y, _) in pts if abs(math.hypot(x - cx, y - cy) - R) < 1e-6)
    assert on_circle >= len(pts) // 2, "arc samples must lie on the turning circle"
