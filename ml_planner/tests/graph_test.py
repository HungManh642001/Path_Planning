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
