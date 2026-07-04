"""Unit tests for core/arc_geometry.py (pure geometry, no planner)."""
import math

import core.arc_geometry as ag

C = (100000.0, 100000.0)
R_C = 20000.0


def test_riding_sense_ccw():
    # Point due east of center, heading north => CCW tangent
    P = (C[0] + R_C, C[1])
    assert ag.riding_sense(P, math.pi / 2, C, R_C) == 1


def test_riding_sense_cw():
    P = (C[0] + R_C, C[1])
    assert ag.riding_sense(P, -math.pi / 2, C, R_C) == -1


def test_riding_sense_rejects_off_boundary():
    P = (C[0] + R_C + 50.0, C[1])  # 50 m off the boundary; tol is 1 m
    assert ag.riding_sense(P, math.pi / 2, C, R_C) == 0


def test_riding_sense_rejects_non_tangent_heading():
    P = (C[0] + R_C, C[1])
    assert ag.riding_sense(P, math.pi / 4, C, R_C) == 0  # 45 deg off tangent


def test_tangent_heading_matches_sense():
    P = (C[0] + R_C, C[1])
    assert math.isclose(ag.tangent_heading(P, C, +1), math.pi / 2, abs_tol=1e-9)
    assert math.isclose(ag.tangent_heading(P, C, -1), -math.pi / 2, abs_tol=1e-9)


def test_arc_angle_quarter_turns():
    P = (C[0] + R_C, C[1])  # polar angle 0
    Q = (C[0], C[1] + R_C)  # polar angle pi/2
    assert math.isclose(ag.arc_angle(P, Q, C, +1), math.pi / 2, rel_tol=1e-9)
    assert math.isclose(ag.arc_angle(P, Q, C, -1), 3 * math.pi / 2, rel_tol=1e-9)


def test_arc_angle_same_point_is_zero():
    P = (C[0] + R_C, C[1])
    assert ag.arc_angle(P, P, C, +1) == 0.0
