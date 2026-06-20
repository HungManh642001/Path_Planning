import math

import gui.interaction as gi


# --- parse_param: robust numeric parsing for GUI entry fields -----------------
def test_parse_param_valid_number():
    assert gi.parse_param('9000', default=8000.0) == 9000.0
    assert gi.parse_param('  12.5 ', default=0.0) == 12.5


def test_parse_param_empty_returns_default():
    assert gi.parse_param('', default=8000.0) == 8000.0
    assert gi.parse_param('   ', default=8000.0) == 8000.0


def test_parse_param_garbage_returns_default():
    assert gi.parse_param('abc', default=8000.0) == 8000.0
    assert gi.parse_param('1,2', default=5.0) == 5.0


# --- heading_from_drag: drag vector -> heading radians ------------------------
def test_heading_from_drag_east_is_zero():
    h = gi.heading_from_drag((0.0, 0.0), (100.0, 0.0), min_dist=10.0)
    assert h is not None
    assert abs(h - 0.0) < 1e-9


def test_heading_from_drag_north_is_half_pi():
    h = gi.heading_from_drag((0.0, 0.0), (0.0, 100.0), min_dist=10.0)
    assert abs(h - math.pi / 2) < 1e-9


def test_heading_from_drag_too_short_returns_none():
    # A drag shorter than min_dist is a click, not an aim -> no heading.
    assert gi.heading_from_drag((0.0, 0.0), (3.0, 0.0), min_dist=10.0) is None
