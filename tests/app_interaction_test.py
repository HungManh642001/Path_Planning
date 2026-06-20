"""Interaction state-machine tests for PlannerApp (press / drag / release).

These exercise the click-to-place + drag-to-aim logic and the in-progress
drawing overlay. They need a Tk display; skip headlessly.
"""
import math

import pytest

import matplotlib
matplotlib.use('Agg')


def _make_app():
    try:
        import tkinter as tk
    except Exception:
        pytest.skip("tkinter unavailable")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display available")
    from gui.app import PlannerApp
    return root, PlannerApp(root)


def test_start_drag_sets_position_and_heading():
    root, app = _make_app()
    app._set_mode('start')
    app.on_map_press(100000.0, 100000.0)
    app.on_map_drag(100000.0, 150000.0)
    app.on_map_release(100000.0, 150000.0)     # drag north -> heading ~ pi/2
    assert app.state['start'] == (100000.0, 100000.0)
    assert abs(app.state['start_heading'] - math.pi / 2) < 1e-6
    assert app.mode == 'idle'
    root.destroy()


def test_start_click_without_drag_keeps_position_no_heading_change():
    root, app = _make_app()
    app.state['start_heading'] = 0.0
    app._set_mode('start')
    app.on_map_press(200000.0, 200000.0)
    app.on_map_release(200000.0, 200000.0)     # no drag -> heading unchanged
    assert app.state['start'] == (200000.0, 200000.0)
    assert app.state['start_heading'] == 0.0
    root.destroy()


def test_circle_press_drag_release_commits_obstacle():
    root, app = _make_app()
    app._set_mode('circle')
    app.on_map_press(300000.0, 300000.0)
    app.on_map_drag(300000.0, 330000.0)
    app.on_map_release(300000.0, 330000.0)
    obs = app.state['obstacles']
    assert len(obs) == 1 and obs[0]['type'] == 'circle'
    assert obs[0]['center'] == (300000.0, 300000.0)
    assert abs(obs[0]['radius'] - 30000.0) < 1e-6
    assert app.mode == 'idle'
    root.destroy()


def test_polygon_clicks_show_overlay_then_commit_on_close():
    root, app = _make_app()
    app._set_mode('polygon')
    app.on_map_press(100000.0, 100000.0)
    app.on_map_release(100000.0, 100000.0)
    app.on_map_press(150000.0, 100000.0)
    app.on_map_release(150000.0, 100000.0)
    app.on_map_press(125000.0, 140000.0)
    app.on_map_release(125000.0, 140000.0)
    # in-progress polygon is visible via the overlay before closing
    assert len(app._current_overlay().get('poly_pts', [])) == 3
    # click near the first vertex to close
    app.on_map_press(100500.0, 100500.0)
    app.on_map_release(100500.0, 100500.0)
    polys = [o for o in app.state['obstacles'] if o['type'] == 'polygon']
    assert len(polys) == 1
    assert len(polys[0]['polygon']) == 3
    assert app._current_overlay().get('poly_pts', []) == []
    root.destroy()
