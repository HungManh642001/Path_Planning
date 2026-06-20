import matplotlib
matplotlib.use('Agg')
import tkinter as tk
import pytest
import gui.map_canvas as mc


def test_canvas_constructs_and_renders_without_error():
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display available")
    events = []
    canvas = mc.MapCanvas(root, on_press=lambda x, y: events.append(('press', x, y)),
                          on_drag=lambda x, y: events.append(('drag', x, y)),
                          on_release=lambda x, y: events.append(('release', x, y)))
    state = {'start': (10000.0, 10000.0), 'start_heading': 0.0,
             'goal': (400000.0, 400000.0), 'goal_heading': 0.0,
             'obstacles': [{'type': 'circle', 'center': (200000.0, 200000.0), 'radius': 30000.0}]}
    canvas.render(state, result=None, preprocessed=None, render_mode='dubins')
    # in-progress overlay must render without error
    canvas.render(state, result=None, preprocessed=None, render_mode='dubins',
                  overlay={'poly_pts': [(1.0, 2.0), (3.0, 4.0)], 'rubber': (5.0, 6.0),
                           'circle_preview': ((100000.0, 100000.0), 20000.0),
                           'aim': ((0.0, 0.0), (50000.0, 50000.0))})
    root.destroy()
