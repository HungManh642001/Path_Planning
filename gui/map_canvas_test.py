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
    clicks = []
    canvas = mc.MapCanvas(root, on_map_click=lambda x, y: clicks.append((x, y)))
    state = {'start': (10000.0, 10000.0), 'start_heading': 0.0,
             'goal': (400000.0, 400000.0), 'goal_heading': 0.0,
             'obstacles': [{'type': 'circle', 'center': (200000.0, 200000.0), 'radius': 30000.0}]}
    canvas.render(state, result=None, preprocessed=None, render_mode='dubins')
    root.destroy()
