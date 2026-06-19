"""Smoke tests for gui.config_panel.ConfigPanel."""
import tkinter as tk
import pytest
import gui.config_panel as cp


def test_config_panel_constructs_and_reports_values():
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display available")
    noop = lambda *a, **k: None
    panel = cp.ConfigPanel(root, on_run=noop, on_set_start=noop, on_set_goal=noop,
                           on_draw_polygon=noop, on_draw_circle=noop, on_clear=noop,
                           on_load=noop, on_save=noop)
    vals = panel.values()
    assert 'turn_radius' in vals and 'wrap_step_m' in vals
    root.destroy()
