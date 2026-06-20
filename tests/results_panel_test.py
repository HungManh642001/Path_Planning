import tkinter as tk
import pytest
import gui.results_panel as rp


def test_results_panel_shows_summary_and_reports_mode():
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display available")
    panel = rp.ResultsPanel(root, on_render_mode_change=lambda mode: None)
    panel.show_summary({'success': True, 'distance_km': 123.4, 'num_waypoints': 5,
                        'num_turns': 3, 'max_turn_deg': 42.0, 'runtime_ms': 88.0,
                        'iterations': 120, 'valid': True})
    panel.log('done')
    assert panel.render_mode() in ('dubins', 'straight')
    root.destroy()
