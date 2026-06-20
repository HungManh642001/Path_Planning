import math
import gui.app as app


def test_run_pipeline_solves_simple_scenario():
    state = {'start': (10000.0, 250000.0), 'start_heading': 0.0,
             'goal': (400000.0, 250000.0), 'goal_heading': 0.0,
             'obstacles': []}
    import gui.params as gp
    result, preprocessed, raw_circles, raw_polys, runtime_s = app.run_pipeline(
        state, gp.default_values())
    assert result['success'] is True
    assert runtime_s >= 0.0
    assert preprocessed['turn_radius'] > 0
