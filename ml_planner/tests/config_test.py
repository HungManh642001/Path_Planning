import ml_planner.config as mlcfg


def test_focal_constants_present():
    assert mlcfg.FOCAL_EPS == 0.05
    assert abs(mlcfg.FOCAL_WEIGHT - (1.0 + mlcfg.FOCAL_EPS)) < 1e-12
    # Phase-2 placeholders exist so later tasks can import them.
    assert mlcfg.GRID_RES == 256
    assert isinstance(mlcfg.MODEL_PATH, str)
