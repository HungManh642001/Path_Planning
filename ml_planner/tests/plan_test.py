import core.map_generator as mg
import core.preprocessing as prep
from ml_planner.plan import plan_trajectory_focal, path_length


def test_plan_trajectory_focal_solves_open_ocean():
    pre = prep.prepare_scenario(mg.scenario1_open_ocean())
    res = plan_trajectory_focal(pre)
    assert res['success'] is True
    assert res['path'] is not None
    assert path_length(res['path']) > 0.0
    assert 'stats' in res and 'planner' in res


def test_path_length_of_two_points():
    path = [((0.0, 0.0), 0.0), ((3.0, 4.0), 0.0)]
    assert abs(path_length(path) - 5.0) < 1e-9
