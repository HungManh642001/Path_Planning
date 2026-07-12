import numpy as np
import core.map_generator as mg
import core.preprocessing as prep
from ml_planner.raster import compute_crop, build_channels


def _prep():
    return prep.prepare_scenario(mg.scenario2_single_obstacle())


def test_crop_contains_start_goal_and_roundtrips():
    pre = _prep()
    aff = compute_crop(pre, grid_res=256)
    for pt in (pre['start_pos'], pre['goal_pos']):
        gx, gy = aff.world_to_grid(*pt)
        assert 0.0 <= gx <= 256.0 and 0.0 <= gy <= 256.0
        x, y = aff.grid_to_world(gx, gy)
        assert abs(x - pt[0]) < 1e-3 and abs(y - pt[1]) < 1e-3


def test_crop_covers_inflated_circles():
    pre = _prep()
    aff = compute_crop(pre, grid_res=256)
    for (cx, cy), r in pre['circle_obstacles']:
        for corner in ((cx - r, cy - r), (cx + r, cy + r)):
            gx, gy = aff.world_to_grid(*corner)
            assert -1.0 <= gx <= 257.0 and -1.0 <= gy <= 257.0


def test_channels_shape_and_occupancy():
    pre = _prep()
    aff = compute_crop(pre, grid_res=64)
    ch = build_channels(pre, aff, grid_res=64)
    assert ch.shape == (4, 64, 64)
    assert ch.dtype == np.float32
    # A cell at an inflated circle center must be marked occupied (channel 0).
    (cx, cy), r = pre['circle_obstacles'][0]
    gx, gy = aff.world_to_grid(cx, cy)
    assert ch[0, int(gy), int(gx)] == 1.0
    # No safezone in this scenario -> channel 1 is all ones.
    assert np.all(ch[1] == 1.0)
    # Distance-to-goal channel is ~0 at the goal cell.
    ggx, ggy = aff.world_to_grid(*pre['goal_pos'])
    if 0 <= int(ggy) < 64 and 0 <= int(ggx) < 64:
        assert ch[2, int(ggy), int(ggx)] < 0.05
