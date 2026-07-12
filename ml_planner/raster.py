"""Shared crop/affine + channel rasterization for the CNN guidance map.

Used by BOTH dataset_gen (labels) and guidance (inference) so the two agree
on exactly one crop + channel definition. Grid arrays are indexed [iy, ix]
where ix is grid-x from world-x and iy is grid-y from world-y.
"""

import numpy as np
from matplotlib.path import Path as _MplPath


class Affine:
    """Square-crop world<->grid mapping. gx = (x-x0)*scale, gy = (y-y0)*scale."""

    def __init__(self, x0, y0, scale, grid_res):
        self.x0 = x0
        self.y0 = y0
        self.scale = scale          # grid units per world meter
        self.grid_res = grid_res

    def world_to_grid(self, x, y):
        return ((x - self.x0) * self.scale, (y - self.y0) * self.scale)

    def grid_to_world(self, gx, gy):
        return (self.x0 + gx / self.scale, self.y0 + gy / self.scale)


def compute_crop(preprocessed, grid_res, margin_frac=0.1):
    """Square crop covering start, goal, and all obstacles, plus a margin."""
    xs = [preprocessed['start_pos'][0], preprocessed['goal_pos'][0]]
    ys = [preprocessed['start_pos'][1], preprocessed['goal_pos'][1]]
    for (cx, cy), r in preprocessed['circle_obstacles']:
        xs += [cx - r, cx + r]
        ys += [cy - r, cy + r]
    for poly in preprocessed['polygon_obstacles']:
        for px, py in poly:
            xs.append(px)
            ys.append(py)
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
    side = max(xmax - xmin, ymax - ymin)
    if side <= 0.0:
        side = 1.0
    side *= (1.0 + 2.0 * margin_frac)
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    x0 = cx - 0.5 * side
    y0 = cy - 0.5 * side
    return Affine(x0, y0, grid_res / side, grid_res)


def _cell_centers_world(affine, grid_res):
    """Convert grid cell indices to world coordinates at cell centers."""
    idx = np.arange(grid_res, dtype=np.float64) + 0.5
    gx, gy = np.meshgrid(idx, idx)          # both shape (grid_res, grid_res), [iy, ix]
    wx = affine.x0 + gx / affine.scale
    wy = affine.y0 + gy / affine.scale
    return wx, wy


def build_channels(preprocessed, affine, grid_res):
    """(4, H, W) float32: occupancy, safezone, dist-to-goal, start marker."""
    wx, wy = _cell_centers_world(affine, grid_res)
    pts = np.column_stack([wx.ravel(), wy.ravel()])

    occ = np.zeros((grid_res, grid_res), dtype=bool)
    for (cx, cy), r in preprocessed['circle_obstacles']:
        occ |= ((wx - cx) ** 2 + (wy - cy) ** 2) < r * r
    for poly in preprocessed['polygon_obstacles']:
        occ |= _MplPath(poly).contains_points(pts).reshape(grid_res, grid_res)

    safezones = preprocessed.get('safezones')
    if safezones:
        inside = np.zeros((grid_res, grid_res), dtype=bool)
        for poly in safezones:
            inside |= _MplPath(poly).contains_points(pts).reshape(grid_res, grid_res)
        safe = inside.astype(np.float32)
    else:
        safe = np.ones((grid_res, grid_res), dtype=np.float32)

    gx_goal, gy_goal = preprocessed['goal_pos']
    dist = np.sqrt((wx - gx_goal) ** 2 + (wy - gy_goal) ** 2)
    diag = np.sqrt(2.0) * grid_res / affine.scale
    dgoal = (dist / diag).astype(np.float32)

    start = np.zeros((grid_res, grid_res), dtype=np.float32)
    sgx, sgy = affine.world_to_grid(*preprocessed['start_pos'])
    si, sj = int(round(sgy)), int(round(sgx))
    if 0 <= si < grid_res and 0 <= sj < grid_res:
        start[si, sj] = 1.0

    return np.stack([occ.astype(np.float32), safe, dgoal, start], axis=0)
