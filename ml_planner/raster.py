"""Shared crop/affine + channel rasterization for the CNN guidance map.

Used by BOTH dataset_gen (labels) and guidance (inference) so the two agree
on exactly one crop + channel definition. Grid arrays are indexed [iy, ix]
where ix is grid-x from world-x and iy is grid-y from world-y.
"""

import numpy as np


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
