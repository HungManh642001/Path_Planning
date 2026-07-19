"""AI corridor: rasterize the GNN per-node cost-to-go into a boolean grid.

Corridor membership is a FOCAL ordering tiebreak only — never correctness:
FOCAL still holds every in-band node, in-corridor nodes are merely expanded
first, so a wrong model can only cost time, never the epsilon bound.
(Admission gating was tried first and BROKE the bound — non-reopening lets
worse paths permanently close cells while in-band nodes are held out; see
the seed-6011 regression test.)
"""

import numpy as np

import ml_planner.config as mlcfg
import ml_planner.raster as raster


class Corridor:
    def __init__(self, mask, affine):
        self.mask = mask                     # (G, G) bool, [iy, ix]
        self.affine = affine
        self.grid_res = mask.shape[0]

    def contains(self, x, y):
        gx, gy = self.affine.world_to_grid(x, y)
        ix, iy = int(gx), int(gy)
        if 0 <= iy < self.grid_res and 0 <= ix < self.grid_res:
            return bool(self.mask[iy, ix])
        return False                         # outside crop: not admitted


def build_corridor(preprocessed, graph_guidance, delta=None, grid_res=None):
    """Boolean corridor from the GNN value field, or None (clean fallback)
    when the guidance is missing/unavailable or anything fails."""
    if graph_guidance is None or not getattr(graph_guidance, 'available', False):
        return None
    try:
        delta = mlcfg.CORRIDOR_DELTA if delta is None else delta
        grid_res = mlcfg.CORRIDOR_GRID_RES if grid_res is None else grid_res
        graph_guidance.build_field(preprocessed)
        g = graph_guidance.graph
        aff = raster.compute_crop(preprocessed, grid_res)
        wx, wy = raster._cell_centers_world(aff, grid_res)
        pts = np.column_stack([wx.ravel(), wy.ravel()])
        k = min(3, len(g.nodes))
        d, idx = g.kdtree.query(pts, k=k)
        d = np.atleast_2d(d).reshape(len(pts), -1)
        idx = np.atleast_2d(idx).reshape(len(pts), -1)
        vhat = (d + graph_guidance.values[idx]).min(axis=1).reshape(grid_res, grid_res)
        ox, oy = preprocessed['start_pos']
        d_start = np.hypot(wx - ox, wy - oy)
        cap = (1.0 + delta) * graph_guidance.lookup(preprocessed['start_pos'])
        mask = (d_start + vhat) <= cap
        # Start and goal cells are corridor members by definition.
        for pt in (preprocessed['start_pos'], preprocessed['goal_pos']):
            gx, gy = aff.world_to_grid(*pt)
            ix, iy = int(gx), int(gy)
            if 0 <= iy < grid_res and 0 <= ix < grid_res:
                mask[iy, ix] = True
        return Corridor(mask, aff)
    except Exception:
        return None
