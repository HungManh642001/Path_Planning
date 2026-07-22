"""Goal-rooted admissible distance field for the A* heuristic.

Precomputes, once per scenario, a coarse-grid lower bound on the continuous
obstacle-avoiding (and safezone-constrained) distance from any point to the
goal waypoint. Spec: docs/superpowers/specs/
2026-07-18-goal-distance-field-heuristic-design.md.

Every construction choice is biased so the result can only UNDER-estimate
the true remaining cost (admissibility):
- cells are blocked only when fully inside an obstacle (eroded by the cell
  half-diagonal) or fully outside the safezone (dilated by it);
- grid distances are divided by the 8-connectivity stretch factor and
  reduced by a 2-cell digitisation slack;
- the grid border is Euclid-seeded, so paths that leave the gridded area
  (unbounded worlds) are still lower-bounded;
- queries take the best reverse-triangle bound over the 4 surrounding cell
  centers (interpolated lower bounds are not lower bounds).
"""
import math

import numpy as np
import shapely
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from shapely.geometry import Polygon
from shapely.ops import unary_union

import config

# An 8-connected grid path exceeds the continuous shortest by <= this factor
# (away from cell-scale narrow passages; the query slack absorbs the rest).
_STRETCH = 1.0 / math.cos(math.pi / 8.0)


class GoalDistanceField:
    """Admissible lower-bound distance-to-goal field on a coarse grid."""

    def __init__(self, pre):
        gx, gy = pre['goal_state']['waypoint']
        self._goal = (float(gx), float(gy))
        x0, y0, w, h = self._extent(pre)
        cell = max(w, h) / int(config.HEURISTIC_GRID_N)
        self._x0, self._y0, self._cell = x0, y0, cell
        self._nx = max(4, int(math.ceil(w / cell)))
        self._ny = max(4, int(math.ceil(h / cell)))
        cx = x0 + (np.arange(self._nx) + 0.5) * cell
        cy = y0 + (np.arange(self._ny) + 0.5) * cell
        X, Y = np.meshgrid(cx, cy)                      # [iy, ix]
        self._blocked = self._block_cells(pre, X, Y, cell)
        self._d = self._solve(X, Y, self._blocked, cell)

    # ------------------------------------------------------------------
    def _extent(self, pre):
        """(x0, y0, width, height) of the gridded area. Safezone bbox wins;
        else the explicit map_bounds rectangle; else (permissive world) the
        obstacle/start/goal bbox padded 10% of its diagonal."""
        szs = pre.get('safezones')
        if szs:
            u = unary_union([Polygon(s) for s in szs])
            x0, y0, x1, y1 = u.bounds
            pad = max(x1 - x0, y1 - y0) / config.HEURISTIC_GRID_N
            return x0 - pad, y0 - pad, (x1 - x0) + 2 * pad, (y1 - y0) + 2 * pad
        mb = pre.get('map_bounds')
        if mb is not None:
            return 0.0, 0.0, float(mb[0]), float(mb[1])
        xs = [self._goal[0], pre['start_pos'][0]]
        ys = [self._goal[1], pre['start_pos'][1]]
        for (c, r) in pre['circle_obstacles']:
            xs += [c[0] - r, c[0] + r]
            ys += [c[1] - r, c[1] + r]
        for coords in pre['polygon_obstacles']:
            for (px, py) in coords:
                xs.append(px)
                ys.append(py)
        x0, x1 = min(xs), max(xs)
        y0, y1 = min(ys), max(ys)
        pad = 0.10 * math.hypot(x1 - x0, y1 - y0) + 1.0
        return x0 - pad, y0 - pad, (x1 - x0) + 2 * pad, (y1 - y0) + 2 * pad

    def _block_cells(self, pre, X, Y, cell):
        """Under-blocked occupancy: a cell is blocked only when its center is
        so deep inside an obstacle (or outside the safezone) that the WHOLE
        cell must be infeasible."""
        half = cell * math.sqrt(2.0) / 2.0
        blocked = np.zeros(X.shape, dtype=bool)
        for (c, r) in pre['circle_obstacles']:
            rr = r - half
            if rr > 0:
                blocked |= (X - c[0]) ** 2 + (Y - c[1]) ** 2 <= rr * rr
        for coords in pre['polygon_obstacles']:
            eroded = Polygon(coords).buffer(-half)
            if not eroded.is_empty:
                blocked |= shapely.contains_xy(eroded, X, Y)
        szs = pre.get('safezones')
        if szs:
            dilated = unary_union([Polygon(s) for s in szs]).buffer(half)
            blocked |= ~shapely.contains_xy(dilated, X, Y)
        return blocked

    def _solve(self, X, Y, blocked, cell):
        """One multi-source Dijkstra: a virtual super-source connects to the
        goal-neighborhood cells AND every border cell, each seeded with plain
        Euclid-to-goal (a universal lower bound, so leaving the grid stays
        sound). Returns d[iy, ix] (inf where unreachable/blocked)."""
        ny, nx = blocked.shape
        M = ny * nx
        idx = np.arange(M).reshape(ny, nx)
        free = ~blocked
        rows, cols, wts = [], [], []
        for dy, dx, wstep in ((0, 1, cell), (1, 0, cell),
                              (1, 1, cell * math.sqrt(2.0)),
                              (1, -1, cell * math.sqrt(2.0))):
            a_iy = slice(max(0, -dy), ny - max(0, dy))
            a_ix = slice(max(0, -dx), nx - max(0, dx))
            b_iy = slice(max(0, dy), ny - max(0, -dy))
            b_ix = slice(max(0, dx), nx - max(0, -dx))
            ok = free[a_iy, a_ix] & free[b_iy, b_ix]
            rows.append(idx[a_iy, a_ix][ok])
            cols.append(idx[b_iy, b_ix][ok])
            wts.append(np.full(rows[-1].shape, wstep))
        gx, gy = self._goal
        seeds = np.zeros(blocked.shape, dtype=bool)
        seeds[0, :] = seeds[-1, :] = True
        seeds[:, 0] = seeds[:, -1] = True
        gix = int((gx - self._x0) / cell)
        giy = int((gy - self._y0) / cell)
        seeds[max(0, giy - 2):giy + 3, max(0, gix - 2):gix + 3] = True
        seeds &= free
        siy, six = np.nonzero(seeds)
        rows.append(np.full(siy.shape, M))
        cols.append(idx[siy, six])
        wts.append(np.hypot(X[siy, six] - gx, Y[siy, six] - gy))
        graph = csr_matrix(
            (np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))),
            shape=(M + 1, M + 1))
        d = dijkstra(graph, directed=False, indices=M)
        return d[:M].reshape(ny, nx)

    # ------------------------------------------------------------------
    def query(self, p):
        """Admissible lower bound on the continuous distance p -> goal, or
        -inf when the grid has nothing sound to say (caller maxes with
        Euclid). Reverse-triangle over the 4 surrounding cell centers."""
        fx = (p[0] - self._x0) / self._cell - 0.5
        fy = (p[1] - self._y0) / self._cell - 0.5
        ix0 = int(math.floor(fx))
        iy0 = int(math.floor(fy))
        slack = 2.0 * self._cell
        best = -math.inf
        for iy in (iy0, iy0 + 1):
            if not 0 <= iy < self._ny:
                continue
            for ix in (ix0, ix0 + 1):
                if not 0 <= ix < self._nx:
                    continue
                if self._blocked[iy, ix]:
                    continue
                d = self._d[iy, ix]
                if not math.isfinite(d):
                    continue
                cx = self._x0 + (ix + 0.5) * self._cell
                cy = self._y0 + (iy + 0.5) * self._cell
                cand = d / _STRETCH - slack - math.hypot(p[0] - cx, p[1] - cy)
                if cand > best:
                    best = cand
        return best
