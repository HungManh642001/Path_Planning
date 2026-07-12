"""CNN guidance-map inference: one ONNX forward per problem, O(1) bilinear
lookup per state. Falls back cleanly (available=False) when the model file or
onnxruntime is absent.
"""

import math
import os

import numpy as np

import ml_planner.config as mlcfg
import ml_planner.raster as raster

LARGE = 1e18


def bilinear_lookup(field, gx, gy):
    """Bilinear sample of a (H, W) field at grid coords (gx, gy), indexed
    [iy=gy, ix=gx]. None if outside the grid."""
    h, w = field.shape
    if gx < 0.0 or gy < 0.0 or gx > w - 1 or gy > h - 1:
        return None
    ix0 = int(math.floor(gx))
    iy0 = int(math.floor(gy))
    ix1 = min(ix0 + 1, w - 1)
    iy1 = min(iy0 + 1, h - 1)
    fx = gx - ix0
    fy = gy - iy0
    v00 = field[iy0, ix0]
    v01 = field[iy0, ix1]
    v10 = field[iy1, ix0]
    v11 = field[iy1, ix1]
    return float(v00 * (1 - fx) * (1 - fy) + v01 * fx * (1 - fy)
                 + v10 * (1 - fx) * fy + v11 * fx * fy)


class Guidance:
    """Loads an ONNX cost-to-go model; builds one field per problem.

    Note: grid_res must equal the ONNX model's exported spatial size (the training
    notebook exports static 256x256 axes), or build_field will fail and the planner
    falls back to hand-crafted guidance.
    """

    def __init__(self, model_path=mlcfg.MODEL_PATH, grid_res=mlcfg.GRID_RES):
        self.model_path = model_path
        self.grid_res = grid_res
        self._sess = None
        self.available = False
        self.field = None
        self.affine = None
        if os.path.exists(model_path):
            try:
                import onnxruntime as ort
                self._sess = ort.InferenceSession(
                    model_path, providers=['CPUExecutionProvider'])
                self.available = True
            except Exception:
                self.available = False

    def build_field(self, preprocessed):
        self.affine = raster.compute_crop(preprocessed, self.grid_res)
        channels = raster.build_channels(preprocessed, self.affine, self.grid_res)
        inp = channels[None].astype(np.float32)             # (1, 4, H, W)
        out = self._sess.run(['cost_to_go'], {'channels': inp})[0]
        self.field = np.asarray(out)[0, 0]                  # (H, W)

    def lookup(self, waypoint):
        gx, gy = self.affine.world_to_grid(*waypoint)
        v = bilinear_lookup(self.field, gx, gy)
        return LARGE if v is None else v


def make_guidance_secondary(preprocessed, model_path=None, guidance_obj=None):
    """Build the guidance field once and return (secondary_callable, True), or
    (None, False) when no model is available or build_field fails (caller falls back
    to hand-crafted)."""
    g = guidance_obj if guidance_obj is not None else Guidance(
        model_path or mlcfg.MODEL_PATH)
    if not g.available:
        return None, False
    try:
        g.build_field(preprocessed)
    except Exception:
        return None, False
    return (lambda state: g.lookup(state.waypoint)), True
