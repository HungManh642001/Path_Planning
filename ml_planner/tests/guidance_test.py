import numpy as np
import core.map_generator as mg
import core.preprocessing as prep
from ml_planner.guidance import bilinear_lookup, Guidance, make_guidance_secondary, LARGE
from ml_planner.raster import compute_crop


def test_bilinear_interpolates_and_flags_out_of_range():
    field = np.array([[0.0, 10.0], [20.0, 30.0]], dtype=np.float32)
    assert abs(bilinear_lookup(field, 0.0, 0.0) - 0.0) < 1e-6
    assert abs(bilinear_lookup(field, 1.0, 0.0) - 10.0) < 1e-6
    assert abs(bilinear_lookup(field, 0.5, 0.5) - 15.0) < 1e-6   # center = mean
    assert bilinear_lookup(field, -0.1, 0.0) is None
    assert bilinear_lookup(field, 1.6, 0.0) is None


def test_guidance_unavailable_without_model():
    g = Guidance(model_path="/nonexistent/guidance.onnx")
    assert g.available is False


def test_make_guidance_secondary_falls_back_without_model():
    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    cb, available = make_guidance_secondary(pre, model_path="/nonexistent/guidance.onnx")
    assert available is False
    assert cb is None


class _StubGuidance:
    """Guidance-shaped stub: field = distance-to-goal in grid units."""
    available = True

    def __init__(self, grid_res=64):
        self.grid_res = grid_res
        self.field = None
        self.affine = None

    def build_field(self, preprocessed):
        self.affine = compute_crop(preprocessed, self.grid_res)
        gx, gy = self.affine.world_to_grid(*preprocessed['goal_pos'])
        iy, ix = np.mgrid[0:self.grid_res, 0:self.grid_res]
        self.field = np.sqrt((ix - gx) ** 2 + (iy - gy) ** 2).astype(np.float32)

    def lookup(self, waypoint):
        from ml_planner.guidance import bilinear_lookup as bl
        gx, gy = self.affine.world_to_grid(*waypoint)
        v = bl(self.field, gx, gy)
        return LARGE if v is None else float(v)


def test_stub_guidance_secondary_builds_and_looks_up():
    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    stub = _StubGuidance()
    cb, available = make_guidance_secondary(pre, guidance_obj=stub)
    assert available is True
    # Near the goal the guidance cost is small; near the start it is larger.
    from core.kinodynamic_astar import State
    near_goal = State(pre['goal_pos'], 0.0)
    near_start = State(pre['start_pos'], 0.0)
    assert cb(near_goal) < cb(near_start)


def test_real_onnx_roundtrip(tmp_path):
    onnx = __import__('pytest').importorskip("onnx")
    __import__('pytest').importorskip("onnxruntime")
    from onnx import helper, TensorProto
    # Trivial model: cost_to_go = mean over the 4 channels -> (1,1,H,W).
    node = helper.make_node("ReduceMean", ["channels"], ["cost_to_go"],
                            axes=[1], keepdims=1)
    graph = helper.make_graph(
        [node], "g",
        [helper.make_tensor_value_info("channels", TensorProto.FLOAT, [1, 4, 64, 64])],
        [helper.make_tensor_value_info("cost_to_go", TensorProto.FLOAT, [1, 1, 64, 64])],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    path = str(tmp_path / "guidance.onnx")
    onnx.save(model, path)

    pre = prep.prepare_scenario(mg.scenario2_single_obstacle())
    g = Guidance(model_path=path, grid_res=64)
    assert g.available is True
    g.build_field(pre)
    assert g.field.shape == (64, 64)
    assert g.lookup(pre['goal_pos']) < LARGE
