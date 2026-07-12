import json
import os


def test_training_notebook_is_valid_and_covers_contract():
    path = os.path.join("ml_planner", "train", "train_guidance.ipynb")
    assert os.path.exists(path)
    with open(path) as f:
        nb = json.load(f)
    assert nb.get("nbformat") == 4
    assert isinstance(nb.get("cells"), list) and len(nb["cells"]) >= 4
    text = "\n".join("".join(c.get("source", [])) for c in nb["cells"])
    # The notebook must document the hard I/O contract and masked loss.
    for token in ("channels", "cost_to_go", "256", "mask", "onnx"):
        assert token in text


def test_models_dir_ignores_onnx():
    with open(os.path.join("ml_planner", "models", ".gitignore")) as f:
        assert "*.onnx" in f.read()
